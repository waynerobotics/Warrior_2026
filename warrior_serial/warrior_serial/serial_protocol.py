"""
warrior_serial.serial_protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Low-level framing / parsing helpers for the Warrior ASCII wire protocol.

Every message is:  <TYPE,field1,field2,...>\n
Both directions use the same format.
"""

import re
import termios
import time
from typing import Optional

import serial


# Regex: capture everything between < and > (non-greedy, no nested brackets)
_FRAME_RE = re.compile(r'<([^<>]+)>')

BAUD_RATE_DEFAULT = 115_200
OPEN_RESET_DELAY_S = 2.0   # Arduino resets on DTR toggle; must wait this long


# ---------------------------------------------------------------------------
# Low-level framing
# ---------------------------------------------------------------------------

def encode_message(*fields: str) -> bytes:
    """Encode fields into a framed message: b'<field0,field1,...>\\n'"""
    body = ','.join(str(f) for f in fields)
    return f'<{body}>\n'.encode('ascii')


def parse_message(raw: str) -> Optional[list]:
    """
    Extract the first framed message from *raw*.

    Returns a list of string fields, or None if no complete frame is present.
    E.g. '<MOT,02_swerve,50,-30>' -> ['MOT', '02_swerve', '50', '-30']
    """
    m = _FRAME_RE.search(raw)
    if m is None:
        return None
    return m.group(1).split(',')


# ---------------------------------------------------------------------------
# WarriorSerial — thin wrapper around pyserial
# ---------------------------------------------------------------------------

class WarriorSerial:
    """
    Manages a single serial connection to one Arduino.

    After :meth:`open` the caller must wait ``OPEN_RESET_DELAY_S`` seconds
    before sending — the DTR toggle resets the Arduino and the bootloader
    takes ~2 s to hand off to user code.
    """

    def __init__(self, port: str, baud_rate: int = BAUD_RATE_DEFAULT,
                 read_timeout_s: float = 0.1):
        self._port = port
        self._baud_rate = baud_rate
        self._read_timeout_s = read_timeout_s
        self._ser: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open the serial port exclusively and wait for the Arduino reset.

        ``exclusive=True`` acquires a kernel-level lock (TIOCEXCL on Linux)
        so that a second process attempting to open the same port gets
        ``[Errno 11] Resource temporarily unavailable`` instead of silently
        stealing bytes from the first opener.
        """
        self._ser = serial.Serial(
            self._port,
            baudrate=self._baud_rate,
            timeout=self._read_timeout_s,
            exclusive=True,
        )
        time.sleep(OPEN_RESET_DELAY_S)

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ------------------------------------------------------------------
    # I/O
    # ------------------------------------------------------------------

    def read_line(self) -> Optional[str]:
        """
        Read one line from the serial port (non-blocking up to read_timeout_s).

        Returns the decoded line (stripped) or None if nothing arrived.
        Raises serial.SerialException on port error.
        """
        if self._ser is None:
            return None
        line = self._ser.readline()
        if not line:
            return None
        return line.decode('ascii', errors='replace').strip()

    def write_message(self, *fields: str) -> None:
        """
        Encode *fields* as a framed message and write to the serial port.

        Raises serial.SerialException on port error.
        """
        if self._ser is None:
            raise serial.SerialException('Port is not open')
        try:
            self._ser.write(encode_message(*fields))
            self._ser.flush()  # ensure bytes leave the OS TX buffer immediately
        except termios.error as exc:
            raise serial.SerialException(f'flush failed: {exc}') from exc


# ---------------------------------------------------------------------------
# Discovery helper
# ---------------------------------------------------------------------------

def query_device_name(port: str, baud_rate: int = BAUD_RATE_DEFAULT,
                      timeout_s: float = 3.0) -> Optional[str]:
    """
    Open *port*, send ``<WHO>``, and return the device's reported name.

    Filters out non-``<NAME,…>`` frames (e.g. streaming ``<MOT,…>`` traffic
    from ``00_base``) until a ``<NAME,…>`` reply arrives or *timeout_s*
    elapses.

    Returns the name string (e.g. ``"00_base"``) or ``None`` on timeout /
    unrecognised reply.
    """
    ws = WarriorSerial(port, baud_rate, read_timeout_s=0.2)
    try:
        ws.open()  # raises SerialException immediately if port is locked by another driver
        ws.write_message('WHO')
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = ws.read_line()
            if line is None:
                continue
            fields = parse_message(line)
            if fields and fields[0] == 'NAME' and len(fields) >= 2:
                return fields[1]
        return None
    except serial.SerialException:
        return None
    finally:
        ws.close()
