import time
from typing import Optional
import serial


START = "<"
END = ">"


class WarriorSerial:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None

    def open(self) -> None:
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(2.0)  # Arduino reset time after opening serial

    def close(self) -> None:
        if self.serial and self.serial.is_open:
            self.serial.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def send_message(self, message_type: str, *fields) -> None:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port is not open")

        payload = ",".join([message_type, *[str(field) for field in fields]])
        packet = f"{START}{payload}{END}\n"
        self.serial.write(packet.encode("utf-8"))
        self.serial.flush()

    def read_message(self, timeout: Optional[float] = None) -> Optional[str]:
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port is not open")

        timeout = self.timeout if timeout is None else timeout
        deadline = time.time() + timeout

        inside_message = False
        buffer = ""

        while time.time() < deadline:
            byte = self.serial.read(1)

            if not byte:
                continue

            char = byte.decode("utf-8", errors="ignore")

            if char == START:
                inside_message = True
                buffer = ""
            elif char == END and inside_message:
                return buffer
            elif inside_message:
                buffer += char

        return None

    def request(self, message_type: str, *fields, timeout: Optional[float] = None) -> Optional[str]:
        self.send_message(message_type, *fields)
        return self.read_message(timeout=timeout)


def query_device_name(port: str, baudrate: int = 115200, timeout: float = 1.0) -> Optional[str]:
    """Send WHO and read messages until we see a NAME response or hit timeout.

    Devices may be streaming other messages (e.g. PWM at 20 Hz), so we can't
    trust the first message back — we have to skip past unrelated traffic.
    """
    with WarriorSerial(port, baudrate, timeout) as device:
        device.send_message("WHO")
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.05, deadline - time.time())
            response = device.read_message(timeout=remaining)
            if response is None:
                continue
            parts = response.split(",")
            if len(parts) == 2 and parts[0] == "NAME":
                return parts[1]

    return None