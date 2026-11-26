"""Stream simulated RGB-D frames from Isaac to the ROS adapter over localhost TCP.

A 640x480 RGB + float32 depth frame is ~2 MB, far above a UDP datagram, so
frames use one TCP connection that the ROS adapter (isaac_ros_bridge.py)
listens on at telemetry port + 2. Isaac connects as the client and resends
only the newest frame: a slow or absent reader never blocks physics.

Wire format, little-endian, one frame:
    HEADER  magic b'WRGB', stamp f64 (sim seconds), width u32, height u32,
            fx f32, fy f32, cx f32, cy f32
    rgb     height*width*3 uint8
    depth   height*width float32, metres along the optical axis, inf = no return
"""
import socket
import struct
import threading
import time

HEADER = struct.Struct('<4sdIIffff')
MAGIC = b'WRGB'


def pack(stamp, rgb, depth, intrinsics):
    """Serialize one frame; rgb is (H, W, 3) uint8, depth (H, W) float32."""
    height, width = depth.shape
    if rgb.shape != (height, width, 3):
        raise ValueError(f'rgb {rgb.shape} does not match depth {depth.shape}')
    header = HEADER.pack(MAGIC, stamp, width, height, *intrinsics)
    return header + rgb.astype('uint8', copy=False).tobytes() + depth.astype('<f4', copy=False).tobytes()


class FrameSender:
    """Background TCP client that always sends the most recent frame."""

    def __init__(self, port):
        self.port = port
        self._frame = None
        self._condition = threading.Condition()
        self._closed = False
        self.sent = 0
        self._thread = threading.Thread(target=self._run, name='rgbd-sender', daemon=True)
        self._thread.start()

    def submit(self, payload):
        with self._condition:
            self._frame = payload
            self._condition.notify()

    def close(self):
        with self._condition:
            self._closed = True
            self._condition.notify()

    def _run(self):
        connection = None
        while not self._closed:
            if connection is None:
                try:
                    connection = socket.create_connection(('127.0.0.1', self.port), timeout=1.0)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection.settimeout(5.0)
                except OSError:
                    connection = None
                    time.sleep(1.0)
                    continue
            with self._condition:
                while self._frame is None and not self._closed:
                    self._condition.wait()
                payload, self._frame = self._frame, None
            if payload is None:
                break
            try:
                connection.sendall(payload)
                self.sent += 1
            except OSError:
                # The ROS adapter restarted; reconnect and drop this frame.
                connection.close()
                connection = None
        if connection is not None:
            connection.close()
