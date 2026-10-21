"""Serial link to the SiK radio: writes MAVLink frames, reads and parses incoming frames.

No ROS dependency: takes a logger (.info/.warn/.error) and an on_message(msg) callback. I/O errors
(port missing or unplugged) are not raised; the port is marked closed and the node calls
ensure_open() periodically to reopen it.
"""

import threading
import time

import serial

from drone_telemetry import drone_dialect as mavlink


class RadioLink:
    def __init__(self, port, baud, system_id, component_id, logger, on_message=None):
        self.port = port
        self.baud = baud
        self._log = logger
        self._on_message = on_message
        self._ser = None
        self._lock = threading.Lock()
        self._reader = None
        self._stop = threading.Event()
        # file=None: frames are encoded with pack(); a separate parser handles receive
        self.mav = mavlink.MAVLink(None, srcSystem=system_id, srcComponent=component_id)
        self._parser = mavlink.MAVLink(None)
        self._parser.robust_parsing = True
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.tx_errors = 0
        self._last_open_warn = 0.0

    @property
    def is_open(self):
        return self._ser is not None

    def ensure_open(self):
        if self._ser is not None:
            return True
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=0.5)
        except (serial.SerialException, OSError) as e:
            if time.monotonic() - self._last_open_warn > 10.0:
                self._last_open_warn = time.monotonic()
                self._log.warn(f"Cannot open {self.port}: {e}")
            return False
        with self._lock:
            self._ser = ser
        self._log.info(f"Da mo {self.port} @ {self.baud}")
        self._reader = threading.Thread(target=self._read_loop, args=(ser,), daemon=True)
        self._reader.start()
        return True

    def send(self, msg, force_v1=False):
        """Encode and write one MAVLink message. Returns False if the port is closed or the write fails.

        force_v1: SiK 1.9 radios only inject RADIO_STATUS (RSSI, txbuf) after a MAVLink v1 heartbeat.
        """
        with self._lock:
            if self._ser is None:
                return False
            try:
                buf = msg.pack(self.mav, force_mavlink1=force_v1)
                self._ser.write(buf)
                self.tx_bytes += len(buf)
                return True
            except (serial.SerialException, OSError) as e:
                self.tx_errors += 1
                self._log.error(f"Write to {self.port} failed: {e}")
                self._close_locked()
                return False

    def close(self):
        self._stop.set()
        with self._lock:
            self._close_locked()

    def _close_locked(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _read_loop(self, ser):
        while not self._stop.is_set():
            try:
                data = ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError) as e:
                # TypeError: raised by pyserial when another thread closed the port
                with self._lock:
                    if self._ser is ser:
                        self._log.error(f"Read from {self.port} failed: {e}")
                        self._close_locked()
                return
            if not data:
                continue
            self.rx_bytes += len(data)
            try:
                msgs = self._parser.parse_buffer(data) or []
            except mavlink.MAVError:
                continue
            if self._on_message:
                for m in msgs:
                    self._on_message(m)
