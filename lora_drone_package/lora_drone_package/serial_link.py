"""LoRa serial link — owns the serial port, the writer lock, reconnect, and the RX read loop.

Infrastructure (not pure policy): it touches real serial I/O. It is ROS-agnostic though — it
takes a logger and an `is_alive()` predicate, so it is reusable by any node (e.g. the gimbal
bridge). Framing/parsing stays in the caller; this class moves bytes and keeps the link up.

Concurrency: a single lock serialises the WRITERS (telemetry + all ACKs). Reads and writes on a
serial port are independent, so the blocking readline() in read_loop() does NOT hold the lock —
otherwise it would stall every writer for up to the read timeout.
"""

import time
from threading import Lock

import serial


class SerialLink:
    def __init__(self, path, baud, logger, is_alive):
        self.path = path
        self.baud = baud
        self._log = logger                 # object with .info/.warn/.error/.fatal (e.g. node.get_logger())
        self._alive = is_alive             # callable -> bool; False => loops/reopen should stop
        self._lock = Lock()                # serialises writers
        self._reopen_lock = Lock()         # at most one reconnect at a time
        self.ser = None

    # --- open / close ------------------------------------------------------------------
    def _open_once(self):
        ser = serial.Serial(self.path, self.baud, timeout=1, write_timeout=0.5)
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        return ser

    def open_with_retries(self, attempts=5):
        """Initial open with a few retries (USB-LoRa may enumerate slightly late), then fail
        loudly. Raises RuntimeError if the port never opens."""
        for attempt in range(1, attempts + 1):
            try:
                self.ser = self._open_once()
                self._log.info(f"✅ Serial connected at {self.path} (attempt {attempt})")
                return
            except Exception as e:
                self._log.error(f"❌ Open serial failed ({attempt}/{attempts}): {e}")
                time.sleep(1.0)
        self._log.fatal(f"❌ Cannot open {self.path} after retries — aborting node init")
        raise RuntimeError(f"Cannot open serial {self.path}")

    def reopen(self, reason=""):
        if not self._alive():
            return False
        if not self._reopen_lock.acquire(blocking=False):
            return False
        try:
            self._log.warn(f"🔁 Reopening LoRa serial{': ' + reason if reason else ''}")
            with self._lock:
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass

                for attempt in range(1, 4):
                    try:
                        self.ser = self._open_once()
                        self._log.info(
                            f"✅ Serial reconnected at {self.path} (attempt {attempt})")
                        return True
                    except Exception as e:
                        self._log.error(f"❌ Serial reopen failed ({attempt}/3): {e}")
                        time.sleep(0.5)
            return False
        finally:
            self._reopen_lock.release()

    def close(self):
        with self._lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception as e:
                print(f"⚠️ serial close error: {e}")

    # --- write -------------------------------------------------------------------------
    def write(self, frame_bytes, priority=True) -> bool:
        """Write one already-framed line to the port. priority=True blocks for the lock
        (commands/ACKs); priority=False is best-effort — it skips this write if the lock is
        held (telemetry must not queue behind control traffic on a weak link). Reopens the
        port on a serial error. Returns True on a successful write."""
        try:
            if priority:
                with self._lock:
                    if not self.ser or not self.ser.is_open:
                        raise serial.SerialException("serial is not open")
                    self.ser.write(frame_bytes)
            else:
                if not self._lock.acquire(blocking=False):
                    return False
                try:
                    if not self.ser or not self.ser.is_open:
                        raise serial.SerialException("serial is not open")
                    self.ser.write(frame_bytes)
                finally:
                    self._lock.release()
            return True
        except (serial.SerialTimeoutException, serial.SerialException, OSError) as e:
            self._log.error(f"❌ JSON send serial error: {e}")
            self.reopen(str(e))
            return False

    # --- read loop ---------------------------------------------------------------------
    def read_loop(self, on_line):
        """Blocking RX loop: read one line at a time and hand it to on_line(str). Reconnects on
        a closed port / serial error. Runs until is_alive() returns False."""
        while self._alive():
            try:
                # Do NOT hold the writer lock around the blocking readline(): it would stall
                # every writer (telemetry + all ACKs) for up to the read timeout (1s).
                if not self.ser or not self.ser.is_open:
                    self.reopen("serial closed")
                    time.sleep(0.2)
                    continue
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                on_line(line)
            except (serial.SerialException, OSError) as e:
                if not self._alive():
                    break
                self._log.error(f"❌ Serial read error: {e}")
                self.reopen(str(e))
                time.sleep(0.2)
            except Exception as e:
                self._log.error(f"❌ Serial read error: {e}")
                time.sleep(0.2)
