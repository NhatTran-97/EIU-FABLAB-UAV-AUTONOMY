"""Link: 1 duong truyen MAVLink (lop chung cho Serial / UDP / ...).

Lop con chi can cai dat 4 ham I/O: _open, _read, _write, _close. Lop chung lo:
  * Thread doc: read -> parse -> on_messages(link, [msgs]) theo lo (PX4 ~100 msg/s; giam so lan
    chuyen sang main thread).
  * Thread heartbeat GCS 1 Hz, doc lap voi UI (mat heartbeat GCS -> failsafe NAV_DLL_ACT). Heartbeat
    cung kich SiK chen RADIO_STATUS; SiK 1.9 chi nhan heartbeat v1 -> tuy chon heartbeat_v1.
  * Loi I/O -> mo lai moi RECONNECT_S den khi stop().

on_messages / on_state chay tren thread cua link; tang Qt chuyen ve main thread.
"""

import threading

from ground_station_ui.core.mavlink import drone_dialect as mavlink

GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = mavlink.MAV_COMP_ID_MISSIONPLANNER
RECONNECT_S = 2.0


class Link:
    kind = 'base'

    def __init__(self, name, cfg, on_messages, on_state):
        self.name = name
        self.cfg = cfg
        self.heartbeat_v1 = bool(cfg.get('heartbeat_v1', False))
        self._on_messages = on_messages
        self._on_state = on_state

        self.status = 'Not connected'
        self.is_open = False
        self.active = False            # trang thai yeu cau (Connect / Disconnect)
        self.rx_bytes = 0
        self.tx_bytes = 0

        self._handle = None
        self._lock = threading.Lock()  # bao ve _handle khi ghi / dong
        self._stop = threading.Event()
        self._reader = None
        self.mav = mavlink.MAVLink(None, srcSystem=GCS_SYSTEM_ID, srcComponent=GCS_COMPONENT_ID)

    # --- lop con cai dat ------------------------------------------------------------------
    def _open(self):
        raise NotImplementedError

    def _read(self, handle):
        """Tra ve bytes (co the rong); block toi da ~0.1 s. Nem OSError khi mat ket noi."""
        raise NotImplementedError

    def _write(self, handle, data):
        raise NotImplementedError

    def _close(self, handle):
        raise NotImplementedError

    def describe(self):
        return self.name

    # --- dieu khien ------------------------------------------------------------------------
    def start(self):
        self.stop()
        self.active = True
        # dat trang thai truoc khi start thread: thread co the ghi 'Open ...' ngay lap tuc
        self._set_status(f'Opening {self.describe()}...')
        # Event moi cho moi lan start: thread cu khong bi danh thuc boi lan Connect sau
        self._stop = stop = threading.Event()
        self._reader = threading.Thread(target=self._run, args=(stop,),
                                        name=f'{self.name}-rx', daemon=True)
        self._reader.start()
        threading.Thread(target=self._heartbeat_loop, args=(stop,),
                         name=f'{self.name}-hb', daemon=True).start()

    def stop(self):
        self.active = False
        self._stop.set()
        with self._lock:
            self._close_locked()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=1.0)
        self._reader = None
        self._set_status('Not connected')

    def send(self, msg, force_v1=False):
        """Dong goi + ghi 1 message (thread-safe). False neu link dong / loi."""
        with self._lock:
            if self._handle is None:
                return False
            try:
                data = msg.pack(self.mav, force_mavlink1=force_v1)
                self._write(self._handle, data)
                self.tx_bytes += len(data)
                return True
            except OSError:
                self._close_locked()
                return False

    # --- noi bo ----------------------------------------------------------------------------
    def _set_status(self, text):
        self.status = text
        self._on_state(self)

    def _close_locked(self):
        if self._handle is not None:
            try:
                self._close(self._handle)
            except Exception:
                pass
            self._handle = None
            self.is_open = False

    def _run(self, stop):
        parser = mavlink.MAVLink(None)
        parser.robust_parsing = True
        while not stop.is_set():
            try:
                handle = self._open()
            except OSError as e:
                self._set_status(f'Cannot open {self.describe()}: {getattr(e, "strerror", None) or e}')
                stop.wait(RECONNECT_S)
                continue
            with self._lock:
                if stop.is_set():          # stop() trong luc _open
                    self._close(handle)
                    return
                self._handle = handle
                self.is_open = True
            self._set_status(f'Open {self.describe()}')
            self._read_loop(handle, parser, stop)
            with self._lock:
                if self._handle is handle:
                    self._close_locked()
            if not stop.is_set():
                self._set_status(f'Lost {self.describe()}, retrying...')
                stop.wait(RECONNECT_S)

    def _read_loop(self, handle, parser, stop):
        while not stop.is_set():
            try:
                data = self._read(handle)
            except (OSError, TypeError):    # TypeError: pyserial, cong bi dong tu thread khac
                return
            if not data:
                continue
            self.rx_bytes += len(data)
            try:
                msgs = parser.parse_buffer(data) or []
            except mavlink.MAVError:
                continue
            msgs = [m for m in msgs if m.get_type() != 'BAD_DATA']
            if msgs:
                self._on_messages(self, msgs)

    def _heartbeat_loop(self, stop):
        hb = self.mav.heartbeat_encode(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID,
                                       0, 0, mavlink.MAV_STATE_ACTIVE)
        while not stop.wait(1.0):
            self.send(hb, force_v1=self.heartbeat_v1)
