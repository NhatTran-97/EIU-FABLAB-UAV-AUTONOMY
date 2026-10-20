"""Lenh bay tu ground station (radio 433) -> PX4 (uXRCE-DDS /fmu/in/vehicle_command).

Khong phu thuoc ROS; node ROS noi cac callback:
    send_px4(command, params[7])   publish VehicleCommand
    reply(gcs_command, result)     COMMAND_ACK ve ground station
    status(text, severity)         STATUSTEXT ve ground station (ly do tu choi...)

Take-off / Land co 2 cach (telemetry.yaml commands.takeoff_land_via):
    mission (mac dinh): -> mission_manager (/gcs/command) -> offboard_control -> PX4 OFFBOARD qua DDS.
        Take-off: mission_manager chap nhan -> cho PX4 ACK DO_SET_MODE (OFFBOARD) + ARM (offboard_control
        gui) -> ACCEPTED. Land: mission_manager chap nhan -> cho PX4 ACK NAV_LAND -> ACCEPTED.
        Ground thay IN_PROGRESS trong luc cho. mission_manager tu choi / im lang, PX4 tu choi / im lang
        -> ket qua + ly do (STATUSTEXT).
    px4: gui thang VehicleCommand nhu duoi (che do Takeoff / Land tu dong cua PX4).

Lenh ground -> viec gui PX4 (cach px4; Hold / RTL luon theo cach nay):
    NAV_TAKEOFF (22, param7 = do cao so voi diem cat canh, m)
        [ARM neu chua arm] -> NAV_TAKEOFF (param7 = AMSL hien tai + do cao; khong co do cao
        toan cuc (trong nha) -> NaN = PX4 dung MIS_TAKEOFF_ALT). PX4 len do cao roi tu Hold.
    NAV_LAND (21)               -> NAV_LAND tai cho
    NAV_LOITER_UNLIM (17)       -> DO_SET_MODE AUTO.LOITER (Hold)
    NAV_RETURN_TO_LAUNCH (20)   -> NAV_RETURN_TO_LAUNCH (can GPS / vi tri toan cuc)
    MISSION_START (300)         -> mission_manager bay waypoint da upload (tu HOVER)
    USER_1 (31010) = PING       -> khong gui PX4; tra ACCEPTED + STATUSTEXT (kiem tra link ground <-> Jetson)

An toan: Take-off chi khi o mat dat; Land / Hold / RTL chi khi dang bay; du lieu PX4 cu hon
PX4_FRESH_S -> tu choi. Lenh moi thay lenh dang chay.

Lenh gui lai (COMMAND_LONG.confirmation > 0) khong duoc thuc hien lai:
  * lenh dang chay -> IN_PROGRESS
  * lenh vua xong  -> ket qua cu (ACK bi mat tren radio), giu trong RESULT_HOLD_S
                      (>= retries x ack_timeout cua ground)
"""

import math

# MAV_RESULT (MAVLink) va VEHICLE_CMD_RESULT (PX4) cung gia tri
ACCEPTED, TEMP_REJECTED, DENIED, UNSUPPORTED, FAILED, IN_PROGRESS = 0, 1, 2, 3, 4, 5

CMD_HOLD = 17          # MAV_CMD_NAV_LOITER_UNLIM
CMD_RTL = 20
CMD_LAND = 21
CMD_TAKEOFF = 22
CMD_MISSION_START = 300   # MAV_CMD_MISSION_START
CMD_PING = 31010       # MAV_CMD_USER_1: ping ground <-> Jetson
PX4_SET_MODE = 176     # VEHICLE_CMD_DO_SET_MODE
PX4_ARM = 400          # VEHICLE_CMD_COMPONENT_ARM_DISARM

NAMES = {CMD_TAKEOFF: 'Take-off', CMD_LAND: 'Land', CMD_HOLD: 'Hold', CMD_RTL: 'RTL',
         CMD_MISSION_START: 'Start mission'}
MISSION_CMDS = {CMD_TAKEOFF: 'takeoff', CMD_LAND: 'land', CMD_MISSION_START: 'mission'}
NAN = float('nan')

SEV_WARNING, SEV_INFO = 4, 6


class CommandBridge:
    PX4_FRESH_S = 2.0        # du lieu PX4 cu hon -> tu choi lenh
    STEP_TIMEOUT_S = 3.0     # buoc khong co ACK -> gui lai 1 lan, roi FAILED
    KEEPALIVE_S = 0.5        # chu ky IN_PROGRESS (< ack_timeout 1.5 s cua ground)
    RESULT_HOLD_S = 10.0     # >= retries x ack_timeout cua ground (3 x 1.5 s)
    MISSION_ACK_TIMEOUT_S = 1.0     # cho ACK mission_manager; gui lai 1 lan
    PX4_CONFIRM_TIMEOUT_S = 6.0     # offboard_control stream setpoint ~1 s truoc khi xin OFFBOARD + ARM

    def __init__(self, send_px4, reply, status, send_mission=None):
        """send_mission(dict): Take-off / Land qua mission_manager; None = gui thang PX4."""
        self.send_px4, self.reply, self.status = send_px4, reply, status
        self.send_mission = send_mission
        self._req = 0
        self.landed = None
        self.armed = None
        self.amsl = None          # m AMSL; None khi khong co vi tri toan cuc
        self._px4_at = None
        self._job = None          # {'gcs', 'steps', 'i', 'sent_at', 'tries', 'kept_at'}
        self._last = None         # (lenh, ket qua, t xong) cho lenh gui lai
        self.pings = 0
        self._now = 0.0

    # --- trang thai PX4 (tu cac topic /fmu/out) -------------------------------------------
    def update_px4(self, now, landed=None, armed=None, amsl=None, amsl_valid=None):
        self._px4_at = now
        if landed is not None:
            self.landed = bool(landed)
        if armed is not None:
            self.armed = bool(armed)
        if amsl_valid is not None:
            self.amsl = float(amsl) if amsl_valid and amsl is not None and math.isfinite(amsl) else None

    @property
    def busy(self):
        return self._job is not None

    # --- lenh tu ground ---------------------------------------------------------------------
    def handle(self, cmd, params, now, confirmation=0):
        self._now = now
        if cmd == CMD_PING:                      # ping: tra loi ngay, khong qua PX4
            self.pings += 1
            self.reply(cmd, ACCEPTED)
            self.status(f'Link test OK: Jetson got ping #{self.pings}'
                        + (f' (retry {confirmation})' if confirmation else ''), SEV_INFO)
            return
        name = NAMES.get(cmd)
        if name is None:
            self.reply(cmd, UNSUPPORTED)
            return
        job = self._job
        if job and job['gcs'] == cmd:            # lenh gui lai
            self.reply(cmd, IN_PROGRESS)
            return
        last = self._last
        if (confirmation > 0 and job is None and last and last[0] == cmd
                and now - last[2] <= self.RESULT_HOLD_S):
            self.reply(cmd, last[1])             # lenh vua xong: tra lai ket qua
            return
        if self._px4_at is None or now - self._px4_at > self.PX4_FRESH_S or self.landed is None:
            self._deny(cmd, TEMP_REJECTED, f'{name}: no PX4 data (DDS agent?)')
            return

        if cmd == CMD_TAKEOFF and not self.landed:
            self._deny(cmd, DENIED, 'Take-off: already flying')
            return
        # Land khi dang cat canh (da ARM, hoac Take-off dang chay chua ARM): cho phep -> mission_manager huy
        takeoff_running = job is not None and job['gcs'] == CMD_TAKEOFF
        abort_takeoff = self.send_mission and cmd == CMD_LAND and (self.armed or takeoff_running)
        if cmd != CMD_TAKEOFF and self.landed and not abort_takeoff:
            self._deny(cmd, DENIED, f'{name}: vehicle is on the ground')
            return
        if self.send_mission and cmd in MISSION_CMDS:
            self._start_mission(cmd, name, params, now, job)
            return
        if cmd == CMD_MISSION_START:              # chi ho tro qua mission_manager
            self._deny(cmd, UNSUPPORTED, 'Start mission: needs commands.takeoff_land_via = mission')
            return

        if cmd == CMD_TAKEOFF:
            rel = params[6] if len(params) > 6 and math.isfinite(params[6]) else NAN
            alt = self.amsl + rel if (self.amsl is not None and math.isfinite(rel)) else NAN
            steps = [] if self.armed else [(PX4_ARM, [1.0, 0, 0, 0, 0, 0, 0])]
            steps.append((CMD_TAKEOFF, [NAN, 0, 0, NAN, NAN, NAN, alt]))
            if not math.isfinite(alt):
                self.status('Take-off: no global altitude, PX4 uses MIS_TAKEOFF_ALT', SEV_INFO)
        else:
            if cmd == CMD_LAND:
                steps = [(CMD_LAND, [NAN, 0, 0, NAN, NAN, NAN, NAN])]
            elif cmd == CMD_HOLD:
                steps = [(PX4_SET_MODE, [1.0, 4.0, 3.0, 0, 0, 0, 0])]     # custom: AUTO.LOITER
            else:
                steps = [(CMD_RTL, [0] * 7)]

        if job:                                   # lenh moi thay lenh dang chay
            self.status(f"{NAMES[job['gcs']]} superseded by {name}", SEV_INFO)
        self._job = {'gcs': cmd, 'steps': steps, 'i': 0, 'sent_at': now, 'tries': 1, 'kept_at': now}
        self.reply(cmd, IN_PROGRESS)
        self._send_step()

    # --- Take-off / Land qua mission_manager (OFFBOARD) --------------------------------------
    def _start_mission(self, cmd, name, params, now, job):
        rel = params[6] if len(params) > 6 and math.isfinite(params[6]) else None
        self._req += 1
        req = {'id': self._req, 'cmd': MISSION_CMDS[cmd]}
        if cmd == CMD_TAKEOFF:
            req['alt'] = rel
        if job:
            self.status(f"{NAMES[job['gcs']]} superseded by {name}", SEV_INFO)
        # ACK PX4 can cho: Take-off -> DO_SET_MODE (OFFBOARD) + ARM; Land -> NAV_LAND.
        # Khong cho: Land khi chua ARM (khong co lenh PX4), Start mission (chi doi setpoint).
        if cmd == CMD_TAKEOFF:
            expect = {PX4_SET_MODE, PX4_ARM}
        elif cmd == CMD_LAND:
            expect = {CMD_LAND} if self.armed else set()
        else:
            expect = set()
        self._job = {'gcs': cmd, 'mission': req, 'phase': 'mission', 'expect': expect,
                     'sent_at': now, 'tries': 1, 'kept_at': now}
        self.reply(cmd, IN_PROGRESS)
        self.send_mission(req)

    def on_mission_ack(self, req_id, result, text, now):
        """Tra loi cua mission_manager (/gcs/command_ack)."""
        self._now = now
        job = self._job
        if job is None or 'mission' not in job or job['mission']['id'] != req_id or job['phase'] != 'mission':
            return
        name = NAMES[job['gcs']]
        if result != 'accepted':
            self._finish(DENIED, f'{name}: mission_manager: {text}')
            return
        if job['expect']:
            self.status(f'{name}: mission_manager {text}, waiting for PX4', SEV_INFO)
        job['phase'], job['sent_at'] = 'px4', now
        if not job['expect']:                       # khong con ACK PX4 can cho
            if job['gcs'] == CMD_MISSION_START:
                text = f'{name}: {text}'
            elif job['gcs'] == CMD_TAKEOFF or self.armed:
                text = f'{name}: accepted by PX4 (offboard)'
            else:
                text = f'{name}: take-off cancelled before arming'
            self._finish(ACCEPTED, text)

    def _deny(self, cmd, result, text):
        self.status(text, SEV_WARNING)
        self.reply(cmd, result)

    def _send_step(self):
        cmd, params = self._job['steps'][self._job['i']]
        self.send_px4(cmd, params)

    def _finish(self, result, text=None):
        job, self._job = self._job, None
        self._last = (job['gcs'], result, self._now)
        if text:
            self.status(text, SEV_INFO if result == ACCEPTED else SEV_WARNING)
        self.reply(job['gcs'], result)

    # --- ACK tu PX4 (/fmu/out/vehicle_command_ack) ----------------------------------------
    def on_px4_ack(self, px4_cmd, result, now):
        self._now = now
        job = self._job
        if job is not None and 'mission' in job:
            self._on_px4_ack_mission(job, px4_cmd, result, now)
            return
        if job is None or job['steps'][job['i']][0] != px4_cmd:
            return
        name = NAMES[job['gcs']]
        if result == IN_PROGRESS:
            job['sent_at'] = now
            return
        if result != ACCEPTED:
            what = 'arming' if px4_cmd == PX4_ARM else name
            self._finish(result if result in (TEMP_REJECTED, DENIED, UNSUPPORTED, FAILED) else FAILED,
                         f'{name}: PX4 rejected {what} (result {result}) -- see PX4 messages')
            return
        job['i'] += 1
        if job['i'] >= len(job['steps']):
            self._finish(ACCEPTED, f'{name}: accepted by PX4')
        else:
            job['sent_at'], job['tries'] = now, 1
            self._send_step()

    def _on_px4_ack_mission(self, job, px4_cmd, result, now):
        if px4_cmd not in job['expect']:
            return
        name = NAMES[job['gcs']]
        if result == IN_PROGRESS:
            job['sent_at'] = now
            return
        if result != ACCEPTED:
            what = {PX4_ARM: 'arming', PX4_SET_MODE: 'OFFBOARD mode'}.get(px4_cmd, name)
            self._finish(result if result in (TEMP_REJECTED, DENIED, UNSUPPORTED, FAILED) else FAILED,
                         f'{name}: PX4 rejected {what} (result {result}) -- see PX4 messages')
            return
        job['expect'].discard(px4_cmd)
        if not job['expect'] and job['phase'] == 'px4':
            self._finish(ACCEPTED, f'{name}: accepted by PX4 (offboard)')

    def _tick_mission(self, job, now):
        name = NAMES[job['gcs']]
        if job['phase'] == 'mission' and now - job['sent_at'] > self.MISSION_ACK_TIMEOUT_S:
            if job['tries'] >= 2:
                self._finish(FAILED, f'{name}: mission_manager not responding (running?)')
                return
            job['tries'] += 1
            job['sent_at'] = now
            self.send_mission(job['mission'])       # cung id: mission_manager khong chay lai
        elif job['phase'] == 'px4' and now - job['sent_at'] > self.PX4_CONFIRM_TIMEOUT_S:
            what = 'OFFBOARD / ARM' if job['gcs'] == CMD_TAKEOFF else 'NAV_LAND'
            self._finish(FAILED, f'{name}: no PX4 ACK for {what} (offboard_control running?)')
            return
        if now - job['kept_at'] >= self.KEEPALIVE_S:
            job['kept_at'] = now
            self.reply(job['gcs'], IN_PROGRESS)

    def tick(self, now):
        self._now = now
        job = self._job
        if job is None:
            return
        if 'mission' in job:
            self._tick_mission(job, now)
            return
        if now - job['sent_at'] > self.STEP_TIMEOUT_S:
            if job['tries'] >= 2:
                self._finish(FAILED, f"{NAMES[job['gcs']]}: no ACK from PX4")
                return
            job['tries'] += 1
            job['sent_at'] = now
            self._send_step()
        if now - job['kept_at'] >= self.KEEPALIVE_S:
            job['kept_at'] = now
            self.reply(job['gcs'], IN_PROGRESS)


class MissionReceiver:
    """Nhan waypoint bang giao thuc mission MAVLink (ground -> Jetson).

    COUNT(n) -> REQUEST_INT(seq) -> ITEM_INT ... -> ACK(ACCEPTED) -> on_mission([{seq, lat, lon, alt,
    command, frame}]). Khong co ITEM sau TIMEOUT_S -> xin lai; qua RETRIES lan -> huy.
    """
    TIMEOUT_S = 1.5
    RETRIES = 5
    ACK_HOLD_S = 10.0       # ITEM cuoi gui lai (MISSION_ACK bi mat) -> tra lai ACK

    def __init__(self, mav, send, on_mission):
        self.mav, self.send, self.on_mission = mav, send, on_mission
        self._rx = None     # {'peer': (sys, comp), 'items': [...], 'next', 'sent_at', 'tries'}
        self._done = None   # (peer, so diem, t xong) cua lan nhan cuoi
        self.last = []

    def on_count(self, m, now):
        peer = (m.get_srcSystem(), m.get_srcComponent())
        if getattr(m, 'mission_type', 0) != 0:
            self.send(self.mav.mission_ack_encode(*peer, 13, m.mission_type))   # MAV_MISSION_UNSUPPORTED
            return
        if m.count == 0:
            self._accept(peer, [])
            return
        self._rx = {'peer': peer, 'items': [None] * m.count, 'next': 0, 'sent_at': now, 'tries': 0}
        self._request(now)

    def _request(self, now):
        rx = self._rx
        rx['sent_at'], rx['tries'] = now, rx['tries'] + 1
        self.send(self.mav.mission_request_int_encode(*rx['peer'], rx['next'], 0))

    def on_item(self, m, now):
        rx = self._rx
        peer = (m.get_srcSystem(), m.get_srcComponent())
        if rx is None:
            done = self._done
            # ITEM cuoi gui lai: tra lai MISSION_ACK, khong nhan lai
            if done and done[0] == peer and m.seq == done[1] - 1 and now - done[2] <= self.ACK_HOLD_S:
                self.send(self.mav.mission_ack_encode(*peer, 0, 0))
            return
        if peer != rx['peer'] or m.seq != rx['next']:
            return
        rx['items'][m.seq] = {'seq': m.seq, 'lat': m.x / 1e7, 'lon': m.y / 1e7, 'alt': float(m.z),
                              'command': m.command, 'frame': m.frame}
        rx['next'] += 1
        rx['tries'] = 0
        if rx['next'] >= len(rx['items']):
            self._rx = None
            self._done = (rx['peer'], len(rx['items']), now)
            self._accept(rx['peer'], rx['items'])
        else:
            self._request(now)

    def _accept(self, peer, items):
        self.send(self.mav.mission_ack_encode(*peer, 0, 0))                      # MAV_MISSION_ACCEPTED
        self.last = items
        self.on_mission(items)

    def tick(self, now):
        rx = self._rx
        if rx is None or now - rx['sent_at'] < self.TIMEOUT_S:
            return
        if rx['tries'] >= self.RETRIES:
            self._rx = None
            return
        self._request(now)
