"""Ground station commands (radio 433) routed to mission_manager (/gcs/command) or PX4 (/fmu/in/vehicle_command).

No ROS dependency; the ROS node wires up the callbacks:
    send_px4(command, params[7])   publish a VehicleCommand
    reply(gcs_command, result)     COMMAND_ACK to the ground station
    status(text, severity)         STATUSTEXT to the ground station (e.g. rejection reason)

Take-off / Land have two routes (telemetry.yaml commands.takeoff_land_via):
    mission (default): mission_manager (/gcs/command) -> offboard_control -> PX4 OFFBOARD over DDS.
        Take-off is ACCEPTED once mission_manager accepts and PX4 acknowledges DO_SET_MODE (OFFBOARD) and
        ARM sent by offboard_control. Land is ACCEPTED once PX4 acknowledges NAV_LAND. The ground sees
        IN_PROGRESS meanwhile; a rejection or silence from mission_manager or PX4 is reported with a
        reason (STATUSTEXT).
    px4: send VehicleCommands directly, as below (PX4 auto Takeoff / Land modes).

Ground command -> what is sent to PX4 (px4 route; Hold and RTL always use it):
    NAV_TAKEOFF (22, param7 = altitude above take-off point, m)
        [ARM if disarmed] -> NAV_TAKEOFF (param7 = current AMSL + altitude; without a global altitude,
        e.g. indoors, NaN so PX4 uses MIS_TAKEOFF_ALT). PX4 climbs, then holds.
    NAV_LAND (21)               -> NAV_LAND in place
    NAV_LOITER_UNLIM (17)       -> DO_SET_MODE AUTO.LOITER (Hold)
    NAV_RETURN_TO_LAUNCH (20)   -> NAV_RETURN_TO_LAUNCH (needs a global position)
    MISSION_START (300)         -> mission_manager flies the uploaded mission (from HOVER);
                                   param3 = option bitmask (bit 0: obstacle avoidance),
                                   param4 = speed between waypoints (m/s), param5 / param6 = arrival
                                   tolerance xy / z (m); 0 = mission_manager default
    DO_PAUSE_CONTINUE (193)     -> param1 = 1: mission_manager continues after a manual task or a pause
    USER_1 (31010) = PING       -> nothing sent to PX4; reply ACCEPTED + STATUSTEXT (ground <-> Jetson link test)
    USER_2 (31011) = SETTINGS   -> forwarded to mission_manager at once (speed, xy / z tolerance of the running
                                   mission); never replaces the command in progress

Safety: Take-off only on the ground; Land / Hold / RTL only in the air; PX4 data older than PX4_FRESH_S
rejects every command. A new command replaces the running one.

A resent command (COMMAND_LONG.confirmation > 0) is never executed twice:
  * still running   -> IN_PROGRESS
  * just finished   -> the previous result (its ACK was lost on the radio), kept for RESULT_HOLD_S
                       (>= ground retries x ack_timeout)
"""

import math

# MAV_RESULT (MAVLink) and VEHICLE_CMD_RESULT (PX4) share values
ACCEPTED, TEMP_REJECTED, DENIED, UNSUPPORTED, FAILED, IN_PROGRESS = 0, 1, 2, 3, 4, 5

CMD_HOLD = 17          # MAV_CMD_NAV_LOITER_UNLIM
CMD_RTL = 20
CMD_LAND = 21
CMD_TAKEOFF = 22
CMD_MISSION_START = 300   # MAV_CMD_MISSION_START
CMD_PAUSE_CONTINUE = 193  # MAV_CMD_DO_PAUSE_CONTINUE (param1 = 1: continue)
OPT_AVOIDANCE = 1         # MISSION_START param3 bit 0
CMD_PING = 31010       # MAV_CMD_USER_1: ground <-> Jetson ping
CMD_SETTINGS = 31011   # MAV_CMD_USER_2: live mission settings (p1 speed, p2 xy / p3 z tolerance; 0 = unchanged)
PX4_SET_MODE = 176     # VEHICLE_CMD_DO_SET_MODE
PX4_ARM = 400          # VEHICLE_CMD_COMPONENT_ARM_DISARM

NAMES = {CMD_TAKEOFF: 'Take-off', CMD_LAND: 'Land', CMD_HOLD: 'Hold', CMD_RTL: 'RTL',
         CMD_MISSION_START: 'Start mission', CMD_PAUSE_CONTINUE: 'Continue'}
MISSION_CMDS = {CMD_TAKEOFF: 'takeoff', CMD_LAND: 'land', CMD_MISSION_START: 'mission',
                CMD_PAUSE_CONTINUE: 'continue'}
NAN = float('nan')

SEV_WARNING, SEV_INFO = 4, 6


class CommandBridge:
    PX4_FRESH_S = 2.0        # reject commands if PX4 data is older
    STEP_TIMEOUT_S = 3.0     # step without ACK: resend once, then FAILED
    KEEPALIVE_S = 0.5        # IN_PROGRESS period (< ground ack_timeout of 1.5 s)
    RESULT_HOLD_S = 10.0     # >= ground retries x ack_timeout (3 x 1.5 s)
    MISSION_ACK_TIMEOUT_S = 1.0     # wait for the mission_manager ACK; resend once
    PX4_CONFIRM_TIMEOUT_S = 6.0     # offboard_control streams setpoints ~1 s before requesting OFFBOARD + ARM

    def __init__(self, send_px4, reply, status, send_mission=None):
        """send_mission(dict): Take-off / Land via mission_manager; None sends them directly to PX4."""
        self.send_px4, self.reply, self.status = send_px4, reply, status
        self.send_mission = send_mission
        self._req = 0
        self.landed = None
        self.armed = None
        self.amsl = None          # m AMSL; None without a global position
        self._px4_at = None
        self._job = None          # {'gcs', 'steps', 'i', 'sent_at', 'tries', 'kept_at'}
        self._last = None         # (command, result, finish time) for resent commands
        self.pings = 0
        self._now = 0.0

    # --- PX4 state (from /fmu/out topics) ------------------------------------------------
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

    # --- ground commands ------------------------------------------------------------------
    def handle(self, cmd, params, now, confirmation=0):
        self._now = now
        if cmd == CMD_PING:                      # ping: reply immediately, PX4 not involved
            self.pings += 1
            self.reply(cmd, ACCEPTED)
            self.status(f'Link test OK: Jetson got ping #{self.pings}'
                        + (f' (retry {confirmation})' if confirmation else ''), SEV_INFO)
            return
        if cmd == CMD_SETTINGS:                  # live settings: forwarded, never a job
            self._forward_settings(params)
            return
        name = NAMES.get(cmd)
        if name is None:
            self.reply(cmd, UNSUPPORTED)
            return
        job = self._job
        if job and job['gcs'] == cmd:            # resent command
            self.reply(cmd, IN_PROGRESS)
            return
        last = self._last
        if (confirmation > 0 and job is None and last and last[0] == cmd
                and now - last[2] <= self.RESULT_HOLD_S):
            self.reply(cmd, last[1])             # just finished: resend the result
            return
        if self._px4_at is None or now - self._px4_at > self.PX4_FRESH_S or self.landed is None:
            self._deny(cmd, TEMP_REJECTED, f'{name}: no PX4 data (DDS agent?)')
            return

        if cmd == CMD_TAKEOFF and not self.landed:
            self._deny(cmd, DENIED, 'Take-off: already flying')
            return
        # Land during take-off (armed, or Take-off running but not armed yet) is allowed; mission_manager cancels
        takeoff_running = job is not None and job['gcs'] == CMD_TAKEOFF
        abort_takeoff = self.send_mission and cmd == CMD_LAND and (self.armed or takeoff_running)
        if cmd != CMD_TAKEOFF and self.landed and not abort_takeoff:
            self._deny(cmd, DENIED, f'{name}: vehicle is on the ground')
            return
        if cmd == CMD_PAUSE_CONTINUE and not (len(params) > 0 and params[0] == 1):
            self._deny(cmd, UNSUPPORTED, 'Pause: not supported (use Hold)')
            return
        if self.send_mission and cmd in MISSION_CMDS:
            self._start_mission(cmd, name, params, now, job)
            return
        if cmd in (CMD_MISSION_START, CMD_PAUSE_CONTINUE):   # only supported via mission_manager
            self._deny(cmd, UNSUPPORTED, f'{name}: needs commands.takeoff_land_via = mission')
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

        if job:                                   # a new command replaces the running one
            self.status(f"{NAMES[job['gcs']]} superseded by {name}", SEV_INFO)
        self._job = {'gcs': cmd, 'steps': steps, 'i': 0, 'sent_at': now, 'tries': 1, 'kept_at': now}
        self.reply(cmd, IN_PROGRESS)
        self._send_step()

    def _forward_settings(self, params):
        """Send live mission settings to mission_manager and acknowledge at once. They do not touch PX4 and
        do not replace the command in progress; mission_manager reports the values it applied (STATUSTEXT)."""
        if not self.send_mission:
            self.reply(CMD_SETTINGS, UNSUPPORTED)
            return

        def positive(i):
            return float(params[i]) if len(params) > i and math.isfinite(params[i]) and params[i] > 0 else 0.0

        self._req += 1
        self.send_mission({'id': self._req, 'cmd': 'settings', 'options': {
            'speed': positive(0), 'xy_tolerance': positive(1), 'z_tolerance': positive(2)}})
        self.reply(CMD_SETTINGS, ACCEPTED)

    # --- Take-off / Land via mission_manager (OFFBOARD) ----------------------------------
    def _start_mission(self, cmd, name, params, now, job):
        rel = params[6] if len(params) > 6 and math.isfinite(params[6]) else None
        self._req += 1
        req = {'id': self._req, 'cmd': MISSION_CMDS[cmd]}
        if cmd == CMD_TAKEOFF:
            req['alt'] = rel
        if cmd == CMD_MISSION_START:
            bits = int(params[2]) if len(params) > 2 and math.isfinite(params[2]) else 0
            def positive(i):
                return float(params[i]) if len(params) > i and math.isfinite(params[i]) and params[i] > 0 else 0.0
            req['options'] = {'avoidance': bool(bits & OPT_AVOIDANCE), 'speed': positive(3),
                              'xy_tolerance': positive(4), 'z_tolerance': positive(5)}
        if job:
            self.status(f"{NAMES[job['gcs']]} superseded by {name}", SEV_INFO)
        # PX4 ACKs to wait for: Take-off -> DO_SET_MODE (OFFBOARD) + ARM; Land -> NAV_LAND.
        # None for Land before ARM (no PX4 command is sent), Start mission and Continue (only setpoints change).
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
        """Reply from mission_manager (/gcs/command_ack)."""
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
        if not job['expect']:                       # no PX4 ACK left to wait for
            if job['gcs'] in (CMD_MISSION_START, CMD_PAUSE_CONTINUE):
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

    # --- PX4 ACKs (/fmu/out/vehicle_command_ack) -----------------------------------------
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
            self.send_mission(job['mission'])       # same id: mission_manager does not run it again
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
    """Receive waypoints with the MAVLink mission protocol (ground -> Jetson).

    COUNT(n) -> REQUEST_INT(seq) -> ITEM_INT ... -> ACK(ACCEPTED) -> on_mission([{seq, lat, lon, alt,
    command, frame, params}]). A missing ITEM is requested again after TIMEOUT_S; after RETRIES the upload is dropped.
    """
    TIMEOUT_S = 1.5
    RETRIES = 5
    ACK_HOLD_S = 10.0       # last ITEM resent (MISSION_ACK lost): resend the ACK

    def __init__(self, mav, send, on_mission):
        self.mav, self.send, self.on_mission = mav, send, on_mission
        self._rx = None     # {'peer': (sys, comp), 'items': [...], 'next', 'sent_at', 'tries'}
        self._done = None   # (peer, item count, finish time) of the last upload
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
            # last ITEM resent: resend MISSION_ACK, do not restart the upload
            if done and done[0] == peer and m.seq == done[1] - 1 and now - done[2] <= self.ACK_HOLD_S:
                self.send(self.mav.mission_ack_encode(*peer, 0, 0))
            return
        if peer != rx['peer'] or m.seq != rx['next']:
            return
        params = [float(m.param1), float(m.param2), float(m.param3), float(m.param4)]
        rx['items'][m.seq] = {'seq': m.seq, 'lat': m.x / 1e7, 'lon': m.y / 1e7, 'alt': float(m.z),
                              'command': m.command, 'frame': m.frame,
                              'params': [v if math.isfinite(v) else None for v in params]}
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
