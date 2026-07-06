import os
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
import threading

# Mirror control.py's opt-in verbose logging (GROUND_GUI_DEBUG=1).
DEBUG = os.environ.get("GROUND_GUI_DEBUG", "0") == "1"


class LoraBridge(QObject):
    # ---------- Signals ----------
    positionUpdated      = pyqtSignal(float, float, float)
    positionUpdatedLocal = pyqtSignal(float, float, float)
    positionUpdatedGPS   = pyqtSignal(float, float, float)

    batteryUpdated       = pyqtSignal(float, float)
    speedUpdated         = pyqtSignal(float)
    headingUpdated       = pyqtSignal(float)   # yaw deg, ENU (0°=East, CCW+)
    linkUpdated          = pyqtSignal(bool)
    connectionStarted    = pyqtSignal(bool, str)
    modePushed           = pyqtSignal(bool, str, str)
    missionAck           = pyqtSignal(bool, str)   # (ok, msg) — explicit upload result

    # New signals for UI top bar
    stateUpdated         = pyqtSignal(str)
    flightStateUpdated   = pyqtSignal(str)
    gpsStatusUpdated     = pyqtSignal(str)
    gpsFixUpdated        = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.controller = None

    def set_controller(self, controller):
        self.controller = controller
        if hasattr(self.controller, "set_gui_bridge"):
            self.controller.set_gui_bridge(self)

    # ---------- Telemetry passthrough ----------
    @pyqtSlot(float, float, float)
    def update_position(self, x, y, z):
        # JS wires positionUpdated and positionUpdatedLocal to the same handler,
        # so emit once to avoid doing the same DOM work twice per telemetry packet.
        self.positionUpdated.emit(x, y, z)

    @pyqtSlot(float, float, float)
    def update_local_position(self, x, y, z):
        self.positionUpdatedLocal.emit(x, y, z)
        self.positionUpdated.emit(x, y, z)

    @pyqtSlot(float, float, float)
    def update_global_position(self, lat, lon, alt):
        self.positionUpdatedGPS.emit(lat, lon, alt)

    @pyqtSlot(float, float)
    def update_battery(self, percent, voltage):
        self.batteryUpdated.emit(percent, voltage)

    @pyqtSlot(float)
    def update_speed(self, spd):
        self.speedUpdated.emit(spd)

    @pyqtSlot(float)
    def update_heading(self, hdg: float):
        self.headingUpdated.emit(hdg)

    @pyqtSlot(bool)
    def update_link(self, ok: bool):
        self.linkUpdated.emit(bool(ok))

    # ---------- State / GPS status ----------
    @pyqtSlot(str)
    def update_state(self, state):
        state = str(state) if state else "UNKNOWN"
        if DEBUG:
            print(f"[BRIDGE][STATE] {state}")
        self.stateUpdated.emit(state)
        self.flightStateUpdated.emit(state)

    @pyqtSlot(bool)
    def update_gps(self, ok: bool):
        status = "3D FIX" if bool(ok) else "NO FIX"
        self.gpsStatusUpdated.emit(status)
        self.gpsFixUpdated.emit(status)

    # ---------- Link control ----------
    @pyqtSlot()
    def startConnection(self):
        if not self.controller:
            print("⚠️ No controller attached.")
            return

        print("🟢 GUI requested: START LoRa connection")

        # Opening the serial port can block; run it off the GUI thread so the UI never freezes.
        def _do_start():
            self.controller.start()
            if hasattr(self.controller, "set_gui_bridge"):
                self.controller.set_gui_bridge(self)
            started = self.controller.read_position_from_drone()
            if started:
                self.connectionStarted.emit(True, "LoRa reader started")
            else:
                self.connectionStarted.emit(False, "Serial is not connected or reader already running")

        threading.Thread(target=_do_start, daemon=True).start()

    @pyqtSlot()
    def stopConnection(self):
        if not self.controller:
            print("⚠️ No controller attached.")
            return

        print("🔴 GUI requested: STOP LoRa connection")
        # stop() joins the reader/heartbeat threads and closes serial; keep it off the GUI thread.
        threading.Thread(target=self.controller.stop, daemon=True).start()

    @pyqtSlot()
    def landConnect(self):
        if self.controller:
            print("[INFO] Sending LAND request via LoRa")
            self.controller.land_req()
        else:
            print("⚠️ No controller attached.")

    @pyqtSlot()
    def offBoardConnect(self):
        if self.controller:
            print("[INFO] Sending OFFBOARD request via LoRa")
            self.controller.offboard_req()
        else:
            print("⚠️ No controller attached.")

    # ---------- Waypoint / Mission ----------
    @pyqtSlot(list)
    def receivedTargetWaypoint(self, waypoints):
        if not self.controller:
            print("⚠️ No controller attached.")
            self.modePushed.emit(False, "SENDMISSION", "No controller attached")
            return

        print(f"✅ Received {len(waypoints)} waypoints from JS:")
        for i, wp in enumerate(waypoints, 1):
            print(f"  {i}: {wp}")

        try:
            self.controller.update_waypoints(waypoints)
            self.controller.send_waypoints_to_drone()
        except Exception as e:
            print("❌ Mission error:", e)
            self.modePushed.emit(False, "SENDMISSION", str(e))

    # ---------- ACK passthrough ----------
    @pyqtSlot(bool, str, str)
    def mode_push(self, ok: bool, mode: str, msg: str):
        self.modePushed.emit(bool(ok), str(mode), str(msg))

    modePush = mode_push

    @pyqtSlot(bool, str)
    def mission_ack(self, ok: bool, msg: str):
        self.missionAck.emit(bool(ok), str(msg))

    # ---------- Gimbal passthrough placeholders ----------
    @pyqtSlot(str)
    def gimbalPtz(self, command):
        print("[GROUND][PTZ]", command)

    @pyqtSlot(bool, bool, float, float, float, float)
    def gimbalPosition(self, enable_pitch, enable_yaw, pitch_deg, yaw_deg, pitch_speed_dps, yaw_speed_dps):
        print("[GROUND][POSITION]", enable_pitch, enable_yaw, pitch_deg, yaw_deg, pitch_speed_dps, yaw_speed_dps)

    @pyqtSlot(float, float)
    def gimbalVelocity(self, yaw_norm, pitch_norm):
        print("[GROUND][VELOCITY]", yaw_norm, pitch_norm)
