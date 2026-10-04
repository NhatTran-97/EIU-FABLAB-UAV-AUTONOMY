import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "../components"

// Telemetry: GIA TRI hien tai cua drone (vi tri, tu the, van toc, gimbal, dinh vi, trang thai bay)
// + do thi thoi gian thuc. Suc khoe thiet bi (healthy / loi / tan so node) o tab Diagnostics.
ScrollView {
    id: page
    clip: true
    contentWidth: availableWidth

    readonly property var st: vehicle.status.values
    readonly property var gps: vehicle.gps.values
    readonly property var fl: vehicle.flight.values
    readonly property var att: vehicle.attitude.values
    readonly property var pose: vehicle.localPose.values
    readonly property var gim: vehicle.gimbal.values
    readonly property var m: telemetry.msgs
    property bool localFrame: false

    function has(x) { return x !== null && x !== undefined }
    function f(x, d, u) { return has(x) ? Number(x).toFixed(d) + (u ? " " + u : "") : "" }
    function age(type) { return m[type] ? m[type].age : -1 }
    function rate(type) { return m[type] ? m[type].rate : -1 }
    function newest() {                       // tuoi nho nhat trong cac message
        var best = -1
        for (var i = 0; i < arguments.length; i++) {
            var a = age(arguments[i])
            if (a >= 0 && (best < 0 || a < best)) best = a
        }
        return best
    }
    function duration(sec) {
        var mm = Math.floor(sec / 60), ss = sec % 60
        return (mm < 10 ? "0" : "") + mm + ":" + (ss < 10 ? "0" : "") + ss
    }
    function wrap180(d) { return ((d + 540) % 360) - 180 }

    ColumnLayout {
        width: page.availableWidth
        spacing: Theme.gap

        RowLayout {
            Text { text: "Telemetry"; color: Theme.text; font.pixelSize: 20; font.bold: true }
            Text { text: "  live vehicle values  ·  device health: Diagnostics"; color: Theme.textDim; font.pixelSize: 12 }
        }

        // Luoi: so cot theo do rong; cac the cung hang CAO BANG NHAU (Layout.fillHeight)
        GridLayout {
            id: cards
            Layout.fillWidth: true
            columns: Math.max(1, Math.floor((page.availableWidth + Theme.gap) / (340 + Theme.gap)))
            columnSpacing: Theme.gap; rowSpacing: Theme.gap

            // ---------------- Position ----------------
            TelemetryCard {
                Layout.fillHeight: true; Layout.fillWidth: true; Layout.maximumWidth: 420
                title: "Position"
                expectedHz: 4
                icon: appInfo.imagesUrl + "position_icon.png"
                iconBg: Theme.panelAlt
                source: page.localFrame ? "Jetson · DRONE_LOCAL_POSE (PX4 vehicle_odometry)" : "PX4 · GLOBAL_POSITION_INT / LOCAL_POSITION_NED"
                ageS: page.localFrame ? page.age("DRONE_LOCAL_POSE") : page.newest("GLOBAL_POSITION_INT", "LOCAL_POSITION_NED")
                rateHz: page.localFrame ? page.rate("DRONE_LOCAL_POSE")
                                        : Math.max(page.rate("GLOBAL_POSITION_INT"), page.rate("LOCAL_POSITION_NED"))
                rows: page.localFrame
                      ? [["East (X)", page.f(pose.e, 2, "m")], ["North (Y)", page.f(pose.n, 2, "m")], ["Up (Z)", page.f(pose.u, 2, "m")]]
                      : [["Latitude", page.has(gps.lat) && gps.lat !== 0 ? gps.lat.toFixed(7) + "°" : ""],
                         ["Longitude", page.has(gps.lon) && gps.lon !== 0 ? gps.lon.toFixed(7) + "°" : ""],
                         ["Altitude AMSL", page.f(fl.altAmsl, 1, "m")],
                         [fl.altSource === "local" ? "Altitude (sensor)" : "Altitude Rel", page.f(fl.altRel, 2, "m")]]
                extra: Row {
                    spacing: 0
                    Repeater {
                        model: [{ id: false, t: "Global" }, { id: true, t: "Local ENU" }]
                        delegate: Rectangle {
                            required property var modelData
                            width: 96; height: 28
                            color: page.localFrame === modelData.id ? Theme.accent : Theme.panelAlt
                            border.color: Theme.border
                            Text { anchors.centerIn: parent; text: parent.modelData.t; color: Theme.text; font.pixelSize: 12 }
                            MouseArea { anchors.fill: parent; onClicked: page.localFrame = parent.modelData.id }
                        }
                    }
                }
            }

            // ---------------- Attitude ----------------
            TelemetryCard {
                Layout.fillHeight: true; Layout.fillWidth: true; Layout.maximumWidth: 420
                title: "Attitude"
                expectedHz: 4
                icon: appInfo.imagesUrl + "attitude_icon.png"
                iconBg: Theme.panelAlt
                source: "PX4 · ATTITUDE"
                ageS: page.age("ATTITUDE"); rateHz: page.rate("ATTITUDE")
                rows: [["Roll", page.f(att.roll, 1, "°")], ["Pitch", page.f(att.pitch, 1, "°")],
                       ["Heading (yaw)", page.f(att.heading, 1, "°")]]
            }

            // ---------------- Velocity ----------------
            TelemetryCard {
                Layout.fillHeight: true; Layout.fillWidth: true; Layout.maximumWidth: 420
                title: "Velocity"
                expectedHz: 4
                icon: appInfo.imagesUrl + "velocity_icon.png"
                iconBg: Theme.panelAlt
                source: "PX4 · VFR_HUD  +  Jetson · DRONE_LOCAL_POSE"
                ageS: page.newest("VFR_HUD", "DRONE_LOCAL_POSE"); rateHz: page.rate("VFR_HUD")
                rows: [["Ground speed", page.f(fl.groundSpeed, 2, "m/s")], ["Vertical speed", page.f(fl.climbRate, 2, "m/s")],
                       ["V East", pose.valid ? page.f(pose.ve, 2, "m/s") : ""], ["V North", pose.valid ? page.f(pose.vn, 2, "m/s") : ""],
                       ["V Up", pose.valid ? page.f(pose.vu, 2, "m/s") : ""]]
            }

            // ---------------- Gimbal ----------------
            TelemetryCard {
                Layout.fillHeight: true; Layout.fillWidth: true; Layout.maximumWidth: 420
                title: "Gimbal"
                expectedHz: 1.5
                icon: appInfo.imagesUrl + "gimbal_icon.png"
                iconBg: Theme.panelAlt
                source: "Jetson · DRONE_GIMBAL_STATE (/gimbal_state)"
                ageS: page.age("DRONE_GIMBAL_STATE"); rateHz: page.rate("DRONE_GIMBAL_STATE")
                rows: [["Yaw (vs body)", page.f(gim.yaw, 1, "°")], ["Pitch", page.f(gim.pitch, 1, "°")],
                       ["Roll", page.f(gim.roll, 1, "°")],
                       ["Drone heading", page.f(att.heading, 1, "°")],
                       ["Gimbal heading", page.has(gim.yaw) && att.valid ? ((att.heading + gim.yaw + 360) % 360).toFixed(1) + "°" : ""],
                       ["Mode", gim.valid ? gim.mode + (gim.connected ? "" : " · disconnected") : ""]]
            }

            // ---------------- Localization ----------------
            TelemetryCard {
                Layout.fillHeight: true; Layout.fillWidth: true; Layout.maximumWidth: 420
                readonly property bool indoor: flightEnv.state.effective === "indoor"
                title: "Localization"
                expectedHz: 1
                icon: appInfo.imagesUrl + "localization_icon.png"
                iconBg: Theme.panelAlt
                source: "PX4 · GPS_RAW_INT  +  EKF2 config"
                ageS: page.newest("GPS_RAW_INT", "LOCAL_POSITION_NED"); rateHz: page.rate("GPS_RAW_INT")
                rows: [["Source", indoor ? "Optical flow + range (EKF2)" : "GPS + EKF2"],
                       ["GPS fix", gps.fix], ["Satellites", String(gps.satellites)],
                       ["HDOP", page.f(gps.hdop, 2, "")], ["H accuracy", page.f(gps.hAcc, 2, "m")],
                       ["V accuracy", page.f(gps.vAcc, 2, "m")]]
            }

            // ---------------- Flight state ----------------
            TelemetryCard {
                Layout.fillHeight: true; Layout.fillWidth: true; Layout.maximumWidth: 420
                title: "Flight state"
                expectedHz: 0.8
                icon: appInfo.imagesUrl + "flight_state_icon.png"
                iconBg: Theme.panelAlt
                source: "PX4 · HEARTBEAT / EXTENDED_SYS_STATE"
                ageS: page.age("HEARTBEAT"); rateHz: page.rate("HEARTBEAT")
                staleAfter: 3.5
                rows: [["Mode", st.linkAlive ? String(st.mode).toUpperCase() : ""],
                       ["Arming", st.linkAlive ? (st.armed ? "ARMED" : "DISARMED") : ""],
                       ["Landed state", st.landed], ["Flight time", page.duration(st.flightTime || 0)],
                       ["Failsafe", st.linkAlive ? (st.failsafe ? "ACTIVE" : "None") : ""]]
            }
        }

        // ---------------- Real-time plot ----------------
        Rectangle {
            Layout.fillWidth: true
            // chiem phan con lai cua man hinh (toi thieu 300 px)
            Layout.preferredHeight: Math.max(300, page.availableHeight - cards.height - 90)
            radius: Theme.radius; color: Theme.panel; border.color: Theme.border

            ColumnLayout {
                anchors.fill: parent; anchors.margins: 14
                spacing: 8
                RowLayout {
                    Text { text: "Real-time"; color: Theme.text; font.pixelSize: 15; font.bold: true }
                    Item { width: 16 }
                    Text { text: "Signal"; color: Theme.textDim; font.pixelSize: 12 }
                    ComboBox { id: sig; model: telemetry.signalNames; implicitWidth: 220 }
                    Text { text: "Range"; color: Theme.textDim; font.pixelSize: 12 }
                    ComboBox { id: rng; model: ["30 s", "60 s", "120 s", "300 s"]; currentIndex: 1; implicitWidth: 100 }
                    Button { text: plot.paused ? "▶ Resume" : "⏸ Pause"; onClicked: plot.paused = !plot.paused }
                    Item { Layout.fillWidth: true }
                    Text { id: statVal; color: Theme.textDim; font.pixelSize: 13; font.family: "monospace" }
                    Text { id: lastVal; color: Theme.text; font.pixelSize: 16; font.bold: true; font.family: "monospace" }
                }
                Canvas {
                    id: plot
                    Layout.fillWidth: true; Layout.fillHeight: true
                    property var pts: []
                    property bool paused: false
                    readonly property real windowS: parseInt(rng.currentText)
                    Connections {
                        target: telemetry
                        enabled: page.visible              // trang an -> khong ve lai (tiet kiem CPU)
                        function onChanged() {
                            if (plot.paused) return
                            plot.pts = telemetry.series(sig.currentText, plot.windowS)
                            var lo = Infinity, hi = -Infinity
                            for (var i = 0; i < plot.pts.length; i++) { lo = Math.min(lo, plot.pts[i][1]); hi = Math.max(hi, plot.pts[i][1]) }
                            statVal.text = plot.pts.length ? "min " + lo.toFixed(2) + "  max " + hi.toFixed(2) + "   now " : ""
                            lastVal.text = plot.pts.length ? plot.pts[plot.pts.length - 1][1].toFixed(2) : "—"
                            plot.requestPaint()
                        }
                    }
                    onPaint: {
                        var ctx = getContext("2d"), w = width, h = height, padL = 54, padB = 22
                        ctx.reset()
                        ctx.fillStyle = "#0d1117"; ctx.fillRect(0, 0, w, h)
                        if (pts.length < 2) {
                            ctx.fillStyle = "#8b949e"; ctx.font = "13px sans-serif"
                            ctx.fillText("No data yet", w / 2 - 50, h / 2); return
                        }
                        var lo = pts[0][1], hi = lo
                        for (var i = 1; i < pts.length; i++) { lo = Math.min(lo, pts[i][1]); hi = Math.max(hi, pts[i][1]) }
                        if (hi - lo < 1e-3) { hi += 0.5; lo -= 0.5 }
                        var pad = (hi - lo) * 0.1; hi += pad; lo -= pad
                        var X = function (t) { return padL + (t + windowS) / windowS * (w - padL - 8) }
                        var Y = function (v) { return 6 + (hi - v) / (hi - lo) * (h - padB - 12) }
                        // luoi + nhan
                        ctx.strokeStyle = "#2a3240"; ctx.lineWidth = 1; ctx.fillStyle = "#8b949e"; ctx.font = "11px monospace"
                        for (var k = 0; k <= 4; k++) {
                            var v = lo + (hi - lo) * k / 4, y = Y(v)
                            ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - 8, y); ctx.stroke()
                            ctx.fillText(v.toFixed(2), 4, y + 4)
                        }
                        for (var s = 0; s <= 4; s++) {
                            var t = -windowS + windowS * s / 4
                            ctx.fillText((t === 0 ? "now" : t.toFixed(0) + " s"), X(t) - 14, h - 4)
                        }
                        // duong du lieu
                        ctx.strokeStyle = "#5b9cf0"; ctx.lineWidth = 2; ctx.beginPath()
                        for (var j = 0; j < pts.length; j++) {
                            var px = X(pts[j][0]), py = Y(pts[j][1])
                            // khoang trong > 1 s (mat du lieu) -> ngat net, khong noi qua
                            if (j === 0 || pts[j][0] - pts[j - 1][0] > 1.0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
                        }
                        ctx.stroke()
                    }
                }
            }
        }
    }
}
