import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Nut Take-off / Land / Hold / RTL / Send WP (giua duoi ban do) -> JETSON (radio 433 -> DDS -> PX4).
// Lenh thang PX4 (915) nam o o PX4 tren thanh tren cung (TopBar). Moi nut khoa theo commands.gates.jetson
// (mat Jetson, Jetson khong co du lieu PX4, khong GPS, khoa cheo khi duong PX4 dang giu quyen --
// core/commands/flight.py). Phai GIU nut xac nhan 1 giay de tranh bam nham.
Item {
    id: root
    readonly property var g: commands.gates.jetson || ({})      // action -> ly do khoa ('' = mo)
    // 🧪 che do TEST (phong thi nghiem): Take-off / Land / Send WP mo khoa, gui thang Jetson bo qua
    // dieu kien phia GCS (DDS, GPS, khoa cheo) -- de xem lenh co len Jetson + phan hoi ve khong.
    // Khong luu: tat app / bam Test lan nua -> ve binh thuong (ra ngoai bay van du dieu kien).
    property bool testMode: false
    readonly property var testable: ["takeoff", "land", "upload"]
    function open(a) { return (testMode && testable.indexOf(a) >= 0) || g[a] === "" }
    readonly property var st: commands.values
    property string confirming: ""          // "" | "takeoff" | "land" | "rtl" | "upload"
    // dang bay (PX4 EXTENDED_SYS_STATE): IN AIR / TAKING OFF / LANDING
    readonly property bool inAir: ["IN AIR", "TAKING OFF", "LANDING"].indexOf(vehicle.status.values.landed) >= 0
    readonly property var up: commands.missionUpload.values
    readonly property int wpCount: mission.items.length
    // waypoint nam trong vung cam / han che (tinh khi mo hop xac nhan gui)
    function conflicts() {
        var out = []
        for (var i = 0; i < mission.items.length; i++) {
            var z = airspace.namesAt(mission.items[i].lat, mission.items[i].lon)
            if (z !== "") out.push("#" + (i + 1) + " " + z)
        }
        return out
    }
    function uploadText() {
        switch (up.state) {
        case "sending": return "Uploading waypoint " + up.sent + "/" + up.total + "…"
        case "done":    return "✓ " + up.message
        case "timeout": return "⚠ Waypoint upload: Jetson not responding"
        case "no_link": return "⚠ " + up.message
        case "error":   return "✕ Waypoint upload: " + up.message
        default:        return ""
        }
    }

    implicitWidth: col.implicitWidth
    implicitHeight: col.implicitHeight

    function stateText() {
        var step = st.steps > 1 ? " step " + st.step + "/" + st.steps : ""
        var took = st.elapsed >= 0 ? " (" + st.elapsed.toFixed(1) + " s)" : ""
        switch (st.state) {
        case "sending":     return "Sending " + st.label + step + "… (try " + st.attempt + "/" + st.retries + ")"
        case "in_progress": return st.label + ": Jetson in progress…"
                                   + (st.attempt > 0 ? " (no reply, asking " + st.attempt + "/" + st.retries + ")" : "")
        case "accepted":    return "✓ " + st.label + ": accepted by Jetson" + took
        case "rejected":    return "✕ " + st.label + ": temporarily rejected" + step
        case "denied":      return "✕ " + st.label + ": denied by Jetson — see PX4 messages"
        // Jetson ban cu chua biet ping -> van tra ACK: lenh DA len toi Jetson
        case "unsupported": return st.label === "Link test" ? "✓ Link test: reached Jetson" + took
                                                              + " (old drone_telemetry — rebuild for ping)"
                                                            : "✕ " + st.label + ": not supported"
        case "failed":      return "✕ " + st.label + ": failed" + step
        case "timeout":     return "⚠ " + st.label + ": Jetson not responding" + took + " — check radio 433"
        case "no_link":     return "⚠ Link not open: " + commands.targetLink + " (tab Connections)"
        case "blocked":     return "✕ " + st.label + ": " + st.message
        default:            return ""
        }
    }
    // ly do khoa cua cac nut dang hien (gop trung) -> dong trang thai khi ranh
    function lockText() {
        var acts = inAir ? ["upload", "hold", "rtl", "land"] : ["takeoff", "upload", "land"]
        var seen = []
        for (var i = 0; i < acts.length; i++) {
            var why = g[acts[i]]
            if (why && seen.indexOf(why) < 0) seen.push(why)
        }
        return seen.length ? "🔒 " + seen.join("  ·  ") : ""
    }
    function stateColor() {
        if (st.state === "accepted" || (st.state === "unsupported" && st.label === "Link test")) return Theme.ok
        if (st.state === "sending" || st.state === "in_progress") return Theme.text
        return Theme.error
    }

    ColumnLayout {
        id: col
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        spacing: 8

        // ---- hop xac nhan ----
        Rectangle {
            visible: root.confirming !== ""
            Layout.alignment: Qt.AlignHCenter
            implicitWidth: confirmCol.implicitWidth + 32
            implicitHeight: confirmCol.implicitHeight + 24
            radius: Theme.radius
            color: "#f2161b22"
            border.color: root.confirming === "takeoff" ? Theme.ok : (root.confirming === "land" || root.confirming === "rtl") ? Theme.warn : Theme.accent

            ColumnLayout {
                id: confirmCol
                anchors.centerIn: parent
                spacing: 10
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: (root.testMode && root.confirming !== "rtl" ? "🧪 TEST — " : "")
                        + (root.confirming === "takeoff" ? "Take off? (" + flightEnv.state.effectiveLabel + ")"
                        : root.confirming === "land" ? "Land at current position?"
                        : root.confirming === "rtl" ? "Return to launch and land?"
                        : "Send " + root.wpCount + " waypoints to Jetson?")
                    color: Theme.text; font.pixelSize: 15; font.bold: true
                }
                Text {
                    visible: root.confirming === "takeoff" || root.confirming === "land" || root.confirming === "rtl"
                    Layout.alignment: Qt.AlignHCenter
                    text: "via Jetson · 433  (Jetson → DDS → PX4)"
                    color: Theme.textDim; font.pixelSize: 12
                }
                RowLayout {
                    visible: root.confirming === "takeoff"
                    Layout.alignment: Qt.AlignHCenter
                    Text { text: "Altitude"; color: Theme.textDim; font.pixelSize: 12 }
                    // don vi noi bo = dm (SpinBox chi nhan so nguyen) -> buoc 0.5 m, hien 1 so le
                    SpinBox {
                        id: altBox
                        readonly property real meters: value / 10
                        from: 5; to: 1200; stepSize: 5; editable: true
                        // mac dinh theo moi truong (Cai dat -> Moi truong bay)
                        value: Math.round((commands.takeoffAltitudes[flightEnv.state.effective] || 1.5) * 10)
                        textFromValue: (v, locale) => (v / 10).toFixed(1)
                        valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 10)
                        validator: DoubleValidator { bottom: 0.5; top: 120; decimals: 1 }
                    }
                    Text { text: "m"; color: Theme.textDim }
                }
                // ---- Pre-flight check (chi khi Take-off): van de nghiem trong -> khoa nut ----
                ColumnLayout {
                    objectName: "preflight"
                    visible: root.confirming === "takeoff"
                    Layout.maximumWidth: 420
                    spacing: 3
                    Text {
                        text: alerts.critical > 0 ? "⛔ Not ready for take-off"
                            : (alerts.warning > 0 ? "✓ Ready (" + alerts.warning + " warnings)" : "✓ Pre-flight check: ready")
                        color: alerts.critical > 0 ? Theme.error : Theme.ok; font.pixelSize: 13; font.bold: true
                    }
                    Repeater {
                        model: root.confirming === "takeoff" ? alerts.items : []
                        delegate: Text {
                            required property var modelData
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                            text: (modelData.level === "critical" ? "⛔ " : "⚠ ") + modelData.text
                            color: modelData.level === "critical" ? Theme.error : Theme.warn; font.pixelSize: 12
                        }
                    }
                }
                Text {
                    id: conflictText
                    readonly property var list: root.confirming === "upload" ? (airspace.revision, root.conflicts()) : []
                    visible: list.length > 0
                    Layout.maximumWidth: 420; wrapMode: Text.WordWrap
                    text: "⚠ " + list.length + " waypoint(s) inside no-fly / restricted zones:\n" + list.join("\n")
                    color: Theme.error; font.pixelSize: 12; font.bold: true
                }
                RowLayout {
                    visible: root.confirming === "upload"
                    Layout.alignment: Qt.AlignHCenter
                    spacing: 10
                    Button { text: "Cancel"; onClicked: root.confirming = "" }
                    Button {
                        objectName: "uploadConfirm"
                        text: conflictText.visible ? "Send anyway" : "Send"
                        highlighted: true
                        onClicked: {
                            if (root.testMode) commands.testUploadMission(); else commands.uploadMission()
                            root.confirming = ""
                        }
                    }
                }
                RowLayout {
                    visible: root.confirming === "takeoff" || root.confirming === "land" || root.confirming === "rtl"
                    spacing: 10
                    Button { text: "Cancel"; onClicked: root.confirming = "" }
                    // giu 1 giay de xac nhan
                    Rectangle {
                        id: holdBtn
                        objectName: "holdConfirm"
                        // khoa: dieu kien cua lenh mat trong luc mo hop (gates), hoac Take-off co van de
                        // nghiem trong o pre-flight
                        readonly property bool testing: root.testMode && root.testable.indexOf(root.confirming) >= 0
                        readonly property string gateWhy: testing ? "" : (root.g[root.confirming] || "")
                        readonly property bool blocked: gateWhy !== ""
                                                        || (!testing && root.confirming === "takeoff" && alerts.critical > 0)
                        enabled: !blocked
                        opacity: blocked ? 0.4 : 1
                        implicitWidth: Math.max(200, holdLabel.implicitWidth + 24); implicitHeight: 40; radius: 6
                        color: Theme.panelAlt; border.color: Theme.border; clip: true
                        property real progress: 0
                        Rectangle {
                            width: parent.width * parent.progress; height: parent.height
                            color: root.confirming === "takeoff" ? Theme.ok : Theme.warn
                        }
                        Text {
                            id: holdLabel
                            anchors.centerIn: parent
                            text: holdBtn.gateWhy !== "" ? "Locked — " + holdBtn.gateWhy
                                : holdBtn.blocked ? "Locked — fix critical issues" : "Hold to confirm"
                            color: Theme.text; font.pixelSize: 13; font.bold: true
                        }
                        NumberAnimation on progress {
                            id: holdAnim
                            running: false; from: 0; to: 1; duration: 1000
                            onFinished: {
                                if (holdBtn.progress < 1) return
                                if (holdBtn.testing && root.confirming === "takeoff") commands.testTakeoff(altBox.meters)
                                else if (holdBtn.testing) commands.testLand()
                                else if (root.confirming === "takeoff") commands.takeoff(altBox.meters)
                                else if (root.confirming === "rtl") commands.rtl()
                                else commands.land()
                                root.confirming = ""
                                holdBtn.progress = 0
                            }
                        }
                        MouseArea {
                            anchors.fill: parent
                            onPressed: holdAnim.restart()
                            onReleased: { holdAnim.stop(); holdBtn.progress = 0 }
                            onCanceled: { holdAnim.stop(); holdBtn.progress = 0 }
                        }
                    }
                }
            }
        }

        // ---- bang TEST (phong thi nghiem): gui toi Jetson bo qua dieu kien phia GCS ----
        Rectangle {
            objectName: "testPanel"
            visible: root.testMode && root.confirming === ""
            Layout.alignment: Qt.AlignHCenter
            implicitWidth: Math.max(420, testCol.implicitWidth + 28)
            implicitHeight: testCol.implicitHeight + 24
            radius: Theme.radius
            color: "#f2161b22"; border.color: Theme.warn; border.width: 2

            ColumnLayout {
                id: testCol
                anchors.fill: parent; anchors.margins: 14
                spacing: 8
                Text { text: "🧪 TEST MODE — Jetson link (radio 433)"; color: Theme.warn; font.pixelSize: 15; font.bold: true }
                Text {
                    Layout.fillWidth: true; Layout.maximumWidth: 460; wrapMode: Text.WordWrap
                    text: "Take-off / Land / Send WP below are UNLOCKED and go straight to Jetson (GCS checks skipped: "
                        + "PX4 DDS, GPS, cross-lock) — to see whether commands reach Jetson and replies come back. "
                        + "Jetson still applies its own rules: no PX4 → replies \"no PX4 data\". "
                        + "If PX4 IS connected via DDS and on the ground, Take-off WILL ARM — remove propellers. "
                        + "Press 🧪 Test again to leave test mode."
                    color: Theme.warn; font.pixelSize: 12
                }
                Button {
                    objectName: "pingBtn"
                    text: "📡 Ping Jetson"
                    ToolTip.visible: hovered; ToolTip.text: "Jetson replies ACK + message, does nothing with PX4"
                    onClicked: commands.ping()
                }
                // tra loi cua Jetson / PX4 (STATUSTEXT) moi nhat -- thay ngay lenh co len toi Jetson khong
                Text { text: "Latest messages"; color: Theme.textDim; font.pixelSize: 11 }
                Repeater {
                    model: Math.min(3, vehicle.messages.items.length)
                    delegate: Text {
                        required property int index
                        readonly property var msg: vehicle.messages.items[index] || ({})
                        Layout.fillWidth: true; Layout.maximumWidth: 460; elide: Text.ElideRight
                        text: msg.time + "  " + msg.text
                        color: index === 0 ? Theme.text : Theme.textDim
                        font.pixelSize: 12; font.family: "monospace"
                    }
                }
            }
        }

        // ---- 2 nut chinh ----
        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 12
            component CmdBtn: Rectangle {
                property string label: ""
                property color tint: Theme.ok
                property bool active: false
                signal clicked()
                implicitWidth: 136; implicitHeight: 46; radius: 23
                opacity: enabled ? 1 : 0.45
                color: active ? tint : "#e6161b22"
                border.color: tint; border.width: 2
                Text { anchors.centerIn: parent; text: parent.label; color: "white"; font.pixelSize: 15; font.bold: true }
                MouseArea { anchors.fill: parent; enabled: parent.enabled; onClicked: parent.clicked() }
            }
            CmdBtn {
                objectName: "takeoffBtn"
                label: "⬆  Take-off"; tint: Theme.ok
                // dang bay: an Take-off (tranh bam nham), chi con Send WP / Land
                visible: !root.inAir
                enabled: root.open("takeoff")
                active: root.confirming === "takeoff"
                onClicked: root.confirming = root.confirming === "takeoff" ? "" : "takeoff"
            }
            CmdBtn {
                objectName: "uploadBtn"
                label: root.up.busy ? "■  Cancel upload" : "⇪  Send WP (" + root.wpCount + ")"
                tint: Theme.accent
                enabled: root.up.busy || (root.open("upload") && root.wpCount > 0)
                active: root.confirming === "upload" || root.up.busy
                onClicked: {
                    if (root.up.busy) { commands.cancelUpload(); return }
                    root.confirming = root.confirming === "upload" ? "" : "upload"
                }
            }
            // dang bay: Hold (dung ngay, khong can xac nhan) + RTL (can GPS -> khoa khi trong nha)
            CmdBtn {
                objectName: "holdModeBtn"
                visible: root.inAir
                label: "⏸  Hold"; tint: Theme.accent
                enabled: root.open("hold")
                onClicked: { root.confirming = ""; commands.hold() }
            }
            CmdBtn {
                objectName: "rtlBtn"
                visible: root.inAir
                label: "⌂  RTL"; tint: Theme.warn
                enabled: root.open("rtl")
                active: root.confirming === "rtl"
                onClicked: root.confirming = root.confirming === "rtl" ? "" : "rtl"
            }
            CmdBtn {
                objectName: "landBtn"
                label: "⬇  Land"; tint: Theme.warn
                enabled: root.open("land")
                active: root.confirming === "land"
                onClicked: root.confirming = root.confirming === "land" ? "" : "land"
            }
            CmdBtn {
                objectName: "testBtn"
                // luon bam duoc: kiem tra duong Jetson khi cac nut khac dang khoa (phong thi nghiem)
                label: root.testMode ? "🧪  Test ON" : "🧪  Test"; tint: root.testMode ? Theme.warn : Theme.textDim
                implicitWidth: root.testMode ? 120 : 96
                active: root.testMode
                onClicked: { root.testMode = !root.testMode; root.confirming = "" }
            }
        }

        // ---- trang thai lenh ----
        Rectangle {
            readonly property string busyMsg: [root.stateText(), root.uploadText()].filter(t => t !== "").join("   ·   ")
            // ranh: ly do khoa (che do test: nhac dang bo qua dieu kien)
            readonly property string msg: busyMsg !== "" ? busyMsg
                                        : root.testMode ? "🧪 TEST MODE — GCS checks off, buttons send straight to Jetson"
                                        : root.lockText()
            visible: msg !== ""
            Layout.alignment: Qt.AlignHCenter
            implicitWidth: stText.implicitWidth + 20; implicitHeight: 26; radius: 6
            color: "#cc161b22"
            Text {
                id: stText
                anchors.centerIn: parent
                text: parent.msg
                color: parent.busyMsg === "" ? Theme.textDim
                     : (root.st.state === "idle" && (root.up.state === "done" || root.up.state === "sending")) ? Theme.ok
                     : root.stateColor()
                font.pixelSize: 12
            }
        }
    }
}
