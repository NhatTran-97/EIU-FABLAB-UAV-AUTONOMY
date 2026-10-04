import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Nut Take-off / Land (giua duoi ban do). Lenh gui toi Jetson qua radio 433 (commands.*),
// phai GIU nut xac nhan 1 giay de tranh bam nham.
Item {
    id: root
    readonly property bool linkOk: vehicle.companion.values.linkAlive === true
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
        switch (st.state) {
        case "sending":     return "Sending " + st.label + "… (try " + st.attempt + "/" + st.retries + ")"
        case "in_progress": return st.label + ": Jetson in progress…"
        case "accepted":    return "✓ " + st.label + ": accepted by Jetson"
        case "rejected":    return "✕ " + st.label + ": temporarily rejected"
        case "denied":      return "✕ " + st.label + ": denied by Jetson"
        case "unsupported": return "✕ " + st.label + ": not supported by Jetson"
        case "failed":      return "✕ " + st.label + ": failed"
        case "timeout":     return "⚠ " + st.label + ": Jetson not responding"
        case "no_link":     return "⚠ Link not open: " + commands.targetLink + " (tab Links)"
        default:            return ""
        }
    }
    function stateColor() {
        if (st.state === "accepted") return Theme.ok
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
                    text: root.confirming === "takeoff" ? "Take off? (" + flightEnv.state.effectiveLabel + ")"
                        : root.confirming === "land" ? "Land at current position?"
                        : root.confirming === "rtl" ? "Return to launch and land?"
                        : "Send " + root.wpCount + " waypoints to Jetson?"
                    color: Theme.text; font.pixelSize: 15; font.bold: true
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
                        onClicked: { commands.uploadMission(); root.confirming = "" }
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
                        // Take-off bi khoa khi pre-flight co van de nghiem trong; Land luon cho phep
                        readonly property bool blocked: root.confirming === "takeoff" && alerts.critical > 0
                        enabled: !blocked
                        opacity: blocked ? 0.4 : 1
                        implicitWidth: 200; implicitHeight: 40; radius: 6
                        color: Theme.panelAlt; border.color: Theme.border; clip: true
                        property real progress: 0
                        Rectangle {
                            width: parent.width * parent.progress; height: parent.height
                            color: root.confirming === "takeoff" ? Theme.ok : Theme.warn
                        }
                        Text {
                            anchors.centerIn: parent
                            text: holdBtn.blocked ? "Locked — fix critical issues" : "Hold to confirm"
                            color: Theme.text; font.pixelSize: 13; font.bold: true
                        }
                        NumberAnimation on progress {
                            id: holdAnim
                            running: false; from: 0; to: 1; duration: 1000
                            onFinished: {
                                if (holdBtn.progress < 1) return
                                if (root.confirming === "takeoff") commands.takeoff(altBox.meters)
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
                enabled: root.linkOk
                active: root.confirming === "takeoff"
                onClicked: root.confirming = root.confirming === "takeoff" ? "" : "takeoff"
            }
            CmdBtn {
                objectName: "uploadBtn"
                label: root.up.busy ? "■  Cancel upload" : "⇪  Send WP (" + root.wpCount + ")"
                tint: Theme.accent
                enabled: root.linkOk && (root.wpCount > 0 || root.up.busy)
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
                enabled: root.linkOk
                onClicked: { root.confirming = ""; commands.hold() }
            }
            CmdBtn {
                objectName: "rtlBtn"
                visible: root.inAir
                readonly property bool needsGps: flightEnv.state.effective !== "outdoor"
                label: needsGps ? "⌂  RTL (GPS)" : "⌂  RTL"; tint: Theme.warn
                enabled: root.linkOk && !needsGps
                active: root.confirming === "rtl"
                onClicked: root.confirming = root.confirming === "rtl" ? "" : "rtl"
            }
            CmdBtn {
                objectName: "landBtn"
                label: "⬇  Land"; tint: Theme.warn
                enabled: root.linkOk
                active: root.confirming === "land"
                onClicked: root.confirming = root.confirming === "land" ? "" : "land"
            }
        }

        // ---- trang thai lenh ----
        Rectangle {
            readonly property string msg: !root.linkOk ? "Jetson not connected (radio 433) — buttons locked"
                                                       : [root.stateText(), root.uploadText()].filter(t => t !== "").join("   ·   ")
            visible: msg !== ""
            Layout.alignment: Qt.AlignHCenter
            implicitWidth: stText.implicitWidth + 20; implicitHeight: 26; radius: 6
            color: "#cc161b22"
            Text {
                id: stText
                anchors.centerIn: parent
                text: parent.msg
                color: !root.linkOk ? Theme.textDim
                     : (root.st.state === "idle" && (root.up.state === "done" || root.up.state === "sending")) ? Theme.ok
                     : root.stateColor()
                font.pixelSize: 12
            }
        }
    }
}
