import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Take-off / Land / Hold / RTL / Send WP via the Jetson (radio 433 -> DDS -> PX4).
// Locked by commands.gates.jetson (see core/commands/flight.py). Confirm by holding the button 1 s.
// Direct PX4 commands (915): DirectCommandPopup (TopBar).
Item {
    id: root
    readonly property var g: commands.gates.jetson || ({})      // action -> lock reason ('' = enabled)
    // test mode: Take-off / Land / Send WP skip GCS-side checks (DDS, GPS, cross-route lock).
    // Not persisted across app restarts.
    property bool testMode: false
    readonly property var testable: ["takeoff", "land", "upload"]
    function open(a) { return (testMode && testable.indexOf(a) >= 0) || g[a] === "" }
    readonly property var st: commands.values
    property string confirming: ""          // "" | "takeoff" | "land" | "rtl" | "upload"
    // in air (PX4 EXTENDED_SYS_STATE): IN AIR / TAKING OFF / LANDING
    readonly property bool inAir: ["IN AIR", "TAKING OFF", "LANDING"].indexOf(vehicle.status.values.landed) >= 0
    readonly property var up: commands.missionUpload.values
    readonly property int wpCount: mission.items.length
    // mission progress from the Jetson; 'paused' = waiting for the operator (manual task or pilot took over)
    readonly property var prog: vehicle.missionProgress.values
    readonly property bool missionPaused: prog.valid === true && prog.state === "paused"
    function taskSummary() {
        var auto = 0, manual = 0
        for (var i = 0; i < mission.items.length; i++) {
            if (mission.items[i].task === "pick_auto") auto++
            else if (mission.items[i].task === "pick_manual") manual++
        }
        if (auto + manual === 0) return "No tasks: the drone flies through every waypoint."
        return "Pick tasks: " + (auto ? auto + " automatic" : "") + (auto && manual ? ", " : "")
            + (manual ? manual + " manual (the pilot hooks the load, then Continue)" : "") + "."
    }
    // waypoints inside prohibited / restricted zones (evaluated when the upload dialog opens)
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
        case "rejected":    return "✕ " + st.label + ": temporarily rejected" + step + (st.message ? " — " + st.message : "")
        case "denied":      return "✕ " + st.label + ": denied by Jetson" + (st.message ? " — " + st.message : "")
        // Link test + UNSUPPORTED: drone_telemetry has no ping handler, but the packet reached the Jetson
        case "unsupported": return st.label === "Link test" ? "✓ Link test: reached Jetson" + took
                                                              + " (old drone_telemetry — rebuild for ping)"
                                                            : "✕ " + st.label + ": not supported"
        case "failed":      return "✕ " + st.label + ": failed" + step + (st.message ? " — " + st.message : "")
        case "timeout":     return "⚠ " + st.label + ": Jetson not responding" + took + " — check radio 433"
        case "no_link":     return "⚠ Link not open: " + commands.targetLink + " (tab Connections)"
        case "blocked":     return "✕ " + st.label + ": " + st.message
        default:            return ""
        }
    }
    // lock reasons of the visible buttons (deduplicated) -> status line when idle
    function lockText() {
        var acts = inAir ? ["upload", "mission", "hold", "rtl", "land"] : ["takeoff", "upload", "land"]
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

        // ---- confirm dialog ----
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
                        : root.confirming === "mission" ? "Fly the waypoints sent to Jetson?"
                        : "Send " + root.wpCount + " waypoints to Jetson?")
                    color: Theme.text; font.pixelSize: 15; font.bold: true
                }
                Text {
                    visible: root.confirming === "takeoff" || root.confirming === "land" || root.confirming === "rtl" || root.confirming === "mission"
                    Layout.alignment: Qt.AlignHCenter
                    text: "via Jetson · 433  (Jetson → DDS → PX4)"
                    color: Theme.textDim; font.pixelSize: 12
                }
                Text {
                    visible: root.confirming === "mission"
                    Layout.alignment: Qt.AlignHCenter; Layout.maximumWidth: 420; wrapMode: Text.WordWrap
                    text: "Uses the waypoints last sent with Send WP" + (root.up.state === "done" ? " ✓" : " (send them first)")
                        + ". At the end the drone hovers at the last waypoint."
                    color: root.up.state === "done" ? Theme.textDim : Theme.warn; font.pixelSize: 12
                }
                Text {
                    visible: root.confirming === "mission"
                    Layout.alignment: Qt.AlignHCenter
                    text: "Speed " + (mission.values.speed || 2).toFixed(1) + " m/s · tolerance XY "
                        + (mission.values.xyTolerance || 0.5).toFixed(2) + " / Z " + (mission.values.zTolerance || 0.5).toFixed(2)
                        + " m (waypoint list)"
                    color: Theme.text; font.pixelSize: 12
                }
                Text {
                    visible: root.confirming === "mission"
                    Layout.alignment: Qt.AlignHCenter; Layout.maximumWidth: 420; wrapMode: Text.WordWrap
                    text: root.taskSummary()
                    color: Theme.textDim; font.pixelSize: 12
                }
                // mission options, fixed for the whole mission; the Jetson refuses options it cannot run
                CheckBox {
                    id: avoidBox
                    visible: root.confirming === "mission"
                    Layout.alignment: Qt.AlignHCenter
                    text: "Obstacle avoidance"
                    checked: false
                    ToolTip.visible: hovered
                    ToolTip.text: "Every leg uses the obstacle_avoidance node. Refused if the node is not running or not allowed on the drone."
                }
                RowLayout {
                    visible: root.confirming === "takeoff"
                    Layout.alignment: Qt.AlignHCenter
                    Text { text: "Altitude"; color: Theme.textDim; font.pixelSize: 12 }
                    // stored in dm (SpinBox only takes integers) -> 0.5 m steps, one decimal shown
                    SpinBox {
                        id: altBox
                        readonly property real meters: value / 10
                        from: 5; to: 1200; stepSize: 5; editable: true
                        // default per environment (Settings -> Flight environment)
                        value: Math.round((commands.takeoffAltitudes[flightEnv.state.effective] || 1.5) * 10)
                        textFromValue: (v, locale) => (v / 10).toFixed(1)
                        valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 10)
                        validator: DoubleValidator { bottom: 0.5; top: 120; decimals: 1 }
                    }
                    Text { text: "m"; color: Theme.textDim }
                }
                // ---- pre-flight check (Take-off only): critical problems lock the button ----
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
                        implicitWidth: 110; implicitHeight: 36
                        // filled primary button: blue (red when waypoints are in a no-fly zone), darker while pressed
                        readonly property color base: conflictText.visible ? Theme.error : Theme.accent
                        background: Rectangle {
                            radius: 6
                            color: parent.down ? Qt.darker(parent.base, 1.4)
                                 : parent.hovered ? Qt.lighter(parent.base, 1.15) : parent.base
                        }
                        contentItem: Text {
                            text: parent.text; color: "white"; font.pixelSize: 14; font.bold: true
                            horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                        }
                        onClicked: {
                            if (root.testMode) commands.testUploadMission(); else commands.uploadMission()
                            root.confirming = ""
                        }
                    }
                }
                RowLayout {
                    visible: root.confirming === "takeoff" || root.confirming === "land" || root.confirming === "rtl" || root.confirming === "mission"
                    spacing: 10
                    Button { text: "Cancel"; onClicked: root.confirming = "" }
                    // hold 1 s to confirm
                    Rectangle {
                        id: holdBtn
                        objectName: "holdConfirm"
                        // locked: gates changed while the dialog is open, or a critical pre-flight problem (Take-off)
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
                                else if (root.confirming === "mission") commands.startMission(avoidBox.checked)
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

        // ---- test panel: send to the Jetson, skipping GCS-side checks ----
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
                // latest STATUSTEXT from the Jetson / PX4
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

        // ---- mission progress; Continue after a manual task or a pause ----
        Rectangle {
            objectName: "missionProgress"
            visible: root.prog.valid === true && (root.prog.state === "active" || root.prog.state === "paused")
            Layout.alignment: Qt.AlignHCenter
            implicitWidth: progRow.implicitWidth + 24; implicitHeight: progRow.implicitHeight + 16
            radius: Theme.radius
            color: "#f2161b22"
            border.color: root.missionPaused ? Theme.warn : Theme.border; border.width: root.missionPaused ? 2 : 1
            RowLayout {
                id: progRow
                anchors.centerIn: parent
                spacing: 12
                ColumnLayout {
                    spacing: 2
                    // status only (not a button): no play symbol, the current step, and a progress bar
                    RowLayout {
                        spacing: 6
                        Rectangle {
                            width: 8; height: 8; radius: 4
                            color: root.missionPaused ? Theme.warn : Theme.ok
                        }
                        Text {
                            text: (root.missionPaused ? "Mission paused" : "Mission running") + " · waypoint "
                                + root.prog.seq + "/" + root.prog.total
                                + (root.prog.step && !root.missionPaused ? " · " + root.prog.step : "")
                            color: root.missionPaused ? Theme.warn : Theme.text; font.pixelSize: 13; font.bold: true
                        }
                    }
                    Rectangle {
                        Layout.fillWidth: true; height: 3; radius: 1.5; color: Theme.border
                        Rectangle {
                            height: parent.height; radius: 1.5
                            width: parent.width * (root.prog.total > 0 ? Math.max(0, root.prog.seq - 1) / root.prog.total : 0)
                            color: root.missionPaused ? Theme.warn : Theme.ok
                        }
                    }
                    Text {
                        visible: root.missionPaused && root.prog.text !== ""
                        Layout.maximumWidth: 360; wrapMode: Text.WordWrap
                        text: root.prog.text
                        color: Theme.text; font.pixelSize: 12
                    }
                }
                Button {
                    objectName: "continueBtn"
                    visible: root.missionPaused
                    text: "▶  Continue"
                    enabled: root.open("continue")
                    implicitHeight: 36
                    // the only clickable item here: filled button, so it cannot be mistaken for the status text
                    background: Rectangle {
                        radius: 6; opacity: parent.enabled ? 1 : 0.4
                        color: parent.down ? Qt.darker(Theme.ok, 1.4) : parent.hovered ? Qt.lighter(Theme.ok, 1.15) : Theme.ok
                    }
                    contentItem: Text {
                        text: parent.text; color: "white"; font.pixelSize: 14; font.bold: true
                        horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                        leftPadding: 10; rightPadding: 10
                    }
                    ToolTip.visible: hovered
                    ToolTip.text: root.g["continue"] ? root.g["continue"]
                        : "Manual task done (or resume after a pause): the drone takes back OFFBOARD and goes on"
                    onClicked: commands.continueMission()
                }
            }
        }

        // ---- two main buttons ----
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
                // in air: Take-off hidden
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
            // in air: Hold (immediate, no confirmation) + RTL (needs GPS -> locked indoors)
            CmdBtn {
                objectName: "holdModeBtn"
                visible: root.inAir
                label: "⏸  Hold"; tint: Theme.accent
                enabled: root.open("hold")
                onClicked: { root.confirming = ""; commands.hold() }
            }
            CmdBtn {
                objectName: "missionBtn"
                visible: root.inAir
                label: "▶  Start mission"; tint: Theme.ok
                implicitWidth: 170
                enabled: root.open("mission")
                active: root.confirming === "mission"
                onClicked: root.confirming = root.confirming === "mission" ? "" : "mission"
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
                // always enabled: ping the Jetson
                label: root.testMode ? "🧪  Test ON" : "🧪  Test"; tint: root.testMode ? Theme.warn : Theme.textDim
                implicitWidth: root.testMode ? 120 : 96
                active: root.testMode
                onClicked: { root.testMode = !root.testMode; root.confirming = "" }
            }
        }

        // ---- command status ----
        Rectangle {
            readonly property string busyMsg: [root.stateText(), root.uploadText()].filter(t => t !== "").join("   ·   ")
            // when idle: lock reasons / test-mode warning
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
