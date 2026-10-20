import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Popup tu o PX4 (TopBar): Take-off / Land thang PX4 qua radio 915 (khong qua Jetson).
// Khoa theo commands.gates.px4 (link 915, khoa cheo, trang thai bay). Xac nhan: giu nut 1 s.
Popup {
    id: pop
    width: 400
    padding: 14
    readonly property var g: commands.gates.px4 || ({})
    readonly property var st: commands.directCommand.values
    readonly property bool inAir: ["IN AIR", "TAKING OFF", "LANDING"].indexOf(vehicle.status.values.landed) >= 0
    readonly property bool onGround: vehicle.status.values.landed === "ON GROUND"

    function stateText() {
        var step = st.steps > 1 ? " (step " + st.step + "/" + st.steps + ")" : ""
        var took = st.elapsed >= 0 ? " (" + st.elapsed.toFixed(1) + " s)" : ""
        switch (st.state) {
        case "sending":     return "Sending " + st.label + step + "… try " + st.attempt + "/" + st.retries
        case "in_progress": return st.label + ": PX4 in progress…"
        case "accepted":    return "✓ " + st.label + ": accepted by PX4" + took
        case "rejected":    return "✕ " + st.label + ": temporarily rejected by PX4" + step
        case "denied":      return "✕ " + st.label + ": denied by PX4" + step + " — see PX4 messages"
        case "unsupported": return "✕ " + st.label + ": not supported by PX4"
        case "failed":      return "✕ " + st.label + ": failed" + step
        case "timeout":     return "⚠ " + st.label + ": PX4 not responding" + step
        case "no_link":     return "⚠ " + st.label + ": PX4 link not open (tab Connections)"
        case "blocked":     return "✕ " + st.label + ": " + st.message
        default:            return ""
        }
    }

    background: Rectangle { color: Theme.panel; border.color: Theme.accent; radius: Theme.radius }

    contentItem: ColumnLayout {
        spacing: 10

        ColumnLayout {
            spacing: 2
            Text { text: "Direct to PX4  ·  radio 915"; color: Theme.text; font.pixelSize: 15; font.bold: true }
            Text {
                Layout.fillWidth: true; wrapMode: Text.WordWrap
                text: "MAVLink straight to the flight controller — bypasses Jetson. "
                    + "Buttons under the map send via Jetson; whichever route starts a flight keeps control until landed + disarmed."
                color: Theme.textDim; font.pixelSize: 11
            }
        }

        // ---- Take-off ----
        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border }
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            Text { text: "Take-off"; color: Theme.text; font.pixelSize: 13; font.bold: true; Layout.fillWidth: true }
            Text { text: "Altitude"; color: Theme.textDim; font.pixelSize: 12 }
            // don vi noi bo = dm (SpinBox chi nhan so nguyen) -> buoc 0.5 m
            SpinBox {
                id: altBox
                readonly property real meters: value / 10
                Layout.preferredWidth: 120
                from: 5; to: 1200; stepSize: 5; editable: true
                value: Math.round((commands.takeoffAltitudes[flightEnv.state.effective] || 1.5) * 10)
                textFromValue: (v, locale) => (v / 10).toFixed(1)
                valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 10)
                validator: DoubleValidator { bottom: 0.5; top: 120; decimals: 1 }
            }
            Text { text: "m"; color: Theme.textDim }
        }
        Text {
            visible: alerts.critical > 0 && !pop.inAir
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: "⛔ Pre-flight: " + alerts.critical + " critical issue(s) — see ⚠ Alerts"
            color: Theme.error; font.pixelSize: 12
        }
        HoldButton {
            objectName: "directTakeoff"
            Layout.fillWidth: true
            text: "⬆  Hold to take off (" + flightEnv.state.effectiveLabel + ")"
            tint: Theme.ok
            lockedText: pop.g.takeoff ? pop.g.takeoff
                      : pop.inAir ? "already flying"
                      : alerts.critical > 0 ? "fix critical issues" : ""
            onConfirmed: commands.directTakeoff(altBox.meters)
        }

        // ---- Land ----
        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border }
        HoldButton {
            objectName: "directLand"
            Layout.fillWidth: true
            text: "⬇  Hold to land here"
            tint: Theme.warn
            lockedText: pop.g.land ? pop.g.land : (pop.onGround ? "vehicle is on the ground" : "")
            onConfirmed: commands.directLand()
        }

        // ---- trang thai lenh thang PX4 ----
        Text {
            visible: text !== ""
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: pop.stateText()
            color: pop.st.state === "accepted" ? Theme.ok
                 : (pop.st.state === "sending" || pop.st.state === "in_progress") ? Theme.text : Theme.error
            font.pixelSize: 12
        }
        Text {
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: "RC transmitter always overrides. Kill switch stays active."
            color: Theme.textDim; font.pixelSize: 11
        }
    }
}
