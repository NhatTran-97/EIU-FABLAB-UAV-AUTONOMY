import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "../components"

ScrollView {
    id: page
    clip: true
    contentWidth: availableWidth

    readonly property var st: vehicle.status.values
    readonly property var gps: vehicle.gps.values
    readonly property var bat: vehicle.battery.values
    readonly property var fl: vehicle.flight.values
    readonly property var comp: vehicle.companion.values
    readonly property var px4Link: links.values.px4 || ({})
    readonly property var jetLink: links.values.jetson || ({})

    component SectionTitle: Text {
        color: Theme.text; font.pixelSize: 16; font.bold: true
        Layout.topMargin: 8
    }

    function sensorText(s) {
        return ({ ok: "Healthy", error: "Error", off: "Off", none: "Not present", unknown: "—" })[s] || s
    }
    function levelOf(s) { return s === "ok" ? "ok" : (s === "error" ? "error" : "stale") }
    function has(x) { return x !== null && x !== undefined }
    function num(x, digits, unit) { return has(x) && st.linkAlive ? x.toFixed(digits) + " " + unit : "—" }
    // bo dem phat con trong cua radio: >= 50 % ok, 30-50 % sap nghen, < 30 % nghen
    function bufLevel(b) { return !has(b) ? "stale" : (b >= 50 ? "ok" : (b >= 30 ? "warn" : "error")) }
    function bufText(b) { return has(b) ? "  ·  uplink buffer " + b + "%" : "" }
    function rssi(l) { return has(l.rssiDbm) ? l.rssiDbm + " / " + l.remoteRssiDbm + " dBm" : "—" }
    function duration(sec) {
        var m = Math.floor(sec / 60), s = sec % 60
        return (m < 10 ? "0" : "") + m + ":" + (s < 10 ? "0" : "") + s
    }

    ColumnLayout {
        width: page.availableWidth
        spacing: Theme.gap

        // ---------------- System Health (problem-first: van de len truoc) ----------------
        Rectangle {
            objectName: "systemHealth"
            Layout.fillWidth: true
            implicitHeight: healthCol.implicitHeight + 24
            radius: Theme.radius; color: Theme.panel
            border.color: alerts.critical > 0 ? Theme.error : (alerts.warning > 0 ? Theme.warn : Theme.ok)
            ColumnLayout {
                id: healthCol
                anchors.fill: parent; anchors.margins: 12
                spacing: 6
                RowLayout {
                    spacing: 18
                    Text { text: "System Health"; color: Theme.text; font.pixelSize: 16; font.bold: true }
                    Text { text: "⛔ " + alerts.critical + " critical"; color: alerts.critical > 0 ? Theme.error : Theme.textDim; font.pixelSize: 14; font.bold: alerts.critical > 0 }
                    Text { text: "⚠ " + alerts.warning + (alerts.warning === 1 ? " warning" : " warnings"); color: alerts.warning > 0 ? Theme.warn : Theme.textDim; font.pixelSize: 14; font.bold: alerts.warning > 0 }
                    Text { visible: alerts.items.length === 0; text: "✓ All systems normal"; color: Theme.ok; font.pixelSize: 14; font.bold: true }
                }
                Repeater {
                    model: alerts.items
                    delegate: Text {
                        required property var modelData
                        Layout.fillWidth: true; wrapMode: Text.WordWrap
                        text: (modelData.level === "critical" ? "⛔ " : "⚠ ") + modelData.text
                        color: modelData.level === "critical" ? Theme.error : Theme.warn
                        font.pixelSize: 14
                    }
                }
            }
        }

        // ---------------- PX4 ----------------
        SectionTitle { text: "PX4  ·  radio 915 MHz" }
        Flow {
            Layout.fillWidth: true
            spacing: Theme.gap

            StatusCard {
                title: "Flight Controller"
                status: st.linkAlive ? "ok" : (st.heartbeatAge >= 0 ? "error" : "stale")
                value: st.linkAlive ? st.mode : "Disconnected"
                detail: st.heartbeatAge >= 0 ? "Heartbeat " + st.heartbeatAge + " s ago" : "No heartbeat yet"
            }
            StatusCard {
                title: "GPS"
                status: !st.linkAlive ? "stale" : (gps.fixType >= 3 ? "ok"
                        : (flightEnv.state.effective === "indoor" ? "none" : (gps.fixType === 2 ? "warn" : "error")))
                readonly property bool notRequired: flightEnv.state.effective === "indoor" && gps.fixType < 3
                value: notRequired ? "Not required" : gps.fix
                detail: notRequired ? "Indoor mode · " + gps.fix : gps.satellites + " satellites"
            }
            StatusCard {
                title: "Battery"
                status: !st.linkAlive ? "stale" : bat.percent < 0 ? "warn"
                        : (bat.percent > 30 ? "ok" : (bat.percent > 15 ? "warn" : "error"))
                value: bat.percent >= 0 ? bat.percent + " %" : (page.has(bat.voltage) ? bat.voltage.toFixed(2) + " V" : "—")
                detail: (bat.percent >= 0 && page.has(bat.voltage) ? bat.voltage.toFixed(2) + " V" : "PX4 gives no %")
                        + (page.has(bat.current) ? "  ·  " + bat.current.toFixed(1) + " A" : "")
            }
            StatusCard {
                title: "Telemetry 915"
                status: !page.has(px4Link.rssiDbm) ? "stale" : page.bufLevel(px4Link.txbuf)
                value: page.rssi(px4Link)
                detail: "RSSI ground / drone" + page.bufText(px4Link.txbuf)
            }
            Repeater {
                model: vehicle.sensors.items.length
                delegate: StatusCard {
                    required property int index
                    readonly property var s: vehicle.sensors.items[index] || ({})
                    title: s.name || ""
                    status: st.linkAlive ? page.levelOf(s.status) : "stale"
                    value: page.sensorText(s.status)
                }
            }
        }

        // ---------------- Jetson ----------------
        SectionTitle { text: "Jetson  ·  radio 433 MHz" }
        Flow {
            Layout.fillWidth: true
            spacing: Theme.gap

            StatusCard {
                title: "Jetson companion"
                status: comp.linkAlive ? "ok" : (comp.heartbeatAge >= 0 ? "error" : "stale")
                value: comp.linkAlive ? "Connected" : "Disconnected"
                detail: comp.heartbeatAge >= 0 ? "Heartbeat " + comp.heartbeatAge + " s ago" : "No heartbeat yet"
            }
            StatusCard {
                title: "Telemetry 433"
                status: !page.has(jetLink.rssiDbm) ? "stale" : page.bufLevel(jetLink.txbuf)
                value: page.rssi(jetLink)
                detail: "RSSI ground / drone" + page.bufText(jetLink.txbuf)
            }
            Repeater {
                model: vehicle.companion.items.length
                delegate: StatusCard {
                    required property int index
                    readonly property var c: vehicle.companion.items[index] || ({})
                    title: c.name || ""
                    status: c.level || "stale"
                    value: c.state || ""
                    detail: page.has(c.rateHz) ? (c.unit === "B/s" ? Math.round(c.rateHz) + " B/s down"
                                                                   : c.rateHz.toFixed(1) + " Hz") : ""
                }
            }
        }

        // ---------------- Messages ----------------
        SectionTitle { text: "PX4 messages" }
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: msgList.count === 0 ? 44 : Math.min(msgList.contentHeight + 16, 260)   // rong khi trong
            radius: Theme.radius; color: Theme.panel; border.color: Theme.border

            Text {
                anchors.centerIn: parent
                visible: msgList.count === 0
                text: "No messages"; color: Theme.textDim
            }
            ListView {
                id: msgList
                anchors.fill: parent; anchors.margins: 8
                clip: true
                model: vehicle.messages.items
                delegate: Text {
                    required property var modelData
                    width: msgList.width
                    text: modelData.time + "  [" + modelData.severity + "]  " + modelData.text
                    color: modelData.level <= 3 ? Theme.error : (modelData.level === 4 ? Theme.warn : Theme.text)
                    font.pixelSize: 13; font.family: "monospace"
                    wrapMode: Text.WordWrap
                }
            }
        }
        Item { Layout.preferredHeight: 8 }
    }
}
