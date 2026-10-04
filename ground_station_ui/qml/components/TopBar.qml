import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

Rectangle {
    id: bar
    height: 64
    color: Theme.panel

    readonly property var st: vehicle.status.values
    readonly property var gps: vehicle.gps.values
    readonly property var bat: vehicle.battery.values
    readonly property var fl: vehicle.flight.values
    readonly property var px4Link: links.values.px4 || ({})
    readonly property var jetLink: links.values.jetson || ({})
    readonly property bool jetsonAlive: vehicle.companion.values.linkAlive

    component Item_: ColumnLayout {
        property string label: ""
        property string value: ""
        property color valueColor: Theme.text
        spacing: 2
        Text { text: parent.label; color: Theme.textDim; font.pixelSize: 12 }
        Text { text: parent.value; color: parent.valueColor; font.pixelSize: 15; font.bold: true }
    }

    function rssiText(dbm) { return dbm === null || dbm === undefined ? "—" : dbm + " dBm" }

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 20
        anchors.rightMargin: 20
        spacing: bar.width < 1400 ? 14 : 28            // cua so hep -> sat lai (logo nam o dau menu)


        // PX4: ket noi + CHE DO BAY (to) + ARMED / DISARMED (noi bat khi armed)
        Rectangle {
            Layout.preferredHeight: 48
            Layout.preferredWidth: px4Row.implicitWidth + 24
            radius: Theme.radius; color: Theme.panelAlt
            border.color: st.armed ? Theme.error : Theme.border; border.width: st.armed ? 2 : 1
            RowLayout {
                id: px4Row
                anchors.centerIn: parent; spacing: 12
                Rectangle { width: 12; height: 12; radius: 6; color: st.linkAlive ? Theme.ok : Theme.error }
                ColumnLayout {
                    spacing: 0
                    Text {
                        text: st.linkAlive ? "PX4" : "PX4 lost"
                        color: st.linkAlive ? Theme.textDim : Theme.error; font.pixelSize: 12; font.bold: true
                    }
                    Text {
                        objectName: "modeText"
                        text: st.linkAlive ? String(st.mode).toUpperCase() : "—"
                        color: Theme.text; font.pixelSize: 18; font.bold: true
                    }
                }
                Rectangle {
                    visible: st.linkAlive === true
                    objectName: "armBadge"
                    implicitWidth: armText.implicitWidth + 16; implicitHeight: 26; radius: 4
                    color: st.armed ? Theme.error : "transparent"
                    border.color: st.armed ? Theme.error : Theme.textDim
                    Text {
                        id: armText
                        anchors.centerIn: parent
                        text: st.armed ? "⚠ ARMED" : "DISARMED"
                        color: st.armed ? "white" : Theme.textDim; font.pixelSize: 13; font.bold: true
                    }
                }
            }
        }

        Item_ {
            label: "GPS"
            value: gps.fix + (gps.satellites > 0 ? " (" + gps.satellites + ")" : "")
            // trong nha khong co GPS la binh thuong -> xam, khong bao do
            valueColor: gps.fixType >= 3 ? Theme.ok
                      : (flightEnv.state.effective === "indoor" ? Theme.textDim
                      : (gps.fixType === 2 ? Theme.warn : Theme.error))
        }
        Item_ {
            // nguong 25 % canh bao, 15 % nguy cap (khop core/alerts/health.py) -- co bieu tuong, khong chi mau
            readonly property string lvl: bat.percent < 0 ? "" : (bat.percent <= 15 ? "crit" : (bat.percent <= 25 ? "warn" : ""))
            label: "Battery"
            value: (lvl === "crit" ? "⛔ " : lvl === "warn" ? "⚠ " : "")
                 + (bat.percent >= 0 ? bat.percent + "%  " + bat.voltage.toFixed(1) + " V"
                                     : (bat.voltage ? bat.voltage.toFixed(1) + " V" : "—"))
            valueColor: bat.percent < 0 ? Theme.text : (lvl === "crit" ? Theme.error : (lvl === "warn" ? Theme.warn : Theme.ok))
        }
        Item_ {
            label: fl.altSource === "local" ? "Alt (sensor)" : "Alt (Rel)"
            value: fl.altRel === null || fl.altRel === undefined ? "—" : fl.altRel.toFixed(1) + " m"
        }
        Item_ { label: "Telemetry 915"; value: bar.rssiText(px4Link.rssiDbm) }
        Item_ {
            label: "Jetson 433"
            value: jetsonAlive ? bar.rssiText(jetLink.rssiDbm) : "Disconnected"
            valueColor: jetsonAlive ? Theme.text : Theme.error
        }

        Item { Layout.fillWidth: true }

        // ---- Alert Center: so van de; bam -> danh sach ----
        Rectangle {
            objectName: "alertButton"
            Layout.preferredHeight: 40
            Layout.preferredWidth: alertRow.implicitWidth + 24
            radius: Theme.radius
            color: alerts.critical > 0 ? Theme.error : (alerts.warning > 0 ? "#3d2f12" : Theme.panelAlt)
            border.color: alerts.critical > 0 ? Theme.error : (alerts.warning > 0 ? Theme.warn : Theme.border)
            RowLayout {
                id: alertRow
                anchors.centerIn: parent; spacing: 6
                Text {
                    text: alerts.items.length === 0 ? "✓ System Healthy" : "⚠ " + alerts.items.length + (alerts.items.length === 1 ? " Alert" : " Alerts")
                    color: alerts.critical > 0 ? "white" : (alerts.warning > 0 ? Theme.warn : Theme.ok)
                    font.pixelSize: 15; font.bold: true
                }
            }
            MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: alertPopup.open() }

            Popup {
                id: alertPopup
                y: parent.height + 6
                x: parent.width - width
                width: 420
                padding: 12
                background: Rectangle { color: Theme.panel; border.color: Theme.border; radius: Theme.radius }
                contentItem: ColumnLayout {
                    spacing: 8
                    Text {
                        text: alerts.items.length === 0 ? "No issues" : "Alerts (" + alerts.critical + " critical, " + alerts.warning + " warnings)"
                        color: Theme.text; font.pixelSize: 14; font.bold: true
                    }
                    Repeater {
                        model: alerts.items
                        delegate: RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            spacing: 8
                            Text {
                                text: modelData.level === "critical" ? "⛔" : "⚠"
                                color: modelData.level === "critical" ? Theme.error : Theme.warn; font.pixelSize: 14
                            }
                            Text {
                                Layout.fillWidth: true; wrapMode: Text.WordWrap
                                text: modelData.text
                                color: Theme.text; font.pixelSize: 13
                            }
                        }
                    }
                }
            }
        }

        Text {
            id: clock
            color: Theme.text; font.pixelSize: 16
            Timer {
                interval: 1000; running: true; repeat: true; triggeredOnStart: true
                onTriggered: clock.text = Qt.formatDateTime(new Date(), "HH:mm:ss")
            }
        }
    }
}
