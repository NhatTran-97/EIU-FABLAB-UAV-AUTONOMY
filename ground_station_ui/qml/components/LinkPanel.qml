import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// One link in the Links tab. `info` = one entry of links.items (Python LinkManager).
Rectangle {
    id: panel
    property var info: ({})
    readonly property bool isSerial: info.kind === "serial"

    implicitHeight: col.implicitHeight + 32
    radius: Theme.radius
    color: Theme.panel
    border.color: Theme.border

    ColumnLayout {
        id: col
        anchors.fill: parent
        anchors.margins: 16
        spacing: 10

        RowLayout {
            Layout.fillWidth: true
            Rectangle {
                width: 12; height: 12; radius: 6
                color: panel.info.alive ? Theme.ok : (panel.info.isOpen ? Theme.warn : Theme.idle)
            }
            Text { text: panel.info.title; color: Theme.text; font.pixelSize: 16; font.bold: true }
            Text {                                      // narrow card -> description elided, name and numbers kept
                text: panel.info.hint; color: Theme.textDim; font.pixelSize: 12
                Layout.fillWidth: true; Layout.minimumWidth: 0; elide: Text.ElideRight
            }
            Text {
                visible: panel.info.isOpen
                text: panel.info.msgRate + " msg/s"
                      + (panel.info.rssiDbm !== null && panel.info.rssiDbm !== undefined
                         ? "  ·  RSSI " + panel.info.rssiDbm + " / " + panel.info.remoteRssiDbm + " dBm" : "")
                color: Theme.textDim; font.pixelSize: 12
            }
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            ComboBox {
                id: portBox
                visible: panel.isSerial
                Layout.fillWidth: true                  // stretches with the card; long port names are elided
                Layout.minimumWidth: 140
                Layout.maximumWidth: 420
                model: ports.ports
                textRole: "label"
                valueRole: "device"
                enabled: !panel.info.active
                // connected: show the port in use (it may no longer be listed)
                displayText: panel.info.active ? panel.info.port : currentText
                Component.onCompleted: currentIndex = Math.max(0, indexOfValue(panel.info.port))
            }
            ComboBox {
                id: baudBox
                visible: panel.isSerial
                Layout.preferredWidth: 110
                model: [57600, 115200, 230400, 921600]
                enabled: !panel.info.active
                Component.onCompleted: currentIndex = Math.max(0, model.indexOf(panel.info.baud))
            }
            Button {
                visible: panel.isSerial
                text: "↻"
                enabled: !panel.info.active
                onClicked: ports.refresh()
            }
            Button {
                text: panel.info.active ? "Disconnect" : "Connect"
                enabled: panel.info.active || !panel.isSerial || portBox.currentValue !== undefined
                onClicked: {
                    if (panel.info.active && vehicle.status.values.armed) { confirmDisc.open(); return }
                    if (panel.info.active) links.disconnectLink(panel.info.name)
                    else links.connectLink(panel.info.name,
                                               panel.isSerial ? portBox.currentValue : "",
                                               panel.isSerial ? baudBox.currentValue : 0)
                }
                // in air: disconnecting needs confirmation (PX4 loses the GCS heartbeat -> failsafe)
                Dialog {
                    id: confirmDisc
                    modal: true; anchors.centerIn: Overlay.overlay
                    title: "Drone is ARMED"
                    standardButtons: Dialog.Cancel | Dialog.Yes
                    Text {
                        text: "Disconnect " + panel.info.title + " while the drone is armed?\n"
                            + "Losing the GCS link can trigger the PX4 data-link failsafe."
                        color: Theme.text; font.pixelSize: 13
                    }
                    onAccepted: links.disconnectLink(panel.info.name)
                }
            }
        }

        Text {
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            text: panel.info.status
                  + (panel.info.isOpen && !panel.info.alive ? "  ·  no heartbeat from drone" : "")
            color: Theme.textDim; font.pixelSize: 12
        }
    }
}
