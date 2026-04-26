import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

import "components"

ApplicationWindow {
    id: root
    visible: true
    width:  1200
    height: 720
    minimumWidth:  900
    minimumHeight: 600
    title: "Gimbal Ground Station"
    color: "#181825"

    // ── Header bar ─────────────────────────────────────────────
    header: Rectangle {
        height: 48; color: "#11111b"

        RowLayout {
            anchors { fill: parent; leftMargin: 16; rightMargin: 16 }

            Label {
                text: "Gimbal Ground Station"
                color: "#cdd6f4"; font { pixelSize: 17; bold: true }
            }

            Item { Layout.fillWidth: true }

            // UART status chip
            Rectangle {
                height: 24; radius: 12
                color: serialBridge.connected ? "#1a3a2a" : "#2a1a1a"
                border.color: serialBridge.connected ? "#50fa7b" : "#ff5555"
                border.width: 1
                width: statusChipLabel.implicitWidth + 34
                Layout.rightMargin: 50

                Label {
                    id: statusChipLabel
                    anchors.centerIn: parent
                    text: serialBridge.connected
                          ? "● LoRa  " + serialBridge.portName
                          : "○ Chưa kết nối"
                    color: serialBridge.connected ? "#50fa7b" : "#ff5555"
                    font.pixelSize: 11
                }
            }

            Label {
                id: clock
                color: "#6272a4"; font.pixelSize: 12
                Timer { interval: 1000; repeat: true; running: true
                    onTriggered: clock.text = new Date().toLocaleTimeString(Qt.locale(), "hh:mm:ss") }
            }
        }
    }

    // ── Main layout ────────────────────────────────────────────
    RowLayout {
        anchors { fill: parent; margins: 10 }
        spacing: 10

        // Cột trái: UART / LoRa
        SerialPanel {
            Layout.preferredWidth: 270
            Layout.fillHeight: true
        }

        // Cột phải: Gimbal Control
        GimbalPanel {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

    // ── Error toast ────────────────────────────────────────────
    Rectangle {
        id: toast
        anchors { bottom: parent.bottom; horizontalCenter: parent.horizontalCenter; bottomMargin: 20 }
        width: toastTxt.implicitWidth + 32; height: 34; radius: 17
        color: "#ff5555"; opacity: 0; z: 100

        Label { id: toastTxt; anchors.centerIn: parent; color: "#fff"; font.pixelSize: 12 }
        Behavior on opacity { NumberAnimation { duration: 180 } }
        Timer { id: toastTimer; interval: 2500; onTriggered: toast.opacity = 0 }

        Connections {
            target: serialBridge
            function onErrorOccurred(err) {
                toastTxt.text = "⚠  " + err
                toast.opacity = 0.95
                toastTimer.restart()
            }
        }
    }
}
