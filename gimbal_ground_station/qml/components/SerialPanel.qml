import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

// Panel kết nối UART/LoRa
Rectangle {
    id: root
    color: "#1e1e2e"
    radius: 8
    border.color: serialBridge.connected ? "#50fa7b" : "#6272a4"
    border.width: 1

    readonly property var baudRates: [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

    ColumnLayout {
        anchors { fill: parent; margins: 12 }
        spacing: 8

        // ── Header ──────────────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true

            Rectangle {
                width: 10; height: 10; radius: 5
                color: serialBridge.connected ? "#50fa7b" : "#ff5555"
                Behavior on color { ColorAnimation { duration: 200 } }
            }

            Label {
                text: "UART / LoRa"
                color: "#cdd6f4"
                font { pixelSize: 14; bold: true }
                Layout.fillWidth: true
                leftPadding: 6
            }

            ToolButton {
                text: "↻"
                font.pixelSize: 16
                ToolTip.text: "Làm mới danh sách cổng"
                ToolTip.visible: hovered
                onClicked: serialBridge.refreshPorts()
            }
        }

        // ── Chọn cổng COM ────────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            spacing: 6

            Label { text: "Cổng:"; color: "#a6adc8"; font.pixelSize: 12; Layout.preferredWidth: 50 }

            ComboBox {
                id: portCombo
                Layout.fillWidth: true
                model: serialBridge.availablePorts
                enabled: !serialBridge.connected
                onCurrentTextChanged: serialBridge.portName = currentText
                onModelChanged: {
                    if (count > 0) {
                        currentIndex = 0
                        serialBridge.portName = currentText
                    }
                }
                contentItem: Text {
                    leftPadding: 8
                    text: portCombo.displayText
                    color: "#cdd6f4"
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 12
                }
                background: Rectangle {
                    color: "#313244"; radius: 4
                    border.color: portCombo.activeFocus ? "#89b4fa" : "#45475a"
                }
            }
        }

        // ── Baud rate ─────────────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            spacing: 6

            Label { text: "Baud:"; color: "#a6adc8"; font.pixelSize: 12; Layout.preferredWidth: 50 }

            ComboBox {
                id: baudCombo
                Layout.fillWidth: true
                model: root.baudRates
                currentIndex: 4   // 115200 mặc định
                enabled: !serialBridge.connected
                onCurrentValueChanged: serialBridge.baudRate = currentValue
                contentItem: Text {
                    leftPadding: 8
                    text: baudCombo.displayText
                    color: "#cdd6f4"
                    verticalAlignment: Text.AlignVCenter
                    font.pixelSize: 12
                }
                background: Rectangle {
                    color: "#313244"; radius: 4
                    border.color: baudCombo.activeFocus ? "#89b4fa" : "#45475a"
                }
            }
        }

        // ── Nút kết nối / ngắt ───────────────────────────────────
        Button {
            Layout.fillWidth: true
            text: serialBridge.connected ? "Ngắt kết nối" : "Kết nối"
            onClicked: serialBridge.connected
                       ? serialBridge.disconnectPort()
                       : serialBridge.connectPort()

            background: Rectangle {
                radius: 4
                color: serialBridge.connected
                       ? (parent.pressed ? "#cc2244" : "#ff5555")
                       : (parent.pressed ? "#1a7a3a" : "#50fa7b")
                Behavior on color { ColorAnimation { duration: 150 } }
            }
            contentItem: Text {
                text: parent.text
                color: serialBridge.connected ? "#ffffff" : "#1e1e2e"
                font { pixelSize: 13; bold: true }
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        // ── Status message ────────────────────────────────────────
        Label {
            Layout.fillWidth: true
            text: serialBridge.statusMessage
            color: "#a6adc8"
            font.pixelSize: 11
            wrapMode: Text.WordWrap
            elide: Text.ElideRight
        }

        // ── Log nhận dữ liệu thô ─────────────────────────────────
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#11111b"
            radius: 4
            clip: true

            ScrollView {
                anchors.fill: parent
                contentWidth: availableWidth

                TextArea {
                    id: logArea
                    readOnly: true
                    color: "#a6e3a1"
                    font.family: "Monospace"
                    font.pixelSize: 11
                    background: null
                    wrapMode: TextArea.Wrap
                    placeholderText: "Dữ liệu nhận từ LoRa sẽ hiển thị ở đây..."
                    placeholderTextColor: "#45475a"

                    Connections {
                        target: serialBridge
                        function onDataReceived(data) {
                            var hex = ""
                            for (var i = 0; i < data.length; i++) {
                                var b = data[i] & 0xFF
                                hex += ("0" + b.toString(16)).slice(-2).toUpperCase() + " "
                            }
                            logArea.append("[RX] " + hex.trim())
                            // Giữ tối đa 200 dòng
                            var lines = logArea.text.split("\n")
                            if (lines.length > 200)
                                logArea.text = lines.slice(lines.length - 200).join("\n")
                        }
                        function onErrorOccurred(error) {
                            logArea.append("[ERR] " + error)
                        }
                    }
                }
            }
        }

        // ── Xóa log ───────────────────────────────────────────────
        Button {
            Layout.alignment: Qt.AlignRight
            text: "Xóa log"
            flat: true
            font.pixelSize: 11
            onClicked: logArea.clear()
            contentItem: Text {
                text: parent.text; color: "#6272a4"
                font: parent.font
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
            background: Rectangle { color: "transparent" }
        }
    }
}
