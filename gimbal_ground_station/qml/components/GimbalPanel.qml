import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    color: "#1e1e2e"
    radius: 8
    border.color: "#45475a"
    border.width: 1

    ColumnLayout {
        anchors { fill: parent; margins: 12 }
        spacing: 10

        // ══════════════════════════════════════════════════════
        // Header: gimbal link status + Manual/Auto toggle
        // ══════════════════════════════════════════════════════
        RowLayout {
            Layout.fillWidth: true

            Rectangle {
                width: 10; height: 10; radius: 5
                color: gimbalController.gimbalLinked ? "#50fa7b" : "#ff5555"
                Behavior on color { ColorAnimation { duration: 200 } }
            }

            Label {
                text: "Gimbal Control"
                color: "#cdd6f4"
                font.pixelSize: 14
                font.bold: true
                leftPadding: 6
                Layout.fillWidth: true
            }

            // Manual / Auto pill
            Rectangle {
                width: 120; height: 28; radius: 14
                color: "#313244"
                border.color: "#45475a"
                border.width: 1

                RowLayout {
                    anchors { fill: parent; margins: 2 }
                    spacing: 0

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        radius: 12
                        color: gimbalController.controlMode === 0 ? "#89b4fa" : "transparent"
                        Behavior on color { ColorAnimation { duration: 120 } }
                        MouseArea {
                            anchors.fill: parent
                            onClicked: gimbalController.controlMode = 0
                        }
                        Label {
                            anchors.centerIn: parent
                            text: "MANUAL"
                            font.pixelSize: 10
                            font.bold: true
                            color: gimbalController.controlMode === 0 ? "#1e1e2e" : "#6272a4"
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        radius: 12
                        color: gimbalController.controlMode === 1 ? "#f38ba8" : "transparent"
                        Behavior on color { ColorAnimation { duration: 120 } }
                        MouseArea {
                            anchors.fill: parent
                            onClicked: gimbalController.controlMode = 1
                        }
                        Label {
                            anchors.centerIn: parent
                            text: "AUTO"
                            font.pixelSize: 10
                            font.bold: true
                            color: gimbalController.controlMode === 1 ? "#1e1e2e" : "#6272a4"
                        }
                    }
                }
            }
        }

        // ══════════════════════════════════════════════════════
        // Mode tabs: PTZ / Position / Velocity
        // ══════════════════════════════════════════════════════
        Rectangle {
            Layout.fillWidth: true
            height: 36
            radius: 8
            color: "#11111b"
            border.color: "#313244"
            border.width: 1

            RowLayout {
                anchors { fill: parent; margins: 3 }
                spacing: 3

                Repeater {
                    model: ["PTZ", "Position", "Velocity"]
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        radius: 6
                        color: gimbalController.gimbalMode === index ? "#89b4fa" : "transparent"
                        Behavior on color { ColorAnimation { duration: 150 } }
                        MouseArea {
                            anchors.fill: parent
                            onClicked: gimbalController.gimbalMode = index
                        }
                        Label {
                            anchors.centerIn: parent
                            text: modelData
                            font.pixelSize: 12
                            font.bold: gimbalController.gimbalMode === index
                            color: gimbalController.gimbalMode === index ? "#1e1e2e" : "#6272a4"
                        }
                    }
                }
            }
        }

        // ══════════════════════════════════════════════════════
        // Main area: Joystick (left) + Controls (right)
        // ══════════════════════════════════════════════════════
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 14

            // ── Virtual joystick ──────────────────────────────
            Joystick {
                id: joystick
                Layout.preferredWidth:  190
                Layout.preferredHeight: 190
                Layout.alignment: Qt.AlignVCenter
                label: {
                    if (gimbalController.gimbalMode === 0) return "Kéo để điều hướng"
                    if (gimbalController.gimbalMode === 1) return "Kéo để đặt góc"
                    return "Kéo để đặt tốc độ"
                }
                onMoved: gimbalController.setJoystickInput(x, y)
                onJoystickReleased: {
                    if (gimbalController.gimbalMode === 0)
                        gimbalController.sendPtzCmd(0)
                }
            }

            // ── Mode-specific panels ──────────────────────────
            StackLayout {
                currentIndex: gimbalController.gimbalMode
                Layout.fillWidth: true
                Layout.fillHeight: true

                // ─── [ 0 ] PTZ ───────────────────────────────
                ColumnLayout {
                    spacing: 6

                    Label {
                        text: "PTZ Commands"
                        color: "#a6adc8"
                        font.pixelSize: 11
                    }

                    GridLayout {
                        columns: 3
                        rowSpacing: 4
                        columnSpacing: 4

                        Item { implicitWidth: 1 }
                        PtzButton { ptzLabel: "▲ UP";   cmd: 1; Layout.fillWidth: true }
                        Item { implicitWidth: 1 }

                        Item { implicitWidth: 1 }
                        PtzButton { ptzLabel: "■ STOP"; cmd: 0; accent: true; Layout.fillWidth: true }
                        Item { implicitWidth: 1 }

                        Item { implicitWidth: 1 }
                        PtzButton { ptzLabel: "▼ DOWN"; cmd: 2; Layout.fillWidth: true }
                        Item { implicitWidth: 1 }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#313244" }

                    PtzButton { ptzLabel: "⌂ CENTER";  cmd: 3; Layout.fillWidth: true }
                    PtzButton { ptzLabel: "↻ FOLLOW";  cmd: 4; Layout.fillWidth: true }
                    PtzButton { ptzLabel: "⊡ LOCK";    cmd: 5; Layout.fillWidth: true }

                    Item { Layout.fillHeight: true }
                }

                // ─── [ 1 ] Position ──────────────────────────
                ColumnLayout {
                    spacing: 8

                    Label {
                        text: "Position Mode"
                        color: "#a6adc8"
                        font.pixelSize: 11
                    }

                    // Pitch slider
                    ColumnLayout {
                        spacing: 2
                        Layout.fillWidth: true

                        RowLayout {
                            Label { text: "Pitch"; color: "#cdd6f4"; font.pixelSize: 12; Layout.fillWidth: true }
                            Label {
                                text: gimbalController.pitchDeg.toFixed(1) + "°"
                                color: "#89b4fa"
                                font.pixelSize: 13
                                font.bold: true
                            }
                        }
                        GimbalSlider {
                            Layout.fillWidth: true
                            from: -90; to: 90; stepSize: 0.5
                            value: gimbalController.pitchDeg
                            onMoved: { gimbalController.pitchDeg = value; gimbalController.sendPositionCmd() }
                        }
                    }

                    // Yaw slider
                    ColumnLayout {
                        spacing: 2
                        Layout.fillWidth: true

                        RowLayout {
                            Label { text: "Yaw"; color: "#cdd6f4"; font.pixelSize: 12; Layout.fillWidth: true }
                            Label {
                                text: gimbalController.yawDeg.toFixed(1) + "°"
                                color: "#89b4fa"
                                font.pixelSize: 13
                                font.bold: true
                            }
                        }
                        GimbalSlider {
                            Layout.fillWidth: true
                            from: -90; to: 90; stepSize: 0.5
                            value: gimbalController.yawDeg
                            onMoved: { gimbalController.yawDeg = value; gimbalController.sendPositionCmd() }
                        }
                    }

                    // Reset về giữa
                    Rectangle {
                        Layout.fillWidth: true
                        height: 34
                        radius: 6
                        color: resetArea.pressed ? "#1a4a2a" : "#1e3a2a"
                        border.color: "#50fa7b"
                        border.width: 1

                        RowLayout {
                            anchors.centerIn: parent
                            spacing: 6
                            Label { text: "⊕"; color: "#50fa7b"; font.pixelSize: 14 }
                            Label {
                                text: "Reset về giữa  (0° / 0°)"
                                color: "#50fa7b"
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }

                        MouseArea {
                            id: resetArea
                            anchors.fill: parent
                            onClicked: gimbalController.resetToCenter()
                        }
                    }

                    // Enable axis toggles
                    RowLayout {
                        Layout.fillWidth: true; spacing: 12

                        CheckBox {
                            text: "Enable Pitch"
                            checked: gimbalController.enablePitch
                            onCheckedChanged: gimbalController.enablePitch = checked
                            contentItem: Label {
                                leftPadding: parent.indicator.width + 6
                                text: parent.text; color: "#cdd6f4"; font.pixelSize: 11
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                        CheckBox {
                            text: "Enable Yaw"
                            checked: gimbalController.enableYaw
                            onCheckedChanged: gimbalController.enableYaw = checked
                            contentItem: Label {
                                leftPadding: parent.indicator.width + 6
                                text: parent.text; color: "#cdd6f4"; font.pixelSize: 11
                                verticalAlignment: Text.AlignVCenter
                            }
                        }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#313244" }

                    Label { text: "Tốc độ (°/s)"; color: "#a6adc8"; font.pixelSize: 11 }

                    // Pitch speed
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Label { text: "P"; color: "#cdd6f4"; font.pixelSize: 11 }
                        GimbalSlider {
                            Layout.fillWidth: true
                            from: 5; to: 200; stepSize: 5
                            value: gimbalController.pitchSpeed
                            accent: "#fab387"
                            onMoved: gimbalController.pitchSpeed = value
                        }
                        Label {
                            text: gimbalController.pitchSpeed.toFixed(0)
                            color: "#fab387"; font.pixelSize: 11
                            Layout.preferredWidth: 28
                        }
                    }

                    // Yaw speed
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Label { text: "Y"; color: "#cdd6f4"; font.pixelSize: 11 }
                        GimbalSlider {
                            Layout.fillWidth: true
                            from: 5; to: 200; stepSize: 5
                            value: gimbalController.yawSpeed
                            accent: "#fab387"
                            onMoved: gimbalController.yawSpeed = value
                        }
                        Label {
                            text: gimbalController.yawSpeed.toFixed(0)
                            color: "#fab387"; font.pixelSize: 11
                            Layout.preferredWidth: 28
                        }
                    }

                    Item { Layout.fillHeight: true }
                }

                // ─── [ 2 ] Velocity ──────────────────────────
                ColumnLayout {
                    spacing: 10

                    Label {
                        text: "Velocity Mode  (20 Hz)"
                        color: "#a6adc8"
                        font.pixelSize: 11
                    }

                    VelGauge {
                        Layout.fillWidth: true
                        axisLabel: "Pitch vel"
                        value: gimbalController.pitchVel
                        maxVal: 200
                    }

                    VelGauge {
                        Layout.fillWidth: true
                        axisLabel: "Yaw vel"
                        value: gimbalController.yawVel
                        maxVal: 200
                    }

                    Label {
                        Layout.fillWidth: true
                        text: "Kéo joystick để đặt tốc độ.\nThả joystick → dừng."
                        color: "#6272a4"
                        font.pixelSize: 11
                        wrapMode: Text.WordWrap
                    }

                    Item { Layout.fillHeight: true }
                }
            }
        }

        // ══════════════════════════════════════════════════════
        // Attitude feedback bar
        // ══════════════════════════════════════════════════════
        Rectangle {
            Layout.fillWidth: true
            height: 56
            radius: 6
            color: "#11111b"
            border.color: "#313244"
            border.width: 1

            RowLayout {
                anchors { fill: parent; margins: 8 }
                spacing: 0

                AttitudeBox {
                    Layout.fillWidth: true
                    axisLabel: "YAW"
                    val: gimbalController.fbYaw
                    accentColor: "#89b4fa"
                }
                Rectangle {
                    width: 1; height: parent.height * 0.6
                    color: "#313244"
                    Layout.alignment: Qt.AlignVCenter
                }
                AttitudeBox {
                    Layout.fillWidth: true
                    axisLabel: "PITCH"
                    val: gimbalController.fbPitch
                    accentColor: "#a6e3a1"
                }
                Rectangle {
                    width: 1; height: parent.height * 0.6
                    color: "#313244"
                    Layout.alignment: Qt.AlignVCenter
                }
                AttitudeBox {
                    Layout.fillWidth: true
                    axisLabel: "ROLL"
                    val: gimbalController.fbRoll
                    accentColor: "#fab387"
                }
            }
        }
    }
}
