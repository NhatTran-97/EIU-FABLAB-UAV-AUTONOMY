import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "../components"

// Cai dat chung. Moi muc 1 khoi; them muc moi = them 1 khoi vao cot trai / phai.
// Rong >= twoColumnMin: 2 cot chia deu chieu ngang; hep hon: 1 cot. About: hang rieng o duoi.
ScrollView {
    id: settingsPage
    clip: true
    contentWidth: availableWidth

    readonly property int twoColumnMin: 1100
    readonly property bool wide: availableWidth >= twoColumnMin

ColumnLayout {
    width: settingsPage.availableWidth
    spacing: Theme.gap

    Text { text: "Settings"; color: Theme.text; font.pixelSize: 20; font.bold: true }

    GridLayout {
        Layout.fillWidth: true
        columns: settingsPage.wide ? 2 : 1
        columnSpacing: Theme.gap * 2
        rowSpacing: Theme.gap

        // =========================== COT TRAI ===========================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.preferredWidth: 1          // 2 cot cung preferredWidth -> chia deu
            Layout.minimumWidth: 0
            Layout.alignment: Qt.AlignTop
            spacing: Theme.gap

            // ===================== MOI TRUONG BAY =====================
            Text { text: "Flight environment"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }
            Rectangle {
                Layout.fillWidth: true
                implicitHeight: envCol.implicitHeight + 28
                radius: Theme.radius; color: Theme.panel; border.color: Theme.border

                ColumnLayout {
                    id: envCol
                    anchors.fill: parent; anchors.margins: 14
                    spacing: 10

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 0
                        Repeater {
                            model: [{ id: "auto", title: "Auto" }, { id: "indoor", title: "Indoor" }, { id: "outdoor", title: "Outdoor" }]
                            delegate: Rectangle {
                                required property var modelData
                                objectName: "env_" + modelData.id
                                Layout.fillWidth: true
                                Layout.preferredHeight: 36
                                color: flightEnv.state.mode === modelData.id ? Theme.accent : Theme.panelAlt
                                border.color: Theme.border
                                Text { anchors.centerIn: parent; text: parent.modelData.title; color: Theme.text; font.pixelSize: 13; font.bold: true }
                                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: flightEnv.setMode(parent.modelData.id) }
                            }
                        }
                    }
                    Text {
                        Layout.fillWidth: true; wrapMode: Text.WordWrap
                        color: Theme.textDim; font.pixelSize: 12
                        text: "Auto: follows the PX4 config (EKF2_GPS_CTRL / EKF2_HGT_REF) — GPS off or range-sensor height → Indoor, "
                            + "GPS on + baro / GPS height → Outdoor. Until PX4 params are read: GPS 3D fix → Outdoor, otherwise → Indoor.\n"
                            + "Indoor: altitude from PX4 EKF (baro; range sensor if fitted), no position without GPS, default take-off "
                            + commands.takeoffAltitudes.indoor + " m, no GPS is not an error.\n"
                            + "Outdoor: altitude above Home (GPS), default take-off " + commands.takeoffAltitudes.outdoor + " m."
                    }
                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2; columnSpacing: 16; rowSpacing: 4
                        Text { text: "Applied"; color: Theme.textDim; font.pixelSize: 12 }
                        Text {
                            readonly property var st: flightEnv.state
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                            text: "<b>" + st.effectiveLabel + "</b>  "
                                + (st.source === "px4" ? "(auto, from PX4 config)"
                                 : st.source === "gps" ? "(auto, from GPS fix — PX4 params not read yet)"
                                 : "(manual)")
                            textFormat: Text.StyledText
                            color: Theme.text; font.pixelSize: 13
                        }
                        Text { text: "PX4 config"; color: Theme.textDim; font.pixelSize: 12 }
                        Text {
                            readonly property var st: flightEnv.state
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                            text: st.px4Profile === "unknown" ? "unknown (PX4 not connected?)"
                                : st.px4ProfileLabel + "  (EKF2_GPS_CTRL = " + st.gpsCtrl + ", EKF2_HGT_REF = " + st.hgtRef + ")"
                            color: Theme.text; font.pixelSize: 13
                        }
                    }
                    Text {
                        visible: text !== ""
                        Layout.fillWidth: true; wrapMode: Text.WordWrap
                        text: flightEnv.state.mismatch ? "⚠ " + flightEnv.state.mismatch : ""
                        color: Theme.error; font.pixelSize: 13; font.bold: true
                    }
                }
            }

            // ===================== GIONG NOI =====================
            Text { text: "Voice"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }
            Rectangle {
                Layout.fillWidth: true
                implicitHeight: voiceCol.implicitHeight + 28
                radius: Theme.radius; color: Theme.panel; border.color: Theme.border
                ColumnLayout {
                    id: voiceCol
                    anchors.fill: parent; anchors.margins: 14
                    spacing: 10
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 12
                        Switch {
                            objectName: "voiceSwitch"
                            checked: voice.enabled
                            enabled: voice.available
                            onToggled: voice.enabled = checked
                        }
                        Text {
                            text: voice.available ? "Spoken alerts: flight mode, arm / disarm, link loss, low battery"
                                                  : "No speech engine (install speech-dispatcher + espeak-ng)"
                            color: voice.available ? Theme.text : Theme.warn; font.pixelSize: 13
                            Layout.fillWidth: true; wrapMode: Text.WordWrap
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 16
                        Button { text: "▶ Test"; enabled: voice.available && voice.enabled; onClicked: voice.test() }
                        Text {
                            Layout.fillWidth: true; elide: Text.ElideRight
                            text: voice.lastText ? "Last: “" + voice.lastText + "”" : ""
                            color: Theme.textDim; font.pixelSize: 12
                        }
                    }
                }
            }
        }

        // =========================== COT PHAI ===========================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.preferredWidth: 1
            Layout.minimumWidth: 0
            Layout.alignment: Qt.AlignTop
            spacing: Theme.gap

            // ===================== BAN DO =====================
            Text { text: "Map — base layer"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }

            GridLayout {
                id: layerGrid
                readonly property int cardMin: 240
                Layout.fillWidth: true
                // so cot = so lop (toi da) hoac so the vua chieu ngang -> the gian deu het hang
                columns: Math.max(1, Math.min(mapService.layers.length,
                                              Math.floor((width + Theme.gap) / (cardMin + Theme.gap))))
                columnSpacing: Theme.gap; rowSpacing: Theme.gap

                Repeater {
                    model: mapService.layers
                    delegate: Rectangle {
                        id: card
                        required property var modelData
                        readonly property bool selected: mapService.currentLayer === modelData.id
                        readonly property var c: mapService.cache[modelData.id] || ({})
                        objectName: "layer_" + modelData.id
                        Layout.fillWidth: true
                        Layout.preferredWidth: 1
                        Layout.minimumWidth: 0
                        Layout.preferredHeight: 96
                        radius: Theme.radius
                        color: selected ? Theme.panelAlt : Theme.panel
                        border.color: selected ? Theme.accent : Theme.border
                        border.width: selected ? 2 : 1

                        ColumnLayout {
                            anchors.fill: parent; anchors.margins: 14
                            spacing: 4
                            RowLayout {
                                Layout.fillWidth: true
                                Text {
                                    text: card.modelData.title
                                    color: Theme.text; font.pixelSize: 15; font.bold: true
                                    Layout.fillWidth: true
                                }
                                Text {
                                    visible: card.modelData.offlineOnly === true
                                    text: "offline only"; color: Theme.warn; font.pixelSize: 12
                                }
                                Rectangle {
                                    width: 20; height: 20; radius: 10
                                    color: card.selected ? Theme.accent : "transparent"
                                    border.color: card.selected ? Theme.accent : Theme.textDim
                                    Text { anchors.centerIn: parent; visible: card.selected; text: "✓"; color: "white"; font.pixelSize: 12; font.bold: true }
                                }
                            }
                            Text {
                                Layout.fillWidth: true; elide: Text.ElideRight
                                text: "© " + (card.modelData.attribution || "—")
                                color: Theme.textDim; font.pixelSize: 12
                            }
                            Text {
                                text: card.c.tiles !== undefined
                                      ? "Cache: " + card.c.tiles + " tiles · " + card.c.sizeMb + " MB" : "Cache: —"
                                color: Theme.textDim; font.pixelSize: 12
                            }
                        }
                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: mapService.currentLayer = card.modelData.id
                        }
                    }
                }
            }

            Text {
                Layout.fillWidth: true; wrapMode: Text.WordWrap
                color: Theme.textDim; font.pixelSize: 12
                text: "Choice is remembered. Add / remove layers: config/gcs.yaml · "
                    + "API key (Azure): config/keys.yaml · Offline map cache: data/maps/ in the package."
            }
        }
    }

    // ===================== ABOUT (hang rieng, full ngang) =====================
    Text { text: "About"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }
    Rectangle {
        Layout.fillWidth: true
        implicitHeight: aboutRow.implicitHeight + 28
        radius: Theme.radius; color: Theme.panel; border.color: Theme.border
        RowLayout {
            id: aboutRow
            anchors.fill: parent; anchors.margins: 14
            spacing: 20
            ColumnLayout {
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                spacing: 6
                RowLayout {
                    spacing: 12
                    Image {
                        source: appInfo.imagesUrl + "logo_dark.png"
                        Layout.preferredHeight: 34; Layout.preferredWidth: 34 * 413 / 115
                        fillMode: Image.PreserveAspectFit; smooth: true; mipmap: true
                    }
                    Text { text: "Drone GCS"; color: Theme.text; font.pixelSize: 20; font.bold: true }
                }
                Rectangle { width: 70; height: 3; radius: 2; color: Theme.accent }
                Text {
                    Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: "Ground Control & Autonomous Flight Platform"; color: Theme.text; font.pixelSize: 14
                }
                Text {
                    Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: "Developed at Eastern International University"; color: Theme.textDim; font.pixelSize: 13
                }
                Text {
                    Layout.topMargin: 6; Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: "Version " + appInfo.version + "  ·  PX4 1.17  ·  ROS 2 Humble (Jetson)  ·  MAVLink · uXRCE-DDS"
                    color: Theme.textDim; font.pixelSize: 12
                }
            }
            Rectangle {                      // anh campus giu nguyen ti le (co san ten truong trong anh)
                visible: aboutRow.width >= 560   // the qua hep -> an anh, giu chu
                Layout.preferredWidth: 300; Layout.preferredHeight: 167
                radius: 8; clip: true; color: "transparent"
                Image {
                    anchors.fill: parent
                    source: appInfo.imagesUrl + "eiu.png"
                    fillMode: Image.PreserveAspectFit
                    smooth: true
                }
            }
        }
    }

    Item { Layout.fillHeight: true }
}
}
