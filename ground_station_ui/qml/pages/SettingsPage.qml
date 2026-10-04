import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "../components"

// Cai dat chung. Moi muc 1 khoi; them muc moi = them 1 khoi o day.
ScrollView {
    id: settingsPage
    clip: true
    contentWidth: availableWidth

ColumnLayout {
    width: settingsPage.availableWidth
    spacing: Theme.gap

    Text { text: "Settings"; color: Theme.text; font.pixelSize: 20; font.bold: true }

    // ===================== MOI TRUONG BAY =====================
    Text { text: "Flight environment"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }
    Rectangle {
        Layout.fillWidth: true
        Layout.maximumWidth: 860
        implicitHeight: envCol.implicitHeight + 28
        radius: Theme.radius; color: Theme.panel; border.color: Theme.border
        readonly property var st: flightEnv.state

        ColumnLayout {
            id: envCol
            anchors.fill: parent; anchors.margins: 14
            spacing: 10

            Row {
                spacing: 0
                Repeater {
                    model: [{ id: "auto", title: "Auto" }, { id: "indoor", title: "Indoor" }, { id: "outdoor", title: "Outdoor" }]
                    delegate: Rectangle {
                        required property var modelData
                        objectName: "env_" + modelData.id
                        width: 130; height: 36
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
                text: "Auto: GPS 3D fix → Outdoor, otherwise → Indoor.\n"
                    + "Indoor: altitude from range sensor (MTF-02), default take-off "
                    + commands.takeoffAltitudes.indoor + " m, no GPS is not an error.\n"
                    + "Outdoor: altitude above Home (GPS), default take-off " + commands.takeoffAltitudes.outdoor + " m."
            }
            GridLayout {
                columns: 2; columnSpacing: 16; rowSpacing: 4
                Text { text: "Applied"; color: Theme.textDim; font.pixelSize: 12 }
                Text { text: flightEnv.state.effectiveLabel; color: Theme.text; font.pixelSize: 13; font.bold: true }
                Text { text: "PX4 config"; color: Theme.textDim; font.pixelSize: 12 }
                Text {
                    readonly property var st: flightEnv.state
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
        Layout.maximumWidth: 860
        implicitHeight: voiceCol.implicitHeight + 28
        radius: Theme.radius; color: Theme.panel; border.color: Theme.border
        ColumnLayout {
            id: voiceCol
            anchors.fill: parent; anchors.margins: 14
            spacing: 10
            RowLayout {
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
                spacing: 0
                Button { text: "▶ Test"; enabled: voice.available && voice.enabled; onClicked: voice.test() }
                Item { width: 16 }
                Text { text: voice.lastText ? "Last: “" + voice.lastText + "”" : ""; color: Theme.textDim; font.pixelSize: 12 }
            }
        }
    }

    // ===================== BAN DO =====================
    Text { text: "Map — base layer"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }

    Flow {
        Layout.fillWidth: true
        spacing: Theme.gap

        Repeater {
            model: mapService.layers
            delegate: Rectangle {
                id: card
                required property var modelData
                readonly property bool selected: mapService.currentLayer === modelData.id
                readonly property var c: mapService.cache[modelData.id] || ({})
                objectName: "layer_" + modelData.id
                width: 280; height: 96
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

    // ===================== ABOUT =====================
    Text { text: "About"; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.topMargin: 6 }
    Rectangle {
        Layout.fillWidth: true
        Layout.maximumWidth: 860
        implicitHeight: aboutRow.implicitHeight + 28
        radius: Theme.radius; color: Theme.panel; border.color: Theme.border
        RowLayout {
            id: aboutRow
            anchors.fill: parent; anchors.margins: 14
            spacing: 20
            ColumnLayout {
                Layout.fillWidth: true
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
                Text { text: "Ground Control & Autonomous Flight Platform"; color: Theme.text; font.pixelSize: 14 }
                Text { text: "Developed at Eastern International University"; color: Theme.textDim; font.pixelSize: 13 }
                Text {
                    Layout.topMargin: 6; Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: "Version " + appInfo.version + "  ·  PX4 1.17  ·  ROS 2 Humble (Jetson)  ·  MAVLink · uXRCE-DDS"
                    color: Theme.textDim; font.pixelSize: 12
                }
            }
            Rectangle {                      // anh campus giu nguyen ti le (co san ten truong trong anh)
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
