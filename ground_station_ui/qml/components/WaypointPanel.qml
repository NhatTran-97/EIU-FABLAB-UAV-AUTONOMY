import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Danh sach waypoint dang soan: bat che do them diem, sua do cao, xoa.
Rectangle {
    id: panel
    property bool addMode: false
    property int selected: -1
    signal addModeToggled()
    signal select(int index)
    signal hideRequested()

    radius: Theme.radius
    color: Theme.panel
    border.color: Theme.border

    ColumnLayout {
        anchors.fill: parent; anchors.margins: 14
        spacing: 10

        RowLayout {
            Text {
                text: "Waypoints (" + mission.values.count + ")"
                color: Theme.text; font.pixelSize: 16; font.bold: true
                Layout.fillWidth: true
            }
            Button {
                text: panel.addMode ? "✓ Xong" : "＋ Add"
                highlighted: panel.addMode
                onClicked: panel.addModeToggled()
            }
            Button {
                text: "»"
                Layout.preferredWidth: 32
                ToolTip.visible: hovered; ToolTip.text: "Hide waypoint list"
                onClicked: panel.hideRequested()
            }
        }

        RowLayout {
            Text { text: "Default altitude"; color: Theme.textDim; font.pixelSize: 12; Layout.fillWidth: true }
            SpinBox {
                from: 1; to: 500; editable: true; implicitWidth: 110
                value: mission.values.defaultAltitude
                onValueModified: mission.setDefaultAltitude(value)
            }
            Text { text: "m"; color: Theme.textDim }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border }

        ListView {
            id: list
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            spacing: 6
            // model = SO LUONG: sua do cao khong lam tao lai dong dang go
            model: mission.items.length
            delegate: Rectangle {
                id: row
                required property int index
                readonly property var wp: mission.items[index] || ({ seq: 0, lat: 0, lon: 0, alt: 0 })
                // ten vung cam / han che chua waypoint ('' = an toan); revision: tinh lai khi nap lai vung
                readonly property string zone: (airspace.revision, airspace.namesAt(wp.lat, wp.lon))
                width: list.width
                height: zone !== "" ? 74 : 56
                radius: 6
                color: panel.selected === index ? Theme.panelAlt : "transparent"
                border.color: panel.selected === index ? Theme.warn : Theme.border

                MouseArea { anchors.fill: parent; onClicked: panel.select(row.index) }

                RowLayout {
                    anchors.fill: parent; anchors.margins: 8
                    spacing: 8
                    Rectangle {
                        width: 26; height: 26; radius: 13
                        color: panel.selected === row.index ? Theme.warn : Theme.accent
                        Text { anchors.centerIn: parent; text: row.wp.seq; color: "white"; font.bold: true; font.pixelSize: 12 }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 0
                        Text { text: row.wp.lat.toFixed(6); color: Theme.text; font.pixelSize: 12; font.family: "monospace" }
                        Text { text: row.wp.lon.toFixed(6); color: Theme.text; font.pixelSize: 12; font.family: "monospace" }
                        Text {
                            visible: row.zone !== ""
                            text: "⚠ " + row.zone
                            color: Theme.error; font.pixelSize: 12; font.bold: true
                            elide: Text.ElideRight; Layout.fillWidth: true
                        }
                    }
                    SpinBox {
                        Layout.preferredWidth: 110
                        from: 1; to: 500; editable: true
                        value: row.wp.alt
                        onValueModified: mission.setAltitude(row.index, value)
                    }
                    Button {
                        text: "✕"
                        Layout.preferredWidth: 32
                        onClicked: mission.remove(row.index)
                    }
                }
            }

            Text {
                anchors.centerIn: parent
                visible: list.count === 0
                width: parent.width - 20
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
                color: Theme.textDim; font.pixelSize: 12
                text: "No waypoints.\nPress “＋ Add” then click the map."
            }
        }

        RowLayout {
            Button { text: "Clear all"; enabled: mission.values.count > 0; onClicked: mission.clear() }
            Text {
                Layout.fillWidth: true; wrapMode: Text.WordWrap; horizontalAlignment: Text.AlignRight
                text: "Send to drone: ⇪ button below the map"
                color: Theme.textDim; font.pixelSize: 12
            }
        }
    }
}
