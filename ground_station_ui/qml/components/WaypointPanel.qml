import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Waypoint list being edited: add-point mode, altitude and task per waypoint, delete.
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

        RowLayout {
            Text {
                text: "Mission speed"; color: Theme.textDim; font.pixelSize: 12; Layout.fillWidth: true
                ToolTip.visible: speedHover.hovered
                ToolTip.text: "Speed between waypoints for the whole mission, sent with Start mission"
                HoverHandler { id: speedHover }
            }
            // stored in dm/s (SpinBox only takes integers) -> 0.5 m/s steps
            SpinBox {
                from: 5; to: 100; stepSize: 5; editable: true; implicitWidth: 110
                value: Math.round((mission.values.speed || 2) * 10)
                textFromValue: (v, locale) => (v / 10).toFixed(1)
                valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 10)
                onValueModified: mission.setSpeed(value / 10)
            }
            Text { text: "m/s"; color: Theme.textDim }
        }

        RowLayout {
            Text {
                text: "Arrival tolerance"; color: Theme.textDim; font.pixelSize: 12; Layout.fillWidth: true
                ToolTip.visible: tolHover.hovered
                ToolTip.text: "Distance at which a waypoint counts as reached (horizontal XY / vertical Z), for the whole mission"
                HoverHandler { id: tolHover }
            }
            // stored in cm (SpinBox only takes integers) -> 5 cm steps
            component TolBox: SpinBox {
                stepSize: 5; editable: true; implicitWidth: 104
                textFromValue: (v, locale) => (v / 100).toFixed(2)
                valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 100)
            }
            Text { text: "XY"; color: Theme.textDim; font.pixelSize: 12 }
            TolBox {
                from: 20; to: 500
                value: Math.round((mission.values.xyTolerance || 0.75) * 100)
                onValueModified: mission.setTolerances(value / 100, 0)
            }
            Text { text: "Z"; color: Theme.textDim; font.pixelSize: 12 }
            TolBox {
                from: 10; to: 300
                value: Math.round((mission.values.zTolerance || 0.5) * 100)
                onValueModified: mission.setTolerances(0, value / 100)
            }
            Text { text: "m"; color: Theme.textDim }
        }

        // while a mission runs, speed and tolerance changes are applied at once (MAV_CMD_USER_2)
        Text {
            readonly property var st: commands.liveSettings.values
            readonly property bool running: vehicle.missionProgress.values.valid === true
                && ["active", "paused"].indexOf(vehicle.missionProgress.values.state) >= 0
            readonly property bool failed: ["timeout", "no_link", "unsupported", "denied", "rejected", "failed"].indexOf(st.state) >= 0
            Layout.fillWidth: true; wrapMode: Text.WordWrap; font.pixelSize: 11
            text: !running ? "Sent with Start mission; during a mission, changes apply at once."
                : st.state === "accepted" ? "✓ Applied to the running mission"
                : st.state === "sending" || st.state === "in_progress" ? "Sending to the running mission…"
                : failed ? "⚠ Not applied: " + (st.message || st.state)
                : "Changes apply to the running mission at once"
            color: running && st.state === "accepted" ? Theme.ok : running && failed ? Theme.warn : Theme.textDim
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border }

        ListView {
            id: list
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            spacing: 6
            // model = count: editing an altitude does not recreate the delegate being typed in
            model: mission.items.length
            delegate: Rectangle {
                id: row
                required property int index
                readonly property var wp: mission.items[index] || ({ seq: 0, lat: 0, lon: 0, alt: 0, task: "none" })
                readonly property bool pick: (wp.task || "none") !== "none"
                // prohibited / restricted zone containing the waypoint ('' = clear); revision: re-evaluated when zones reload
                readonly property string zone: (airspace.revision, airspace.namesAt(wp.lat, wp.lon))
                width: list.width
                height: rowCol.implicitHeight + 16
                radius: 6
                color: panel.selected === index ? Theme.panelAlt : "transparent"
                border.color: panel.selected === index ? Theme.warn : Theme.border

                MouseArea { anchors.fill: parent; onClicked: panel.select(row.index) }

                ColumnLayout {
                    id: rowCol
                    anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                    anchors.margins: 8
                    spacing: 6
                RowLayout {
                    Layout.fillWidth: true
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
                // task at this waypoint; pick = descend, align on the ArUco marker, hook, climb back
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 6
                    Text { text: "Task"; color: Theme.textDim; font.pixelSize: 12 }
                    ComboBox {
                        id: taskBox
                        Layout.fillWidth: true
                        readonly property var keys: ["none", "pick_auto", "pick_manual"]
                        model: ["Fly through", "Pick (auto hook)", "Pick (manual hook)"]
                        currentIndex: Math.max(0, keys.indexOf(row.wp.task || "none"))
                        onActivated: (i) => mission.setTask(row.index, keys[i])
                    }
                }
                RowLayout {
                    visible: row.pick
                    Layout.fillWidth: true
                    spacing: 6
                    ToolTip.visible: heightsHover.hovered
                    ToolTip.text: "Heights above ground (rangefinder): search for the marker, then hook the load"
                    HoverHandler { id: heightsHover }
                    // stored in dm (SpinBox only takes integers) -> 0.5 m steps
                    component HeightBox: SpinBox {
                        from: 5; to: 300; stepSize: 5; editable: true
                        Layout.fillWidth: true; Layout.minimumWidth: 110
                        textFromValue: (v, locale) => (v / 10).toFixed(1)
                        valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 10)
                    }
                    Text { text: "Search"; color: Theme.textDim; font.pixelSize: 12 }
                    HeightBox {
                        value: Math.round((row.wp.searchAlt || 3) * 10)
                        onValueModified: mission.setTaskHeights(row.index, value / 10, row.wp.hookAlt || 1)
                    }
                    Text { text: "Hook"; color: Theme.textDim; font.pixelSize: 12 }
                    HeightBox {
                        value: Math.round((row.wp.hookAlt || 1) * 10)
                        onValueModified: mission.setTaskHeights(row.index, row.wp.searchAlt || 3, value / 10)
                    }
                    Text { text: "m AGL"; color: Theme.textDim; font.pixelSize: 12 }
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
