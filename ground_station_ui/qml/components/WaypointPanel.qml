import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Waypoint list being edited: add-point mode, altitude and task per waypoint, delete.
Rectangle {
    id: panel
    property bool addMode: false
    property bool addLocked: false      // mission running: no new waypoints until Hold / hover / landed
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
            // add mode toggle: outlined when off, filled while clicks on the map add waypoints
            Button {
                text: panel.addMode ? "✓ Done" : "＋ Add"
                implicitWidth: 96; implicitHeight: 36
                enabled: !panel.addLocked
                opacity: enabled ? 1 : 0.4
                ToolTip.visible: hovered || (panel.addLocked && addLockHover.hovered)
                ToolTip.text: panel.addLocked ? "Mission running: press Hold to add waypoints"
                            : panel.addMode ? "Adding: click the map to place waypoints. Click to stop."
                                            : "Click the map to add waypoints"
                HoverHandler { id: addLockHover }
                background: Rectangle {
                    radius: 6
                    color: panel.addMode ? (parent.down ? Qt.darker(Theme.accent, 1.4)
                                                        : parent.hovered ? Qt.lighter(Theme.accent, 1.15) : Theme.accent)
                                         : (parent.down ? Qt.darker(Theme.panelAlt, 1.3)
                                                        : parent.hovered ? Qt.lighter(Theme.panelAlt, 1.3) : Theme.panelAlt)
                    border.color: Theme.accent; border.width: panel.addMode ? 0 : 1.5
                }
                contentItem: Text {
                    text: parent.text; font.pixelSize: 14; font.bold: panel.addMode
                    color: panel.addMode ? "white" : Theme.text
                    horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                }
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

        Text {
            text: "Arrival tolerance (m)"; color: Theme.textDim; font.pixelSize: 12
            ToolTip.visible: tolHover.hovered
            ToolTip.text: "Distance at which a waypoint counts as reached (horizontal XY / vertical Z), for the whole mission"
            HoverHandler { id: tolHover }
        }
        RowLayout {
            // stored in cm (SpinBox only takes integers) -> 5 cm steps, shown in metres
            component TolBox: SpinBox {
                stepSize: 5; editable: true
                Layout.fillWidth: true; Layout.minimumWidth: 120
                textFromValue: (v, locale) => (v / 100).toFixed(2)
                valueFromText: (text, locale) => Math.round(parseFloat(text.replace(",", ".")) * 100)
            }
            Text { text: "XY"; color: Theme.textDim; font.pixelSize: 12 }
            TolBox {
                from: 20; to: 500
                value: Math.round((mission.values.xyTolerance || 0.5) * 100)
                onValueModified: mission.setTolerances(value / 100, 0)
            }
            Text { text: "Z"; color: Theme.textDim; font.pixelSize: 12 }
            TolBox {
                from: 10; to: 300
                value: Math.round((mission.values.zTolerance || 0.5) * 100)
                onValueModified: mission.setTolerances(0, value / 100)
            }
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
            // keep the waypoint being flown to in view
            Connections {
                target: commands
                function onFlyingIndexChanged() {
                    if (commands.flyingIndex >= 0) list.positionViewAtIndex(commands.flyingIndex, ListView.Contain)
                }
            }
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
                // mission progress: the waypoint being flown to, and those already passed
                readonly property bool flying: commands.flyingIndex === index
                readonly property bool passed: commands.flyingIndex > index
                // prohibited / restricted zone containing the waypoint ('' = clear); revision: re-evaluated when zones reload
                readonly property string zone: (airspace.revision, airspace.namesAt(wp.lat, wp.lon))
                width: list.width
                height: rowCol.implicitHeight + 16
                radius: 6
                color: flying ? Qt.rgba(Theme.ok.r, Theme.ok.g, Theme.ok.b, 0.12)
                     : panel.selected === index ? Theme.panelAlt : "transparent"
                border.color: flying ? Theme.ok : panel.selected === index ? Theme.warn : Theme.border
                border.width: flying ? 2 : 1
                opacity: passed ? 0.55 : 1

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
                        color: row.flying ? Theme.ok : panel.selected === row.index ? Theme.warn : Theme.accent
                        Text { anchors.centerIn: parent; text: row.passed ? "✓" : row.wp.seq; color: "white"; font.bold: true; font.pixelSize: 12 }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 0
                        Text { text: row.wp.lat.toFixed(6); color: Theme.text; font.pixelSize: 12; font.family: "monospace" }
                        Text { text: row.wp.lon.toFixed(6); color: Theme.text; font.pixelSize: 12; font.family: "monospace" }
                        Text {
                            visible: row.flying
                            text: vehicle.missionProgress.values.state === "paused" ? "⏸ Paused here" : "➤ Flying to this point"
                            color: Theme.ok; font.pixelSize: 11; font.bold: true
                        }
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
