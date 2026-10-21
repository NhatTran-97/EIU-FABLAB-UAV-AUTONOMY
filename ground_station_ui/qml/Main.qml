import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "components"

ApplicationWindow {
    id: win
    width: 1536
    height: 960
    minimumWidth: 1100
    minimumHeight: 700
    visible: true
    title: "EIU Drone GCS"
    color: Theme.bg

    // Colours for Basic-style controls (ComboBox, Button...) on the dark background
    palette.window: Theme.panel
    palette.base: Theme.panelAlt
    palette.button: Theme.panelAlt
    palette.buttonText: Theme.text
    palette.text: Theme.text
    palette.windowText: Theme.text
    palette.highlight: Theme.accent
    palette.highlightedText: Theme.text
    palette.mid: Theme.border
    palette.light: Theme.panelAlt
    palette.dark: Theme.border
    palette.disabled.buttonText: Theme.textDim
    palette.disabled.text: Theme.textDim

    function featureIndex(id) {
        for (var i = 0; i < features.length; i++)
            if (features[i].id === id) return i
        return 0
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TopBar { Layout.fillWidth: true }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            NavBar {
                id: nav
                objectName: "nav"
                Layout.fillHeight: true
                items: features
                // default page: Map
                Component.onCompleted: currentIndex = win.featureIndex("map")
            }

            // one page per feature, generated from the `features` list (Python)
            StackLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.margins: Theme.gap
                currentIndex: nav.currentIndex

                Repeater {
                    model: features
                    delegate: Loader {
                        required property var modelData
                        source: modelData.page
                        onLoaded: if (item.hasOwnProperty("title")) item.title = modelData.title
                    }
                }
            }
        }
    }

    // splash ~7 s, click to skip
    Splash {}
}
