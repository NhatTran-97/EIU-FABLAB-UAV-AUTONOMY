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

    // Mau cho cac control cua style Basic (ComboBox, Button...) theo nen toi
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
                // Mo app luon vao Map (trang van hanh chinh); ket noi xem o thanh tren cung / Connections
                Component.onCompleted: currentIndex = win.featureIndex("map")
            }

            // Moi tinh nang 1 trang, sinh tu danh sach `features` (Python)
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

    // Man hinh khoi dong (nhan dien EIU), tu an sau ~7 s (bam de bo qua)
    Splash {}
}
