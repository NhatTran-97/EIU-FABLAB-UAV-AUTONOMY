import QtQuick
import QtQuick.Layouts

Rectangle {
    id: nav
    property int currentIndex: 0
    property var items: []          // [{title, enabled}]

    width: 200
    color: Theme.panel

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 6

        // ---- nhan dien: logo EIU (ban nen toi) + ten san pham ----
        RowLayout {
            objectName: "brand"
            Layout.fillWidth: true
            Layout.leftMargin: 4; Layout.topMargin: 4; Layout.bottomMargin: 6
            spacing: 10
            Image {
                source: appInfo.imagesUrl + "logo_dark.png"
                Layout.preferredHeight: 28
                Layout.preferredWidth: 28 * 413 / 115
                fillMode: Image.PreserveAspectFit
                smooth: true; mipmap: true
            }
            Text {
                text: "Drone\nGCS"
                color: Theme.textDim; font.pixelSize: 12; font.bold: true; lineHeight: 0.9
            }
        }
        Rectangle { Layout.fillWidth: true; height: 1; color: Theme.border; Layout.bottomMargin: 4 }

        Repeater {
            model: nav.items
            delegate: Rectangle {
                required property var modelData
                required property int index
                Layout.fillWidth: true
                height: 44
                radius: Theme.radius
                color: nav.currentIndex === index ? Theme.accent
                     : (mouse.containsMouse ? Theme.panelAlt : "transparent")
                opacity: modelData.enabled ? 1.0 : 0.45

                Column {
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.left: parent.left; anchors.leftMargin: 14
                    Text { text: modelData.title; color: Theme.text; font.pixelSize: 14 }
                }
                MouseArea {
                    id: mouse
                    anchors.fill: parent; hoverEnabled: true
                    onClicked: nav.currentIndex = index
                }
            }
        }
        Item { Layout.fillHeight: true }
    }
}
