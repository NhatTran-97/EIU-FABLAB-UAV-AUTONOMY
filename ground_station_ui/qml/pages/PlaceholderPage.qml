import QtQuick
import "../components"

Rectangle {
    property string title: ""
    color: Theme.panel
    radius: Theme.radius
    Text {
        anchors.centerIn: parent
        text: parent.title + " — coming later"
        color: Theme.textDim; font.pixelSize: 18
    }
}
