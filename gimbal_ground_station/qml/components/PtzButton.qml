import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    id: root
    property string ptzLabel: ""
    property int    cmd: 0
    property bool   accent: false

    height: 32
    radius: 6
    color: area.pressed
           ? Qt.darker(accent ? "#f38ba8" : "#45475a", 1.4)
           : (accent ? "#f38ba8" : "#313244")
    border.color: accent ? "#f38ba8" : "#45475a"
    border.width: 1

    Behavior on color { ColorAnimation { duration: 80 } }

    Label {
        anchors.centerIn: parent
        text: root.ptzLabel
        color: root.accent ? "#1e1e2e" : "#cdd6f4"
        font.pixelSize: 11
        font.bold: root.accent
    }

    MouseArea {
        id: area
        anchors.fill: parent
        onPressed:  gimbalController.sendPtzCmd(root.cmd)
        onReleased: {
            // UP / DOWN → stop khi nhả
            if (root.cmd === 1 || root.cmd === 2)
                gimbalController.sendPtzCmd(0)
        }
    }
}
