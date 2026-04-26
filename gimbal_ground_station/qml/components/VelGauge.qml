import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    property string axisLabel: ""
    property real   value:  0.0
    property real   maxVal: 200.0

    implicitHeight: 36

    RowLayout {
        anchors.fill: parent
        spacing: 8

        Label {
            text: root.axisLabel
            color: "#a6adc8"
            font.pixelSize: 11
            Layout.preferredWidth: 55
        }

        // Bar track
        Rectangle {
            Layout.fillWidth: true
            height: 14
            radius: 7
            color: "#313244"

            // Filled bar (centred, grows left or right)
            Rectangle {
                id: bar
                readonly property real norm: Math.min(Math.abs(root.value) / root.maxVal, 1.0)
                height: parent.height
                radius: parent.radius
                width:  norm * parent.width / 2
                x:      root.value < 0 ? (parent.width / 2 - width) : parent.width / 2
                color:  norm > 0.7 ? "#f38ba8" : "#89b4fa"
                Behavior on width { NumberAnimation { duration: 60 } }
                Behavior on x     { NumberAnimation { duration: 60 } }
            }

            // Centre tick
            Rectangle {
                anchors.centerIn: parent
                width: 2; height: parent.height
                color: "#45475a"
            }
        }

        Label {
            text: root.value.toFixed(1) + " °/s"
            color: "#89b4fa"
            font.pixelSize: 11
            font.bold: true
            Layout.preferredWidth: 62
            horizontalAlignment: Text.AlignRight
        }
    }
}
