import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    property string axisLabel:   ""
    property real   val:          0.0
    property color  accentColor: "#89b4fa"

    ColumnLayout {
        anchors.centerIn: parent
        spacing: 2

        Label {
            Layout.alignment: Qt.AlignHCenter
            text: axisLabel
            color: "#6272a4"
            font.pixelSize: 10
        }
        Label {
            Layout.alignment: Qt.AlignHCenter
            text: val.toFixed(1) + "°"
            color: accentColor
            font.pixelSize: 15
            font.bold: true
        }
    }
}
