import QtQuick 2.15
import QtQuick.Controls 2.15

Slider {
    id: control
    property color accent: "#89b4fa"

    // vị trí 0 trên track (normalised 0..1)
    readonly property real zeroPos: (0 - control.from) / (control.to - control.from)

    background: Rectangle {
        x: control.leftPadding
        y: control.topPadding + control.availableHeight / 2 - height / 2
        implicitWidth: 200
        implicitHeight: 4
        width: control.availableWidth
        height: implicitHeight
        radius: 2
        color: "#313244"

        // Bar từ 0 ra giá trị hiện tại
        Rectangle {
            readonly property real zp: control.zeroPos
            readonly property real vp: control.visualPosition
            x:      Math.min(zp, vp) * parent.width
            width:  Math.abs(vp - zp) * parent.width
            height: parent.height
            radius: 2
            color:  control.accent
        }

        // Vạch đứng tại 0
        Rectangle {
            x:      control.zeroPos * parent.width - 1
            width:  2
            height: parent.height + 4
            anchors.verticalCenter: parent.verticalCenter
            color:  "#6272a4"
            radius: 1
        }
    }

    handle: Rectangle {
        x: control.leftPadding + control.visualPosition * (control.availableWidth - width)
        y: control.topPadding + control.availableHeight / 2 - height / 2
        implicitWidth: 14
        implicitHeight: 14
        radius: 7
        color: control.pressed ? "#cba6f7" : control.accent
        border.color: "#1e1e2e"
        border.width: 2
    }
}
