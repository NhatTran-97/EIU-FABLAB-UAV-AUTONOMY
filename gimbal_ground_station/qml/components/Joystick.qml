import QtQuick 2.15

Item {
    id: root
    width: 200
    height: 200

    property real joystickX: 0.0   // -1 .. 1  (yaw)
    property real joystickY: 0.0   // -1 .. 1  (pitch, up = negative)
    property bool active: false
    property color baseColor:   "#313244"
    property color stickColor:  "#89b4fa"
    property color activeColor: "#cba6f7"
    property string label: ""

    signal moved(real x, real y)
    signal joystickReleased()

    readonly property real _r:      Math.min(width, height) / 2
    readonly property real _sr:     _r * 0.33
    readonly property real _maxD:   _r - _sr - 6

    // ── Base circle ────────────────────────────────────────────
    Rectangle {
        anchors.centerIn: parent
        width: root._r * 2; height: root._r * 2; radius: root._r
        color: root.baseColor
        border.color: root.active ? root.activeColor : "#45475a"
        border.width: 2

        // Crosshair
        Rectangle { anchors.centerIn: parent; width: parent.width - 20; height: 1; color: "#45475a"; opacity: 0.5 }
        Rectangle { anchors.centerIn: parent; width: 1; height: parent.height - 20; color: "#45475a"; opacity: 0.5 }

        // Range rings
        Rectangle {
            anchors.centerIn: parent
            width: parent.width * 0.6; height: parent.width * 0.6; radius: width / 2
            color: "transparent"; border.color: "#45475a"; border.width: 1; opacity: 0.3
        }
    }

    // ── Stick ──────────────────────────────────────────────────
    Rectangle {
        id: stick
        width: root._sr * 2; height: root._sr * 2; radius: root._sr
        color: root.active ? root.activeColor : root.stickColor
        x: root.width  / 2 - root._sr + root.joystickX * root._maxD
        y: root.height / 2 - root._sr + root.joystickY * root._maxD

        Behavior on x { enabled: !area.pressed; NumberAnimation { duration: 120; easing.type: Easing.OutQuad } }
        Behavior on y { enabled: !area.pressed; NumberAnimation { duration: 120; easing.type: Easing.OutQuad } }

        // Shine dot
        Rectangle {
            anchors { top: parent.top; left: parent.left; topMargin: 4; leftMargin: 4 }
            width: parent.width * 0.3; height: parent.width * 0.3; radius: width / 2
            color: "#ffffff"; opacity: 0.35
        }
    }

    // ── Label ──────────────────────────────────────────────────
    Text {
        anchors { bottom: parent.bottom; horizontalCenter: parent.horizontalCenter; bottomMargin: -40 }
        text: root.label
        color: "#6272a4"; font.pixelSize: 15
        visible: root.label !== ""
    }

    // ── Mouse / touch ──────────────────────────────────────────
    MouseArea {
        id: area
        anchors.fill: parent

        onPressed:  root.active = true

        onReleased: {
            root.active    = false
            root.joystickX = 0
            root.joystickY = 0
            root.joystickReleased()
            root.moved(0, 0)
        }

        onPositionChanged: {
            var dx = mouseX - root.width  / 2
            var dy = mouseY - root.height / 2
            var d  = Math.sqrt(dx * dx + dy * dy)
            if (d > root._maxD) { dx = dx * root._maxD / d; dy = dy * root._maxD / d }
            root.joystickX = dx / root._maxD
            root.joystickY = dy / root._maxD
            root.moved(root.joystickX, root.joystickY)
        }
    }
}
