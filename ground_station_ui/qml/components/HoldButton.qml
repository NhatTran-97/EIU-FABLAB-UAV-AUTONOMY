import QtQuick

// Nut "giu 1 giay de xac nhan" (lenh bay: tranh bam nham). Khoa: `lockedText` != "".
Rectangle {
    id: btn
    property string text: "Hold to confirm"
    property string lockedText: ""          // != "" -> khoa, hien ly do
    property color tint: Theme.ok
    property int holdMs: 1000
    signal confirmed()

    readonly property bool locked: lockedText !== ""
    property real progress: 0

    implicitWidth: Math.max(180, label.implicitWidth + 24); implicitHeight: 40; radius: 6
    color: Theme.panelAlt; border.color: locked ? Theme.border : tint; clip: true
    opacity: locked ? 0.5 : 1

    Rectangle { width: parent.width * btn.progress; height: parent.height; color: btn.tint }
    Text {
        id: label
        anchors.centerIn: parent
        text: btn.locked ? "🔒 " + btn.lockedText : btn.text
        color: Theme.text; font.pixelSize: 13; font.bold: true
    }
    NumberAnimation on progress {
        id: anim
        running: false; from: 0; to: 1; duration: btn.holdMs
        onFinished: {
            if (btn.progress < 1 || btn.locked) return
            btn.progress = 0
            btn.confirmed()
        }
    }
    MouseArea {
        anchors.fill: parent
        enabled: !btn.locked
        cursorShape: Qt.PointingHandCursor
        onPressed: anim.restart()
        onReleased: { anim.stop(); btn.progress = 0 }
        onCanceled: { anim.stop(); btn.progress = 0 }
    }
}
