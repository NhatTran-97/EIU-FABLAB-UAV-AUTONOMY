import QtQuick
import QtQuick.Layouts

// Splash: anh campus EIU + lop phu, ten app o phan duoi; hien holdMs roi fade out; click de bo qua.
Rectangle {
    id: splash
    anchors.fill: parent
    z: 1000
    color: "#0b1220"
    property int holdMs: 7000          // thoi gian hien splash (ms); bam vao de bo qua

    Image {
        anchors.fill: parent
        source: appInfo.imagesUrl + "eiu.png"
        fillMode: Image.PreserveAspectCrop
        smooth: true
    }
    Rectangle { anchors.fill: parent; color: "#0b1220"; opacity: 0.45 }       // dark navy overlay
    Rectangle {                                                               // gradient toi phan duoi (nen cho chu)
        anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
        height: parent.height * 0.42
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#000b1220" }
            GradientStop { position: 0.55; color: "#d90b1220" }
            GradientStop { position: 1.0; color: "#f20b1220" }
        }
    }

    ColumnLayout {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom; anchors.bottomMargin: parent.height * 0.045
        spacing: 8
        Image {
            Layout.alignment: Qt.AlignHCenter
            Layout.bottomMargin: 6
            source: appInfo.imagesUrl + "logo_dark.png"
            Layout.preferredHeight: 48
            Layout.preferredWidth: 48 * 413 / 115
            fillMode: Image.PreserveAspectFit
            smooth: true; mipmap: true
        }
        Text {
            Layout.alignment: Qt.AlignHCenter
            text: "DRONE GCS"
            color: "white"; font.pixelSize: 34; font.bold: true; font.letterSpacing: 3
        }
        Rectangle { Layout.alignment: Qt.AlignHCenter; width: 120; height: 3; radius: 2; color: Theme.accent }
        Text {
            Layout.alignment: Qt.AlignHCenter
            text: "Ground Control & Autonomous Flight Platform  ·  Eastern International University"
            color: "#d6dde8"; font.pixelSize: 18
        }
        Text {
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 4
            text: "v" + appInfo.version
            color: "#8e9aad"; font.pixelSize: 13; font.letterSpacing: 1
        }
    }

    // fade out; Timer thu 2 dam bao splash tat ke ca khi animation khong ket thuc
    Behavior on opacity { NumberAnimation { duration: 450 } }
    function close() { opacity = 0; hideTimer.start() }
    Timer { running: true; interval: splash.holdMs; onTriggered: splash.close() }
    Timer { id: hideTimer; interval: 500; onTriggered: splash.visible = false }
    MouseArea { anchors.fill: parent; onClicked: splash.close() }
}
