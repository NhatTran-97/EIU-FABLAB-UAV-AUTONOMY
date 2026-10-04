import QtQuick
import QtQuick.Layouts

// The trang thai: cham mau + ten + dong chinh + dong phu.
Rectangle {
    id: card
    property string title: ""
    property string status: "unknown"     // ok / warn / error / stale / off / none / unknown
    property string value: ""
    property string detail: ""
    property bool showDot: true            // false: the chi hien so lieu (thong so bay)

    width: 200
    height: 78
    radius: Theme.radius
    color: Theme.panelAlt
    border.color: status === "error" ? Theme.error : Theme.border

    RowLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 12

        Rectangle {
            visible: card.showDot
            Layout.alignment: Qt.AlignVCenter
            width: 14; height: 14; radius: 7
            color: Theme.statusColor(card.status)
        }
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 2
            Text { text: card.title; color: Theme.textDim; font.pixelSize: 12; elide: Text.ElideRight; Layout.fillWidth: true }
            Text {
                text: card.value
                color: Theme.statusColor(card.status) === Theme.idle ? Theme.text : Theme.statusColor(card.status)
                font.pixelSize: 15; font.bold: true
                elide: Text.ElideRight; Layout.fillWidth: true
            }
            Text {
                text: card.detail; visible: text !== ""
                color: Theme.textDim; font.pixelSize: 12
                elide: Text.ElideRight; Layout.fillWidth: true
            }
        }
    }
}
