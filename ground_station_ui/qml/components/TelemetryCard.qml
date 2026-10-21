import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Telemetry card: title + data source + rate / STALE, followed by "label  value" rows.
// rows: [[label, value], ...]  (value "" / null -> "—"). Stale data is dimmed.
Rectangle {
    id: card
    property string title: ""
    property url icon: ""               // component icon next to the title (optional)
    property color iconBg: "#5b6b82"     // round background behind the icon: light for dark devices, dark for light icons
    property string source: ""          // e.g. "PX4 · VFR_HUD", shown in the tooltip
    property real rateHz: -1             // < 0: hidden
    property real ageS: -1               // < 0: never received
    property real staleAfter: 2.5
    property real expectedHz: 0          // > 0: rate below half of this -> SLOW (yellow)
    property var rows: []
    property alias extra: extraSlot.data // extra content (e.g. a small horizon)
    readonly property bool stale: ageS < 0 || ageS > staleAfter
    readonly property bool slow: !stale && expectedHz > 0 && rateHz >= 0 && rateHz < expectedHz * 0.5
    // FRESH green · SLOW yellow · STALE red · NO DATA grey
    readonly property color stateColor: ageS < 0 ? Theme.idle : (stale ? Theme.error : (slow ? Theme.warn : Theme.ok))

    implicitWidth: 340
    implicitHeight: col.implicitHeight + 28
    radius: Theme.radius
    color: Theme.panel
    border.color: stale ? Theme.border : "#2f3b4d"

    ColumnLayout {
        id: col
        anchors.fill: parent; anchors.margins: 14
        spacing: 8

        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 40          // same title height on every card so value rows line up
            spacing: 10
            // light round background so dark devices stand out on the dark card
            Rectangle {
                visible: card.icon != ""
                Layout.preferredWidth: 40; Layout.preferredHeight: 40; radius: 20
                color: card.iconBg; border.color: Qt.lighter(card.iconBg, 1.3)
                Image {
                    anchors.centerIn: parent
                    width: 32; height: 32
                    source: card.icon
                    fillMode: Image.PreserveAspectFit
                    smooth: true; mipmap: true
                }
            }
            Text { text: card.title; color: Theme.text; font.pixelSize: 15; font.bold: true; Layout.fillWidth: true }
            Rectangle {
                implicitWidth: badge.implicitWidth + 12; implicitHeight: 20; radius: 10
                color: Qt.rgba(card.stateColor.r, card.stateColor.g, card.stateColor.b, 0.18)
                Text {
                    id: badge
                    anchors.centerIn: parent
                    font.pixelSize: 11; font.bold: true
                    color: card.stateColor
                    text: card.ageS < 0 ? "NO DATA"
                        : card.stale ? "STALE · " + card.ageS.toFixed(1) + " s"
                        : (card.rateHz >= 0 ? (card.slow ? "SLOW · " : "") + card.rateHz.toFixed(1) + " Hz" : "LIVE")
                }
                MouseArea { id: srcHover; anchors.fill: parent; hoverEnabled: true }
                ToolTip.visible: card.source !== "" && srcHover.containsMouse
                ToolTip.text: "Source: " + card.source + (card.expectedHz > 0 ? "\nExpected ≥ " + (card.expectedHz * 0.5).toFixed(1) + " Hz" : "")
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 2; columnSpacing: 16; rowSpacing: 4
            opacity: card.stale ? 0.45 : 1          // stale: dimmed
            Repeater {
                model: card.rows.length * 2
                delegate: Text {
                    required property int index
                    readonly property var row: card.rows[Math.floor(index / 2)] || ["", ""]
                    readonly property bool isValue: index % 2 === 1
                    text: isValue ? ((row[1] === null || row[1] === undefined || row[1] === "") ? "—" : row[1]) : row[0]
                    color: isValue ? Theme.text : Theme.textDim
                    font.pixelSize: isValue ? 16 : 13
                    font.bold: isValue
                    font.family: isValue ? "monospace" : Qt.application.font.family
                    Layout.alignment: isValue ? Qt.AlignRight : Qt.AlignLeft
                    Layout.fillWidth: !isValue
                }
            }
        }
        Item { Layout.fillHeight: true }       // card stretched to the row height -> content stays at the top
        Item { id: extraSlot; Layout.fillWidth: true; implicitHeight: childrenRect.height; visible: children.length > 0 }
    }
}
