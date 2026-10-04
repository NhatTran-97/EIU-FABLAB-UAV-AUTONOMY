import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// The Telemetry: tieu de + nguon du lieu + tan so / STALE, ben duoi la cac dong "nhan  gia tri".
// rows: [[label, value], ...]  (value "" / null -> "—").  Het du lieu (stale) -> lam mo so cu.
Rectangle {
    id: card
    property string title: ""
    property url icon: ""               // bieu tuong thiet bi canh tieu de (tuy chon)
    property color iconBg: "#5b6b82"     // nen tron sau icon: sang cho thiet bi mau toi, toi cho icon sang
    property string source: ""          // vd "PX4 · VFR_HUD" -- hien o tooltip
    property real rateHz: -1             // < 0: khong hien
    property real ageS: -1               // < 0: chua tung nhan
    property real staleAfter: 2.5
    property real expectedHz: 0          // > 0: tan so thap hon 1/2 muc nay -> SLOW (vang)
    property var rows: []
    property alias extra: extraSlot.data // noi dung them (vd chan troi nho)
    readonly property bool stale: ageS < 0 || ageS > staleAfter
    readonly property bool slow: !stale && expectedHz > 0 && rateHz >= 0 && rateHz < expectedHz * 0.5
    // FRESH xanh · SLOW vang · STALE do · NO DATA xam
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
            Layout.preferredHeight: 40          // moi the cung chieu cao tieu de -> cac dong so thang hang
            spacing: 10
            // nen tron sang nhe: thiet bi mau toi van noi tren the toi
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
            opacity: card.stale ? 0.45 : 1          // so cu: mo di, khong gia vo la live
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
        Item { Layout.fillHeight: true }       // the bi keo cao bang hang -> noi dung van o tren
        Item { id: extraSlot; Layout.fillWidth: true; implicitHeight: childrenRect.height; visible: children.length > 0 }
    }
}
