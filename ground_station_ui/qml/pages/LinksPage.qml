import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "../components"

ScrollView {
    id: page
    clip: true
    contentWidth: availableWidth

    // So cot theo do rong: moi the toi thieu panelMin; khong nhieu cot hon so link.
    readonly property int panelMin: 520

ColumnLayout {
    width: page.availableWidth
    spacing: Theme.gap

    GridLayout {
        Layout.fillWidth: true
        columns: Math.max(1, Math.min(links.items.length,
                                      Math.floor((page.availableWidth + Theme.gap) / (page.panelMin + Theme.gap))))
        columnSpacing: Theme.gap; rowSpacing: Theme.gap

        // Danh sach link lay tu config/gcs.yaml. Model = SO LUONG (khong phai list): list duoc
        // tao moi moi tick, neu dung lam model thi delegate bi tao lai -> ComboBox mat lua chon.
        Repeater {
            model: links.items.length
            delegate: LinkPanel {
                required property int index
                Layout.fillWidth: true
                Layout.fillHeight: true            // the cung hang cao bang nhau
                Layout.preferredWidth: 1           // chia deu cot
                Layout.minimumWidth: 0
                info: links.items[index] || ({})
            }
        }
    }
    Text {
        Layout.fillWidth: true
        wrapMode: Text.WordWrap
        color: Theme.textDim; font.pixelSize: 12
        text: "Selected ports are remembered and reconnect on start. Add / remove links: config/gcs.yaml. "
            + "Firmware / PX4 setup: close this app and open QGroundControl (one app per port)."
    }
    Item { Layout.fillHeight: true }
}
}
