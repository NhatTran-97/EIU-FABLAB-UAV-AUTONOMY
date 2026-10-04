import QtQuick
import QtQuick.Layouts
import "../components"

ColumnLayout {
    spacing: Theme.gap

    // Danh sach link lay tu config/gcs.yaml. Model = SO LUONG (khong phai list): list duoc
    // tao moi moi tick, neu dung lam model thi delegate bi tao lai -> ComboBox mat lua chon.
    Repeater {
        model: links.items.length
        delegate: LinkPanel {
            required property int index
            Layout.fillWidth: true
            Layout.maximumWidth: 1100          // khong keo gian het man hinh rong
            info: links.items[index] || ({})
        }
    }
    Text {
        Layout.fillWidth: true
        Layout.maximumWidth: 1100
        wrapMode: Text.WordWrap
        color: Theme.textDim; font.pixelSize: 12
        text: "Selected ports are remembered and reconnect on start. Add / remove links: config/gcs.yaml. "
            + "Firmware / PX4 setup: close this app and open QGroundControl (one app per port)."
    }
    Item { Layout.fillHeight: true }
}
