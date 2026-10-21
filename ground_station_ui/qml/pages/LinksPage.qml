import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import "../components"

ScrollView {
    id: page
    clip: true
    contentWidth: availableWidth

    // Column count from the width: each card at least panelMin wide; never more columns than links.
    readonly property int panelMin: 520

ColumnLayout {
    width: page.availableWidth
    spacing: Theme.gap

    GridLayout {
        Layout.fillWidth: true
        columns: Math.max(1, Math.min(links.items.length,
                                      Math.floor((page.availableWidth + Theme.gap) / (page.panelMin + Theme.gap))))
        columnSpacing: Theme.gap; rowSpacing: Theme.gap

        // links from config/gcs.yaml. model = count: the list is rebuilt every tick, and using it as the model would
        // recreate the delegates (the ComboBox would lose its selection).
        Repeater {
            model: links.items.length
            delegate: LinkPanel {
                required property int index
                Layout.fillWidth: true
                Layout.fillHeight: true            // cards in a row have equal height
                Layout.preferredWidth: 1           // equal column widths
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
