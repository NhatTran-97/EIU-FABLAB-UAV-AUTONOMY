import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

// Tai truoc vung dang hien thi vao cache .mbtiles de bay khi khong co mang (nhu QGC Offline Maps).
Rectangle {
    id: panel
    property var map: null
    property string layerId: ""
    property string layerTitle: ""
    property string error: ""

    readonly property var prog: mapService.prefetch
    readonly property bool running: prog.running === true
    readonly property var bbox: {
        if (!map || map.width <= 0 || map.height <= 0) return null
        void map.center; void map.zoomLevel; void map.width; void map.height  // tinh lai khi xem vung khac
        var r = map.visibleRegion.boundingGeoRectangle()
        if (!r.isValid) return null
        return { west: r.topLeft.longitude, north: r.topLeft.latitude,
                 east: r.bottomRight.longitude, south: r.bottomRight.latitude }
    }
    readonly property int estimate: bbox ? mapService.estimateTiles(layerId, bbox.west, bbox.south, bbox.east,
                                                                     bbox.north, zmin.value, zmax.value) : 0

    width: 360          // co dinh; noi dung ben trong (SpinBox 110 px) vua khung, chu dai tu xuong dong
    implicitHeight: col.implicitHeight + 28
    radius: Theme.radius
    color: "#ee161b22"
    border.color: Theme.border

    ColumnLayout {
        id: col
        anchors.fill: parent; anchors.margins: 14
        spacing: 8

        Text { text: "Offline area download"; color: Theme.text; font.pixelSize: 15; font.bold: true }
        Text {
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            color: Theme.textDim; font.pixelSize: 12
            text: "Area = the visible map, layer " + panel.layerTitle
                  + ". Do this while online; in the field the map is read from cache."
        }
        RowLayout {
            Text { text: "Zoom"; color: Theme.text; font.pixelSize: 12 }
            SpinBox {
                id: zmin; from: 1; to: 21; editable: true; implicitWidth: 110
                value: panel.map ? Math.max(1, Math.floor(panel.map.zoomLevel) - 2) : 14
            }
            Text { text: "→"; color: Theme.text }
            SpinBox { id: zmax; from: 1; to: 21; editable: true; value: 19; implicitWidth: 110 }
        }
        Text {
            color: panel.estimate > mapService.maxPrefetchTiles ? Theme.error : Theme.text
            font.pixelSize: 12
            // anh ve tinh ~20 KB/o
            text: panel.estimate > mapService.maxPrefetchTiles
                  ? "Area too large (> " + mapService.maxPrefetchTiles + " tiles) — zoom in to the flight area or lower max zoom"
                  : "Estimate: " + panel.estimate + " tiles  (~" + (panel.estimate * 0.02).toFixed(0) + " MB)"
            wrapMode: Text.WordWrap; Layout.fillWidth: true
        }
        RowLayout {
            Button {
                text: panel.running ? "Cancel" : "Download"
                enabled: panel.running || (panel.bbox !== null && zmin.value <= zmax.value)
                onClicked: {
                    if (panel.running) { mapService.cancelPrefetch(); return }
                    var b = panel.bbox
                    panel.error = mapService.startPrefetch(panel.layerId, b.west, b.south, b.east, b.north,
                                                           zmin.value, zmax.value)
                }
            }
            Text {
                visible: panel.running
                text: panel.prog.done + " / " + panel.prog.total
                color: Theme.text; font.pixelSize: 12
            }
        }
        ProgressBar {
            Layout.fillWidth: true
            visible: panel.running
            value: panel.prog.total > 0 ? panel.prog.done / panel.prog.total : 0
        }
        Text {
            Layout.fillWidth: true; wrapMode: Text.WordWrap
            visible: text !== ""
            text: panel.error !== "" ? panel.error : (panel.prog.message || "")
            color: panel.error !== "" ? Theme.error : Theme.textDim; font.pixelSize: 12
        }
    }
}
