import QtCore
import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import QtLocation
import QtPositioning
import "../components"

// Ban do OFFLINE (QtLocation + tile server cuc bo co cache .mbtiles) + soan waypoint.
Item {
    id: page

    readonly property var gps: vehicle.gps.values
    readonly property var home: vehicle.home.values
    readonly property var fl: vehicle.flight.values
    readonly property bool hasDrone: gps.lat !== null && gps.lat !== undefined && gps.lat !== 0
    readonly property string layerId: mapService.currentLayer     // chon o trang Cai dat
    property bool addMode: false
    readonly property bool indoor: flightEnv.state.effective === "indoor"
    property bool showWaypoints: false      // panel waypoint an / hien (nho qua Settings)
    Settings { category: "mapPage"; property alias showWaypoints: page.showWaypoints }
    property int selected: -1
    property var pin: null              // vi tri vua nhap toa do (ghim tam)
    property string searchError: ""

    // Giu vi tri / zoom khi doi lop (doi lop = tao lai Map voi plugin khac)
    property var viewCenter: QtPositioning.coordinate(mapService.defaultCenter[0], mapService.defaultCenter[1])
    property real viewZoom: mapService.defaultZoom
    property bool centeredOnDrone: false
    readonly property var map: mapLoader.item

    onHasDroneChanged: if (hasDrone && !centeredOnDrone && map) {
        map.center = QtPositioning.coordinate(gps.lat, gps.lon)
        centeredOnDrone = true
    }

    // Nhap toa do -> bay toi + ghim
    function goTo(text) {
        var ll = mapService.parseLatLon(text)
        if (ll.length !== 2) {
            searchError = "Invalid coordinates. Example: 11.052919, 106.666057"
            return
        }
        searchError = ""
        pin = QtPositioning.coordinate(ll[0], ll[1])
        if (map) {
            map.center = pin
            if (map.zoomLevel < 17) map.zoomLevel = 17
        }
    }

    // Luu vung dang xem (debounce) -> lan mo app sau hien lai dung cho nay
    Timer {
        id: saveViewTimer
        interval: 1500
        onTriggered: mapService.saveView(page.viewCenter.latitude, page.viewCenter.longitude, page.viewZoom)
    }
    onViewCenterChanged: saveViewTimer.restart()
    onViewZoomChanged: saveViewTimer.restart()

    function layerInfo(id) {
        for (var i = 0; i < mapService.layers.length; i++)
            if (mapService.layers[i].id === id) return mapService.layers[i]
        return ({})
    }

    // Mau theo cambay.mod.gov.vn: cam bay do dam, han che bay cam
    readonly property var zoneKinds: ({
        prohibited: { title: "No-fly zone",       color: "#b3261e", fill: 0.62, border: 0 },
        military:   { title: "Military area",        color: "#b3261e", fill: 0.62, border: 0 },
        restricted: { title: "Restricted zone",   color: "#f28c28", fill: 0.55, border: 0 },
        danger:     { title: "Danger zone",     color: "#f2c94c", fill: 0.40, border: 1 },
        ctr:        { title: "Airport zone (CTR)", color: "#2f80ed", fill: 0.18, border: 2 },
        other:      { title: "Other zone",          color: "#9b51e0", fill: 0.30, border: 1 }
    })
    function zoneStyle(kind) { return zoneKinds[kind] || zoneKinds.other }

    function zoneCenter(path) {
        var la = 0, lo = 0
        for (var i = 0; i < path.length; i++) { la += path[i].lat; lo += path[i].lon }
        return QtPositioning.coordinate(la / Math.max(1, path.length), lo / Math.max(1, path.length))
    }

    RowLayout {
        anchors.fill: parent
        spacing: Theme.gap

        // ===================== BAN DO =====================
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: Theme.radius
            color: Theme.panel
            clip: true

            Loader {
                id: mapLoader
                anchors.fill: parent
                sourceComponent: mapComponent
                // Doi lop -> nap lai Map voi Plugin moi (plugin cua Map khong doi duoc sau khi tao)
                property string layerKey: page.layerId
                onLayerKeyChanged: { active = false; active = true }
            }

            // ---- thong bao goc trai (duoi o tim kiem) ----
            Column {
                anchors.top: parent.top; anchors.topMargin: 60
                anchors.left: parent.left; anchors.leftMargin: 12
                width: Math.min(560, parent.width - 340)
                spacing: 8
                z: 26

                // mat telemetry PX4 sau khi da tung ket noi: so lieu tren man hinh la so cu
                Rectangle {
                    objectName: "px4LostBanner"
                    readonly property var st: vehicle.status.values
                    visible: !st.linkAlive && st.heartbeatAge >= 0
                    width: parent.width; height: lostText.implicitHeight + 16
                    radius: 6; color: "#e6b3261e"
                    Text {
                        id: lostText
                        anchors.fill: parent; anchors.margins: 8
                        text: "⛔ PX4 telemetry lost — last received " + Number(parent.st.heartbeatAge).toFixed(1) + " s ago. Values shown are stale."
                        color: "white"; font.pixelSize: 13; font.bold: true; wrapMode: Text.WordWrap
                    }
                }

                // bo tham so PX4 khong khop moi truong bay
                Rectangle {
                    objectName: "envMismatch"
                    visible: flightEnv.state.mismatch !== ""
                    width: parent.width; height: mmText.implicitHeight + 16
                    radius: 6; color: "#e6b3261e"
                    Text {
                        id: mmText
                        anchors.fill: parent; anchors.margins: 8
                        text: "⚠ " + flightEnv.state.mismatch
                        color: "white"; font.pixelSize: 12; font.bold: true; wrapMode: Text.WordWrap
                    }
                }

            }

            // ---- dong ho tu the + la ban + do cao (goc tren phai, giong QGC) ----
            InstrumentPanel {
                id: instruments
                anchors.top: parent.top; anchors.right: parent.right; anchors.margins: 12
                z: 25
                // ban do hep (cua so nho / dang mo panel waypoint) -> thu nho dong ho
                transformOrigin: Item.TopRight
                scale: parent.width < 900 ? 0.7 : 1
                readonly property real shownWidth: width * scale
            }

            // ---- cong cu ben phai (duoi khung dong ho) ----
            Column {
                anchors.right: parent.right; anchors.top: instruments.bottom
                anchors.rightMargin: 12; anchors.topMargin: 12
                spacing: 8
                component ToolBtn: Rectangle {
                    id: tb
                    property string label: ""
                    property string tip: ""            // chu thich khi re chuot
                    property bool checked: false
                    signal clicked()
                    width: 44; height: 44; radius: 22
                    color: checked ? Theme.accent : (tbMouse.containsMouse ? "#2a3445" : Theme.panelAlt)
                    border.color: checked ? Theme.text : Theme.border
                    Text { anchors.centerIn: parent; text: parent.label; color: Theme.text; font.pixelSize: 18 }
                    MouseArea { id: tbMouse; anchors.fill: parent; hoverEnabled: true; onClicked: parent.clicked() }
                    ToolTip.visible: tip !== "" && tbMouse.containsMouse
                    ToolTip.delay: 400
                    ToolTip.text: tip + (checked ? "  (on)" : "")
                }
                // panel waypoint: an / hien; so nho = so waypoint
                ToolBtn {
                    label: "⚑"; tip: "Waypoint list"
                    checked: page.showWaypoints
                    onClicked: { page.showWaypoints = !page.showWaypoints; if (!page.showWaypoints) page.addMode = false }
                    Rectangle {
                        visible: mission.values.count > 0
                        anchors.top: parent.top; anchors.right: parent.right
                        width: 18; height: 18; radius: 9; color: Theme.warn
                        Text { anchors.centerIn: parent; text: mission.values.count; color: "white"; font.pixelSize: 11; font.bold: true }
                    }
                }
                ToolBtn { label: "+"; tip: "Zoom in"; onClicked: if (page.map) page.map.zoomLevel += 1 }
                ToolBtn { label: "−"; tip: "Zoom out"; onClicked: if (page.map) page.map.zoomLevel -= 1 }
                ToolBtn {
                    label: "⌖"; tip: "Center on drone"
                    onClicked: if (page.map && page.hasDrone) page.map.center = QtPositioning.coordinate(gps.lat, gps.lon)
                }
                ToolBtn { label: "⬇"; tip: "Download offline map"; checked: prefetchPanel.visible; onClicked: prefetchPanel.visible = !prefetchPanel.visible }
                // vung cam / han che bay: bat-tat; nhan giu = nap lai file
                ToolBtn {
                    label: "⛔"; tip: "No-fly zones (long-press: reload)"
                    visible: airspace.count > 0
                    checked: airspace.visible
                    MouseArea {
                        anchors.fill: parent
                        onClicked: airspace.visible = !airspace.visible
                        onPressAndHold: airspace.reload()
                    }
                }
            }

            // ---- Take-off / Land / Gui waypoint (giua duoi) -> Jetson ----
            FlightCommands {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom; anchors.bottomMargin: 50   // tren dong trang thai ban do
                z: 30
            }

            // ---- chu thich vung bay (giong cambay.mod.gov.vn): tick de an / hien tung loai ----
            Rectangle {
                id: zoneLegend
                visible: airspace.visible && airspace.count > 0     // chua co du lieu vung -> an
                anchors.left: parent.left; anchors.bottom: parent.bottom
                anchors.leftMargin: 12; anchors.bottomMargin: 48
                width: legendCol.implicitWidth + 24; height: legendCol.implicitHeight + 20
                radius: 6; color: "#f2ffffff"
                Column {
                    id: legendCol
                    anchors.centerIn: parent
                    spacing: 6
                    Repeater {
                        // luon co 2 loai chinh nhu trang BQP + cac loai khac neu du lieu co
                        model: ["restricted", "prohibited"].concat(airspace.kinds.filter(
                                   k => k !== "restricted" && k !== "prohibited"))
                        delegate: Row {
                            required property string modelData
                            readonly property bool shown: airspace.hiddenKinds.indexOf(modelData) < 0
                            spacing: 8
                            Rectangle {
                                width: 16; height: 16; radius: 3
                                anchors.verticalCenter: parent.verticalCenter
                                color: parent.shown ? "#1f6feb" : "white"; border.color: "#1f6feb"
                                Text { anchors.centerIn: parent; text: "✓"; color: "white"; font.pixelSize: 12; visible: parent.parent.shown }
                            }
                            Rectangle {
                                width: 18; height: 14; radius: 2
                                anchors.verticalCenter: parent.verticalCenter
                                color: page.zoneStyle(modelData).color
                            }
                            Text {
                                text: page.zoneStyle(modelData).title
                                color: "#1b1f24"; font.pixelSize: 13; font.bold: true
                            }
                            TapHandler { onTapped: airspace.toggleKind(modelData) }
                        }
                    }
                    Text {
                        visible: airspace.count === 0
                        width: 250; wrapMode: Text.WordWrap
                        color: "#57606a"; font.pixelSize: 12
                        text: "No airspace data. Copy .geojson / OpenAir files into\n" + airspace.dirs[0]
                              + "\nthen long-press ⛔ to reload."
                    }
                    Text {
                        text: "<a href='http://cambay.mod.gov.vn'>Reference: cambay.mod.gov.vn ↗</a>"
                        textFormat: Text.RichText; linkColor: "#1f6feb"; font.pixelSize: 12
                        onLinkActivated: (link) => Qt.openUrlExternally(link)
                    }
                }
            }

            // ---- trang thai ban do (duoi trai) ----
            Rectangle {
                anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 12
                width: statusText.implicitWidth + 20; height: 28; radius: 6
                color: "#cc161b22"
                Text {
                    id: statusText
                    anchors.centerIn: parent
                    color: Theme.textDim; font.pixelSize: 12
                    readonly property var c: mapService.cache[page.layerId] || ({})
                    text: (mapService.online ? "● Online" : "● Offline (cache)")
                          + "   ·   cache " + (c.tiles || 0) + " tiles, " + (c.sizeMb || 0) + " MB"
                          + "   ·   zoom " + (page.map ? page.map.zoomLevel.toFixed(1) : "")
                          + "   ·   © " + (page.layerInfo(page.layerId).attribution || "")
                }
            }

            // ---- huong dan che do them diem ----
            Rectangle {
                visible: page.addMode
                anchors.top: parent.top; anchors.horizontalCenter: parent.horizontalCenter; anchors.topMargin: 12
                width: hint.implicitWidth + 24; height: 32; radius: 16
                color: Theme.accent
                Text {
                    id: hint
                    anchors.centerIn: parent; color: Theme.text; font.pixelSize: 12
                    text: "Click the map to add waypoints  ·  drag to move  ·  right-click to delete"
                }
            }

            // ---- nhap toa do (goc tren trai) ----
            Column {
                id: searchBox
                anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 12
                spacing: 6
                // khong de len khung dong ho ben phai khi cua so hep
                width: Math.max(220, Math.min(380, parent.width - instruments.shownWidth - 48))

                Rectangle {
                    width: parent.width; height: 40; radius: 8
                    color: "#ee161b22"; border.color: page.searchError !== "" ? Theme.error : Theme.border
                    RowLayout {
                        anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 6
                        spacing: 6
                        Text { text: "⌕"; color: Theme.textDim; font.pixelSize: 18 }
                        TextField {
                            id: coordInput
                            Layout.fillWidth: true
                            placeholderText: "Enter coordinates: 11.052919, 106.666057"
                            placeholderTextColor: Theme.textDim
                            color: Theme.text
                            background: null
                            selectByMouse: true
                            onAccepted: page.goTo(text)
                        }
                        Button { text: "Go"; onClicked: page.goTo(coordInput.text) }
                    }
                }
                Text {
                    visible: page.searchError !== ""
                    text: page.searchError; color: Theme.error; font.pixelSize: 12
                }
                // ghim vua nhap nam trong vung cam / han che?
                Text {
                    readonly property string zone: page.pin ? (airspace.revision, airspace.namesAt(page.pin.latitude, page.pin.longitude)) : ""
                    visible: zone !== ""
                    width: parent.width; wrapMode: Text.WordWrap
                    text: "⚠ Inside zone: " + zone
                    color: Theme.error; font.pixelSize: 12; font.bold: true; style: Text.Outline; styleColor: "black"
                }
                // ghim vua nhap: thong tin + them lam waypoint
                Rectangle {
                    visible: page.pin !== null
                    width: parent.width; height: 44; radius: 8
                    color: "#ee161b22"; border.color: Theme.border
                    RowLayout {
                        anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 6
                        Text {
                            Layout.fillWidth: true
                            text: page.pin ? page.pin.latitude.toFixed(6) + ", " + page.pin.longitude.toFixed(6) : ""
                            color: Theme.text; font.pixelSize: 12; font.family: "monospace"
                            elide: Text.ElideRight
                        }
                        Button {
                            objectName: "pinToWaypoint"
                            text: "＋ Waypoint"
                            Layout.preferredWidth: 96
                            onClicked: {
                                mission.add(page.pin.latitude, page.pin.longitude)
                                page.selected = mission.items.length - 1
                                page.pin = null
                            }
                        }
                        Button { text: "✕"; Layout.preferredWidth: 32; onClicked: page.pin = null }
                    }
                }
            }

            PrefetchPanel {
                id: prefetchPanel
                visible: false
                width: Math.max(320, Math.min(360, parent.width - instruments.shownWidth - 48))
                anchors.left: parent.left; anchors.top: searchBox.bottom; anchors.margins: 12
                map: page.map
                layerId: page.layerId
                layerTitle: page.layerInfo(page.layerId).title || ""
            }
        }

        // ===================== DANH SACH WAYPOINT =====================
        WaypointPanel {
            visible: page.showWaypoints
            Layout.preferredWidth: page.width < 1100 ? 290 : 340
            Layout.fillHeight: true
            onHideRequested: { page.showWaypoints = false; page.addMode = false }
            addMode: page.addMode
            selected: page.selected
            onAddModeToggled: page.addMode = !page.addMode
            onSelect: (i) => {
                page.selected = i
                var wp = mission.items[i]
                if (wp && page.map) page.map.center = QtPositioning.coordinate(wp.lat, wp.lon)
            }
        }
    }

    // ===================== MAP =====================
    Component {
        id: mapComponent

        Map {
            id: map
            center: page.viewCenter
            zoomLevel: page.viewZoom
            maximumZoomLevel: page.layerInfo(page.layerId).maxZoom || 19
            copyrightsVisible: false
            onCenterChanged: page.viewCenter = map.center
            onZoomLevelChanged: page.viewZoom = map.zoomLevel

            plugin: Plugin {
                name: "osm"
                // Moi o lay tu tile server cuc bo (cache .mbtiles) -> chay duoc khi mat mang
                PluginParameter {
                    name: "osm.mapping.custom.host"
                    value: "http://127.0.0.1:" + mapService.port + "/" + page.layerId + "/%z/%x/%y.png"
                }
                PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: true }
                // Cache cua QtLocation tach rieng tung lop (khong lan anh giua cac lop);
                // cache chinh la file .mbtiles nen cache dia cua QtLocation de nho
                PluginParameter { name: "osm.mapping.cache.directory"; value: mapService.qtCacheDir + "/" + page.layerId }
                PluginParameter { name: "osm.mapping.cache.disk.size"; value: 20000000 }
                PluginParameter { name: "osm.mapping.highdpi_tiles"; value: false }
            }

            Component.onCompleted: {
                for (var i = 0; i < supportedMapTypes.length; i++)
                    if (supportedMapTypes[i].style === MapType.CustomMap) activeMapType = supportedMapTypes[i]
            }

            // ---- dieu khien: keo de di chuyen, cuon / pinch de zoom ----
            DragHandler {
                target: null
                onTranslationChanged: (delta) => map.pan(-delta.x, -delta.y)
            }
            WheelHandler {
                acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
                rotationScale: 1 / 120
                property: "zoomLevel"
            }
            PinchHandler {
                id: pinch
                target: null
                property real startZoom: 0
                onActiveChanged: if (active) startZoom = map.zoomLevel
                onScaleChanged: (delta) => map.zoomLevel = startZoom + Math.log2(pinch.activeScale)
            }
            TapHandler {
                onTapped: (eventPoint) => {
                    if (!page.addMode) return
                    var c = map.toCoordinate(eventPoint.position)
                    mission.add(c.latitude, c.longitude)
                    page.selected = mission.items.length - 1
                }
            }

            // ---- vung cam / han che bay: to mau dac nhu cambay.mod.gov.vn (do = cam, cam = han che) ----
            MapItemView {
                model: airspace.visible ? airspace.zones : []
                delegate: MapPolygon {
                    required property var modelData
                    readonly property var st: page.zoneStyle(modelData.kind)
                    readonly property color c: st.color
                    visible: airspace.hiddenKinds.indexOf(modelData.kind) < 0
                    path: modelData.path.map(p => QtPositioning.coordinate(p.lat, p.lon))
                    color: Qt.rgba(c.r, c.g, c.b, st.fill)
                    border.color: c
                    border.width: st.border
                }
            }
            MapItemView {
                model: airspace.visible && map.zoomLevel >= 15 ? airspace.zones : []
                delegate: MapQuickItem {
                    required property var modelData
                    visible: modelData.name !== "" && airspace.hiddenKinds.indexOf(modelData.kind) < 0
                    coordinate: page.zoneCenter(modelData.path)
                    anchorPoint.x: zl.width / 2; anchorPoint.y: zl.height / 2
                    sourceItem: Text {
                        id: zl
                        text: modelData.name + (modelData.upper ? "\n" + (modelData.lower || "SFC") + " – " + modelData.upper : "")
                        horizontalAlignment: Text.AlignHCenter
                        color: "white"; style: Text.Outline; styleColor: "black"
                        font.pixelSize: 12; font.bold: true
                    }
                }
            }

            // ---- duong bay ----
            MapPolyline {
                line.width: 3
                line.color: Theme.accent
                path: {
                    var p = []
                    if (page.home.valid) p.push(QtPositioning.coordinate(page.home.lat, page.home.lon))
                    for (var i = 0; i < mission.items.length; i++)
                        p.push(QtPositioning.coordinate(mission.items[i].lat, mission.items[i].lon))
                    return p
                }
            }

            // ---- waypoint: model = SO LUONG de delegate khong bi tao lai khi keo ----
            MapItemView {
                model: mission.items.length
                delegate: MapQuickItem {
                    id: wpItem
                    required property int index
                    readonly property var wp: mission.items[index] || ({ lat: 0, lon: 0, alt: 0, seq: 0 })
                    coordinate: QtPositioning.coordinate(wp.lat, wp.lon)
                    anchorPoint.x: marker.width / 2
                    anchorPoint.y: marker.height / 2
                    z: 10
                    sourceItem: Rectangle {
                        id: marker
                        width: 30; height: 30; radius: 15
                        color: page.selected === wpItem.index ? Theme.warn : Theme.accent
                        border.color: "white"; border.width: 2
                        Text { anchors.centerIn: parent; text: wpItem.wp.seq; color: "white"; font.bold: true; font.pixelSize: 12 }
                        Text {
                            anchors.top: parent.bottom; anchors.horizontalCenter: parent.horizontalCenter
                            text: wpItem.wp.alt + " m"; color: "white"; font.pixelSize: 12; style: Text.Outline
                        }
                        DragHandler {
                            id: wpDrag
                            target: null
                            onCentroidChanged: if (active) {
                                var p = map.mapFromItem(null, centroid.scenePosition.x, centroid.scenePosition.y)
                                var c = map.toCoordinate(Qt.point(p.x, p.y))
                                mission.move(wpItem.index, c.latitude, c.longitude)
                            }
                            onActiveChanged: if (active) page.selected = wpItem.index
                        }
                        TapHandler { onTapped: page.selected = wpItem.index }
                        TapHandler { acceptedButtons: Qt.RightButton; onTapped: { mission.remove(wpItem.index); page.selected = -1 } }
                    }
                }
            }

            // ---- Ghim toa do vua nhap ----
            MapQuickItem {
                visible: page.pin !== null
                coordinate: page.pin || QtPositioning.coordinate(0, 0)
                anchorPoint.x: 14; anchorPoint.y: 34
                z: 15
                sourceItem: Item {
                    width: 28; height: 36
                    Rectangle {
                        width: 28; height: 28; radius: 14
                        color: Theme.error; border.color: "white"; border.width: 2
                        Rectangle { anchors.centerIn: parent; width: 10; height: 10; radius: 5; color: "white" }
                    }
                    Rectangle { x: 13; y: 27; width: 2; height: 9; color: Theme.error }
                }
            }

            // ---- Home ----
            MapQuickItem {
                visible: page.home.valid === true
                coordinate: QtPositioning.coordinate(page.home.lat || 0, page.home.lon || 0)
                anchorPoint.x: 14; anchorPoint.y: 14
                sourceItem: Rectangle {
                    width: 28; height: 28; radius: 14; color: "#1f6feb"; border.color: "white"; border.width: 2
                    Text { anchors.centerIn: parent; text: "H"; color: "white"; font.bold: true }
                }
            }

            // ---- Drone (mui ten theo heading) ----
            MapQuickItem {
                visible: page.hasDrone
                coordinate: QtPositioning.coordinate(page.gps.lat || 0, page.gps.lon || 0)
                anchorPoint.x: 18; anchorPoint.y: 18
                z: 20
                sourceItem: Canvas {
                    width: 36; height: 36
                    readonly property var att: vehicle.attitude.values
                    rotation: (att.valid ? att.heading : (page.fl.heading || 0)) - map.bearing
                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.beginPath()
                        ctx.moveTo(18, 2); ctx.lineTo(32, 32); ctx.lineTo(18, 24); ctx.lineTo(4, 32)
                        ctx.closePath()
                        ctx.fillStyle = "#2ea043"; ctx.fill()
                        ctx.lineWidth = 2; ctx.strokeStyle = "white"; ctx.stroke()
                    }
                }
            }
        }
    }
}
