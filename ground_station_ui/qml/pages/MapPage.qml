import QtCore
import QtQuick
import QtQuick3D
import QtQuick.Layouts
import QtQuick.Controls.Basic
import QtLocation
import QtPositioning
import "../components"
import "../models/eiu_drone"

// Ban do offline (QtLocation + tile server localhost, cache .mbtiles) + soan waypoint.
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
    property bool showTrack: true          // vet bay GPS (nut 〰; nhan giu = xoa)
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
                // doi lop -> tao lai Map (Map.plugin chi gan duoc luc tao)
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

                // mat telemetry PX4 sau khi da ket noi: so lieu dang hien la gia tri cu
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
                // dat lai bearing = 0, tilt = 0
                ToolBtn {
                    visible: page.map !== null && (Math.abs(page.map.bearing) > 0.5 || page.map.tilt > 0.5)
                    label: "⟲N"; tip: "Reset rotation / tilt (right-drag to rotate & tilt)"
                    onClicked: { page.map.bearing = 0; page.map.tilt = 0 }
                }
                ToolBtn { label: "+"; tip: "Zoom in"; onClicked: if (page.map) page.map.zoomLevel += 1 }
                ToolBtn { label: "−"; tip: "Zoom out"; onClicked: if (page.map) page.map.zoomLevel -= 1 }
                ToolBtn {
                    label: "⌖"; tip: "Center on drone"
                    onClicked: if (page.map && page.hasDrone) page.map.center = QtPositioning.coordinate(gps.lat, gps.lon)
                }
                ToolBtn { label: "⬇"; tip: "Download offline map"; checked: prefetchPanel.visible; onClicked: prefetchPanel.visible = !prefetchPanel.visible }
                // vet bay: click = bat / tat; giu = xoa vet hien thi (GPX khong doi)
                ToolBtn {
                    readonly property var tr: vehicle.track.values
                    label: "〰"
                    tip: "Flight track: " + (tr.lengthM || 0).toFixed(0) + " m"
                         + (tr.recording ? " · recording" : "")
                         + (tr.lastFile ? "\nSaved: " + tr.lastFile : "") + "\n(long-press: clear)"
                    checked: page.showTrack
                    MouseArea {
                        anchors.fill: parent
                        onClicked: page.showTrack = !page.showTrack
                        onPressAndHold: vehicle.clearTrack()
                    }
                    Rectangle {                            // cham do: dang ghi (armed)
                        visible: parent.tr.recording === true
                        anchors.top: parent.top; anchors.right: parent.right
                        width: 12; height: 12; radius: 6; color: Theme.error
                        border.color: "white"; border.width: 1.5
                    }
                }
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
                            text: mapService.searchDefault          // dien san (gcs.yaml map.search_default)
                            placeholderText: "Enter coordinates: lat, lon"
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
                // o ban do tu tile server localhost (cache .mbtiles)
                PluginParameter {
                    name: "osm.mapping.custom.host"
                    value: "http://127.0.0.1:" + mapService.port + "/" + page.layerId + "/%z/%x/%y.png"
                }
                PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: true }
                // cache dia QtLocation: rieng tung lop, dung luong nho (cache chinh la .mbtiles)
                PluginParameter { name: "osm.mapping.cache.directory"; value: mapService.qtCacheDir + "/" + page.layerId }
                PluginParameter { name: "osm.mapping.cache.disk.size"; value: 20000000 }
                PluginParameter { name: "osm.mapping.highdpi_tiles"; value: false }
            }

            Component.onCompleted: {
                for (var i = 0; i < supportedMapTypes.length; i++)
                    if (supportedMapTypes[i].style === MapType.CustomMap) activeMapType = supportedMapTypes[i]
                rebuildTrack()                       // vet da co truoc khi tao Map (vd doi lop)
            }

            // ---- dieu khien: keo trai = pan, cuon / pinch = zoom,
            //      keo phai: dx -> bearing, dy -> tilt (nut ⟲N dat lai) ----
            DragHandler {
                target: null
                onTranslationChanged: (delta) => map.pan(-delta.x, -delta.y)
            }
            DragHandler {
                target: null
                acceptedButtons: Qt.RightButton           // tap phai (xoa waypoint) do TapHandler rieng
                onTranslationChanged: (delta) => {
                    map.bearing = (map.bearing + delta.x * 0.4 + 360) % 360
                    map.tilt = Math.max(map.minimumTilt, Math.min(map.maximumTilt, map.tilt - delta.y * 0.3))
                }
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

            // ---- waypoint: model = so luong -> delegate khong bi tao lai khi keo ----
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

            // ---- vet bay GPS: 1 MapPolyline / doan (1 doan = 1 lan arm).
            //      vehicle.trackPath: appended -> addCoordinate; reset -> rebuildTrack() ----
            property var trackLines: []
            Component {
                id: trackLineComp
                MapPolyline { z: 15; line.width: 3; line.color: "#2f9bff"; visible: page.showTrack }
            }
            function trackLine(i) {
                while (trackLines.length <= i) {
                    var l = trackLineComp.createObject(map)
                    map.addMapItem(l)
                    trackLines.push(l)
                }
                return trackLines[i]
            }
            function rebuildTrack() {
                for (var i = 0; i < trackLines.length; i++) { map.removeMapItem(trackLines[i]); trackLines[i].destroy() }
                trackLines = []
                var segs = vehicle.trackPath.paths()
                for (var s = 0; s < segs.length; s++) {
                    var path = []
                    for (var k = 0; k < segs[s].length; k++)
                        path.push(QtPositioning.coordinate(segs[s][k][0], segs[s][k][1]))
                    trackLine(s).path = path
                }
            }
            Connections {
                target: vehicle.trackPath
                function onReset() { map.rebuildTrack() }
                function onAppended(points) {
                    for (var i = 0; i < points.length; i++)
                        map.trackLine(points[i][0]).addCoordinate(QtPositioning.coordinate(points[i][1], points[i][2]))
                }
            }

            // ---- drone ----
            //  coordinate = vi tri GPS (bong tren mat dat).
            //  tilt <= 5 do : anh drone_top.png, rotation = heading - bearing.
            //  tilt  > 5 do : EiuDrone (View3D), camera cung goc tilt / bearing voi ban do; roll / pitch tu attitude.
            //  lift (px) = altRel / mpp * sin(tilt): drone ve cao hon bong, noi bang cot do cao.
            //  kich thuoc: ti le theo ban do = kich thuoc that (spanM) / mpp x exaggeration, gioi han [minPx, maxPx].
            MapQuickItem {
                id: droneMarker
                visible: page.hasDrone
                coordinate: QtPositioning.coordinate(page.gps.lat || 0, page.gps.lon || 0)
                // noi suy vi tri giua cac mau (UI 5 Hz, Jetson 2 Hz); tat cho den khi co vi tri dau tien
                // (tranh animate tu (0, 0))
                property bool smoothReady: false
                Timer { running: page.hasDrone; interval: 500; onTriggered: droneMarker.smoothReady = true }
                Connections { target: page; function onHasDroneChanged() { if (!page.hasDrone) droneMarker.smoothReady = false } }
                Behavior on coordinate {
                    enabled: droneMarker.smoothReady
                    CoordinateAnimation { duration: 250 }
                }
                anchorPoint.x: droneIcon.width / 2; anchorPoint.y: droneIcon.height - droneIcon.box / 2
                z: 20
                sourceItem: Item {
                    id: droneIcon
                    readonly property real spanM: 0.9                   // be ngang khung drone_top.png (m)
                    readonly property real exaggeration: 12
                    readonly property real minPx: 24
                    readonly property real maxPx: 240
                    // box: khung ve; drone (anh 2D) chiem 0.75 box
                    readonly property real box: Math.max(minPx, Math.min(maxPx, spanM / mpp * exaggeration / 0.75))
                    readonly property var att: vehicle.attitude.values
                    readonly property real heading: att.valid ? att.heading : (page.fl.heading || 0)
                    readonly property real alt: Math.max(0, page.fl.altRel || 0)
                    // m/px (Web Mercator, o 256 px) tai vi do drone
                    readonly property real mpp: 156543.03392 * Math.cos((page.gps.lat || 0) * Math.PI / 180)
                                               / Math.pow(2, map.zoomLevel)
                    readonly property real lift: Math.min(600, alt / mpp * Math.sin(map.tilt * Math.PI / 180))
                    readonly property bool view3d: map.tilt > 5
                    width: box; height: box + lift

                    // bong (diem GPS) + cot do cao
                    Rectangle {
                        visible: droneIcon.lift > 3
                        width: droneIcon.box * 0.35; height: width * Math.max(0.25, Math.cos(map.tilt * Math.PI / 180)); radius: width / 2
                        x: (droneIcon.width - width) / 2; y: droneIcon.height - droneIcon.box / 2 - height / 2
                        color: "#73000000"
                    }
                    Rectangle {
                        visible: droneIcon.lift > 3
                        width: 4; height: droneIcon.lift
                        x: (droneIcon.width - width) / 2; y: droneIcon.box / 2
                        color: "#2ea043"
                    }

                    // nhan do cao
                    Rectangle {
                        visible: droneIcon.lift > 3 && droneIcon.alt >= 0.3
                        x: droneIcon.box - 14; y: droneIcon.box / 2 - height / 2
                        width: altText.implicitWidth + 10; height: 18; radius: 4
                        color: "#cc161b22"; border.color: "#2f9bff"
                        Text {
                            id: altText
                            anchors.centerIn: parent
                            text: droneIcon.alt.toFixed(1) + " m"
                            color: "white"; font.pixelSize: 11; font.bold: true
                        }
                    }

                    // --- tilt <= 5: anh 2D ---
                    Item {
                        visible: !droneIcon.view3d
                        width: droneIcon.box * 0.75; height: width
                        x: (droneIcon.box - width) / 2; y: (droneIcon.box - height) / 2
                        transformOrigin: Item.Center
                        rotation: droneIcon.heading - map.bearing
                        Image {                              // halo ve san trong PNG (add_halo.py)
                            anchors.fill: parent
                            source: appInfo.imagesUrl + "drone_top.png"
                            smooth: true; mipmap: true
                        }
                        Canvas {                             // chevron huong mui
                            width: 18; height: 12
                            anchors.horizontalCenter: parent.horizontalCenter
                            y: -10
                            onPaint: {
                                var ctx = getContext("2d")
                                ctx.reset()
                                ctx.beginPath()
                                ctx.moveTo(9, 0); ctx.lineTo(17, 11); ctx.lineTo(9, 7); ctx.lineTo(1, 11)
                                ctx.closePath()
                                ctx.fillStyle = "#2ea043"; ctx.fill()
                                ctx.lineWidth = 1.5; ctx.strokeStyle = "white"; ctx.stroke()
                            }
                        }
                    }

                    // --- tilt > 5: mo hinh 3D ---
                    View3D {
                        visible: droneIcon.view3d
                        width: droneIcon.box; height: droneIcon.box
                        camera: iconCam
                        environment: SceneEnvironment {
                            backgroundMode: SceneEnvironment.Transparent
                            antialiasingMode: SceneEnvironment.MSAA
                            antialiasingQuality: SceneEnvironment.High
                        }
                        // camera: goc ngang (90 - tilt) do, nhin ve phia tren man hinh (= huong bearing)
                        OrthographicCamera {
                            id: iconCam
                            readonly property real elev: 90 - map.tilt
                            position: Qt.vector3d(0, 400 * Math.sin(elev * Math.PI / 180), 400 * Math.cos(elev * Math.PI / 180))
                            eulerRotation.x: -elev
                            horizontalMagnification: droneIcon.box / 95; verticalMagnification: droneIcon.box / 95
                            clipNear: 1; clipFar: 2000
                        }
                        DirectionalLight { eulerRotation.x: -60; eulerRotation.y: 25; brightness: 1.4 }
                        DirectionalLight { eulerRotation.x: -20; eulerRotation.y: 200; brightness: 0.5 }
                        // scene: X = Dong, Y = len, -Z = Bac; yaw = heading - bearing
                        Node {
                            eulerRotation.y: 90 - (droneIcon.heading - map.bearing)
                            Node {
                                eulerRotation.z: droneIcon.att.valid ? droneIcon.att.pitch : 0    // pitch > 0: mui len
                                Node {
                                    eulerRotation.x: droneIcon.att.valid ? droneIcon.att.roll : 0  // roll > 0: canh phai xuong
                                    EiuDrone { scale: Qt.vector3d(100, 100, 100) }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
