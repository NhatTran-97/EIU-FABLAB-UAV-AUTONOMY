import QtCore
import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import QtQuick3D
import QtQuick3D.Helpers
import DroneGcs3D
import "../components"
import "../models/eiu_drone"

// Local 3D view: ground grid, map axes, drone (base_link), flight trail, planned path (last Send WP).
// Pose: vehicle.localPose (Jetson DRONE_LOCAL_POSE, PX4 LOCAL_POSITION_NED as fallback).
// Data is ENU (m); converted to the scene (Y up) only in scenePos / sceneRot.
Item {
    id: page
    readonly property var pose: vehicle.localPose.values
    readonly property bool hasPose: pose.valid === true
    property string camMode: "follow"        // follow | free | top
    property bool showPlanned: true          // planned path from the last Send WP (remembered)
    Settings { category: "view3d"; property alias showPlanned: page.showPlanned }
    readonly property bool follow: camMode !== "free"
    readonly property var att: vehicle.attitude.values
    // Ground grid sized to the flight area: side = 2 x (largest distance reached from the origin + margin),
    // rounded up to a readable size; cell size chosen so the grid has at most ~40 cells per side.
    readonly property real reach: Math.max(Math.abs(pose.e || 0), Math.abs(pose.n || 0), pose.extentM || 0,
                                           showPlanned ? plannedPath.reach : 0)
    readonly property real gridSize: {
        var need = 2 * (reach + Math.max(5, 0.15 * reach))
        var sizes = [40, 60, 100, 150, 200, 300, 400, 600, 1000, 1500, 2000]
        for (var i = 0; i < sizes.length; i++) if (sizes[i] >= need) return sizes[i]
        return Math.ceil(need / 1000) * 1000
    }
    readonly property real gridStep: {
        var steps = [1, 2, 5, 10, 20, 50, 100]
        for (var i = 0; i < steps.length; i++) if (gridSize / steps[i] <= 40) return steps[i]
        return 100
    }
    // Visibility scaling: beyond VIS_DIST the drone model grows with its distance to the camera, so it keeps
    // the on-screen size it has at VIS_DIST (~7 % of the view height); closer it is drawn at real size.
    readonly property real visDist: 1200                      // scene units (12 m)
    readonly property real camDist: camera.scenePosition.minus(drone.scenePosition).length()
    readonly property real droneScale: Math.max(1, camDist / visDist)
    // line widths follow the zoom in powers of two, so the ribbons are rebuilt only at each doubling
    readonly property real lineScale: Math.pow(2, Math.max(0, Math.round(Math.log(Math.max(1, camera.z / visDist)) / Math.LN2)))

    // camera distance (scene units) that shows the whole grid from above (60 deg field of view)
    readonly property real topDistance: gridSize * 100 * 0.95
    onTopDistanceChanged: if (camMode === "top") camera.z = topDistance

    function setCam(m) {
        camMode = m
        if (m === "top") { orbit.eulerRotation = Qt.vector3d(-89.9, 0, 0); orbit.position = Qt.vector3d(0, 0, 0); camera.z = topDistance }
        else if (m === "follow") { orbit.eulerRotation = Qt.vector3d(-30, -35, 0); orbit.position = Qt.vector3d(0, 0, 0); camera.z = 450 }
    }

    function scenePos(e, n, u) { return Qt.vector3d(e * 100, u * 100, -n * 100) }
    function sceneRot(q) { return Qt.quaternion(q[0], q[1], q[3], -q[2]) }   // (w,x,y,z) ENU -> scene

    Rectangle { anchors.fill: parent; radius: Theme.radius; color: "#0b0f14"; clip: true

        View3D {
            id: view
            anchors.fill: parent
            camera: camera
            environment: SceneEnvironment {
                clearColor: "#0b0f14"
                backgroundMode: SceneEnvironment.Color
                antialiasingMode: SceneEnvironment.MSAA
                antialiasingQuality: SceneEnvironment.High
            }

            DirectionalLight { eulerRotation.x: -45; eulerRotation.y: 30; brightness: 1.2 }

            // ---- orbit camera: left drag = rotate, wheel = zoom, right drag = pan ----
            // target follows the drone; the child orbit node is driven by OrbitCameraController
            Node {
                id: target
                position: page.follow && page.hasPose ? drone.position : Qt.vector3d(0, 0, 0)
                Node {
                    id: orbit
                    eulerRotation.x: -30
                    eulerRotation.y: -35
                    PerspectiveCamera { id: camera; z: 450; clipNear: 1; clipFar: 100000 }
                }
            }

            // ---- ground + map axes (PX4 EKF origin) ----
            Model {
                geometry: LineGrid { sizeM: page.gridSize; stepM: page.gridStep }
                materials: PrincipledMaterial { baseColor: "#2a3240"; lighting: PrincipledMaterial.NoLighting }
            }
            Axes3D { length: 100; thickness: 2.5 }

            // ---- flight trail ----
            Model {
                visible: vehicle.localPose.values.trailPoints > 1
                geometry: Polyline { source: vehicle.trail3d; widthM: 0.05 * page.lineScale }   // 5 cm at close range
                materials: PrincipledMaterial {
                    baseColor: "#2f9bff"; lighting: PrincipledMaterial.NoLighting
                    cullMode: PrincipledMaterial.NoCulling
                }
            }

            // ---- planned path: last uploaded waypoints, anchored on the drone GPS + local pose ----
            Model {
                visible: page.showPlanned && plannedPath.valid
                geometry: Polyline { points: plannedPath.points; widthM: 0.06 * page.lineScale }
                materials: PrincipledMaterial {
                    baseColor: Theme.accent; lighting: PrincipledMaterial.NoLighting
                    cullMode: PrincipledMaterial.NoCulling
                }
            }
            Repeater3D {
                model: page.showPlanned ? plannedPath.markers : []
                delegate: Node {
                    id: wpNode
                    required property var modelData
                    readonly property color tint: modelData.task !== "none" ? Theme.ok : Theme.accent
                    // same visibility scaling as the drone, from this marker's own distance to the camera
                    readonly property real vis: Math.max(1, camera.scenePosition.minus(scenePosition).length() / page.visDist)
                    position: page.scenePos(modelData.e, modelData.n, modelData.u)
                    // waypoint: 0.25 m sphere at close range (#Sphere is 100 units = 1 m across)
                    Model {
                        source: "#Sphere"; scale: Qt.vector3d(0.25 * wpNode.vis, 0.25 * wpNode.vis, 0.25 * wpNode.vis)
                        materials: PrincipledMaterial { baseColor: wpNode.tint; lighting: PrincipledMaterial.NoLighting }
                    }
                    // drop line to the ground (u = 0): #Cylinder is 100 units tall, centred
                    Model {
                        source: "#Cylinder"
                        y: -wpNode.modelData.u * 50
                        scale: Qt.vector3d(0.02 * wpNode.vis, Math.max(0.01, wpNode.modelData.u), 0.02 * wpNode.vis)
                        materials: PrincipledMaterial { baseColor: wpNode.tint; lighting: PrincipledMaterial.NoLighting; opacity: 0.5 }
                    }
                }
            }

            // ---- drone (base_link): model + FLU axes ----
            Node {
                id: drone
                visible: page.hasPose
                position: page.scenePos(page.pose.e || 0, page.pose.n || 0, page.pose.u || 0)
                rotation: page.sceneRot(page.pose.q || [1, 0, 0, 0])
                // pose at ~5 Hz -> interpolated
                Behavior on position { Vector3dAnimation { duration: 200 } }

                // EiuDrone: tools/drone_model/build.sh (Blender -> glTF -> balsam).
                // base_link frame (FLU, origin = PX4, nose = +X), metres -> scale 100.
                Node {
                    scale: Qt.vector3d(page.droneScale, page.droneScale, page.droneScale)   // visibility scaling
                    EiuDrone { scale: Qt.vector3d(100, 100, 100) }
                    Axes3D { length: 60; thickness: 2 }
                }
            }
        }

        OrbitCameraController { anchors.fill: parent; origin: orbit; camera: camera }

        // ---- info panel (left): Position / Orientation / Velocity ----
        Rectangle {
            anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 12
            width: 250; height: info.implicitHeight + 24
            radius: Theme.radius; color: "#e6161b22"; border.color: Theme.border
            ColumnLayout {
                id: info
                anchors.fill: parent; anchors.margins: 12
                spacing: 6
                Text {
                    // pose source: jetson | px4
                    text: !page.hasPose ? "● No pose yet (Jetson / PX4 link?)"
                        : page.pose.source === "jetson" ? "● Pose  ·  Jetson (DDS) → radio 433"
                        : "● Pose  ·  PX4 direct (" + (vehicle.status.values.source || "MAVLink") + ")"
                    color: page.hasPose ? Theme.ok : Theme.warn; font.pixelSize: 12; font.bold: true
                }
                component Block: ColumnLayout {
                    property string title
                    property var labels: []
                    property var values: []
                    property string unit
                    spacing: 2
                    Text { text: parent.title; color: Theme.textDim; font.pixelSize: 12; font.bold: true }
                    GridLayout {
                        columns: 3; columnSpacing: 10; rowSpacing: 0
                        Repeater {
                            model: 3
                            delegate: ColumnLayout {
                                required property int index
                                spacing: 0
                                Text { text: labels[index]; color: Theme.textDim; font.pixelSize: 11 }
                                Text {
                                    text: values[index] === undefined || values[index] === null ? "—" : values[index] + unit
                                    color: Theme.text; font.pixelSize: 15; font.bold: true; font.family: "monospace"
                                }
                            }
                        }
                    }
                }
                Block {
                    title: "Position (m, ENU)"
                    labels: ["X / East", "Y / North", "Z / Up"]
                    values: page.hasPose ? [page.pose.e.toFixed(2), page.pose.n.toFixed(2), page.pose.u.toFixed(2)] : []
                }
                Block {
                    title: "Orientation"
                    labels: ["Roll", "Pitch", "Yaw"]; unit: "°"
                    values: page.att.valid ? [page.att.roll.toFixed(0), page.att.pitch.toFixed(0), page.att.heading.toFixed(0)] : []
                }
                Block {
                    title: "Velocity (m/s)"
                    labels: ["Vx / E", "Vy / N", "Vz / Up"]
                    values: page.hasPose ? [page.pose.ve.toFixed(2), page.pose.vn.toFixed(2), page.pose.vu.toFixed(2)] : []
                }
                Text {
                    text: "Trail: " + vehicle.localPose.values.trailPoints + " points"
                          + (page.droneScale > 1.05 ? "  ·  model ×" + page.droneScale.toFixed(1) + " (zoomed out)" : "")
                    color: Theme.textDim; font.pixelSize: 12
                }
                Text {
                    Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: "Axes: red X · green Y · blue Z (ENU, like RViz)"
                    color: Theme.textDim; font.pixelSize: 11
                }
            }
        }

        // ---- tools, top right ----
        Row {
            anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
            spacing: 8
            // camera mode
            Row {
                spacing: 0
                Repeater {
                    model: [{ id: "follow", title: "Follow" }, { id: "free", title: "Free" }, { id: "top", title: "Top" }]
                    delegate: Rectangle {
                        required property var modelData
                        objectName: "cam_" + modelData.id
                        width: 76; height: 36
                        color: page.camMode === modelData.id ? Theme.accent : Theme.panelAlt
                        border.color: Theme.border
                        Text { anchors.centerIn: parent; text: parent.modelData.title; color: Theme.text; font.pixelSize: 13; font.bold: true }
                        MouseArea { anchors.fill: parent; onClicked: page.setCam(parent.modelData.id) }
                    }
                }
            }
            CheckBox {
                objectName: "plannedBox"
                text: "Planned path"
                checked: page.showPlanned
                onToggled: page.showPlanned = checked
                ToolTip.visible: hovered
                ToolTip.text: plannedPath.valid ? "Waypoints from the last Send WP"
                                                : "Send WP first; needs a GPS position and the 3D pose"
            }
            Button { text: "Clear trail"; onClicked: vehicle.clearTrail() }
            Button {
                text: "Reset view"
                onClicked: page.setCam(page.camMode === "top" ? "top" : "follow")
            }
        }

        Text {
            anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 10
            text: "Grid: " + page.gridSize + " m, 1 cell = " + page.gridStep + " m  ·  ground Z = 0 (EKF origin)"
                  + "  ·  blue: flown trail · violet: planned path"
            color: Theme.textDim; font.pixelSize: 12
        }
        Text {
            anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.margins: 10
            text: "Left drag: rotate · wheel: zoom · right / middle drag: pan"
            color: Theme.textDim; font.pixelSize: 12
        }
    }
}
