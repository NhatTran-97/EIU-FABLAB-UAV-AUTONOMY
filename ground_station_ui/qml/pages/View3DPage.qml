import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic
import QtQuick3D
import QtQuick3D.Helpers
import DroneGcs3D
import "../components"

// Khung nhin 3D cuc bo (thay RViz): luoi san, truc map, drone + base_link, vet bay.
// Pose: Jetson doc PX4 /fmu/out/vehicle_odometry (DDS) -> DRONE_LOCAL_POSE qua radio 433.
// Du lieu app la ENU (met); doi sang scene Qt Quick 3D (Y len) chi o scenePos / sceneRot.
Item {
    id: page
    readonly property var pose: vehicle.localPose.values
    readonly property bool hasPose: pose.valid === true
    property string camMode: "follow"        // follow | free | top
    readonly property bool follow: camMode !== "free"
    readonly property var att: vehicle.attitude.values
    function setCam(m) {
        camMode = m
        if (m === "top") { orbit.eulerRotation = Qt.vector3d(-89.9, 0, 0); orbit.position = Qt.vector3d(0, 0, 0); camera.z = 1200 }
        else if (m === "follow") { orbit.eulerRotation = Qt.vector3d(-30, -35, 0); orbit.position = Qt.vector3d(0, 0, 0); camera.z = 450 }
    }

    function scenePos(e, n, u) { return Qt.vector3d(e * 100, u * 100, -n * 100) }
    function sceneRot(q) { return Qt.quaternion(q[0], q[1], q[3], -q[2]) }   // (w,x,y,z) ENU -> scene

    Rectangle { anchors.fill: parent; radius: Theme.radius; color: "#0b0f14"; clip: true

        View3D {
            id: view
            anchors.fill: parent
            environment: SceneEnvironment {
                clearColor: "#0b0f14"
                backgroundMode: SceneEnvironment.Color
                antialiasingMode: SceneEnvironment.MSAA
                antialiasingQuality: SceneEnvironment.High
            }

            DirectionalLight { eulerRotation.x: -45; eulerRotation.y: 30; brightness: 1.2 }

            // ---- camera quy dao: keo chuot trai xoay, cuon = zoom, keo chuot phai di chuyen ----
            // target bam theo drone; orbit (con) do OrbitCameraController xoay / keo
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

            // ---- san + truc map (goc EKF cua PX4) ----
            Model {
                geometry: LineGrid { sizeM: 40; stepM: 1 }
                materials: PrincipledMaterial { baseColor: "#2a3240"; lighting: PrincipledMaterial.NoLighting }
            }
            Axes3D { length: 100; thickness: 2.5 }

            // ---- vet bay ----
            Model {
                visible: (vehicle.localPose.items || []).length > 1
                geometry: Polyline { points: vehicle.localPose.items }
                materials: PrincipledMaterial { baseColor: "#ffcc00"; lighting: PrincipledMaterial.NoLighting }
            }

            // ---- drone (base_link): than + 4 tay + mui do + truc FLU ----
            Node {
                id: drone
                visible: page.hasPose
                position: page.scenePos(page.pose.e || 0, page.pose.n || 0, page.pose.u || 0)
                rotation: page.sceneRot(page.pose.q || [1, 0, 0, 0])
                // pose ~5 Hz qua radio -> noi suy cho muot
                Behavior on position { Vector3dAnimation { duration: 200 } }

                Model {
                    source: "#Cube"; scale: Qt.vector3d(0.16, 0.05, 0.16)
                    materials: PrincipledMaterial { baseColor: "#8b949e"; roughness: 0.6 }
                }
                Repeater3D {
                    model: [45, 135, 225, 315]
                    delegate: Node {
                        required property var modelData
                        eulerRotation.y: modelData
                        Model {   // tay
                            source: "#Cube"; x: 18; scale: Qt.vector3d(0.36, 0.02, 0.03)
                            materials: PrincipledMaterial { baseColor: "#57606a" }
                        }
                        Model {   // vong canh quat
                            source: "#Cylinder"; x: 36; y: 3; scale: Qt.vector3d(0.24, 0.01, 0.24)
                            materials: PrincipledMaterial { baseColor: "#30363d"; opacity: 0.85 }
                        }
                    }
                }
                Model {   // mui (huong bay) mau do
                    source: "#Cone"; x: 16; eulerRotation.z: -90; scale: Qt.vector3d(0.06, 0.1, 0.06)
                    materials: PrincipledMaterial { baseColor: "#ff453a" }
                }
                Axes3D { length: 60; thickness: 2 }
            }
        }

        OrbitCameraController { anchors.fill: parent; origin: orbit; camera: camera }

        // ---- bang thong so (trai): Position / Orientation / Velocity ----
        Rectangle {
            anchors.left: parent.left; anchors.top: parent.top; anchors.margins: 12
            width: 250; height: info.implicitHeight + 24
            radius: Theme.radius; color: "#e6161b22"; border.color: Theme.border
            ColumnLayout {
                id: info
                anchors.fill: parent; anchors.margins: 12
                spacing: 6
                Text {
                    text: page.hasPose ? "● Pose  ·  Jetson → radio 433" : "● No pose yet (Jetson / PX4 DDS?)"
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
                    text: "Trail: " + (vehicle.localPose.items || []).length + " points"
                    color: Theme.textDim; font.pixelSize: 12
                }
                Text {
                    Layout.fillWidth: true; wrapMode: Text.WordWrap
                    text: "Axes: red X · green Y · blue Z (ENU, like RViz)"
                    color: Theme.textDim; font.pixelSize: 11
                }
            }
        }

        // ---- cong cu goc tren phai ----
        Row {
            anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 12
            spacing: 8
            // che do camera
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
            Button { text: "Clear trail"; onClicked: vehicle.clearTrail() }
            Button {
                text: "Reset view"
                onClicked: page.setCam(page.camMode === "top" ? "top" : "follow")
            }
        }

        Text {
            anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 10
            text: "Grid: 1 cell = 1 m  ·  ground Z = 0 (EKF origin)"
            color: Theme.textDim; font.pixelSize: 12
        }
        Text {
            anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.margins: 10
            text: "Left drag: rotate · wheel: zoom · right / middle drag: pan"
            color: Theme.textDim; font.pixelSize: 12
        }
    }
}
