import QtQuick
import QtQuick3D

// Truc toa do kieu RViz: x do, y xanh la, z xanh duong (theo quy uoc ROS ENU / FLU).
// Doi sang Qt Quick 3D (Y len): x -> +X, y -> -Z, z -> +Y  (xem qt/geometry3d.py).
Node {
    id: axes
    property real length: 100          // don vi scene (100 = 1 m)
    property real thickness: 3

    component Bar: Model {
        source: "#Cylinder"
        scale: Qt.vector3d(axes.thickness / 100, axes.length / 100, axes.thickness / 100)
        property color tint
        materials: PrincipledMaterial { baseColor: tint; lighting: PrincipledMaterial.NoLighting }
    }
    Bar { tint: "#ff3b30"; eulerRotation.z: -90; position: Qt.vector3d(axes.length / 2, 0, 0) }   // x
    Bar { tint: "#34c759"; eulerRotation.x: -90; position: Qt.vector3d(0, 0, -axes.length / 2) }  // y
    Bar { tint: "#0a84ff"; position: Qt.vector3d(0, axes.length / 2, 0) }                        // z
}
