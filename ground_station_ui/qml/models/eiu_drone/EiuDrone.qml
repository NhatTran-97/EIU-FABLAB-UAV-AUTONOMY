import QtQuick
import QtQuick3D

Node {
    id: node

    // Resources
    PrincipledMaterial {
        id: cream_material
        objectName: "cream"
        baseColor: "#ffefead4"
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: white_001_material
        objectName: "white.001"
        baseColor: "#fff6f6f3"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: silver_material
        objectName: "silver"
        baseColor: "#ffdcdcdf"
        metalness: 0.699999988079071
        roughness: 0.3499999940395355
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: red_material
        objectName: "red"
        baseColor: "#ffe05045"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: mount_blue_material
        objectName: "mount_blue"
        baseColor: "#ff7bcef1"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: motor_blue_001_material
        objectName: "motor_blue.001"
        baseColor: "#ff5090e0"
        metalness: 0.800000011920929
        roughness: 0.30000001192092896
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: glass_001_material
        objectName: "glass.001"
        baseColor: "#ff273855"
        metalness: 0.8999999761581421
        roughness: 0.05000000074505806
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: copper_001_material
        objectName: "copper.001"
        baseColor: "#ffdcad7b"
        metalness: 0.800000011920929
        roughness: 0.3499999940395355
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: carbon_material
        objectName: "carbon"
        baseColor: "#ff3f3f42"
        metalness: 0.3499999940395355
        roughness: 0.3499999940395355
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: c12_orange_001_material
        objectName: "c12_orange.001"
        baseColor: "#fff9a938"
        roughness: 0.44999998807907104
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: c12_body_001_material
        objectName: "c12_body.001"
        baseColor: "#ff525255"
        metalness: 0.10000000149011612
        roughness: 0.6000000238418579
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: brown_material
        objectName: "brown"
        baseColor: "#ff906c55"
        roughness: 0.699999988079071
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }
    PrincipledMaterial {
        id: black_material
        objectName: "black"
        baseColor: "#ff272727"
        roughness: 0.5
        cullMode: PrincipledMaterial.NoCulling
        alphaMode: PrincipledMaterial.Opaque
    }

    // Nodes:
    Node {
        id: root
        objectName: "ROOT"
        Model {
            id: black
            objectName: "black"
            position: Qt.vector3d(0.015, 0.0892, 0)
            source: "meshes/cylinder_004_mesh.mesh"
            materials: [
                black_material
            ]
        }
        Model {
            id: brown
            objectName: "brown"
            position: Qt.vector3d(0, 0.0462, 0)
            source: "meshes/cube_003_mesh.mesh"
            materials: [
                brown_material
            ]
        }
        Model {
            id: c12_body_001
            objectName: "c12_body.001"
            position: Qt.vector3d(0.035, -0.1098, 0)
            source: "meshes/part_009_001_mesh.mesh"
            materials: [
                c12_body_001_material
            ]
        }
        Model {
            id: c12_orange_001
            objectName: "c12_orange.001"
            position: Qt.vector3d(0.035, -0.1098, 0)
            source: "meshes/c12_bezel_mesh.mesh"
            materials: [
                c12_orange_001_material
            ]
        }
        Model {
            id: carbon
            objectName: "carbon"
            position: Qt.vector3d(0, -0.0303, 0)
            source: "meshes/cube_mesh.mesh"
            materials: [
                carbon_material
            ]
        }
        Model {
            id: copper_001
            objectName: "copper.001"
            position: Qt.vector3d(0.282843, -0.0268, -0.282843)
            source: "meshes/part_004_002_mesh.mesh"
            materials: [
                copper_001_material
            ]
        }
        Model {
            id: cream
            objectName: "cream"
            position: Qt.vector3d(0, -0.0113, 0)
            source: "meshes/cube_002_mesh.mesh"
            materials: [
                cream_material
            ]
        }
        Model {
            id: glass_001
            objectName: "glass.001"
            position: Qt.vector3d(0.035, -0.1098, 0)
            source: "meshes/c12_glass_mesh.mesh"
            materials: [
                glass_001_material
            ]
        }
        Model {
            id: motor_blue_001
            objectName: "motor_blue.001"
            position: Qt.vector3d(0.282843, -0.0268, -0.282843)
            source: "meshes/part_001_002_mesh.mesh"
            materials: [
                motor_blue_001_material
            ]
        }
        Model {
            id: mount_blue
            objectName: "mount_blue"
            position: Qt.vector3d(0.01, -0.0698, 0)
            source: "meshes/cube_007_mesh.mesh"
            materials: [
                mount_blue_material
            ]
        }
        Model {
            id: red
            objectName: "red"
            position: Qt.vector3d(-0.055, -0.0418, 0)
            source: "meshes/cube_004_mesh.mesh"
            materials: [
                red_material
            ]
        }
        Model {
            id: silver
            objectName: "silver"
            position: Qt.vector3d(0, -0.0948, 0)
            source: "meshes/cube_008_mesh.mesh"
            materials: [
                silver_material
            ]
        }
        Model {
            id: white_001
            objectName: "white.001"
            position: Qt.vector3d(-0.08, 0.1162, 0)
            source: "meshes/part_002_mesh.mesh"
            materials: [
                white_001_material
            ]
        }
    }

    // Animations:
}
