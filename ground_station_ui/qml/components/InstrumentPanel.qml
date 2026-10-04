import QtQuick
import QtQuick.Layouts

// Khung dong ho goc tren phai ban do (giong QGC): tu the + la ban + do cao / toc do.
Rectangle {
    id: panel
    readonly property var att: vehicle.attitude.values
    readonly property var fl: vehicle.flight.values
    readonly property bool hasData: att.valid === true

    implicitWidth: col.implicitWidth + 16
    implicitHeight: col.implicitHeight + 16
    radius: Theme.radius
    color: "#cc0d1117"
    border.color: Theme.border

    function num(v, unit, digits) {
        return (v === undefined || v === null) ? "—" : Number(v).toFixed(digits) + " " + unit
    }

    ColumnLayout {
        id: col
        anchors.centerIn: parent
        spacing: 8

        RowLayout {
            spacing: 8
            AttitudeIndicator {
                objectName: "attitudeIndicator"
                roll: panel.att.roll || 0
                pitch: panel.att.pitch || 0
                valid: panel.hasData
            }
            CompassDial {
                objectName: "compassDial"
                heading: panel.att.heading || 0
                valid: panel.hasData
            }
        }

        // so lieu: do cao lon nhat (quan trong nhat khi bay trong nha), toc do nho hon
        GridLayout {
            Layout.fillWidth: true
            columns: 3
            columnSpacing: 6; rowSpacing: 0

            component Value: ColumnLayout {
                property string label: ""
                property string value: ""
                property bool big: false
                spacing: 0
                Layout.fillWidth: true
                Text { text: parent.label; color: Theme.textDim; font.pixelSize: 11; Layout.alignment: Qt.AlignHCenter }
                Text {
                    text: parent.value; color: Theme.text
                    font.pixelSize: parent.big ? 20 : 14; font.bold: true
                    Layout.alignment: Qt.AlignHCenter
                }
            }
            Value {
                objectName: "altValue"
                // nguon: cam bien khoang cach (trong nha) / so voi Home (ngoai troi)
                label: panel.fl.altSource === "local" ? "Altitude · sensor" : "Altitude · Home"
                value: panel.num(panel.fl.altRel, "m", 1); big: true
            }
            Value { label: "Climb"; value: panel.num(panel.fl.climbRate, "m/s", 1) }
            Value { label: "Speed"; value: panel.num(panel.fl.groundSpeed, "m/s", 1) }
        }
    }
}
