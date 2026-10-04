import QtQuick

// La ban: mat so xoay theo heading, mui drone luon chi len tren (giong QGC).
Item {
    id: root
    property real heading: 0       // do, 0-360
    property bool valid: false
    // heading "mo cuon" de animation di duong ngan nhat qua 0/360
    property real shown: 0

    width: 140; height: 140 + 22     // mat dong ho vuong + 1 dong chu ben duoi

    onHeadingChanged: {
        var cur = ((shown % 360) + 360) % 360
        var d = ((heading - cur + 540) % 360) - 180
        shown = shown + d
    }
    Behavior on shown { NumberAnimation { duration: 180 } }
    onShownChanged: canvas.requestPaint()
    onValidChanged: canvas.requestPaint()

    Canvas {
        id: canvas
        anchors.top: parent.top; anchors.horizontalCenter: parent.horizontalCenter
        width: root.width; height: root.width
        antialiasing: true
        onPaint: {
            var ctx = getContext("2d")
            var w = width, h = height, r = Math.min(w, h) / 2 - 2
            ctx.reset()
            ctx.save()
            ctx.translate(w / 2, h / 2)
            ctx.fillStyle = "#161b22"
            ctx.beginPath(); ctx.arc(0, 0, r, 0, 2 * Math.PI); ctx.fill()

            // mat so xoay nguoc heading
            ctx.save()
            ctx.rotate(-root.shown * Math.PI / 180)
            ctx.textAlign = "center"; ctx.textBaseline = "middle"
            for (var deg = 0; deg < 360; deg += 10) {
                var a = (deg - 90) * Math.PI / 180, major = deg % 30 === 0
                ctx.strokeStyle = major ? "#e6edf3" : "#8b949e"; ctx.lineWidth = major ? 1.6 : 1
                var len = major ? r * 0.12 : r * 0.07
                ctx.beginPath()
                ctx.moveTo(Math.cos(a) * r, Math.sin(a) * r)
                ctx.lineTo(Math.cos(a) * (r - len), Math.sin(a) * (r - len))
                ctx.stroke()
            }
            var labels = { 0: "N", 90: "E", 180: "S", 270: "W" }
            for (var k in labels) {
                var ang = (Number(k) - 90) * Math.PI / 180, rr = r * 0.72
                ctx.save()
                ctx.translate(Math.cos(ang) * rr, Math.sin(ang) * rr)
                ctx.rotate(Number(k) * Math.PI / 180)       // chu dung theo mat so
                ctx.fillStyle = k === "0" ? "#ff453a" : "#e6edf3"
                ctx.font = "bold " + Math.round(r * 0.2) + "px sans-serif"
                ctx.fillText(labels[k], 0, 0)
                ctx.restore()
            }
            ctx.restore()

            // drone co dinh, mui chi len
            ctx.fillStyle = root.valid ? "#ff453a" : "#6e7681"
            ctx.beginPath()
            ctx.moveTo(0, -r * 0.42); ctx.lineTo(r * 0.2, r * 0.3); ctx.lineTo(0, r * 0.16); ctx.lineTo(-r * 0.2, r * 0.3)
            ctx.closePath(); ctx.fill()
            ctx.restore()

            ctx.strokeStyle = "#e6edf3"; ctx.lineWidth = 2
            ctx.beginPath(); ctx.arc(w / 2, h / 2, r, 0, 2 * Math.PI); ctx.stroke()
            // vach chi huong co dinh o dinh
            ctx.fillStyle = "#ffcc00"
            ctx.beginPath(); ctx.moveTo(w / 2, 2); ctx.lineTo(w / 2 - 6, 12); ctx.lineTo(w / 2 + 6, 12); ctx.closePath(); ctx.fill()
        }
    }

    Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        width: hdgText.implicitWidth + 10; height: hdgText.implicitHeight + 2; radius: 3
        color: "transparent"; border.color: "#e6edf3"
        Text {
            id: hdgText
            anchors.centerIn: parent
            text: root.valid ? ("00" + Math.round(root.heading) % 360).slice(-3) + "°" : "—"
            color: "white"; font.pixelSize: 13; font.bold: true; font.family: "monospace"
        }
    }
}
