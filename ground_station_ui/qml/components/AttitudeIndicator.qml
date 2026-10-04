import QtQuick

// Dong ho tu the (chan troi nhan tao): roll xoay chan troi, pitch day len / xuong.
// Du lieu: vehicle.attitude (ATTITUDE cua PX4). Animation ngan cho muot khi telemetry ~5 Hz.
Item {
    id: root
    property real roll: 0          // do, duong = nghieng phai
    property real pitch: 0         // do, duong = chui mui len
    property bool valid: false
    property real pxPerDeg: canvas.width / 70     // ~ +/-35 do thay duoc

    width: 140; height: 140 + 22     // mat dong ho vuong + 1 dong chu ben duoi

    Behavior on roll { NumberAnimation { duration: 180 } }
    Behavior on pitch { NumberAnimation { duration: 180 } }
    onRollChanged: canvas.requestPaint()
    onPitchChanged: canvas.requestPaint()
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
            ctx.beginPath(); ctx.arc(0, 0, r, 0, 2 * Math.PI); ctx.clip()

            // --- bau troi / mat dat: xoay theo roll, dich theo pitch ---
            ctx.save()
            ctx.rotate(-root.roll * Math.PI / 180)
            ctx.translate(0, root.pitch * root.pxPerDeg)
            ctx.fillStyle = root.valid ? "#2f80d1" : "#3a4250"; ctx.fillRect(-2 * r, -4 * r, 4 * r, 4 * r)
            ctx.fillStyle = root.valid ? "#7a5230" : "#2a3038"; ctx.fillRect(-2 * r, 0, 4 * r, 4 * r)
            ctx.strokeStyle = "white"; ctx.lineWidth = 2
            ctx.beginPath(); ctx.moveTo(-2 * r, 0); ctx.lineTo(2 * r, 0); ctx.stroke()
            // thang pitch moi 10 do
            ctx.lineWidth = 1.2; ctx.fillStyle = "white"
            ctx.font = Math.round(r * 0.14) + "px sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "middle"
            for (var p = -30; p <= 30; p += 10) {
                if (p === 0) continue
                var y = -p * root.pxPerDeg, half = r * 0.22
                ctx.beginPath(); ctx.moveTo(-half, y); ctx.lineTo(half, y); ctx.stroke()
                ctx.fillText(Math.abs(p), half + 4, y)
            }
            ctx.restore()

            // --- vach roll + kim chi roll (co dinh / xoay) ---
            ctx.strokeStyle = "white"; ctx.lineWidth = 1.5
            var ticks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]
            for (var i = 0; i < ticks.length; i++) {
                var a = (ticks[i] - 90) * Math.PI / 180, len = (ticks[i] % 30 === 0) ? r * 0.12 : r * 0.07
                ctx.beginPath()
                ctx.moveTo(Math.cos(a) * r, Math.sin(a) * r)
                ctx.lineTo(Math.cos(a) * (r - len), Math.sin(a) * (r - len))
                ctx.stroke()
            }
            ctx.save()
            ctx.rotate(-root.roll * Math.PI / 180)
            ctx.fillStyle = "#ffcc00"
            ctx.beginPath(); ctx.moveTo(0, -r + r * 0.14); ctx.lineTo(-r * 0.06, -r + r * 0.25); ctx.lineTo(r * 0.06, -r + r * 0.25); ctx.closePath(); ctx.fill()
            ctx.restore()

            // --- ky hieu may bay (co dinh) ---
            ctx.strokeStyle = "#ffcc00"; ctx.lineWidth = 3; ctx.lineCap = "round"
            ctx.beginPath(); ctx.moveTo(-r * 0.5, 0); ctx.lineTo(-r * 0.16, 0); ctx.lineTo(0, r * 0.1)
            ctx.lineTo(r * 0.16, 0); ctx.lineTo(r * 0.5, 0); ctx.stroke()
            ctx.fillStyle = "#ffcc00"; ctx.beginPath(); ctx.arc(0, 0, 2.5, 0, 2 * Math.PI); ctx.fill()
            ctx.restore()

            // vien
            ctx.strokeStyle = "#e6edf3"; ctx.lineWidth = 2
            ctx.beginPath(); ctx.arc(w / 2, h / 2, r, 0, 2 * Math.PI); ctx.stroke()
        }
    }

    Text {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        text: root.valid ? "Roll " + root.roll.toFixed(0) + "°   Pitch " + root.pitch.toFixed(0) + "°" : "—"
        color: "#e6edf3"; font.pixelSize: 12; font.bold: true
    }
}
