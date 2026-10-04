pragma Singleton
import QtQuick

QtObject {
    readonly property color bg: "#0d1117"
    readonly property color panel: "#161b22"
    readonly property color panelAlt: "#1c2330"
    readonly property color border: "#2a3240"
    readonly property color text: "#e6edf3"
    readonly property color textDim: "#8b949e"
    readonly property color accent: "#5b5bf0"

    readonly property color ok: "#2ea043"
    readonly property color warn: "#d29922"
    readonly property color error: "#f85149"
    readonly property color idle: "#6e7681"

    readonly property int radius: 8
    readonly property int gap: 12

    // ok / warn / error / stale / off / none / unknown -> mau
    function statusColor(s) {
        if (s === "ok") return ok
        if (s === "warn") return warn
        if (s === "error") return error
        return idle
    }
}
