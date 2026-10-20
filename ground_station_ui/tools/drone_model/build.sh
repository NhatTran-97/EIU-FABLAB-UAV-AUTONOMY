#!/usr/bin/env bash
# Dung mo hinh drone: Blender (build_drone.py) -> .glb -> QML (pyside6-balsam).
#   ./tools/drone_model/build.sh            (chay tu thu muc ground_station_ui)
# Ket qua: qml/models/eiu_drone/EiuDrone.qml + meshes/ ; tools/drone_model/eiu_drone.blend (mo de chinh tay)
#          tools/drone_model/preview*.png; images/drone_top.png (icon ban do)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PKG="$(cd "$HERE/../.." && pwd)"
BLENDER="${BLENDER:-/snap/bin/blender}"      # can Blender >= 5.x
OUT="$PKG/qml/models/eiu_drone"
TMP="$(mktemp -d)"
# images/drone_top.png: icon drone tab Map
"$BLENDER" --background --python "$HERE/build_drone.py" -- "$TMP/eiu_drone.glb" "$HERE/preview.png" "$HERE/eiu_drone.blend" \
    "$PKG/images/drone_top.png" | grep -E "base_link|EXPORTED|PREVIEW|TOPVIEW|Error"
python3 "$HERE/add_halo.py" "$PKG/images/drone_top.png"     # halo (Pillow)
rm -rf "$OUT" && mkdir -p "$OUT"
pyside6-balsam -o "$TMP/qml" "$TMP/eiu_drone.glb"
cp -r "$TMP/qml/meshes" "$OUT/"
# balsam sinh Eiu_drone.qml -> doi ten EiuDrone.qml
cp "$TMP/qml/"*.qml "$OUT/EiuDrone.qml"
rm -rf "$TMP"
echo "OK -> $OUT"
