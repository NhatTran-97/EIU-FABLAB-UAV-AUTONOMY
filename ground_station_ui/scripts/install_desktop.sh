#!/usr/bin/env bash
# Add an "EIU Drone GCS" launcher to the application menu (current user only, no sudo):
#   icon   -> ~/.local/share/icons/hicolor/<size>/apps/drone-gcs.png
#   desktop-> ~/.local/share/applications/drone-gcs.desktop
# Run it in the Python environment used for the app (e.g. conda activate drone_env), or set PYTHON=...
# Uninstall: ./install_desktop.sh --uninstall
set -euo pipefail

PKG_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ICONS="$HOME/.local/share/icons/hicolor"
DESKTOP="$HOME/.local/share/applications/drone-gcs.desktop"

if [[ "${1:-}" == "--uninstall" ]]; then
    rm -f "$DESKTOP"
    for s in 32 48 64 128 256 512; do rm -f "$ICONS/${s}x${s}/apps/drone-gcs.png"; done
    update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true
    echo "Removed the EIU Drone GCS launcher"; exit 0
fi

PYTHON="${PYTHON:-$(command -v python3 || command -v python)}"
for s in 32 48 64 128 256 512; do
    install -Dm644 "$PKG_DIR/images/icons/drone-gcs-$s.png" "$ICONS/${s}x${s}/apps/drone-gcs.png"
done
mkdir -p "$(dirname "$DESKTOP")"
cat > "$DESKTOP" <<DESK
[Desktop Entry]
Type=Application
Name=EIU Drone GCS
Comment=Ground Control & Autonomous Flight Platform
Exec=$PYTHON $PKG_DIR/ground_station_ui/main.py
Path=$PKG_DIR/ground_station_ui
Icon=drone-gcs
Terminal=false
Categories=Science;Engineering;
StartupWMClass=drone-gcs
DESK
gtk-update-icon-cache -f -t "$ICONS" 2>/dev/null || true
update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true
echo "Created launcher: $DESKTOP"
echo "  python: $PYTHON"
