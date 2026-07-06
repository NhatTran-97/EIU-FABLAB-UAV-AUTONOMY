#!/bin/bash
# Install & apply the ground-station LoRa / telemetry udev rules so the USB radios get
# STABLE names (/dev/lora_ground, /dev/lora_drone, /dev/lora_telem) regardless of
# the ttyUSB enumeration order. Run from anywhere:  ./install_lora_rules.sh
set -euo pipefail

RULES_NAME="99-lora-ground.rules"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="$SCRIPT_DIR/$RULES_NAME"
RULES_DST="/etc/udev/rules.d/$RULES_NAME"

if [[ ! -f "$RULES_SRC" ]]; then
    echo "❌ Không tìm thấy rule file: $RULES_SRC" >&2
    exit 1
fi

# Installing into /etc/udev needs root — re-run through sudo (sudo sets SUDO_USER for us).
if [[ $EUID -ne 0 ]]; then
    echo "ℹ️  Cần quyền root — chạy lại bằng sudo..."
    exec sudo "$0" "$@"
fi
INVOKING_USER="${SUDO_USER:-$USER}"

echo "ℹ️  Cài rule: $RULES_SRC"
echo "         -> $RULES_DST"
install -m 0644 "$RULES_SRC" "$RULES_DST"

echo "ℹ️  Reload + trigger udev (action=add)..."
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty
udevadm settle

echo ""
echo "=== Symlink kết quả ==="
found=0
for link in /dev/lora_ground /dev/lora_drone /dev/lora_telem; do
    if [[ -e "$link" ]]; then
        printf "  ✅ %-22s -> %s\n" "$link" "$(readlink -f "$link")"
        found=1
    else
        printf "  ⚠️  %-22s (chưa có — thiết bị chưa cắm?)\n" "$link"
    fi
done

# Rule đặt GROUP=dialout, MODE=0660 → user phải thuộc nhóm dialout mới mở được cổng.
if ! id -nG "$INVOKING_USER" 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
    echo ""
    echo "⚠️  User '$INVOKING_USER' CHƯA thuộc nhóm 'dialout'."
    echo "    Chạy:  sudo usermod -aG dialout $INVOKING_USER   (rồi đăng xuất/đăng nhập lại)"
fi

echo ""
if [[ $found -eq 1 ]]; then
    echo "✅ Xong. Nếu thiết bị vừa cắm mà symlink chưa hiện → rút/cắm lại cáp USB."
else
    echo "⚠️  Chưa thấy symlink nào — kiểm tra đã cắm radio chưa, rồi chạy lại script."
fi
