#!/usr/bin/env bash
# Sinh lai drone_telemetry/drone_dialect.py tu mavlink/drone_telemetry.xml.
#
# Cach dung:
#   ./gen_dialect.sh [thu muc message_definitions/v1.0 chua common.xml]
# Bo trong tham so -> tai common.xml/standard.xml/minimal.xml tu repo mavlink tren GitHub.
# Can pymavlink luc sinh code; file sinh ra khong phu thuoc pymavlink.
set -euo pipefail

PKG_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEFS_DIR="${1:-}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

for f in common.xml standard.xml minimal.xml; do
    if [[ -n "$DEFS_DIR" ]]; then
        cp "$DEFS_DIR/$f" "$WORK/"
    else
        curl -fsSL "https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/$f" -o "$WORK/$f"
    fi
done
cp "$PKG_DIR/mavlink/drone_telemetry.xml" "$WORK/"

python3 - "$WORK/drone_telemetry.xml" "$PKG_DIR/drone_telemetry/drone_dialect" <<'EOF'
import sys
from pymavlink.generator import mavgen
opts = mavgen.Opts(sys.argv[2], wire_protocol="2.0", language="Python3", validate=False)  # schema cua pymavlink co the cu hon common.xml
if not mavgen.mavgen(opts, [sys.argv[1]]):
    sys.exit("mavgen failed")
EOF
echo "Da sinh $PKG_DIR/drone_telemetry/drone_dialect.py"
