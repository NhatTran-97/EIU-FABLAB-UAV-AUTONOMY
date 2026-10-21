#!/usr/bin/env bash
# Regenerate drone_telemetry/drone_dialect.py from mavlink/drone_telemetry.xml.
#
# Usage:
#   ./gen_dialect.sh [message_definitions/v1.0 directory containing common.xml]
# Without an argument, common.xml / standard.xml / minimal.xml are downloaded from the mavlink GitHub repo.
# pymavlink is needed only for generation; the generated module does not depend on it.
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
opts = mavgen.Opts(sys.argv[2], wire_protocol="2.0", language="Python3", validate=False)  # pymavlink schema may be older than common.xml
if not mavgen.mavgen(opts, [sys.argv[1]]):
    sys.exit("mavgen failed")
EOF
echo "Generated $PKG_DIR/drone_telemetry/drone_dialect.py"
