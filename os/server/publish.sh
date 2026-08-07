#!/usr/bin/env bash
# Publish a RAUC bundle to a static update-server directory.
#
# Usage: ./publish.sh <bundle.raucb> <wwwdir>
#
# The device polls <base-url>/manifest.json; serve <wwwdir> with any
# static HTTPS server (nginx, caddy, python3 -m http.server for bench).
set -euo pipefail

BUNDLE="${1:?usage: publish.sh <bundle.raucb> <wwwdir>}"
WWWDIR="${2:?usage: publish.sh <bundle.raucb> <wwwdir>}"

NAME="$(basename "$BUNDLE")"
# openmower-<version>.raucb
VERSION="${NAME#openmower-}"
VERSION="${VERSION%.raucb}"
case "$VERSION" in
    ''|*[!0-9]*) echo "cannot derive numeric version from '$NAME'" >&2; exit 1 ;;
esac

SHA256="$(sha256sum "$BUNDLE" | cut -d' ' -f1)"

mkdir -p "$WWWDIR"
cp "$BUNDLE" "$WWWDIR/$NAME"

cat > "$WWWDIR/manifest.json.new" <<EOF
{
  "version": "$VERSION",
  "url": "$NAME",
  "sha256": "$SHA256"
}
EOF
mv "$WWWDIR/manifest.json.new" "$WWWDIR/manifest.json"

echo "published $NAME (version $VERSION) to $WWWDIR"
