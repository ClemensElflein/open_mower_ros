# Update server

The device polls `$UPDATE_URL/manifest.json` (see
`/etc/openmower-updater.conf` on the device, overridable in
`/data/openmower-updater.conf`) and installs the referenced bundle when its
version is newer than the running one.

The "server" is any static file host:

```sh
./publish.sh ../output/images/openmower-<version>.raucb /var/www/openmower
```

Bench test without TLS:

```sh
./publish.sh ../output/images/openmower-*.raucb /tmp/www
python3 -m http.server -d /tmp/www 8000
# on the device:
echo 'UPDATE_URL="http://<host-ip>:8000"' > /data/openmower-updater.conf
systemctl start openmower-updater
```

Security model: transport is HTTPS in production, but authenticity comes
from the RAUC bundle signature — the device only installs bundles signed
by the certificate baked into its `/etc/rauc/keyring.pem`.

Note: bundles now carry the full vendored ROS/Ubuntu payload and can be
multi-GB — plan server storage/bandwidth accordingly.

Note: `scripts/migrate-to-openmower.sh` (stock Raspberry Pi OS -> OpenMower
OS, see the README's "Migrating from stock Raspberry Pi OS" section) does
*not* use this server -- it downloads a `openmower-<version>.img.gz` +
`.sha256` straight from a GitHub Releases asset via a plain `--url`, no
manifest involved. See `external/board/openmower-cm4/post-image.sh` for
how that asset is built.
