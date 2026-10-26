# soundctl

Command line tool for the HW v2 sound system: it owns `sounds_*.yaml`, its MP3
files and the `tts:` announcements, and it talks to the low level over the xbot
service protocol (FileService SID 12, SoundService SID 13).

The user-facing story (sound files, overlays, startup behaviour) is documented in
[`sounds/README.md`](../../sounds/README.md); this file is about the code.

## Commands

| command | purpose |
|---|---|
| `validate` | check `sounds_*.yaml` against `services/sound_service.json` (ids, types, ranges) and the MP3s (16 kHz mono) |
| `tts` | render the `tts:` texts into 16 kHz mono CBR MP3s (sherpa-onnx + ffmpeg), with a `.json` sidecar as cache; `--check` reports missing/stale renders |
| `tts install` / `tts verify` | fetch / check the pinned engine + model (~220 MB, `$OPENMOWER_TTS_DIR`, default `/opt/open_mower_ros/tts`) |
| `sync` | upload changed MP3s, push the definitions blob, remove orphans, optionally set the volume |
| `play` / `stop` | trigger a tone, sequence, MP3 or configured sound on a running robot |
| `discover` | list the services the low level advertises (first thing to check when `sync` cannot reach it) |

## Layout

```
cmd/soundctl      main
internal/cli      cobra commands (flags only, no logic)
internal/fwsync   domain layer: config, validation, MP3 parser, layout/overlay,
                  sidecar cache, xbot sync + play
internal/tts      engine (sherpa-onnx), ffmpeg encoder, render pipeline, installer
xbot              minimal xbot protocol client (discovery, claim, RPC, heatshrink)
```

`internal/cli` stays thin on purpose: everything testable lives in `fwsync`/`tts`
and is covered by table-driven tests (`*_test.go` next to the code).

## Build and test

```bash
cd utils/soundctl
go build ./cmd/soundctl
go test ./...          # no robot, no network, no ffmpeg needed
go vet ./... && staticcheck ./...
```

The binary is CGO-free (`CGO_ENABLED=0`) so it runs in the focal-based image;
that also means no glibc version of the build host leaks in.

## Runtime notes

- `--sound-path` overrides the YAML's `sound_path` for host runs from a checkout
  (the YAMLs carry the absolute image path).
- The writable overlay is `/data/sounds` (`$OPENMOWER_SOUNDS_DIR`, `--sounds-dir`):
  renders and customised files go there and win over the shipped ones.
- File resolution order: `<overlay>/<lang>`, `<sound_path>/<lang>`, `<overlay>`,
  `<sound_path>`.
- An xbot service advertises every 1 s only while unclaimed and younger than
  60 s, afterwards every 10 s — hence `sync --wait` defaults to 30 s and the
  discovery errors list what was actually advertised.
