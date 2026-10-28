# soundctl

Command line tool for the HW v2 sound system: it owns `sounds_*.yaml`, its MP3
files and the `tts:` announcements, and it talks to the low level over the xbot
service protocol (FileService SID 12, SoundService SID 13).

The user-facing story (sound files, overlays, startup behaviour) is documented in
[`sounds/README.md`](../../sounds/README.md); this file is about the tool and the code,
including how to run it against a checkout (see *Local use*).

## Commands

| command                      | purpose                                                                                                                                                        |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `validate`                   | check `sounds_*.yaml` against `services/sound_service.json` (ids, types, ranges) and the MP3s (16 kHz mono)                                                    |
| `tts`                        | render the `tts:` texts into 16 kHz mono CBR MP3s (Supertonic or Piper, then ffmpeg), with a `.json` sidecar as cache; `--check` reports missing/stale renders |
| `say`                        | render **one** ad-hoc text with the same chain and play it on this machine — no YAML, no cache. The way to audition a text before it goes into a definition    |
| `tts install` / `tts verify` | fetch / check a pinned engine: Supertonic (~220 MB) and/or Piper + one voice (~110 MB), below `$OPENMOWER_TTS_DIR`                                             |
| `sync`                       | upload changed MP3s, push the definitions blob, remove orphans, optionally set the volume                                                                      |
| `play` / `stop`              | trigger a tone, sequence, MP3 or configured sound on a running robot                                                                                           |
| `discover`                   | list the services the low level advertises (first thing to check when `sync` cannot reach it)                                                                  |

## Local use (dev machine)

### Install the engines

`soundctl tts install` fetches the pinned assets, verifies their SHA-256 and
unpacks them. The image build runs the same two commands, so a robot never
downloads anything:

```bash
cd utils/soundctl
export OPENMOWER_TTS_DIR=~/.cache/openmower-tts        # default: /opt/open_mower_ros/tts

# supertonic: sherpa-onnx CLI + libs + the Supertonic 3 model (~220 MB)
go run ./cmd/soundctl tts install --dir "$OPENMOWER_TTS_DIR"
# piper: standalone bundle (binary, espeak-ng data, libs) + one voice (~110 MB)
go run ./cmd/soundctl tts install --engine piper --dir "$OPENMOWER_TTS_DIR"

# completeness check without downloading or running an engine
go run ./cmd/soundctl tts verify --dir "$OPENMOWER_TTS_DIR"
go run ./cmd/soundctl tts verify --engine piper --dir "$OPENMOWER_TTS_DIR"
```

| path below `$OPENMOWER_TTS_DIR` | content                                                        |
| ------------------------------- | -------------------------------------------------------------- |
| `bin/`, `lib/`                  | sherpa-onnx CLI (`sherpa-onnx-offline-tts`) + `libonnxruntime` |
| `models/<model>/`               | Supertonic 3 int8 model (`--model` picks another name)         |
| `piper/`                        | piper binary, `espeak-ng-data`, its shared libraries           |
| `piper/voices/`                 | `<voice>.onnx` + `<voice>.onnx.json`                           |
| `install.json`, `.cache/`       | what was installed (arch, checksums), the archives             |

`--voice` selects one of the pinned Piper voices: `en_US-bryce-medium` (default),
`en_US-joe-medium`, `en_US-mike-medium`, `en_US-norman-medium`,
`de_DE-thorsten-high`. `--arch` installs for another architecture, `--base-url`
uses a mirror, `--force` re-downloads. A render also accepts any `.onnx` path.

### Audition a text

```bash
go run ./cmd/soundctl say "Hallo, ich bin Steve" --lang de       # speaks now
go run ./cmd/soundctl say "Der Akku ist fast leer" --lang de --speed 1.1
go run ./cmd/soundctl say --engine piper --voice en_US-bryce-medium "Hi" --speed 1.2
go run ./cmd/soundctl say "Hello" --sid 3 --out /tmp/hello.mp3 --play=false
go run ./cmd/soundctl say "Test" --player "mpg123 -q"            # own player
```

`say` uses the identical engine call, loudness normalisation, encoder and format
check as `soundctl tts`, so what you hear is what the mower will play. Playback is
`ffplay -nodisp -autoexit` (comes with the ffmpeg the encoder needs anyway);
`--player` accepts any command that takes the file as last argument.

### Check and render the definitions

Outside the image the YAML's absolute `/opt/open_mower_ros/...` paths do not
exist, so pass `--sound-ids` and `--sound-path` (paths are relative to
`utils/soundctl`):

```bash
# paths, 16 kHz mono, tts texts; an unrendered `tts:` sound counts as `pending`
go run ./cmd/soundctl validate --sound-ids ../../services/sound_service.json \
  --sound-path ../../sounds --config ../../src/open_mower/params/sounds_en.yaml --check-mp3

# render into a scratch dir; --check only reports missing/stale (exit 1 if any)
go run ./cmd/soundctl tts --sound-ids ../../services/sound_service.json \
  --sound-path ../../sounds --config ../../src/open_mower/params/sounds_en.yaml --sounds-dir /tmp/sounds
go run ./cmd/soundctl tts --sound-ids ../../services/sound_service.json \
  --sound-path ../../sounds --config ../../src/open_mower/params/sounds_en.yaml --sounds-dir /tmp/sounds --check

# every language into the build context, exactly like CI
for lang in en de; do
  go run ./cmd/soundctl tts \
    --config "../../src/open_mower/params/sounds_${lang}.yaml" \
    --sound-ids ../../services/sound_service.json \
    --sound-path ../../sounds --sounds-dir ../../sounds
done
```

### Engines, voices and licences

Which engine speaks which language, and the `tts:` block of a definition, are
documented in [`sounds/README.md`](../../sounds/README.md). From the CLI:

- `soundctl tts` renders with the engine the YAML names, `soundctl say --engine …`
  with the one given on the command line.
- `--piper-bin` / `--piper-dir` override the piper executable and its voices dir,
  `--tts-bin` / `--tts-lib-dir` / `--tts-model-dir` the sherpa-onnx side.
- piper from pip (`pip install piper-tts`, slower because of the Python start)
  also works: pass `--piper-bin ~/.venvs/piper/bin/piper`.
- `--voice` is a name below `--piper-dir` (`en_US-bryce-medium`) or a path to a
  `.onnx`; the language is derived from the voice name, so `--lang` is not needed.
- `--speed 1.2` maps to piper's `--length_scale 0.833` (its scale is the
  reciprocal of speed); a per-sound `sid:` maps to `-s` for multi-speaker voices;
  `sentence_silence` is passed through.
- Both piper generations work: the 2023 standalone binary (no Python, ~4x faster,
  what the image installs) and the current `piper1-gpl` CLI. The text is fed via
  stdin and only `-m/-f/-s`, `--sentence_silence` and `--length_scale` are used,
  because those exist in both.
- Voice licenses are a two-level thing: the voice repository
  (`rhasspy/piper-voices`) is published under **MIT** and Piper states it imposes
  *no additional restrictions* on the voice models — but every voice directory
  has a `MODEL_CARD` naming the **dataset** it was trained on, and those differ:
  `de_DE-thorsten-*` → CC0, `en_GB-alba-*` → CC BY 4.0,
  `en_US-bryce-*` / `en_US-joe-*` / `en_US-mike-*` / `en_US-norman-*` → public
  domain / CC0, `en_US-ryan-*` / `en_US-hfc_*` → CC BY-NC-SA 4.0 (non-commercial
  source data). The weights are MIT-labelled; the model card is about provenance.

## Layout

```
cmd/soundctl      main
internal/cli      cobra commands (flags only, no logic)
internal/fwsync   domain layer: config, validation, MP3 parser, layout/overlay,
                  sidecar cache, sync + play
internal/tts      engines (sherpa-onnx, piper), ffmpeg encoder, render pipeline, installer
xbot/services     service specific clients: fileservice (SID 12), soundservice (SID 13)
```

The protocol itself — discovery, claim, heartbeat, RPC calls, DATA and
configuration transactions, heatshrink — lives in the xbot_framework Go module
(`github.com/xtech/xbot_framework/xbot_service_interface_go/xbot`). The two
packages below `xbot/services` only add the ids and parameter layouts of
`services/file_service.json` and `services/sound_service.json`.

While the module is still being tested, `go.mod` pins it with a `replace` to the
submodule checkout (`../../src/lib/xbot_framework/xbot_service_interface_go`);
once it is released upstream the replace disappears and only the versioned
`require` remains.

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
  renders and customised files go there and win over the shipped ones. On a stock
  robot this directory is *not* mounted, i.e. it lives in the container and is
  empty after the next restart — fine for the shipped sounds (they are read from
  the image), relevant only for customisations (see `sounds/README.md`).
- File resolution order: `<overlay>/<lang>`, `<sound_path>/<lang>`, `<overlay>`,
  `<sound_path>`.
- An xbot service advertises every 1 s only while unclaimed and younger than
  60 s, afterwards every 10 s — hence `sync --wait` defaults to 30 s and the
  discovery errors list what was actually advertised.
