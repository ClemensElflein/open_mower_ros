# soundctl

Command line tool for the HW v2 sound system: it owns `sounds_*.yaml`, its MP3
files and the `tts:` announcements, and it talks to the low level over the xbot
service protocol (FileService SID 12, SoundService SID 13).

The user-facing story (sound files, overlays, startup behaviour) is documented in
[`sounds/README.md`](../../sounds/README.md); this file is about the code.

## Commands

| command                      | purpose                                                                                                                                                     |
| ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `validate`                   | check `sounds_*.yaml` against `services/sound_service.json` (ids, types, ranges) and the MP3s (16 kHz mono)                                                 |
| `tts`                        | render the `tts:` texts into 16 kHz mono CBR MP3s (sherpa-onnx + ffmpeg), with a `.json` sidecar as cache; `--check` reports missing/stale renders          |
| `say`                        | render **one** ad-hoc text with the same chain and play it on this machine — no YAML, no cache. The way to audition a text before it goes into a definition |
| `tts install` / `tts verify` | fetch / check the pinned engine + model (~220 MB, `$OPENMOWER_TTS_DIR`, default `/opt/open_mower_ros/tts`)                                                  |
| `sync`                       | upload changed MP3s, push the definitions blob, remove orphans, optionally set the volume                                                                   |
| `play` / `stop`              | trigger a tone, sequence, MP3 or configured sound on a running robot                                                                                        |
| `discover`                   | list the services the low level advertises (first thing to check when `sync` cannot reach it)                                                               |

## Auditioning a text on a dev machine

```bash
cd utils/soundctl
go run ./cmd/soundctl tts install --dir ~/.cache/openmower-tts   # once, ~220 MB
export OPENMOWER_TTS_DIR=~/.cache/openmower-tts

go run ./cmd/soundctl say "Hallo, ich bin Steve" --lang de       # speaks now
go run ./cmd/soundctl say "Der Akku ist fast leer" --lang de --speed 1.1
go run ./cmd/soundctl say "Hello" --sid 3 --out /tmp/hello.mp3 --play=false
go run ./cmd/soundctl say "Test" --player "mpg123 -q"            # own player
```

`say` uses the identical engine call, loudness normalisation, encoder and format
check as `soundctl tts`, so what you hear is what the mower will play. Playback is
`ffplay -nodisp -autoexit` (comes with the ffmpeg the encoder needs anyway);
`--player` accepts any command that takes the file as last argument.

### Engines and voices

Two engines are supported, and a definition picks one in its `tts:` block:

```yaml
tts:
  engine: piper        # or supertonic
  speed: 1.2           # both engines (piper: length_scale = 1/speed)
  bitrate: 32k         # our encoder
  loudnorm: true       # our ffmpeg stage
  piper:               # only used when engine: piper
    voice: en_US-bryce-medium
    sentence_silence: 0.15
  supertonic:          # only used when engine: supertonic
    sid: 5
```

Only the block of the selected engine is used, so the other one can stay in the
file as the documented alternative. `soundctl tts` renders with the engine the
YAML names; `soundctl say --engine …` does the same ad hoc.

```bash
# Both engines are installed by soundctl (pinned by SHA256, no Python needed for
# piper). ~330 MB in total; --voice picks one of the tested voices.
go run ./cmd/soundctl tts install --dir ~/.cache/openmower-tts
go run ./cmd/soundctl tts install --engine piper --dir ~/.cache/openmower-tts
go run ./cmd/soundctl tts install --engine piper --voice en_US-joe-medium --dir ~/.cache/openmower-tts
export OPENMOWER_TTS_DIR=~/.cache/openmower-tts

# Ad-hoc auditioning (no YAML, no cache):
go run ./cmd/soundctl say --engine piper --voice en_US-bryce-medium "Hi, I am Steve" --speed 1.2
go run ./cmd/soundctl say --lang de --sid 5 "Hallo, ich bin Steve"

# Render every language into the build context, exactly like CI does:
for lang in en de; do
  go run ./cmd/soundctl tts \
    --config "../src/open_mower/params/sounds_${lang}.yaml" \
    --sound-ids ../services/sound_service.json \
    --sound-path ../sounds --sounds-dir ../sounds \
    --tts-dir "$OPENMOWER_TTS_DIR"
done
```

Alternatively piper from pip (`pip install piper-tts`, slower because of the
Python start) works too — pass `--piper-bin ~/.venvs/piper/bin/piper`.

Notes:
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
