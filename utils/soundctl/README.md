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
| `tts`                        | render the tts variants into 16 kHz mono CBR MP3s (ElevenLabs, then ffmpeg); a `.json` sidecar per file is the cache; `--check` reports missing/stale renders and needs no key, `--keep-going` renders the rest when one variant fails |
| `tts voices`                 | list the ElevenLabs voices the key may use (`--all` includes the Voice Library ones, which need a paid plan)                                                        |
| `say`                        | render **one** ad-hoc text with the same chain and play it on this machine — no YAML, no cache. The way to audition a text before it goes into a definition    |
| `sync`                       | upload changed MP3s, push the definitions blob, remove orphans, optionally set the volume                                                                      |
| `play` / `stop`              | trigger a tone, sequence, MP3 or configured sound on a running robot                                                                                           |
| `discover`                   | list the services the low level advertises (first thing to check when `sync` cannot reach it)                                                                  |

## Local use (dev machine)

### The ElevenLabs key

Nothing is downloaded or installed: the only asset is the API key. Resolution
order (first hit wins):

```
--api-key  >  --api-key-file  >  $ELEVENLABS_API_KEY  >  /data/sounds/.elevenlabs.key
```

On the mower the key belongs in `/data/sounds/.elevenlabs.key` (mode 600): that
directory is the writable overlay, it is mounted by everyone who customises
sounds anyway, and it is never uploaded (`sync` only sends the files named in
`sounds_*.yaml`) nor deleted (`cleanup` compares the low level's listing against
those names). Locally, exporting `ELEVENLABS_API_KEY` is enough.

`tts --check` and `validate --check-mp3` need no key at all, so CI and a fork can
gate the definitions without a secret.

Which voices the plan may actually use through the API is a licence question, not
a technical one: the built-in premade voices (`voice_type=default`) work on the
free tier, Voice Library voices (`community`, e.g. most non-English ones) need a
paid subscription. `soundctl tts voices` asks the API instead of the portal:

```bash
soundctl tts voices                    # premade, free-tier usable
soundctl tts voices --all --language de # everything German, incl. the library ones
```

### Audition a text

```bash
export ELEVENLABS_API_KEY=sk-...
go run ./cmd/soundctl say "Hallo, ich bin Steve" --lang de --voice I5mfJk5ETJFOHZxoRrHq
go run ./cmd/soundctl say "Der Akku ist fast leer" --lang de --voice I5mfJk5ETJFOHZxoRrHq --stability 0.4
go run ./cmd/soundctl say "Hello" --voice 5gaEGrWSk4v20HMfIhgC --out /tmp/hello.mp3 --play=false
go run ./cmd/soundctl say "Test" --voice 5gaEGrWSk4v20HMfIhgC --player "mpg123 -q"
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

### Engine, voices and licences

Which voice speaks which language, and the `tts:` block of a definition, are
documented in [`sounds/README.md`](../../sounds/README.md). From the CLI:

- `soundctl tts` renders with the engine the YAML names (only `elevenlabs` is
  left; the offline engines were removed), `soundctl say` takes the same settings
  as flags: `--voice`, `--model`, `--stability`, `--similarity-boost`, `--style`,
  `--speaker-boost`.
- `--voice` is the ElevenLabs voice **id** (20 characters, e.g.
  `5gaEGrWSk4v20HMfIhgC` for "Jeremy"), not a name. `--model` is the model id
  (`eleven_v4`, `eleven_multilingual_v2`, `eleven_flash_v2_5`, ...).
- `--api-url` points the client at another API root, e.g. the EU residency host
  `https://api.eu.residency.elevenlabs.io`.
- Every voice setting is part of the render cache key: changing one re-renders the
  announcements on the next `tts` run, which costs one API request per file.
- Licences: the *voice* is licensed by ElevenLabs (and the voice owner), the
  generated audio by the plan you booked - check both before shipping the files
  in a public image. The API also logs the requests unless zero-retention (an
  enterprise feature) is enabled; the texts are generic announcements.
## Layout

```
cmd/soundctl      main
internal/cli      cobra commands (flags only, no logic)
internal/fwsync   domain layer: config, validation, MP3 parser, layout/overlay,
                  sidecar cache, sync + play
internal/tts      ElevenLabs engine, ffmpeg encoder, render pipeline (sidecar cache)
```

The protocol (discovery, claim, heartbeat, RPC calls, DATA and configuration
transactions, heatshrink), lives in the xbot_framework Go module
(`github.com/xtech/xbot_framework/xbot_service_interface_go/xbot`), and the
service clients for file service (SID 12) and sound service (SID 13) are
generated from `services/file_service.json` / `services/sound_service.json` and
published as `github.com/xtech/definitions-open-mower/golang` (`fileservice`,
`soundservice`).

Two details stay local because they are tool policy, not protocol:
`internal/fwsync/filelist.go` decodes the raw `FileList` payload (the schema
declares it as opaque bytes) and defines the 256 byte `FileWrite` chunk size.

`go.mod` pins both released modules (`github.com/xtech/definitions-open-mower/golang`
for the service clients, `github.com/xtech/xbot_framework/xbot_service_interface_go`
for the protocol). A schema change means: regenerate the clients in the
definitions repository, release it, and bump the `require` here - nothing in this
tool has to change.

### What happens when a render fails

A render is a cache lookup, not a build step: for every variant `soundctl tts`
compares the sidecar fingerprint with the identity the definition currently
describes and renders only what differs. That is the whole mechanism - there is no
committed list of fingerprints to keep in sync.

If a render fails (API down, licence, no key), the previous file stays untouched
(renders are written to a temp file and renamed only after the format check):

- `soundctl tts --keep-going` tries every variant, prints each failure and still
  exits non-zero, so the caller decides whether that is fatal.
- `soundctl sync` skips files whose sidecar no longer matches and logs the reason.
  They stay referenced in the definitions blob, so the low level keeps playing the
  file it already has - an old announcement instead of a mismatched one - and the
  rest of the sounds still sync.
- The image build renders best effort (`continue-on-error`), so an API problem
  never blocks the image; the step is marked failed in the run and `tts --check`
  names what is stale.

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
