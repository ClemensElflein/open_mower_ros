# soundctl

Command line tool for the HW v2 sound system: it owns `sounds_*.yaml`, its MP3
files and the `tts:` announcements, and it talks to the low level over the xbot
service protocol (FileService SID 12, SoundService SID 13).

The user-facing story (sound files, overlays, startup behaviour) is documented in
[`sounds/README.md`](../../sounds/README.md); this file is about the tool and the code,
including how to run it against a checkout (see *Local use*).

## Commands

| command         | purpose                                                                                                                                                                                                                                                                       |
| --------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `validate`      | check `sounds_*.yaml` against `services/sound_service.json` (ids, types, ranges) and the MP3s (16 kHz mono)                                                                                                                                                                   |
| `tts`           | render the tts variants into 16 kHz mono CBR MP3s (the engine the YAML names: ElevenLabs or Cartesia, then ffmpeg); a `.json` sidecar per file is the cache; `--check` reports missing/stale renders and needs no key, `--keep-going` renders the rest when one variant fails |
| `tts voices`    | list the voices of an engine (`--engine`, default ElevenLabs; `--all` also shows its Voice Library, which needs a paid plan)                                                                                                                                                  |
| `say`           | render **one** ad-hoc text with the same chain and play it on this machine — no YAML, no cache. ElevenLabs only (no engine flag); the way to audition a text before it goes into a definition                                                                                 |
| `sync`          | upload changed MP3s, push the definitions blob, remove orphans, optionally set the volume                                                                                                                                                                                     |
| `play` / `stop` | trigger a tone, sequence, MP3 or configured sound on a running robot                                                                                                                                                                                                          |
| `discover`      | list the services the low level advertises (first thing to check when `sync` cannot reach it)                                                                                                                                                                                 |

## Local use (dev machine)

### The API keys

Nothing is downloaded or installed: the only assets are the API keys of the two
engines (the engine is chosen per language file). Resolution order, first hit
wins, per engine:

```
--api-key > --api-key-file > $ELEVENLABS_API_KEY > /data/sounds/.elevenlabs.key
                            $CARTESIA_API_KEY   > /data/sounds/.cartesia.key
```

On the mower the key belongs in `/data/sounds/.elevenlabs.key` (mode 600): that
directory is the writable overlay, it is mounted by everyone who customises
sounds anyway, and it is never uploaded (`sync` only sends the files named in
`sounds_*.yaml`) nor deleted (`cleanup` compares the low level's listing against
those names). Locally, exporting the engine's variable is enough
(`ELEVENLABS_API_KEY` / `CARTESIA_API_KEY`).

`tts --check` and `validate --check-mp3` need no key at all, so CI and a fork can
gate the definitions without a secret.

Which voices the plan may actually use through the API is a licence question, not
a technical one: the built-in premade voices (`voice_type=default`) work on the
free tier, Voice Library voices (`community`, e.g. most non-English ones) need a
paid subscription. `soundctl tts voices` asks the API instead of the portal:

```bash
soundctl tts voices                    # ElevenLabs, premade (free tier)
soundctl tts voices --all --language de # ElevenLabs, everything German, incl. the library ones
soundctl tts voices --engine cartesia --language de   # Cartesia voices
```

The model id of a Cartesia engine block goes to the API unchanged: `GET /models`
lists the release aliases (`sonic-3.6`, `sonic-3.5`, `sonic-3`), and dated ids
such as `sonic-3.6-2026-08-27` pin a single build without being listed.

### Audition a text

```bash
export ELEVENLABS_API_KEY=sk-...
go run ./cmd/soundctl say "Hallo, ich bin Steve" --lang de --voice I5mfJk5ETJFOHZxoRrHq
go run ./cmd/soundctl say "Der Akku ist fast leer" --lang de --voice I5mfJk5ETJFOHZxoRrHq --stability 0.4
go run ./cmd/soundctl say "Hello" --voice 5gaEGrWSk4v20HMfIhgC --out /tmp/hello.mp3 --play=false
go run ./cmd/soundctl say "Test" --voice 5gaEGrWSk4v20HMfIhgC --player "mpg123 -q"
```

`say` renders with ElevenLabs (`DefaultTTSEngine`; there is no `--engine` flag), so
it cannot audition the Cartesia announcements of the German file - render those
with `soundctl tts` and listen to `sounds/de/*.mp3`. Apart from the engine it uses
the identical call, loudness normalisation, encoder and format check as
`soundctl tts`. Playback is `ffplay -nodisp -autoexit` (comes with the ffmpeg the
encoder needs anyway); `--player` accepts any command that takes the file as last
argument.

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

- `soundctl tts` renders with the engine the YAML names (`elevenlabs` or
  `cartesia`; those are the only two). `soundctl say` takes the same
  ElevenLabs settings as flags: `--voice`, `--model`, `--stability`,
  `--similarity-boost`, `--style`, `--speaker-boost`.
- `--voice` is the ElevenLabs voice **id** (20 characters, e.g.
  `5gaEGrWSk4v20HMfIhgC` for "Jeremy"), not a name. `--model` is the model id
  (`eleven_v4`, `eleven_multilingual_v2`, `eleven_flash_v2_5`, ...).
- `--api-url` points the client at another API root, e.g. the EU residency host
  `https://api.eu.residency.elevenlabs.io`.
- Every voice setting is part of the render cache key: changing one re-renders the
  announcements on the next `tts` run, which costs one API request per file.
- Licences: the *voice* is licensed by the provider (and the voice owner), the
  generated audio by the plan you booked - check both before shipping the files
  in a public image. [`sounds/LICENSE.md`](../../sounds/LICENSE.md) records the
  licence of what this repository ships. The API also logs the requests unless
  zero-retention (an enterprise feature) is enabled; the texts are generic
  announcements.

## Layout

```
cmd/soundctl      main
internal/cli      cobra commands (flags only, no logic) + output formatting
internal/settings deployment policy: where the API keys and the writable sound
                  overlay live, and how a tts block becomes an engine
internal/fwsync   domain layer: config, TTS settings, render identity, render
                  jobs, validation, MP3 parser, layout/overlay, sidecar cache,
                  sync + play
internal/tts      ElevenLabs + Cartesia engines on a shared HTTP layer, ffmpeg
                  encoder, render pipeline (sidecar cache)
```

`internal/cli` stays thin on purpose: a command turns flags into the arguments of
a `fwsync`/`tts`/`settings` call and prints the result. Everything testable lives
below it. The split inside `fwsync` follows the concepts: `config.go` (YAML),
`ttssettings.go` (the resolved `tts:` block), `identity.go` (the render cache key),
`renderjobs.go` (what to render + the definitions blob), `validate.go`, `mp3.go`,
`resolve.go`/`sidecar.go` (file lookup + cache), `sync.go`, `play.go`.

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

The other dependencies are deliberately few and each earns its place:
`spf13/cobra` (commands), `go.yaml.in/yaml/v3` (the YAML definitions — the
maintained home of `gopkg.in/yaml.v3`), `hajimehoshi/go-mp3` (decodability
cross-check next to our own frame walk), `golang.org/x/term` (is stdout a
terminal, the only thing that turns colour on) and, tests only, `google/go-cmp`.
The protocol modules above come on top.

The Go toolchain lives in one place: the `go` directive of `go.mod`. CI
(`actions/setup-go`) reads it, so bumping it there is a one-line change. Only the
`gobuild` stage in `docker/Dockerfile` pins its own `golang:<version>` tag and has
to be bumped alongside it. Nothing else holds a `golang.org/x/*` release back, and
since there is no cgo the same source cross-compiles for the mower.

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
  rest of the sounds still sync. A stale copy never hides a valid one further down
  the lookup order (see below): if the overlay still holds a render of an older
  YAML while the image ships a matching one, the shipped render is uploaded and the
  ignored copy is named in a warning (`ignoring a stale render`).
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
  `<sound_path>`. A *render* (anything with a `.json` sidecar) is resolved by
  `Layout.FindRender`: the first copy whose sidecar still matches the YAML wins, so
  a leftover overlay render from an older definition does not mute the announcement
  the image ships (it is warned about as `ignoring a stale render`).
- Log format: `--log-time=false` drops the CLI's own date/time and
  `--log-prefix '[soundctl] '` prefixes every line — log lines *and* the progress
  lines a command prints itself. `sound_sync` uses both, because the ROS log already
  stamps each line: `[soundctl sync] INFO uploading file=de_x.mp3`. A manual run
  keeps the timestamp and no prefix.
- Colour: `validate`, `tts --check`, `discover` and the error line colour their
  status markers **only when stdout/stderr is a terminal** (`golang.org/x/term`) and
  neither `--no-color`, `$NO_COLOR` nor `TERM=dumb` says otherwise. That is what
  keeps the ROS log clean: `sound_sync` reads the CLI through `popen()`, i.e. a
  pipe, and a redirected log is a pipe too — so no escape sequence ever reaches a
  log file. The level token of a log line is coloured at the writer of the `log`
  package, so the line format itself is unchanged. `sound_sync` passes
  `--no-color` on top, so even a hand-run binary cannot colour its output.
- An xbot service advertises every 1 s only while unclaimed and younger than
  60 s, afterwards every 10 s — hence `sync --wait` defaults to 30 s and the
  discovery errors list what was actually advertised.
