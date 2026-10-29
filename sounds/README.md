# Sounds

MP3 files for the HW v2 sound system, plus the `sounds_<lang>.yaml` definitions
(`src/open_mower/params/`) that reference them.

## Files

Flat in this directory, prefixed so that a listing groups them:

| prefix    | what it is                                                   | example                |
| --------- | ------------------------------------------------------------ | ---------------------- |
| `zz_`     | effect / background sound, sorts last                        | `zz_ping.mp3`          |
| `<lang>_` | one language's version of an announcement (render/recording) | `de_boot_complete.mp3` |

A definition references a file by bare name (`file: zz_ping.mp3`); its
`sound_path` points here.

**Format:** MP3, 16 kHz mono, CBR — the firmware decodes at 16 kHz and does *not*
resample. `soundctl validate --check-mp3` checks it.

**Announcements with a `tts:` text are not checked in.** `sounds/<language>/` is
generated (see `.gitignore`) and rendered by `soundctl tts` in CI before the image
build. A hand-made recording still works: give the sound a `file:` and no `tts:`.

## Engine and voices

Both languages use ElevenLabs; the engine is picked per definition, so a language
can use another voice, model or voice setting:

| language | voice                           | model       |
| -------- | ------------------------------- | ----------- |
| English  | `5gaEGrWSk4v20HMfIhgC` (Jeremy) | `eleven_v4` |
| German   | `I5mfJk5ETJFOHZxoRrHq` (Hans)   | `eleven_v4` |

```yaml
tts:
  engine: elevenlabs
  bitrate: 32k             # our encoder
  loudnorm: true           # EBU R128, so all announcements are equally loud
  elevenlabs:
    model: eleven_v4       # or eleven_multilingual_v2 / eleven_flash_v2_5
    voice: 5gaEGrWSk4v20HMfIhgC
    stability: 0.5
    similarity_boost: 0.75
    style: 0.0
    use_speaker_boost: true
```

The text goes to the ElevenLabs API and the returned 16 kHz audio is normalised
and encoded by ffmpeg. Nothing runs locally.
The API key never belongs in these files: it
comes from `--api-key`, `--api-key-file`, `$ELEVENLABS_API_KEY` or
`/data/sounds/.elevenlabs.key` (see `utils/soundctl/README.md`).

Because a hosted model can change, the render fingerprint is a *request*
fingerprint: `soundctl tts --force` re-rolls an announcement after a model update.

### Variants and ambience

Every mp3 sound lists its takes under `variants`: a plain string is a tts text,
`{file: x.mp3}` an existing recording. A tts variant is stored as
`<language>_<sound>_<n>.mp3`. The low level picks one take per playback
(`pick: random` (default) | `rotate` | `first`); until its firmware knows the
list it plays the first take, while all of them are already rendered and
uploaded.

`every` and `chance` let the *low level* decide whether a background sound plays
at all: `every: 120s` evaluates once per interval, `chance: 0.25` plays it with
25 % probability (a sound without `every` only plays when the high level asks for
it). soundctl validates both and puts them into the definitions blob; the
playback itself is a firmware feature that is still to come.
## Runtime

- Image: this directory at `/opt/open_mower_ros/sounds/`
  (`COPY --link ./ /opt/open_mower_ros`), the engines at
  `/opt/open_mower_ros/tts/`, ffmpeg for loudnorm/encoding.
- Writable overlay: `/data/sounds` (`--sounds-dir` / `$OPENMOWER_SOUNDS_DIR`).
  A file is looked up in `<overlay>/<lang>`, `<sound_path>/<lang>`, `<overlay>`,
  `<sound_path>` — a customised file wins over the shipped one. Renders land in
  `<overlay>/<lang>/`; delete the `.json` sidecar (or edit the text) to re-render.
- The overlay is **not persistent on a stock robot**: the compose file mounts
  `/data/ros`, `/data/recordings` and `/data/params` individually, so `/data/sounds`
  lives in the container and starts empty after `openmower restart`. Playing the
  shipped sounds does not need it (they are read from the image); customising does.
  The one-time mount is described in the header of `sounds_en.yaml` /
  `sounds_de.yaml`.
- Startup (and every low-level connect) runs `sound_sync` from `mower_comms_v2`:
  1. `soundctl tts` — renders missing/stale announcements (a no-op with the CI
     pre-renders in the image);
  2. `soundctl sync` — validates the definition, uploads missing/changed MP3s via
     the FileService to `/sounds/<file>`, deletes orphaned files from `/sounds/`,
     pushes the `sounds:` map (heatshrink) to the SoundService and optionally sets
     the master volume (`--volume`; without it the firmware value stays).
- Boards without an amplifier: the SoundService (SID 13) never starts, the sync
  stops after the probe (`--sound-check-timeout`, 10 s) without uploading. The
  FileService (SID 12) is always there.
- Discovery: an xbot service advertises every 1 s while it is unclaimed and younger
  than 60 s, afterwards every 10 s — hence `sync --wait` defaults to 30 s, and that
  window starts after the sound probe. If a sync reports `FileService not
  reachable`, run `soundctl discover --wait 25s` in the container: SID 12 listed →
  retry; empty list → the LL is not up (or not on this interface).

## Measured on development mower (CM5)

| step                                                 | time                        |
| ---------------------------------------------------- | --------------------------- |
| render one announcement (ElevenLabs + ffmpeg)        | 1–2 s (API round trip)      |
| SoundService probe (multicast)                       | 3–10 s                      |
| FileService discovery (10 s advertisement period)    | ≤ 10 s typical, 30 s window |
| upload, per MP3 (256-byte RPC chunks + flash commit) | 3–5 s (19 kB)               |

With the pre-renders in the image a start only pays discovery plus uploads
(`0 rendered, x up to date`) — rendering is the exception (new/changed text).

## Tooling

The engines, the renderer and the checks live in `soundctl`:
[`utils/soundctl/README.md`](../../utils/soundctl/README.md) covers installing both
engines and their models/voices, auditioning a text with `say`, and validating or
rendering a definition from a checkout.
