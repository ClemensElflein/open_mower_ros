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

## Engines

Each definition picks its engine in its `tts:` block, so the languages differ:

| language | engine                     | voice                           |
| -------- | -------------------------- | ------------------------------- |
| English  | Piper                      | `en_US-bryce-medium`, speed 1.2 |
| German   | Supertonic 3 (sherpa-onnx) | speaker 5, speed 1.0            |

```yaml
tts:
  engine: piper            # or supertonic
  speed: 1.2               # both engines (piper: length_scale = 1/speed)
  bitrate: 32k             # our encoder, both engines
  loudnorm: true           # EBU R128, both engines
  piper:                   # only read when engine: piper
    voice: en_US-bryce-medium
    sentence_silence: 0.15
  supertonic:              # only read when engine: supertonic
    sid: 5
```

Options both engines understand are flat, engine specific ones live in the block
named after the engine. The unused block stays as the documented alternative, so
switching the engine is one line (and re-renders that language). Both engines ship
in the image (~220 MB Supertonic + ~110 MB Piper under `/opt/open_mower_ros/tts/`)
and need no network — but they only *run* when something has to be rendered, which
is the exception because the image already carries the pre-rendered MP3s. Both
sample their output, so re-rendering unchanged text yields a slightly different
"take"; `soundctl tts --force` re-rolls one.

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
| render one announcement, English (Piper)             | 1.3–1.6 s                   |
| render one announcement, German (Supertonic)         | 4.8–7.6 s                   |
| ... similar on the CI runner (arm64)                 | 0.8 s / 3.2–6.2 s           |
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
