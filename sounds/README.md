# Sounds

MP3 files for the HW v2 sound system, plus the `sounds_<lang>.yaml` definitions
(`src/open_mower/params/`) that reference them.

## Files

The effects and music sit flat in this directory; the generated announcements sit in one
subdirectory per language:

| what                                                   | example                     |
| ------------------------------------------------------ | --------------------------- |
| `zz_*.mp3` - effect / background sound, sorts last     | `zz_ping.mp3`               |
| `<language>/<language>_<sound>_<n>.mp3` - a tts render | `de/de_boot_complete_1.mp3` |

A definition references a file by bare name (`file: zz_ping.mp3`), a tts render derives its
name from language and sound id. The lookup adds the language subdirectory by itself (see
[Runtime](#runtime)).

**Format:** MP3, 16 kHz mono, CBR — the firmware decodes at 16 kHz and does *not*
resample. `soundctl validate --check-mp3` checks it.

**Announcements with a `tts:` text are not checked in.** `sounds/<language>/` is
generated (see `.gitignore`) and rendered by `soundctl tts` in CI before the image
build - but only for a language whose engine key is configured. Without one (a
pull request from a fork, or a revoked/rotated key) the image ships without those
MP3s and the robot renders them at startup with its own key. A hand-made recording
still works: give the sound a `file:` and no `tts:`.

## Sound IDs

`SoundId` in `services/sound_service.json` is the catalogue of playable sounds, and the
keys in `sounds_<lang>.yaml` are these names. The list is ordered so that the sounds the
low level needs most sit together (flash-friendly grouping of the persisted overrides).

The value is wire format **and** the key of the overrides the low level persists in flash,
so from now on values are only ever **appended**, never renumbered or reordered — a
renumbered id would silently address a different sound.

|   id | name                  | when                                                                          |
| ---: | --------------------- | ----------------------------------------------------------------------------- |
|    0 | `BOOT_PING`           | stage 2 waits for ROS (repeats every 2 s)                                     |
|    1 | `BOOT_COMPLETE`       | the firmware is up                                                            |
|    2 | `SUCCESS`             | generic positive ack                                                          |
|    3 | `WARNING`             | generic warning                                                               |
|    4 | `ERROR`               | generic error                                                                 |
|    5 | `EMERGENCY`           | an audible emergency was latched (preempts everything)                        |
|    6 | `EMERGENCY_CLEARED`   | the emergency is over                                                         |
|    7 | `ROS_CONNECTED`       | the high level is up                                                          |
|    8 | `ROS_DISCONNECTED`    | the high level heartbeat stopped                                              |
|    9 | `GPS_RTK_FIX`         | RTK fix acquired                                                              |
|   10 | `GPS_RTK_LOST`        | RTK fix lost                                                                  |
|   11 | `AUTONOMOUS_NOTICE`   | "stay back, autonomous mower at work"                                         |
|   12 | `VOLUME_UP`           | one step louder                                                               |
|   13 | `VOLUME_DOWN`         | one step quieter                                                              |
|   14 | `LOW_BATTERY`         | battery is low                                                                |
|   15 | `CHARGING_START`      | charging started                                                              |
|   16 | `CHARGING_DONE`       | battery full                                                                  |
|   17 | `SHUTDOWN_SAFE`       | the Pi is down, safe to power off                                             |
|   18 | `MOWING_START`        | starting to mow (the theme song, one take at random)                          |
|   19 | `MOWING_PAUSED`       | mowing paused (state `PAUSED`)                                                |
|   20 | `MOWING_RESUMED`      | mowing continues                                                              |
|   21 | `MOWING_COMPLETE`     | not yet used, due to missing progress data                                    |
|   22 | `MOWING_AMBIENCE`     | background effects while mowing (rolled by the LL)                            |
|   23 | `DOCKING_START`       | heading back when the reason is not "lawn done"                               |
|   24 | `AREA_RECORDING_MODE` | entering the area-recording mode                                              |
|   25 | `RAIN_DETECTED`       | rain, heading back: defined, but not triggered yet (no rain input, see below) |
|   26 | `LIFT_TRIGGERED`      | wheel-lift sensor triggered an emergency                                      |
|   27 | `STOP_BUTTON`         | the stop button was pressed                                                   |
|   28 | `COLLISION_TRIGGERED` | collision sensor triggered an emergency                                       |
|   29 | `MOWER_RPM_TIMEOUT`   | the blade never reached its speed                                             |
|   30 | `MOWER_RPM_LIMIT`     | the blade RPM went over the safe limit                                        |
|   31 | `RECOVERED`           | Not yet used                                                                  |

Ids the high level fills in carry **no ROM sound**: they stay silent until
`sounds_<lang>.yaml` defines an announcement (TTS text or recording), so a trigger that
has no definition yet cannot beep by accident.

### Who triggers what

The **high level** decides what the robot does and reports it in
`/high_level_status` (`HighLevelStatus`: IDLE / AUTONOMOUS / RECORDING plus the behaviour
name `UNDOCKING`, `MOWING`, `PAUSED`, `DOCKING`, `AREA_RECORDING`). The firmware's sound
state machine turns that into the announcements, so a definition needs no high-level code.

### Debugging a silent mower

The firmware logs **every change of a sound input**, with or without a sound:

```
Sound: input link: 1 -> 0
Sound: input hl: IDLE -> UNDOCKING
Sound: input blade: 0 -> 1
Sound: input emergency: NONE -> STOP
Sound: input emergency_latched: 0 -> 1
Sound: announced 'lift_triggered' (alert=EMERGENCY hl=DOCKING charging=0 blade=0, queued)
```

So `grep 'Sound:'` in the ROS log tells the whole story of an announcement: what the
firmware saw, when it changed and what it played. `Sound: input …` lines appear only on a
change, so a steady robot is silent in the log.

## Engine and voices

The engine is picked per language file, so one language can use ElevenLabs and another
Cartesia. Every language file currently uses **ElevenLabs** with the voice
`TX3LPaxmHKxFdv7VOQHJ` ("Liam", `eleven_v4`); the definition's `language` travels with the
request as the language hint, so a two-word announcement is not left to the model's language detection. The `cartesia` block is kept alongside so that
one language can be switched to `engine: cartesia` (voice
`b7187e84-fe22-4344-ba4a-bc013fcb533e`, "Sebastian", `sonic-3.6`) without touching anything
else.

```yaml
# ElevenLabs (sounds_en.yaml)
tts:
  engine: elevenlabs
  bitrate: 32k             # our encoder
  loudnorm: true           # EBU R128, so all announcements are equally loud
  elevenlabs:
    model: eleven_v4       # or eleven_multilingual_v2 / eleven_flash_v2_5
    voice: TX3LPaxmHKxFdv7VOQHJ
    stability: 0.5
    similarity_boost: 0.75
    style: 0.0
    use_speaker_boost: true

# Cartesia (sounds_de.yaml)
tts:
  engine: cartesia
  bitrate: 32k
  loudnorm: true
  cartesia:
    model: sonic-3.6             # release alias: follows the newest 3.6 build
    voice: b7187e84-fe22-4344-ba4a-bc013fcb533e
```


The text goes to the selected engine; the returned audio is converted to 16 kHz mono,
normalised and encoded by ffmpeg. Nothing runs locally.

The API key never belongs in these files: it comes from `--api-key`,
`--api-key-file`, `$ELEVENLABS_API_KEY` / `$CARTESIA_API_KEY` or
`/data/sounds/.elevenlabs.key` / `/data/sounds/.cartesia.key`, one chain per engine
(see `utils/soundctl/README.md`).

Because a hosted model can change, the render fingerprint is a *request*
fingerprint: `soundctl tts --force` re-rolls an announcement after a model update.

### Variants and ambience

Every mp3 sound lists its takes under `variants`: a plain string is a tts text,
`{file: x.mp3}` an existing recording. A tts variant is stored as
`<language>_<sound>_<n>.mp3`. The low level picks one take per playback
(`pick: random` (default) | `rotate` | `first`), from a small store that `sync`
uploads together with the definitions.

`every` and `chance` let the *low level* decide whether a background sound plays
at all: `every: 120s` evaluates once per interval, `chance: 0.25` plays it with
25 % probability (a sound without `every` only plays when the high level asks for
it). soundctl validates both and puts them into the definitions blob; the state
machine below rolls them.

## Runtime

- Image: this directory at `/opt/open_mower_ros/sounds/`
  (`COPY --link ./ /opt/open_mower_ros`), the engines at
  `/opt/open_mower_ros/tts/`, ffmpeg for loudnorm/encoding.
- Writable overlay: `/data/sounds` (`--sounds-dir` / `$OPENMOWER_SOUNDS_DIR`).
  A file is looked up in `<overlay>/<lang>`, `<sound_path>/<lang>`, `<overlay>`,
  `<sound_path>` — a customised file wins over the shipped one. Renders land in
  `<overlay>/<lang>/`; delete the `.json` sidecar (or edit the text) to re-render.
  A *render* whose sidecar no longer matches the YAML (leftover of an older
  definition) is skipped in favour of the next copy, so a stale overlay render
  never mutes the announcement the image ships — the log names it as
  `ignoring a stale render`.
  The fallback only covers *unchanged* definitions: if an own YAML changes a text,
  the voice or a tts setting, the shipped render belongs to a different identity and
  is not played (it would say the old text). Such a file stays silent until
  `soundctl tts` rendered it into the overlay — so a missing/failing key only costs
  the announcements that were actually edited, not all of them.
- The overlay is **not persistent on a stock robot**: the compose file mounts
  `/data/ros`, `/data/recordings` and `/data/params` individually, so `/data/sounds`
  lives in the container and starts empty after `openmower restart`. Playing the
  shipped sounds does not need it (they are read from the image); customising does.
  The one-time mount is described in the header of `sounds_en.yaml` /
  `sounds_de.yaml`.
- Startup (and every low-level connect) runs `sound_sync` from `mower_comms_v2`:
  1. `soundctl tts` — renders missing/stale announcements (a no-op when the
     image already carries the CI renders);
  2. `soundctl sync` — validates the definition, uploads missing/changed MP3s via
     the FileService to `/sounds/<file>`, deletes orphaned files from `/sounds/`,
     pushes the `sounds:` map (heatshrink) to the SoundService and optionally sets
     the master volume (`--volume`; without it the firmware value stays).
  Both run with `--log-time=false --no-color --log-prefix '[soundctl tts|sync] '`:
  the ROS log already stamps every line, so the CLI's own date/time would be a
  duplicate, and the prefix keeps a line attributable in the node's log. Colour is
  off on a pipe anyway (see the soundctl README) — `--no-color` only makes sure an
  escape sequence can never reach the log.
- Boards without an amplifier: the SoundService (SID 13) never starts, the sync
  stops after the probe (`--sound-check-timeout`, 10 s) without uploading. The
  FileService (SID 12) is always there.
- Discovery: an xbot service advertises every 1 s while it is unclaimed and younger
  than 60 s, afterwards every 10 s — hence `sync --wait` defaults to 30 s, and that
  window starts after the sound probe. If a sync reports `FileService not
  reachable`, run `soundctl discover --wait 25s` in the container: SID 12 listed →
  retry; empty list → the LL is not up (or not on this interface).

## Tooling

The engines, the renderer and the checks live in `soundctl`:
[`utils/soundctl/README.md`](../utils/soundctl/README.md) covers installing both
engines and their models/voices, auditioning a text with `say`, and validating or
rendering a definition from a checkout.
