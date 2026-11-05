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

|   id | name                  | when                                                   |
| ---: | --------------------- | ------------------------------------------------------ |
|    0 | `BOOT_PING`           | stage 2 waits for ROS (repeats every 2 s)              |
|    1 | `BOOT_COMPLETE`       | the firmware is up                                     |
|    2 | `SUCCESS`             | generic positive ack                                   |
|    3 | `WARNING`             | generic warning                                        |
|    4 | `ERROR`               | generic error                                          |
|    5 | `EMERGENCY`           | an audible emergency was latched (preempts everything) |
|    6 | `EMERGENCY_CLEARED`   | the emergency is over                                  |
|    7 | `ROS_CONNECTED`       | the high level is up                                   |
|    8 | `ROS_DISCONNECTED`    | the high level heartbeat stopped                       |
|    9 | `GPS_RTK_FIX`         | RTK fix acquired                                       |
|   10 | `GPS_RTK_LOST`        | RTK fix lost                                           |
|   11 | `AUTONOMOUS_NOTICE`   | "stay back, autonomous mower at work"                  |
|   12 | `VOLUME_UP`           | one step louder                                        |
|   13 | `VOLUME_DOWN`         | one step quieter                                       |
|   14 | `LOW_BATTERY`         | battery is low                                         |
|   15 | `CHARGING_START`      | charging started                                       |
|   16 | `CHARGING_DONE`       | battery full                                           |
|   17 | `SHUTDOWN_SAFE`       | the Pi is down, safe to power off                      |
|   18 | `MOWING_START`        | starting to mow                                        |
|   19 | `MOWING_PAUSED`       | mowing paused (state `PAUSED`)                         |
|   20 | `MOWING_RESUMED`      | mowing continues                                       |
|   21 | `MOWING_COMPLETE`     | done, heading back to the station                      |
|   22 | `MOWING_AMBIENCE`     | background effects while mowing (rolled by the LL)     |
|   23 | `DOCKING_START`       | heading back to the charging station                   |
|   24 | `AREA_RECORDING_MODE` | entering the area-recording mode                       |
|   25 | `RAIN_DETECTED`       | rain, heading back to the station                      |
|   26 | `LIFT_TRIGGERED`      | wheel-lift sensor triggered an emergency               |
|   27 | `STOP_BUTTON`         | the stop button was pressed                            |
|   28 | `COLLISION_TRIGGERED` | collision sensor triggered an emergency                |
|   29 | `MOWER_RPM_TIMEOUT`   | the blade never reached its speed                      |
|   30 | `MOWER_RPM_LIMIT`     | the blade RPM went over the safe limit                 |
|   31 | `RECOVERED`           | a recovery manoeuvre finished                          |

Ids the high level fills in carry **no ROM sound**: they stay silent until
`sounds_<lang>.yaml` defines an announcement (TTS text or recording), so a trigger that
has no definition yet cannot beep by accident. Two exceptions carry a real ROM sound and
work without any configuration:

- `AUTONOMOUS_NOTICE` — the reversing alarm of lorries and forklifts.
- `VOLUME_UP` / `VOLUME_DOWN` — a rising and a falling figure for the volume slider.

`MOWING_AMBIENCE` is the odd one out: the high level never triggers it. Its definition
carries `every`/`chance`/`pick`, and the **low level** rolls on its own once per interval
and plays a random take — but only while its state machine is mowing (that condition is
firmware logic, not a definition field).

## Engine and voices

The engine is picked per language file, so one language can use ElevenLabs and
another Cartesia:

| language | engine       | voice                                              | model       |
| -------- | ------------ | -------------------------------------------------- | ----------- |
| English  | `elevenlabs` | `TX3LPaxmHKxFdv7VOQHJ` (Liam)                      | `eleven_v4` |
| German   | `cartesia`   | `b7187e84-fe22-4344-ba4a-bc013fcb533e` (Sebastian) | `sonic-3.6` |

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

Cartesia has no counterpart of the ElevenLabs voice settings (stability, style,
speaker boost). Both engines are asked for 16 kHz audio, which is the rate the
firmware plays.

The German definition uses the release alias `sonic-3.6` and therefore follows
the newest stable 3.6 build; a dated id (`sonic-3.6-2026-08-27`) pins one build
instead, and `GET /models` lists only the aliases. Cartesia does not render
bit-identically - two identical requests return different bytes - so pinning
buys the voice *character*, not the bytes. What stays untouched is a file that
is already rendered: its `.json` sidecar records text, voice and model, and
`soundctl tts` only re-renders on a mismatch, so the whole set is rendered from
one model in one go. A single variant re-rendered later (after a text change)
may sound slightly different from its unchanged neighbours - that is the price
of the alias.

The text goes to the selected engine and the returned 16 kHz audio is normalised
and encoded by ffmpeg. Nothing runs locally.
The API key never belongs in these files: it
comes from `--api-key`, `--api-key-file`, `$ELEVENLABS_API_KEY` /
`$CARTESIA_API_KEY` or `/data/sounds/.elevenlabs.key` /
`/data/sounds/.cartesia.key`, one chain per engine (see
`utils/soundctl/README.md`).

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
  Both run with `--log-time=false --log-prefix '[soundctl tts|sync] '`: the ROS log
  already stamps every line, so the CLI's own date/time would be a duplicate, and
  the prefix keeps a line attributable in the node's log.
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

With the pre-renders in the image (an image built with a TTS key) a start only
pays discovery plus uploads (`0 rendered, x up to date`) — rendering is the
exception (new/changed text).

## Tooling

The engines, the renderer and the checks live in `soundctl`:
[`utils/soundctl/README.md`](../utils/soundctl/README.md) covers installing both
engines and their models/voices, auditioning a text with `say`, and validating or
rendering a definition from a checkout.
