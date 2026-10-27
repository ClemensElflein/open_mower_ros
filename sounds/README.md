# Sounds

Default sound files (MP3) for the HW v2 sound system.

## Layout

Files live flat in this directory with a short prefix that also controls the
alphabetical grouping in a directory listing:

| Prefix    | Kind                                        | Example                    |
| --------- | ------------------------------------------- | -------------------------- |
| `<lang>_` | recording (checked in, hand-made; optional) | `de_meine-ansage.mp3`      |
| `tone_`   | tone / beep effect                          | `tone_warning.mp3`         |
| `zz_`     | background sound (sorts last)               | `zz_minion-fire-alarm.mp3` |

The `sounds_<lang>.yaml` definition references them by bare filename via
`file: zz_ping.mp3`; its top-level `sound_path` points here.

Text announcements are normally **not** checked in: a sound with a `tts:` text is
rendered by `soundctl tts` into `sounds/<language>/<file>` (e.g.
`sounds/en/en_boot_complete.mp3`, a 16 kHz mono CBR MP3 plus a `.json` sidecar
that caches the text/voice and marks the file as up to date). Both shipped
languages speak their announcements that way; a hand-made recording is still
possible — give a sound a `file:` and no `tts:` text. (The German recordings that
used to ship here were removed in favour of the TTS voice; `git log` has them if
one is ever wanted back.)

The `<language>/` subdirectories are **not** committed (see `.gitignore`): CI
renders **every** language into them before the image build — the engines cannot
run under buildx emulation — so the image already ships the announcements and a
mower only has to upload them on its first start. A local `soundctl tts` writes to
the same place, which is why `soundctl validate --check-mp3` finds them.

## Engines: which voice speaks which language

Each definition file picks its engine in the `tts:` block, so the two languages
can use different ones. Both engines are baked into the image and render into
`/data/sounds/<language>/` on the first start — no network at runtime.

| language    | engine                         | voice                            | why                                                                         |
| ----------- | ------------------------------ | -------------------------------- | --------------------------------------------------------------------------- |
| **English** | **Piper**                      | `en_US-bryce-medium` @ speed 1.2 | sounded best, is public domain, and half the model size of a "high" voice   |
| **German**  | **Supertonic 3** (sherpa-onnx) | speaker 5 @ speed 1.0            | none of the German Piper voices convinced (thorsten/kerstin/mls were tried) |

```
tts:
  engine: piper            # or supertonic
  speed: 1.2               # both engines (piper: length_scale = 1/speed)
  bitrate: 32k             # our encoder, both engines
  loudnorm: true           # EBU R128, both engines
  piper:                   # only used when engine: piper
    voice: en_US-bryce-medium
    sentence_silence: 0.15
  supertonic:              # only used when engine: supertonic
    sid: 5
```

Options both engines understand stay flat; everything engine specific sits in the
block named after the engine. The block of the engine that is *not* selected is
ignored, so it can stay in the file as the documented alternative — switching the
engine is a one-line change (and re-renders that language on the next start).

Image footprint: Supertonic (engine + model via `soundctl tts install`, ~220 MB)
+ Piper (standalone binary 49 MB + voice 61 MB), both under
`/opt/open_mower_ros/tts/`.

**Licences.** The engines: Piper's standalone binary is MIT, `piper1-gpl` is
GPL-3.0 (same as this project), sherpa-onnx is Apache-2.0 and the Supertonic 3
weights are `openrail`. The Piper *voices* are published under MIT, but each voice
has a `MODEL_CARD` naming the dataset it was trained on — `bryce` is public
domain, `thorsten` is CC0, while e.g. `ryan` and `hfc_*` come from
non-commercial (CC BY-NC-SA) data. `en_US-bryce-medium` therefore ships with a
clean provenance.

The engines that were compared before this choice (all offline unless noted):
Supertonic 3, Piper (en_US/… and the German voices), Kokoro v0.19 (English only)
and — as reference only — Google Translate TTS / Azure "edge" voices, which are
keyless cloud endpoints and unsuitable for a robot.

## Requirements

- MP3 only, **16 kHz mono** — the firmware decodes at 16 kHz and does NOT
  resample. Verify with `soundctl validate --check-mp3`.

## Runtime

- This directory is baked into the image at `/opt/open_mower_ros/sounds/`
  (see `docker/Dockerfile`, `COPY --link ./ /opt/open_mower_ros`).
- The TTS engine + model live at `/opt/open_mower_ros/tts/` (installed at image
  build time by `soundctl tts install`, overridable with `--tts-dir` /
  `$OPENMOWER_TTS_DIR`). ffmpeg, which does the loudness normalisation and MP3
  encoding, is part of the image.
- `/data/sounds` is the **writable overlay** (a mount of the data partition,
  `--sounds-dir` / `$OPENMOWER_SOUNDS_DIR`). A file is looked up in
  `<overlay>/<language>`, `<sound_path>/<language>`, `<overlay>`,
  `<sound_path>` — in that order, so a customised file always wins over the
  shipped one. TTS renders and `soundctl tts` output go to
  `<overlay>/<language>/`, so they survive an image update; delete a `.json`
  sidecar (or edit the text) to force a re-render.
- On startup (and on every low-level connect) `sound_sync` in `mower_comms_v2`
  runs

  1. `soundctl tts`: renders missing/stale announcements (a no-op when the
     sidecars are up to date — with the CI pre-renders in the image that is
     normally the case, so the first start only uploads instead of rendering);
  2. `soundctl sync`:
     * reads `sound_path` + `file`, computes the CRC32 and uploads
       missing/changed files via the FileService to the hardcoded LL path
       `/sounds/<file>`;
     * lists `/sounds/` (FileList) and removes files that are no longer
       referenced by the definition;
     * pushes the sound definitions (the `sounds:` map, heatshrink-compressed)
       to the SoundService "Sound Definitions" register — this is what makes the
       uploaded MP3s actually play;
     * optionally sets the master volume (`--volume 0..100`; omitted = leave the
       firmware value unchanged).
- Per-sound fields (including `preempt`, which marks an alert that interrupts a
  running sound and clears the queue) are documented in the header of
  `src/open_mower/params/sounds_en.yaml`.
- On boards **without** sound hardware the LL never starts the SoundService
  (SID 13); `soundctl sync` detects this via multicast discovery and exits
  without uploading anything. The probe timeout is `--sound-check-timeout`
  (default 10s). The FileService (SID 12) is always present.
- **Discovery timing:** an xbot service advertises itself every second, but only
  while it is unclaimed *and* younger than 60 s (`xbot/config.hpp`). Afterwards
  (and once claimed) it advertises every **10 s**. `soundctl sync` therefore
  waits up to `--wait` (30 s, three advertisement periods) for the FileService,
  and the window starts after the SoundService probe, not at process start.
  If a sync reports `FileService not reachable`, run `soundctl discover --wait 25s`
  inside the container: if SID 12 is listed, the LL is fine (retry / check the
  network); if the list is empty, the LL is not up or not on this interface.

## Measured on the reference mower (CM4)

First start after an image update, i.e. the worst case (nothing rendered yet):

| step                                                          | time                                                    |
| ------------------------------------------------------------- | ------------------------------------------------------- |
| render `boot_complete` / `ros_connected` / `ros_disconnected` | 7.6 s / 6.0 s / 4.8 s (x86 host: 2.1 s / 1.7 s / 1.4 s) |
| SoundService probe (multicast)                                | 3–10 s                                                  |
| FileService discovery (10 s advertisement period)             | ≤ 10 s typical, 30 s window                             |
| upload, per MP3 (256-byte RPC chunks + flash commit)          | 3–5 s (19 kB)                                           |
| whole startup sync                                            | ~30 s                                                   |

A second start is `0 rendered, 3 up to date` + `up to date` for every file, i.e.
only the discovery time. `boot_ping` with `repeat_ms: 4000` repeats with a
measured period of **3.8 s** start-to-start, so `repeat_ms` is the period, not a
gap after the file.

## Local use

```bash
# audition a text with the mower's chain (engine + loudnorm + 16 kHz mono CBR)
# straight from the speakers; no YAML, no cache. On a dev machine point --tts-dir
# ($OPENMOWER_TTS_DIR) at an engine fetched with "soundctl tts install".
soundctl say "Hallo, ich bin Steve" --lang de
soundctl say "Der Akku ist fast leer" --lang de --speed 1.1 --out /tmp/akku.mp3 --play=false

# check the definitions (paths, 16 kHz/mono, tts texts) without a mower.
# --sound-path is needed outside the image, where the absolute
# /opt/open_mower_ros/sounds of the YAML does not exist. An MP3 that only a
# `tts:` text produces and that was not rendered yet is reported as `pending`,
# not as an error.
soundctl validate --sound-path sounds --config src/open_mower/params/sounds_en.yaml --check-mp3

# render the announcements into a scratch directory and inspect them
soundctl tts --sound-path sounds --config src/open_mower/params/sounds_en.yaml --sounds-dir /tmp/sounds

# only report whether anything is missing/stale (exit 1 if so)
soundctl tts --sound-path sounds --config src/open_mower/params/sounds_en.yaml --sounds-dir /tmp/sounds --check
```
