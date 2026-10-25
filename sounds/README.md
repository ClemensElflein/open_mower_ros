# Sounds

Default sound files (MP3) for the HW v2 sound system.

## Layout

Files live flat in this directory with a short prefix that also controls the
alphabetical grouping in a directory listing:

| Prefix | Kind | Example |
|---|---|---|
| `<lang>_` | recording (checked in, hand-made) | `de_ros-connected.mp3` |
| `tone_` | tone / beep effect | `tone_warning.mp3` |
| `zz_` | background sound (sorts last) | `zz_minion-fire-alarm.mp3` |

The `sounds_<lang>.yaml` definition references them by bare filename via
`file: de_ros-connected.mp3`; its top-level `sound_path` points here.

Text announcements are normally **not** checked in: a sound with a `tts:` text is
rendered by `soundctl tts` into `sounds/<language>/<file>` (e.g.
`sounds/en/en_boot_complete.mp3`, a 16 kHz mono CBR MP3 plus a `.json` sidecar
that caches the text/voice and marks the file as up to date). A recording only
exists for the few announcements that need a specific voice (e.g. the "Steve"
persona in German).

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
     sidecars are up to date);
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
  (default 3s). The FileService (SID 12) is always present.

## Local use

```bash
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
