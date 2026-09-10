# Sounds

Default sound files (MP3) for the HW v2 sound system.

## Layout

Files live flat in this directory with a short prefix that also controls the
alphabetical grouping in a directory listing:

| Prefix | Kind | Example |
|---|---|---|
| `<lang>_` | text announcement (spoken) | `en_boot_complete.mp3` |
| `tone_` | tone / beep effect | `tone_warning.mp3` |
| `zz_` | background sound (sorts last) | `zz_mowing.mp3` |

The `sounds_<lang>.yaml` override file references them by bare filename via
`file: en_boot_complete.mp3`; its top-level `sound_path` points here.

## Requirements

- MP3 only, **16 kHz mono** — the firmware decodes at 16 kHz and does NOT
  resample. Verify with `soundctl validate --check-mp3`.

## Runtime

- This directory is baked into the image at `/opt/open_mower_ros/sounds/`
  (see `docker/Dockerfile`, `COPY --link ./ /opt/open_mower_ros`).
- `soundctl sync`:
  1. reads `sound_path` + `file`, computes the CRC32 and uploads
     missing/changed files via the FileService to the hardcoded LL path
     `/sounds/<file>`;
  2. lists `/sounds/` (FileList) and removes files that are no longer
     referenced by the definition;
  3. pushes the sound definitions (the `sounds:` map, heatshrink-compressed) to
     the SoundService "Sound Definitions" register — this is what makes the
     uploaded MP3s actually play;
  4. optionally sets the master volume (`--volume 0..100`; omitted = leave the
     firmware value unchanged).
