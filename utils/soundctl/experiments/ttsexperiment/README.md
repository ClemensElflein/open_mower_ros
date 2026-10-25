# ttsexperiment

Throw-away harness to compare TTS engines for the `text:` sounds of
`soundctl`. It renders the strings of the firmware sound sets with every
candidate engine and writes listenable MP3s plus a report.

This directory is its **own Go module**, so the dependency on
`gorilla/websocket` (only needed for the edge-tts experiment) does not leak
into `soundctl`'s `go.mod`.

## Requirements

- `ffmpeg` and `ffprobe` in `$PATH` (conversion to 16 kHz mono MP3 + probing)
- ~400 MB free disk for the models, network access for the first run
- x86-64 or aarch64 Linux (the sherpa-onnx runtime is downloaded per host)

## Usage

```bash
cd utils/soundctl/experiments/ttsexperiment
go run . -lang en            # full English run
go run . -lang de            # full German run
go run . -limit 3 -only supertonic,piper-100 -sids   # quick subset
go run . -lang de -only supertonic-08 -speeds 1.0,1.1,1.2,1.3 -loudnorm
                                                     # tempo A/B, level-matched
```

Everything is written below `-work` (default `/tmp/ttsexperiment`):

```
index.html   A/B player: one row per phrase, one column per voice
report.md    wall time, RTF, max RSS, native sample rate per voice
out/<voice>/<id>.mp3   16 kHz mono 32 kbit/s, exactly the firmware format
out/_supertonic-sids/  one phrase for all 10 Supertonic speakers (-sids)
```

## What is compared

| column | engine | type |
|---|---|---|
| `supertonic-*` | Supertonic 3, 99M, int8 ONNX | offline, en **and** de |
| `kokoro-*` | Kokoro v0.19 int8 (English only) | offline |
| `piper-*` | Piper `en_US-libritts_r-medium` (904 speakers) | offline, English only |
| `google-*` | Google Translate TTS | keyless cloud, reference only |
| `edge-*` | Azure neural voices via Edge (DRM v2 -> 403) | keyless cloud, `-edge` |

Piper is English only here: sherpa-onnx v2.0.0 dropped espeak-ng and the
release does not ship a German lexicon, so a German Piper voice cannot be
driven by this CLI. A German run therefore compares Supertonic (3 speakers)
against Google.

Models and the sherpa-onnx CLI are downloaded to `cache/` on the first run and
reused afterwards. `build_lexicon`-generated `lexicon-en-us.txt` is required for
Kokoro/Piper because sherpa-onnx v2.0.0 no longer ships espeak-ng.

## Notes

See `report.md`, section "Observations".

## Useful flags

| flag | meaning |
|---|---|
| `-lang en\|de` | phrase set and (for Supertonic) the `--lang` passed to the model |
| `-only a,b` | only voices whose slug contains one of the substrings |
| `-limit N` | only the first N phrases |
| `-speeds 1.0,1.1` | render the Supertonic voices once per speaking rate |
| `-loudnorm` | EBU R128 normalise to -16 LUFS / -1.5 dBTP (fair A/B) |
| `-sids` | additionally render every Supertonic speaker for one phrase |
| `-picker-only` | only re-render that speaker picker |
| `-no-cloud` | skip the Google reference |
| `-edge` | also try the Azure/edge voices (expect 403) |
| `-threads N` | ONNX threads (default 4) |
| `-work DIR` | cache + output directory (default `/tmp/ttsexperiment`) |
