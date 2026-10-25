package main

// notesSection is appended to report.md.
const notesSection = `
## Observations

- **sherpa-onnx v2.0.0 dropped espeak-ng.** VITS (all Piper voices) and Kokoro
  now need an explicit lexicon: ` + "`--vits-lexicon` / `--kokoro-lexicon lexicon-en-us.txt`" + `.
  Without it the CLI aborts with "no lexicon, and no frontend available".
- **There is no German lexicon in the release** (` + "`lexicon-de.txt` returns 404" + `).
  So Piper German is *not* usable with the v2.0.0 CLI as-is; it would need a
  German G2P/lexicon of our own (sherpa-onnx v1.x still used espeak-ng and
  worked, so an older binary is a fallback for German Piper).
- **Supertonic 3 needs no lexicon** (character level + ` + "`unicode_indexer.bin`" + `) and
  supports ` + "`--lang=de`" + ` with the very same model files - one model for both languages.
- Native sample rates of the engines: Supertonic **44.1 kHz**, Kokoro **24 kHz**,
  Piper medium (libritts_r) **22.05 kHz**. Everything is resampled to the
  16 kHz mono MP3 the firmware expects.
- The RTF column is the value sherpa-onnx prints for itself and therefore only
  covers the ONNX inference (model load is not included). ` + "`wall`" + ` includes the
  process start -> model load -> inference, i.e. the cost of one CLI call.
  A Cortex-A72 (CM4) is roughly 5-15x slower per core than this x86-64 host.
- ` + "`edge`" + ` (the Azure neural voices behind the Edge "read aloud" button) is
  protected by Microsoft's **DRM v2** since 2025: the ` + "`Sec-MS-GEC`" + ` token is now
  built with an obfuscated SHA-256 variant (` + "`Sec-MS-GEC-Version: 2-143.0.3650.75`" + `)
  and a handshake without it is rejected with **HTTP 403**. It is only included
  with ` + "`-edge`" + ` and is expected to fail; re-implementing that hash is not worth
  it for an unofficial endpoint.
- ` + "`google`" + ` (Translate TTS) still works without any key: 24 kHz mono MP3, one
  request per phrase, ~0.2-0.4 s per call. It is the old (non-Neural2) Google
  voice and only a yardstick - the assets in ` + "`soundfiles/`" + ` were made with
  Google **Neural2**, which is noticeably better and needs an account.
- **The engines are not level-matched.** Measured mean/peak level of the same
  phrase in the raw conversion: Supertonic -25.6/-10.8 dB, Kokoro -23.0/-5.7 dB,
  Google -19.1/-4.9 dB, existing ` + "`sounds/*.mp3`" + ` -21.3/-6.2 dB. Google is ~6 dB
  louder than Supertonic, which alone can make it sound "more awake". Use
  ` + "`-loudnorm`" + ` (EBU R128, -16 LUFS, TP -1.5 dB) for any listening comparison;
  it equalises both to ~-16 dB mean.
- **Only the *conversion* is affected by a speed change, not the voice**: at
  Supertonic ` + "`--speed`" + ` 1.0/1.1/1.2/1.3 the German set shrinks from 65.9 s to
  50.7 s total for the same 23 phrases (speaker 8).
- **Licences** (relevant because the MP3s are committed): Kokoro weights
  Apache-2.0 (trained on permissive data), Supertonic 3 *weights* OpenRAIL-M
  (use-based restrictions in Attachment A, MIT only for the sample code),
  Piper voices vary (libritts_r is CC BY 4.0). Generated audio is not restricted
  by any of them for our use; if the model ever ships inside the image, the
  OpenRAIL-M terms would have to travel with it.
`
