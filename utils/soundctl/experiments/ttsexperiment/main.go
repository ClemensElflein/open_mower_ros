package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

// voice is one speaker of one engine that shall be compared.
type voice struct {
	slug   string // directory name below out/
	label  string // shown in the report/index
	engine string // supertonic | kokoro | piper-en | edge | google
	sid    int
	speed  float64 // Supertonic only: speaking rate, 1.0 = model default
	remote string  // edge voice name or google language code
}

// chosenSupertonicSids are the two speakers that survived the speaker-picker
// listening test. Speaker 5 is the project default (slightly clearer than 6);
// 6 is kept as the second column so a regression in 5 is immediately audible.
var chosenSupertonicSids = []int{5, 6}

// isChosenSupertonicSid reports whether sid is one of the short-listed speakers.
func isChosenSupertonicSid(sid int) bool {
	for _, s := range chosenSupertonicSids {
		if s == sid {
			return true
		}
	}
	return false
}

func defaultVoices(lang string) []voice {
	voices := []voice{
		{slug: "supertonic-05", label: "Supertonic 3 · speaker 5 (default)", engine: "supertonic", sid: 5},
		{slug: "supertonic-06", label: "Supertonic 3 · speaker 6", engine: "supertonic", sid: 6},
	}
	if lang == "de" {
		return voices
	}
	// English-only alternatives/yardsticks. Piper is out (rejected on quality,
	// 2026-10) but the engine stays wired up for future model versions.
	return append(voices,
		voice{slug: "kokoro-05", label: "Kokoro · speaker 5", engine: "kokoro", sid: 5},
	)
}

// googleVoice is the keyless cloud reference. It is not offline, so it is only
// a yardstick and never a production candidate.
func googleVoice(lang string) voice {
	if lang == "de" {
		return voice{slug: "google-de", label: "Google Translate · de", engine: "google", remote: "de"}
	}
	return voice{slug: "google-en", label: "Google Translate · en", engine: "google", remote: "en"}
}

// edgeVoices are the Azure neural voices served by the Edge "read aloud"
// endpoint. As of 10/2026 they are protected by the v2 DRM (Sec-MS-GEC v2),
// so these columns will very likely fail with http 403 - kept for reference.
func edgeVoices(lang string) []voice {
	if lang == "de" {
		return []voice{
			{slug: "edge-katja", label: "Azure (edge) · de-DE-KatjaNeural", engine: "edge", remote: "de-DE-KatjaNeural"},
			{slug: "edge-conrad", label: "Azure (edge) · de-DE-ConradNeural", engine: "edge", remote: "de-DE-ConradNeural"},
		}
	}
	return []voice{
		{slug: "edge-aria", label: "Azure (edge) · en-US-AriaNeural", engine: "edge", remote: "en-US-AriaNeural"},
		{slug: "edge-guy", label: "Azure (edge) · en-US-GuyNeural", engine: "edge", remote: "en-US-GuyNeural"},
	}
}

// sample is the outcome of one synthesis call.
type sample struct {
	Voice    string
	PhraseID string
	Text     string
	Native   int     // sample rate of the engine output in Hz
	Audio    float64 // audio duration in seconds
	Wall     float64 // wall clock time in seconds
	RTF      float64
	RSSMB    int64
	File     string // relative path below work dir
	Err      string
}

func main() {
	work := flag.String("work", "/tmp/ttsexperiment", "working directory (cache + output)")
	lang := flag.String("lang", "en", "language to synthesise: en or de")
	only := flag.String("only", "", "comma separated list of voice slugs (substring match)")
	threads := flag.Int("threads", 4, "number of ONNX threads")
	limit := flag.Int("limit", 0, "only the first N phrases (0 = all)")
	sids := flag.Bool("sids", false, "additionally render all Supertonic speakers for one phrase")
	skipCloud := flag.Bool("no-cloud", false, "skip the cloud reference (Google Translate)")
	withEdge := flag.Bool("edge", false, "also try the Azure/edge voices (currently blocked by DRM v2)")
	pickerOnly := flag.Bool("picker-only", false, "only render the Supertonic speaker picker")
	speeds := flag.String("speeds", "", "comma separated Supertonic speaking rates to compare, e.g. 1.0,1.1,1.2")
	loud := flag.Bool("loudnorm", false, "normalise loudness to -16 LUFS / -1.5 dBTP (use for a fair A/B)")
	bitrate := flag.String("bitrate", "32k", "CBR bitrate of the generated MP3s (16k, 32k, 64k)")
	flag.Parse()
	loudNorm = *loud
	mp3Bitrate = *bitrate

	cache := filepath.Join(*work, "cache")
	outDir := filepath.Join(*work, "out")
	for _, d := range []string{cache, outDir, filepath.Join(*work, "tmp")} {
		if err := os.MkdirAll(d, 0o755); err != nil {
			fail(err)
		}
	}

	phrases := phrasesFor(*lang)
	if *limit > 0 && *limit < len(phrases) {
		phrases = phrases[:*limit]
	}

	fmt.Printf("== assets (%s) ==\n", *lang)
	if err := ensureAssets(cache, []string{*lang}); err != nil {
		fail(err)
	}

	sc := sherpaCfg{cache: cache, threads: *threads, lang: *lang}

	if *pickerOnly {
		renderSpeakerPicker(*work, outDir, sc, phrases[0].Text)
		fmt.Printf("\nPicker: %s\n", filepath.Join(outDir, "_supertonic-sids", "index.html"))
		return
	}

	voices := defaultVoices(*lang)
	if !*skipCloud {
		voices = append(voices, googleVoice(*lang))
	}
	if *withEdge {
		voices = append(voices, edgeVoices(*lang)...)
	}
	voices = filterVoices(expandSpeeds(voices, *speeds), *only)

	fmt.Printf("== synthesising %d phrases with %d voices ==\n", len(phrases), len(voices))
	var samples []sample
	for vi, v := range voices {
		fmt.Printf("[%d/%d] %s\n", vi+1, len(voices), v.label)
		samples = append(samples, synthVoice(*work, outDir, sc, v, phrases)...)
	}

	if *sids {
		renderSpeakerPicker(*work, outDir, sc, phrases[0].Text)
	}

	if err := writeReport(*work, *lang, *threads, voices, phrases, samples); err != nil {
		fail(err)
	}
	if err := writeIndex(*work, *lang, voices, phrases, samples); err != nil {
		fail(err)
	}
	fmt.Printf("\nDone.\n  index:  %s\n  report: %s\n", filepath.Join(*work, "index.html"), filepath.Join(*work, "report.md"))
}

// expandSpeeds multiplies the Supertonic voices by the requested speaking rates,
// so a single run can A/B the delivery tempo. Engines without a speed knob are
// emitted once, unchanged.
func expandSpeeds(voices []voice, spec string) []voice {
	if strings.TrimSpace(spec) == "" {
		return voices
	}
	var speeds []float64
	for _, f := range strings.Split(spec, ",") {
		v, err := strconv.ParseFloat(strings.TrimSpace(f), 64)
		if err != nil || v <= 0 {
			fail(fmt.Errorf("invalid -speeds value %q (expected positive floats)", f))
		}
		speeds = append(speeds, v)
	}

	var out []voice
	for _, v := range voices {
		if v.engine != "supertonic" {
			out = append(out, v)
			continue
		}
		for _, s := range speeds {
			nv := v
			nv.speed = s
			nv.slug = fmt.Sprintf("%s-s%03d", v.slug, int(s*100+0.5))
			nv.label = fmt.Sprintf("%s · %.2fx", v.label, s)
			out = append(out, nv)
		}
	}
	return out
}

func filterVoices(all []voice, only string) []voice {
	var out []voice
	for _, v := range all {
		if only != "" {
			match := false
			for _, want := range strings.Split(only, ",") {
				if strings.Contains(v.slug, strings.TrimSpace(want)) {
					match = true
				}
			}
			if !match {
				continue
			}
		}
		out = append(out, v)
	}
	return out
}
