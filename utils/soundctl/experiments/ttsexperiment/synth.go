package main

import (
	"fmt"
	"html"
	"os"
	"path/filepath"
	"strings"
)

// synthVoice renders all phrases with one voice and returns the measurements.
func synthVoice(work, outDir string, sc sherpaCfg, v voice, phrases []phrase) []sample {
	dir := filepath.Join(outDir, v.slug)
	if err := os.MkdirAll(dir, 0o755); err != nil {
		fail(err)
	}

	var samples []sample
	for _, p := range phrases {
		s := sample{Voice: v.slug, PhraseID: p.ID, Text: p.Text}
		raw := filepath.Join(work, "tmp", v.slug+"-"+p.ID)
		src := raw + ".wav"
		_ = os.Remove(src)
		_ = os.Remove(raw + ".mp3")

		var res synthResult
		var err error
		switch v.engine {
		case "edge":
			src = raw + ".mp3"
			res, err = synthEdge(v.remote, p.Text, src)
		case "google":
			src = raw + ".mp3"
			res, err = synthGoogle(p.Text, v.remote, src)
		default:
			res, err = sc.run(v.engine, v.sid, v.speed, p.Text, src)
		}
		if err != nil {
			s.Err = shortErr(err)
			fmt.Printf("    %s  FAILED: %s\n", p.ID, s.Err)
			samples = append(samples, s)
			continue
		}

		rate, dur, err := probe(src)
		if err != nil {
			s.Err = shortErr(err)
			fmt.Printf("    %s  FAILED: %s\n", p.ID, s.Err)
			samples = append(samples, s)
			continue
		}

		mp3 := filepath.Join(dir, p.ID+".mp3")
		if err := toFirmwareMP3(src, mp3); err != nil {
			s.Err = shortErr(err)
			fmt.Printf("    %s  FAILED: %s\n", p.ID, s.Err)
			samples = append(samples, s)
			continue
		}

		s.Native, s.Audio, s.Wall = rate, dur, res.Wall.Seconds()
		s.RTF, s.RSSMB = res.RTF, res.MaxRSSKB/1024
		s.File = rel(work, mp3)
		samples = append(samples, s)
		rtf := "-"
		if s.RTF > 0 {
			rtf = fmt.Sprintf("%.2f", s.RTF)
		}
		fmt.Printf("    %s  %5.2fs audio  %5.2fs wall  rtf %s\n", p.ID, dur, s.Wall, rtf)
	}
	return samples
}

// renderSpeakerPicker writes one phrase for every Supertonic speaker so that
// a voice can be picked by ear.
func renderSpeakerPicker(work, outDir string, sc sherpaCfg, text string) {
	pickerDir := filepath.Join(outDir, "_supertonic-sids")
	if err := os.MkdirAll(pickerDir, 0o755); err != nil {
		fail(err)
	}
	fmt.Printf("== voice picker: all Supertonic speakers ==\n")
	var b strings.Builder
	b.WriteString(`<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><title>Supertonic speakers</title>
<style>body{font-family:system-ui,sans-serif;margin:1.5rem;background:#fafafa}
table{border-collapse:collapse;background:#fff}td,th{border:1px solid #ddd;padding:.4rem .6rem}
th{background:#f0f0f0}audio{height:2rem;width:16rem}</style></head><body>
`)
	fmt.Fprintf(&b, "<h1>Supertonic 3 · speaker picker</h1>\n<p><small>%s</small></p>\n<table><tr><th>sid</th><th>sample</th></tr>\n", html.EscapeString(text))
	for sid := 0; sid < 10; sid++ {
		wav := filepath.Join(work, "tmp", fmt.Sprintf("pick-%d.wav", sid))
		mp3 := filepath.Join(pickerDir, fmt.Sprintf("sid-%02d.mp3", sid))
		if _, err := sc.run("supertonic", sid, 0, text, wav); err != nil {
			fmt.Printf("    sid %d FAILED: %s\n", sid, shortErr(err))
			fmt.Fprintf(&b, "<tr><td>%d</td><td><span class=\"err\">%s</span></td></tr>\n", sid, html.EscapeString(shortErr(err)))
			continue
		}
		if err := toFirmwareMP3(wav, mp3); err != nil {
			fmt.Printf("    sid %d FAILED: %s\n", sid, shortErr(err))
			fmt.Fprintf(&b, "<tr><td>%d</td><td><span class=\"err\">%s</span></td></tr>\n", sid, html.EscapeString(shortErr(err)))
			continue
		}
		mark := ""
		if isChosenSupertonicSid(sid) {
			mark = " <strong>(chosen)</strong>"
		}
		fmt.Printf("    sid %d ok\n", sid)
		fmt.Fprintf(&b, "<tr><td>%d%s</td><td><audio controls preload=\"none\" src=\"sid-%02d.mp3\"></audio></td></tr>\n", sid, mark, sid)
	}
	b.WriteString("</table></body></html>\n")
	if err := os.WriteFile(filepath.Join(pickerDir, "index.html"), []byte(b.String()), 0o644); err != nil {
		fail(err)
	}
}
