package main

import (
	"fmt"
	"html"
	"os"
	"path/filepath"
	"strings"
	"time"
)

// aggregate summarises all samples of one voice.
type aggregate struct {
	Voice  string
	Label  string
	Engine string
	Native int
	OK     int
	Failed int
	Audio  float64
	Wall   float64
	RTF    float64 // mean of the RTFs reported by sherpa
	RSSMB  int64
	Errors []string
}

func aggregateVoice(v voice, samples []sample) aggregate {
	a := aggregate{Voice: v.slug, Label: v.label, Engine: v.engine}
	var rtfSum float64
	var rtfCount int
	for _, s := range samples {
		if s.Voice != v.slug {
			continue
		}
		if s.Err != "" {
			a.Failed++
			if len(a.Errors) < 3 && !contains(a.Errors, s.Err) {
				a.Errors = append(a.Errors, s.Err)
			}
			continue
		}
		a.OK++
		a.Audio += s.Audio
		a.Wall += s.Wall
		if s.Native > 0 {
			a.Native = s.Native
		}
		if s.RTF > 0 {
			rtfSum += s.RTF
			rtfCount++
		}
		if s.RSSMB > a.RSSMB {
			a.RSSMB = s.RSSMB
		}
	}
	if rtfCount > 0 {
		a.RTF = rtfSum / float64(rtfCount)
	} else if a.Audio > 0 && a.Wall > 0 {
		a.RTF = a.Wall / a.Audio
	}
	return a
}

func contains(list []string, s string) bool {
	for _, x := range list {
		if x == s {
			return true
		}
	}
	return false
}

func writeReport(work, lang string, threads int, voices []voice, phrases []phrase, samples []sample) error {
	var b strings.Builder
	fmt.Fprintf(&b, "# TTS experiment (%s) — %s\n\n", lang, time.Now().Format("2006-01-02 15:04"))
	fmt.Fprintf(&b, "sherpa-onnx %s, %d ONNX threads, %d phrases.\n\n", sherpaVersion, threads, len(phrases))
	fmt.Fprintf(&b, "All files below `out/` are already converted to the format the firmware needs: **16 kHz mono MP3, 32 kbit/s**.\n")
	fmt.Fprintf(&b, "`native` is the sample rate the engine produced before conversion.\n\n")
	fmt.Fprintf(&b, "| voice | engine | native | ok/fail | audio | wall | RTF | max RSS |\n")
	fmt.Fprintf(&b, "|---|---|---:|---:|---:|---:|---:|---:|\n")
	for _, v := range voices {
		a := aggregateVoice(v, samples)
		fmt.Fprintf(&b, "| %s | %s | %s | %d/%d | %.1f s | %.1f s | %.2f | %d MB |\n",
			a.Label, a.Engine, rateLabel(a.Native), a.OK, a.Failed, a.Audio, a.Wall, a.RTF, a.RSSMB)
	}

	var failed []string
	for _, v := range voices {
		a := aggregateVoice(v, samples)
		for _, e := range a.Errors {
			failed = append(failed, fmt.Sprintf("- **%s**: %s", a.Label, e))
		}
	}
	if len(failed) > 0 {
		b.WriteString("\n## Errors\n\n")
		b.WriteString(strings.Join(failed, "\n"))
		b.WriteString("\n")
	}

	b.WriteString("\n## Phrase list\n\n")
	for _, p := range phrases {
		fmt.Fprintf(&b, "- `%s` %s\n", p.ID, p.Text)
	}

	b.WriteString(notesSection)
	return os.WriteFile(filepath.Join(work, "report.md"), []byte(b.String()), 0o644)
}

func rateLabel(rate int) string {
	if rate == 0 {
		return "-"
	}
	return fmt.Sprintf("%.1f kHz", float64(rate)/1000)
}

// writeIndex builds a small HTML page for A/B listening.
func writeIndex(work, lang string, voices []voice, phrases []phrase, samples []sample) error {
	byKey := map[string]sample{}
	for _, s := range samples {
		byKey[s.Voice+"/"+s.PhraseID] = s
	}

	var b strings.Builder
	b.WriteString(`<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<title>TTS experiment</title><style>
body{font-family:system-ui,sans-serif;margin:1.5rem;background:#fafafa;color:#111}
table{border-collapse:collapse;background:#fff}
th,td{border:1px solid #ddd;padding:.35rem .5rem;vertical-align:top;font-size:.85rem}
th{background:#f0f0f0;position:sticky;top:0}
th.rot{writing-mode:vertical-rl;transform:rotate(180deg);height:7rem;white-space:nowrap}
td.txt{max-width:26rem}
audio{height:2rem;width:11rem}
tr:hover td{background:#fffbe6}
.err{color:#b00;font-size:.75rem}
small{color:#666}
</style></head><body>
`)
	fmt.Fprintf(&b, "<h1>TTS experiment &middot; %s</h1>\n", html.EscapeString(lang))
	fmt.Fprintf(&b, "<p><small>%d phrases, generated %s. All clips are 16 kHz mono MP3 (firmware format). ",
		len(phrases), time.Now().Format("2006-01-02 15:04"))
	b.WriteString(`Details and RTF measurements: <a href="report.md">report.md</a></small></p>
<p><small>Tip: playing a clip stops the others. German run: <code>go run . -lang de</code>.</small></p>
`)
	if _, err := os.Stat(filepath.Join(work, "out", "_supertonic-sids", "index.html")); err == nil {
		b.WriteString(`<p><small>All 10 Supertonic speakers on one phrase: <a href="out/_supertonic-sids/index.html">speaker picker</a></small></p>
`)
	}
	b.WriteString(`<table><thead><tr><th>#</th><th>text</th>`)
	for _, v := range voices {
		fmt.Fprintf(&b, "<th class=\"rot\">%s</th>", html.EscapeString(v.label))
	}
	b.WriteString("</tr></thead><tbody>\n")

	for _, p := range phrases {
		fmt.Fprintf(&b, "<tr><td>%s</td><td class=\"txt\">%s</td>", p.ID, html.EscapeString(p.Text))
		for _, v := range voices {
			s, ok := byKey[v.slug+"/"+p.ID]
			switch {
			case !ok:
				b.WriteString("<td>-</td>")
			case s.Err != "":
				fmt.Fprintf(&b, "<td><span class=\"err\">%s</span></td>", html.EscapeString(shortErr(fmt.Errorf("%s", s.Err))))
			default:
				fmt.Fprintf(&b, "<td><audio controls preload=\"none\" src=\"%s\"></audio><br><small>%.2fs</small></td>",
					html.EscapeString(s.File), s.Audio)
			}
		}
		b.WriteString("</tr>\n")
	}
	b.WriteString(`</tbody></table>
<script>
let cur=null;
for (const a of document.querySelectorAll('audio')) {
  a.addEventListener('play', () => { if (cur && cur!==a) cur.pause(); cur=a; });
}
</script>
</body></html>
`)
	return os.WriteFile(filepath.Join(work, "index.html"), []byte(b.String()), 0o644)
}
