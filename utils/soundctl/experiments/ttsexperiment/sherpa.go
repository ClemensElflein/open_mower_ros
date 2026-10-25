package main

import (
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"syscall"
	"time"
)

const (
	sherpaVersion = "v2.0.0"
	sherpaDir     = "sherpa-onnx-v2.0.0-linux-x64-shared"

	supertonicDir = "sherpa-onnx-supertonic-3-tts-int8-2026-05-11"
	kokoroDir     = "kokoro-int8-en-v0_19"
	piperEnDir    = "vits-piper-en_US-libritts_r-medium"

	releaseBase = "https://github.com/k2-fsa/sherpa-onnx/releases/download"
	modelsBase  = releaseBase + "/tts-models"
)

// modelAsset is one download that the sherpa engine needs.
type modelAsset struct {
	name   string // file name inside the cache directory
	url    string
	dir    string // directory that must exist after extraction ("" = plain file)
	langs  []string
	mbytes int
}

var allAssets = []modelAsset{
	{name: sherpaDir + ".tar.bz2", url: releaseBase + "/" + sherpaVersion + "/" + sherpaDir + ".tar.bz2", dir: sherpaDir, langs: []string{"en", "de"}, mbytes: 27},
	{name: supertonicDir + ".tar.bz2", url: modelsBase + "/" + supertonicDir + ".tar.bz2", dir: supertonicDir, langs: []string{"en", "de"}, mbytes: 123},
	{name: kokoroDir + ".tar.bz2", url: modelsBase + "/" + kokoroDir + ".tar.bz2", dir: kokoroDir, langs: []string{"en"}, mbytes: 103},
	{name: piperEnDir + ".tar.bz2", url: modelsBase + "/" + piperEnDir + ".tar.bz2", dir: piperEnDir, langs: []string{"en"}, mbytes: 82},
	{name: "lexicon-en-us.txt", url: modelsBase + "/lexicon-en-us.txt", langs: []string{"en"}, mbytes: 6},
}

// sherpaCfg holds everything needed to call the sherpa-onnx-offline-tts CLI.
type sherpaCfg struct {
	cache   string
	threads int
	lang    string
}

func (c sherpaCfg) bin() string {
	return filepath.Join(c.cache, sherpaDir, "bin", "sherpa-onnx-offline-tts")
}
func (c sherpaCfg) libDir() string               { return filepath.Join(c.cache, sherpaDir, "lib") }
func (c sherpaCfg) dir(name string) string       { return filepath.Join(c.cache, name) }
func (c sherpaCfg) file(dir, name string) string { return filepath.Join(c.cache, dir, name) }

// args returns the CLI arguments for one model/speaker/text combination.
func (c sherpaCfg) args(model string, sid int, speed float64, text, out string) ([]string, error) {
	common := []string{
		fmt.Sprintf("--num-threads=%d", c.threads),
		fmt.Sprintf("--sid=%d", sid),
		"--output-filename=" + out,
	}
	switch model {
	case "supertonic":
		return append([]string{
			"--supertonic-duration-predictor=" + c.file(supertonicDir, "duration_predictor.int8.onnx"),
			"--supertonic-text-encoder=" + c.file(supertonicDir, "text_encoder.int8.onnx"),
			"--supertonic-vector-estimator=" + c.file(supertonicDir, "vector_estimator.int8.onnx"),
			"--supertonic-vocoder=" + c.file(supertonicDir, "vocoder.int8.onnx"),
			"--supertonic-tts-json=" + c.file(supertonicDir, "tts.json"),
			"--supertonic-unicode-indexer=" + c.file(supertonicDir, "unicode_indexer.bin"),
			"--supertonic-voice-style=" + c.file(supertonicDir, "voice.bin"),
			"--lang=" + c.lang,
			"--num-steps=8",
			fmt.Sprintf("--speed=%.2f", supertonicSpeed(speed)),
		}, append(common, text)...), nil
	case "kokoro":
		return append([]string{
			"--kokoro-model=" + c.file(kokoroDir, "model.int8.onnx"),
			"--kokoro-voices=" + c.file(kokoroDir, "voices.bin"),
			"--kokoro-tokens=" + c.file(kokoroDir, "tokens.txt"),
			"--kokoro-lexicon=" + c.file("", "lexicon-en-us.txt"),
		}, append(common, text)...), nil
	case "piper-en":
		return append([]string{
			"--vits-model=" + c.file(piperEnDir, "en_US-libritts_r-medium.onnx"),
			"--vits-tokens=" + c.file(piperEnDir, "tokens.txt"),
			"--vits-lexicon=" + c.file("", "lexicon-en-us.txt"),
		}, append(common, text)...), nil
	}
	return nil, fmt.Errorf("unknown sherpa model %q", model)
}

// supertonicSpeed normalises the speaking rate: 0 means "not set" and falls back
// to the model default of 1.0, which sherpa also accepts as the CLI default.
func supertonicSpeed(s float64) float64 {
	if s <= 0 {
		return 1
	}
	return s
}

// synthResult collects what we measured for one synthesis call.
type synthResult struct {
	Wall     time.Duration
	RTF      float64 // as reported by sherpa (0 when unknown)
	MaxRSSKB int64
}

// run executes the sherpa CLI and returns timing plus the reported RTF.
func (c sherpaCfg) run(model string, sid int, speed float64, text, out string) (synthResult, error) {
	args, err := c.args(model, sid, speed, text, out)
	if err != nil {
		return synthResult{}, err
	}
	cmd := exec.Command(c.bin(), args...)
	cmd.Env = append(os.Environ(), "LD_LIBRARY_PATH="+c.libDir())
	start := time.Now()
	outBytes, err := cmd.CombinedOutput()
	res := synthResult{Wall: time.Since(start)}
	if cmd.ProcessState != nil {
		if ru, ok := cmd.ProcessState.SysUsage().(*syscall.Rusage); ok {
			res.MaxRSSKB = int64(ru.Maxrss)
		}
	}
	if err != nil {
		return res, fmt.Errorf("%v: %s", err, tail(string(outBytes), 400))
	}
	res.RTF = parseRTF(string(outBytes))
	return res, nil
}

// parseRTF extracts the "Real-time factor (RTF): a/b = c" line that
// sherpa-onnx-offline-tts prints on stderr.
func parseRTF(s string) float64 {
	const marker = "(RTF): "
	idx := strings.LastIndex(s, marker)
	if idx < 0 {
		return 0
	}
	rest := s[idx+len(marker):]
	if eq := strings.Index(rest, "= "); eq >= 0 {
		rest = rest[eq+2:]
	}
	var rtf float64
	if _, err := fmt.Sscanf(rest, "%g", &rtf); err != nil {
		return 0
	}
	return rtf
}

func tail(s string, n int) string {
	if len(s) <= n {
		return strings.TrimSpace(s)
	}
	return strings.TrimSpace(s[len(s)-n:])
}

// ensureAssets downloads and extracts everything that the given languages need.
func ensureAssets(cache string, langs []string) error {
	for _, a := range allAssets {
		if !assetNeeded(a, langs) {
			continue
		}
		done := a.name
		if a.dir != "" {
			done = a.dir
		}
		if _, err := os.Stat(filepath.Join(cache, done)); err == nil {
			continue
		}
		archive := filepath.Join(cache, a.name)
		if _, err := os.Stat(archive); err != nil {
			fmt.Printf("  download %-62s (%d MB)\n", a.name, a.mbytes)
			if err := downloadFile(a.url, archive); err != nil {
				return fmt.Errorf("download %s: %w", a.name, err)
			}
		}
		if a.dir != "" {
			fmt.Printf("  extract  %s\n", a.name)
			if out, err := exec.Command("tar", "-xjf", archive, "-C", cache).CombinedOutput(); err != nil {
				return fmt.Errorf("extract %s: %v: %s", a.name, err, tail(string(out), 300))
			}
		}
	}
	return nil
}

func assetNeeded(a modelAsset, langs []string) bool {
	for _, l := range langs {
		for _, al := range a.langs {
			if l == al {
				return true
			}
		}
	}
	return false
}

func downloadFile(url, dest string) error {
	resp, err := http.Get(url)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return fmt.Errorf("http %d", resp.StatusCode)
	}
	tmp := dest + ".part"
	f, err := os.Create(tmp)
	if err != nil {
		return err
	}
	if _, err := io.Copy(f, resp.Body); err != nil {
		f.Close()
		return err
	}
	if err := f.Close(); err != nil {
		return err
	}
	return os.Rename(tmp, dest)
}
