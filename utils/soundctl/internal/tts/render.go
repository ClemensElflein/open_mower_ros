package tts

import (
	"fmt"
	"hash/crc32"
	"os"
	"path/filepath"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// Logf is a progress sink (fmt.Printf style). A nil Logf discards progress.
type Logf func(format string, args ...any)

// Options configures a render run.
type Options struct {
	// OutDir is the destination directory (normally the language directory of
	// the writable overlay). Empty uses layout.OutDir().
	OutDir string
	// Force re-renders even when the sidecar says the file is up to date.
	Force bool
	// Check renders nothing and only reports which sounds are missing/stale.
	Check bool
	// Generator is written into the sidecar ("soundctl 0.1.0").
	Generator string
	Log       Logf
}

// Result is the outcome for one sound.
type Result struct {
	Name     string
	File     string
	Path     string
	Skipped  bool // sidecar matched, nothing to do
	Stale    bool // Check only: a render is missing or out of date
	Info     fwsync.MP3Info
	RenderMs int64
	Bytes    int64
}

func (o Options) logf(format string, args ...any) {
	if o.Log != nil {
		o.Log(format, args...)
	}
}

// Pending returns the sound ids whose render is missing or stale, i.e. what
// "soundctl tts --check" prints and what sync refuses to upload.
func Pending(cfg *fwsync.Config, layout fwsync.Layout) []string {
	var out []string
	for _, name := range cfg.TTSSounds() {
		id, ok := cfg.Identity(name)
		if !ok {
			continue
		}
		path, err := layout.Find(cfg.FileName(name))
		if err != nil || !upToDate(path, id) {
			out = append(out, name)
		}
	}
	return out
}

// upToDate reports whether mp3 was rendered from exactly this identity and is
// still the file the sidecar describes.
func upToDate(mp3 string, id fwsync.TTSIdentity) bool {
	sc, err := fwsync.ReadSidecar(mp3)
	if err != nil || !sc.Matches(id) {
		return false
	}
	data, err := os.ReadFile(mp3)
	if err != nil {
		return false
	}
	return sc.MatchesFile(data)
}

// Render synthesises and encodes every "tts:" sound of the config into the
// overlay's language directory and writes the sidecar cache next to each file.
// Sounds whose sidecar already matches are skipped (unless opts.Force).
func Render(cfg *fwsync.Config, layout fwsync.Layout, eng Sherpa, enc Encoder, opts Options) ([]Result, error) {
	names := cfg.TTSSounds()
	if len(names) == 0 {
		return nil, nil
	}

	outDir := opts.OutDir
	if outDir == "" {
		outDir = layout.OutDir()
	}
	if !opts.Check {
		if err := os.MkdirAll(outDir, 0o755); err != nil {
			return nil, fmt.Errorf("create %s: %w", outDir, err)
		}
	}

	results := make([]Result, 0, len(names))
	for i, name := range names {
		id, ok := cfg.Identity(name)
		if !ok {
			continue
		}
		file := cfg.FileName(name)
		dst := filepath.Join(outDir, file)

		// Prefer an up-to-date copy wherever the layout finds it: a render that
		// already sits in the image does not have to be duplicated into the
		// overlay.
		if existing, err := layout.Find(file); err == nil && !opts.Force && upToDate(existing, id) {
			opts.logf("[%d/%d] %-24s up to date (%s)", i+1, len(names), name, existing)
			results = append(results, Result{Name: name, File: file, Path: existing, Skipped: true})
			continue
		}

		if opts.Check {
			results = append(results, Result{Name: name, File: file, Path: dst, Stale: true})
			continue
		}

		res, err := renderOne(name, id, dst, eng, enc, opts)
		if err != nil {
			return results, fmt.Errorf("%s: %w", name, err)
		}
		opts.logf("[%d/%d] %-24s %s, %d kB, %d ms", i+1, len(names), name, res.Info, res.Bytes/1024, res.RenderMs)
		results = append(results, res)
	}
	return results, nil
}

// renderOne renders, verifies and installs a single sound.
func renderOne(name string, id fwsync.TTSIdentity, dst string, eng Sherpa, enc Encoder, opts Options) (Result, error) {
	res := Result{Name: name, File: filepath.Base(dst), Path: dst}

	kbps, err := fwsync.ParseBitrate(id.Bitrate)
	if err != nil {
		return res, err
	}

	tmpDir, err := os.MkdirTemp("", "soundctl-synth-")
	if err != nil {
		return res, fmt.Errorf("create temp dir: %w", err)
	}
	defer os.RemoveAll(tmpDir)

	wav := filepath.Join(tmpDir, "speech.wav")
	synthTime, err := eng.Synthesize(id, wav)
	if err != nil {
		return res, err
	}
	opts.logf("  %-24s synthesised in %s (%s)", name, synthTime.Round(time.Millisecond), id)

	// Encode next to the destination (not into the temp dir), so the final move
	// is a same-filesystem rename and never leaves a partial file behind.
	tmpMP3 := dst + ".tmp"
	start := time.Now()
	warnings, err := enc.Encode(wav, tmpMP3, kbps, id.LoudNorm)
	if err != nil {
		_ = os.Remove(tmpMP3)
		return res, err
	}
	for _, w := range warnings {
		opts.logf("  %-24s warning: %s", name, w)
	}

	// Never install a file the firmware cannot play: the render is the only
	// place where the format can still be corrected cheaply.
	info, err := fwsync.CheckMP3(tmpMP3)
	if err != nil {
		_ = os.Remove(tmpMP3)
		return res, fmt.Errorf("encoded file is not a valid MP3: %w", err)
	}
	if !info.Is16kMono() || !info.HasBitrate(kbps) || info.DurationMs <= 0 {
		_ = os.Remove(tmpMP3)
		return res, fmt.Errorf("encoded file has the wrong format (%s), expected 16 kHz mono CBR %s", info, fwsync.FormatBitrate(kbps))
	}

	data, err := os.ReadFile(tmpMP3)
	if err != nil {
		_ = os.Remove(tmpMP3)
		return res, err
	}
	if err := os.Rename(tmpMP3, dst); err != nil {
		_ = os.Remove(tmpMP3)
		return res, err
	}

	sc := fwsync.NewSidecar(id, info, int64(len(data)), crc32.ChecksumIEEE(data))
	sc.Stamped(opts.Generator, time.Since(start).Milliseconds()+synthTime.Milliseconds(), time.Now())
	if err := fwsync.WriteSidecar(dst, sc); err != nil {
		return res, fmt.Errorf("write sidecar: %w", err)
	}

	res.Info = info
	res.Bytes = int64(len(data))
	res.RenderMs = sc.RenderMs
	return res, nil
}
