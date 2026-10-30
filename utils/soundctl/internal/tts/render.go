package tts

import (
	"errors"
	"fmt"
	"hash/crc32"
	"os"
	"path/filepath"
	"strings"
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
	// KeepGoing continues with the next variant when one fails (a failed render
	// leaves the previous file untouched). The CLI reports the failures at the end
	// and still exits non-zero, so the caller decides whether that is fatal.
	KeepGoing bool
	// Generator is written into the sidecar ("soundctl 0.1.0").
	Generator string
	Log       Logf
}

// Result is the outcome for one sound.
type Result struct {
	Name     string
	File     string
	Path     string
	Skipped  bool  // sidecar matched, nothing to do
	Stale    bool  // Check only: a render is missing or out of date
	Failed   bool  // the render failed, the previous file was kept
	Err      error // why it failed (Failed only)
	Info     fwsync.MP3Info
	RenderMs int64
	Bytes    int64
}

func (o Options) logf(format string, args ...any) {
	if o.Log != nil {
		o.Log(format, args...)
	}
}

// Pending returns the labels of the renders that are missing or stale, i.e. what
// "soundctl tts --check" prints and what sync refuses to upload.
func Pending(cfg *fwsync.Config, layout fwsync.Layout) []string {
	var out []string
	for _, job := range cfg.RenderJobs() {
		path, err := layout.Find(job.File)
		if err != nil || !upToDate(path, job.Identity) {
			out = append(out, job.Label)
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

// ensureWritableDir creates the output directory of a render. The hint matters:
// on the mower the renders belong in the writable overlay (/data/sounds, a mount
// of the data partition) while sound_path is the read-only image, so a missing
// mount shows up as a permission error on a /opt/... path.
func ensureWritableDir(dir string) error {
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return fmt.Errorf("create %s: %w (the renders need a writable sound overlay: mount "+
			"/data/sounds on the mower, or pass --sounds-dir)", dir, err)
	}
	return nil
}

// Render synthesises and encodes every tts variant of the config into the
// overlay's language directory and writes the sidecar cache next to each file.
// Renders whose sidecar already matches are skipped (unless opts.Force).
func Render(cfg *fwsync.Config, layout fwsync.Layout, eng Engine, enc Encoder, opts Options) ([]Result, error) {
	jobs := cfg.RenderJobs()
	if len(jobs) == 0 {
		return nil, nil
	}

	outDir := opts.OutDir
	if outDir == "" {
		outDir = layout.OutDir()
	}
	if !opts.Check {
		if err := ensureWritableDir(outDir); err != nil {
			return nil, err
		}
	}

	results := make([]Result, 0, len(jobs))
	for i, job := range jobs {
		dst := filepath.Join(outDir, job.File)

		// Prefer an up-to-date copy wherever the layout finds it: a render that
		// already sits in the image does not have to be duplicated into the overlay.
		if existing, err := layout.Find(job.File); err == nil && !opts.Force && upToDate(existing, job.Identity) {
			opts.logf("[%d/%d] %-24s up to date (%s)", i+1, len(jobs), job.Label, existing)
			results = append(results, Result{Name: job.Label, File: job.File, Path: existing, Skipped: true})
			continue
		}

		if opts.Check {
			results = append(results, Result{Name: job.Label, File: job.File, Path: dst, Stale: true})
			continue
		}

		res, err := renderOne(job, dst, eng, enc, opts)
		if err != nil {
			if !opts.KeepGoing {
				return results, fmt.Errorf("%s: %w", job.Label, err)
			}
			opts.logf("[%d/%d] %-24s FAILED: %v (previous file kept)", i+1, len(jobs), job.Label, err)
			results = append(results, Result{Name: job.Label, File: job.File, Path: dst, Failed: true, Err: err})
			continue
		}
		opts.logf("[%d/%d] %-24s %s, %d kB, %d ms", i+1, len(jobs), job.Label, res.Info, res.Bytes/1024, res.RenderMs)
		results = append(results, res)
	}
	return results, nil
}

// encodeResult is the outcome of one synthesis + encode pass.
type encodeResult struct {
	Info     fwsync.MP3Info
	Data     []byte
	RenderMs int64
}

// encode synthesises id.Text and encodes it into dst in the firmware format
// (16 kHz mono CBR), verifies the result and only then installs it with a rename
// — a partial or mis-encoded file never appears under the final name. It is the
// shared core of `soundctl tts` (YAML definitions, cached via a sidecar) and
// `soundctl say` (ad-hoc text, no cache), so a local test hears exactly what the
// mower would play. label only shapes the log lines.
func encode(label string, id fwsync.TTSIdentity, dst string, eng Engine, enc Encoder, logf Logf) (encodeResult, error) {
	var res encodeResult

	kbps, err := fwsync.ParseBitrate(id.Bitrate)
	if err != nil {
		return res, err
	}

	tmpDir, err := os.MkdirTemp("", "soundctl-tts-")
	if err != nil {
		return res, fmt.Errorf("create temp dir: %w", err)
	}
	defer os.RemoveAll(tmpDir)

	wav := filepath.Join(tmpDir, "speech.wav")
	synthTime, err := eng.Synthesize(id, wav)
	if err != nil {
		return res, err
	}
	logf("  %-24s synthesised in %s (%s)", label, synthTime.Round(time.Millisecond), eng.Describe(id))

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
		logf("  %-24s warning: %s", label, w)
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

	res.Info = info
	res.Data = data
	res.RenderMs = time.Since(start).Milliseconds() + synthTime.Milliseconds()
	return res, nil
}

// RenderText synthesises one ad-hoc text into dst and returns the result. It uses
// the same engine call, loudness normalisation, encoder and format checks as
// `soundctl tts`, but without a YAML definition and without a sidecar cache (an
// ad-hoc text has no definition to cache against).
func RenderText(text string, id fwsync.TTSIdentity, dst string, eng Engine, enc Encoder, logf Logf) (Result, error) {
	text = strings.TrimSpace(text)
	if text == "" {
		return Result{}, errors.New("no text to speak")
	}
	id.Text = text
	res := Result{Name: "say", File: filepath.Base(dst), Path: dst}
	r, err := encode("say", id, dst, eng, enc, logf)
	if err != nil {
		return res, err
	}
	res.Info = r.Info
	res.Bytes = int64(len(r.Data))
	res.RenderMs = r.RenderMs
	return res, nil
}

// renderOne renders, verifies, installs and caches one variant.
func renderOne(job fwsync.RenderJob, dst string, eng Engine, enc Encoder, opts Options) (Result, error) {
	res := Result{Name: job.Label, File: filepath.Base(dst), Path: dst}

	r, err := encode(job.Label, job.Identity, dst, eng, enc, opts.logf)
	if err != nil {
		return res, err
	}

	sc := fwsync.NewSidecar(job.Identity, r.Info, int64(len(r.Data)), crc32.ChecksumIEEE(r.Data))
	sc.Stamped(opts.Generator, r.RenderMs, time.Now())
	if err := fwsync.WriteSidecar(dst, sc); err != nil {
		return res, fmt.Errorf("write sidecar: %w", err)
	}

	res.Info = r.Info
	res.Bytes = int64(len(r.Data))
	res.RenderMs = sc.RenderMs
	return res, nil
}
