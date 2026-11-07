package tts

import (
	"errors"
	"hash/crc32"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// TestEnsureWritableDirPinsTheMountHint: a render into a path that cannot be
// created must say *what* is missing (the overlay mount), not just the errno —
// that message is the first thing a user sees when /data/sounds is not mounted.
func TestEnsureWritableDirPinsTheMountHint(t *testing.T) {
	file := filepath.Join(t.TempDir(), "not-a-dir")
	if err := os.WriteFile(file, []byte("x"), 0o644); err != nil {
		t.Fatal(err)
	}

	// A file in the middle of the path makes MkdirAll fail for every uid, so the
	// test also holds when it runs as root in CI.
	err := ensureWritableDir(filepath.Join(file, "en"))
	if err == nil {
		t.Fatal("ensureWritableDir succeeded below a regular file")
	}
	for _, want := range []string{"create ", "/data/sounds", "--sounds-dir"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error %q does not mention %q", err, want)
		}
	}
}

func TestEnsureWritableDirCreates(t *testing.T) {
	dir := filepath.Join(t.TempDir(), "overlay", "en")
	if err := ensureWritableDir(dir); err != nil {
		t.Fatalf("ensureWritableDir: %v", err)
	}
	if st, err := os.Stat(dir); err != nil || !st.IsDir() {
		t.Fatalf("directory not created: %v", err)
	}
}

// fakeEngine satisfies Engine; Synthesize must never be reached in these tests.
type fakeEngine struct{}

func (fakeEngine) Available() error { return nil }

func (fakeEngine) Synthesize(fwsync.TTSIdentity, string) (time.Duration, error) {
	return 0, errors.New("fake engine must not run")
}

func (fakeEngine) Describe(fwsync.TTSIdentity) string { return "fake" }

// TestRenderTextNeedsText pins the only branch of RenderText that does not need
// the external engine.
func TestRenderTextNeedsText(t *testing.T) {
	if _, err := RenderText("   \n", fwsync.TTSIdentity{}, filepath.Join(t.TempDir(), "none.mp3"), fakeEngine{}, Encoder{}, nil); err == nil {
		t.Fatal("an empty text must be rejected before the engine runs")
	}
}

// TestRenderTextWithEngine exercises the real chain (ElevenLabs + ffmpeg) and is
// skipped unless a key and a voice id are in the environment, e.g.
//
//	ELEVENLABS_API_KEY=... OPENMOWER_TTS_VOICE=5gaEGrWSk4v20HMfIhgC go test ./internal/tts -run RenderText -v
//
// It asserts what the mower depends on: the firmware format, a real file and no
// cache sidecar for an ad-hoc render.
func TestRenderTextWithEngine(t *testing.T) {
	key := os.Getenv("ELEVENLABS_API_KEY")
	voice := os.Getenv("OPENMOWER_TTS_VOICE")
	if key == "" || voice == "" {
		t.Skip("ELEVENLABS_API_KEY and OPENMOWER_TTS_VOICE not set (no cloud render to test against)")
	}
	enc := Encoder{FFmpeg: "ffmpeg"}
	if err := enc.Available(); err != nil {
		t.Skipf("ffmpeg not available: %v", err)
	}
	eng := ElevenLabs{APIKey: key}

	dst := filepath.Join(t.TempDir(), "say.mp3")
	id := fwsync.TTSIdentity{
		Engine:          fwsync.DefaultTTSEngine,
		Model:           fwsync.DefaultTTSModel,
		Voice:           voice,
		Language:        "de",
		Bitrate:         "32k",
		LoudNorm:        true,
		Stability:       fwsync.DefaultTTSStability,
		SimilarityBoost: fwsync.DefaultTTSSimilarityBoost,
		Style:           fwsync.DefaultTTSStyle,
		UseSpeakerBoost: fwsync.DefaultTTSUseSpeakerBoost,
	}
	res, err := RenderText("Hallo Welt.", id, dst, eng, enc, t.Logf)
	if err != nil {
		t.Fatalf("RenderText: %v", err)
	}
	if !res.Info.Is16kMono() {
		t.Errorf("rendered %s, want 16 kHz mono", res.Info)
	}
	if st, err := os.Stat(dst); err != nil || st.Size() == 0 {
		t.Fatalf("no output file: %v", err)
	}
	// An ad-hoc render must not leave a cache sidecar behind.
	if _, err := os.Stat(dst + ".json"); err == nil {
		t.Error("RenderText wrote a sidecar; ad-hoc renders are not cached")
	}
}

// TestPendingIgnoresStaleOverlayCopy pins the --check side of the overlay fallback:
// a stale copy in the overlay must not report the render as pending while the image
// ships a matching one. That is the mower case that kept every German announcement
// silent - an ElevenLabs render in /data/sounds under a Cartesia definition.
func TestPendingIgnoresStaleOverlayCopy(t *testing.T) {
	dir := t.TempDir()
	cfgPath := filepath.Join(dir, "sounds_en.yaml")
	doc := `version: 1
language: en
sound_path: /nonexistent
tts:
  engine: elevenlabs
  elevenlabs:
    voice: voice-id
sounds:
  boot_complete:
    type: mp3
    volume: 100
    variants:
      - "one"
`
	if err := os.WriteFile(cfgPath, []byte(doc), 0o644); err != nil {
		t.Fatal(err)
	}
	cfg, err := fwsync.LoadConfig(cfgPath)
	if err != nil {
		t.Fatal(err)
	}
	jobs := cfg.RenderJobs()
	if len(jobs) != 1 {
		t.Fatalf("RenderJobs() = %d, want 1", len(jobs))
	}
	job := jobs[0]
	layout := fwsync.Layout{
		SoundPath: filepath.Join(dir, "image"),
		Overlay:   filepath.Join(dir, "overlay"),
		Language:  "en",
	}

	writeRender := func(base string, id fwsync.TTSIdentity) {
		t.Helper()
		langDir := filepath.Join(base, "en")
		if err := os.MkdirAll(langDir, 0o755); err != nil {
			t.Fatal(err)
		}
		mp3 := filepath.Join(langDir, job.File)
		data := []byte("mp3 rendered from " + id.Fingerprint())
		if err := os.WriteFile(mp3, data, 0o644); err != nil {
			t.Fatal(err)
		}
		info := fwsync.MP3Info{SampleRate: 16000, Channels: 1, BitrateKbps: 32, CBR: true}
		if err := fwsync.WriteSidecar(mp3,
			fwsync.NewSidecar(id, info, int64(len(data)), crc32.ChecksumIEEE(data))); err != nil {
			t.Fatal(err)
		}
	}

	stale := job.Identity
	stale.Text = "an older text"
	writeRender(layout.Overlay, stale)
	writeRender(layout.SoundPath, job.Identity)

	if got := Pending(cfg, layout); len(got) != 0 {
		t.Fatalf("Pending = %v, want none (the image carries a matching render)", got)
	}

	// Without the shipped copy only the stale overlay render is left, and that one
	// has to be reported.
	if err := os.RemoveAll(layout.SoundPath); err != nil {
		t.Fatal(err)
	}
	if got := Pending(cfg, layout); len(got) != 1 {
		t.Fatalf("Pending = %v, want the stale render reported", got)
	}
}

// TestRenderKeepGoing pins the failure policy: a failing variant must not stop the
// others, and the file it had before is left untouched. Without KeepGoing the
// first failure aborts (that is what an interactive `soundctl tts` should do).
func TestRenderKeepGoing(t *testing.T) {
	dir := t.TempDir()
	cfgPath := filepath.Join(dir, "sounds_en.yaml")
	doc := `version: 1
language: en
sound_path: /nonexistent
tts:
  engine: elevenlabs
  elevenlabs:
    voice: voice-id
sounds:
  boot_complete:
    type: mp3
    volume: 100
    variants:
      - "one"
      - "two"
`
	if err := os.WriteFile(cfgPath, []byte(doc), 0o644); err != nil {
		t.Fatal(err)
	}
	cfg, err := fwsync.LoadConfig(cfgPath)
	if err != nil {
		t.Fatal(err)
	}
	layout := fwsync.Layout{SoundPath: dir, Overlay: filepath.Join(dir, "overlay"), Language: "en"}

	// A previous render of the first variant must survive the failure.
	previous := filepath.Join(layout.OutDir(), "en_boot_complete_1.mp3")
	if err := os.MkdirAll(filepath.Dir(previous), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(previous, []byte("previous take"), 0o644); err != nil {
		t.Fatal(err)
	}

	results, err := Render(cfg, layout, fakeEngine{}, Encoder{FFmpeg: "ffmpeg"}, Options{
		OutDir:    layout.OutDir(),
		KeepGoing: true,
		Log:       func(string, ...any) {},
	})
	if err != nil {
		t.Fatalf("KeepGoing must not return an error: %v", err)
	}
	if len(results) != 2 {
		t.Fatalf("got %d result(s), want 2", len(results))
	}
	for _, r := range results {
		if !r.Failed {
			t.Errorf("%s: Failed = false, want true", r.Name)
		}
	}
	data, err := os.ReadFile(previous)
	if err != nil {
		t.Fatalf("read previous render: %v", err)
	}
	if string(data) != "previous take" {
		t.Errorf("previous file changed to %q", data)
	}

	// The default is fail-fast.
	if _, err := Render(cfg, layout, fakeEngine{}, Encoder{FFmpeg: "ffmpeg"}, Options{
		OutDir: layout.OutDir(),
		Log:    func(string, ...any) {},
	}); err == nil {
		t.Fatal("expected an error without KeepGoing")
	}
}
