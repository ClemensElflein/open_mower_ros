package tts

import (
	"errors"
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
