package tts

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

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

// TestRenderTextNeedsText pins the only branch of RenderText that does not need
// the external engine.
func TestRenderTextNeedsText(t *testing.T) {
	if _, err := RenderText("   \n", fwsync.TTSIdentity{}, filepath.Join(t.TempDir(), "none.mp3"), Sherpa{}, Encoder{}, nil); err == nil {
		t.Fatal("an empty text must be rejected before the engine runs")
	}
}

// TestRenderTextWithEngine exercises the real chain (engine + ffmpeg) and is
// skipped unless a local installation is pointed at, e.g.
//
//	OPENMOWER_TTS_DIR=/opt/open_mower_ros/tts go test ./internal/tts -run RenderText -v
//
// It asserts what the mower depends on: the firmware format and a real file.
func TestRenderTextWithEngine(t *testing.T) {
	dir := os.Getenv("OPENMOWER_TTS_DIR")
	if dir == "" {
		t.Skip("OPENMOWER_TTS_DIR not set (no local engine to test against)")
	}
	enc := Encoder{FFmpeg: "ffmpeg"}
	if err := enc.Available(); err != nil {
		t.Skipf("ffmpeg not available: %v", err)
	}
	eng := Sherpa{
		Bin:      filepath.Join(dir, "bin", "sherpa-onnx-offline-tts"),
		LibDir:   filepath.Join(dir, "lib"),
		ModelDir: filepath.Join(dir, "models", fwsync.DefaultTTSModel),
	}
	if err := eng.Available(); err != nil {
		t.Skipf("engine not installed in %s: %v", dir, err)
	}

	dst := filepath.Join(t.TempDir(), "say.mp3")
	id := fwsync.TTSIdentity{
		Engine:   fwsync.DefaultTTSEngine,
		Model:    fwsync.DefaultTTSModel,
		Language: "de",
		Bitrate:  "32k",
		Sid:      fwsync.DefaultTTSSid,
		Speed:    fwsync.DefaultTTSSpeed,
		LoudNorm: true,
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
