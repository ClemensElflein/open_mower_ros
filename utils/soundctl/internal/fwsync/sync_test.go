package fwsync

import (
	"bytes"
	"context"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

type checkFileService struct {
	exists uint8
}

func (fs checkFileService) FileExists(context.Context, string, uint32) (uint8, error) {
	return fs.exists, nil
}

func (checkFileService) FileWrite(context.Context, string, uint32, []byte, uint32) (int32, error) {
	return 0, fmt.Errorf("unexpected FileWrite during --check")
}

func TestUploadCheckReportsOnlyPendingFilesAtInfo(t *testing.T) {
	var output bytes.Buffer
	previous := slog.Default()
	t.Cleanup(func() { slog.SetDefault(previous) })
	slog.SetDefault(slog.New(slog.NewTextHandler(&output, nil)))

	mp3 := filepath.Join("..", "..", "..", "..", "sounds", "zz_ping.mp3")
	for _, exists := range []uint8{0, 1} {
		output.Reset()
		if err := uploadFile(t.Context(), checkFileService{exists}, mp3, "ping.mp3", true); err != nil {
			t.Fatal(err)
		}
		if exists == 0 && (!strings.Contains(output.String(), "would upload") || !strings.Contains(output.String(), "ping.mp3")) {
			t.Errorf("missing file absent from check report: %q", output.String())
		}
		if exists == 1 && output.Len() != 0 {
			t.Errorf("matching file reported at INFO: %q", output.String())
		}
	}
}

func TestSyncRejectsOversizedDefinitionsBeforeDiscovery(t *testing.T) {
	var doc strings.Builder
	doc.WriteString(validBase)
	for i := 0; i < 512; i++ {
		fmt.Fprintf(&doc, "      - {file: clip_%03d.mp3}\n", i)
	}
	configPath := filepath.Join(t.TempDir(), "sounds.yaml")
	if err := os.WriteFile(configPath, []byte(doc.String()), 0o644); err != nil {
		t.Fatal(err)
	}

	// Discovery would fail on the cancelled context. Both modes must report the
	// local packet-size error before attempting a connection.
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for _, check := range []bool{false, true} {
		err := Sync(ctx, Options{
			ConfigPath:   configPath,
			SoundIDsPath: filepath.Join("..", "..", "..", "..", "services", "sound_service.json"),
			Check:        check,
		})
		if err == nil || !strings.Contains(err.Error(), "single packet carries") {
			t.Errorf("Sync(check=%t) = %v, want a definitions size error", check, err)
		}
	}
}

// TestReconnectTimeoutSpansSeveralAdvertisements: the reconnect that follows the
// definitions transaction has to survive a lost UDP packet. A claimed service
// advertises only every 10 s, so its budget must be a multiple of the probe budget -
// using the probe budget itself loses the master volume to one dropped packet.
func TestReconnectTimeoutSpansSeveralAdvertisements(t *testing.T) {
	probe := soundCheckTimeout(Options{})
	if got, want := reconnectTimeout(Options{}), 3*probe; got < want {
		t.Fatalf("reconnectTimeout = %s, want at least %s (several advertisement periods)", got, want)
	}
	// An explicit --sound-check-timeout scales it too, so raising the probe budget
	// never silently yields a shorter reconnect.
	if got := reconnectTimeout(Options{SoundCheckTimeout: 5 * time.Second}); got != 15*time.Second {
		t.Fatalf("reconnectTimeout(--sound-check-timeout 5s) = %s, want 15s", got)
	}
}

// TestResolveRendersFallsBackToTheShippedRender: a stale overlay render must not
// hide the copy the image ships. The shipped render is uploaded, not skipped.
func TestResolveRendersFallsBackToTheShippedRender(t *testing.T) {
	doc := "version: 1\nlanguage: de\nsound_path: /nonexistent\n" + validTTS +
		"sounds:\n  boot_complete:\n    type: mp3\n    volume: 80\n    variants:\n      - \"Hallo Welt.\"\n"
	cfg := loadFixture(t, doc)
	jobs := cfg.RenderJobs()
	if len(jobs) != 1 {
		t.Fatalf("RenderJobs() = %d, want 1", len(jobs))
	}
	job := jobs[0]

	root := t.TempDir()
	layout := Layout{
		SoundPath: filepath.Join(root, "image"),
		Overlay:   filepath.Join(root, "overlay"),
		Language:  "de",
	}
	old := job.Identity
	old.Engine, old.Text = TTSEngineElevenLabs, "Hallo alter Text."
	staleCopy := renderFixture(t, filepath.Join(layout.Overlay, "de"), job.File, old)
	shipped := renderFixture(t, filepath.Join(layout.SoundPath, "de"), job.File, job.Identity)

	use, stale := resolveRenders(layout, cfg)
	if len(stale) != 0 {
		t.Fatalf("stale = %v, want none (the image carries a matching render)", stale)
	}
	if use[job.File] != shipped {
		t.Fatalf("use[%s] = %q, want the shipped render %q", job.File, use[job.File], shipped)
	}

	// With only the stale overlay copy left, sync must skip the file and name it.
	if err := os.RemoveAll(layout.SoundPath); err != nil {
		t.Fatal(err)
	}
	use, stale = resolveRenders(layout, cfg)
	if len(use) != 0 {
		t.Fatalf("use = %v, want nothing uploadable", use)
	}
	if reason := stale[job.File]; !strings.Contains(reason, staleCopy) {
		t.Fatalf("stale[%s] = %q, want it to name %s", job.File, reason, staleCopy)
	}
}
