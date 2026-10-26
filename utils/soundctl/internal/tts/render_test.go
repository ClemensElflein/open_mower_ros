package tts

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
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
