package fwsync

import (
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

// newLayoutDirs builds
//
//	sound/         (image)
//	sound/en/      (pre-rendered by the image)
//	overlay/       (customised shared files)
//	overlay/en/    (customised / re-rendered files)
//
// and returns the layout plus the four directories.
func newLayoutDirs(t *testing.T) (Layout, string, string) {
	t.Helper()
	sound := t.TempDir()
	overlay := t.TempDir()
	if err := os.MkdirAll(filepath.Join(sound, "en"), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(filepath.Join(overlay, "en"), 0o755); err != nil {
		t.Fatal(err)
	}
	write := func(dir, name string) {
		t.Helper()
		if err := os.WriteFile(filepath.Join(dir, name), []byte(name), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	write(sound, "shared.mp3")                    // only shipped
	write(sound, filepath.Join("en", "lang.mp3")) // only shipped, language specific
	write(overlay, "custom-shared.mp3")           // only customised
	write(overlay, filepath.Join("en", "custom-lang.mp3"))
	return NewLayout(&Config{SoundPath: sound, Language: "en"}, overlay), sound, overlay
}

func TestLayoutCandidatesOrder(t *testing.T) {
	layout, sound, overlay := newLayoutDirs(t)
	want := []string{
		filepath.Join(overlay, "en", "x.mp3"), // customised render, this language
		filepath.Join(sound, "en", "x.mp3"),   // pre-rendered, this language
		filepath.Join(overlay, "x.mp3"),       // customised shared
		filepath.Join(sound, "x.mp3"),         // shipped
	}
	if got := layout.Candidates("x.mp3"); !reflect.DeepEqual(got, want) {
		t.Fatalf("Candidates =\n%s\nwant\n%s", strings.Join(got, "\n"), strings.Join(want, "\n"))
	}
}

func TestLayoutCandidatesWithoutOverlay(t *testing.T) {
	sound := t.TempDir()
	layout := NewLayout(&Config{SoundPath: sound, Language: "de"}, "")
	want := []string{filepath.Join(sound, "de", "x.mp3"), filepath.Join(sound, "x.mp3")}
	if got := layout.Candidates("x.mp3"); !reflect.DeepEqual(got, want) {
		t.Fatalf("Candidates = %v, want %v", got, want)
	}
}

func TestLayoutFindPrecedence(t *testing.T) {
	layout, sound, overlay := newLayoutDirs(t)

	cases := map[string]string{
		"lang.mp3":          filepath.Join(sound, "en", "lang.mp3"),
		"shared.mp3":        filepath.Join(sound, "shared.mp3"),
		"custom-lang.mp3":   filepath.Join(overlay, "en", "custom-lang.mp3"),
		"custom-shared.mp3": filepath.Join(overlay, "custom-shared.mp3"),
	}
	for file, want := range cases {
		got, err := layout.Find(file)
		if err != nil {
			t.Fatalf("Find(%q): %v", file, err)
		}
		if got != want {
			t.Fatalf("Find(%q) = %s, want %s", file, got, want)
		}
	}

	// A customised file in the overlay must win over the shipped one.
	if err := os.WriteFile(filepath.Join(sound, "en", "lang.mp3"), []byte("shipped"), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(overlay, "en", "lang.mp3"), []byte("custom"), 0o644); err != nil {
		t.Fatal(err)
	}
	got, err := layout.Find("lang.mp3")
	if err != nil {
		t.Fatal(err)
	}
	if got != filepath.Join(overlay, "en", "lang.mp3") {
		t.Fatalf("Find(lang.mp3) = %s, want the overlay copy", got)
	}
}

func TestLayoutFindMissing(t *testing.T) {
	layout, sound, overlay := newLayoutDirs(t)
	_, err := layout.Find("nope.mp3")
	if err == nil {
		t.Fatal("Find returned no error for a missing file")
	}
	// The message has to name every place that was searched, otherwise a user
	// cannot tell why a file "is there" but is not found.
	for _, p := range []string{
		filepath.Join(overlay, "en", "nope.mp3"),
		filepath.Join(sound, "en", "nope.mp3"),
		filepath.Join(overlay, "nope.mp3"),
		filepath.Join(sound, "nope.mp3"),
	} {
		if !strings.Contains(err.Error(), p) {
			t.Fatalf("error %q does not mention %s", err, p)
		}
	}
}

func TestLayoutOutDir(t *testing.T) {
	cases := []struct {
		overlay, sound, lang, want string
	}{
		{"", "/img", "en", "/img/en"},       // no overlay: write next to the image
		{"/data", "/img", "en", "/data/en"}, // overlay: renders survive an update
		{"/data", "/img", "", "/data"},      // no language: the overlay root
		{"", "/img", "", "/img"},            // neither
	}
	for _, c := range cases {
		layout := Layout{SoundPath: c.sound, Overlay: c.overlay, Language: c.lang}
		if got := layout.OutDir(); got != c.want {
			t.Fatalf("OutDir(overlay=%q, lang=%q) = %s, want %s", c.overlay, c.lang, got, c.want)
		}
	}
}
