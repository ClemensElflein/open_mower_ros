package fwsync

import (
	"hash/crc32"
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

// renderIdentity is the definition a render job describes in the FindRender tests.
var renderIdentity = TTSIdentity{
	Engine:   TTSEngineCartesia,
	Model:    "sonic-3.6",
	Voice:    "voice-id",
	Language: "de",
	Text:     "Hallo Welt.",
	Bitrate:  "32k",
	LoudNorm: true,
}

// renderFixture writes <dir>/<name> plus its sidecar, i.e. a render of id, and
// returns the MP3 path.
func renderFixture(t *testing.T, dir, name string, id TTSIdentity) string {
	t.Helper()
	if err := os.MkdirAll(dir, 0o755); err != nil {
		t.Fatal(err)
	}
	mp3 := filepath.Join(dir, name)
	data := []byte(name + "|" + id.Fingerprint())
	if err := os.WriteFile(mp3, data, 0o644); err != nil {
		t.Fatal(err)
	}
	info := MP3Info{SampleRate: 16000, Channels: 1, BitrateKbps: 32, CBR: true}
	if err := WriteSidecar(mp3, NewSidecar(id, info, int64(len(data)), crc32.ChecksumIEEE(data))); err != nil {
		t.Fatal(err)
	}
	return mp3
}

// TestLayoutFindRenderUsesTheValidCopy: an overlay render of an older definition
// (other engine, other text) must not hide the render the image ships.
func TestLayoutFindRenderUsesTheValidCopy(t *testing.T) {
	sound, overlay := t.TempDir(), t.TempDir()
	old := renderIdentity
	old.Engine, old.Voice, old.Text = TTSEngineElevenLabs, "old-voice", "Hallo alter Text."
	stale := renderFixture(t, filepath.Join(overlay, "de"), "de_x.mp3", old)
	good := renderFixture(t, filepath.Join(sound, "de"), "de_x.mp3", renderIdentity)

	layout := NewLayout(&Config{SoundPath: sound, Language: "de"}, overlay)
	path, shadowed, reason := layout.FindRender("de_x.mp3", renderIdentity)
	if path != good {
		t.Fatalf("FindRender = %q, want the shipped render %q", path, good)
	}
	if reason != "" {
		t.Errorf("reason = %q, want empty for a resolved render", reason)
	}
	if len(shadowed) != 1 || shadowed[0].Path != stale {
		t.Fatalf("shadowed = %+v, want exactly the stale overlay copy %s", shadowed, stale)
	}
	if !strings.Contains(shadowed[0].Reason, old.Fingerprint()) ||
		!strings.Contains(shadowed[0].Reason, renderIdentity.Fingerprint()) {
		t.Errorf("reason %q does not name both fingerprints", shadowed[0].Reason)
	}
}

// TestLayoutFindRenderOverlayWins: a matching overlay copy keeps precedence, so a
// customised render is still the one that gets uploaded.
func TestLayoutFindRenderOverlayWins(t *testing.T) {
	sound, overlay := t.TempDir(), t.TempDir()
	renderFixture(t, filepath.Join(sound, "de"), "de_x.mp3", renderIdentity)
	custom := renderFixture(t, filepath.Join(overlay, "de"), "de_x.mp3", renderIdentity)

	layout := NewLayout(&Config{SoundPath: sound, Language: "de"}, overlay)
	path, shadowed, _ := layout.FindRender("de_x.mp3", renderIdentity)
	if path != custom || len(shadowed) != 0 {
		t.Fatalf("FindRender = %q (shadowed %+v), want the overlay copy %s", path, shadowed, custom)
	}
}

// TestLayoutFindRenderNothingValid reports the most specific existing copy, so the
// caller can name the file that needs a re-render.
func TestLayoutFindRenderNothingValid(t *testing.T) {
	sound, overlay := t.TempDir(), t.TempDir()
	old := renderIdentity
	old.Text = "Hallo alter Text."
	stale := renderFixture(t, filepath.Join(overlay, "de"), "de_x.mp3", old)

	layout := NewLayout(&Config{SoundPath: sound, Language: "de"}, overlay)
	path, shadowed, reason := layout.FindRender("de_x.mp3", renderIdentity)
	if path != "" {
		t.Fatalf("FindRender = %q, want no usable copy", path)
	}
	if len(shadowed) != 1 || !strings.Contains(reason, stale) || !strings.Contains(reason, shadowed[0].Reason) {
		t.Fatalf("reason = %q (shadowed %+v), want the stale copy %s named", reason, shadowed, stale)
	}
}

// TestLayoutFindRenderMissing names every place that was searched, like Find.
func TestLayoutFindRenderMissing(t *testing.T) {
	layout, sound, overlay := newLayoutDirs(t)
	path, shadowed, reason := layout.FindRender("nope.mp3", renderIdentity)
	if path != "" || len(shadowed) != 0 {
		t.Fatalf("FindRender = %q (shadowed %+v), want nothing", path, shadowed)
	}
	for _, p := range []string{
		filepath.Join(overlay, "en", "nope.mp3"),
		filepath.Join(sound, "en", "nope.mp3"),
		filepath.Join(overlay, "nope.mp3"),
		filepath.Join(sound, "nope.mp3"),
	} {
		if !strings.Contains(reason, p) {
			t.Fatalf("reason %q does not mention %s", reason, p)
		}
	}
}
