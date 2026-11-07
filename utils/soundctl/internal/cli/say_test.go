package cli

import (
	"strings"
	"testing"
)

func TestPlayerCommand(t *testing.T) {
	cases := []struct {
		player string
		want   []string
	}{
		{defaultPlayer, []string{"-nodisp", "-autoexit", "-loglevel", "error", "/tmp/a.mp3"}},
		{"mpg123 -q", []string{"-q", "/tmp/a.mp3"}},
		{"paplay", []string{"/tmp/a.mp3"}},
	}
	for _, c := range cases {
		bin, args, err := playerCommand(c.player, "/tmp/a.mp3")
		if err != nil {
			t.Fatalf("%q: %v", c.player, err)
		}
		if bin == "" {
			t.Fatalf("%q: empty binary", c.player)
		}
		if strings.Join(args, " ") != strings.Join(c.want, " ") {
			t.Errorf("%q: args %v, want %v", c.player, args, c.want)
		}
	}

	if _, _, err := playerCommand("   ", "/tmp/a.mp3"); err == nil {
		t.Error("an empty --player must be rejected")
	}
}

// TestSayText pins how the positional arguments become the spoken text: quotes
// are optional, "-" reads stdin.
func TestSayText(t *testing.T) {
	if got, err := sayText([]string{"Hallo", "Welt"}, strings.NewReader("")); err != nil || got != "Hallo Welt" {
		t.Errorf("sayText(joined) = %q/%v, want \"Hallo Welt\"", got, err)
	}
	if got, err := sayText([]string{"-"}, strings.NewReader("  Akku fast leer\n")); err != nil || got != "Akku fast leer" {
		t.Errorf("sayText(stdin) = %q/%v, want \"Akku fast leer\"", got, err)
	}
	if _, err := sayText([]string{"   "}, strings.NewReader("")); err == nil {
		t.Error("empty text must be rejected")
	}
}

// TestSayValidatesBeforeRendering: a missing voice, an out-of-range voice setting,
// a bad bitrate or language has to fail before the engine (and therefore the API
// key) is looked for, so the message is the useful one and nobody waits for a
// render that cannot work.
func TestSayValidatesBeforeRendering(t *testing.T) {
	cases := []struct {
		args []string
		want string
	}{
		{[]string{"Hallo"}, "voice must not be empty"},
		{[]string{"Hallo", "--voice", "x", "--stability", "1.5"}, "stability must be 0..1"},
		{[]string{"Hallo", "--voice", "x", "--bitrate", "320k"}, "bitrate"},
		{[]string{"Hallo", "--voice", "x", "--lang", "DE"}, "language"},
	}
	for _, c := range cases {
		cmd := newSayCmd()
		cmd.SetArgs(c.args)
		cmd.SetOut(discard{})
		cmd.SetErr(discard{})
		err := cmd.Execute()
		if err == nil {
			t.Fatalf("%v: no error", c.args)
		}
		if !strings.Contains(err.Error(), c.want) {
			t.Errorf("%v: error %q does not mention %q", c.args, err, c.want)
		}
		// The key lookup comes after the validation: reaching it would mean this
		// case passes because of a missing key instead of the bad value.
		if strings.Contains(err.Error(), "API key") {
			t.Errorf("%v: validation ran after the key lookup (%q)", c.args, err)
		}
	}
}

// discard swallows command output in tests.
type discard struct{}

func (discard) Write(p []byte) (int, error) { return len(p), nil }
