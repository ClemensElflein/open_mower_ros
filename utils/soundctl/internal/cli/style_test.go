package cli

import (
	"bytes"
	"os"
	"testing"
)

// TestColorEnabled pins every reason to stay plain. The one case a unit test
// cannot reach is a real terminal (go test's stdout is not one), which is why the
// integration check is `soundctl --no-color | cat` and `soundctl | cat` by hand.
func TestColorEnabled(t *testing.T) {
	// A regular file is not a terminal either, so it must stay plain.
	f, err := os.CreateTemp(t.TempDir(), "out")
	if err != nil {
		t.Fatal(err)
	}
	defer func() { _ = f.Close() }()

	cases := []struct {
		name    string
		w       any // what colorEnabled receives
		noColor bool
		env     map[string]string
		want    bool
	}{
		{"flag wins", f, true, nil, false},
		{"NO_COLOR set", f, false, map[string]string{"NO_COLOR": "1"}, false},
		{"NO_COLOR empty is ignored", f, false, map[string]string{"NO_COLOR": ""}, false}, // still not a TTY
		{"TERM=dumb", f, false, map[string]string{"TERM": "dumb"}, false},
		{"a redirected file is not a terminal", f, false, nil, false},
		{"a buffer is never a terminal", &bytes.Buffer{}, false, nil, false},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			for k, v := range c.env {
				t.Setenv(k, v)
			}
			if c.env == nil {
				t.Setenv("NO_COLOR", "")
				t.Setenv("TERM", "xterm")
			}
			var w = c.w.(interface{ Write([]byte) (int, error) })
			if got := colorEnabled(w, c.noColor); got != c.want {
				t.Fatalf("colorEnabled = %v, want %v", got, c.want)
			}
		})
	}
}

// TestLevelColorWriterLeavesPlainOutputAlone: with colours off the log lines must
// be byte-identical to what they were before the feature existed.
func TestLevelColorWriterLeavesPlainOutputAlone(t *testing.T) {
	var buf bytes.Buffer
	w := newLevelColorWriter(&buf, false, "[soundctl sync] ")

	in := "[soundctl sync] INFO uploading file=x.mp3\n"
	if n, err := w.Write([]byte(in)); err != nil || n != len(in) {
		t.Fatalf("Write = %d, %v, want %d, nil", n, err, len(in))
	}
	if got := buf.String(); got != in {
		t.Fatalf("output = %q, want it untouched", got)
	}
}

// TestLevelColorWriterColorsTheLevelToken: the level is the only thing that gains
// colour - the prefix, the message and the attributes stay as they are, so a log
// consumer that strips SGR sequences sees the uncoloured line.
func TestLevelColorWriterColorsTheLevelToken(t *testing.T) {
	var buf bytes.Buffer
	w := newLevelColorWriter(&buf, true, "[soundctl sync] ")

	in := "[soundctl sync] INFO uploading file=x.mp3\n"
	if n, err := w.Write([]byte(in)); err != nil || n != len(in) {
		t.Fatalf("Write = %d, %v, want %d, nil (io.Writer reports its input)", n, err, len(in))
	}
	want := "[soundctl sync] " + colorCyan + "INFO" + colorReset + " uploading file=x.mp3\n"
	if got := buf.String(); got != want {
		t.Fatalf("output = %q, want %q", got, want)
	}

	// A continuation line (a message with a newline) has no token and stays plain.
	buf.Reset()
	if _, err := w.Write([]byte("ERROR first\nsecond line\n")); err != nil {
		t.Fatal(err)
	}
	want = colorRed + "ERROR" + colorReset + " first\nsecond line\n"
	if got := buf.String(); got != want {
		t.Fatalf("multi-line output = %q, want %q", got, want)
	}

	// With --log-time (the default) the log package's timestamp sits in front of
	// the level; it must stay plain and the token must still be found.
	buf.Reset()
	if _, err := w.Write([]byte("[soundctl sync] 2026/11/08 22:21:04 ERROR boom\n")); err != nil {
		t.Fatal(err)
	}
	want = "[soundctl sync] 2026/11/08 22:21:04 " + colorRed + "ERROR" + colorReset + " boom\n"
	if got := buf.String(); got != want {
		t.Fatalf("timestamped output = %q, want %q", got, want)
	}
}

// TestNewLevelColorWriterIsIdempotent: apply() wraps log.Writer(), and wrapping a
// second time must not nest the colouring (it would double the escapes).
func TestNewLevelColorWriterIsIdempotent(t *testing.T) {
	var buf bytes.Buffer
	first := newLevelColorWriter(&buf, true, "p ")
	second := newLevelColorWriter(first, true, "p ")

	if _, err := second.Write([]byte("p INFO x\n")); err != nil {
		t.Fatal(err)
	}
	want := "p " + colorCyan + "INFO" + colorReset + " x\n"
	if got := buf.String(); got != want {
		t.Fatalf("output = %q, want %q", got, want)
	}
}

// TestNoColorFlagIsAcceptedOnASubcommand: sound_sync passes the flag on the
// subcommand line, so it has to be inherited exactly like --log-prefix is.
func TestNoColorFlagIsAcceptedOnASubcommand(t *testing.T) {
	var logBuf bytes.Buffer
	captureStandardLogger(t, &logBuf)

	root := newRootCmd()
	root.SetOut(&bytes.Buffer{})
	root.SetErr(&bytes.Buffer{})
	root.SetArgs([]string{"version", "--no-color"})
	if err := root.Execute(); err != nil {
		t.Fatalf("a subcommand rejected --no-color: %v", err)
	}
}
