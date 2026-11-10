package cli

import (
	"bytes"
	"errors"
	"fmt"
	"log"
	"log/slog"
	"strings"
	"testing"

	"github.com/spf13/cobra"
)

// captureStandardLogger points the log package (which slog's default handler writes
// through) at buf for the duration of the test and restores it afterwards.
func captureStandardLogger(t *testing.T, buf *bytes.Buffer) {
	t.Helper()
	oldOut, oldFlags, oldPrefix := log.Writer(), log.Flags(), log.Prefix()
	log.SetOutput(buf)
	t.Cleanup(func() {
		log.SetOutput(oldOut)
		log.SetFlags(oldFlags)
		log.SetPrefix(oldPrefix)
	})
}

// TestPrefixWriterPrefixesEveryLine: the writer is what makes the progress output of
// a piped run attributable, so every line must start with the prefix - also one that
// is split across two writes - and Write must report the input bytes only.
func TestPrefixWriterPrefixesEveryLine(t *testing.T) {
	var buf bytes.Buffer
	w := newPrefixWriter(&buf, "[soundctl] ")

	in := "one\ntw"
	if n, err := w.Write([]byte(in)); err != nil || n != len(in) {
		t.Fatalf("Write = %d, %v, want %d, nil", n, err, len(in))
	}
	if _, err := w.Write([]byte("o\nthree\n")); err != nil {
		t.Fatal(err)
	}

	want := "[soundctl] one\n[soundctl] two\n[soundctl] three\n"
	if got := buf.String(); got != want {
		t.Fatalf("output = %q, want %q", got, want)
	}
}

// TestLogOptionsApplyForROS pins what sound_sync asks for: no timestamp of its own
// (the ROS log has one) and a prefix on every line, no matter whether it comes from
// slog or from a command's own printing.
func TestLogOptionsApplyForROS(t *testing.T) {
	var logBuf bytes.Buffer
	captureStandardLogger(t, &logBuf)

	// A subcommand of a root, like the real graph: apply() wraps the root's writers,
	// and the subcommand finds them through the parent chain.
	root := &cobra.Command{Use: "soundctl"}
	cmd := &cobra.Command{Use: "sync"}
	root.AddCommand(cmd)
	var out, errOut bytes.Buffer
	root.SetOut(&out)
	root.SetErr(&errOut)

	// No trailing space on purpose: apply() adds it.
	opts := logOptions{time: false, prefix: "[soundctl sync]"}
	opts.apply(cmd)

	slog.Info("SoundService available", "addr", "172.16.78.150:49153",
		"hint", "pass it to 'soundctl play --addr' for instant playback")
	fmt.Fprintln(cmd.OutOrStdout(), "uploading file=de_x.mp3")
	printError(root, errors.New("boom"))

	// The prefix comes first and INFO follows directly, which also proves the
	// timestamp is gone (it would sit between them).
	wantLog := `[soundctl sync] INFO SoundService available addr=172.16.78.150:49153` +
		` hint="pass it to 'soundctl play --addr' for instant playback"` + "\n"
	if got := logBuf.String(); got != wantLog {
		t.Fatalf("log line = %q, want %q", got, wantLog)
	}
	if got, want := out.String(), "[soundctl sync] uploading file=de_x.mp3\n"; got != want {
		t.Fatalf("progress line = %q, want %q", got, want)
	}
	if got, want := errOut.String(), "[soundctl sync] Error: boom\n"; got != want {
		t.Fatalf("error line = %q, want %q", got, want)
	}
}

// TestLogOptionsApplyDefaultKeepsTimeAndWriters: a manual run must look exactly like
// before - timestamp on the log lines, no prefix anywhere.
func TestLogOptionsApplyDefault(t *testing.T) {
	var logBuf bytes.Buffer
	captureStandardLogger(t, &logBuf)

	cmd := &cobra.Command{Use: "sync"}
	var out bytes.Buffer
	cmd.SetOut(&out)

	opts := logOptions{time: true}
	opts.apply(cmd)

	slog.Info("hello")
	if line := logBuf.String(); !strings.Contains(line, " INFO hello\n") || !strings.Contains(line, "/") {
		t.Fatalf("log line = %q, want a date/time before INFO", line)
	}
	fmt.Fprintln(cmd.OutOrStdout(), "progress")
	if got := out.String(); got != "progress\n" {
		t.Fatalf("progress line = %q, want it untouched", got)
	}
}
