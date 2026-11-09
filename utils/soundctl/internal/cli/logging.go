// Package cli contains the soundctl command-line interface.
package cli

import (
	"bytes"
	"fmt"
	"io"
	"log"
	"strings"
	"sync"

	"github.com/spf13/cobra"
)

// logOptions are the knobs the ROS integration needs. A manual run wants the
// timestamp, plain output on a terminal and no prefix, while sound_sync pipes the
// output into the ROS log, which already stamps every line and shows which node
// said it - there the CLI's own date/time is noise, and a prefix makes the line
// attributable at a glance.
type logOptions struct {
	// time keeps the "2006/01/02 15:04:05" prefix of every log line.
	time bool
	// prefix is prepended to every log and progress line. Empty means "no prefix";
	// a missing trailing space is added.
	prefix string
	// noColor disables the escape sequences even on a terminal.
	noColor bool
}

// register adds the flags. They are persistent, so they work before or after the
// subcommand name and every command inherits them.
func (o *logOptions) register(cmd *cobra.Command) {
	cmd.PersistentFlags().BoolVar(&o.time, "log-time", true,
		"prefix log lines with the date/time (disable when a supervisor such as the ROS log already stamps them)")
	cmd.PersistentFlags().StringVar(&o.prefix, "log-prefix", "",
		"prefix every output line with this text, e.g. '[soundctl] ' (a missing space is added)")
	cmd.PersistentFlags().BoolVar(&o.noColor, "no-color", false,
		"never use colours, even on a terminal (also off for a piped/redirected run, $NO_COLOR and TERM=dumb)")
}

// apply installs the requested output format. It must run after the flags are
// parsed and before a command prints anything, hence PersistentPreRun.
//
// Three sinks are configured, in this order:
//
//  1. the colour decision, which has to see the real streams before step 3 wraps
//     them (a prefix writer is not an *os.File, so it cannot be asked afterwards);
//  2. the log package, which is the sink of slog's default handler;
//  3. cobra's own writers, which carry the progress lines a command prints itself
//     and the one error line Execute prints after the run.
//
// So a piped run gets its prefix on both kinds of line and no escape sequences at
// all, and a terminal run keeps its colours.
func (o *logOptions) apply(cmd *cobra.Command) {
	// 1. Colour, while the writers are still os.Stdout/os.Stderr.
	st := style{
		out: colorEnabled(cmd.OutOrStdout(), o.noColor),
		err: colorEnabled(cmd.ErrOrStderr(), o.noColor),
	}
	withStyle(cmd, st)
	// The error line is printed by Execute on the root after the run, so the root
	// needs the style too (a subcommand's context cannot be reached from there).
	if root := cmd.Root(); root != cmd {
		withStyle(root, st)
	}

	prefix := o.prefix
	if prefix != "" && !strings.HasSuffix(prefix, " ") {
		prefix += " "
	}

	// 2. slog: its default handler formats through the log package, so the flags
	// and the prefix of that package are what shapes the line, and wrapping its
	// writer is where the level token gets its colour.
	if !o.time {
		log.SetFlags(0)
	}
	log.SetPrefix(prefix)
	log.SetOutput(newLevelColorWriter(log.Writer(), st.err, prefix))

	// 3. The root command owns the writers: a subcommand's OutOrStdout/ErrOrStderr
	// walks up to it, so wrapping there also covers the error line.
	if prefix == "" {
		return
	}
	root := cmd.Root()
	root.SetOut(newPrefixWriter(root.OutOrStdout(), prefix))
	root.SetErr(newPrefixWriter(root.ErrOrStderr(), prefix))
}

// newPrefixWriter wraps w so that every line it receives starts with prefix. It is
// what makes the CLI's own progress output look like the log lines of a piped run.
//
// The logger has a prefix option of its own (log.SetPrefix, step 2 of apply) and it
// covers the slog lines, so this wrapper is not about those: it covers the output a
// command prints itself to cmd.OutOrStdout(), which never passes through a logger -
// the "uploading …" progress lines, the `validate` report and the per-variant render
// results. Cobra's own output (the error line, a generated usage text) reaches the
// same writers, so wrapping them is the one hook that covers all of it.
func newPrefixWriter(w io.Writer, prefix string) *prefixWriter {
	return &prefixWriter{w: w, prefix: prefix, bol: true}
}

type prefixWriter struct {
	w      io.Writer
	prefix string
	bol    bool       // at the beginning of a line
	mu     sync.Mutex // commands may print from a goroutine
}

// Write prefixes before the first byte of every line. It reports only the bytes of
// b (not the prefix), as io.Writer requires.
func (p *prefixWriter) Write(b []byte) (int, error) {
	p.mu.Lock()
	defer p.mu.Unlock()

	written := 0
	for len(b) > 0 {
		if p.bol {
			if _, err := io.WriteString(p.w, p.prefix); err != nil {
				return written, err
			}
			p.bol = false
		}
		i := bytes.IndexByte(b, '\n')
		if i < 0 {
			n, err := p.w.Write(b)
			return written + n, err
		}
		n, err := p.w.Write(b[:i+1])
		written += n
		if err != nil {
			return written, err
		}
		p.bol = true
		b = b[i+1:]
	}
	return written, nil
}

// printError reports a failed run through the (possibly prefixed) writer, so the
// last line of a piped run is attributed like every other line.
func printError(cmd *cobra.Command, err error) {
	fmt.Fprintln(cmd.ErrOrStderr(), styleFor(cmd).error("Error:"), err)
}
