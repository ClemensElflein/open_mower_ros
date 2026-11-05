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

// logOptions are the two knobs the ROS integration needs. A manual run wants the
// timestamp and no prefix, while sound_sync pipes the output into the ROS log,
// which already stamps every line and shows which node said it - there the CLI's
// own date/time is noise, and a prefix makes the line attributable at a glance.
type logOptions struct {
	// time keeps the "2006/01/02 15:04:05" prefix of every log line.
	time bool
	// prefix is prepended to every log and progress line. Empty means "no prefix";
	// a missing trailing space is added.
	prefix string
}

// register adds the flags. They are persistent, so they work before or after the
// subcommand name and every command inherits them.
func (o *logOptions) register(cmd *cobra.Command) {
	cmd.PersistentFlags().BoolVar(&o.time, "log-time", true,
		"prefix log lines with the date/time (disable when a supervisor such as the ROS log already stamps them)")
	cmd.PersistentFlags().StringVar(&o.prefix, "log-prefix", "",
		"prefix every output line with this text, e.g. '[soundctl] ' (a missing space is added)")
}

// apply installs the requested log format. It must run after the flags are parsed
// and before a command logs anything, hence PersistentPreRun.
//
// slog's default handler formats through the log package, so log.SetFlags removes
// the timestamp without replacing the handler, and the rest of the line looks
// exactly as it does in a manual run. Progress lines a command prints itself do not
// go through slog, so the writers are wrapped as well - otherwise a piped run would
// have a prefix on its log lines and none on its "uploading …" lines.
func (o *logOptions) apply(cmd *cobra.Command) {
	if !o.time {
		log.SetFlags(0)
	}
	prefix := o.prefix
	if prefix == "" {
		return
	}
	if !strings.HasSuffix(prefix, " ") {
		prefix += " "
	}
	log.SetPrefix(prefix)
	// The root command owns the writers: a subcommand's OutOrStdout/ErrOrStderr walks
	// up to it, so this also covers the error line Execute prints after the run.
	root := cmd.Root()
	root.SetOut(newPrefixWriter(root.OutOrStdout(), prefix))
	root.SetErr(newPrefixWriter(root.ErrOrStderr(), prefix))
}

// newPrefixWriter wraps w so that every line it receives starts with prefix. It is
// what makes the CLI's own progress output look like the log lines of a piped run.
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
	fmt.Fprintln(cmd.ErrOrStderr(), "Error:", err)
}
