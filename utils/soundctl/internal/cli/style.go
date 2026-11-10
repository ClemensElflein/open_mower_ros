// Package cli contains the soundctl command-line interface.
package cli

import (
	"bytes"
	"context"
	"io"
	"os"

	"github.com/spf13/cobra"
	"golang.org/x/term"
)

// ANSI SGR sequences. Only the ones the CLI actually needs: colour is a hint for
// a human reading a terminal, never part of the data (see colorEnabled).
const (
	colorReset  = "\x1b[0m"
	colorRed    = "\x1b[31m"
	colorGreen  = "\x1b[32m"
	colorYellow = "\x1b[33m"
	colorCyan   = "\x1b[36m"
	colorGray   = "\x1b[90m"
	colorBold   = "\x1b[1m"
)

// levelColors colour the level token of a slog line. slog's default handler
// formats "LEVEL msg key=value", so the token is the first word.
var levelColors = map[string]string{
	"DEBUG": colorGray,
	"INFO":  colorCyan,
	"WARN":  colorYellow,
	"ERROR": colorRed,
}

// levelScan bounds the search for the level token: the log package's own
// "2006/01/02 15:04:05 " prefix is 20 bytes and a level is at most 5, so the token
// is always within the first 32 bytes of the part after our prefix.
const levelScan = 32

// style paints single-line fragments of command output. The zero value paints
// nothing, which is what a command sees when it runs outside the configured root
// (unit tests) or when colour is off.
type style struct {
	// out and err are kept apart because a shell can redirect them separately
	// (`soundctl validate >ok.log 2>err.log`).
	out bool
	err bool
}

// ok, warn, head and error paint a status marker or a heading. Nothing here may
// span a newline: the prefix writer adds its prefix at every line start and would
// end up inside an escape sequence.
func (s style) ok(text string) string    { return s.paint(s.out, colorGreen, text) }
func (s style) warn(text string) string  { return s.paint(s.out, colorYellow, text) }
func (s style) head(text string) string  { return s.paint(s.out, colorBold, text) }
func (s style) error(text string) string { return s.paint(s.err, colorRed, text) }

func (s style) paint(enabled bool, color, text string) string {
	if !enabled || text == "" {
		return text
	}
	return color + text + colorReset
}

// styleKey is the context key the resolved style travels under. The commands are
// closures that only receive a *cobra.Command, so the context is how the
// PersistentPreRun result reaches them without package-level state.
type styleKey struct{}

// styleFor returns the style the root resolved for this run. A command executed
// without the root (a test calling newSayCmd().Execute()) gets the zero value,
// i.e. plain output.
func styleFor(cmd *cobra.Command) style {
	if cmd == nil {
		return style{}
	}
	if ctx := cmd.Context(); ctx != nil {
		if st, ok := ctx.Value(styleKey{}).(style); ok {
			return st
		}
	}
	return style{}
}

// withStyle stores st in the command's context, keeping whatever context it
// already has (the signal context from Execute).
func withStyle(cmd *cobra.Command, st style) {
	ctx := cmd.Context()
	if ctx == nil {
		ctx = context.Background()
	}
	cmd.SetContext(context.WithValue(ctx, styleKey{}, st))
}

// colorEnabled decides whether w may carry escape sequences.
//
// It has to see the *real* stream, so it runs before the prefix writer wraps it.
// "auto" is what makes the ROS integration safe: sound_sync reads soundctl
// through a pipe (popen), a redirected log is a pipe as well, and neither is a
// terminal - so the log never contains an escape sequence, without sound_sync
// having to ask for it.
//
// NO_COLOR (no-color.org) and TERM=dumb are honoured because they are what users
// and tooling set first; a writer that is not an *os.File (a buffer in a test) is
// never coloured.
func colorEnabled(w io.Writer, noColor bool) bool {
	if noColor || os.Getenv("NO_COLOR") != "" || os.Getenv("TERM") == "dumb" {
		return false
	}
	f, ok := w.(*os.File)
	if !ok {
		return false
	}
	return term.IsTerminal(int(f.Fd()))
}

// levelColorWriter colours the level token of the lines the log package receives
// (slog's default handler writes through it, see logOptions.apply). Colouring at
// the writer keeps the line format unchanged and avoids replacing the handler,
// which would reformat every line.
type levelColorWriter struct {
	w      io.Writer
	colors bool
	prefix string
}

// newLevelColorWriter wraps w. Wrapping an already wrapped writer updates it in
// place, so running the PersistentPreRun twice cannot nest the colouring.
func newLevelColorWriter(w io.Writer, colors bool, prefix string) io.Writer {
	if lw, ok := w.(*levelColorWriter); ok {
		lw.colors, lw.prefix = colors, prefix
		return lw
	}
	return &levelColorWriter{w: w, colors: colors, prefix: prefix}
}

// Write reports the bytes of b, not of the embellished output, as io.Writer
// requires.
func (w *levelColorWriter) Write(b []byte) (int, error) {
	if !w.colors {
		return w.w.Write(b)
	}
	if _, err := w.w.Write(w.paintLevels(b)); err != nil {
		return 0, err
	}
	return len(b), nil
}

// paintLevels colours the level token of every line in b. A log call may carry
// more than one line when a message contains a newline; a continuation line has no
// level token and is left alone.
func (w *levelColorWriter) paintLevels(b []byte) []byte {
	var out []byte
	for len(b) > 0 {
		line := b
		if i := bytes.IndexByte(b, '\n'); i >= 0 {
			line, b = b[:i+1], b[i+1:]
		} else {
			b = nil
		}
		out = append(out, w.paintLine(line)...)
	}
	return out
}

// paintLine colours the level token of one line.
//
// slog's default handler writes "[timestamp ]LEVEL msg key=value", so the level is
// the first *word that matches* - searching from the left finds it before any
// message text can, whether or not the log package put a timestamp in front of it.
// The search stops after levelScan bytes: a continuation line (a message with a
// newline) has no level of its own, and a word further right is message text.
func (w *levelColorWriter) paintLine(line []byte) []byte {
	offset := 0
	if w.prefix != "" && bytes.HasPrefix(line, []byte(w.prefix)) {
		offset = len(w.prefix)
	}
	rest := line[offset:]
	if len(rest) > levelScan {
		rest = rest[:levelScan]
	}

	for start := 0; start < len(rest); {
		end := bytes.IndexByte(rest[start:], ' ')
		if end < 0 {
			break
		}
		end += start
		color, ok := levelColors[string(rest[start:end])]
		if !ok {
			start = end + 1
			continue
		}
		from, to := offset+start, offset+end
		out := make([]byte, 0, len(line)+len(color)+len(colorReset))
		out = append(out, line[:from]...)
		out = append(out, color...)
		out = append(out, line[from:to]...)
		out = append(out, colorReset...)
		out = append(out, line[to:]...)
		return out
	}
	return line
}
