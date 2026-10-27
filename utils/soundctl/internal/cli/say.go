package cli

import (
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"strings"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

// defaultPlayer plays the rendered file on the host without a window. ffplay
// ships with the ffmpeg the render chain already requires, so a working
// `soundctl tts` implies a working `soundctl say`.
const defaultPlayer = "ffplay -nodisp -autoexit -loglevel error"

// sayFlags are the flags of the say command. The ttsFlags part is only used for
// the engine location (--tts-dir/-bin/-lib-dir/-model-dir/--ffmpeg/--threads/
// --steps); --config and friends make no sense without a YAML.
type sayFlags struct {
	ttsFlags

	lang     string
	model    string
	sid      int
	speed    float64
	bitrate  string
	loudNorm bool

	out    string
	play   bool
	player string
}

// settings resolves the flags into a tts block, so the ad-hoc path is validated
// and logged exactly like the YAML one.
func (f *sayFlags) settings() fwsync.TTSSettings {
	return fwsync.TTSSettings{
		Engine:   fwsync.DefaultTTSEngine,
		Model:    f.model,
		Sid:      f.sid,
		Speed:    f.speed,
		Bitrate:  f.bitrate,
		LoudNorm: f.loudNorm,
		Threads:  f.threads,
	}
}

// sayText joins the arguments into one text ("soundctl say Hallo Welt" works
// without quotes) and reads stdin when the single argument is "-".
func sayText(args []string, stdin io.Reader) (string, error) {
	if len(args) == 1 && args[0] == "-" {
		raw, err := io.ReadAll(stdin)
		if err != nil {
			return "", fmt.Errorf("read stdin: %w", err)
		}
		args = []string{string(raw)}
	}
	text := strings.TrimSpace(strings.Join(args, " "))
	if text == "" {
		return "", errors.New("no text to speak")
	}
	return text, nil
}

// playerCommand splits the --player template into binary and arguments and
// appends the file (every supported player - ffplay, mpg123, paplay, ... - takes
// the file last). Kept free of side effects so the flag handling is testable
// without a speaker.
func playerCommand(player, file string) (string, []string, error) {
	fields := strings.Fields(player)
	if len(fields) == 0 {
		return "", nil, errors.New("no player configured (use --player, e.g. \"ffplay -nodisp -autoexit\")")
	}
	return fields[0], append(fields[1:], file), nil
}

// playFile runs the player synchronously, so `soundctl say` returns when the
// sound is done rather than cutting it off.
func playFile(cmd *cobra.Command, player, file string) error {
	bin, args, err := playerCommand(player, file)
	if err != nil {
		return err
	}
	if _, err := exec.LookPath(bin); err != nil {
		return fmt.Errorf("player %q not found (install ffmpeg or pass --player, e.g. \"mpg123 -q\")", bin)
	}
	fmt.Fprintf(cmd.OutOrStdout(), "  %-24s playing with %s\n", "say", bin)
	c := exec.Command(bin, args...)
	c.Stdout = cmd.OutOrStdout()
	c.Stderr = cmd.ErrOrStderr()
	if err := c.Run(); err != nil {
		return fmt.Errorf("%s %s: %w", bin, strings.Join(args, " "), err)
	}
	return nil
}

func newSayCmd() *cobra.Command {
	f := &sayFlags{}
	cmd := &cobra.Command{
		Use:   "say <text>",
		Short: "Render a text with the mower's TTS chain and play it here",
		Long: "Synthesise one text on this machine and play it through the local speakers.\n" +
			"It is the same chain as the announcements on the mower - same engine call,\n" +
			"EBU R128 loudness normalisation, 16 kHz mono CBR MP3 and the same format\n" +
			"check - so a text can be auditioned before it goes into sounds_*.yaml.\n" +
			"\n" +
			"No YAML and no cache: every call renders (~2 s on a PC, ~5-8 s on the CM4).\n" +
			"For the definitions use \"soundctl tts\", which caches via .json sidecars.\n" +
			"\n" +
			"Playback uses \"ffplay\", part of the ffmpeg the encoder needs anyway;\n" +
			"--player takes any command that accepts the file as its last argument.\n" +
			"--out keeps the MP3, without it the file lives in a temp dir that is\n" +
			"removed after playback.\n" +
			"\n" +
			"The text is the joined arguments (quotes optional: say Hallo Welt), or \"-\"\n" +
			"to read it from stdin.",
		Example: "  soundctl say \"Hallo, ich bin Steve\" --lang de\n" +
			"  soundctl say \"Der Akku ist fast leer\" --lang de --speed 1.1\n" +
			"  soundctl say \"Hello\" --sid 3 --out /tmp/hello.mp3 --play=false\n" +
			"  soundctl say \"Test\" --lang de --player \"mpg123 -q\"",
		Args: cobra.MinimumNArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			out := cmd.OutOrStdout()

			text, err := sayText(args, cmd.InOrStdin())
			if err != nil {
				return err
			}

			set := f.settings()
			if errs := fwsync.ValidateTTSSettings(set); len(errs) > 0 {
				return errors.New(strings.Join(errs, "; "))
			}
			if errs := fwsync.ValidateLanguage(f.lang); len(errs) > 0 {
				return errors.New(strings.Join(errs, "; "))
			}
			// Normalise the bitrate so the log line matches the YAML spelling.
			if kbps, err := fwsync.ParseBitrate(set.Bitrate); err == nil {
				set.Bitrate = fwsync.FormatBitrate(kbps)
			}

			eng := f.ttsFlags.engine(set)
			if err := eng.Available(); err != nil {
				return fmt.Errorf("%w (run \"soundctl tts install\" or set --tts-dir)", err)
			}
			enc := tts.Encoder{FFmpeg: f.ffmpeg}
			if err := enc.Available(); err != nil {
				return err
			}

			dst := f.out
			if dst == "" {
				dir, err := os.MkdirTemp("", "soundctl-say-")
				if err != nil {
					return fmt.Errorf("create temp dir: %w", err)
				}
				defer os.RemoveAll(dir)
				dst = filepath.Join(dir, "say.mp3")
			}

			id := fwsync.TTSIdentity{
				Engine:   set.Engine,
				Model:    set.Model,
				Language: f.lang,
				Bitrate:  set.Bitrate,
				Sid:      set.Sid,
				Speed:    set.Speed,
				LoudNorm: set.LoudNorm,
			}
			res, err := tts.RenderText(text, id, dst, eng, enc, func(format string, args ...any) {
				fmt.Fprintf(out, format+"\n", args...)
			})
			if err != nil {
				return err
			}
			fmt.Fprintf(out, "  %-24s %s, %d kB, %d ms (%s)\n", "say", res.Info, res.Bytes/1024, res.RenderMs, res.Path)

			if !f.play {
				return nil
			}
			return playFile(cmd, f.player, dst)
		},
	}

	fl := cmd.Flags()
	fl.StringVar(&f.lang, "lang", "en", "language of the text (the model's --lang: en, de, ...)")
	fl.StringVar(&f.model, "model", fwsync.DefaultTTSModel, "model directory below <tts-dir>/models")
	fl.IntVar(&f.sid, "sid", fwsync.DefaultTTSSid, "speaker id 0..9")
	fl.Float64Var(&f.speed, "speed", fwsync.DefaultTTSSpeed, "speaking rate 0.7..1.5")
	fl.StringVar(&f.bitrate, "bitrate", fwsync.DefaultTTSBitrate, "MP3 bitrate, CBR 16 kHz mono (e.g. 32k, 48k, 64k)")
	fl.BoolVar(&f.loudNorm, "loudnorm", true, "EBU R128 normalise to -16 LUFS (--loudnorm=false for the raw level)")
	fl.StringVar(&f.ttsDir, "tts-dir", defaultTTSDir(), "TTS asset directory (bin/, lib/, models/); $OPENMOWER_TTS_DIR")
	fl.StringVar(&f.ttsBin, "tts-bin", "", "override the sherpa-onnx-offline-tts binary (default: <tts-dir>/bin/...)")
	fl.StringVar(&f.ttsLibDir, "tts-lib-dir", "", "override LD_LIBRARY_PATH of the engine (default: <tts-dir>/lib)")
	fl.StringVar(&f.ttsModelDir, "tts-model-dir", "", "override the model directory (default: <tts-dir>/models/<model>)")
	fl.StringVar(&f.ffmpeg, "ffmpeg", "ffmpeg", "ffmpeg binary used for encoding")
	fl.IntVar(&f.threads, "threads", 0, "ONNX threads (0 = engine default)")
	fl.IntVar(&f.steps, "steps", 0, "flow-matching steps (default: 8; lower = faster, rougher)")
	fl.StringVar(&f.out, "out", "", "keep the rendered MP3 here (default: temp file, removed after playback)")
	fl.BoolVar(&f.play, "play", true, "play the result (--play=false renders only)")
	fl.StringVar(&f.player, "player", defaultPlayer, "playback command; the file is appended as last argument")
	return cmd
}
