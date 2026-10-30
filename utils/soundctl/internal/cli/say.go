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

// sayFlags are the flags of the say command: the ElevenLabs settings plus the
// encoder and playback options. The embedded ttsFlags only contributes the API
// key/URL resolution and --ffmpeg; --config and friends make no sense without a
// YAML.
type sayFlags struct {
	ttsFlags

	lang     string
	model    string
	voice    string
	bitrate  string
	loudNorm bool

	stability       float64
	similarityBoost float64
	style           float64
	speakerBoost    bool

	out    string
	play   bool
	player string
}

// settings resolves the flags into a tts block, so the ad-hoc path is validated
// and logged exactly like the YAML one.
func (f *sayFlags) settings() fwsync.TTSSettings {
	return fwsync.TTSSettings{
		Engine:          fwsync.DefaultTTSEngine,
		Model:           f.model,
		Voice:           f.voice,
		Bitrate:         f.bitrate,
		LoudNorm:        f.loudNorm,
		Stability:       f.stability,
		SimilarityBoost: f.similarityBoost,
		Style:           f.style,
		UseSpeakerBoost: f.speakerBoost,
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
			"It is the same chain as the announcements on the mower - ElevenLabs, the same\n" +
			"encoder, EBU R128 loudness normalisation, 16 kHz mono CBR MP3 and the same\n" +
			"format check - so a text or a voice setting can be auditioned before it goes\n" +
			"into sounds_*.yaml.\n" +
			"\n" +
			"--voice is the ElevenLabs voice id; --model picks the model\n" +
			"(eleven_v4, eleven_multilingual_v2, eleven_flash_v2_5, ...).\n" +
			"\n" +
			"No YAML and no cache: every call renders (one API request, about a second).\n" +
			"For the definitions use \"soundctl tts\", which caches via .json sidecars.\n" +
			"\n" +
			"Playback uses \"ffplay\", part of the ffmpeg the encoder needs anyway;\n" +
			"--player takes any command that accepts the file as its last argument.\n" +
			"--out keeps the MP3, without it the file lives in a temp dir that is\n" +
			"removed after playback.\n" +
			"\n" +
			"The text is the joined arguments (quotes optional: say Hallo Welt), or \"-\"\n" +
			"to read it from stdin.",
		Example: "  soundctl say \"Hallo, ich bin Steve\" --lang de --voice I5mfJk5ETJFOHZxoRrHq\n" +
			"  soundctl say \"Der Akku ist fast leer\" --lang de --voice I5mfJk5ETJFOHZxoRrHq --stability 0.4\n" +
			"  soundctl say \"Hello\" --voice 5gaEGrWSk4v20HMfIhgC --out /tmp/hello.mp3 --play=false\n" +
			"  soundctl say \"Test\" --voice 5gaEGrWSk4v20HMfIhgC --player \"mpg123 -q\"",
		Args: cobra.MinimumNArgs(1),
		RunE: func(cmd *cobra.Command, args []string) error {
			out := cmd.OutOrStdout()

			text, err := sayText(args, cmd.InOrStdin())
			if err != nil {
				return err
			}

			set := f.settings()
			if errs := fwsync.ValidateLanguage(f.lang); len(errs) > 0 {
				return errors.New(strings.Join(errs, "; "))
			}
			if errs := fwsync.ValidateTTSSettings(set); len(errs) > 0 {
				return errors.New(strings.Join(errs, "; "))
			}

			eng, err := f.ttsFlags.engineFor(set)
			if err != nil {
				return err
			}
			if err := eng.Available(); err != nil {
				return err
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
				Engine:          set.Engine,
				Model:           set.Model,
				Voice:           set.Voice,
				Language:        f.lang,
				Bitrate:         set.Bitrate,
				LoudNorm:        set.LoudNorm,
				Stability:       set.Stability,
				SimilarityBoost: set.SimilarityBoost,
				Style:           set.Style,
				UseSpeakerBoost: set.UseSpeakerBoost,
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
	fl.StringVar(&f.lang, "lang", "en", "language of the text (ISO 639-1: en, de, ...)")
	fl.StringVar(&f.model, "model", fwsync.DefaultTTSModel, "ElevenLabs model id (eleven_v4, eleven_multilingual_v2, eleven_flash_v2_5, ...)")
	fl.StringVar(&f.voice, "voice", "", "ElevenLabs voice id (required)")
	fl.Float64Var(&f.stability, "stability", fwsync.DefaultTTSStability, "voice stability 0..1 (lower = more expressive)")
	fl.Float64Var(&f.similarityBoost, "similarity-boost", fwsync.DefaultTTSSimilarityBoost, "similarity boost 0..1")
	fl.Float64Var(&f.style, "style", fwsync.DefaultTTSStyle, "style exaggeration 0..1 (0 = the voice's own delivery)")
	fl.BoolVar(&f.speakerBoost, "speaker-boost", fwsync.DefaultTTSUseSpeakerBoost, "ElevenLabs speaker boost")
	fl.StringVar(&f.bitrate, "bitrate", fwsync.DefaultTTSBitrate, "MP3 bitrate, CBR 16 kHz mono (e.g. 32k, 48k, 64k)")
	fl.BoolVar(&f.loudNorm, "loudnorm", true, "EBU R128 normalise to -16 LUFS (--loudnorm=false for the raw level)")
	fl.StringVar(&f.apiKey, "api-key", "", "ElevenLabs API key (instead of the file/environment)")
	fl.StringVar(&f.apiKeyFile, "api-key-file", "", "file holding the ElevenLabs API key (default: environment, then "+DefaultAPIKeyFile+")")
	fl.StringVar(&f.apiURL, "api-url", "", "ElevenLabs API root (default: "+tts.DefaultElevenLabsURL+")")
	fl.StringVar(&f.ffmpeg, "ffmpeg", "ffmpeg", "ffmpeg binary used for encoding")
	fl.StringVar(&f.out, "out", "", "keep the rendered MP3 here (default: temp file, removed after playback)")
	fl.BoolVar(&f.play, "play", true, "play the result (--play=false renders only)")
	fl.StringVar(&f.player, "player", defaultPlayer, "playback command; the file is appended as last argument")
	return cmd
}
