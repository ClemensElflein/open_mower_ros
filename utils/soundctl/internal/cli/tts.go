package cli

import (
	"fmt"
	"sort"
	"strings"
	"text/tabwriter"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/settings"
	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

// defaultSoundsDir is the flag default of --sounds-dir. The policy behind it (where
// the overlay lives and why) is in internal/settings, so it is testable there
// without a cobra command.
func defaultSoundsDir() string { return settings.SoundsDir() }

// ttsFlags are the flags of the tts command.
type ttsFlags struct {
	config      string
	soundIDs    string
	soundsDir   string
	soundPathOv string
	apiKey      string
	apiKeyFile  string
	apiURL      string
	ffmpeg      string
	force       bool
	check       bool
	keepGoing   bool
	storeKey    bool
}

func (f *ttsFlags) register(cmd *cobra.Command) {
	fl := cmd.Flags()
	fl.StringVar(&f.config, "config", defaultConfigPath, "path to sounds_*.yaml")
	fl.StringVar(&f.soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	fl.StringVar(&f.soundsDir, "sounds-dir", defaultSoundsDir(), "writable overlay with customised/rendered MP3s (empty: only sound_path)")
	layoutFlag(cmd, &f.soundPathOv)
	fl.StringVar(&f.apiKey, "api-key", "", "API key of the engine (instead of the file/environment); never put it in sounds_*.yaml")
	fl.StringVar(&f.apiKeyFile, "api-key-file", "", "file holding the API key of the engine (default: $ELEVENLABS_API_KEY / "+settings.DefaultElevenLabsKeyFile+", or $CARTESIA_API_KEY / "+settings.DefaultCartesiaKeyFile+")")
	fl.StringVar(&f.apiURL, "api-url", "", "API root (default: "+tts.DefaultElevenLabsURL+" for elevenlabs - e.g. the EU host - and "+tts.DefaultCartesiaURL+" for cartesia)")
	fl.StringVar(&f.ffmpeg, "ffmpeg", "ffmpeg", "ffmpeg binary used for encoding")
	fl.BoolVar(&f.force, "force", false, "re-render even if the sidecar says the file is up to date")
	fl.BoolVar(&f.check, "check", false, "render nothing, only report missing/stale renders (exit 1 if any)")
	fl.BoolVar(&f.keepGoing, "keep-going", false, "continue with the other variants when one fails (its previous file is kept, exit code stays non-zero)")
	fl.BoolVar(&f.storeKey, "print-store-key", false, "print the registry tag of this definition's render stand (<language>-<hash>) and exit")
}

// engineFor builds the engine the definition asks for. The construction itself
// (and the API key resolution) is settings.NewEngine, because `soundctl say` needs
// exactly the same factory from flags.
func (f *ttsFlags) engineFor(set fwsync.TTSSettings) (tts.Engine, error) {
	return settings.NewEngine(set, f.apiKey, f.apiKeyFile, f.apiURL)
}

func (f *ttsFlags) run(cmd *cobra.Command) error {
	cfg, err := fwsync.LoadConfig(f.config)
	if err != nil {
		return err
	}

	// The store key names the render stand in the registry (see Config.StoreKey). It is a
	// property of the definition alone - no shared definition, no API key, no ffmpeg and no
	// overlay - which is what makes it usable as the first step of a CI job that pulls a
	// stand (and as a quick local answer to "which tag does this YAML correspond to?").
	if f.storeKey {
		fmt.Fprintln(cmd.OutOrStdout(), cfg.StoreKey())
		return nil
	}

	ids, err := fwsync.LoadSoundIDs(f.soundIDs)
	if err != nil {
		return err
	}
	if errs := cfg.Validate(ids); len(errs) > 0 {
		st := styleFor(cmd)
		for _, e := range errs {
			fmt.Fprintln(cmd.ErrOrStderr(), "  "+st.error("[error]"), e)
		}
		return fmt.Errorf("%s: %d error(s)", f.config, len(errs))
	}

	layout := newLayout(cfg, f.soundPathOv, f.soundsDir)
	out := cmd.OutOrStdout()
	st := styleFor(cmd)
	jobs := cfg.RenderJobs()

	if f.check {
		if len(jobs) == 0 {
			fmt.Fprintf(out, "no tts variants in %s\n", f.config)
			return nil
		}
		pending := tts.Pending(cfg, layout)
		if len(pending) == 0 {
			fmt.Fprintf(out, "%s %d tts render(s) up to date\n", st.ok("OK:"), len(jobs))
			return nil
		}
		for _, label := range pending {
			fmt.Fprintf(out, "%s %s\n", st.warn("stale:"), label)
		}
		return fmt.Errorf("%d of %d tts render(s) missing or stale", len(pending), len(jobs))
	}

	if len(jobs) == 0 {
		fmt.Fprintf(out, "no tts variants in %s\n", f.config)
		return nil
	}

	eng, err := f.engineFor(cfg.TTSSettings())
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

	results, err := tts.Render(cmd.Context(), cfg, layout, eng, enc, tts.Options{
		OutDir:    layout.OutDir(),
		Force:     f.force,
		KeepGoing: f.keepGoing,
		Generator: "soundctl " + version,
		Log: func(format string, args ...any) {
			fmt.Fprintf(out, format+"\n", args...)
		},
	})
	if err != nil {
		return err
	}

	rendered, skipped, failed := 0, 0, 0
	for _, r := range results {
		switch {
		case r.Failed:
			failed++
		case r.Skipped:
			skipped++
		default:
			rendered++
		}
	}
	fmt.Fprintf(out, "%d rendered, %d up to date, %d failed (%s)\n", rendered, skipped, failed, layout.OutDir())
	if failed > 0 {
		return fmt.Errorf("%d render(s) failed - their previous files were kept in %s", failed, layout.OutDir())
	}
	return nil
}

// newTTSVoicesCmd lists the ElevenLabs voices the configured key can use. The
// plan decides which ones are usable via the API: the built-in premade voices
// (voice_type=default) work on the free tier, Voice Library ones (community) need
// a paid subscription.
func newTTSVoicesCmd() *cobra.Command {
	var (
		voiceType  string
		all        bool
		language   string
		search     string
		engine     string
		apiKeyVal  string
		apiKeyFile string
		apiURL     string
	)
	cmd := &cobra.Command{
		Use:   "voices",
		Short: "List the voices the API key of an engine can use",
		Long: "Queries the voice list of the engine (ElevenLabs GET /v2/voices, Cartesia GET /voices)\n" +
			"and prints the id, language and name, so the id can go into the matching\n" +
			"block of sounds_*.yaml (tts.elevenlabs.voice or tts.cartesia.voice).\n" +
			"\n" +
			"For ElevenLabs the default filter shows the premade voices (category premade),\n" +
			"which a free plan may use through the API; \"--all\" also shows the Voice\n" +
			"Library (community) voices, which need a paid plan for API use.\n" +
			"\n" +
			"The LANGUAGE column is the language of the reference recording, followed in\n" +
			"parentheses by the other languages the voice is verified for (\"en (de, pl)\").\n" +
			"A voice speaks every language its model supports, but one it is not verified\n" +
			"for keeps the accent of the reference recording - so \"--language fr\" lists the\n" +
			"voices whose French is verified, not just the ones labelled French.",
		Example: "  soundctl tts voices\n" +
			"  soundctl tts voices --all --language de\n" +
			"  soundctl tts voices --search steve\n" +
			"  soundctl tts voices --engine cartesia --language de",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			key, err := settings.APIKey(engine, apiKeyVal, apiKeyFile)
			if err != nil {
				return err
			}
			out := cmd.OutOrStdout()

			if engine == fwsync.TTSEngineCartesia {
				voices, err := tts.Cartesia{APIKey: key, BaseURL: apiURL}.Voices(cmd.Context())
				if err != nil {
					return err
				}
				w := tabwriter.NewWriter(out, 0, 0, 2, ' ', 0)
				fmt.Fprintln(w, "VOICE_ID\tLANGUAGE\tGENDER\tNAME\tDESCRIPTION")
				shown := 0
				for _, v := range voices {
					if search != "" && !strings.Contains(strings.ToLower(v.Name+" "+v.Description), strings.ToLower(search)) {
						continue
					}
					if language != "" && !strings.Contains(strings.ToLower(v.Language), strings.ToLower(language)) {
						continue
					}
					fmt.Fprintf(w, "%s\t%s\t%s\t%s\t%s\n", v.ID, v.Language, v.Gender, v.Name, v.Description)
					shown++
				}
				if err := w.Flush(); err != nil {
					return err
				}
				fmt.Fprintf(out, "\n%d of %d voice(s) shown.\n", shown, len(voices))
				fmt.Fprintln(out, "The id goes into tts.cartesia.voice of sounds_*.yaml.")
				return nil
			}

			filter := voiceType
			if all {
				filter = ""
			}
			voices, err := tts.ElevenLabs{APIKey: key, BaseURL: apiURL}.Voices(cmd.Context(), filter)
			if err != nil {
				return err
			}

			w := tabwriter.NewWriter(out, 0, 0, 2, ' ', 0)
			fmt.Fprintln(w, "VOICE_ID\tCATEGORY\tLANGUAGE\tNAME\tLABELS")
			shown := 0
			for _, v := range voices {
				if search != "" && !strings.Contains(strings.ToLower(v.Name), strings.ToLower(search)) {
					continue
				}
				if language != "" && !voiceSpeaksLanguage(v, language) {
					continue
				}
				fmt.Fprintf(w, "%s\t%s\t%s\t%s\t%s\n", v.VoiceID, v.Category, voiceLanguages(v), v.Name, voiceLabels(v))
				shown++
			}
			if err := w.Flush(); err != nil {
				return err
			}
			fmt.Fprintf(out, "\n%d of %d voice(s) shown.\n", shown, len(voices))
			fmt.Fprintln(out, "The id goes into tts.elevenlabs.voice of sounds_*.yaml.")
			fmt.Fprintln(out, "The premade (default) voices work with a free plan via the API;",
				"Voice Library (community) voices need a paid subscription.")
			return nil
		},
	}
	fl := cmd.Flags()
	fl.StringVar(&engine, "engine", fwsync.DefaultTTSEngine, "engine whose voices are listed: "+
		fwsync.TTSEngineElevenLabs+" or "+fwsync.TTSEngineCartesia)
	fl.StringVar(&voiceType, "type", "default", "voice_type filter: default (premade), community (Voice Library), personal, workspace")
	fl.BoolVar(&all, "all", false, "no voice_type filter (everything the account can see)")
	fl.StringVar(&language, "language", "", "only voices labelled or verified for this language (e.g. de)")
	fl.StringVar(&search, "search", "", "substring match on the voice name")
	fl.StringVar(&apiKeyVal, "api-key", "", "API key of the engine (instead of the file/environment)")
	fl.StringVar(&apiKeyFile, "api-key-file", "", "file holding the API key of the engine")
	fl.StringVar(&apiURL, "api-url", "", "API root (default: "+tts.DefaultElevenLabsURL+" for elevenlabs, "+tts.DefaultCartesiaURL+" for cartesia)")
	return cmd
}

// voiceLanguages renders the LANGUAGE column: the language of the reference recording,
// followed in parentheses by the other languages the voice is verified for, e.g.
// "en (de, pl)". A voice without verified languages is just its label.
func voiceLanguages(v tts.Voice) string {
	native := v.Labels["language"]
	others := make([]string, 0, len(v.VerifiedLanguages))
	for _, vl := range v.VerifiedLanguages {
		lang := strings.TrimSpace(vl.Language)
		if lang == "" || lang == native {
			continue
		}
		others = append(others, lang)
	}
	sort.Strings(others)
	if len(others) == 0 {
		return native
	}
	return native + " (" + strings.Join(others, ", ") + ")"
}

// voiceSpeaksLanguage reports whether the voice is labelled with lang or verified for it.
// The label alone would be misleading: it names the language of the reference recording,
// while a multilingual model speaks every language it supports - verified or not. A voice
// that is not verified for a language still speaks it, but with the accent of the
// reference recording, which is why this only decides what the listing shows.
func voiceSpeaksLanguage(v tts.Voice, lang string) bool {
	lang = strings.ToLower(strings.TrimSpace(lang))
	if lang == "" {
		return true
	}
	if strings.Contains(strings.ToLower(v.Labels["language"]), lang) {
		return true
	}
	for _, vl := range v.VerifiedLanguages {
		if strings.Contains(strings.ToLower(vl.Language), lang) {
			return true
		}
	}
	return false
}

// voiceLabels renders the label map in a stable order.
func voiceLabels(v tts.Voice) string {
	keys := make([]string, 0, len(v.Labels))
	for k := range v.Labels {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	parts := make([]string, 0, len(keys))
	for _, k := range keys {
		parts = append(parts, k+"="+v.Labels[k])
	}
	return strings.Join(parts, ", ")
}

func newTTSCmd() *cobra.Command {
	f := &ttsFlags{}
	cmd := &cobra.Command{
		Use:   "tts",
		Short: "Render the tts variants of sounds_*.yaml into the firmware MP3s",
		Long: "Synthesise every tts variant and write it as a 16 kHz mono CBR MP3 into\n" +
			"the sound overlay (sounds/<language>/). Each file gets a \".json\" sidecar\n" +
			"with the render fingerprint, so a re-run only touches what changed: the\n" +
			"text, the voice, a voice setting, the language or the bitrate.\n" +
			"\n" +
			"The engine comes from the definition's tts.engine (a hosted API): the text\n" +
			"is sent to it and the returned 16 kHz audio is normalised and encoded by\n" +
			"ffmpeg. The key is resolved per engine: --api-key, --api-key-file,\n" +
			"$ELEVENLABS_API_KEY / " + settings.DefaultElevenLabsKeyFile + ", or\n" +
			"$CARTESIA_API_KEY / " + settings.DefaultCartesiaKeyFile + ".\n" +
			"\n" +
			"Runs once per ROS start inside the container; \"tts --check\" only reports\n" +
			"what would be rendered (it needs no key), and sync refuses to upload a\n" +
			"stale render.",
		Example: "  soundctl tts --config src/open_mower/params/sounds_de.yaml --sounds-dir /data/sounds\n" +
			"  soundctl tts --check",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error { return f.run(cmd) },
	}
	f.register(cmd)
	cmd.AddCommand(newTTSVoicesCmd())
	return cmd
}
