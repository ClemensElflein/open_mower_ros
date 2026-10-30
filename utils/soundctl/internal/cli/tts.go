package cli

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"text/tabwriter"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

// dataDir is the writable data partition of the mower (a mount in the OS image,
// ./data in docker-simulation) and soundsDirName the overlay below it.
const dataDir = "/data"
const soundsDirName = "sounds"

// defaultSoundsDir returns the writable overlay: $OPENMOWER_SOUNDS_DIR, else
// /data/sounds when the data partition exists, else "" (image only).
func defaultSoundsDir() string {
	if d := os.Getenv("OPENMOWER_SOUNDS_DIR"); d != "" {
		return d
	}
	isDir := func(p string) bool {
		st, err := os.Stat(p)
		return err == nil && st.IsDir()
	}
	return dataSoundsDir(dataDir, isDir)
}

// dataSoundsDir is the /data part of defaultSoundsDir. Only the data partition
// has to exist: the overlay directory itself is created by the first render
// (`soundctl tts`), so a mower with a fresh data partition still gets its
// announcements instead of writing into the read-only image.
func dataSoundsDir(data string, isDir func(string) bool) string {
	if !isDir(data) {
		return ""
	}
	return filepath.Join(data, soundsDirName)
}

// DefaultAPIKeyFile is where the mower keeps the ElevenLabs key. The sounds
// overlay is the one directory every user who customises sounds mounts anyway,
// and it lives on the data partition, so the key needs no extra volume and
// survives image updates. It is never uploaded (sync only sends the files named
// in sounds_*.yaml) and never deleted (cleanup compares the low level's listing
// against those names) - hence the dot-file name.
const DefaultAPIKeyFile = "/data/sounds/.elevenlabs.key"

// apiKey resolves the ElevenLabs key: explicit flag, explicit file, then
// $ELEVENLABS_API_KEY, then the default file. The environment is what CI (a
// repository secret) and a dev shell use; the file is what the mower uses.
func apiKey(explicit, explicitFile, defaultFile string) (string, error) {
	if v := strings.TrimSpace(explicit); v != "" {
		return v, nil
	}
	if explicitFile != "" {
		return readKeyFile(explicitFile)
	}
	if v := strings.TrimSpace(os.Getenv("ELEVENLABS_API_KEY")); v != "" {
		return v, nil
	}
	if defaultFile != "" {
		if v, err := readKeyFile(defaultFile); err == nil && v != "" {
			return v, nil
		}
	}
	return "", fmt.Errorf("no ElevenLabs API key: pass --api-key/--api-key-file, set $ELEVENLABS_API_KEY, or write %s", defaultFile)
}

// readKeyFile reads a key file. Editors append a newline, so the value is
// trimmed; anything else (a missing file included) is an error.
func readKeyFile(path string) (string, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return "", fmt.Errorf("read API key file: %w", err)
	}
	key := strings.TrimSpace(string(raw))
	if key == "" {
		return "", fmt.Errorf("%s is empty", path)
	}
	return key, nil
}

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
}

func (f *ttsFlags) register(cmd *cobra.Command) {
	fl := cmd.Flags()
	fl.StringVar(&f.config, "config", defaultConfigPath, "path to sounds_*.yaml")
	fl.StringVar(&f.soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	fl.StringVar(&f.soundsDir, "sounds-dir", defaultSoundsDir(), "writable overlay with customised/rendered MP3s (empty: only sound_path)")
	layoutFlag(cmd, &f.soundPathOv)
	fl.StringVar(&f.apiKey, "api-key", "", "ElevenLabs API key (instead of the file/environment); never put it in sounds_*.yaml")
	fl.StringVar(&f.apiKeyFile, "api-key-file", "", "file holding the ElevenLabs API key (default: $ELEVENLABS_API_KEY, "+DefaultAPIKeyFile+")")
	fl.StringVar(&f.apiURL, "api-url", "", "ElevenLabs API root (default: "+tts.DefaultElevenLabsURL+"; e.g. https://api.eu.residency.elevenlabs.io)")
	fl.StringVar(&f.ffmpeg, "ffmpeg", "ffmpeg", "ffmpeg binary used for encoding")
	fl.BoolVar(&f.force, "force", false, "re-render even if the sidecar says the file is up to date")
	fl.BoolVar(&f.check, "check", false, "render nothing, only report missing/stale renders (exit 1 if any)")
}

// engineFor builds the engine a definition asks for. The same factory serves
// `soundctl tts` (engine from the YAML) and `soundctl say` (engine from flags),
// so both always use the same engine.
func (f *ttsFlags) engineFor(set fwsync.TTSSettings) (tts.Engine, error) {
	if set.Engine != fwsync.DefaultTTSEngine {
		return nil, fmt.Errorf("tts.engine %q is not supported (use %q)", set.Engine, fwsync.DefaultTTSEngine)
	}
	key, err := apiKey(f.apiKey, f.apiKeyFile, DefaultAPIKeyFile)
	if err != nil {
		return nil, err
	}
	return tts.ElevenLabs{APIKey: key, BaseURL: f.apiURL}, nil
}

func (f *ttsFlags) run(cmd *cobra.Command) error {
	cfg, err := fwsync.LoadConfig(f.config)
	if err != nil {
		return err
	}
	ids, err := fwsync.LoadSoundIDs(f.soundIDs)
	if err != nil {
		return err
	}
	if errs := cfg.Validate(ids); len(errs) > 0 {
		for _, e := range errs {
			fmt.Fprintln(cmd.ErrOrStderr(), "  [error]", e)
		}
		return fmt.Errorf("%s: %d error(s)", f.config, len(errs))
	}

	layout := newLayout(cfg, f.soundPathOv, f.soundsDir)
	out := cmd.OutOrStdout()
	jobs := cfg.RenderJobs()

	if f.check {
		if len(jobs) == 0 {
			fmt.Fprintf(out, "no tts variants in %s\n", f.config)
			return nil
		}
		pending := tts.Pending(cfg, layout)
		if len(pending) == 0 {
			fmt.Fprintf(out, "OK: %d tts render(s) up to date\n", len(jobs))
			return nil
		}
		for _, label := range pending {
			fmt.Fprintf(out, "stale: %s\n", label)
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

	results, err := tts.Render(cfg, layout, eng, enc, tts.Options{
		OutDir:    layout.OutDir(),
		Force:     f.force,
		Generator: "soundctl " + version,
		Log: func(format string, args ...any) {
			fmt.Fprintf(out, format+"\n", args...)
		},
	})
	if err != nil {
		return err
	}

	rendered, skipped := 0, 0
	for _, r := range results {
		if r.Skipped {
			skipped++
		} else {
			rendered++
		}
	}
	fmt.Fprintf(out, "%d rendered, %d up to date (%s)\n", rendered, skipped, layout.OutDir())
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
		apiKeyVal  string
		apiKeyFile string
		apiURL     string
	)
	cmd := &cobra.Command{
		Use:   "voices",
		Short: "List the ElevenLabs voices this API key can use",
		Long: "Queries GET /v2/voices and prints id, name, category and labels.\n" +
			"The default filter shows the premade voices (category premade), which a\n" +
			"free plan may use through the API; \"--all\" also shows the Voice Library\n" +
			"(community) voices, which need a paid plan for API use.\n" +
			"\n" +
			"The id goes into tts.elevenlabs.voice of sounds_*.yaml.",
		Example: "  soundctl tts voices\n" +
			"  soundctl tts voices --all --language de\n" +
			"  soundctl tts voices --search steve",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			key, err := apiKey(apiKeyVal, apiKeyFile, DefaultAPIKeyFile)
			if err != nil {
				return err
			}
			filter := voiceType
			if all {
				filter = ""
			}
			voices, err := tts.ElevenLabs{APIKey: key, BaseURL: apiURL}.Voices(filter)
			if err != nil {
				return err
			}

			out := cmd.OutOrStdout()
			w := tabwriter.NewWriter(out, 0, 0, 2, ' ', 0)
			fmt.Fprintln(w, "VOICE_ID\tCATEGORY\tLANGUAGE\tNAME\tLABELS")
			shown := 0
			for _, v := range voices {
				if search != "" && !strings.Contains(strings.ToLower(v.Name), strings.ToLower(search)) {
					continue
				}
				lang := v.Labels["language"]
				if language != "" && !strings.Contains(strings.ToLower(lang), strings.ToLower(language)) {
					continue
				}
				fmt.Fprintf(w, "%s\t%s\t%s\t%s\t%s\n", v.VoiceID, v.Category, lang, v.Name, voiceLabels(v))
				shown++
			}
			if err := w.Flush(); err != nil {
				return err
			}
			fmt.Fprintf(out, "\n%d of %d voice(s) shown.\n", shown, len(voices))
			fmt.Fprintln(out, "The premade (default) voices work with a free plan via the API;",
				"Voice Library (community) voices need a paid subscription.")
			return nil
		},
	}
	fl := cmd.Flags()
	fl.StringVar(&voiceType, "type", "default", "voice_type filter: default (premade), community (Voice Library), personal, workspace")
	fl.BoolVar(&all, "all", false, "no voice_type filter (everything the account can see)")
	fl.StringVar(&language, "language", "", "only voices labelled with this language (e.g. de)")
	fl.StringVar(&search, "search", "", "substring match on the voice name")
	fl.StringVar(&apiKeyVal, "api-key", "", "ElevenLabs API key (instead of the file/environment)")
	fl.StringVar(&apiKeyFile, "api-key-file", "", "file holding the ElevenLabs API key")
	fl.StringVar(&apiURL, "api-url", "", "ElevenLabs API root (default: "+tts.DefaultElevenLabsURL+")")
	return cmd
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
			"The engine is ElevenLabs (cloud): the text is sent to the API, the returned\n" +
			"16 kHz WAV is normalised and encoded by ffmpeg. The API key comes from\n" +
			"--api-key, --api-key-file, $ELEVENLABS_API_KEY,\n" +
			"$ELEVENLABS_API_KEY or " + DefaultAPIKeyFile + ".\n" +
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
