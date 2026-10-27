package cli

import (
	"fmt"
	"os"
	"path/filepath"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

// envOr returns the environment variable key or def.
func envOr(key, def string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return def
}

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

// defaultTTSDir returns the TTS asset directory: image path, then
// $OPENMOWER_TTS_DIR, else the image default.
func defaultTTSDir() string { return envOr("OPENMOWER_TTS_DIR", tts.DefaultDir) }

// piperBinary resolves the piper executable. It is resolved lazily (not as a
// flag default) so that --tts-dir on the command line is honoured: explicit
// flag, $OPENMOWER_PIPER_BIN, <tts-dir>/piper/piper, else "piper" from PATH.
func (f *ttsFlags) piperBinary() string {
	if f.piperBin != "" {
		return f.piperBin
	}
	if v := os.Getenv("OPENMOWER_PIPER_BIN"); v != "" {
		return v
	}
	inTTS := filepath.Join(f.ttsDir, "piper", "piper")
	if st, err := os.Stat(inTTS); err == nil && !st.IsDir() {
		return inTTS
	}
	return "piper"
}

// piperVoiceDir resolves the directory that holds the <voice>.onnx files, with
// the same precedence as piperBinary.
func (f *ttsFlags) piperVoiceDir() string {
	if f.piperDir != "" {
		return f.piperDir
	}
	if v := os.Getenv("OPENMOWER_PIPER_DIR"); v != "" {
		return v
	}
	inTTS := filepath.Join(f.ttsDir, "piper", "voices")
	if st, err := os.Stat(inTTS); err == nil && st.IsDir() {
		return inTTS
	}
	return ""
}

// ttsFlags are the flags of the tts command.
type ttsFlags struct {
	config      string
	soundIDs    string
	soundsDir   string
	soundPathOv string
	ttsDir      string
	ttsBin      string
	ttsLibDir   string
	ttsModelDir string
	piperBin    string
	piperDir    string
	ffmpeg      string
	threads     int
	steps       int
	force       bool
	check       bool
}

func (f *ttsFlags) register(cmd *cobra.Command) {
	fl := cmd.Flags()
	fl.StringVar(&f.config, "config", defaultConfigPath, "path to sounds_*.yaml")
	fl.StringVar(&f.soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	fl.StringVar(&f.soundsDir, "sounds-dir", defaultSoundsDir(), "writable overlay with customised/rendered MP3s (empty: only sound_path)")
	layoutFlag(cmd, &f.soundPathOv)
	fl.StringVar(&f.ttsDir, "tts-dir", defaultTTSDir(), "TTS asset directory (bin/, lib/, models/); $OPENMOWER_TTS_DIR")
	fl.StringVar(&f.ttsBin, "tts-bin", "", "override the sherpa-onnx-offline-tts binary (default: <tts-dir>/bin/...)")
	fl.StringVar(&f.ttsLibDir, "tts-lib-dir", "", "override LD_LIBRARY_PATH of the engine (default: <tts-dir>/lib)")
	fl.StringVar(&f.ttsModelDir, "tts-model-dir", "", "override the model directory (default: <tts-dir>/models/<tts.model>)")
	fl.StringVar(&f.ffmpeg, "ffmpeg", "ffmpeg", "ffmpeg binary used for encoding")
	fl.StringVar(&f.piperBin, "piper-bin", "", "piper executable (engine: piper); default: $OPENMOWER_PIPER_BIN, <tts-dir>/piper/piper, PATH")
	fl.StringVar(&f.piperDir, "piper-dir", "", "voices dir (engine: piper); default: $OPENMOWER_PIPER_DIR, <tts-dir>/piper/voices")
	fl.IntVar(&f.threads, "threads", 0, "ONNX threads (default: tts.threads from the YAML, 0 = engine default)")
	fl.IntVar(&f.steps, "steps", 0, "flow-matching steps (default: 8; lower = faster, rougher)")
	fl.BoolVar(&f.force, "force", false, "re-render even if the sidecar says the file is up to date")
	fl.BoolVar(&f.check, "check", false, "render nothing, only report missing/stale renders (exit 1 if any)")
}

// engineFor builds the engine a definition asks for. The same factory serves
// `soundctl tts` (engine from the YAML) and `soundctl say` (engine from flags),
// so both always use the same binary invocation.
func (f *ttsFlags) engineFor(set fwsync.TTSSettings) (tts.Engine, error) {
	switch set.Engine {
	case "supertonic":
		return f.engine(set), nil
	case "piper":
		return tts.Piper{
			Bin:             f.piperBinary(),
			VoiceDir:        f.piperVoiceDir(),
			Voice:           set.Voice,
			SentenceSilence: set.SentenceSilence,
		}, nil
	}
	return nil, fmt.Errorf("tts.engine %q is not supported (use \"supertonic\" or \"piper\")", set.Engine)
}

// engine assembles the Sherpa engine from the flags and the resolved tts block.
func (f *ttsFlags) engine(set fwsync.TTSSettings) tts.Sherpa {
	bin, lib, modelDir := f.ttsBin, f.ttsLibDir, f.ttsModelDir
	if bin == "" {
		bin = filepath.Join(f.ttsDir, "bin", "sherpa-onnx-offline-tts")
	}
	if lib == "" {
		lib = filepath.Join(f.ttsDir, "lib")
	}
	if modelDir == "" {
		modelDir = filepath.Join(f.ttsDir, "models", set.Model)
	}
	threads := f.threads
	if threads == 0 {
		threads = set.Threads
	}
	return tts.Sherpa{Bin: bin, LibDir: lib, ModelDir: modelDir, Threads: threads, NumSteps: f.steps}
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

	if f.check {
		if len(cfg.TTSSounds()) == 0 {
			fmt.Fprintf(out, "no tts sounds in %s\n", f.config)
			return nil
		}
		pending := tts.Pending(cfg, layout)
		if len(pending) == 0 {
			fmt.Fprintf(out, "OK: %d tts sound(s) up to date\n", len(cfg.TTSSounds()))
			return nil
		}
		for _, name := range pending {
			fmt.Fprintf(out, "stale: %s\n", name)
		}
		return fmt.Errorf("%d of %d tts render(s) missing or stale", len(pending), len(cfg.TTSSounds()))
	}

	if len(cfg.TTSSounds()) == 0 {
		fmt.Fprintf(out, "no tts sounds in %s\n", f.config)
		return nil
	}

	eng, err := f.engineFor(cfg.TTSSettings())
	if err != nil {
		return err
	}
	if err := eng.Available(); err != nil {
		if fw := cfg.TTSSettings().Engine; fw == "piper" {
			return fmt.Errorf("%w (install piper and the voice, or pass --piper-bin/--piper-dir)", err)
		}
		return fmt.Errorf("%w (run \"soundctl tts install\" or set --tts-dir)", err)
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

func newTTSCmd() *cobra.Command {
	f := &ttsFlags{}
	cmd := &cobra.Command{
		Use:   "tts",
		Short: "Render the tts: texts of sounds_*.yaml into the firmware MP3s",
		Long: "Synthesise every sound that has a 'tts:' text and write it as a 16 kHz mono\n" +
			"CBR MP3 into the sound overlay (sounds/<language>/). Each file gets a\n" +
			"\".json\" sidecar with the render fingerprint, so a re-run only touches what\n" +
			"changed: the text, the voice (sid/speed), the language or the bitrate.\n" +
			"\n" +
			"Runs once per ROS start inside the container; \"tts --check\" only reports\n" +
			"what would be rendered (for CI), and sync refuses to upload a stale render.\n" +
			"\n" +
			"The engine is chosen per definition in its \"tts:\" block: \"supertonic\"\n" +
			"(sherpa-onnx, model below --tts-dir, voice by sid) or \"piper\" (voice and\n" +
			"binaries below --piper-dir/--piper-bin). Both are baked into the image;\n" +
			"\"tts install\" fetches the Supertonic engine/model for host use.",
		Example: "  soundctl tts --config src/open_mower/params/sounds_de.yaml --sounds-dir /data/sounds\n" +
			"  soundctl tts --check",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error { return f.run(cmd) },
	}
	f.register(cmd)
	cmd.AddCommand(newTTSInstallCmd(), newTTSVerifyCmd())
	return cmd
}
