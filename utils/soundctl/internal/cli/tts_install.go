package cli

import (
	"fmt"
	"runtime"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

func newTTSInstallCmd() *cobra.Command {
	var (
		dir     string
		arch    string
		engine  string
		model   string
		voice   string
		baseURL string
		force   bool
	)
	cmd := &cobra.Command{
		Use:   "install",
		Short: "Download a TTS engine (and its model/voice) into --dir",
		Long: "Fetch the pinned engine assets, verify their SHA-256 and unpack them:\n" +
			"\n" +
			"  --engine supertonic  sherpa-onnx CLI + Supertonic 3 model into\n" +
			"                       <dir>/bin, <dir>/lib, <dir>/models\n" +
			"  --engine piper       piper standalone bundle + one voice into\n" +
			"                       <dir>/piper (voices below <dir>/piper/voices)\n" +
			"\n" +
			"This is what the image build runs; archives are cached in <dir>/.cache, so\n" +
			"re-running is cheap. Only files below <dir> are touched.",
		Example: "  soundctl tts install --dir /opt/open_mower_ros/tts --arch arm64\n" +
			"  soundctl tts install --engine piper --dir /opt/open_mower_ros/tts\n" +
			"  soundctl tts install --engine piper --voice de_DE-thorsten-high --dir ~/.cache/openmower-tts",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			opts := tts.InstallOptions{
				Dir:     dir,
				Arch:    arch,
				Model:   model,
				Voice:   voice,
				Force:   force,
				BaseURL: baseURL,
				Log: func(format string, args ...any) {
					fmt.Fprintf(cmd.OutOrStdout(), format+"\n", args...)
				},
			}
			switch engine {
			case "", "supertonic":
				return tts.Install(opts)
			case "piper":
				return tts.InstallPiper(opts)
			}
			return fmt.Errorf("--engine %q is not supported (use \"supertonic\" or \"piper\")", engine)
		},
	}
	cmd.Flags().StringVar(&dir, "dir", defaultTTSDir(), "target directory")
	cmd.Flags().StringVar(&arch, "arch", "", fmt.Sprintf("target architecture (default: %s)", runtime.GOARCH))
	cmd.Flags().StringVar(&engine, "engine", "supertonic", "engine to install: supertonic or piper")
	cmd.Flags().StringVar(&model, "model", tts.DefaultModel, "supertonic: model directory name")
	cmd.Flags().StringVar(&voice, "voice", tts.DefaultPiperVoice,
		fmt.Sprintf("piper: voice to fetch (available: %v)", tts.PiperVoiceNames()))
	cmd.Flags().StringVar(&baseURL, "base-url", "", "download host override (mirror); default is GitHub/Hugging Face")
	cmd.Flags().BoolVar(&force, "force", false, "re-download and re-extract even if the files are present")
	return cmd
}

func newTTSVerifyCmd() *cobra.Command {
	var dir, model, engine, voice string
	cmd := &cobra.Command{
		Use:   "verify",
		Short: "Check that a TTS installation is complete",
		Long: "Check that the engine binaries, libraries and model/voice files are present\n" +
			"(no download, no engine run). Used by the image build and the CI render.",
		Example: "  soundctl tts verify --dir /opt/open_mower_ros/tts\n" +
			"  soundctl tts verify --engine piper --dir /opt/open_mower_ros/tts",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			var err error
			switch engine {
			case "", "supertonic":
				err = tts.Verify(dir, model)
			case "piper":
				err = tts.VerifyPiper(dir, voice)
			default:
				return fmt.Errorf("--engine %q is not supported (use \"supertonic\" or \"piper\")", engine)
			}
			if err != nil {
				return fmt.Errorf("%s: %w", dir, err)
			}
			fmt.Fprintf(cmd.OutOrStdout(), "OK: %s (%s)\n", dir, engine)
			return nil
		},
	}
	cmd.Flags().StringVar(&dir, "dir", defaultTTSDir(), "installation directory to check")
	cmd.Flags().StringVar(&engine, "engine", "supertonic", "engine to check: supertonic or piper")
	cmd.Flags().StringVar(&model, "model", tts.DefaultModel, "supertonic: model directory name")
	cmd.Flags().StringVar(&voice, "voice", tts.DefaultPiperVoice, "piper: voice that must be present")
	return cmd
}
