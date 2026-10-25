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
		model   string
		baseURL string
		force   bool
	)
	cmd := &cobra.Command{
		Use:   "install",
		Short: "Download the TTS engine and model into --tts-dir",
		Long: "Fetch the pinned sherpa-onnx build and the Supertonic 3 model archive,\n" +
			"verify their SHA-256 and unpack them into <dir>/bin, <dir>/lib and\n" +
			"<dir>/models. This is what the image build runs; archives are cached in\n" +
			"<dir>/.cache, so re-running is cheap. Only files below <dir> are touched.",
		Example: "  soundctl tts install --dir /opt/open_mower_ros/tts --arch arm64",
		Args:    cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			return tts.Install(tts.InstallOptions{
				Dir:     dir,
				Arch:    arch,
				Model:   model,
				Force:   force,
				BaseURL: baseURL,
				Log: func(format string, args ...any) {
					fmt.Fprintf(cmd.OutOrStdout(), format+"\n", args...)
				},
			})
		},
	}
	cmd.Flags().StringVar(&dir, "dir", defaultTTSDir(), "target directory (bin/, lib/, models/)")
	cmd.Flags().StringVar(&arch, "arch", "", fmt.Sprintf("target architecture (default: %s)", runtime.GOARCH))
	cmd.Flags().StringVar(&model, "model", tts.DefaultModel, "model directory name")
	cmd.Flags().StringVar(&baseURL, "base-url", "", "download host override (mirror); default is the GitHub release")
	cmd.Flags().BoolVar(&force, "force", false, "re-download and re-extract even if the files are present")
	return cmd
}

func newTTSVerifyCmd() *cobra.Command {
	var dir, model string
	cmd := &cobra.Command{
		Use:   "verify",
		Short: "Check that a TTS installation is complete",
		Long: "Check that the engine binary, its shared libraries and the model files are\n" +
			"present (no download, no engine run). Used by the image build.",
		Example: "  soundctl tts verify --dir /opt/open_mower_ros/tts",
		Args:    cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			if err := tts.Verify(dir, model); err != nil {
				return fmt.Errorf("%s: %w", dir, err)
			}
			fmt.Fprintf(cmd.OutOrStdout(), "OK: %s\n", dir)
			return nil
		},
	}
	cmd.Flags().StringVar(&dir, "dir", defaultTTSDir(), "installation directory to check")
	cmd.Flags().StringVar(&model, "model", tts.DefaultModel, "model directory name")
	return cmd
}
