// Package cli contains the soundctl command-line interface.
package cli

import "github.com/spf13/cobra"

const (
	// Container paths inside the open_mower_ros image. Both can be overridden with
	// --config / --sound-ids when running soundctl outside the container.
	defaultConfigPath   = "/opt/open_mower_ros/src/open_mower/params/sounds_en.yaml"
	defaultSoundIDsPath = "/opt/open_mower_ros/services/sound_service.json"
)

// version is what `soundctl version` prints and what `soundctl tts` writes into
// the sidecar's "generator" field. The image build injects the OpenMower version
// with -ldflags "-X .../internal/cli.version=…" (see docker/Dockerfile), so a
// render on the mower can be traced back to an image.
var version = "dev"

// Execute runs the root command.
func Execute() error {
	return newRootCmd().Execute()
}

func newRootCmd() *cobra.Command {
	root := &cobra.Command{
		Use:   "soundctl",
		Short: "Manage the OpenMower HW v2 sound definitions and MP3 files",
		Long: "Manage the sound definitions and MP3 files of an OpenMower HW v2 robot.\n" +
			"\n" +
			"\"sync\" and \"validate\" work on the sounds_*.yaml definition, \"tts\" renders its\n" +
			"tts: texts, \"say\" renders a single text locally and plays it, \"play\" and\n" +
			"\"stop\" trigger sounds on a running robot, and \"discover\" lists the services\n" +
			"the low level advertises (the first thing to look at when a sync cannot\n" +
			"reach it).\n" +
			"\n" +
			"Sound names, types and waveforms come from the shared SoundService definition\n" +
			"(sound_service.json, a git submodule of this repo); override its location with\n" +
			"--sound-ids when running outside the container.",
		SilenceUsage:  true,
		SilenceErrors: true,
	}
	root.AddCommand(newSyncCmd(), newValidateCmd(), newTTSCmd(), newVersionCmd(), newPlayCmd(), newStopCmd(), newDiscoverCmd(), newSayCmd())
	return root
}
