// Package cli contains the soundctl command-line interface.
package cli

import "github.com/spf13/cobra"

const (
	// Container paths inside the open_mower_ros image. Both can be overridden with
	// --config / --sound-ids when running soundctl outside the container.
	defaultConfigPath   = "/opt/open_mower_ros/src/open_mower/params/sounds_en.yaml"
	defaultSoundIDsPath = "/opt/open_mower_ros/services/sound_service.json"
)

var version = "dev"

// SetVersion sets the version string (intended for -ldflags injection).
func SetVersion(v string) { version = v }

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
			"tts: texts, \"play\" and \"stop\" trigger sounds on a running robot, and\n" +
			"\"discover\" lists the services the low level advertises (the first thing to\n" +
			"look at when a sync cannot reach it).\n" +
			"\n" +
			"Sound names, types and waveforms come from the shared SoundService definition\n" +
			"(sound_service.json, a git submodule of this repo); override its location with\n" +
			"--sound-ids when running outside the container.",
		SilenceUsage:  true,
		SilenceErrors: true,
	}
	root.AddCommand(newSyncCmd(), newValidateCmd(), newTTSCmd(), newVersionCmd(), newPlayCmd(), newStopCmd(), newDiscoverCmd())
	return root
}
