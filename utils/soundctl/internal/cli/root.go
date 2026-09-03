// Package cli contains the openmower-soundctl command-line interface.
package cli

import "github.com/spf13/cobra"

const (
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
		Use:           "openmower-soundctl",
		Short:         "Manage the OpenMower HW v2 sound definitions and MP3 files",
		SilenceUsage:  true,
		SilenceErrors: true,
	}
	root.AddCommand(newSyncCmd(), newValidateCmd(), newVersionCmd())
	return root
}
