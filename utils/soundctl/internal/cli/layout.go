package cli

import (
	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// soundPathHelp documents the --sound-path override; it exists for local runs
// from a checkout, where the absolute /opt/open_mower_ros/sounds of the YAML
// does not exist.
const soundPathHelp = "override the YAML sound_path (for host runs from a checkout)"

// newLayout resolves files like the runtime does. soundPath overrides the
// definition's sound_path when it is not empty, overlay is the writable
// --sounds-dir (may be empty).
func newLayout(cfg *fwsync.Config, soundPath, overlay string) fwsync.Layout {
	layout := fwsync.NewLayout(cfg, overlay)
	if soundPath != "" {
		layout.SoundPath = soundPath
	}
	return layout
}

// layoutFlag registers --sound-path on cmd.
func layoutFlag(cmd *cobra.Command, target *string) {
	cmd.Flags().StringVar(target, "sound-path", "", soundPathHelp)
}
