// Package cli contains the soundctl command-line interface.
package cli

import (
	"context"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"
)

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
	root := newRootCmd()

	// One signal context for the whole run. Commands take it from cmd.Context(),
	// so Ctrl-C or SIGTERM also aborts what they are waiting for - a discovery
	// window, an upload chunk or a retry sleep - instead of only ending the
	// process between two steps.
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	// SilenceErrors keeps cobra from printing, so the one error line goes through the
	// (possibly prefixed) writer like every other line.
	err := root.ExecuteContext(ctx)
	if err != nil {
		printError(root, err)
	}
	return err
}

func newRootCmd() *cobra.Command {
	var logOpts logOptions

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
		// The flags are parsed at this point, so this is the first place the log
		// format can be applied - and the last one before a command prints anything.
		PersistentPreRun: func(cmd *cobra.Command, _ []string) {
			logOpts.apply(cmd)
		},
	}
	logOpts.register(root)
	root.AddCommand(newSyncCmd(), newStorageCmd(), newValidateCmd(), newTTSCmd(), newVersionCmd(), newPlayCmd(), newStopCmd(), newDiscoverCmd(), newSayCmd())
	return root
}
