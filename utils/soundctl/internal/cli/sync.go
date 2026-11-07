package cli

import (
	"context"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

func newSyncCmd() *cobra.Command {
	var (
		config     string
		soundIDs   string
		bind       string
		wait       time.Duration
		heartbeat  time.Duration
		timeout    time.Duration
		volume     int
		soundCheck time.Duration
		soundsDir  string
		soundPath  string
	)
	cmd := &cobra.Command{
		Use:   "sync",
		Short: "Upload MP3 files and push the sound definitions to the firmware",
		Long: "Validate sounds_*.yaml, upload the MP3 files it references (skipping files\n" +
			"whose hash already matches), remove orphaned files from /sounds/ and push the\n" +
			"sound definitions to the SoundService.\n" +
			"\n" +
			"This runs once per ROS start, so it is not tuned for latency: it waits for the\n" +
			"services and claims the FileService. An xbot service advertises every second\n" +
			"only while it is unclaimed and younger than a minute, afterwards every 10 s,\n" +
			"so --wait must span a few advertisement periods (that is what the 30 s default\n" +
			"and the 30 s of the startup sync are for).\n" +
			"\n" +
			"The SoundService is probed first (--sound-check-timeout) and never claimed; on a\n" +
			"board without an amplifier the sync stops there. Its address is logged, so it can\n" +
			"be reused for instant \"soundctl play --addr\" calls. When the FileService cannot\n" +
			"be found, the error lists the service IDs that were advertised — run\n" +
			"\"soundctl discover\" to see the full list.",
		Example: "  soundctl sync --volume 80",
		RunE: func(cmd *cobra.Command, args []string) error {
			// Signals only: the wait budget is applied by the domain layer, so that
			// the FileService window starts when the wait starts and not when the
			// process does (a render or the sound probe must not eat into it).
			ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
			defer stop()

			return fwsync.Sync(ctx, fwsync.Options{
				ConfigPath:        config,
				SoundIDsPath:      soundIDs,
				BindIP:            bind,
				Heartbeat:         heartbeat,
				RPCTimeout:        timeout,
				Volume:            volume,
				SoundCheckTimeout: soundCheck,
				SoundsDir:         soundsDir,
				SoundPath:         soundPath,
				Wait:              wait,
			})
		},
	}
	cmd.Flags().StringVar(&config, "config", defaultConfigPath, "path to sounds_*.yaml")
	cmd.Flags().StringVar(&soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	cmd.Flags().StringVar(&bind, "bind", "0.0.0.0", "local IP to bind for discovery")
	cmd.Flags().DurationVar(&wait, "wait", 30*time.Second, "how long to wait for the FileService (a service advertises only every 10 s once the LL has been up for a minute)")
	cmd.Flags().DurationVar(&heartbeat, "heartbeat", 10*time.Second, "heartbeat interval (bump for slow flash writes)")
	cmd.Flags().DurationVar(&timeout, "timeout", 8*time.Second, "per-RPC timeout")
	cmd.Flags().IntVar(&volume, "volume", -1, "master volume 0..100 to set (default: leave firmware value unchanged)")
	cmd.Flags().DurationVar(&soundCheck, "sound-check-timeout", 10*time.Second, "how long to wait for the SoundService (soundless boards skip)")
	cmd.Flags().StringVar(&soundsDir, "sounds-dir", defaultSoundsDir(), "writable overlay with customised/rendered MP3s (wins over sound_path)")
	layoutFlag(cmd, &soundPath)
	return cmd
}
