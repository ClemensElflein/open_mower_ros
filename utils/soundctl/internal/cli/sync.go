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
	)
	cmd := &cobra.Command{
		Use:   "sync",
		Short: "Upload MP3 files and push the sound definitions to the firmware",
		Long: "Validate sounds_*.yaml, upload the MP3 files it references (skipping files\n" +
			"whose hash already matches), remove orphaned files from /sounds/ and push the\n" +
			"sound definitions to the SoundService.\n" +
			"\n" +
			"This runs once per ROS start, so it is not tuned for latency: it waits for the\n" +
			"services (--wait, covering the boot/update window) and claims the FileService.\n" +
			"\n" +
			"The SoundService is probed first (--sound-check-timeout) and never claimed; on a\n" +
			"board without an amplifier the sync stops there. Its address is logged, so it can\n" +
			"be reused for instant \"soundctl play --addr\" calls.",
		Example: "  soundctl sync --volume 80",
		RunE: func(cmd *cobra.Command, args []string) error {
			ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
			defer stop()
			ctx, cancel := context.WithTimeout(ctx, wait)
			defer cancel()

			return fwsync.Sync(ctx, fwsync.Options{
				ConfigPath:        config,
				SoundIDsPath:      soundIDs,
				BindIP:            bind,
				Heartbeat:         heartbeat,
				RPCTimeout:        timeout,
				Volume:            volume,
				SoundCheckTimeout: soundCheck,
			})
		},
	}
	cmd.Flags().StringVar(&config, "config", defaultConfigPath, "path to sounds_*.yaml")
	cmd.Flags().StringVar(&soundIDs, "sound-ids", defaultSoundIDsPath, "path to the shared definition (sound_service.json)")
	cmd.Flags().StringVar(&bind, "bind", "0.0.0.0", "local IP to bind for discovery")
	cmd.Flags().DurationVar(&wait, "wait", 60*time.Second, "how long to wait for the FileService/SoundService")
	cmd.Flags().DurationVar(&heartbeat, "heartbeat", 10*time.Second, "heartbeat interval (bump for slow flash writes)")
	cmd.Flags().DurationVar(&timeout, "timeout", 8*time.Second, "per-RPC timeout")
	cmd.Flags().IntVar(&volume, "volume", -1, "master volume 0..100 to set (default: leave firmware value unchanged)")
	cmd.Flags().DurationVar(&soundCheck, "sound-check-timeout", 10*time.Second, "how long to wait for the SoundService (soundless boards skip)")
	return cmd
}
