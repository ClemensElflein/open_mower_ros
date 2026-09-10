package cli

import (
	"context"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/sync"
)

func newSyncCmd() *cobra.Command {
	var (
		config    string
		soundIDs  string
		bind      string
		wait      time.Duration
		heartbeat time.Duration
		timeout   time.Duration
		volume    int
	)
	cmd := &cobra.Command{
		Use:   "sync",
		Short: "Upload MP3 files and push the sound definitions to the firmware",
		RunE: func(cmd *cobra.Command, args []string) error {
			ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
			defer stop()
			ctx, cancel := context.WithTimeout(ctx, wait)
			defer cancel()

			return sync.Sync(ctx, sync.Options{
				ConfigPath:   config,
				SoundIDsPath: soundIDs,
				BindIP:       bind,
				Heartbeat:    heartbeat,
				RPCTimeout:   timeout,
				Volume:       volume,
			})
		},
	}
	cmd.Flags().StringVar(&config, "config", defaultConfigPath, "path to sounds_*.yaml")
	cmd.Flags().StringVar(&soundIDs, "sound-ids", defaultSoundIDsPath, "path to sound_service.json")
	cmd.Flags().StringVar(&bind, "bind", "0.0.0.0", "local IP to bind for discovery")
	cmd.Flags().DurationVar(&wait, "wait", 60*time.Second, "how long to wait for the FileService/SoundService")
	cmd.Flags().DurationVar(&heartbeat, "heartbeat", 10*time.Second, "heartbeat interval (bump for slow flash writes)")
	cmd.Flags().DurationVar(&timeout, "timeout", 8*time.Second, "per-RPC timeout")
	cmd.Flags().IntVar(&volume, "volume", -1, "master volume 0..100 to set (default: leave firmware value unchanged)")
	return cmd
}
