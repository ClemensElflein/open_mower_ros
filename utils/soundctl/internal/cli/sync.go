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
	)
	cmd := &cobra.Command{
		Use:   "sync",
		Short: "Upload missing or changed MP3 files to the firmware",
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
			})
		},
	}
	cmd.Flags().StringVar(&config, "config", defaultConfigPath, "path to sounds_*.yaml")
	cmd.Flags().StringVar(&soundIDs, "sound-ids", defaultSoundIDsPath, "path to sound_service.json")
	cmd.Flags().StringVar(&bind, "bind", "0.0.0.0", "local IP to bind for discovery")
	cmd.Flags().DurationVar(&wait, "wait", 30*time.Second, "how long to wait for the FileService")
	cmd.Flags().DurationVar(&heartbeat, "heartbeat", 10*time.Second, "heartbeat interval (bump for slow flash writes)")
	cmd.Flags().DurationVar(&timeout, "timeout", 8*time.Second, "per-RPC timeout")
	return cmd
}
