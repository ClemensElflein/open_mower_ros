package cli

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// newStorageCmd reports the low level's LittleFS usage, i.e. whether the next sound
// set still fits. It claims the FileService like `sync` does, because the answer
// goes to the service owner.
func newStorageCmd() *cobra.Command {
	var (
		bind      string
		wait      time.Duration
		heartbeat time.Duration
		timeout   time.Duration
	)
	cmd := &cobra.Command{
		Use:   "storage",
		Short: "Report the low level's file storage usage (free space)",
		Long: "Ask the FileService for the LittleFS usage of the low level: total, used and\n" +
			"free bytes plus the block size. Useful before a large upload - `soundctl\n" +
			"sync` also logs it, but this works without a definition.\n" +
			"\n" +
			"The free value keeps one block back, because LittleFS needs a free block\n" +
			"while it updates metadata.",
		Example: "  soundctl storage\n" +
			"  soundctl storage --wait 40s",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			// Signals only: the wait budget is applied below, so a slow discovery does
			// not eat into the time a caller gave us.
			ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
			defer stop()

			waitCtx, cancel := context.WithTimeout(ctx, wait)
			defer cancel()

			stats, err := fwsync.Storage(waitCtx, fwsync.Options{
				BindIP:     bind,
				Heartbeat:  heartbeat,
				RPCTimeout: timeout,
			})
			if err != nil {
				return err
			}
			fmt.Fprintln(cmd.OutOrStdout(), stats.String())
			return nil
		},
	}
	fl := cmd.Flags()
	fl.StringVar(&bind, "bind", "0.0.0.0", "local IP to bind for discovery")
	fl.DurationVar(&wait, "wait", 30*time.Second, "how long to wait for the FileService (a service advertises only every 10 s once the LL has been up for a minute)")
	fl.DurationVar(&heartbeat, "heartbeat", 10*time.Second, "heartbeat interval")
	fl.DurationVar(&timeout, "rpc-timeout", 10*time.Second, "how long to wait for the RPC answer")
	return cmd
}
