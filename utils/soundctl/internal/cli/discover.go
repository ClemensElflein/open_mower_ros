package cli

import (
	"context"
	"errors"
	"fmt"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/xbot/services/fileservice"
	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

// newDiscoverCmd lists the services the low level advertises on the xbot
// discovery multicast group. It is the first thing to run when a sync cannot
// reach the LL: it answers "is the LL there at all?" and "does this firmware
// expose the FileService (SID 12)?" in one shot.
func newDiscoverCmd() *cobra.Command {
	var (
		bind string
		wait time.Duration
	)
	cmd := &cobra.Command{
		Use:   "discover",
		Short: "List the xbot services the low level advertises",
		Long: "Listen on the xbot service discovery group and print every advertised\n" +
			"service (ID, type, version, endpoint).\n" +
			"\n" +
			"An unclaimed service advertises every second, but only for the first minute\n" +
			"after the low level started; afterwards (and once claimed) it advertises\n" +
			"every 10 s. That is why the window defaults to 25 s and why \"soundctl sync\"\n" +
			"waits 30 s for the FileService.\n" +
			"\n" +
			"Useful when a sync reports that the FileService is not reachable: if SID 12\n" +
			"is missing here, the LL firmware does not implement it; if the list is empty,\n" +
			"the LL is not running or not on this network interface.",
		Example: "  soundctl discover --wait 25s\n" +
			"  soundctl discover --bind 172.16.78.1",
		Args: cobra.NoArgs,
		RunE: func(cmd *cobra.Command, args []string) error {
			ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
			defer stop()
			ctx, cancel := context.WithTimeout(ctx, wait)
			defer cancel()

			services, err := xbot.Discover(ctx, bind)
			out := cmd.OutOrStdout()
			if len(services) == 0 {
				// An empty window is the interesting case: nothing at all on the bus.
				if err == nil || errors.Is(err, context.DeadlineExceeded) {
					return fmt.Errorf("no xbot service advertised within %s — is the LL running on this network?", wait)
				}
				return err
			}
			if err != nil && !errors.Is(err, context.DeadlineExceeded) {
				return err
			}
			fmt.Fprintln(out, "SID TYPE                           VER ENDPOINT")
			for _, svc := range services {
				fmt.Fprintln(out, svc)
			}
			if !hasService(services, fileservice.ServiceID) {
				fmt.Fprintf(cmd.ErrOrStderr(),
					"note: the FileService (SID %d) was not advertised — \"soundctl sync\" cannot upload MP3s\n"+
						"      until the LL firmware implements it.\n", fileservice.ServiceID)
			}
			return nil
		},
	}
	cmd.Flags().StringVar(&bind, "bind", "0.0.0.0", "local IP of the interface to listen on")
	cmd.Flags().DurationVar(&wait, "wait", 25*time.Second, "how long to listen (a claimed/idle service advertises every 10 s)")
	return cmd
}

// hasService reports whether the list contains the given service id.
func hasService(services []xbot.Advertisement, sid uint16) bool {
	for _, svc := range services {
		if svc.ServiceID == sid {
			return true
		}
	}
	return false
}
