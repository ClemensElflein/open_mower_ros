package fwsync

import (
	"context"
	"fmt"

	"github.com/xtech/definitions-open-mower/golang/fileservice"
	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

// DialFileService connects to the FileService (SID 12) and claims it: the sync and
// the storage report both need the service's answers, so neither may use a
// fire-and-forget client. The caller bounds the wait with its context.
func DialFileService(ctx context.Context, opts Options) (*fileservice.Client, error) {
	fs, err := fileservice.Dial(ctx, xbot.Options{
		BindIP:      opts.BindIP,
		Heartbeat:   opts.Heartbeat,
		CallTimeout: opts.RPCTimeout,
	})
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf("FileService not reachable (run 'soundctl discover' to see what the LL advertises): %w", err)
	}
	return fs, nil
}

// Storage reports the LittleFS usage of the low level, i.e. how much room an upload
// still has. Only BindIP, Heartbeat and RPCTimeout from opts are used; the wait is
// the caller's context.
func Storage(ctx context.Context, opts Options) (FileStats, error) {
	fs, err := DialFileService(ctx, opts)
	if err != nil {
		return FileStats{}, err
	}
	defer func() { _ = fs.Close() }()

	raw, err := fs.FileStats(ctx)
	if err != nil {
		return FileStats{}, fmt.Errorf("FileStats: %w", err)
	}
	return ParseFileStats(raw)
}
