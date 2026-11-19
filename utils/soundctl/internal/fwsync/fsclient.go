package fwsync

import (
	"context"
	"errors"
	"fmt"
	"time"

	"github.com/xtech/definitions-open-mower/golang/fileservice"
	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

const (
	busyRetries    = 3
	busyRetryDelay = 2 * time.Millisecond
)

// busyRetry repeats an RPC a few times while the low level answers BUSY. A large return
// value holds one of the pool blocks for a moment, and the low level handles one RPC at a
// time, so the same call normally succeeds right after. Every other error is returned
// unchanged - a real failure will not get better by asking again.
func busyRetry[T any](call func() (T, error)) (T, error) {
	var (
		value T
		err   error
	)
	for attempt := 0; attempt < busyRetries; attempt++ {
		value, err = call()
		if !errors.Is(err, xbot.ErrBusy) {
			return value, err
		}
		time.Sleep(busyRetryDelay)
	}
	return value, err
}

// busyRetryErr is busyRetry for a call that returns only an error (a send-only RPC or a
// configuration transaction). BUSY means the call did not run, so repeating it is safe.
func busyRetryErr(call func() error) error {
	_, err := busyRetry(func() (struct{}, error) { return struct{}{}, call() })
	return err
}

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

	raw, err := busyRetry(func() ([]uint32, error) { return fs.FileStats(ctx) })
	if err != nil {
		return FileStats{}, fmt.Errorf("FileStats: %w", err)
	}
	return ParseFileStats(raw)
}
