package sync

import (
	"context"
	"fmt"
	"hash/crc32"
	"log/slog"
	"os"
	"path/filepath"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/xbot"
)

// Options configures a sync run.
type Options struct {
	ConfigPath   string
	SoundIDsPath string
	BindIP       string
	Heartbeat    time.Duration
	RPCTimeout   time.Duration
}

// Sync uploads the MP3 files referenced by the definition that are missing
// on the firmware or whose content hash differs.
//
// TODO: once the FileService exposes a FileList RPC, also remove firmware
// files that are no longer referenced by the definition.
func Sync(ctx context.Context, opts Options) error {
	cfg, err := LoadConfig(opts.ConfigPath)
	if err != nil {
		return err
	}
	ids, err := LoadSoundIDs(opts.SoundIDsPath)
	if err != nil {
		return err
	}
	if errs := cfg.Validate(ids); len(errs) > 0 {
		for _, e := range errs {
			slog.Error("invalid config", "error", e)
		}
		return fmt.Errorf("invalid config: %d error(s)", len(errs))
	}

	files := cfg.Files()
	if len(files) == 0 {
		slog.Info("no MP3 files in definition, nothing to upload")
		return nil
	}

	slog.Info("connecting to FileService", "bind", opts.BindIP)
	fs, err := xbot.NewFileService(ctx, opts.BindIP, opts.Heartbeat, opts.RPCTimeout)
	if err != nil {
		return fmt.Errorf("FileService: %w", err)
	}
	defer fs.Close()
	slog.Info("FileService connected")

	for _, file := range files {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := uploadFile(ctx, fs, filepath.Join(cfg.SoundPath, file), file); err != nil {
			return err
		}
	}
	return nil
}

func uploadFile(ctx context.Context, fs *xbot.FileService, localPath, remoteName string) error {
	_ = ctx
	data, err := os.ReadFile(localPath)
	if err != nil {
		return fmt.Errorf("read %s: %w", localPath, err)
	}
	crc := crc32.ChecksumIEEE(data)
	remote := "/sounds/" + remoteName

	exists, err := fs.FileExists(remote, crc)
	if err != nil {
		return fmt.Errorf("FileExists(%s): %w", remote, err)
	}
	if exists {
		slog.Info("up to date", "file", remoteName)
		return nil
	}

	slog.Info("uploading", "file", remoteName, "bytes", len(data))
	for off := 0; off < len(data); off += xbot.FileChunkSize {
		end := off + xbot.FileChunkSize
		if end > len(data) {
			end = len(data)
		}
		chunk := data[off:end]
		isLast := end == len(data)
		var h uint32
		if isLast {
			h = crc
		}
		n, err := fs.FileWrite(remote, uint32(off), chunk, h)
		if err != nil {
			return fmt.Errorf("FileWrite(%s @%d): %w", remote, off, err)
		}
		if !isLast && n != int32(len(chunk)) {
			return fmt.Errorf("FileWrite(%s @%d): short write %d != %d", remote, off, n, len(chunk))
		}
	}

	ok, err := fs.FileExists(remote, crc)
	if err != nil {
		return fmt.Errorf("verify FileExists(%s): %w", remote, err)
	}
	if !ok {
		return fmt.Errorf("verify failed for %s", remote)
	}
	slog.Info("uploaded", "file", remoteName)
	return nil
}
