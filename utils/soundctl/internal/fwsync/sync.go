package fwsync

import (
	"context"
	"fmt"
	"hash/crc32"
	"log/slog"
	"net"
	"os"
	"path/filepath"
	"strconv"
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
	Volume       int // master volume 0..100; < 0 leaves the firmware value unchanged

	// SoundCheckTimeout bounds the "does this board have sound hardware?" probe
	// (i.e. is the SoundService advertised). Zero uses defaultSoundCheckTimeout.
	SoundCheckTimeout time.Duration
}

// defaultSoundCheckTimeout bounds the SoundService probe when not overridden. It
// must cover the advertisement interval (a claimed service advertises slowly), but
// should not be so long that a soundless robot wastes the whole --wait window.
const defaultSoundCheckTimeout = 10 * time.Second

// Sync uploads the MP3 files the definition references but the firmware is
// missing (or whose content hash differs), removes /sounds/ files that are no
// longer referenced, and pushes the sound definitions to the SoundService.
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

	// The SoundService only runs when the board has sound hardware, so probe it
	// first: a soundless robot is left completely untouched. It is never claimed,
	// because the high-level system owns it — pushing definitions and the volume
	// are one-way messages.
	soundTimeout := opts.SoundCheckTimeout
	if soundTimeout <= 0 {
		soundTimeout = defaultSoundCheckTimeout
	}
	soundCtx, cancel := context.WithTimeout(ctx, soundTimeout)
	ss, err := xbot.NewSoundServiceNoClaim(soundCtx, opts.BindIP)
	cancel()
	if err != nil {
		slog.Info("no SoundService reachable — nothing to configure", "timeout", soundTimeout, "error", err)
		return nil
	}
	defer func() { _ = ss.Close() }()
	ip, port := ss.Endpoint()
	slog.Info("SoundService available", "addr", net.JoinHostPort(ip, strconv.Itoa(port)),
		"hint", "pass it to \"soundctl play --addr\" for instant playback")

	// The FileService is always present, so connecting to it doubles as the "LL is
	// up" gate: its discovery waits out the boot/update window (--wait). Connecting
	// claims it, which is fine — that is also how the MP3 upload gets its replies.
	slog.Info("connecting to FileService", "bind", opts.BindIP)
	fs, err := xbot.NewFileService(ctx, opts.BindIP, opts.Heartbeat, opts.RPCTimeout)
	if err != nil {
		return fmt.Errorf("FileService not advertised — is the LL running? (%w)", err)
	}
	defer func() { _ = fs.Close() }()
	slog.Info("FileService connected")

	files := cfg.Files()
	if len(files) == 0 {
		slog.Info("no MP3 files in definition")
	} else {
		for _, file := range files {
			if err := ctx.Err(); err != nil {
				return err
			}
			if err := uploadFile(fs, filepath.Join(cfg.SoundPath, file), file); err != nil {
				return err
			}
		}
	}

	if err := cleanup(fs, cfg); err != nil {
		return err
	}

	if err := applyDefinitions(ctx, cfg, opts, ss); err != nil {
		return err
	}
	slog.Info("sync complete")
	return nil
}

// applyDefinitions sends the sound-definitions blob (heatshrink-compressed) to
// the SoundService and, optionally, the master volume.
//
// The update is a configuration transaction, and the firmware restarts the service
// for it (Service::HandleConfigurationTransaction does Stop() followed by Start()).
// That is why the volume is sent over a fresh connection: the one used for the blob
// is stale afterwards. A failure there is only a warning — the definitions, which
// matter most, are already applied.
func applyDefinitions(ctx context.Context, cfg *Config, opts Options, ss *xbot.SoundService) error {
	blob, err := cfg.Blob()
	if err != nil {
		return fmt.Errorf("marshal definitions: %w", err)
	}
	encoded := xbot.HeatshrinkEncode(blob)
	slog.Info("sound definitions", "sounds", len(cfg.Sounds), "json_bytes", len(blob), "compressed_bytes", len(encoded))

	if err := ss.SetDefinitions(encoded); err != nil {
		return fmt.Errorf("SetDefinitions: %w", err)
	}
	slog.Info("sound definitions sent")

	if opts.Volume < 0 {
		return nil
	}
	if err := validateVolume(opts.Volume); err != nil {
		return err
	}

	ss, err = xbot.NewSoundServiceNoClaim(ctx, opts.BindIP)
	if err != nil {
		slog.Warn("could not reconnect after the restart — master volume not sent", "error", err)
		return nil
	}
	defer func() { _ = ss.Close() }()

	if err := ss.SetVolume(uint8(opts.Volume)); err != nil {
		slog.Warn("could not send the master volume", "error", err)
		return nil
	}
	slog.Info("master volume sent", "volume", opts.Volume)
	return nil
}

func uploadFile(fs *xbot.FileService, localPath, remoteName string) error {
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

// cleanup removes any /sounds/ file that is no longer referenced by the
// definition (an orphan), so the firmware never keeps stale MP3s. It also logs
// what it finds so the sync run is easy to follow.
func cleanup(fs *xbot.FileService, cfg *Config) error {
	// Files the definition currently references.
	referenced := make(map[string]bool)
	for _, f := range cfg.Files() {
		referenced["/sounds/"+f] = true
	}

	existing, err := listAllFiles(fs)
	if err != nil {
		return fmt.Errorf("FileList: %w", err)
	}
	slog.Info("found files in /sounds/", "count", len(existing))

	var orphans []xbot.FileEntry
	for _, e := range existing {
		if !referenced[e.Path] {
			orphans = append(orphans, e)
		}
	}
	if len(orphans) == 0 {
		slog.Info("no orphaned files")
		return nil
	}

	slog.Info("orphaned files", "count", len(orphans))
	for _, o := range orphans {
		ok, err := fs.FileRemove(o.Path)
		if err != nil {
			return fmt.Errorf("FileRemove(%s): %w", o.Path, err)
		}
		if ok {
			slog.Info("removed orphan", "path", o.Path)
		} else {
			slog.Warn("failed to remove orphan", "path", o.Path)
		}
	}
	return nil
}

// listAllFiles pages through FileList until every /sounds/ entry is collected.
func listAllFiles(fs *xbot.FileService) ([]xbot.FileEntry, error) {
	var all []xbot.FileEntry
	var offset uint32
	for {
		total, entries, err := fs.FileList("/sounds/", offset)
		if err != nil {
			return nil, err
		}
		all = append(all, entries...)
		offset += uint32(len(entries))
		if len(entries) == 0 || offset >= total {
			break
		}
	}
	return all, nil
}
