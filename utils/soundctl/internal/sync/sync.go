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
	Volume       int // master volume 0..100; < 0 leaves the firmware value unchanged

	// SoundCheckTimeout bounds the "does this board have sound hardware?" probe
	// (i.e. is the SoundService advertised). Zero uses defaultSoundCheckTimeout.
	SoundCheckTimeout time.Duration
}

// defaultSoundCheckTimeout bounds the SoundService probe when not overridden.
// It must be short: soundless robots never start the SoundService, so we want to
// give up quickly rather than burn the long FileService timeout.
const defaultSoundCheckTimeout = 3 * time.Second

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

	// Gate 1: the FileService is always present, so its advertisement doubles as
	// the "LL is up" gate (this waits out the boot/update window via --wait).
	// Discovery only (no claim), so a soundless robot is left completely
	// untouched.
	if !xbot.IsServiceAvailable(ctx, opts.BindIP, xbot.ServiceFile) {
		return fmt.Errorf("FileService not advertised — is the LL running?")
	}
	slog.Info("FileService available")

	// Gate 2: the SoundService is only started when the board actually has sound
	// hardware. Probe it with a short, dedicated timeout (a context deadline
	// means "no SoundService" = soundless board, not an error) so we skip
	// quickly and never upload MP3s to a robot without sound.
	soundTimeout := opts.SoundCheckTimeout
	if soundTimeout <= 0 {
		soundTimeout = defaultSoundCheckTimeout
	}
	soundCtx, cancel := context.WithTimeout(ctx, soundTimeout)
	hasSound := xbot.IsServiceAvailable(soundCtx, opts.BindIP, xbot.ServiceSound)
	cancel()
	if !hasSound {
		slog.Info("LL has no sound hardware (no SoundService) — nothing to sync")
		return nil
	}
	slog.Info("SoundService available")

	// Both services are present: connect to the FileService and do the work.
	slog.Info("connecting to FileService", "bind", opts.BindIP)
	fs, err := xbot.NewFileService(ctx, opts.BindIP, opts.Heartbeat, opts.RPCTimeout)
	if err != nil {
		return fmt.Errorf("FileService: %w", err)
	}
	defer fs.Close()
	slog.Info("FileService connected")

	files := cfg.Files()
	if len(files) == 0 {
		slog.Info("no MP3 files in definition")
	} else {
		for _, file := range files {
			if err := ctx.Err(); err != nil {
				return err
			}
			if err := uploadFile(ctx, fs, filepath.Join(cfg.SoundPath, file), file); err != nil {
				return err
			}
		}
	}

	if err := cleanup(fs, cfg); err != nil {
		return err
	}

	if err := applyDefinitions(ctx, cfg, opts); err != nil {
		return err
	}
	slog.Info("sync complete")
	return nil
}

// applyDefinitions sends the sound-definitions blob (heatshrink-compressed) to
// the SoundService and, optionally, the master volume.
func applyDefinitions(ctx context.Context, cfg *Config, opts Options) error {
	blob, err := cfg.Blob()
	if err != nil {
		return fmt.Errorf("marshal definitions: %w", err)
	}
	encoded := xbot.HeatshrinkEncode(blob)
	slog.Info("sound definitions", "sounds", len(cfg.Sounds), "json_bytes", len(blob), "compressed_bytes", len(encoded))

	slog.Info("connecting to SoundService", "bind", opts.BindIP)
	ss, err := xbot.NewSoundService(ctx, opts.BindIP, opts.Heartbeat)
	if err != nil {
		return fmt.Errorf("SoundService: %w", err)
	}
	defer ss.Close()
	slog.Info("SoundService connected")

	if err := ss.SetDefinitions(encoded); err != nil {
		return fmt.Errorf("SetDefinitions: %w", err)
	}
	slog.Info("sound definitions sent")

	if opts.Volume >= 0 {
		if opts.Volume > 100 {
			return fmt.Errorf("volume must be 0..100 (got %d)", opts.Volume)
		}
		// A definitions update reconfigures (restarts) the service; wait a
		// moment so the running-only Volume input is accepted.
		time.Sleep(500 * time.Millisecond)
		if err := ss.SetVolume(uint8(opts.Volume)); err != nil {
			return fmt.Errorf("SetVolume: %w", err)
		}
		slog.Info("master volume sent", "volume", opts.Volume)
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
