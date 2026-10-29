package fwsync

import (
	"context"
	"errors"
	"fmt"
	"hash/crc32"
	"log/slog"
	"os"
	"time"

	"github.com/xtech/definitions-open-mower/golang/fileservice"
	"github.com/xtech/definitions-open-mower/golang/soundservice"
	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

// Options configures a sync run.
type Options struct {
	ConfigPath   string
	SoundIDsPath string
	BindIP       string
	Heartbeat    time.Duration
	RPCTimeout   time.Duration
	Volume       int // master volume 0..100; < 0 leaves the firmware value unchanged

	// SoundsDir is the writable overlay that holds customised / re-rendered
	// files (on the mower /data/sounds). It takes precedence over sound_path,
	// which stays read-only. Empty means "read everything from sound_path".
	SoundsDir string

	// SoundPath overrides the definition's sound_path when not empty. Only the
	// CLI --sound-path flag sets it (host runs from a checkout, where the
	// absolute image path of the YAML does not exist).
	SoundPath string

	// SoundCheckTimeout bounds the "does this board have sound hardware?" probe
	// (i.e. is the SoundService advertised). Zero uses defaultSoundCheckTimeout.
	SoundCheckTimeout time.Duration

	// Wait bounds the FileService discovery. It is deliberately separate from
	// the SoundService probe and starts when the wait for the FileService starts:
	// a service advertises only every xbot.AdvertisementInterval (10 s) once
	// the low level has been up for a minute, so a shorter window can miss a
	// perfectly healthy LL — and a budget shared with the probe makes it worse.
	// Zero waits as long as the caller's context allows.
	Wait time.Duration
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

	// Resolve where the MP3s live: the writable overlay first, the shipped
	// image second (see Layout). Doing this before touching the robot means a
	// missing or stale file fails the run without side effects.
	layout := NewLayout(cfg, opts.SoundsDir)
	if opts.SoundPath != "" {
		layout.SoundPath = opts.SoundPath
	}
	if err := verifyRenders(layout, cfg); err != nil {
		return err
	}

	// Validate the requested master volume up front: an out-of-range value must
	// fail the run before uploads, cleanup or SetDefinitions change the robot.
	if opts.Volume >= 0 {
		if err := validateVolume(opts.Volume); err != nil {
			return err
		}
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
	ss, err := soundservice.Dial(soundCtx, xbot.Options{NoClaim: true, BindIP: opts.BindIP})
	cancel()
	if err != nil {
		// A cancelled/deadline-exceeded parent context is an abort, not "no sound
		// hardware": propagate it so shutdowns and --wait expiries fail the run.
		if ctxErr := ctx.Err(); ctxErr != nil {
			return ctxErr
		}
		// Only the probe's own timeout means "this board has no SoundService";
		// anything else (bind or multicast-listen failure) is a real error.
		if !errors.Is(err, context.DeadlineExceeded) {
			return fmt.Errorf("SoundService probe: %w", err)
		}
		slog.Info("no SoundService reachable — nothing to configure", "timeout", soundTimeout)
		return nil
	}
	defer func() { _ = ss.Close() }()
	slog.Info("SoundService available", "addr", ss.Endpoint(),
		"hint", "pass it to \"soundctl play --addr\" for instant playback")

	// The FileService is always present, so connecting to it doubles as the "LL is
	// up" gate: its discovery waits out the boot/update window (--wait). Connecting
	// claims it, which is fine — that is also how the MP3 upload gets its replies.
	// The window starts here (not at process start), so the probe above cannot eat
	// into it, and it must cover xbot's slow (10 s) advertisement interval.
	fsCtx := ctx
	if opts.Wait > 0 {
		var cancel context.CancelFunc
		fsCtx, cancel = context.WithTimeout(ctx, opts.Wait)
		defer cancel()
	}
	slog.Info("connecting to FileService", "bind", opts.BindIP, "wait", opts.Wait)
	fs, err := fileservice.Dial(fsCtx, xbot.Options{BindIP: opts.BindIP, Heartbeat: opts.Heartbeat, CallTimeout: opts.RPCTimeout})
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return ctxErr
		}
		return fmt.Errorf("FileService not reachable (run \"soundctl discover\" to see what the LL advertises): %w", err)
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
			local, err := layout.Find(file)
			if err != nil {
				return err
			}
			if err := uploadFile(ctx, fs, local, file); err != nil {
				return err
			}
		}
	}

	// Apply the new definitions before removing orphaned files: if SetDefinitions
	// fails, the firmware still runs the previous definitions, which must not
	// reference files that cleanup is about to delete.
	if err := applyDefinitions(ctx, cfg, opts, ss); err != nil {
		return err
	}

	if err := cleanup(ctx, fs, cfg); err != nil {
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
func applyDefinitions(ctx context.Context, cfg *Config, opts Options, ss *soundservice.Client) error {
	blob, err := cfg.Blob()
	if err != nil {
		return fmt.Errorf("marshal definitions: %w", err)
	}
	encoded := xbot.HeatshrinkEncode(blob)
	slog.Info("sound definitions", "sounds", len(cfg.Sounds), "json_bytes", len(blob), "compressed_bytes", len(encoded))

	if err := ss.SetSoundDefinitions(encoded); err != nil {
		return fmt.Errorf("SetDefinitions: %w", err)
	}
	slog.Info("sound definitions sent")

	if opts.Volume < 0 {
		return nil
	}

	ss, err = soundservice.Dial(ctx, xbot.Options{NoClaim: true, BindIP: opts.BindIP})
	if err != nil {
		slog.Warn("could not reconnect after the restart — master volume not sent", "error", err)
		return nil
	}
	defer func() { _ = ss.Close() }()

	if err := ss.SendVolume(uint8(opts.Volume)); err != nil {
		slog.Warn("could not send the master volume", "error", err)
		return nil
	}
	slog.Info("master volume sent", "volume", opts.Volume)
	return nil
}

// verifyRenders makes sure every tts sound on disk was rendered from the text
// the definition currently contains. A mismatch means the YAML was edited
// without re-rendering them, and uploading the old audio would be silently
// wrong, so this fails the sync instead of guessing.
func verifyRenders(layout Layout, cfg *Config) error {
	for _, name := range cfg.TTSSounds() {
		id, ok := cfg.Identity(name)
		if !ok {
			continue
		}
		file := cfg.FileName(name)
		path, err := layout.Find(file)
		if err != nil {
			return fmt.Errorf("%s: %w (run \"soundctl tts\")", name, err)
		}
		sc, err := ReadSidecar(path)
		if err != nil {
			return fmt.Errorf("%s: no render metadata next to %s (%v) — run \"soundctl tts\"", name, path, err)
		}
		if !sc.Matches(id) {
			return fmt.Errorf("%s: %s is stale (rendered as %s, expected %s) — run \"soundctl tts\"",
				name, path, sc.Fingerprint, id.Fingerprint())
		}
		slog.Info("render up to date", "sound", name, "file", path, "fingerprint", sc.Fingerprint)
	}
	return nil
}

func uploadFile(ctx context.Context, fs *fileservice.Client, localPath, remoteName string) error {
	data, err := os.ReadFile(localPath)
	if err != nil {
		return fmt.Errorf("read %s: %w", localPath, err)
	}

	// The firmware does not resample, so a file in the wrong format plays at the
	// wrong speed. A hand-made file may be intentional, so this only warns — but
	// loudly, because it is almost always a mistake.
	if info, err := ParseMP3(data); err != nil {
		slog.Warn("not a parsable MP3", "file", remoteName, "error", err)
	} else if !info.Is16kMono() {
		slog.Warn("MP3 is not 16 kHz mono — the firmware does not resample", "file", remoteName, "format", info.String())
	}

	crc := crc32.ChecksumIEEE(data)
	remote := "/sounds/" + remoteName

	exists, err := fs.FileExists(ctx, remote, crc)
	if err != nil {
		return fmt.Errorf("FileExists(%s): %w", remote, err)
	}
	if exists == 1 {
		slog.Info("up to date", "file", remoteName)
		return nil
	}

	slog.Info("uploading", "file", remoteName, "bytes", len(data))
	for off := 0; off < len(data); off += fileServiceChunkSize {
		end := off + fileServiceChunkSize
		if end > len(data) {
			end = len(data)
		}
		chunk := data[off:end]
		isLast := end == len(data)
		var h uint32
		if isLast {
			h = crc
		}
		n, err := fs.FileWrite(ctx, remote, uint32(off), chunk, h)
		if err != nil {
			return fmt.Errorf("FileWrite(%s @%d): %w", remote, off, err)
		}
		if !isLast && n != int32(len(chunk)) {
			return fmt.Errorf("FileWrite(%s @%d): short write %d != %d", remote, off, n, len(chunk))
		}
	}

	ok, err := fs.FileExists(ctx, remote, crc)
	if err != nil {
		return fmt.Errorf("verify FileExists(%s): %w", remote, err)
	}
	if ok != 1 {
		return fmt.Errorf("verify failed for %s", remote)
	}
	slog.Info("uploaded", "file", remoteName)
	return nil
}

// cleanup removes any /sounds/ file that is no longer referenced by the
// definition (an orphan), so the firmware never keeps stale MP3s. It also logs
// what it finds so the sync run is easy to follow.
func cleanup(ctx context.Context, fs *fileservice.Client, cfg *Config) error {
	// Files the definition currently references.
	referenced := make(map[string]bool)
	for _, f := range cfg.Files() {
		referenced["/sounds/"+f] = true
	}

	existing, err := listAllFiles(ctx, fs)
	if err != nil {
		return fmt.Errorf("FileList: %w", err)
	}
	slog.Info("found files in /sounds/", "count", len(existing))

	var orphans []fileEntry
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
		ok, err := fs.FileRemove(ctx, o.Path)
		if err != nil {
			return fmt.Errorf("FileRemove(%s): %w", o.Path, err)
		}
		if ok == 0 {
			slog.Info("removed orphan", "path", o.Path)
		} else {
			return fmt.Errorf("FileRemove(%s): firmware reported failure", o.Path)
		}
	}
	return nil
}

// listAllFiles pages through FileList until every /sounds/ entry is collected.
func listAllFiles(ctx context.Context, fs *fileservice.Client) ([]fileEntry, error) {
	var all []fileEntry
	var offset uint32
	for {
		raw, err := fs.FileList(ctx, "/sounds/", offset)
		if err != nil {
			return nil, err
		}
		total, entries, err := parseFileList(raw)
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
