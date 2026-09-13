package fwsync

import (
	"context"
	"fmt"
	"log/slog"
	"net"
	"strconv"
	"strings"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/xbot"
)

// ToneSpec is a runtime tone. The fields are plain ints, so the same validation
// rules as for the YAML definitions can be applied before casting to the wire
// types (a direct uint8()/uint16() cast would silently wrap).
type ToneSpec struct {
	Freq       int
	DurationMs int
	Volume     int
}

// SequenceSpec is a runtime note sequence ("freq:dur[:lfoHzx10[:depth]] ...").
// Notes is the compact text the firmware parses on its own; Waveform is a name
// from the shared definition (sine/square/triangle/saw). AttackMs/DecayMs are the
// per-note envelope (0..255 ms, 0 = instant onset / hold the note).
type SequenceSpec struct {
	Notes    string
	Waveform string
	Volume   int
	Unison   int
	DetuneHz int
	AttackMs int
	DecayMs  int
}

// toSoundDef expresses both specs in the Config/SoundDef representation, i.e. in
// the form validateSound understands. Sharing one representation keeps the
// runtime path (`soundctl play`) and the YAML path (sounds_*.yaml) in sync.
func (t *ToneSpec) toSoundDef() SoundDef {
	return SoundDef{Type: "tone", Volume: t.Volume, Tone: &Tone{Freq: t.Freq, DurationMs: t.DurationMs}}
}

func (s *SequenceSpec) toSoundDef() SoundDef {
	return SoundDef{Type: "sequence", Waveform: s.Waveform, Volume: s.Volume, Unison: s.Unison,
		DetuneHz: s.DetuneHz, AttackMs: s.AttackMs, DecayMs: s.DecayMs}
}

// PlayOptions configures a playback run: exactly one of Stop/Tone/Sequence/
// Mp3Path/Sound is used.
type PlayOptions struct {
	SoundIDsPath string
	BindIP       string
	// Addr is an optional "ip:port" of the SoundService. When set, the (slow)
	// advertisement-based discovery is skipped entirely.
	Addr string

	Stop     bool
	Sound    string // snake_case SoundId, e.g. "boot_ping"
	Tone     *ToneSpec
	Sequence *SequenceSpec
	Mp3Path  string

	// Volume sets the master volume (0..100) before playing; < 0 leaves the
	// firmware value unchanged.
	Volume int
	// Preempt lets the ad-hoc commands (tone/sequence/mp3) stop a running sound and
	// drop the queue. Configured sounds ignore it: their definition decides (see the
	// "preempt" attribute in sounds_*.yaml).
	Preempt bool
}

// Play connects to the SoundService and triggers one playback action at runtime
// (no sound-definition reconfiguration, nothing is persisted).
func Play(ctx context.Context, opts PlayOptions) error {
	// Load the shared definition and validate the request before any network work:
	// a bad sound id or value should fail immediately, not after a slow discovery.
	var ids *SoundIDs
	if needsDefinition(opts) {
		var err error
		if ids, err = LoadSoundIDs(opts.SoundIDsPath); err != nil {
			return err
		}
	}
	if err := validatePlay(opts, ids); err != nil {
		return err
	}

	svc, err := connect(ctx, opts)
	if err != nil {
		return err
	}
	defer func() { _ = svc.Close() }()

	if opts.Volume >= 0 {
		if err := svc.SetVolume(uint8(opts.Volume)); err != nil {
			return fmt.Errorf("set master volume: %w", err)
		}
		slog.Info("master volume set", "volume", opts.Volume)
	}

	if err := dispatchPlay(svc, opts, ids); err != nil {
		return err
	}
	// RPC responses go to the service owner (the high-level system), so there is
	// no result to check — the sound itself is the feedback.
	slog.Info("command sent")
	return nil
}

// connect returns a (unclaimed) SoundService connection: either to the endpoint
// given with --addr, or from the multicast discovery.
//
// There is deliberately no endpoint cache: a stale address would silently swallow
// the fire-and-forget playback RPCs, and when instant calls matter the address
// from the `soundctl sync` log can just be passed with --addr.
func connect(ctx context.Context, opts PlayOptions) (*xbot.SoundService, error) {
	if opts.Addr != "" {
		svc, err := serviceAt(opts.Addr)
		if err != nil {
			return nil, err
		}
		slog.Info("using endpoint from --addr", "addr", opts.Addr)
		return svc, nil
	}

	slog.Info("discovering SoundService (can take a few seconds — claimed services advertise slowly)…")
	svc, err := xbot.NewSoundServiceNoClaim(ctx, opts.BindIP)
	if err != nil {
		return nil, fmt.Errorf("SoundService: %w (hint: pass --addr <ip:port>, e.g. from the `soundctl sync` log)", err)
	}
	ip, port := svc.Endpoint()
	slog.Info("SoundService found", "addr", net.JoinHostPort(ip, strconv.Itoa(port)),
		"hint", "pass it with --addr for instant calls")
	return svc, nil
}

// serviceAt connects to "ip:port" without discovery or claim.
func serviceAt(addr string) (*xbot.SoundService, error) {
	host, portStr, err := net.SplitHostPort(addr)
	if err != nil {
		return nil, fmt.Errorf("bad endpoint %q (want \"ip:port\"): %w", addr, err)
	}
	port, err := strconv.Atoi(portStr)
	if err != nil || port <= 0 || port > 65535 {
		return nil, fmt.Errorf("bad endpoint %q: invalid port", addr)
	}
	svc, err := xbot.NewSoundServiceAt(host, port)
	if err != nil {
		return nil, fmt.Errorf("SoundService: %w", err)
	}
	return svc, nil
}

// needsDefinition reports whether the action has to be checked against the shared
// definition. "stop" and an MP3 path do not reference it, so those keep working
// even when the definition file is missing (stop is the emergency command).
func needsDefinition(opts PlayOptions) bool { return !opts.Stop && opts.Mp3Path == "" }

// validatePlay checks the requested action against the shared definition (same
// rules as sounds_*.yaml, see validateOne). dispatchPlay relies on this, so no
// value is validated twice. ids is nil only when needsDefinition is false.
func validatePlay(opts PlayOptions, ids *SoundIDs) error {
	if opts.Volume >= 0 {
		if err := validateVolume(opts.Volume); err != nil {
			return err
		}
	}

	switch {
	case opts.Stop, opts.Mp3Path != "": // nothing to check here
	case opts.Tone != nil:
		return validateOne("tone", opts.Tone.toSoundDef(), ids)
	case opts.Sequence != nil:
		return validateOne("sequence", opts.Sequence.toSoundDef(), ids)
	case opts.Sound != "":
		if _, ok := ids.Value(opts.Sound); !ok {
			return fmt.Errorf("unknown sound %q (known: %s)", opts.Sound, strings.Join(ids.Names(), ", "))
		}
	default:
		return fmt.Errorf("nothing to play")
	}
	return nil
}

// dispatchPlay sends the single action selected by opts (already validated by
// validatePlay) and logs it. Feedback is the sound itself: the firmware sends the
// RPC result to the service owner, i.e. the high-level system.
func dispatchPlay(svc *xbot.SoundService, opts PlayOptions, ids *SoundIDs) error {
	switch {
	case opts.Stop:
		slog.Info("stopping playback")
		return svc.Stop()

	case opts.Tone != nil:
		tone := opts.Tone.toSoundDef().Tone
		slog.Info("playing tone", "freq", tone.Freq, "duration_ms", tone.DurationMs, "volume", opts.Tone.Volume,
			"preempt", opts.Preempt)
		return svc.PlayTone(uint16(tone.Freq), uint16(tone.DurationMs), uint8(opts.Tone.Volume), opts.Preempt)

	case opts.Sequence != nil:
		waveform, _ := ids.Waveform(opts.Sequence.Waveform) // validated by validatePlay
		slog.Info("playing sequence", "notes", opts.Sequence.Notes, "waveform", opts.Sequence.Waveform,
			"volume", opts.Sequence.Volume, "unison", opts.Sequence.Unison, "detune_hz", opts.Sequence.DetuneHz,
			"attack_ms", opts.Sequence.AttackMs, "decay_ms", opts.Sequence.DecayMs, "preempt", opts.Preempt)
		return svc.PlaySequence(opts.Sequence.Notes, waveform, uint8(opts.Sequence.Volume),
			uint8(opts.Sequence.Unison), uint16(opts.Sequence.DetuneHz), uint8(opts.Sequence.AttackMs),
			uint8(opts.Sequence.DecayMs), opts.Preempt)

	case opts.Mp3Path != "":
		slog.Info("playing mp3", "path", opts.Mp3Path, "preempt", opts.Preempt)
		return svc.PlayMp3(opts.Mp3Path, opts.Preempt)

	case opts.Sound != "":
		id, _ := ids.Value(opts.Sound) // validated by validatePlay
		slog.Info("playing sound", "sound", opts.Sound, "id", id)
		return svc.PlaySound(id)
	}
	return fmt.Errorf("nothing to play")
}
