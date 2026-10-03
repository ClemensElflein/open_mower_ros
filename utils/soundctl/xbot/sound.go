package xbot

import (
	"context"
	"encoding/binary"
	"fmt"
)

// SoundService register/input IDs (see services/sound_service.json).
const (
	soundRegisterDefinitions = 0
	soundInputVolume         = 0
)

// SoundService RPC function IDs (see services/sound_service.json -> "functions").
const (
	soundFunctionPlaySound    uint8 = 0
	soundFunctionPlayTone     uint8 = 1
	soundFunctionPlaySequence uint8 = 2
	soundFunctionPlayMp3      uint8 = 3
	soundFunctionStop         uint8 = 4
)

// Max lengths of the string RPC parameters (see sound_service.json).
const (
	maxSequenceLen = 160
	maxPathLen     = 128
)

// SoundService is a thin client for the SoundService (service id 13).
type SoundService struct {
	c *Conn
}

// NewSoundServiceNoClaim discovers and connects to the SoundService without
// claiming it. This is the normal mode on the robot: the high-level system owns
// (claims) the service, and definitions, the volume and the playback RPCs are all
// one-way messages.
func NewSoundServiceNoClaim(ctx context.Context, bindIP string) (*SoundService, error) {
	c, err := DialNoClaim(ctx, ServiceSound, bindIP)
	if err != nil {
		return nil, err
	}
	return &SoundService{c: c}, nil
}

// NewSoundServiceAt connects to a SoundService at a known endpoint (no discovery,
// no claim) — instant, unlike the advertisement-based discovery. See
// `soundctl play --addr`.
func NewSoundServiceAt(ip string, port int) (*SoundService, error) {
	c, err := DialNoClaimTo(ip, port, ServiceSound)
	if err != nil {
		return nil, err
	}
	return &SoundService{c: c}, nil
}

// Close closes the underlying connection.
func (s *SoundService) Close() error { return s.c.Close() }

// Endpoint returns the unicast address of the connected SoundService.
func (s *SoundService) Endpoint() (string, int) { return s.c.Endpoint() }

// ---------------------------------------------------------------------------
// Configuration: register and input updates (slow, persisted by the firmware).
// ---------------------------------------------------------------------------

// SetDefinitions sends the sound-definitions blob (already heatshrink-encoded)
// as the "Sound Definitions" register. The firmware treats a register update as
// a reconfiguration (it restarts the service) and persists it to flash.
func (s *SoundService) SetDefinitions(blob []byte) error {
	body := append(PackDescriptor(soundRegisterDefinitions, uint32(len(blob))), blob...)
	return s.c.send(MsgTransaction, 1 /* configuration transaction */, 0, body)
}

// SetVolume sends the master volume (0..100) as the "Volume" input. The service
// must be running (i.e. configured) for the input to be accepted.
func (s *SoundService) SetVolume(volume uint8) error {
	return s.c.send(MsgData, 0, soundInputVolume, []byte{volume})
}

// ---------------------------------------------------------------------------
// Runtime playback RPCs — fire and forget: the firmware returns its result to
// the service owner (the high-level system), so we cannot read it here. Whether
// it worked is audible (and logged by the firmware as "Sound: RPC …").
// ---------------------------------------------------------------------------

// PlaySound plays a configured sound (SoundId) immediately.
func (s *SoundService) PlaySound(id uint8) error {
	return s.c.SendRPC(soundFunctionPlaySound, []Param{{ID: 0, Data: []byte{id}}})
}

// PlayTone plays a single synthesised tone. preempt stops a running sound, drops the
// queue and plays immediately.
func (s *SoundService) PlayTone(freq, durationMs uint16, volume uint8, preempt bool) error {
	return s.c.SendRPC(soundFunctionPlayTone, packPlayTone(freq, durationMs, volume, preempt))
}

// PlaySequence plays a compact note sequence, e.g. "250:60 0:40 375:80".
// attackMs/decayMs are the per-note envelope (0 = instant onset / hold the note),
// repeatMs repeats the sequence every n ms until another sound plays (0 = once);
// see the firmware synth and player.
func (s *SoundService) PlaySequence(sequence string, waveform, volume, unison uint8, detuneHz uint16, attackMs, decayMs uint8,
	repeatMs uint16, preempt bool) error {
	if len(sequence) > maxSequenceLen {
		return fmt.Errorf("sequence too long (%d chars, max %d)", len(sequence), maxSequenceLen)
	}
	return s.c.SendRPC(soundFunctionPlaySequence,
		packPlaySequence(sequence, waveform, volume, unison, detuneHz, attackMs, decayMs, repeatMs, preempt))
}

// PlayMp3 plays an MP3 file stored on the firmware (16 kHz mono).
func (s *SoundService) PlayMp3(path string, preempt bool) error {
	if path == "" {
		return fmt.Errorf("empty MP3 path")
	}
	if len(path) > maxPathLen {
		return fmt.Errorf("path too long (%d chars, max %d)", len(path), maxPathLen)
	}
	return s.c.SendRPC(soundFunctionPlayMp3, []Param{
		{ID: 0, Data: []byte(path)},
		{ID: 1, Data: boolByte(preempt)},
	})
}

// Stop stops playback and flushes the player queues.
func (s *SoundService) Stop() error {
	return s.c.SendRPC(soundFunctionStop, nil)
}

// ---------------------------------------------------------------------------
// Parameter packing: the wire layout mirrors the "functions" entry of
// services/sound_service.json — one descriptor per parameter, same order.
// ---------------------------------------------------------------------------

// boolByte encodes a flag as the uint8_t the schema uses for the "Preempt"
// parameters.
func boolByte(v bool) []byte {
	if v {
		return []byte{1}
	}
	return []byte{0}
}

// packPlayTone builds the PlayTone parameters: Freq (u16), DurationMs (u16),
// Volume (u8), Preempt (u8).
func packPlayTone(freq, durationMs uint16, volume uint8, preempt bool) []Param {
	f := make([]byte, 2)
	binary.LittleEndian.PutUint16(f, freq)
	d := make([]byte, 2)
	binary.LittleEndian.PutUint16(d, durationMs)
	return []Param{
		{ID: 0, Data: f},
		{ID: 1, Data: d},
		{ID: 2, Data: []byte{volume}},
		{ID: 3, Data: boolByte(preempt)},
	}
}

// packPlaySequence builds the PlaySequence parameters (sequence, waveform, volume,
// unison, detune in Hz, attack/decay envelope in ms, preempt, repeat in ms).
func packPlaySequence(sequence string, waveform, volume, unison uint8, detuneHz uint16, attackMs, decayMs uint8, repeatMs uint16, preempt bool) []Param {
	detune := make([]byte, 2)
	binary.LittleEndian.PutUint16(detune, detuneHz)
	repeat := make([]byte, 2)
	binary.LittleEndian.PutUint16(repeat, repeatMs)
	return []Param{
		{ID: 0, Data: []byte(sequence)},
		{ID: 1, Data: []byte{waveform}},
		{ID: 2, Data: []byte{volume}},
		{ID: 3, Data: []byte{unison}},
		{ID: 4, Data: detune},
		{ID: 5, Data: []byte{attackMs}},
		{ID: 6, Data: []byte{decayMs}},
		{ID: 7, Data: boolByte(preempt)},
		{ID: 8, Data: repeat},
	}
}
