// Package soundservice is a client for the OpenMower SoundService (service id 13,
// defined in services/sound_service.json): the sound definitions register, the
// volume input and the playback RPCs.
package soundservice

import (
	"context"
	"fmt"

	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

// ServiceID is the SoundService id (services/service_ids.h).
const ServiceID uint16 = 13

// Register and input ids (services/sound_service.json).
const (
	registerDefinitions uint16 = 0
	inputVolume         uint16 = 0
)

// RPC function ids (services/sound_service.json -> "functions").
const (
	fnPlaySound    uint8 = 0
	fnPlayTone     uint8 = 1
	fnPlaySequence uint8 = 2
	fnPlayMp3      uint8 = 3
	fnStop         uint8 = 4
)

// Max lengths of the string RPC parameters (services/sound_service.json).
const (
	maxSequenceLen = 160
	maxPathLen     = 128
)

// Client speaks the SoundService registers, inputs and RPCs.
type Client struct {
	conn *xbot.Conn
}

// DialNoClaim discovers and connects to the SoundService without claiming it.
//
// This is the normal mode on the robot: the high-level system owns (claims) the
// service, and the definitions, the volume and the playback RPCs are one-way
// messages. Claiming would stop a running service and re-target its responses.
func DialNoClaim(ctx context.Context, bindIP string) (*Client, error) {
	conn, err := xbot.Dial(ctx, xbot.Options{ServiceID: ServiceID, BindIP: bindIP, NoClaim: true})
	if err != nil {
		return nil, err
	}
	return &Client{conn: conn}, nil
}

// NewAt connects to a known endpoint ("ip:port"): no discovery, no claim. Use it
// to skip the advertisement-driven discovery on repeat calls (a claimed service
// only advertises every 10 s) — see `soundctl play --addr`.
func NewAt(endpoint string) (*Client, error) {
	conn, err := xbot.Dial(context.Background(), xbot.Options{ServiceID: ServiceID, Endpoint: endpoint})
	if err != nil {
		return nil, err
	}
	return &Client{conn: conn}, nil
}

// Close closes the underlying connection.
func (c *Client) Close() error { return c.conn.Close() }

// Endpoint returns the unicast address of the SoundService as "ip:port".
func (c *Client) Endpoint() string { return c.conn.Endpoint() }

// SetDefinitions sends the sound-definitions blob (already heatshrink encoded)
// as the "Sound Definitions" register. The firmware treats a register update as
// a reconfiguration (it restarts the service) and persists it to flash.
func (c *Client) SetDefinitions(blob []byte) error {
	return c.conn.SendTransaction(true, xbot.Data{ID: registerDefinitions, Data: blob})
}

// SetVolume sends the master volume (0..100) as the "Volume" input. The service
// must be running (i.e. configured) for the input to be accepted.
func (c *Client) SetVolume(volume uint8) error {
	return c.conn.SendData(inputVolume, []byte{volume})
}

// PlaySound plays a configured sound immediately. Playback is fire-and-forget:
// the firmware sends the result to the service owner (the high-level system), so
// success is audible rather than returned here.
func (c *Client) PlaySound(id uint8) error {
	return c.conn.SendRPC(fnPlaySound, xbot.Uint8(0, id))
}

// PlayTone plays one synthesised tone. preempt stops a running sound, drops the
// queue and plays immediately.
func (c *Client) PlayTone(freq, durationMs uint16, volume uint8, preempt bool) error {
	return c.conn.SendRPC(fnPlayTone, toneParams(freq, durationMs, volume, preempt)...)
}

// PlaySequence plays a compact note sequence, e.g. "250:60 0:40 375:80".
// attackMs/decayMs are the per-note envelope (0 = instant onset / hold the note),
// repeatMs repeats the sequence every n ms until another sound plays (0 = once);
// see the firmware synth and player.
func (c *Client) PlaySequence(sequence string, waveform, volume, unison uint8, detuneHz uint16,
	attackMs, decayMs uint8, repeatMs uint16, preempt bool) error {
	if len(sequence) > maxSequenceLen {
		return fmt.Errorf("sequence too long (%d chars, max %d)", len(sequence), maxSequenceLen)
	}
	return c.conn.SendRPC(fnPlaySequence, sequenceParams(sequence, waveform, volume, unison,
		detuneHz, attackMs, decayMs, repeatMs, preempt)...)
}

// PlayMp3 plays an MP3 file stored on the firmware (16 kHz mono).
func (c *Client) PlayMp3(path string, preempt bool) error {
	if path == "" {
		return fmt.Errorf("empty MP3 path")
	}
	if len(path) > maxPathLen {
		return fmt.Errorf("path too long (%d chars, max %d)", len(path), maxPathLen)
	}
	return c.conn.SendRPC(fnPlayMp3, xbot.String(0, path), xbot.Bool(1, preempt))
}

// Stop stops playback and flushes the player queues.
func (c *Client) Stop() error { return c.conn.SendRPC(fnStop) }

// toneParams and sequenceParams build the RPC parameters in the order of the
// "functions" entries in services/sound_service.json. They are separate functions
// so the wire layout is testable without a connection.
func toneParams(freq, durationMs uint16, volume uint8, preempt bool) []xbot.Param {
	return []xbot.Param{
		xbot.Uint16(0, freq),
		xbot.Uint16(1, durationMs),
		xbot.Uint8(2, volume),
		xbot.Bool(3, preempt),
	}
}

func sequenceParams(sequence string, waveform, volume, unison uint8, detuneHz uint16,
	attackMs, decayMs uint8, repeatMs uint16, preempt bool) []xbot.Param {
	return []xbot.Param{
		xbot.String(0, sequence),
		xbot.Uint8(1, waveform),
		xbot.Uint8(2, volume),
		xbot.Uint8(3, unison),
		xbot.Uint16(4, detuneHz),
		xbot.Uint8(5, attackMs),
		xbot.Uint8(6, decayMs),
		xbot.Bool(7, preempt),
		xbot.Uint16(8, repeatMs),
	}
}
