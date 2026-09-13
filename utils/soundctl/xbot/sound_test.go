package xbot

import (
	"encoding/binary"
	"testing"
)

// TestPackPlayTone pins the wire layout of the PlayTone parameters: one
// descriptor per schema parameter (Freq u16, Duration u16, Volume u8, Preempt u8).
func TestPackPlayTone(t *testing.T) {
	params := packPlayTone(400, 60, 45, true)
	if len(params) != 4 {
		t.Fatalf("params = %d, want 4", len(params))
	}
	for i, want := range []uint16{0, 1, 2, 3} {
		if params[i].ID != want {
			t.Errorf("param %d id = %d, want %d", i, params[i].ID, want)
		}
	}
	if len(params[0].Data) != 2 || binary.LittleEndian.Uint16(params[0].Data) != 400 {
		t.Errorf("freq = %v, want 400 (u16)", params[0].Data)
	}
	if len(params[1].Data) != 2 || binary.LittleEndian.Uint16(params[1].Data) != 60 {
		t.Errorf("duration = %v, want 60 (u16)", params[1].Data)
	}
	if len(params[2].Data) != 1 || params[2].Data[0] != 45 {
		t.Errorf("volume = %v, want 45 (u8)", params[2].Data)
	}
	if len(params[3].Data) != 1 || params[3].Data[0] != 1 {
		t.Errorf("preempt = %v, want 1 (u8)", params[3].Data)
	}
}

// TestPackPlaySequence pins the parameter order of PlaySequence: it must match
// the "functions" entry in services/sound_service.json
// (Sequence, Wave, Volume, Unison, DetuneHz, AttackMs, DecayMs, Preempt).
func TestPackPlaySequence(t *testing.T) {
	params := packPlaySequence("250:60 0:40 375:80", 3, 45, 1, 20, 5, 200, true)
	if len(params) != 8 {
		t.Fatalf("params = %d, want 8", len(params))
	}
	for i, want := range []uint16{0, 1, 2, 3, 4, 5, 6, 7} {
		if params[i].ID != want {
			t.Errorf("param %d id = %d, want %d", i, params[i].ID, want)
		}
	}
	if got := string(params[0].Data); got != "250:60 0:40 375:80" {
		t.Errorf("sequence = %q", got)
	}
	if params[1].Data[0] != 3 {
		t.Errorf("waveform = %d, want 3 (saw)", params[1].Data[0])
	}
	if params[2].Data[0] != 45 {
		t.Errorf("volume = %d, want 45", params[2].Data[0])
	}
	if params[3].Data[0] != 1 {
		t.Errorf("unison = %d, want 1", params[3].Data[0])
	}
	if got := binary.LittleEndian.Uint16(params[4].Data); got != 20 {
		t.Errorf("detune = %d, want 20", got)
	}
	if len(params[5].Data) != 1 || params[5].Data[0] != 5 {
		t.Errorf("attack = %v, want 5 (u8)", params[5].Data)
	}
	if len(params[6].Data) != 1 || params[6].Data[0] != 200 {
		t.Errorf("decay = %v, want 200 (u8)", params[6].Data)
	}
	if params[7].Data[0] != 1 {
		t.Errorf("preempt = %d, want 1", params[7].Data[0])
	}
}
