package fwsync

import (
	"path/filepath"
	"testing"
)

// sharedIDs loads the definition shipped with the firmware/high-level repos
// (utils/soundctl/internal/fwsync -> repo root -> services/sound_service.json).
func sharedIDs(t *testing.T) *SoundIDs {
	t.Helper()
	ids, err := LoadSoundIDs(filepath.Join("..", "..", "..", "..", "services", "sound_service.json"))
	if err != nil {
		t.Fatal(err)
	}
	return ids
}

// TestRuntimeSpecsUseYAMLRules pins that the values of `soundctl play` go through
// the same rules as sounds_*.yaml — in particular that out-of-range values are
// rejected instead of silently wrapping in the uint casts.
func TestRuntimeSpecsUseYAMLRules(t *testing.T) {
	ids := sharedIDs(t)

	valid := []struct {
		name string
		def  SoundDef
	}{
		{"tone", (&ToneSpec{Freq: 400, DurationMs: 60, Volume: 80}).toSoundDef()},
		{"tone at the uint16 limit", (&ToneSpec{Freq: 65535, DurationMs: 65535, Volume: 0}).toSoundDef()},
		{"sequence", (&SequenceSpec{Notes: "250:60 0:40", Waveform: "sine", Volume: 45}).toSoundDef()},
		{"sequence upper case waveform", (&SequenceSpec{Notes: "250:60", Waveform: "Saw", Volume: 100, Unison: 7, DetuneHz: 40}).toSoundDef()},
		{"sequence with an envelope", (&SequenceSpec{Notes: "554:360", Waveform: "saw", Volume: 70, AttackMs: 4, DecayMs: 200}).toSoundDef()},
		{"sequence envelope at the limit", (&SequenceSpec{Notes: "554:360", Waveform: "saw", Volume: 70, AttackMs: 255, DecayMs: 255}).toSoundDef()},
	}
	for _, c := range valid {
		if err := validateOne(c.name, c.def, ids); err != nil {
			t.Errorf("%s: valid definition rejected: %v", c.name, err)
		}
	}

	invalid := []struct {
		name string
		def  SoundDef
	}{
		{"tone volume", (&ToneSpec{Freq: 400, DurationMs: 60, Volume: 500}).toSoundDef()},
		{"tone freq", (&ToneSpec{Freq: 70000, DurationMs: 60, Volume: 80}).toSoundDef()},
		{"sequence volume", (&SequenceSpec{Notes: "250:60", Waveform: "sine", Volume: -1}).toSoundDef()},
		{"sequence waveform", (&SequenceSpec{Notes: "250:60", Waveform: "noise", Volume: 45}).toSoundDef()},
		{"sequence unison", (&SequenceSpec{Notes: "250:60", Waveform: "sine", Volume: 45, Unison: 2}).toSoundDef()},
		{"sequence detune", (&SequenceSpec{Notes: "250:60", Waveform: "sine", Volume: 45, DetuneHz: -5}).toSoundDef()},
		{"sequence attack", (&SequenceSpec{Notes: "554:360", Waveform: "saw", Volume: 70, AttackMs: 256}).toSoundDef()},
		{"sequence decay", (&SequenceSpec{Notes: "554:360", Waveform: "saw", Volume: 70, DecayMs: -1}).toSoundDef()},
	}
	for _, c := range invalid {
		if err := validateOne(c.name, c.def, ids); err == nil {
			t.Errorf("%s: invalid definition accepted", c.name)
		}
	}
}
