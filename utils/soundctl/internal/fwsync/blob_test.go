package fwsync

import (
	"encoding/json"
	"testing"
)

func TestConfigBlob(t *testing.T) {
	cfg := &Config{
		Version:   1,
		SoundPath: "/opt/open_mower_ros/sounds",
		Sounds: map[string]SoundDef{
			"boot_complete": {Type: "mp3", Volume: 100, File: "en_hi.mp3", PauseAfterMs: 600},
			"warning": {Type: "sequence", Volume: 80, Waveform: "saw",
				Sequence: []Note{{Freq: 880, DurationMs: 150}, {Freq: 0, DurationMs: 80}}},
			"emergency": {Type: "sequence", Volume: 90, Preempt: true,
				Sequence: []Note{{Freq: 950, DurationMs: 8000, LfoHzX10: 20, LfoDepth: 220}}},
		},
	}
	blob, err := cfg.Blob()
	if err != nil {
		t.Fatal(err)
	}

	var doc map[string]json.RawMessage
	if err := json.Unmarshal(blob, &doc); err != nil {
		t.Fatal(err)
	}
	if _, ok := doc["sounds"]; !ok {
		t.Fatalf("blob missing 'sounds': %s", blob)
	}
	// The HL-only fields must be stripped before sending.
	if _, ok := doc["version"]; ok {
		t.Fatalf("blob must not contain 'version': %s", blob)
	}
	if _, ok := doc["sound_path"]; ok {
		t.Fatalf("blob must not contain 'sound_path': %s", blob)
	}

	// Field names must match what the firmware parser (sound_service.cpp) reads.
	var sounds struct {
		BootComplete SoundDef `json:"boot_complete"`
		Warning      SoundDef `json:"warning"`
		Emergency    SoundDef `json:"emergency"`
	}
	if err := json.Unmarshal(doc["sounds"], &sounds); err != nil {
		t.Fatal(err)
	}
	if sounds.BootComplete.Type != "mp3" || sounds.BootComplete.File != "en_hi.mp3" || sounds.BootComplete.Volume != 100 {
		t.Fatalf("boot_complete = %+v", sounds.BootComplete)
	}
	// The key has to reach the firmware parser (sound_service.cpp) spelled exactly like this.
	if sounds.BootComplete.PauseAfterMs != 600 {
		t.Fatalf("boot_complete.pause_after_ms = %d, want 600", sounds.BootComplete.PauseAfterMs)
	}
	if sounds.Warning.Type != "sequence" || sounds.Warning.Waveform != "saw" || len(sounds.Warning.Sequence) != 2 {
		t.Fatalf("warning = %+v", sounds.Warning)
	}
	if sounds.Warning.Sequence[0].Freq != 880 || sounds.Warning.Sequence[0].DurationMs != 150 {
		t.Fatalf("warning.sequence[0] = %+v", sounds.Warning.Sequence[0])
	}
	if sounds.Warning.Tone != nil {
		t.Fatalf("warning.tone should be omitted, got %+v", sounds.Warning.Tone)
	}
	if sounds.Emergency.Preempt || sounds.Warning.Preempt || sounds.BootComplete.Preempt {
		t.Fatalf("preempt must not reach the firmware: %+v", sounds.Emergency)
	}
}

// TestBlobCarriesTheDefaultPick pins the blob contract for take selection: the firmware
// zeroes its persisted index ("first"), so a sound without an explicit `pick` has to be
// sent as random — otherwise every sound with several takes would always play take 1.
func TestBlobCarriesTheDefaultPick(t *testing.T) {
	cfg := &Config{
		Version:   1,
		SoundPath: "/opt/open_mower_ros/sounds",
		Sounds: map[string]SoundDef{
			"boot_complete": {Type: "mp3", Volume: 90, Variants: []Variant{{TTS: "one"}, {TTS: "two"}}},
			"ros_connected": {Type: "mp3", Volume: 90, Pick: PickRotate,
				Variants: []Variant{{TTS: "a"}, {TTS: "b"}}},
		},
	}
	raw, err := cfg.Blob()
	if err != nil {
		t.Fatalf("Blob: %v", err)
	}
	var blob struct {
		Sounds map[string]SoundDef `json:"sounds"`
	}
	if err := json.Unmarshal(raw, &blob); err != nil {
		t.Fatalf("blob is not JSON: %v", err)
	}
	if got := blob.Sounds["boot_complete"].Pick; got != PickRandom {
		t.Errorf("boot_complete pick = %q, want the default %q", got, PickRandom)
	}
	if got := blob.Sounds["ros_connected"].Pick; got != PickRotate {
		t.Errorf("ros_connected pick = %q, want the configured %q", got, PickRotate)
	}
}
