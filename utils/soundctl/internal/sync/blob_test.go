package sync

import (
	"encoding/json"
	"testing"
)

func TestConfigBlob(t *testing.T) {
	cfg := &Config{
		Version:   1,
		SoundPath: "/opt/open_mower_ros/sounds",
		Sounds: map[string]SoundDef{
			"boot_complete": {Type: "mp3", Volume: 100, File: "en_hi.mp3"},
			"warning": {Type: "sequence", Volume: 80, Waveform: "saw",
				Sequence: []Note{{Freq: 880, DurationMs: 150}, {Freq: 0, DurationMs: 80}}},
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
	}
	if err := json.Unmarshal(doc["sounds"], &sounds); err != nil {
		t.Fatal(err)
	}
	if sounds.BootComplete.Type != "mp3" || sounds.BootComplete.File != "en_hi.mp3" || sounds.BootComplete.Volume != 100 {
		t.Fatalf("boot_complete = %+v", sounds.BootComplete)
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
}
