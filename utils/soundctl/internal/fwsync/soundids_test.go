package fwsync

import (
	"os"
	"path/filepath"
	"testing"
)

func TestLoadSoundIDsFromSharedDefinition(t *testing.T) {
	// Locate the shared definition relative to this package directory
	// (utils/soundctl/internal/fwsync -> repo root -> services/).
	path := filepath.Join("..", "..", "..", "..", "services", "sound_service.json")
	if _, err := os.Stat(path); err != nil {
		t.Fatalf("shared sound_service.json not found at %s: %v", path, err)
	}

	ids, err := LoadSoundIDs(path)
	if err != nil {
		t.Fatal(err)
	}

	want := []string{
		"boot_ping", "boot_complete", "success", "warning", "error",
		"emergency", "low_battery", "charging_start", "charging_done",
		"ros_connected", "ros_disconnected", "gps_rtk_fix", "gps_rtk_lost",
	}
	if got := ids.Names(); len(got) != len(want) {
		t.Fatalf("names = %v, want %v", got, want)
	}
	for _, n := range want {
		if !ids.Has(n) {
			t.Fatalf("missing sound id %q", n)
		}
	}
	if ids.Has("count") || ids.Has("nonsense") {
		t.Fatalf("unexpected sound id present")
	}
}

// TestSoundIDsLookups pins the case-insensitive lookups of all three enums.
func TestSoundIDsLookups(t *testing.T) {
	ids := sharedIDs(t)

	if v, ok := ids.Value("BOOT_PING"); !ok || v != 0 {
		t.Errorf("Value(BOOT_PING) = %d/%v, want 0/true", v, ok)
	}
	if v, ok := ids.SoundType("Sequence"); !ok || v != 1 {
		t.Errorf("SoundType(Sequence) = %d/%v, want 1/true", v, ok)
	}
	if v, ok := ids.Waveform("saw"); !ok || v != 3 {
		t.Errorf("Waveform(saw) = %d/%v, want 3/true", v, ok)
	}
	if _, ok := ids.Waveform("noise"); ok {
		t.Error("Waveform(noise) must not resolve")
	}
	if _, ok := ids.SoundType("beep"); ok {
		t.Error("SoundType(beep) must not resolve")
	}
}
