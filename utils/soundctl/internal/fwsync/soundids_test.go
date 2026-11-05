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
		"area_recording_mode", "autonomous_notice", "boot_complete", "boot_ping", "charging_done",
		"charging_start", "collision_triggered", "docking_start", "emergency", "emergency_cleared",
		"error", "gps_rtk_fix", "gps_rtk_lost", "lift_triggered", "low_battery", "mower_rpm_limit",
		"mower_rpm_timeout", "mowing_ambience", "mowing_complete", "mowing_paused", "mowing_resumed",
		"mowing_start", "rain_detected", "recovered", "ros_connected", "ros_disconnected",
		"shutdown_safe", "stop_button", "success", "volume_down", "volume_up", "warning",
	}
	if got := ids.Names(); len(got) != len(want) {
		t.Fatalf("names = %v, want %v", got, want)
	}
	for _, n := range want {
		if !ids.Has(n) {
			t.Fatalf("missing sound id %q", n)
		}
	}
	// The ids are a wire format (and the key of the persisted overrides):
	// appending is fine, renumbering is not - pin the last one.
	if v, ok := ids.Value("mower_rpm_timeout"); !ok || v != 29 {
		t.Fatalf("mower_rpm_timeout = %d (found=%v), want 29", v, ok)
	}
	if v, ok := ids.Value("mower_rpm_limit"); !ok || v != 30 {
		t.Fatalf("mower_rpm_limit = %d (found=%v), want 30", v, ok)
	}
	if v, ok := ids.Value("recovered"); !ok || v != 31 {
		t.Fatalf("recovered = %d (found=%v), want 31 (appended, never renumbered)", v, ok)
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
