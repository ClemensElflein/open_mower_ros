package sync

import (
	"os"
	"path/filepath"
	"testing"
)

func TestLoadSoundIDsFromSharedDefinition(t *testing.T) {
	// Locate the shared definition relative to this package directory
	// (utils/soundctl/internal/sync -> repo root -> services/).
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
		"gps_rtk_fix", "gps_rtk_lost",
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
