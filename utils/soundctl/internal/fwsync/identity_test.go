package fwsync

import (
	"path/filepath"
	"testing"
)

// TestShippedFingerprints guards the render cache key of the announcements that
// are already on the robots: every sidecar in /data/sounds/<lang>/ stores this
// fingerprint. Changing a tts text, one of the TTS defaults or the shape of
// TTSIdentity.Key() changes it — which re-renders all announcements on the next
// start and makes renders from different images incomparable. Bumping the
// golden values is fine, it just has to be a conscious decision.
//
// The values are the ones the reference renders produced.
func TestShippedFingerprints(t *testing.T) {
	want := map[string]map[string]string{
		"sounds_en.yaml": {
			"boot_complete":    "fc6eeec4",
			"ros_connected":    "2a471827",
			"ros_disconnected": "e5efbd60",
		},
		"sounds_de.yaml": {
			"boot_complete":    "68761a43",
			"ros_connected":    "c93eaf47",
			"ros_disconnected": "5486b22b",
		},
	}
	for file, expected := range want {
		cfg, err := LoadConfig(filepath.Join("..", "..", "..", "..", "src", "open_mower", "params", file))
		if err != nil {
			t.Fatal(err)
		}
		for name, fp := range expected {
			id, ok := cfg.Identity(name)
			if !ok {
				t.Fatalf("%s: %s has no render identity (was its tts: text removed?)", file, name)
			}
			if got := id.Fingerprint(); got != fp {
				t.Errorf("%s: %s: fingerprint %s, want %s — this re-renders the announcement on "+
					"the robots; update the golden value only deliberately", file, name, got, fp)
			}
		}
	}
}
