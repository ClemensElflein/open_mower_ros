package fwsync

import (
	"path/filepath"
	"testing"
)

// TestShippedFingerprints guards the render cache key of the announcements: every
// sidecar in /data/sounds/<lang>/ stores this fingerprint. Changing a tts text,
// the voice, one of the TTS defaults or the shape of TTSIdentity.Key() changes it
// - which re-renders all announcements on the next start and makes renders from
// different images incomparable. Bumping a golden value is fine, it just has to
// be a conscious decision (it costs one API request per announcement).
func TestShippedFingerprints(t *testing.T) {
	want := map[string]string{
		"sounds_en.yaml:boot_complete[1]":    "d0178b26",
		"sounds_en.yaml:boot_complete[2]":    "ec524009",
		"sounds_en.yaml:boot_complete[3]":    "2dca10ba",
		"sounds_en.yaml:boot_complete[4]":    "59787e7b",
		"sounds_en.yaml:boot_complete[5]":    "cf724cb0",
		"sounds_en.yaml:ros_connected[1]":    "ea7ce54c",
		"sounds_en.yaml:ros_connected[2]":    "04a77562",
		"sounds_en.yaml:ros_connected[3]":    "176fd451",
		"sounds_en.yaml:ros_disconnected[1]": "25d4400b",
		"sounds_en.yaml:ros_disconnected[2]": "c4faf371",
		"sounds_en.yaml:ros_disconnected[3]": "6a19cf34",
		"sounds_de.yaml:boot_complete[1]":    "f869ac83",
		"sounds_de.yaml:boot_complete[2]":    "3d6a9709",
		"sounds_de.yaml:boot_complete[3]":    "30216a69",
		"sounds_de.yaml:boot_complete[4]":    "e3860550",
		"sounds_de.yaml:boot_complete[5]":    "86cef078",
		"sounds_de.yaml:ros_connected[1]":    "bdf24209",
		"sounds_de.yaml:ros_connected[2]":    "b142132b",
		"sounds_de.yaml:ros_connected[3]":    "594e1136",
		"sounds_de.yaml:ros_disconnected[1]": "c5961251",
		"sounds_de.yaml:ros_disconnected[2]": "73e4fd85",
		"sounds_de.yaml:ros_disconnected[3]": "17d05ae4",
	}
	seen := 0

	for _, file := range []string{"sounds_en.yaml", "sounds_de.yaml"} {
		cfg, err := LoadConfig(filepath.Join("..", "..", "..", "..", "src", "open_mower", "params", file))
		if err != nil {
			t.Fatal(err)
		}
		for _, job := range cfg.RenderJobs() {
			key := file + ":" + job.Label
			fp, ok := want[key]
			if !ok {
				t.Errorf("no golden fingerprint for %s (%s): %q", key, job.File, job.Identity.Fingerprint())
				continue
			}
			seen++
			if got := job.Identity.Fingerprint(); got != fp {
				t.Errorf("%s: fingerprint %s, want %s - this re-renders the announcement on "+
					"the robots; update the golden value only deliberately", key, got, fp)
			}
		}
	}
	if seen != len(want) {
		t.Errorf("%d of %d golden fingerprints matched a render job", seen, len(want))
	}
}
