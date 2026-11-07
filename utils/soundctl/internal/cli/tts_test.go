package cli

import "testing"

// TestDataSoundsDir pins where the writable sound overlay lives: /data/sounds on
// a mower whose data partition is mounted, "" (image only) otherwise. The
// overlay itself need not exist yet — the first `soundctl tts` render creates it.
func TestDataSoundsDir(t *testing.T) {
	isData := func(p string) bool { return p == "/data" }
	if got := dataSoundsDir("/data", isData); got != "/data/sounds" {
		t.Errorf("dataSoundsDir(/data) = %q, want /data/sounds", got)
	}

	nothing := func(string) bool { return false }
	if got := dataSoundsDir("/data", nothing); got != "" {
		t.Errorf("dataSoundsDir without the data partition = %q, want \"\"", got)
	}
}
