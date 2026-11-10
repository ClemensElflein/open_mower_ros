package settings

import (
	"os"
	"path/filepath"
	"testing"
)

// TestDataSoundsDir pins where the writable sound overlay lives: /data/sounds on a
// mower whose data partition is mounted, "" (image only) otherwise. The overlay
// itself need not exist yet - the first `soundctl tts` render creates it.
func TestDataSoundsDir(t *testing.T) {
	isData := func(p string) bool { return p == "/data" }
	if got := DataSoundsDir("/data", isData); got != "/data/sounds" {
		t.Errorf("DataSoundsDir(/data) = %q, want /data/sounds", got)
	}

	nothing := func(string) bool { return false }
	if got := DataSoundsDir("/data", nothing); got != "" {
		t.Errorf("DataSoundsDir without the data partition = %q, want \"\"", got)
	}
}

// TestAPIKeyResolutionOrder pins the documented order: explicit flag, explicit
// file, then the engine's environment variable, then its default file.
func TestAPIKeyResolutionOrder(t *testing.T) {
	flagFile := writeKey(t, "from-flag-file")
	envKey := "from-environment"

	t.Run("explicit flag wins", func(t *testing.T) {
		t.Setenv("ELEVENLABS_API_KEY", envKey)
		got, err := APIKey("elevenlabs", "from-flag", flagFile)
		if err != nil || got != "from-flag" {
			t.Fatalf("APIKey = %q/%v, want from-flag", got, err)
		}
	})

	t.Run("explicit file beats the environment", func(t *testing.T) {
		t.Setenv("ELEVENLABS_API_KEY", envKey)
		got, err := APIKey("elevenlabs", "", flagFile)
		if err != nil || got != "from-flag-file" {
			t.Fatalf("APIKey = %q/%v, want the file value", got, err)
		}
	})

	t.Run("environment beats the default file", func(t *testing.T) {
		t.Setenv("ELEVENLABS_API_KEY", envKey)
		got, err := APIKey("elevenlabs", "", "")
		if err != nil || got != envKey {
			t.Fatalf("APIKey = %q/%v, want %q", got, err, envKey)
		}
	})

	t.Run("missing key names the way out", func(t *testing.T) {
		t.Setenv("ELEVENLABS_API_KEY", "")
		// The default file lives under /data, which does not exist in a test.
		if _, err := APIKey("elevenlabs", "", ""); err == nil {
			t.Fatal("an empty environment and no default file must fail")
		}
	})

	t.Run("cartesia reads its own variable", func(t *testing.T) {
		t.Setenv("CARTESIA_API_KEY", "car-key")
		got, err := APIKey("cartesia", "", "")
		if err != nil || got != "car-key" {
			t.Fatalf("APIKey = %q/%v, want car-key", got, err)
		}
	})
}

// TestAPIKeySourceNamesTheDefaults keeps the two documented locations honest: the
// README and the flag help both print them.
func TestAPIKeySourceNamesTheDefaults(t *testing.T) {
	cases := []struct{ engine, env, file string }{
		{"elevenlabs", "ELEVENLABS_API_KEY", "/data/sounds/.elevenlabs.key"},
		{"cartesia", "CARTESIA_API_KEY", "/data/sounds/.cartesia.key"},
	}
	for _, c := range cases {
		env, file := APIKeySource(c.engine)
		if env != c.env || file != c.file {
			t.Errorf("APIKeySource(%s) = %q/%q, want %q/%q", c.engine, env, file, c.env, c.file)
		}
	}
}

// TestReadKeyFileTrims: an editor or `echo` appends a newline, which must not
// become part of the header value.
func TestReadKeyFileTrims(t *testing.T) {
	path := writeKey(t, "  sk-abc  \n")
	got, err := ReadKeyFile(path)
	if err != nil || got != "sk-abc" {
		t.Fatalf("ReadKeyFile = %q/%v, want sk-abc", got, err)
	}
	if _, err := ReadKeyFile(path + ".missing"); err == nil {
		t.Fatal("a missing key file must be an error")
	}
}

// TestDefaultKeyFileDistinguishesMissingFromUnreadable: a key file that exists but
// cannot be read must surface its own error. Reporting it as "no API key" would tell
// the user to write a file that is already there - the case that actually bites in
// the simulation, where a mode-0600 file can belong to another uid.
func TestDefaultKeyFileDistinguishesMissingFromUnreadable(t *testing.T) {
	if _, ok, err := defaultKeyFromFile(filepath.Join(t.TempDir(), "absent.key")); err != nil || ok {
		t.Fatalf("a missing file = ok:%v err:%v, want ok:false err:nil", ok, err)
	}
	// A directory exists but os.ReadFile fails on it - a portable stand-in for
	// "present, but unreadable" (chmod 000 would not work when the test runs as root).
	if _, ok, err := defaultKeyFromFile(t.TempDir()); err == nil || ok {
		t.Fatalf("an unreadable path = ok:%v err:%v, want a real error", ok, err)
	}
}

// writeKey writes a temporary key file.
func writeKey(t *testing.T, content string) string {
	t.Helper()
	path := t.TempDir() + "/key"
	if err := os.WriteFile(path, []byte(content), 0o600); err != nil {
		t.Fatal(err)
	}
	return path
}
