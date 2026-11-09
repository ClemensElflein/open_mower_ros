// Package settings holds the deployment policy behind the soundctl commands: where
// the API keys and the writable sound overlay live, and how a resolved tts block
// becomes an engine.
//
// It is separate from internal/cli on purpose. The CLI only turns flags into these
// values (internal/cli stays "flags only"), so everything here is testable without
// a cobra command and without a robot.
package settings

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

const (
	// DataDir is the writable data partition of the mower (a mount in the OS image,
	// ./data in docker-simulation).
	DataDir = "/data"
	// SoundsDirName is the sound overlay below DataDir.
	SoundsDirName = "sounds"

	// DefaultElevenLabsKeyFile is where the mower keeps the ElevenLabs key. The
	// sounds overlay is the one directory every user who customises sounds mounts
	// anyway, and it lives on the data partition, so the key needs no extra volume
	// and survives image updates. It is never uploaded (sync only sends the files
	// named in sounds_*.yaml) and never deleted (cleanup compares the low level's
	// listing against those names) - hence the dot-file name.
	DefaultElevenLabsKeyFile = "/data/sounds/.elevenlabs.key"
	// DefaultCartesiaKeyFile is the same trick for the Cartesia key.
	DefaultCartesiaKeyFile = "/data/sounds/.cartesia.key"
)

// SoundsDir returns the writable overlay: $OPENMOWER_SOUNDS_DIR, else /data/sounds
// when the data partition exists, else "" (image only).
func SoundsDir() string {
	if d := os.Getenv("OPENMOWER_SOUNDS_DIR"); d != "" {
		return d
	}
	return DataSoundsDir(DataDir, isDirectory)
}

// DataSoundsDir is the /data part of SoundsDir. Only the data partition has to
// exist: the overlay directory itself is created by the first render
// (`soundctl tts`), so a mower with a fresh data partition still gets its
// announcements instead of writing into the read-only image.
func DataSoundsDir(data string, isDir func(string) bool) string {
	if !isDir(data) {
		return ""
	}
	return filepath.Join(data, SoundsDirName)
}

func isDirectory(p string) bool {
	st, err := os.Stat(p)
	return err == nil && st.IsDir()
}

// APIKey resolves the key of an engine: explicit flag, explicit file, then the
// engine's environment variable, then its default file. The environment is what CI
// (a repository secret) and a dev shell use; the file is what the mower uses.
func APIKey(engine, explicit, explicitFile string) (string, error) {
	if v := strings.TrimSpace(explicit); v != "" {
		return v, nil
	}
	if explicitFile != "" {
		return ReadKeyFile(explicitFile)
	}
	env, file := APIKeySource(engine)
	if v := strings.TrimSpace(os.Getenv(env)); v != "" {
		return v, nil
	}
	if v, err := ReadKeyFile(file); err == nil && v != "" {
		return v, nil
	}
	return "", fmt.Errorf("no %s API key: pass --api-key/--api-key-file, set $%s, or write %s", engine, env, file)
}

// APIKeySource names the environment variable and the default file of an engine.
func APIKeySource(engine string) (env, file string) {
	if engine == fwsync.TTSEngineCartesia {
		return "CARTESIA_API_KEY", DefaultCartesiaKeyFile
	}
	return "ELEVENLABS_API_KEY", DefaultElevenLabsKeyFile
}

// ReadKeyFile reads a key file. Editors append a newline, so the value is trimmed;
// anything else (a missing file included) is an error.
func ReadKeyFile(path string) (string, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return "", fmt.Errorf("read API key file: %w", err)
	}
	key := strings.TrimSpace(string(raw))
	if key == "" {
		return "", fmt.Errorf("API key file %s is empty", path)
	}
	return key, nil
}

// NewEngine builds the engine a resolved tts block asks for. The same factory
// serves `soundctl tts` (engine from the YAML) and `soundctl say` (engine from
// flags), so both always use the same engine with the same key resolution.
func NewEngine(set fwsync.TTSSettings, apiKey, apiKeyFile, apiURL string) (tts.Engine, error) {
	key, err := APIKey(set.Engine, apiKey, apiKeyFile)
	if err != nil {
		return nil, err
	}
	switch set.Engine {
	case fwsync.TTSEngineCartesia:
		return tts.Cartesia{APIKey: key, BaseURL: apiURL}, nil
	case fwsync.TTSEngineElevenLabs:
		return tts.ElevenLabs{APIKey: key, BaseURL: apiURL}, nil
	default:
		return nil, fmt.Errorf("tts.engine %q is not supported (use %q or %q)", set.Engine,
			fwsync.TTSEngineElevenLabs, fwsync.TTSEngineCartesia)
	}
}
