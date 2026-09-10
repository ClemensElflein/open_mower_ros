// Package sync parses sounds_*.yaml and synchronises the MP3 files with the
// firmware's FileService.
package sync

import (
	"encoding/json"
	"fmt"
	"os"
	"sort"

	"gopkg.in/yaml.v3"
)

// Note is a single note in a sequence definition.
type Note struct {
	Freq       int `yaml:"freq" json:"freq"`
	DurationMs int `yaml:"duration_ms" json:"duration_ms"`
	LfoHzX10   int `yaml:"lfo_hz_x10,omitempty" json:"lfo_hz_x10,omitempty"`
	LfoDepth   int `yaml:"lfo_depth,omitempty" json:"lfo_depth,omitempty"`
}

// Tone is the tone payload of a tone definition.
type Tone struct {
	Freq       int `yaml:"freq" json:"freq"`
	DurationMs int `yaml:"duration_ms" json:"duration_ms"`
}

// SoundDef mirrors a single sound definition entry.
type SoundDef struct {
	Type     string `yaml:"type" json:"type"`
	Volume   int    `yaml:"volume" json:"volume"`
	Waveform string `yaml:"waveform,omitempty" json:"waveform,omitempty"`
	Unison   int    `yaml:"unison,omitempty" json:"unison,omitempty"`
	DetuneHz int    `yaml:"detune_hz,omitempty" json:"detune_hz,omitempty"`
	Tone     *Tone  `yaml:"tone,omitempty" json:"tone,omitempty"`
	Sequence []Note `yaml:"sequence,omitempty" json:"sequence,omitempty"`
	File     string `yaml:"file,omitempty" json:"file,omitempty"`
}

// Config is the parsed sounds_*.yaml file.
type Config struct {
	Version   int                 `yaml:"version"`
	SoundPath string              `yaml:"sound_path"`
	Sounds    map[string]SoundDef `yaml:"sounds"`
}

// LoadConfig parses a sounds_*.yaml file.
func LoadConfig(path string) (*Config, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var cfg Config
	if err := yaml.Unmarshal(raw, &cfg); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	return &cfg, nil
}

// Files returns the MP3 files referenced by the definition (the file field of
// every type=mp3 entry) in a stable order.
func (c *Config) Files() []string {
	names := make([]string, 0, len(c.Sounds))
	for name := range c.Sounds {
		names = append(names, name)
	}
	sort.Strings(names)

	var files []string
	for _, name := range names {
		def := c.Sounds[name]
		if def.Type == "mp3" && def.File != "" {
			files = append(files, def.File)
		}
	}
	return files
}

// Blob marshals the sound overrides into the JSON blob that is sent to the
// SoundService ("Sound Definitions" register). Only the "sounds" map is sent;
// the HL-only top-level fields (version, sound_path) are stripped, exactly like
// the Input-Configs path strips name/actions.
func (c *Config) Blob() ([]byte, error) {
	return json.Marshal(struct {
		Sounds map[string]SoundDef `json:"sounds"`
	}{Sounds: c.Sounds})
}
