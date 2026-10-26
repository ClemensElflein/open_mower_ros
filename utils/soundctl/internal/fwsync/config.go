// Package fwsync parses sounds_*.yaml, validates it against the shared
// SoundService definition and synchronises the MP3 files with the firmware's
// FileService. It is the domain layer behind the soundctl commands.
package fwsync

import (
	"encoding/json"
	"fmt"
	"hash/crc32"
	"os"
	"sort"
	"strconv"
	"strings"

	"gopkg.in/yaml.v3"
)

// TTS defaults. They are part of the render cache key (see TTSIdentity), so
// changing one of them re-renders every announcement on the next synth run.
const (
	DefaultTTSEngine  = "supertonic"
	DefaultTTSModel   = "sherpa-onnx-supertonic-3-tts-int8-2026-05-11"
	DefaultTTSSid     = 5
	DefaultTTSSpeed   = 1.0
	DefaultTTSBitrate = "64k"

	// MaxMP3NameLen is the firmware limit for a file name in /sounds/
	// (sound_definition.hpp stores it in a fixed-size buffer).
	MaxMP3NameLen = 55
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
	// Preempt marks alerts: the firmware stops a running sound, drops the pending
	// queue and plays it immediately (the ROM default sets it for EMERGENCY).
	Preempt bool `yaml:"preempt,omitempty" json:"preempt,omitempty"`
	// AttackMs/DecayMs are the per-note envelope of a sequence: a fade-in at the
	// start of every note and a fade-out that turns a note into a "ping" instead of
	// a hard-gated rectangle. Both are 0..255 ms; 0 = instant onset / hold the note.
	AttackMs int `yaml:"attack_ms,omitempty" json:"attack_ms,omitempty"`
	DecayMs  int `yaml:"decay_ms,omitempty" json:"decay_ms,omitempty"`
	// RepeatMs repeats the sound every n ms until another sound plays (0 = play once).
	// The firmware player restarts the sound itself, so a repeating sound cannot fill
	// its request queue (see the repeat note in the firmware's sound_player.cpp).
	RepeatMs int    `yaml:"repeat_ms,omitempty" json:"repeat_ms,omitempty"`
	Tone     *Tone  `yaml:"tone,omitempty" json:"tone,omitempty"`
	Sequence []Note `yaml:"sequence,omitempty" json:"sequence,omitempty"`
	File     string `yaml:"file,omitempty" json:"file,omitempty"`

	// TTS is the text to speak. soundctl synth renders it into File; the field
	// is host-only (json:"-") because the firmware never sees the text, only the
	// MP3 it was rendered into.
	TTS string `yaml:"tts,omitempty" json:"-"`
	// Sid/Speed override the tts block for this single sound (pointer, so an
	// explicit 0 or 1.0 can be told apart from "not set").
	Sid   *int     `yaml:"sid,omitempty" json:"-"`
	Speed *float64 `yaml:"speed,omitempty" json:"-"`
}

// TTSConfig is the top-level "tts:" block: the defaults used for every sound
// that has a "tts:" text. Every field is optional.
type TTSConfig struct {
	Engine   string   `yaml:"engine"`
	Model    string   `yaml:"model"`
	Sid      *int     `yaml:"sid"`
	Speed    *float64 `yaml:"speed"`
	Bitrate  string   `yaml:"bitrate"`
	LoudNorm *bool    `yaml:"loudnorm"`
	// Threads is the ONNX thread count of the TTS engine (0 = the CLI default).
	Threads int `yaml:"threads"`
}

// Config is the parsed sounds_*.yaml file.
type Config struct {
	Version   int                 `yaml:"version"`
	Language  string              `yaml:"language"`
	SoundPath string              `yaml:"sound_path"`
	TTS       TTSConfig           `yaml:"tts"`
	Sounds    map[string]SoundDef `yaml:"sounds"`
}

// LoadConfig parses a sounds_*.yaml file and fills in the defaults the rest of
// the package relies on (see normalize).
func LoadConfig(path string) (*Config, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}
	var cfg Config
	if err := yaml.Unmarshal(raw, &cfg); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}
	cfg.normalize()
	return &cfg, nil
}

// normalize fills in the defaults that both the host tool and the firmware blob
// depend on. A sound with a tts text but without a file name is stored as
// "<sound id>.mp3", so the firmware always has a name to play.
func (c *Config) normalize() {
	for name, def := range c.Sounds {
		if def.File == "" && def.TTS != "" {
			def.File = name + ".mp3"
			c.Sounds[name] = def
		}
	}
}

// Names returns the sound ids of the definition in a stable order.
func (c *Config) Names() []string {
	names := make([]string, 0, len(c.Sounds))
	for name := range c.Sounds {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// Files returns the MP3 files referenced by the definition (the file field of
// every type=mp3 entry) in a stable order.
func (c *Config) Files() []string {
	var files []string
	for _, name := range c.Names() {
		if file := c.FileName(name); file != "" {
			files = append(files, file)
		}
	}
	return files
}

// FileName returns the MP3 file name of a definition ("" for every other type).
func (c *Config) FileName(name string) string {
	def, ok := c.Sounds[name]
	if !ok || def.Type != "mp3" {
		return ""
	}
	return def.File
}

// TTSSounds returns the sound ids that carry a tts text, in a stable order.
func (c *Config) TTSSounds() []string {
	var names []string
	for _, name := range c.Names() {
		if c.Sounds[name].TTS != "" {
			names = append(names, name)
		}
	}
	return names
}

// TTSSettings is the resolved (i.e. defaulted) tts block. The resolved values
// are part of the render cache key, so a change in the YAML or in the defaults
// above invalidates the previously rendered MP3s.
type TTSSettings struct {
	Engine   string
	Model    string
	Sid      int
	Speed    float64
	Bitrate  string
	LoudNorm bool
	Threads  int
}

// TTSSettings resolves the top-level tts block, applying the package defaults.
func (c *Config) TTSSettings() TTSSettings {
	s := TTSSettings{
		Engine:  DefaultTTSEngine,
		Model:   DefaultTTSModel,
		Sid:     DefaultTTSSid,
		Speed:   DefaultTTSSpeed,
		Bitrate: DefaultTTSBitrate,
		// Loudness normalisation is on by default: raw engine output is ~9 dB
		// quieter than the rest of the corpus and varies by up to 3 LU between
		// sentences, which is audible over the mower.
		LoudNorm: true,
		Threads:  c.TTS.Threads,
	}
	if c.TTS.Engine != "" {
		s.Engine = c.TTS.Engine
	}
	if c.TTS.Model != "" {
		s.Model = c.TTS.Model
	}
	if c.TTS.Sid != nil {
		s.Sid = *c.TTS.Sid
	}
	if c.TTS.Speed != nil {
		s.Speed = *c.TTS.Speed
	}
	if c.TTS.Bitrate != "" {
		s.Bitrate = c.TTS.Bitrate
	}
	if c.TTS.LoudNorm != nil {
		s.LoudNorm = *c.TTS.LoudNorm
	}
	return s
}

// TTSIdentity is everything that determines *which audio* a sound should have:
// the text, the voice and the encoder settings. It is the cache key of a render.
//
// Two renders with the same identity produce the same audio on the same
// toolchain (engine binary + ffmpeg build); across ffmpeg versions the files can
// differ by a byte or two of container metadata, which is why the sidecar stores
// the identity (inputs) and the file's CRC32 separately instead of comparing
// audio between machines.
type TTSIdentity struct {
	Engine   string
	Model    string
	Language string
	Text     string
	Bitrate  string
	Sid      int
	Speed    float64
	LoudNorm bool
}

// ttsKeyVersion is bumped whenever Key changes shape, so old caches miss.
const ttsKeyVersion = "tts1"

// Key is the canonical string the fingerprint hashes. Keep it stable: it is the
// cache key that lives in the sidecar files on the robot.
func (t TTSIdentity) Key() string {
	return strings.Join([]string{
		ttsKeyVersion,
		t.Engine,
		t.Model,
		t.Language,
		strconv.Itoa(t.Sid),
		strconv.FormatFloat(t.Speed, 'f', 3, 64),
		strconv.FormatBool(t.LoudNorm),
		t.Bitrate,
		t.Text,
	}, "|")
}

// Fingerprint is a short, stable hash of Key (used in logs and in the sidecar).
func (t TTSIdentity) Fingerprint() string {
	return fmt.Sprintf("%08x", crc32.ChecksumIEEE([]byte(t.Key())))
}

// String renders the identity for log output, without the (long) text.
func (t TTSIdentity) String() string {
	loud := "raw"
	if t.LoudNorm {
		loud = "loudnorm"
	}
	return fmt.Sprintf("%s sid=%d %.2fx %s %s %s", t.Engine, t.Sid, t.Speed, t.Bitrate, loud, t.Language)
}

// Identity returns the render identity of one sound. ok is false for sounds
// that have no tts text (nothing to synthesise).
func (c *Config) Identity(name string) (TTSIdentity, bool) {
	def, ok := c.Sounds[name]
	if !ok || def.TTS == "" {
		return TTSIdentity{}, false
	}
	set := c.TTSSettings()
	id := TTSIdentity{
		Engine:   set.Engine,
		Model:    set.Model,
		Language: c.Language,
		Text:     def.TTS,
		Bitrate:  set.Bitrate,
		Sid:      set.Sid,
		Speed:    set.Speed,
		LoudNorm: set.LoudNorm,
	}
	if def.Sid != nil {
		id.Sid = *def.Sid
	}
	if def.Speed != nil {
		id.Speed = *def.Speed
	}
	return id, true
}

// Blob marshals the sound overrides into the JSON blob that is sent to the
// SoundService ("Sound Definitions" register). Only the "sounds" map is sent;
// the HL-only top-level fields (version, language, sound_path, tts) are
// stripped, exactly like the Input-Configs path strips name/actions.
func (c *Config) Blob() ([]byte, error) {
	return json.Marshal(struct {
		Sounds map[string]SoundDef `json:"sounds"`
	}{Sounds: c.Sounds})
}
