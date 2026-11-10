// Package fwsync parses sounds_*.yaml, validates it against the shared
// SoundService definition and synchronises the MP3 files with the firmware's
// FileService. It is the domain layer behind the soundctl commands.
package fwsync

import (
	"fmt"
	"os"
	"sort"

	"go.yaml.in/yaml/v3"
)

// TTS defaults. They are part of the render cache key (see TTSIdentity), so
// changing one of them re-renders every announcement on the next tts run.
const (
	// DefaultTTSEngine is what a definition gets when it does not name one: the
	// ElevenLabs cloud API.
	DefaultTTSEngine = TTSEngineElevenLabs
	// TTSEngineElevenLabs and TTSEngineCartesia are the cloud engines. The engine is
	// chosen per file (sounds_<lang>.yaml), so one language can use ElevenLabs and
	// another Cartesia.
	TTSEngineElevenLabs = "elevenlabs"
	TTSEngineCartesia   = "cartesia"
	// DefaultCartesiaModel is the Cartesia release alias: it follows the newest
	// stable 3.6 build. A dated id (sonic-3.6-2026-08-27) pins one build instead;
	// the API accepts those, `GET /models` only lists the aliases.
	DefaultCartesiaModel = "sonic-3.6"
	// DefaultTTSModel is the ElevenLabs model id. eleven_v4 is the quality
	// model; eleven_multilingual_v2 is the more conservative alternative and
	// eleven_flash_v2_5 the cheap/fast one (see the ElevenLabs model list).
	DefaultTTSModel   = "eleven_v4"
	DefaultTTSBitrate = "32k"

	// DefaultTTSSettings are ElevenLabs' own defaults for a voice; the YAML can
	// override each of them.
	DefaultTTSStability       = 0.5
	DefaultTTSSimilarityBoost = 0.75
	DefaultTTSStyle           = 0.0
	DefaultTTSUseSpeakerBoost = true

	// MaxMP3NameLen is the firmware limit for a file name in /sounds/
	// (sound_definition.hpp stores it in a fixed-size buffer).
	MaxMP3NameLen = 55
)

// Pick policies for a sound with several variants. The firmware picks one
// variant when it plays the sound; soundctl only carries the policy.
const (
	PickRandom = "random" // any variant (the default when pick is empty)
	PickRotate = "rotate" // never the same variant twice in a row
	PickFirst  = "first"  // always the first variant
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
	// Preempt is only decoded so validate() can refuse it: the low level's sound
	// state machine stops the player itself and then queues the announcement plus
	// the alert, so a sound needs no "cut everything" switch.
	Preempt bool `yaml:"preempt,omitempty" json:"-"`
	// AttackMs/DecayMs are the per-note envelope of a sequence: a fade-in at the
	// start of every note and a fade-out that turns a note into a "ping" instead of
	// a hard-gated rectangle. Both are 0..255 ms; 0 = instant onset / hold the note.
	AttackMs int `yaml:"attack_ms,omitempty" json:"attack_ms,omitempty"`
	DecayMs  int `yaml:"decay_ms,omitempty" json:"decay_ms,omitempty"`
	// RepeatMs repeats the sound every n ms until another sound plays (0 = play once).
	// The firmware player restarts the sound itself, so a repeating sound cannot fill
	// its request queue (see the repeat note in the firmware's sound_player.cpp).
	RepeatMs int `yaml:"repeat_ms,omitempty" json:"repeat_ms,omitempty"`
	// PauseAfterMs keeps the low level silent for that time after this sound ended,
	// before the next queued sound starts (0 = play through). The pause belongs to
	// this sound, so it works for every type; stop() cuts it short, and a repeating
	// sound (repeat_ms) does not pause between its own iterations.
	PauseAfterMs int    `yaml:"pause_after_ms,omitempty" json:"pause_after_ms,omitempty"`
	Tone         *Tone  `yaml:"tone,omitempty" json:"tone,omitempty"`
	Sequence     []Note `yaml:"sequence,omitempty" json:"sequence,omitempty"`

	// Variants are the playable takes of a type=mp3 sound: a tts text that
	// `soundctl tts` renders, or an existing MP3. Every sound (even one with a
	// single take) uses this list, so there is only one code path.
	Variants []Variant `yaml:"variants,omitempty" json:"variants,omitempty"`
	// File is the file the LL plays: the first variant. It is derived in normalize
	// (never set in the YAML) and stays in the blob for the firmware's single-file
	// field.
	File string `yaml:"-" json:"file,omitempty"`
	// Pick selects a variant: random (the default when empty), rotate or first.
	// It is a hint for the firmware; soundctl only carries it.
	Pick string `yaml:"pick,omitempty" json:"pick,omitempty"`
	// Every makes the LL decide on its own whether to play the sound, once per
	// interval ("120s") - background ambience, not an event. Chance (0..1) is
	// the probability of playing after such a decision. Without Every the sound
	// only plays when the high level requests it.
	Every  string   `yaml:"every,omitempty" json:"-"`
	Chance *float64 `yaml:"chance,omitempty" json:"chance,omitempty"`
	// EveryMs is Every in milliseconds for the blob (the firmware does not parse
	// duration strings). Filled in by Blob.
	EveryMs int `yaml:"-" json:"every_ms,omitempty"`

	// LegacyFile and LegacyTTS are decoded only so the validator can refuse a
	// definition that puts file/tts on the sound instead of on a variant.
	LegacyFile string `yaml:"file,omitempty" json:"-"`
	LegacyTTS  string `yaml:"tts,omitempty" json:"-"`
}

// Variant is one playable take of a sound. A plain string in the YAML is
// shorthand for {tts: "..."}; {file: "x.mp3"} names an existing recording.
type Variant struct {
	// File is the MP3 file name in /sounds/. For a tts variant an empty name is
	// filled in by normalize ("<language>_<sound>_<n>.mp3").
	File string `yaml:"file,omitempty" json:"file,omitempty"`
	// TTS is the text to speak; `soundctl tts` renders it into File. Host-only:
	// the firmware never sees the text.
	TTS string `yaml:"tts,omitempty" json:"-"`
}

// UnmarshalYAML accepts "text to speak" as shorthand for {tts: "text to speak"}.
func (v *Variant) UnmarshalYAML(node *yaml.Node) error {
	if node.Kind == yaml.ScalarNode {
		v.TTS = node.Value
		return nil
	}
	type plain Variant
	var p plain
	if err := node.Decode(&p); err != nil {
		return err
	}
	*v = Variant(p)
	return nil
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

// normalize derives what both the host tool and the firmware blob depend on: the
// file name of every variant (a tts variant without one becomes
// "<language>_<sound>_<n>.mp3") and the single-file field the current firmware
// plays (the first variant).
func (c *Config) normalize() {
	for name, def := range c.Sounds {
		for i := range def.Variants {
			v := &def.Variants[i]
			if v.File == "" && v.TTS != "" {
				v.File = variantFileName(c.Language, name, i+1)
			}
		}
		if len(def.Variants) > 0 {
			def.File = def.Variants[0].File
		}
		c.Sounds[name] = def
	}
}

// variantFileName is the derived MP3 name of a tts variant:
// "<language>_<sound>_<n>.mp3" (e.g. "en_boot_complete_2.mp3"). It shares the
// firmware's /sounds/ namespace, so it has to stay short and unique.
func variantFileName(language, sound string, n int) string {
	return fmt.Sprintf("%s_%s_%d.mp3", language, sound, n)
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

// Files returns the MP3 files referenced by the definition in a stable order:
// every variant of every type=mp3 entry.
func (c *Config) Files() []string {
	var files []string
	for _, name := range c.Names() {
		files = append(files, c.FileNames(name)...)
	}
	return files
}

// FileNames returns the MP3 files of one sound (nil for any other type).
func (c *Config) FileNames(name string) []string {
	def, ok := c.Sounds[name]
	if !ok || def.Type != "mp3" {
		return nil
	}
	files := make([]string, 0, len(def.Variants))
	for _, v := range def.Variants {
		if v.File != "" {
			files = append(files, v.File)
		}
	}
	return files
}

// FileName returns the MP3 file the current firmware plays for a definition: the
// first variant ("" for any other type).
func (c *Config) FileName(name string) string {
	def, ok := c.Sounds[name]
	if !ok || def.Type != "mp3" || len(def.Variants) == 0 {
		return ""
	}
	return def.Variants[0].File
}

// TTSSounds returns the sound ids that have at least one tts variant, in a stable
// order.
func (c *Config) TTSSounds() []string {
	var names []string
	for _, name := range c.Names() {
		for _, v := range c.Sounds[name].Variants {
			if v.TTS != "" {
				names = append(names, name)
				break
			}
		}
	}
	return names
}
