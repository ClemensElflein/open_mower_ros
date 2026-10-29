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
	"time"

	"gopkg.in/yaml.v3"
)

// TTS defaults. They are part of the render cache key (see TTSIdentity), so
// changing one of them re-renders every announcement on the next tts run.
const (
	// DefaultTTSEngine is the only engine: the ElevenLabs cloud API. The
	// offline engines (Supertonic, Piper) were removed; an older definition that
	// still names one fails with "unsupported tts engine".
	DefaultTTSEngine = "elevenlabs"
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

	// Variants are the playable takes of a type=mp3 sound: a tts text that
	// `soundctl tts` renders, or an existing MP3. Every sound (even one with a
	// single take) uses this list, so there is only one code path.
	Variants []Variant `yaml:"variants,omitempty" json:"variants,omitempty"`
	// File is the file the LL plays today: the first variant. It is derived in
	// normalize (never set in the YAML) and stays in the blob for the firmware's
	// single-file field.
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

	// Legacy inputs: the single-source form (file/tts directly on the sound) was
	// replaced by variants. They are decoded only so the validator can name them
	// instead of silently ignoring the value.
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

// TTSConfig is the top-level "tts:" block: the defaults for every sound with a
// tts variant.
//
// Engine-agnostic options are flat (bitrate, loudnorm), everything that belongs
// to ElevenLabs sits in the "elevenlabs:" block. Selecting an engine therefore
// stays a one-liner, and a future engine is another block next to it.
//
//	bitrate  MP3 bitrate of the encoder (ffmpeg)
//	loudnorm EBU R128 normalisation in ffmpeg
type TTSConfig struct {
	Engine   string `yaml:"engine"`
	Bitrate  string `yaml:"bitrate"`
	LoudNorm *bool  `yaml:"loudnorm"`

	// ElevenLabs is used when engine is "elevenlabs" (the cloud engine).
	ElevenLabs ElevenLabsConfig `yaml:"elevenlabs"`

	// Deprecated inputs of the removed offline engines. They are decoded only so
	// the validator can refuse an old file instead of silently voicing it with a
	// different engine.
	Supertonic SupertonicConfig `yaml:"supertonic"`
	Piper      PiperConfig      `yaml:"piper"`
	Speed      *float64         `yaml:"speed"`
	LegacySid  *int             `yaml:"sid"`
}

// ElevenLabsConfig is the engine block of the ElevenLabs cloud engine.
type ElevenLabsConfig struct {
	// Model is the ElevenLabs model id ("" = DefaultTTSModel, i.e. eleven_v4).
	Model string `yaml:"model"`
	// Voice is the ElevenLabs voice *id* (not a name); `soundctl tts voices`
	// lists the ids of the account.
	Voice string `yaml:"voice"`
	// Stability, SimilarityBoost and Style are 0..1, UseSpeakerBoost is a
	// switch. They are sent with every request and are part of the render cache
	// key, so changing one re-renders the announcements.
	Stability       *float64 `yaml:"stability"`
	SimilarityBoost *float64 `yaml:"similarity_boost"`
	Style           *float64 `yaml:"style"`
	UseSpeakerBoost *bool    `yaml:"use_speaker_boost"`
}

// SupertonicConfig is the engine block of the sherpa-onnx Supertonic engine.
type SupertonicConfig struct {
	// Model is the directory below <tts-dir>/models ("" = DefaultTTSModel).
	Model string `yaml:"model"`
	// Sid is the speaker id (0..9; the model ships 10 speakers).
	Sid *int `yaml:"sid"`
	// Threads is the ONNX thread count (0 = engine default).
	Threads int `yaml:"threads"`
}

// PiperConfig is the engine block of Piper.
type PiperConfig struct {
	// Voice is the voice name below <piper-dir> (e.g. "en_US-bryce-medium") or a
	// path to the .onnx file. It decides the language and the timber; there is no
	// speaker id for the single-speaker voices.
	Voice string `yaml:"voice"`
	// SentenceSilence is seconds of silence after each sentence (0 = none; the
	// binaries default to 0.2 (standalone) or 0 (piper1-gpl), so being explicit
	// keeps the output identical whichever binary renders).
	SentenceSilence *float64 `yaml:"sentence_silence"`
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

// TTSSettings is the resolved (i.e. defaulted) tts block. The resolved values are
// part of the render cache key, so a change in the YAML or in the defaults above
// invalidates the previously rendered MP3s.
type TTSSettings struct {
	Engine          string
	Bitrate         string
	LoudNorm        bool
	Model           string // ElevenLabs model id
	Voice           string // ElevenLabs voice id
	Stability       float64
	SimilarityBoost float64
	Style           float64
	UseSpeakerBoost bool
}

// TTSSettings resolves the top-level tts block, applying the package defaults.
func (c *Config) TTSSettings() TTSSettings {
	s := TTSSettings{
		Engine:          DefaultTTSEngine,
		Bitrate:         DefaultTTSBitrate,
		Model:           DefaultTTSModel,
		Stability:       DefaultTTSStability,
		SimilarityBoost: DefaultTTSSimilarityBoost,
		Style:           DefaultTTSStyle,
		UseSpeakerBoost: DefaultTTSUseSpeakerBoost,
		// Loudness normalisation is on by default: raw engine output is ~9 dB
		// quieter than the rest of the corpus and varies by up to 3 LU between
		// sentences, which is audible over the mower.
		LoudNorm: true,
	}

	if c.TTS.Engine != "" {
		s.Engine = c.TTS.Engine
	}
	if c.TTS.Bitrate != "" {
		s.Bitrate = c.TTS.Bitrate
	}
	if c.TTS.LoudNorm != nil {
		s.LoudNorm = *c.TTS.LoudNorm
	}

	el := c.TTS.ElevenLabs
	if el.Model != "" {
		s.Model = el.Model
	}
	s.Voice = el.Voice
	if el.Stability != nil {
		s.Stability = *el.Stability
	}
	if el.SimilarityBoost != nil {
		s.SimilarityBoost = *el.SimilarityBoost
	}
	if el.Style != nil {
		s.Style = *el.Style
	}
	if el.UseSpeakerBoost != nil {
		s.UseSpeakerBoost = *el.UseSpeakerBoost
	}
	return s
}

// TTSIdentity is everything that determines *which audio* a sound should have:
// the text, the voice and the encoder settings. It is the cache key of a render.
//
// The values are the *request*: a cloud engine may return different audio for the
// same identity when it updates a model, which is what `tts --force` is for.
// Everything that goes into the request is part of the key, so changing a voice
// setting re-renders the announcements.
type TTSIdentity struct {
	Engine          string
	Model           string // ElevenLabs model id
	Voice           string // ElevenLabs voice id
	Language        string
	Text            string
	Bitrate         string
	LoudNorm        bool
	Stability       float64
	SimilarityBoost float64
	Style           float64
	UseSpeakerBoost bool
}

// ttsKeyVersion is bumped whenever Key changes shape, so old caches miss.
// tts3: the ElevenLabs engine replaced the offline engines (supertonic/piper).
const ttsKeyVersion = "tts3"

// Key is the canonical string the fingerprint hashes. Keep it stable: it is the
// cache key that lives in the sidecar files on the robot.
func (t TTSIdentity) Key() string {
	return strings.Join([]string{
		ttsKeyVersion,
		t.Engine,
		t.Model,
		t.Voice,
		t.Language,
		strconv.FormatFloat(t.Stability, 'f', 2, 64),
		strconv.FormatFloat(t.SimilarityBoost, 'f', 2, 64),
		strconv.FormatFloat(t.Style, 'f', 2, 64),
		strconv.FormatBool(t.UseSpeakerBoost),
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
	boost := "noboost"
	if t.UseSpeakerBoost {
		boost = "boost"
	}
	return fmt.Sprintf("%s %s voice=%s stability=%.2f style=%.2f %s %s %s %s",
		t.Engine, t.Model, t.Voice, t.Stability, t.Style, boost, t.Bitrate, loud, t.Language)
}

// RenderJob is one MP3 that `soundctl tts` has to produce.
type RenderJob struct {
	Sound   string // sound id in the YAML ("boot_complete")
	Variant int    // 1-based number within the sound
	Label   string // "boot_complete" or "boot_complete[2]", for logs and --check
	File    string // MP3 file name in /sounds/
	// Identity is the render cache key of this variant.
	Identity TTSIdentity
}

// RenderJobs lists the variants that need a render, in a stable order. A variant
// with a file but without a tts text is a ready-made recording: there is nothing
// to synthesise, so it is not listed.
func (c *Config) RenderJobs() []RenderJob {
	set := c.TTSSettings()
	var jobs []RenderJob
	for _, name := range c.Names() {
		def := c.Sounds[name]
		if def.Type != "mp3" {
			continue
		}
		for i, v := range def.Variants {
			if v.TTS == "" {
				continue
			}
			label := name
			if len(def.Variants) > 1 {
				label = fmt.Sprintf("%s[%d]", name, i+1)
			}
			jobs = append(jobs, RenderJob{
				Sound:   name,
				Variant: i + 1,
				Label:   label,
				File:    v.File,
				Identity: TTSIdentity{
					Engine:          set.Engine,
					Model:           set.Model,
					Voice:           set.Voice,
					Language:        c.Language,
					Text:            v.TTS,
					Bitrate:         set.Bitrate,
					LoudNorm:        set.LoudNorm,
					Stability:       set.Stability,
					SimilarityBoost: set.SimilarityBoost,
					Style:           set.Style,
					UseSpeakerBoost: set.UseSpeakerBoost,
				},
			})
		}
	}
	return jobs
}

// Blob marshals the sound overrides into the JSON blob that is sent to the
// SoundService ("Sound Definitions" register). Only the "sounds" map is sent;
// the HL-only top-level fields (version, language, sound_path, tts) are
// stripped, exactly like the Input-Configs path strips name/actions.
func (c *Config) Blob() ([]byte, error) {
	// every_ms is derived here (the firmware does not parse duration strings);
	// copy the map so a second Blob call starts from the YAML values again.
	sounds := make(map[string]SoundDef, len(c.Sounds))
	for name, def := range c.Sounds {
		if def.Every != "" {
			d, err := time.ParseDuration(def.Every)
			if err != nil {
				return nil, fmt.Errorf("%s: invalid 'every' %q: %w", name, def.Every, err)
			}
			if d <= 0 {
				return nil, fmt.Errorf("%s: 'every' must be positive (got %q)", name, def.Every)
			}
			def.EveryMs = int(d.Milliseconds())
		}
		sounds[name] = def
	}
	return json.Marshal(struct {
		Sounds map[string]SoundDef `json:"sounds"`
	}{Sounds: sounds})
}
