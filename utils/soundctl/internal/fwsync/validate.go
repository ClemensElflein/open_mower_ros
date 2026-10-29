package fwsync

import (
	"errors"
	"fmt"
	"strings"
	"time"
)

// validUnison and maxNotes mirror the firmware contract (the synth only supports
// odd voice counts, the note parser stops after maxNotes entries). Everything
// else — sound ids, types and waveforms — comes from the shared definition.
var validUnison = map[int]bool{1: true, 3: true, 5: true, 7: true}

const maxNotes = 8

// maxTTSText bounds a tts text: a long paragraph would take minutes to render on
// the mower and is never a "sound" anyway.
const maxTTSText = 500

// Validate checks the parsed config against the firmware contract and returns
// a list of human-readable errors (empty means valid).
func (c *Config) Validate(ids *SoundIDs) []string {
	var errs []string

	if c.Version != 1 {
		errs = append(errs, fmt.Sprintf("top-level 'version' must be 1 (got %d)", c.Version))
	}
	if c.SoundPath == "" || c.SoundPath[0] != '/' {
		errs = append(errs, fmt.Sprintf("top-level 'sound_path' must be an absolute path (got %q)", c.SoundPath))
	}

	// The tts block and the language only matter when something is spoken; a
	// pure tone/sequence definition must not be forced to carry them.
	if len(c.TTSSounds()) > 0 {
		errs = append(errs, validateLanguage(c.Language)...)
		errs = append(errs, ValidateTTSSettings(c.TTSSettings())...)
	}
	// The offline engines were removed; a leftover block would silently voice the
	// texts with ElevenLabs, so name it instead of ignoring the values.
	if c.TTS.Supertonic.Model != "" || c.TTS.Supertonic.Sid != nil || c.TTS.Supertonic.Threads != 0 {
		errs = append(errs, "the 'supertonic' engine was removed: delete tts.supertonic (engine: elevenlabs)")
	}
	if c.TTS.Piper.Voice != "" || c.TTS.Piper.SentenceSilence != nil {
		errs = append(errs, "the 'piper' engine was removed: delete tts.piper (engine: elevenlabs)")
	}
	if c.TTS.Speed != nil {
		errs = append(errs, "tts.speed was removed (ElevenLabs's speaking rate is not used by this tool)")
	}
	if c.TTS.LegacySid != nil {
		errs = append(errs, "tts.sid was removed, use tts.elevenlabs.voice")
	}
	for name, def := range c.Sounds {
		if !ids.Has(name) {
			errs = append(errs, fmt.Sprintf("unknown sound id %q (valid: %v)", name, ids.Names()))
			continue
		}
		errs = append(errs, validateSound(name, def, ids)...)
	}

	errs = append(errs, c.validateFiles()...)
	return errs
}

// ValidateTTSSettings checks a resolved tts block. It is the rule set shared by
// the YAML definitions and the ad-hoc `soundctl say` path, so a quick local test
// can never use a value that sounds_*.yaml would reject (and vice versa).
func ValidateTTSSettings(set TTSSettings) []string {
	var errs []string

	if _, err := ParseBitrate(set.Bitrate); err != nil {
		errs = append(errs, "tts."+err.Error())
	}
	if set.Engine != "elevenlabs" {
		return append(errs, fmt.Sprintf("tts.engine %q is not supported (use \"elevenlabs\")", set.Engine))
	}
	if strings.TrimSpace(set.Model) == "" {
		errs = append(errs, "tts.elevenlabs.model must not be empty (e.g. \"eleven_v4\")")
	}
	if strings.TrimSpace(set.Voice) == "" {
		errs = append(errs, "tts.elevenlabs.voice must not be empty (an ElevenLabs voice id; `soundctl tts voices` lists them)")
	}
	errs = append(errs, validateUnit("tts.elevenlabs.stability", set.Stability)...)
	errs = append(errs, validateUnit("tts.elevenlabs.similarity_boost", set.SimilarityBoost)...)
	errs = append(errs, validateUnit("tts.elevenlabs.style", set.Style)...)
	return errs
}

// ValidateLanguage exposes the language rule (it becomes the sounds/<lang>/
// directory) to the ad-hoc `soundctl say` path.
func ValidateLanguage(lang string) []string { return validateLanguage(lang) }

// validateLanguage checks the language, which becomes the sounds/<lang>/
// sub directory and therefore has to be a safe path component.
func validateLanguage(lang string) []string {
	if lang == "" {
		return []string{"top-level 'language' is required when 'tts' texts are present (e.g. \"en\")"}
	}
	if len(lang) > 12 {
		return []string{fmt.Sprintf("language %q is too long (max 12 characters)", lang)}
	}
	for _, r := range lang {
		if (r < 'a' || r > 'z') && (r < '0' || r > '9') && r != '-' {
			return []string{fmt.Sprintf("language %q must be lowercase letters, digits or '-' (it is used as a directory name)", lang)}
		}
	}
	return nil
}

// validateUnit checks a 0..1 voice setting (stability, similarity_boost, style).
func validateUnit(name string, v float64) []string {
	if v < 0 || v > 1 {
		return []string{fmt.Sprintf("%s must be 0..1 (got %g)", name, v)}
	}
	return nil
}

// validateFiles cross-checks the MP3 files the definition references: two sounds
// pointing at one file would fight over the same upload, and the firmware stores
// the name in a fixed-size buffer.
func (c *Config) validateFiles() []string {
	var errs []string
	seen := make(map[string]string)
	for _, name := range c.Names() {
		for _, file := range c.FileNames(name) {
			if prev, ok := seen[file]; ok {
				if prev != name {
					errs = append(errs, fmt.Sprintf("%s: file %q is also used by %q", name, file, prev))
				}
				continue
			}
			seen[file] = name
			if strings.ContainsAny(file, `/\`) {
				errs = append(errs, fmt.Sprintf("%s: file %q must be a plain file name, not a path", name, file))
				continue
			}
			if len(file) > MaxMP3NameLen {
				errs = append(errs, fmt.Sprintf("%s: file %q is %d characters, the firmware limit is %d", name, file, len(file), MaxMP3NameLen))
			}
			if !strings.HasSuffix(strings.ToLower(file), ".mp3") {
				errs = append(errs, fmt.Sprintf("%s: file %q must end in .mp3", name, file))
			}
		}
	}
	return errs
}

// validateOne validates a runtime definition (`soundctl play`). It applies the
// same header rules as the YAML path; a sequence's note text is parsed by the
// firmware (its length is checked while packing the RPC), so only a tone has a
// payload to check here.
func validateOne(name string, def SoundDef, ids *SoundIDs) error {
	errs := validateHeader(name, def, ids)
	if def.Tone != nil {
		errs = append(errs, validateTone(name, "", *def.Tone)...)
	}
	if len(errs) > 0 {
		return errors.New(strings.Join(errs, "; "))
	}
	return nil
}

// validateVolume checks a master volume value (0..100).
func validateVolume(v int) error {
	if v < 0 || v > 100 {
		return fmt.Errorf("master volume must be 0..100 (got %d)", v)
	}
	return nil
}

// validateSound validates one complete YAML definition: the shared header rules
// plus the payload of the selected type. Every playable take is a variant, so an
// mp3 sound with one take is variants: ["text to speak"] (or [{file: "x.mp3"}]).
func validateSound(name string, def SoundDef, ids *SoundIDs) []string {
	errs := validateHeader(name, def, ids)

	// The single-source form was replaced by variants.
	if def.LegacyFile != "" || def.LegacyTTS != "" {
		errs = append(errs, fmt.Sprintf("%s: 'file'/'tts' at sound level was replaced by 'variants' (e.g. variants: [\"text to speak\"], variants: [{file: \"x.mp3\"}])", name))
	}
	if def.Type != "mp3" && len(def.Variants) > 0 {
		errs = append(errs, fmt.Sprintf("%s: 'variants' only apply to type: mp3 (got %q)", name, def.Type))
	}
	if def.Type != "mp3" && (def.Every != "" || def.Chance != nil || def.Pick != "") {
		errs = append(errs, fmt.Sprintf("%s: 'pick'/'every'/'chance' only apply to type: mp3 (got %q)", name, def.Type))
	}

	switch def.Type {
	case "tone":
		if def.Tone == nil {
			errs = append(errs, fmt.Sprintf("%s: tone entry requires a 'tone' mapping", name))
			break
		}
		errs = append(errs, validateTone(name, "tone.", *def.Tone)...)
	case "sequence":
		if len(def.Sequence) == 0 || len(def.Sequence) > maxNotes {
			errs = append(errs, fmt.Sprintf("%s: 'sequence' must have 1..%d notes (got %d)", name, maxNotes, len(def.Sequence)))
			break
		}
		for i, n := range def.Sequence {
			prefix := fmt.Sprintf("sequence[%d].", i)
			errs = append(errs, validateTone(name, prefix, Tone{Freq: n.Freq, DurationMs: n.DurationMs})...)
			errs = append(errs, validateRange(name, prefix+"lfo_hz_x10", n.LfoHzX10, 0xFFFF)...)
			errs = append(errs, validateRange(name, prefix+"lfo_depth", n.LfoDepth, 0xFFFF)...)
		}
	case "mp3":
		errs = append(errs, validateMP3(name, def)...)
	}

	return errs
}

// validateMP3 validates the variants of an MP3 sound: every take is a tts text
// (`soundctl tts` renders it) or a ready-made file. The playback policy fields
// (pick/every/chance) are hints for the firmware; they are validated here so a
// typo does not travel silently.
func validateMP3(name string, def SoundDef) []string {
	var errs []string
	if len(def.Variants) == 0 {
		return []string{fmt.Sprintf("%s: mp3 entry needs 'variants' (e.g. variants: [\"text to speak\"] or variants: [{file: \"x.mp3\"}])", name)}
	}
	for i, v := range def.Variants {
		label := fmt.Sprintf("%s: variants[%d]", name, i+1)
		if v.TTS == "" && v.File == "" {
			errs = append(errs, label+": needs a 'tts' text or a 'file'")
			continue
		}
		if v.TTS != "" {
			text := strings.TrimSpace(v.TTS)
			if text == "" {
				errs = append(errs, label+": 'tts' must not be empty or whitespace only")
			} else if n := len([]rune(text)); n > maxTTSText {
				errs = append(errs, fmt.Sprintf("%s: 'tts' text has %d characters, the limit is %d", label, n, maxTTSText))
			}
		}
	}

	switch def.Pick {
	case "", PickRandom, PickRotate, PickFirst:
	default:
		errs = append(errs, fmt.Sprintf("%s: 'pick' must be %q, %q or %q (got %q)", name, PickRandom, PickRotate, PickFirst, def.Pick))
	}
	if def.Every != "" {
		d, err := time.ParseDuration(def.Every)
		if err != nil {
			errs = append(errs, fmt.Sprintf("%s: 'every' is not a duration such as \"120s\" (%v)", name, err))
		} else if d < time.Second {
			errs = append(errs, fmt.Sprintf("%s: 'every' must be at least 1s (got %s)", name, d))
		}
	}
	if def.Chance != nil {
		if def.Every == "" {
			errs = append(errs, fmt.Sprintf("%s: 'chance' needs 'every' (it is the probability per evaluation)", name))
		}
		if *def.Chance < 0 || *def.Chance > 1 {
			errs = append(errs, fmt.Sprintf("%s: 'chance' must be 0..1 (got %g)", name, *def.Chance))
		}
	}
	return errs
}

// validateHeader validates the fields every definition shares: type, volume,
// waveform, unison and detune. The runtime path uses it too, so a value that
// sounds_*.yaml rejects cannot slip through `soundctl play`.
func validateHeader(name string, def SoundDef, ids *SoundIDs) []string {
	var errs []string

	if _, ok := ids.SoundType(def.Type); !ok {
		return []string{fmt.Sprintf("%s: invalid or missing 'type' (got %q, valid: %v)", name, def.Type, ids.types.names())}
	}

	if def.Volume < 0 || def.Volume > 100 {
		errs = append(errs, fmt.Sprintf("%s: 'volume' must be 0..100 (got %d)", name, def.Volume))
	}
	if def.Waveform != "" {
		if _, ok := ids.Waveform(def.Waveform); !ok {
			errs = append(errs, fmt.Sprintf("%s: invalid 'waveform' %q (valid: %v)", name, def.Waveform, ids.waveforms.names()))
		}
	}
	if def.Unison != 0 && !validUnison[def.Unison] {
		errs = append(errs, fmt.Sprintf("%s: invalid 'unison' %d (use 1, 3, 5 or 7)", name, def.Unison))
	}
	if def.DetuneHz < 0 || def.DetuneHz > 0xFFFF {
		errs = append(errs, fmt.Sprintf("%s: 'detune_hz' must be 0..65535 (got %d)", name, def.DetuneHz))
	}
	// The firmware stores the envelope in a byte each (sound_definition.hpp).
	errs = append(errs, validateRange(name, "attack_ms", def.AttackMs, 255)...)
	errs = append(errs, validateRange(name, "decay_ms", def.DecayMs, 255)...)
	// repeat_ms is a uint16 in the definition (0 = play once).
	errs = append(errs, validateRange(name, "repeat_ms", def.RepeatMs, 0xFFFF)...)

	return errs
}

// validateTone validates the tone ranges. Everything is uint16 — the RPC parameters,
// the sequence notes and the flat tone fields — so a YAML value that the firmware
// would truncate is rejected here, and prefix qualifies the field names ("tone." for a
// definition, "" for the RPC arguments).
func validateTone(name, prefix string, tone Tone) []string {
	var errs []string
	errs = append(errs, validateRange(name, prefix+"freq", tone.Freq, 0xFFFF)...)
	errs = append(errs, validateRange(name, prefix+"duration_ms", tone.DurationMs, 0xFFFF)...)
	return errs
}

// validateRange checks one integer field against [0, max]. It is the single place
// that knows how an out-of-range value is reported.
func validateRange(name, field string, v int, max uint64) []string {
	if v < 0 || uint64(v) > max {
		return []string{fmt.Sprintf("%s: %s must be 0..%d (got %d)", name, field, max, v)}
	}
	return nil
}
