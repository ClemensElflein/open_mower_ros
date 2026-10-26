package fwsync

import (
	"errors"
	"fmt"
	"strings"
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
		errs = append(errs, c.validateTTS()...)
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

// validateTTS checks the top-level tts block. Its resolved values are the
// defaults for every tts: text (and part of the render cache key), so a bad one
// would break every announcement.
func (c *Config) validateTTS() []string {
	set := c.TTSSettings()
	var errs []string

	if set.Engine != "supertonic" {
		errs = append(errs, fmt.Sprintf("tts.engine %q is not supported (use \"supertonic\")", set.Engine))
	}
	if strings.TrimSpace(set.Model) == "" {
		errs = append(errs, "tts.model must not be empty")
	}
	if _, err := ParseBitrate(set.Bitrate); err != nil {
		errs = append(errs, "tts."+err.Error())
	}
	errs = append(errs, validateSid("tts", set.Sid)...)
	errs = append(errs, validateSpeed("tts", set.Speed)...)
	if set.Threads < 0 || set.Threads > 64 {
		errs = append(errs, fmt.Sprintf("tts.threads must be 0..64 (got %d)", set.Threads))
	}
	return errs
}

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

// validateSid checks a Supertonic speaker id (the model ships 10 speakers).
func validateSid(name string, sid int) []string {
	if sid < 0 || sid > 9 {
		return []string{fmt.Sprintf("%s: 'sid' must be 0..9 (got %d)", name, sid)}
	}
	return nil
}

// validateSpeed checks the speaking rate. Outside this window the delivery gets
// either unintelligible (slow) or chipmunk-like (fast).
func validateSpeed(name string, speed float64) []string {
	if speed < 0.7 || speed > 1.5 {
		return []string{fmt.Sprintf("%s: 'speed' must be 0.7..1.5 (got %g)", name, speed)}
	}
	return nil
}

// validateFiles cross-checks the MP3 file names the definition references: two
// sound ids pointing at one file would fight over the same upload, and the
// firmware stores the name in a fixed-size buffer.
func (c *Config) validateFiles() []string {
	var errs []string
	seen := make(map[string]string, len(c.Sounds))
	for _, name := range c.Names() {
		file := c.FileName(name)
		if file == "" {
			continue
		}
		if prev, ok := seen[file]; ok {
			errs = append(errs, fmt.Sprintf("%s: 'file' %q is also used by %q", name, file, prev))
			continue
		}
		seen[file] = name
		if len(file) > MaxMP3NameLen {
			errs = append(errs, fmt.Sprintf("%s: 'file' is %d characters, the firmware limit is %d (%q)", name, len(file), MaxMP3NameLen, file))
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
// plus the payload of the selected type.
func validateSound(name string, def SoundDef, ids *SoundIDs) []string {
	errs := validateHeader(name, def, ids)

	if def.TTS != "" && def.Type != "mp3" {
		errs = append(errs, fmt.Sprintf("%s: 'tts' is only supported for type: mp3 (got %q)", name, def.Type))
	}
	if def.TTS == "" && (def.Sid != nil || def.Speed != nil) {
		errs = append(errs, fmt.Sprintf("%s: 'sid'/'speed' only apply to a sound with a 'tts' text", name))
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

// validateMP3 validates an MP3 definition. It always needs a source: either a
// tts: text (rendered by `soundctl tts`) or a file name. Both may be given —
// the file name is then just the name the render is stored under.
func validateMP3(name string, def SoundDef) []string {
	var errs []string
	file := def.File
	if file == "" && def.TTS != "" {
		file = name + ".mp3" // what normalize fills in
	}
	if file == "" && def.TTS == "" {
		return []string{fmt.Sprintf("%s: mp3 entry needs either a 'tts' text or a 'file'", name)}
	}

	if file != "" {
		if strings.ContainsAny(file, `/\`) {
			errs = append(errs, fmt.Sprintf("%s: 'file' must be a plain file name, not a path (got %q)", name, file))
		} else if !strings.HasSuffix(strings.ToLower(file), ".mp3") {
			errs = append(errs, fmt.Sprintf("%s: 'file' must end in .mp3 (got %q)", name, file))
		}
	}

	if def.TTS != "" {
		text := strings.TrimSpace(def.TTS)
		if text == "" {
			errs = append(errs, fmt.Sprintf("%s: 'tts' must not be empty or whitespace only", name))
		} else if n := len([]rune(text)); n > maxTTSText {
			errs = append(errs, fmt.Sprintf("%s: 'tts' text has %d characters, the limit is %d", name, n, maxTTSText))
		}
	}
	if def.Sid != nil {
		errs = append(errs, validateSid(name, *def.Sid)...)
	}
	if def.Speed != nil {
		errs = append(errs, validateSpeed(name, *def.Speed)...)
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
