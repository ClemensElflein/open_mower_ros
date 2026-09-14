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

	for name, def := range c.Sounds {
		if !ids.Has(name) {
			errs = append(errs, fmt.Sprintf("unknown sound id %q (valid: %v)", name, ids.Names()))
			continue
		}
		errs = append(errs, validateSound(name, def, ids)...)
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
		if def.File == "" {
			errs = append(errs, fmt.Sprintf("%s: mp3 entry requires a non-empty 'file'", name))
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
