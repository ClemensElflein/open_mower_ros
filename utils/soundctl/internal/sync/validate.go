package sync

import "fmt"

var validTypes = map[string]bool{"tone": true, "sequence": true, "mp3": true}
var validWaveforms = map[string]bool{"sine": true, "square": true, "triangle": true, "saw": true}
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
		errs = append(errs, validateSound(name, def)...)
	}
	return errs
}

func validateSound(name string, def SoundDef) []string {
	var errs []string

	if !validTypes[def.Type] {
		return append(errs, fmt.Sprintf("%s: invalid or missing 'type' (got %q)", name, def.Type))
	}

	if def.Volume < 0 || def.Volume > 100 {
		errs = append(errs, fmt.Sprintf("%s: 'volume' must be 0..100 (got %d)", name, def.Volume))
	}
	if def.Waveform != "" && !validWaveforms[def.Waveform] {
		errs = append(errs, fmt.Sprintf("%s: invalid 'waveform' %q", name, def.Waveform))
	}
	if def.Unison != 0 && !validUnison[def.Unison] {
		errs = append(errs, fmt.Sprintf("%s: invalid 'unison' %d (use 1, 3, 5 or 7)", name, def.Unison))
	}
	if def.DetuneHz < 0 || def.DetuneHz > 0xFFFF {
		errs = append(errs, fmt.Sprintf("%s: 'detune_hz' must be 0..65535 (got %d)", name, def.DetuneHz))
	}

	switch def.Type {
	case "tone":
		if def.Tone == nil {
			errs = append(errs, fmt.Sprintf("%s: tone entry requires a 'tone' mapping", name))
		} else {
			if def.Tone.Freq < 0 || def.Tone.Freq > 0xFFFFFFFF {
				errs = append(errs, fmt.Sprintf("%s: 'tone.freq' out of range", name))
			}
			if def.Tone.DurationMs < 0 || def.Tone.DurationMs > 0xFFFFFFFF {
				errs = append(errs, fmt.Sprintf("%s: 'tone.duration_ms' out of range", name))
			}
		}
	case "sequence":
		if len(def.Sequence) == 0 || len(def.Sequence) > maxNotes {
			errs = append(errs, fmt.Sprintf("%s: 'sequence' must have 1..%d notes (got %d)", name, maxNotes, len(def.Sequence)))
		} else {
			for i, n := range def.Sequence {
				if n.Freq < 0 || n.Freq > 0xFFFF {
					errs = append(errs, fmt.Sprintf("%s: sequence[%d].freq out of range", name, i))
				}
				if n.DurationMs < 0 || n.DurationMs > 0xFFFF {
					errs = append(errs, fmt.Sprintf("%s: sequence[%d].duration_ms out of range", name, i))
				}
				if n.LfoHzX10 < 0 || n.LfoHzX10 > 0xFFFF {
					errs = append(errs, fmt.Sprintf("%s: sequence[%d].lfo_hz_x10 out of range", name, i))
				}
				if n.LfoDepth < 0 || n.LfoDepth > 0xFFFF {
					errs = append(errs, fmt.Sprintf("%s: sequence[%d].lfo_depth out of range", name, i))
				}
			}
		}
	case "mp3":
		if def.File == "" {
			errs = append(errs, fmt.Sprintf("%s: mp3 entry requires a non-empty 'file'", name))
		}
	}

	return errs
}
