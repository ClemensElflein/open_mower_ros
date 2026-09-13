package fwsync

import (
	"encoding/json"
	"fmt"
	"os"
	"sort"
	"strings"
)

// enum is one "name -> value" enum of the shared definition, looked up
// case-insensitively (the JSON names are SCREAMING_CASE, the CLI/YAML use lower
// case).
type enum map[string]uint8

func newEnum(values map[string]int) enum {
	e := make(enum, len(values))
	for name, value := range values {
		e[strings.ToLower(name)] = uint8(value)
	}
	return e
}

func (e enum) value(name string) (uint8, bool) {
	v, ok := e[strings.ToLower(name)]
	return v, ok
}

func (e enum) names() []string {
	out := make([]string, 0, len(e))
	for n := range e {
		out = append(out, n)
	}
	sort.Strings(out)
	return out
}

// SoundIDs holds the enums the host tool has to translate: sound names (the keys
// of the YAML definition), SoundType (YAML "type") and Waveform (YAML "waveform"
// / --wave). All of them live in the shared sound_service.json, the single source
// of truth for LL + HL + Go.
type SoundIDs struct {
	sounds    enum
	types     enum
	waveforms enum
}

// LoadSoundIDs parses the enums of a service definition JSON file.
func LoadSoundIDs(path string) (*SoundIDs, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("read %s: %w", path, err)
	}

	var doc struct {
		Enums []struct {
			ID     string         `json:"id"`
			Values map[string]int `json:"values"`
		} `json:"enums"`
	}
	if err := json.Unmarshal(raw, &doc); err != nil {
		return nil, fmt.Errorf("parse %s: %w", path, err)
	}

	enums := make(map[string]enum, len(doc.Enums))
	for _, e := range doc.Enums {
		enums[e.ID] = newEnum(e.Values)
	}
	ids := &SoundIDs{
		sounds:    enums["SoundId"],
		types:     enums["SoundType"],
		waveforms: enums["Waveform"],
	}
	if len(ids.sounds) == 0 {
		return nil, fmt.Errorf("no SoundId enum found in %s", path)
	}
	return ids, nil
}

// Value returns the numeric SoundId for a name.
func (s *SoundIDs) Value(name string) (uint8, bool) { return s.sounds.value(name) }

// Has reports whether the given sound id is known.
func (s *SoundIDs) Has(name string) bool {
	_, ok := s.sounds.value(name)
	return ok
}

// Names returns the sorted sound ids.
func (s *SoundIDs) Names() []string { return s.sounds.names() }

// SoundType returns the numeric SoundType for a name ("tone"/"sequence"/"mp3").
func (s *SoundIDs) SoundType(name string) (uint8, bool) { return s.types.value(name) }

// Waveform returns the numeric Waveform for a name (e.g. "sine").
func (s *SoundIDs) Waveform(name string) (uint8, bool) { return s.waveforms.value(name) }
