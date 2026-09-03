package sync

import (
	"encoding/json"
	"fmt"
	"os"
	"sort"
	"strings"
)

// SoundIDs is the canonical set of sound identifiers, loaded from the shared
// sound_service.json definition (single source of truth for LL + HL + Go).
type SoundIDs struct {
	names map[string]uint8
}

// LoadSoundIDs parses the SoundId enum from a service definition JSON file.
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

	for _, e := range doc.Enums {
		if e.ID != "SoundId" {
			continue
		}
		ids := &SoundIDs{names: make(map[string]uint8, len(e.Values))}
		for name, value := range e.Values {
			ids.names[strings.ToLower(name)] = uint8(value)
		}
		return ids, nil
	}
	return nil, fmt.Errorf("no SoundId enum found in %s", path)
}

// Has reports whether the given snake_case sound id is known.
func (s *SoundIDs) Has(name string) bool {
	_, ok := s.names[name]
	return ok
}

// Names returns the sorted snake_case sound id names.
func (s *SoundIDs) Names() []string {
	out := make([]string, 0, len(s.names))
	for n := range s.names {
		out = append(out, n)
	}
	sort.Strings(out)
	return out
}
