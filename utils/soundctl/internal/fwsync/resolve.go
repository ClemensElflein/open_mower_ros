package fwsync

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

// Layout maps a logical file name to the MP3 on disk.
//
// The firmware only knows the flat /sounds/<file> contract, but on the host the
// same logical file can live in three places: the image (read only, shipped),
// a language sub directory of the image (pre-rendered by the CI) and the
// writable overlay (--sounds-dir, on the mower /data/sounds, holding the
// customised / re-rendered files). Find and Candidates implement the precedence
// "runtime before image, language specific before shared", so a customised file
// always wins over the shipped one without copying anything around.
type Layout struct {
	// SoundPath is the sound_path of the definition: the directory the image
	// ships (read only at runtime).
	SoundPath string
	// Overlay is the writable runtime directory. It may be empty, which means
	// "no overlay" (everything is read from the image).
	Overlay string
	// Language is the language sub directory (sounds/<lang>/...).
	Language string
}

// NewLayout builds the layout of a definition.
func NewLayout(cfg *Config, overlay string) Layout {
	return Layout{SoundPath: cfg.SoundPath, Overlay: strings.TrimSpace(overlay), Language: cfg.Language}
}

// Candidates returns the paths to try for a file, most specific first.
func (l Layout) Candidates(file string) []string {
	var out []string
	add := func(base, sub string) {
		if base == "" {
			return
		}
		if sub != "" {
			out = append(out, filepath.Join(base, sub, file))
			return
		}
		out = append(out, filepath.Join(base, file))
	}
	add(l.Overlay, l.Language)   // customised render for this language
	add(l.SoundPath, l.Language) // pre-rendered by the image/CI
	add(l.Overlay, "")           // customised shared asset
	add(l.SoundPath, "")         // shipped shared asset
	return out
}

// Find returns the first existing candidate for a file.
func (l Layout) Find(file string) (string, error) {
	cands := l.Candidates(file)
	for _, p := range cands {
		if st, err := os.Stat(p); err == nil && st.Mode().IsRegular() {
			return p, nil
		}
	}
	return "", fmt.Errorf("%s not found (looked in %s)", file, strings.Join(cands, ", "))
}

// OutDir is where generated files for the definition's language are written:
// the overlay if one is configured (so renders survive an image update), else
// the image's own language directory.
func (l Layout) OutDir() string {
	base := l.Overlay
	if base == "" {
		base = l.SoundPath
	}
	if l.Language == "" {
		return base
	}
	return filepath.Join(base, l.Language)
}
