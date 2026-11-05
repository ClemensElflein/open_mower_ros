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

// Shadowed is a candidate that was not used because its sidecar does not match the
// definition any more - typically a customised overlay render left over from an
// older YAML. It is reported (not silently dropped) so the user can re-render or
// delete it.
type Shadowed struct {
	Path   string
	Reason string // why the copy was skipped; the path is reported separately
}

// FindRender resolves a render job: it returns the most specific candidate whose
// sidecar was rendered from id.
//
// Unlike Find, a stale candidate does not shadow a valid one further down the
// list. That case is not theoretical: the overlay keeps an old render of a changed
// announcement while the image ships a fresh one, and treating the stale overlay
// copy as "the file" left the mower silent - sync skipped it and never looked at
// the shipped render. The skipped copies are returned so the caller can name the
// file that needs `soundctl tts`.
//
// When nothing matches, path is empty and reason says what is wrong with the most
// specific candidate that does exist (or that none exists at all).
func (l Layout) FindRender(file string, id TTSIdentity) (path string, shadowed []Shadowed, reason string) {
	for _, p := range l.Candidates(file) {
		st, err := os.Stat(p)
		if err != nil || !st.Mode().IsRegular() {
			continue
		}
		sc, err := ReadSidecar(p)
		switch {
		case err != nil:
			shadowed = append(shadowed, Shadowed{Path: p,
				Reason: fmt.Sprintf("no render metadata next to it (%v)", err)})
		case !sc.Matches(id):
			shadowed = append(shadowed, Shadowed{Path: p,
				Reason: fmt.Sprintf("rendered as %s, expected %s", sc.Fingerprint, id.Fingerprint())})
		default:
			return p, shadowed, ""
		}
	}
	if len(shadowed) > 0 {
		// Name the copy and the fix: the caller logs the reason without a path.
		return "", shadowed, fmt.Sprintf("%s: %s - run \"soundctl tts\"", shadowed[0].Path, shadowed[0].Reason)
	}
	return "", nil, fmt.Sprintf("%s not found (looked in %s)", file, strings.Join(l.Candidates(file), ", "))
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
