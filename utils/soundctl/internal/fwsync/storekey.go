package fwsync

import (
	"crypto/sha256"
	"fmt"
	"sort"
	"strings"
)

// StoreKey is the registry tag of a definition's render stand: "<language>-<hash>", where
// the hash covers every render identity of the file (texts, engine, voice, settings,
// bitrate - everything that goes into the audio), sorted, plus their count.
//
// The name is derived from the content and not from a branch or a run, which gives three
// properties for free:
//   - a text/voice/settings edit produces a new tag, so the previous stand stays reachable
//     under its own name: undoing an edit costs nothing;
//   - a branch cannot overwrite main's stand, so main never re-renders because of a branch;
//   - after a merge main has the same definitions and therefore the same tag, so it finds
//     the renders the branch already paid for.
//
// The language comes first because it is the one thing the hash must not encode twice: the
// tarball it names is sounds/<language>.tgz, and a tag that claimed a different language
// than the definitions inside would restore the wrong files.
//
// SHA-256 truncated to 12 hex characters, not the CRC32 of TTSIdentity.Fingerprint: a
// collision there would hand back a stand for 62 files at once - all sidecars would
// mismatch and the whole language would be billed again - so the cheap hash is fine per
// file and too weak here.
func (c *Config) StoreKey() string {
	jobs := c.RenderJobs()
	ids := make([]string, 0, len(jobs))
	for _, job := range jobs {
		ids = append(ids, job.Identity.Fingerprint())
	}
	sort.Strings(ids)

	h := sha256.New()
	// The count is part of the input: adding or removing a variant must change the key even
	// in the (hypothetical) case where the remaining fingerprints hash to the same value.
	fmt.Fprintf(h, "%d\n", len(ids))
	for _, id := range ids {
		fmt.Fprintln(h, id)
	}
	sum := fmt.Sprintf("%x", h.Sum(nil)[:6])

	// Without a language there is nothing to prefix and a leading "-" would not be a valid
	// registry tag; Validate() rejects that case for any file with tts texts anyway.
	if lang := strings.TrimSpace(c.Language); lang != "" {
		return lang + "-" + sum
	}
	return sum
}
