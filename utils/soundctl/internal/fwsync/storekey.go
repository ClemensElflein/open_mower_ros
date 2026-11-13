package fwsync

import (
	"crypto/sha256"
	"fmt"
	"sort"
	"strings"
)

// StoreKey is the registry tag of a definition's render stand: "<language>-<hash>". The
// hash covers every render job of the file - the MP3 name it writes plus the full render
// identity (texts, engine, voice, settings, bitrate: everything that goes into the audio).
// The CI uses it to find the stand of exactly these definitions from any branch; see
// sounds/README.md ("Where the renders live").
//
// The input is the full TTSIdentity.Key, not its CRC32 Fingerprint: hashing CRC32s would
// cap the whole key at CRC32 strength. The file name is part of it because the stand is a
// set of files: two definitions that put the same texts under different names (e.g. two
// variants swapped) are different stands.
//
// The language comes first because the tag names the tarball sounds/<language>.tgz.
//
// A change of this format is cheap: the old content tags simply miss once, and the CI falls
// back to the branch and base tags, whose sidecars still match.
func (c *Config) StoreKey() string {
	jobs := c.RenderJobs()
	entries := make([]string, 0, len(jobs))
	for _, job := range jobs {
		entries = append(entries, job.File+"\t"+job.Identity.Key())
	}
	sort.Strings(entries)

	h := sha256.New()
	for _, e := range entries {
		// NUL-terminated: unambiguous for any text without a NUL, which a spoken text has no
		// reason to contain.
		fmt.Fprintf(h, "%s\x00", e)
	}
	sum := fmt.Sprintf("%x", h.Sum(nil)[:6])

	// Without a language there is nothing to prefix and a leading "-" would not be a valid
	// registry tag; Validate() rejects that case for any file with tts texts anyway.
	if lang := strings.TrimSpace(c.Language); lang != "" {
		return lang + "-" + sum
	}
	return sum
}
