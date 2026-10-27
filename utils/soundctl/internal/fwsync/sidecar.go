package fwsync

import (
	"encoding/json"
	"fmt"
	"hash/crc32"
	"os"
	"time"
)

// SidecarSuffix is appended to the MP3 name of every rendered sound. The sidecar
// is both the renderer's cache entry and the staleness marker that sync checks,
// so a text change without a re-render is caught here instead of on the robot.
const SidecarSuffix = ".json"

// sidecarVersion is the layout version of the sidecar file itself.
const sidecarVersion = 1

// Sidecar describes one rendered MP3. It lives next to the file it describes
// (<file>.mp3.json), so it travels with the file into the image and into the
// writable overlay without any global index to keep in sync.
type Sidecar struct {
	Version     int     `json:"version"`
	Fingerprint string  `json:"fingerprint"`
	Text        string  `json:"text"`
	Engine      string  `json:"engine"`
	Model       string  `json:"model"`
	Language    string  `json:"language"`
	Sid         int     `json:"sid"`
	Speed       float64 `json:"speed"`
	LoudNorm    bool    `json:"loudnorm"`
	BitrateKbps int     `json:"bitrate_kbps"`
	SampleRate  int     `json:"sample_rate"`
	Channels    int     `json:"channels"`
	DurationMs  int     `json:"duration_ms"`
	Bytes       int64   `json:"bytes"`
	CRC32       uint32  `json:"crc32"`

	// Bookkeeping only; never compared (it must not affect the cache).
	RenderedAt string `json:"rendered_at,omitempty"`
	RenderMs   int64  `json:"render_ms,omitempty"`
	Generator  string `json:"generator,omitempty"`
}

// SidecarPath returns the sidecar path of an MP3.
func SidecarPath(mp3 string) string { return mp3 + SidecarSuffix }

// NewSidecar builds the sidecar of a freshly rendered MP3.
func NewSidecar(id TTSIdentity, info MP3Info, bytes int64, crc uint32) Sidecar {
	return Sidecar{
		Version:     sidecarVersion,
		Fingerprint: id.Fingerprint(),
		Text:        id.Text,
		Engine:      id.Engine,
		Model:       id.Model,
		Language:    id.Language,
		Sid:         id.Sid,
		Speed:       id.Speed,
		LoudNorm:    id.LoudNorm,
		BitrateKbps: info.BitrateKbps,
		SampleRate:  info.SampleRate,
		Channels:    info.Channels,
		DurationMs:  info.DurationMs,
		Bytes:       bytes,
		CRC32:       crc,
	}
}

// Stamped fills in the bookkeeping fields (they do not take part in Matches).
func (s *Sidecar) Stamped(generator string, renderMs int64, now time.Time) {
	s.RenderedAt = now.UTC().Format(time.RFC3339)
	s.RenderMs = renderMs
	s.Generator = generator
}

// Matches reports whether the sidecar was rendered from exactly this identity.
// The text is compared in addition to the fingerprint so that a (theoretical)
// hash collision cannot silently keep a stale file.
func (s Sidecar) Matches(id TTSIdentity) bool {
	return s.Version == sidecarVersion &&
		s.Fingerprint == id.Fingerprint() &&
		s.Text == id.Text
}

// MatchesFile cross-checks the recorded size and CRC against the file on disk,
// so a sidecar that survived a truncated copy does not count as valid.
func (s Sidecar) MatchesFile(data []byte) bool {
	return int64(len(data)) == s.Bytes && crc32.ChecksumIEEE(data) == s.CRC32
}

// ReadSidecar loads (and sanity checks) the sidecar of an MP3.
func ReadSidecar(mp3 string) (Sidecar, error) {
	raw, err := os.ReadFile(SidecarPath(mp3))
	if err != nil {
		return Sidecar{}, err
	}
	var s Sidecar
	if err := json.Unmarshal(raw, &s); err != nil {
		return Sidecar{}, fmt.Errorf("parse %s: %w", SidecarPath(mp3), err)
	}
	if s.Version != sidecarVersion {
		return Sidecar{}, fmt.Errorf("%s: unsupported sidecar version %d", SidecarPath(mp3), s.Version)
	}
	return s, nil
}

// WriteSidecar writes the sidecar atomically (tmp + rename), so a killed render
// never leaves a half-written cache entry behind.
func WriteSidecar(mp3 string, s Sidecar) error {
	raw, err := json.MarshalIndent(s, "", "  ")
	if err != nil {
		return err
	}
	raw = append(raw, '\n')

	path := SidecarPath(mp3)
	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, raw, 0o644); err != nil {
		return err
	}
	if err := os.Rename(tmp, path); err != nil {
		_ = os.Remove(tmp)
		return err
	}
	return nil
}
