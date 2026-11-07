// Package tts renders the "tts:" texts of sounds_*.yaml into the 16 kHz mono
// MP3 files the firmware plays.
//
// The text is synthesised by the ElevenLabs cloud engine (elevenlabs.go) and
// encoded by ffmpeg (loudness normalisation, resampling, MP3). The Go build stays
// CGO-free: no model runs on the mower, which is what makes the image ~330 MB
// smaller than the offline engines it replaced.
package tts

import (
	"strings"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// Engine renders a text into a WAV file; the encoder then turns that into the
// firmware MP3 (16 kHz mono CBR, EBU R128).
type Engine interface {
	// Available reports why the engine cannot be used (nil = it can). It must not
	// touch the network: it runs on every start, also on the mower at boot.
	Available() error
	// Synthesize writes the spoken text to wav (16-bit PCM) and returns how long
	// the engine took.
	Synthesize(id fwsync.TTSIdentity, wav string) (time.Duration, error)
	// Describe renders the voice/settings of one synthesis for the log line.
	Describe(id fwsync.TTSIdentity) string
}

// tail returns the last n characters of s (error output is long and the useful
// part is at the end).
func tail(s string, n int) string {
	s = strings.TrimSpace(s)
	if len(s) <= n {
		return s
	}
	return s[len(s)-n:]
}
