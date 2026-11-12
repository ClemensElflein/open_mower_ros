package fwsync

import (
	"fmt"
	"hash/crc32"
	"math"
	"strconv"
	"strings"
)

// Round2 rounds a voice setting to the two decimals that Key() encodes.
//
// The render request and the cache key have to agree on the precision. They did not: Key()
// formatted the values with two decimals while the request sent the raw float, so a
// definition with "stability: 0.504" asked the engine for 0.504 and kept a sidecar whose
// fingerprint still said 0.50 - the render looked up to date and the new setting was never
// applied.
//
// Rounding here rather than widening Key() is deliberate: every shipped definition uses
// 0.5 / 0.75 / 0.0, so this changes no fingerprint and re-renders nothing. Widening the key
// would invalidate every stored stand at once (ttsKeyVersion bump) for a precision that
// ElevenLabs' knobs do not audibly resolve.
func Round2(v float64) float64 {
	return math.Round(v*100) / 100
}

// TTSIdentity is everything that determines *which audio* a sound should have:
// the text, the voice and the encoder settings. It is the cache key of a render.
//
// The values are the *request*: a cloud engine may return different audio for the
// same identity when it updates a model, which is what `tts --force` is for.
// Everything that goes into the request is part of the key, so changing a voice
// setting re-renders the announcements.
//
// The voice settings carry the two decimals of Key() (see Round2), so the key, the request
// and the sidecar always describe the same audio.
type TTSIdentity struct {
	Engine          string
	Model           string // engine model id (ElevenLabs or Cartesia)
	Voice           string // engine voice id
	Language        string // ISO 639-1; sent to the engine and used as sounds/<lang>/
	Text            string
	Bitrate         string
	LoudNorm        bool
	Stability       float64
	SimilarityBoost float64
	Style           float64
	UseSpeakerBoost bool
}

// ttsKeyVersion is bumped whenever Key changes shape, so old caches miss.
const ttsKeyVersion = "tts3"

// Key is the canonical string the fingerprint hashes. Keep it stable: it is the
// cache key that lives in the sidecar files on the robot.
func (t TTSIdentity) Key() string {
	return strings.Join([]string{
		ttsKeyVersion,
		t.Engine,
		t.Model,
		t.Voice,
		t.Language,
		strconv.FormatFloat(t.Stability, 'f', 2, 64),
		strconv.FormatFloat(t.SimilarityBoost, 'f', 2, 64),
		strconv.FormatFloat(t.Style, 'f', 2, 64),
		strconv.FormatBool(t.UseSpeakerBoost),
		strconv.FormatBool(t.LoudNorm),
		t.Bitrate,
		t.Text,
	}, "|")
}

// Fingerprint is a short, stable hash of Key (used in logs and in the sidecar).
func (t TTSIdentity) Fingerprint() string {
	return fmt.Sprintf("%08x", crc32.ChecksumIEEE([]byte(t.Key())))
}

// String renders the identity for log output, without the (long) text.
func (t TTSIdentity) String() string {
	loud := "raw"
	if t.LoudNorm {
		loud = "loudnorm"
	}
	if t.Engine == TTSEngineCartesia {
		return fmt.Sprintf("%s %s voice=%s %s %s %s", t.Engine, t.Model, t.Voice, t.Bitrate, loud, t.Language)
	}
	boost := "noboost"
	if t.UseSpeakerBoost {
		boost = "boost"
	}
	return fmt.Sprintf("%s %s voice=%s stability=%.2f style=%.2f %s %s %s %s",
		t.Engine, t.Model, t.Voice, t.Stability, t.Style, boost, t.Bitrate, loud, t.Language)
}
