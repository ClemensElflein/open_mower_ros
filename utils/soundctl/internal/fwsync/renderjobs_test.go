package fwsync

import (
	"testing"

	"github.com/google/go-cmp/cmp"
)

// TestRenderJobsBuildsTheFullIdentity pins everything a render job carries,
// including the identity that becomes the cache key. It is the contract between
// the YAML and `soundctl tts`: a field that silently stops being copied into the
// identity would keep a stale MP3 for ever, because the sidecar fingerprint would
// still match.
func TestRenderJobsBuildsTheFullIdentity(t *testing.T) {
	doc := "version: 1\nlanguage: de\nsound_path: /opt/open_mower_ros/sounds\n" +
		"tts:\n" +
		"  engine: elevenlabs\n" +
		"  bitrate: 48k\n" +
		"  loudnorm: false\n" +
		"  elevenlabs:\n" +
		"    model: eleven_multilingual_v2\n" +
		"    voice: abc123\n" +
		"    stability: 0.4\n" +
		"    similarity_boost: 0.8\n" +
		"    style: 0.2\n" +
		"    use_speaker_boost: false\n" +
		"sounds:\n" +
		"  boot_complete:\n" +
		"    type: mp3\n" +
		"    volume: 80\n" +
		"    variants:\n" +
		"      - \"Erster Text\"\n" +
		"      - \"Zweiter Text\"\n" +
		"  deck_clean:\n" +
		"    type: mp3\n" +
		"    volume: 80\n" +
		"    variants:\n" +
		"      - {file: de_recording.mp3}\n" +
		"  startup:\n" +
		"    type: tone\n" +
		"    volume: 80\n" +
		"    tone: {freq: 440, duration_ms: 100}\n"
	cfg := loadFixture(t, doc)

	want := []RenderJob{
		{
			Sound: "boot_complete", Variant: 1, Label: "boot_complete[1]",
			File: "de_boot_complete_1.mp3",
			Identity: TTSIdentity{
				Engine: TTSEngineElevenLabs, Model: "eleven_multilingual_v2", Voice: "abc123",
				Language: "de", Text: "Erster Text", Bitrate: "48k", LoudNorm: false,
				Stability: 0.4, SimilarityBoost: 0.8, Style: 0.2, UseSpeakerBoost: false,
			},
		},
		{
			Sound: "boot_complete", Variant: 2, Label: "boot_complete[2]",
			File: "de_boot_complete_2.mp3",
			Identity: TTSIdentity{
				Engine: TTSEngineElevenLabs, Model: "eleven_multilingual_v2", Voice: "abc123",
				Language: "de", Text: "Zweiter Text", Bitrate: "48k", LoudNorm: false,
				Stability: 0.4, SimilarityBoost: 0.8, Style: 0.2, UseSpeakerBoost: false,
			},
		},
	}
	if diff := cmp.Diff(want, cfg.RenderJobs()); diff != "" {
		t.Fatalf("RenderJobs mismatch (-want +got):\n%s", diff)
	}
}

// TestRenderJobsCartesiaLeavesElevenLabsKnobsEmpty: Cartesia has no counterparts of
// stability/style/boost, so they stay zero - which also keeps them out of the
// render identity, so switching engines does not look like a settings change.
func TestRenderJobsCartesiaLeavesElevenLabsKnobsEmpty(t *testing.T) {
	doc := "version: 1\nlanguage: de\nsound_path: /opt/open_mower_ros/sounds\n" +
		"tts:\n  engine: cartesia\n  cartesia:\n    model: sonic-3.6\n    voice: car-voice\n" +
		"sounds:\n  boot_complete:\n    type: mp3\n    volume: 80\n    variants:\n      - \"Hallo\"\n"
	cfg := loadFixture(t, doc)

	jobs := cfg.RenderJobs()
	if len(jobs) != 1 {
		t.Fatalf("RenderJobs() = %d jobs, want 1", len(jobs))
	}
	want := TTSIdentity{
		Engine: TTSEngineCartesia, Model: "sonic-3.6", Voice: "car-voice",
		Language: "de", Text: "Hallo", Bitrate: DefaultTTSBitrate, LoudNorm: true,
	}
	if diff := cmp.Diff(want, jobs[0].Identity); diff != "" {
		t.Fatalf("identity mismatch (-want +got):\n%s", diff)
	}
}
