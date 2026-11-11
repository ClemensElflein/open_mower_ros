package fwsync

import "testing"

// identityFixture is a render identity with every field carrying a value, so a
// change to Key's shape or to any field's formatting shows up in the fingerprint.
func identityFixture() TTSIdentity {
	return TTSIdentity{
		Engine:          "elevenlabs",
		Model:           "eleven_v4",
		Voice:           "5gaEGrWSk4v20HMfIhgC",
		Language:        "en",
		Text:            "The robot operating system has started.",
		Bitrate:         "32k",
		LoudNorm:        true,
		Stability:       0.5,
		SimilarityBoost: 0.75,
		Style:           0.0,
		UseSpeakerBoost: true,
	}
}

// TestIdentityFingerprintIsStable pins the cache key of a render. The sidecars on
// the robot store this fingerprint, so anything that changes it silently
// re-renders (and re-bills) every announcement that is already on the mower.
// A deliberate change means: bump ttsKeyVersion, then update the value here.
func TestIdentityFingerprintIsStable(t *testing.T) {
	const want = "ea7ce54c"
	if got := identityFixture().Fingerprint(); got != want {
		t.Errorf("Fingerprint() = %s, want %s - the cache key changed, so every existing render is invalidated. Bump ttsKeyVersion deliberately and update this value, or revert the change to Key()", got, want)
	}
}

// TestIdentityKeyCoversLanguage is the contract behind sending language_code to
// the engine: because Language is part of the key, the request gained a field
// without the key changing shape. If this ever fails, the hint would be sent for
// some renders and not for others with the same key, which is exactly the
// silent-stale-cache case ttsKeyVersion exists for.
func TestIdentityKeyCoversLanguage(t *testing.T) {
	en := identityFixture()
	de := identityFixture()
	de.Language = "de"
	if en.Fingerprint() == de.Fingerprint() {
		t.Fatal("two identities that differ only in Language share a fingerprint")
	}
}
