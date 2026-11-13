package fwsync

import (
	"fmt"
	"strings"
	"testing"
)

// TestRenderJobsRoundVoiceSettingsToTheKeyPrecision pins the fix for a real staleness bug:
// Key() formats the voice settings with two decimals, so the request has to carry the same
// precision - otherwise "stability: 0.504" changed the request while the fingerprint still
// said 0.50 and the old render was kept as up to date.
func TestRenderJobsRoundVoiceSettingsToTheKeyPrecision(t *testing.T) {
	doc := "version: 1\nlanguage: de\nsound_path: /opt/open_mower_ros/sounds\n" +
		"tts:\n" +
		"  engine: elevenlabs\n" +
		"  bitrate: 32k\n" +
		"  loudnorm: true\n" +
		"  elevenlabs:\n" +
		"    model: eleven_v4\n" +
		"    voice: abc123\n" +
		"    stability: %s\n" +
		"    similarity_boost: 0.754\n" +
		"    style: 0.008\n" +
		"sounds:\n" +
		"  boot_complete:\n" +
		"    type: mp3\n" +
		"    volume: 90\n" +
		"    variants:\n" +
		"      - \"Ein Text\"\n"

	plain := loadFixture(t, fmt.Sprintf(doc, "0.5"))
	odd := loadFixture(t, fmt.Sprintf(doc, "0.504"))

	id := odd.RenderJobs()[0].Identity
	if id.Stability != 0.5 || id.SimilarityBoost != 0.75 || id.Style != 0.01 {
		t.Errorf("identity = stability %v, similarity %v, style %v; want them rounded to two decimals",
			id.Stability, id.SimilarityBoost, id.Style)
	}
	if a, b := plain.StoreKey(), odd.StoreKey(); a != b {
		t.Errorf("0.5 and 0.504 give different store keys (%s vs %s): the request would change without the key", a, b)
	}
}

// storeKeyDoc builds a minimal definition; only the parts that reach the render identity
// are parameterised, because those are the ones the key has to follow.
func storeKeyDoc(text, voice, bitrate string) string {
	return "version: 1\nlanguage: de\nsound_path: /opt/open_mower_ros/sounds\n" +
		"tts:\n" +
		"  engine: elevenlabs\n" +
		"  bitrate: " + bitrate + "\n" +
		"  loudnorm: true\n" +
		"  elevenlabs:\n" +
		"    model: eleven_v4\n" +
		"    voice: " + voice + "\n" +
		"    stability: 0.5\n" +
		"    similarity_boost: 0.75\n" +
		"    style: 0.0\n" +
		"    use_speaker_boost: true\n" +
		"sounds:\n" +
		"  boot_complete:\n" +
		"    type: mp3\n" +
		"    volume: 90\n" +
		"    variants:\n" +
		"      - \"" + text + "\"\n" +
		"      - \"Ein zweiter Text\"\n" +
		"  boot_ping:\n" +
		"    type: mp3\n" +
		"    volume: 80\n" +
		"    variants:\n" +
		"      - {file: zz_ping.mp3}\n"
}

// TestStoreKeyIsDeterministic: the CI computes the key in one job and pushes under it in
// another run, so the same definition must always give the same "<language>-<12 hex>" tag.
// (A deliberate format change is harmless: old content tags miss once and the CI falls back
// to the branch and base tags - so no golden value is pinned here.)
func TestStoreKeyIsDeterministic(t *testing.T) {
	cfg := loadFixture(t, storeKeyDoc("Ein Text", "abc123", "32k"))
	first := cfg.StoreKey()
	if !strings.HasPrefix(first, "de-") || len(first) != len("de-")+12 {
		t.Fatalf("StoreKey() = %q, want de-<12 hex characters>", first)
	}
	if again := loadFixture(t, storeKeyDoc("Ein Text", "abc123", "32k")).StoreKey(); again != first {
		t.Errorf("the same definition gave %q and %q", first, again)
	}
}

// TestStoreKeyCarriesTheLanguage: the tag names sounds/<language>.tgz, so the language has
// to be part of the name - otherwise two definitions that differ only in language would
// collide and restore the wrong files.
func TestStoreKeyCarriesTheLanguage(t *testing.T) {
	de := loadFixture(t, storeKeyDoc("Ein Text", "abc123", "32k")).StoreKey()
	fr := loadFixture(t, strings.Replace(storeKeyDoc("Ein Text", "abc123", "32k"),
		"language: de", "language: fr", 1)).StoreKey()
	if de == fr {
		t.Fatalf("de and fr share the store key %q", de)
	}
	if !strings.HasPrefix(fr, "fr-") {
		t.Errorf("StoreKey() = %q, want the fr- prefix", fr)
	}
}

// TestStoreKeyFollowsTheRenderRelevantFields is the contract behind the registry tags:
// every field that changes the audio must change the key, so that a new stand is stored
// under a new name and the old one stays reachable.
// Every field that does NOT change the audio must leave it alone, so that an unrelated
// edit does not fork the stand.
func TestStoreKeyFollowsTheRenderRelevantFields(t *testing.T) {
	base := loadFixture(t, storeKeyDoc("Ein Text", "abc123", "32k")).StoreKey()

	changed := map[string]string{
		"text":    loadFixture(t, storeKeyDoc("Anderer Text", "abc123", "32k")).StoreKey(),
		"voice":   loadFixture(t, storeKeyDoc("Ein Text", "xyz789", "32k")).StoreKey(),
		"bitrate": loadFixture(t, storeKeyDoc("Ein Text", "abc123", "48k")).StoreKey(),
	}
	for field, key := range changed {
		if key == base {
			t.Errorf("changing %s left the store key at %q - two different stands would collide", field, key)
		}
	}
	if t.Failed() {
		t.Logf("base key: %s", base)
	}
}

// TestStoreKeyIgnoresNonRenderFields: a field that does not reach the audio must not fork
// the stand. Otherwise a volume or pick edit would create a new tag, force a full download
// and make the old tag look like a different render - the key is meant to name what was
// actually rendered, nothing else.
func TestStoreKeyIgnoresNonRenderFields(t *testing.T) {
	doc := storeKeyDoc("Ein Text", "abc123", "32k")
	base := loadFixture(t, doc).StoreKey()

	tweaked := strings.Replace(doc, "volume: 90", "volume: 40", 1)
	tweaked = strings.Replace(tweaked, "    volume: 80\n    variants:\n      - {file: zz_ping.mp3}",
		"    volume: 55\n    pick: random\n    variants:\n      - {file: zz_ping.mp3}", 1)
	if tweaked == doc {
		t.Fatal("the fixture changed shape - update this test")
	}
	if got := loadFixture(t, tweaked).StoreKey(); got != base {
		t.Errorf("volume/pick changed the store key (%q -> %q); a stand would be forked for nothing", base, got)
	}
}

// TestStoreKeyCountsVariants: adding a variant adds a file to the stand.
func TestStoreKeyCountsVariants(t *testing.T) {
	one := loadFixture(t, "version: 1\nlanguage: de\nsound_path: /opt/open_mower_ros/sounds\n"+
		"tts:\n  engine: elevenlabs\n  bitrate: 32k\n  loudnorm: true\n"+
		"  elevenlabs:\n    model: eleven_v4\n    voice: abc123\n"+
		"sounds:\n  boot_complete:\n    type: mp3\n    volume: 90\n    variants:\n      - \"Ein Text\"\n")

	two := loadFixture(t, "version: 1\nlanguage: de\nsound_path: /opt/open_mower_ros/sounds\n"+
		"tts:\n  engine: elevenlabs\n  bitrate: 32k\n  loudnorm: true\n"+
		"  elevenlabs:\n    model: eleven_v4\n    voice: abc123\n"+
		"sounds:\n  boot_complete:\n    type: mp3\n    volume: 90\n    variants:\n      - \"Ein Text\"\n      - \"Noch einer\"\n")

	if one.StoreKey() == two.StoreKey() {
		t.Errorf("a definition with one and one with two variants share the key %q", one.StoreKey())
	}
}

// TestStoreKeyFollowsTheFileNames: the stand is a set of files, so the same texts under
// different names are a different stand. Swapping two variants keeps every identity but
// moves the texts to the other file - the key has to change, or the "exact" content tag
// would hand back files whose sidecars all mismatch.
func TestStoreKeyFollowsTheFileNames(t *testing.T) {
	doc := storeKeyDoc("Ein Text", "abc123", "32k")
	swapped := strings.Replace(doc,
		"      - \"Ein Text\"\n      - \"Ein zweiter Text\"\n",
		"      - \"Ein zweiter Text\"\n      - \"Ein Text\"\n", 1)
	if swapped == doc {
		t.Fatal("the fixture changed shape - update this test")
	}
	if a, b := loadFixture(t, doc).StoreKey(), loadFixture(t, swapped).StoreKey(); a == b {
		t.Errorf("swapping two variants left the store key at %q", a)
	}
}
