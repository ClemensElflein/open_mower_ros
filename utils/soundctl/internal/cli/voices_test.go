package cli

import (
	"testing"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/tts"
)

// voiceFixture builds a listed voice: the language of its reference recording plus the
// languages it is verified for.
func voiceFixture(native string, verified ...string) tts.Voice {
	v := tts.Voice{Labels: map[string]string{"language": native}}
	for _, lang := range verified {
		v.VerifiedLanguages = append(v.VerifiedLanguages, tts.VerifiedVoiceLanguage{Language: lang})
	}
	return v
}

// TestVoiceLanguagesShowsVerifiedInParentheses pins the LANGUAGE column: the label first,
// the other verified languages sorted in parentheses, without repeating the label.
func TestVoiceLanguagesShowsVerifiedInParentheses(t *testing.T) {
	cases := []struct {
		name  string
		voice tts.Voice
		want  string
	}{
		{"label only", voiceFixture("en"), "en"},
		{"label repeated in verified", voiceFixture("en", "en"), "en"},
		{"verified sorted", voiceFixture("en", "pl", "de"), "en (de, pl)"},
		{"non-english reference", voiceFixture("de", "de", "en"), "de (en)"},
		{"blank entries ignored", voiceFixture("en", " ", "de"), "en (de)"},
	}
	for _, c := range cases {
		if got := voiceLanguages(c.voice); got != c.want {
			t.Errorf("%s: voiceLanguages = %q, want %q", c.name, got, c.want)
		}
	}
}

// TestVoiceSpeaksLanguageFollowsVerifiedLanguages is the contract behind the filter: a
// premade voice labelled "en" answers --language de when German is verified for it, and
// does not answer --language fr without it. That difference is the whole point - an
// unverified language is still spoken, but with the accent of the reference recording.
func TestVoiceSpeaksLanguageFollowsVerifiedLanguages(t *testing.T) {
	liam := voiceFixture("en", "en", "de", "pl")

	for _, lang := range []string{"en", "EN", "de", "pl"} {
		if !voiceSpeaksLanguage(liam, lang) {
			t.Errorf("voiceSpeaksLanguage(%q) = false, want true", lang)
		}
	}
	if voiceSpeaksLanguage(liam, "fr") {
		t.Error("voiceSpeaksLanguage(fr) = true, but French is not verified for this voice")
	}
	if !voiceSpeaksLanguage(liam, "") {
		t.Error("an empty filter must not filter anything out")
	}
	if !voiceSpeaksLanguage(voiceFixture("fr"), "fr") {
		t.Error("a voice labelled French does not answer --language fr")
	}
}
