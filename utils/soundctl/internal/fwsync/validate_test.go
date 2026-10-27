package fwsync

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// loadFixture parses a YAML document from a temp file, so the tests exercise the
// same LoadConfig -> normalize -> Validate path as the CLI.
func loadFixture(t *testing.T, doc string) *Config {
	t.Helper()
	path := filepath.Join(t.TempDir(), "sounds.yaml")
	if err := os.WriteFile(path, []byte(doc), 0o644); err != nil {
		t.Fatal(err)
	}
	cfg, err := LoadConfig(path)
	if err != nil {
		t.Fatalf("LoadConfig: %v", err)
	}
	return cfg
}

// validBase is a minimal valid definition; every case below replaces one part of
// it to isolate a single rule.
const validBase = `
version: 1
language: en
sound_path: /opt/open_mower_ros/sounds
tts:
  speed: 1.0
  bitrate: 32k
  supertonic:
    sid: 5
sounds:
  boot_complete:
    type: mp3
    volume: 80
    tts: "Robot boot complete"
`

// TestValidateShippedDefinitions keeps the two checked-in definitions valid: the
// robot runs `soundctl sync` on them, and a validation error there means silence.
func TestValidateShippedDefinitions(t *testing.T) {
	ids := sharedIDs(t)
	for _, name := range []string{"sounds_en.yaml", "sounds_de.yaml"} {
		path := filepath.Join("..", "..", "..", "..", "src", "open_mower", "params", name)
		cfg, err := LoadConfig(path)
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if errs := cfg.Validate(ids); len(errs) > 0 {
			t.Fatalf("%s: %d validation error(s):\n%s", name, len(errs), strings.Join(errs, "\n"))
		}
	}
}

func TestValidateFillsTTSFileName(t *testing.T) {
	cfg := loadFixture(t, validBase)
	if got := cfg.FileName("boot_complete"); got != "boot_complete.mp3" {
		t.Fatalf("FileName = %q, want boot_complete.mp3 (filled in for a tts-only sound)", got)
	}
	if errs := cfg.Validate(sharedIDs(t)); len(errs) > 0 {
		t.Fatalf("valid definition rejected: %v", errs)
	}
}

func TestValidateRejectsContractViolations(t *testing.T) {
	ids := sharedIDs(t)
	cases := []struct {
		name string
		doc  string
		want string
	}{
		{
			name: "unknown sound id",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  nonsense:\n    type: tone\n    volume: 50\n    tone: {freq: 440, duration_ms: 100}\n",
			want: `unknown sound id "nonsense"`,
		},
		{
			name: "relative sound_path",
			doc:  "version: 1\nsound_path: sounds\nsounds:\n  success:\n    type: tone\n    volume: 50\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "must be an absolute path",
		},
		{
			name: "wrong version",
			doc:  "version: 2\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    type: tone\n    volume: 50\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "'version' must be 1",
		},
		{
			name: "volume out of range",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    type: tone\n    volume: 101\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "'volume' must be 0..100 (got 101)",
		},
		{
			name: "unknown waveform",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_ping:\n    type: tone\n    volume: 50\n    waveform: noise\n    tone: {freq: 440, duration_ms: 100}\n",
			want: `invalid 'waveform' "noise"`,
		},
		{
			name: "even unison",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_ping:\n    type: tone\n    volume: 50\n    unison: 2\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "invalid 'unison' 2",
		},
		{
			name: "attack out of byte range",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_ping:\n    type: tone\n    volume: 50\n    attack_ms: 300\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "attack_ms must be 0..255 (got 300)",
		},
		{
			name: "repeat out of uint16 range",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_ping:\n    type: tone\n    volume: 50\n    repeat_ms: 70000\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "repeat_ms must be 0..65535 (got 70000)",
		},
		{
			name: "sequence too long",
			doc: "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    type: sequence\n    volume: 50\n" +
				"    sequence:\n" + strings.Repeat("      - {freq: 440, duration_ms: 100}\n", maxNotes+1),
			want: "'sequence' must have 1..8 notes",
		},
		{
			name: "file with a path",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    file: sub/boot.mp3\n",
			want: "'file' must be a plain file name",
		},
		{
			name: "file without mp3 suffix",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    file: boot.wav\n",
			want: "'file' must end in .mp3",
		},
		{
			name: "mp3 without source",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n",
			want: "needs either a 'tts' text or a 'file'",
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			assertValidateError(t, loadFixture(t, c.doc), ids, c.want)
		})
	}
}

// assertValidateError checks that Validate reports want (a substring).
func assertValidateError(t *testing.T, cfg *Config, ids *SoundIDs, want string) {
	t.Helper()
	errs := cfg.Validate(ids)
	if len(errs) == 0 {
		t.Fatalf("no error reported, want %q", want)
	}
	if !strings.Contains(strings.Join(errs, "\n"), want) {
		t.Fatalf("errors do not contain %q:\n%s", want, strings.Join(errs, "\n"))
	}
}

func TestValidateRejectsBadTTSSettings(t *testing.T) {
	ids := sharedIDs(t)
	cases := []struct {
		name string
		doc  string
		want string
	}{
		{
			name: "tts on a tone",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_ping:\n    type: tone\n    volume: 50\n    tts: hello\n    tone: {freq: 440, duration_ms: 100}\n",
			want: "'tts' is only supported for type: mp3",
		},
		{
			name: "blank tts text",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    file: boot.mp3\n    tts: \"   \"\n",
			want: "'tts' must not be empty or whitespace only",
		},
		{
			name: "sid without tts",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    file: boot.mp3\n    sid: 3\n",
			want: "'sid'/'speed' only apply to a sound with a 'tts' text",
		},
		{
			name: "sid out of range",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n    sid: 12\n",
			want: "boot_complete: 'sid' must be 0..9 (got 12)",
		},
		{
			name: "speed out of range",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n    speed: 2.5\n",
			want: "'speed' must be 0.7..1.5",
		},
		{
			name: "missing language",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "'language' is required",
		},
		{
			name: "unsafe language",
			doc:  "version: 1\nlanguage: ../EN\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "must be lowercase letters, digits or '-'",
		},
		{
			name: "unknown engine",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  engine: espeak\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: `tts.engine "espeak" is not supported`,
		},
		{
			name: "piper without a voice",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  engine: piper\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "tts.piper.voice must not be empty",
		},
		{
			name: "legacy flat tts.sid",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  sid: 5\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "tts.sid is no longer supported, use tts.supertonic.sid",
		},
		{
			name: "piper sentence_silence out of range",
			doc: "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" +
				"tts:\n  engine: piper\n  piper:\n    voice: en_US-bryce-medium\n    sentence_silence: 9\n" +
				"sounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "sentence_silence must be 0..5 s",
		},
		{
			name: "invalid bitrate",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  bitrate: 320k\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "not a 16 kHz mono MP3 bitrate",
		},
		{
			name: "tts text too long",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: \"" + strings.Repeat("a", maxTTSText+1) + "\"\n",
			want: "the limit is 500",
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			assertValidateError(t, loadFixture(t, c.doc), ids, c.want)
		})
	}
}

func TestValidateRejectsDuplicateFile(t *testing.T) {
	doc := "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n" +
		"  success:\n    type: mp3\n    volume: 50\n    file: same.mp3\n" +
		"  warning:\n    type: mp3\n    volume: 50\n    file: same.mp3\n"
	assertValidateError(t, loadFixture(t, doc), sharedIDs(t), `'file' "same.mp3" is also used by "success"`)
}

func TestValidateRejectsLongFileName(t *testing.T) {
	doc := "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    type: mp3\n    volume: 50\n    file: " +
		strings.Repeat("a", MaxMP3NameLen) + ".mp3\n"
	assertValidateError(t, loadFixture(t, doc), sharedIDs(t), "the firmware limit is")
}
