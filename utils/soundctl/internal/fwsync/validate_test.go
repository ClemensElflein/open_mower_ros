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

// validTTS is the minimal valid tts block of the fixture below.
const validTTS = "tts:\n  engine: elevenlabs\n  elevenlabs:\n    voice: 5gaEGrWSk4v20HMfIhgC\n"

// validBase is a minimal valid definition; the cases below replace one part of
// it to isolate a single rule.
const validBase = "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" +
	validTTS +
	"sounds:\n  boot_complete:\n    type: mp3\n    volume: 80\n    variants:\n      - \"Robot boot complete\"\n"

// TestValidBaseIsAccepted keeps the minimal fixture honest: it has to validate,
// otherwise the "rejects …" cases below could pass for the wrong reason.
func TestValidBaseIsAccepted(t *testing.T) {
	if errs := loadFixture(t, validBase).Validate(sharedIDs(t)); len(errs) > 0 {
		t.Fatalf("the minimal fixture is not valid: %v", errs)
	}
}

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

// TestNormalizeDerivesVariantFiles pins the file naming: a tts variant without a
// file name becomes <language>_<sound>_<n>.mp3, and the first variant is what the
// current firmware plays.
func TestNormalizeDerivesVariantFiles(t *testing.T) {
	doc := "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" + validTTS +
		"sounds:\n  boot_complete:\n    type: mp3\n    volume: 80\n    variants:\n" +
		"      - \"one\"\n      - \"two\"\n      - {file: zz_custom.mp3}\n"
	cfg := loadFixture(t, doc)

	want := []string{"en_boot_complete_1.mp3", "en_boot_complete_2.mp3", "zz_custom.mp3"}
	got := cfg.Files()
	if strings.Join(got, ",") != strings.Join(want, ",") {
		t.Fatalf("Files() = %v, want %v", got, want)
	}
	if first := cfg.FileName("boot_complete"); first != want[0] {
		t.Errorf("FileName() = %q, want the first variant %q", first, want[0])
	}
	jobs := cfg.RenderJobs()
	if len(jobs) != 2 {
		t.Fatalf("RenderJobs() = %d jobs, want 2 (the file variant needs no render)", len(jobs))
	}
	if jobs[1].Label != "boot_complete[2]" || jobs[1].Identity.Text != "two" {
		t.Errorf("second job = %+v, want the second tts variant", jobs[1])
	}
	if errs := cfg.Validate(sharedIDs(t)); len(errs) > 0 {
		t.Fatalf("valid definition rejected: %v", errs)
	}
}

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

func TestValidateRejectsContractViolations(t *testing.T) {
	ids := sharedIDs(t)
	tone := "type: tone\n    volume: 50\n    tone: {freq: 440, duration_ms: 100}\n"
	cases := []struct {
		name string
		doc  string
		want string
	}{
		{
			name: "unknown sound id",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  nonsense:\n    " + tone,
			want: `unknown sound id "nonsense"`,
		},
		{
			name: "relative sound_path",
			doc:  "version: 1\nsound_path: sounds\nsounds:\n  success:\n    " + tone,
			want: "must be an absolute path",
		},
		{
			name: "wrong version",
			doc:  "version: 2\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    " + tone,
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
			name: "mp3 without variants",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n",
			want: "mp3 entry needs 'variants'",
		},
		{
			name: "legacy file at sound level",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    file: zz_ping.mp3\n",
			want: "'file'/'tts' at sound level was replaced by 'variants'",
		},
		{
			name: "legacy tts at sound level",
			doc: "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" + validTTS +
				"sounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    tts: hello\n",
			want: "'file'/'tts' at sound level was replaced by 'variants'",
		},
		{
			name: "variants on a tone",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    " + tone + "    variants:\n      - \"nope\"\n",
			want: "'variants' only apply to type: mp3",
		},
		{
			name: "empty variant",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    variants:\n      - {}\n",
			want: "needs a 'tts' text or a 'file'",
		},
		{
			name: "invalid pick",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    pick: sometimes\n    variants:\n      - {file: zz_ping.mp3}\n",
			want: "'pick' must be",
		},
		{
			name: "invalid every",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    every: soon\n    variants:\n      - {file: zz_ping.mp3}\n",
			want: "'every' is not a duration",
		},
		{
			name: "every too short",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    every: 100ms\n    variants:\n      - {file: zz_ping.mp3}\n",
			want: "'every' must be at least 1s",
		},
		{
			name: "chance without every",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    chance: 0.5\n    variants:\n      - {file: zz_ping.mp3}\n",
			want: "'chance' needs 'every'",
		},
		{
			name: "chance out of range",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    every: 60s\n    chance: 2\n    variants:\n      - {file: zz_ping.mp3}\n",
			want: "'chance' must be 0..1",
		},
		{
			name: "missing language",
			doc:  "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    variants:\n      - \"hello\"\n",
			want: "'language' is required",
		},
		{
			name: "unsafe language",
			doc:  "version: 1\nlanguage: ../EN\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    variants:\n      - \"hello\"\n",
			want: "must be lowercase letters, digits or '-'",
		},
		{
			name: "tts text too long",
			doc: "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" + validTTS +
				"sounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    variants:\n      - \"" + strings.Repeat("a", maxTTSText+1) + "\"\n",
			want: "the limit is 500",
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			assertValidateError(t, loadFixture(t, c.doc), ids, c.want)
		})
	}
}

func TestValidateRejectsBadTTSSettings(t *testing.T) {
	ids := sharedIDs(t)
	sound := "sounds:\n  boot_complete:\n    type: mp3\n    volume: 50\n    variants:\n      - \"hello\"\n"
	cases := []struct {
		name string
		doc  string
		want string
	}{
		{
			name: "unknown engine",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  engine: espeak\n" + sound,
			want: `tts.engine "espeak" is not supported`,
		},
		{
			name: "missing voice",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  engine: elevenlabs\n" + sound,
			want: "tts.elevenlabs.voice must not be empty",
		},
		{
			name: "stability out of range",
			doc: "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" +
				"tts:\n  engine: elevenlabs\n  elevenlabs:\n    voice: abc\n    stability: 1.5\n" + sound,
			want: "tts.elevenlabs.stability must be 0..1",
		},
		{
			name: "invalid bitrate",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  bitrate: 320k\n  elevenlabs:\n    voice: abc\n" + sound,
			want: "not a 16 kHz mono MP3 bitrate",
		},
		{
			name: "removed piper engine",
			doc: "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" +
				"tts:\n  engine: elevenlabs\n  elevenlabs:\n    voice: abc\n  piper:\n    voice: en_US-bryce-medium\n" + sound,
			want: "the 'piper' engine was removed",
		},
		{
			name: "removed supertonic engine",
			doc: "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\n" +
				"tts:\n  engine: elevenlabs\n  elevenlabs:\n    voice: abc\n  supertonic:\n    sid: 5\n" + sound,
			want: "the 'supertonic' engine was removed",
		},
		{
			name: "removed speed",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  speed: 1.2\n  elevenlabs:\n    voice: abc\n" + sound,
			want: "tts.speed was removed",
		},
		{
			name: "removed sid",
			doc:  "version: 1\nlanguage: en\nsound_path: /opt/open_mower_ros/sounds\ntts:\n  sid: 5\n  elevenlabs:\n    voice: abc\n" + sound,
			want: "tts.sid was removed",
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
		"  success:\n    type: mp3\n    volume: 50\n    variants:\n      - {file: same.mp3}\n" +
		"  warning:\n    type: mp3\n    volume: 50\n    variants:\n      - {file: same.mp3}\n"
	assertValidateError(t, loadFixture(t, doc), sharedIDs(t), `file "same.mp3" is also used by "success"`)
}

func TestValidateRejectsLongFileName(t *testing.T) {
	doc := "version: 1\nsound_path: /opt/open_mower_ros/sounds\nsounds:\n  success:\n    type: mp3\n    volume: 50\n    variants:\n      - {file: " +
		strings.Repeat("a", MaxMP3NameLen) + ".mp3}\n"
	assertValidateError(t, loadFixture(t, doc), sharedIDs(t), "the firmware limit is")
}
