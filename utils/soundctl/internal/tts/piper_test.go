package tts

import (
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// TestPiperArgs pins the flag mapping, above all the speed → length_scale
// inversion (piper's length_scale is the reciprocal of our "speed") and the
// "--" guard that keeps a text starting with "-" from being read as a flag.
func TestPiperArgs(t *testing.T) {
	id := fwsync.TTSIdentity{Text: "Hallo Welt.", Speed: 1.0}

	cases := []struct {
		name  string
		piper Piper
		speed float64
		sid   int
		text  string
		want  []string
	}{
		{
			name:  "minimal",
			piper: Piper{Bin: "piper", Voice: "de_DE-thorsten-high"},
			speed: 1.0,
			text:  "Hallo Welt.",
			want:  []string{"-m", "de_DE-thorsten-high.onnx", "-f", "/tmp/x.wav", "--sentence_silence", "0.00"},
		},
		{
			name:  "voice dir and faster",
			piper: Piper{Bin: "piper", Voice: "de_DE-thorsten-high", VoiceDir: "/voices"},
			speed: 1.25,
			text:  "Schneller.",
			want: []string{"-m", "/voices/de_DE-thorsten-high.onnx", "-f", "/tmp/x.wav",
				"--sentence_silence", "0.00", "--length_scale", "0.800"},
		},
		{
			name:  "slower, multi-speaker voice and sentence silence",
			piper: Piper{Bin: "piper", Voice: "en_US-libritts_r-medium", SentenceSilence: 0.2},
			speed: 0.8,
			sid:   3,
			text:  "Slow.",
			want: []string{"-m", "en_US-libritts_r-medium.onnx", "-f", "/tmp/x.wav", "-s", "3",
				"--sentence_silence", "0.20", "--length_scale", "1.250"},
		},
		{
			name:  "absolute voice path",
			piper: Piper{Bin: "piper", Voice: "/voices/x.onnx"},
			speed: 1.0,
			text:  "- Test",
			want:  []string{"-m", "/voices/x.onnx", "-f", "/tmp/x.wav", "--sentence_silence", "0.00"},
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			id.Speed = c.speed
			id.Text = c.text
			id.Sid = c.sid
			got := c.piper.Args(id, "/tmp/x.wav")
			if strings.Join(got, " ") != strings.Join(c.want, " ") {
				t.Errorf("Args() =\n  %v\nwant\n  %v", got, c.want)
			}
		})
	}
}

// TestPiperAvailable pins the three things a user gets wrong first: no binary on
// PATH, no voice, and a voice name that does not resolve to a file.
func TestPiperAvailable(t *testing.T) {
	if err := (Piper{Bin: "definitely-not-a-piper-binary", Voice: "en_US-bryce-medium"}).Available(); err == nil {
		t.Error("missing binary must be reported")
	} else if !strings.Contains(err.Error(), "pip install piper-tts") {
		t.Errorf("error %q should say how to install piper", err)
	}
	if err := (Piper{Bin: "sh"}).Available(); err == nil || !strings.Contains(err.Error(), "--voice") {
		t.Errorf("missing voice: got %v, want a --voice hint", err)
	}
	if err := (Piper{Bin: "sh", Voice: "does-not-exist", VoiceDir: t.TempDir()}).Available(); err == nil ||
		!strings.Contains(err.Error(), "piper voice not found") {
		t.Errorf("unresolvable voice: got %v, want a \"piper voice not found\" hint", err)
	}

	// A voice name below a directory is resolved to <dir>/<name>.onnx.
	dir := t.TempDir()
	voice := filepath.Join(dir, "en_US-bryce-medium.onnx")
	if err := os.WriteFile(voice, []byte("not a real model"), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := (Piper{Bin: "sh", Voice: "en_US-bryce-medium", VoiceDir: dir}).Available(); err != nil {
		t.Errorf("Available() with a real binary and a voice: %v", err)
	}
}

// TestPiperVoicePath pins the name → path resolution (that is what lets one
// argument set work for both piper generations, see Args).
func TestPiperVoicePath(t *testing.T) {
	cases := []struct {
		piper Piper
		want  string
	}{
		{Piper{Voice: "en_US-bryce-medium", VoiceDir: "/v"}, "/v/en_US-bryce-medium.onnx"},
		{Piper{Voice: "en_US-bryce-medium.onnx", VoiceDir: "/v"}, "/v/en_US-bryce-medium.onnx"},
		{Piper{Voice: "/abs/x.onnx", VoiceDir: "/v"}, "/abs/x.onnx"},
		{Piper{Voice: "sub/x.onnx"}, "sub/x.onnx"},
		{Piper{Voice: "en_US-bryce-medium"}, "en_US-bryce-medium.onnx"},
		{Piper{}, ""},
	}
	for _, c := range cases {
		if got := c.piper.VoicePath(); got != c.want {
			t.Errorf("VoicePath(%+v) = %q, want %q", c.piper, got, c.want)
		}
	}
}

// TestVoiceLanguage: the voice name carries the language, a bare path does not.
func TestVoiceLanguage(t *testing.T) {
	cases := map[string]string{
		"de_DE-thorsten-high":            "de",
		"en_GB-alba-medium":              "en",
		"en_US-ryan-high.onnx":           "en",
		"/voices/de_DE-eva_k-x_low.onnx": "de",
		"/voices/random.onnx":            "",
		"novoice":                        "",
	}
	for voice, want := range cases {
		if got := VoiceLanguage(voice); got != want {
			t.Errorf("VoiceLanguage(%q) = %q, want %q", voice, got, want)
		}
	}
}

// TestPiperWithBinary runs the real chain when a piper binary and a voice are
// pointed at, e.g.
//
//	OPENMOWER_PIPER_BIN=/path/to/piper OPENMOWER_PIPER_VOICE=de_DE-thorsten-high \
//	OPENMOWER_PIPER_DIR=/path/to/voices go test ./internal/tts -run PiperWith -v
func TestPiperWithBinary(t *testing.T) {
	bin := os.Getenv("OPENMOWER_PIPER_BIN")
	voice := os.Getenv("OPENMOWER_PIPER_VOICE")
	if bin == "" || voice == "" {
		t.Skip("OPENMOWER_PIPER_BIN/OPENMOWER_PIPER_VOICE not set")
	}
	enc := Encoder{FFmpeg: "ffmpeg"}
	if err := enc.Available(); err != nil {
		t.Skipf("ffmpeg not available: %v", err)
	}
	eng := Piper{Bin: bin, Voice: voice, VoiceDir: os.Getenv("OPENMOWER_PIPER_DIR")}
	if err := eng.Available(); err != nil {
		t.Skipf("piper not usable: %v", err)
	}

	dst := filepath.Join(t.TempDir(), "say.mp3")
	id := fwsync.TTSIdentity{
		Language: VoiceLanguage(voice),
		Bitrate:  "32k",
		Speed:    1.0,
		LoudNorm: true,
	}
	res, err := RenderText("Guten Tag.", id, dst, eng, enc, t.Logf)
	if err != nil {
		t.Fatalf("RenderText via piper: %v", err)
	}
	if !res.Info.Is16kMono() {
		t.Errorf("rendered %s, want 16 kHz mono", res.Info)
	}
}
