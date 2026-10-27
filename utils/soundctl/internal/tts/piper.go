package tts

import (
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// Piper drives the piper CLI (OHF-Voice/piper1-gpl): one VITS .onnx per voice with
// espeak-ng embedded for phonemisation — which is what makes German work (the
// sherpa-onnx CLI we use for Supertonic dropped espeak-ng in v2.0.0).
//
// It is wired up for auditioning only (`soundctl say --engine piper`); the shipped
// definitions still render with Supertonic, so nothing here touches the render
// cache or the sidecars.
type Piper struct {
	// Bin is the piper executable: the standalone C++ binary from the 2023
	// release (no Python, fastest) or "piper" from `pip install piper-tts`.
	Bin string
	// VoiceDir is where a bare Voice name is resolved to "<dir>/<voice>.onnx".
	// The resolved path is passed as -m, because the standalone binary has no
	// --data-dir; that keeps one argument set valid for both binaries.
	VoiceDir string
	// Voice is a voice name ("en_US-bryce-medium") or a path to a .onnx file.
	Voice string
	// SentenceSilence is passed as --sentence_silence on every call: the
	// standalone binary defaults to 0.2 s, the piper1-gpl CLI to 0, and both
	// spellings differ, so being explicit is what makes the two comparable.
	SentenceSilence float64
}

// VoicePath resolves the configured voice to the .onnx path handed to piper.
func (p Piper) VoicePath() string {
	if p.Voice == "" {
		return ""
	}
	if filepath.IsAbs(p.Voice) || strings.ContainsRune(p.Voice, os.PathSeparator) {
		return p.Voice
	}
	name := p.Voice
	if !strings.HasSuffix(name, ".onnx") {
		name += ".onnx"
	}
	if p.VoiceDir == "" {
		return name
	}
	return filepath.Join(p.VoiceDir, name)
}

// Available reports why piper cannot be used (nil = it can).
func (p Piper) Available() error {
	if p.Bin == "" {
		return errors.New("no piper binary configured (use --piper-bin)")
	}
	if _, err := exec.LookPath(p.Bin); err != nil {
		return fmt.Errorf("piper binary %q not found (install it with \"pip install piper-tts\" or "+
			"\"python3 -m pip install piper-tts\", or pass --piper-bin)", p.Bin)
	}
	if p.Voice == "" {
		return errors.New("no piper voice configured (use --voice, e.g. en_US-bryce-medium)")
	}
	if path := p.VoicePath(); path != "" {
		if st, err := os.Stat(path); err != nil || st.IsDir() {
			return fmt.Errorf("piper voice not found at %s (download it with "+
				"\"python3 -m piper.download_voices <voice> --data-dir <dir>\", or pass --voice-dir)", path)
		}
	}
	return nil
}

// Args builds the piper command line. Pure, so the mapping (including the
// speed → length_scale inversion) is testable without a voice on disk. Only
// options both piper generations understand are used: -m/-f/-s and the
// underscore spellings of --sentence_silence/--length_scale.
//
// id.Sid selects the speaker of a multi-speaker voice (-s); 0 means "not set",
// which is what single-speaker voices need.
func (p Piper) Args(id fwsync.TTSIdentity, wav string) []string {
	args := []string{"-m", p.VoicePath(), "-f", wav}
	if id.Sid > 0 {
		args = append(args, "-s", strconv.Itoa(id.Sid))
	}
	args = append(args, "--sentence_silence", strconv.FormatFloat(p.SentenceSilence, 'f', 2, 64))
	// piper's length_scale multiplies the phoneme duration, i.e. it is the
	// inverse of our "speed" (2.0 = twice as long, 1.0 = unchanged).
	if id.Speed > 0 && id.Speed != 1.0 {
		args = append(args, "--length_scale", strconv.FormatFloat(1/id.Speed, 'f', 3, 64))
	}
	// The text is *not* part of the command line: the standalone binary only
	// reads stdin, the piper1-gpl CLI reads stdin as well when no positional text
	// is given — so Synthesize feeds it there (which also survives a text that
	// starts with "-").
	return args
}

// Synthesize writes the spoken text to wav (16-bit PCM at the voice's native rate,
// 22.05 kHz for the medium/high voices) and returns how long piper took. The text
// goes through stdin, which both the standalone binary and the piper1-gpl CLI
// read (see Args).
func (p Piper) Synthesize(id fwsync.TTSIdentity, wav string) (time.Duration, error) {
	if err := p.Available(); err != nil {
		return 0, err
	}
	return runEngine(p.Bin, p.Args(id, wav), nil, id.Text, wav)
}

// Describe renders the voice and settings for the log line ("piper
// de_DE-thorsten-high 1.00x 32k loudnorm").
func (p Piper) Describe(id fwsync.TTSIdentity) string {
	loud := "raw"
	if id.LoudNorm {
		loud = "loudnorm"
	}
	speed := id.Speed
	if speed <= 0 {
		speed = 1
	}
	return fmt.Sprintf("piper %s %.2fx %s %s", p.Voice, speed, id.Bitrate, loud)
}

// VoiceLanguage derives the language from a piper voice name ("de_DE-thorsten-high"
// → "de"), so log lines and the identity say what is being spoken. It returns ""
// for a bare path (there the language is whatever the voice was trained on).
func VoiceLanguage(voice string) string {
	base := filepath.Base(strings.TrimSuffix(voice, ".onnx"))
	if i := strings.Index(base, "_"); i > 0 {
		lang := base[:i]
		if errs := fwsync.ValidateLanguage(lang); len(errs) == 0 {
			return lang
		}
	}
	return ""
}
