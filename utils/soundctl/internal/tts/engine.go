// Package tts renders the "tts:" texts of sounds_*.yaml into the 16 kHz mono
// MP3 files the firmware plays.
//
// It drives two external programs: the sherpa-onnx offline TTS CLI (Supertonic 3,
// engine "supertonic") and ffmpeg (loudness normalisation, resampling, MP3
// encoding). Both are baked into the image, so nothing is downloaded at runtime
// and the Go build stays CGO-free.
package tts

import (
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// DefaultDir is where the image ships the TTS assets (bin/, lib/, models/).
// It can be overridden with --tts-dir or $OPENMOWER_TTS_DIR.
const DefaultDir = "/opt/open_mower_ros/tts"

// defaultNumSteps is the flow-matching step count of Supertonic. 8 is what the
// model ships with: fewer steps get audibly rough, more only cost time.
const defaultNumSteps = 8

// Sherpa invokes the sherpa-onnx-offline-tts CLI.
type Sherpa struct {
	Bin      string // path of sherpa-onnx-offline-tts
	LibDir   string // LD_LIBRARY_PATH for the shared build ("" = static build)
	ModelDir string // model directory (tts.json, *.onnx, voice.bin)
	Threads  int    // ONNX threads (0 = CLI default)
	NumSteps int    // 0 = defaultNumSteps
}

// Available reports why the engine cannot be used (nil = it can).
func (s Sherpa) Available() error {
	if s.Bin == "" {
		return errors.New("no TTS engine binary configured (use --tts-bin)")
	}
	if _, err := os.Stat(s.Bin); err != nil {
		return fmt.Errorf("TTS engine binary: %w", err)
	}
	if s.ModelDir == "" {
		return errors.New("no TTS model directory configured (use --tts-model-dir)")
	}
	if _, err := os.Stat(filepath.Join(s.ModelDir, "tts.json")); err != nil {
		return fmt.Errorf("TTS model: %w", err)
	}
	return nil
}

// modelFile resolves a model file, preferring the int8 quantisation the image
// ships (smaller and faster on the CM4) but accepting a float model directory.
func (s Sherpa) modelFile(base string) (string, error) {
	for _, name := range []string{base + ".int8.onnx", base + ".onnx"} {
		p := filepath.Join(s.ModelDir, name)
		if _, err := os.Stat(p); err == nil {
			return p, nil
		}
	}
	return "", fmt.Errorf("%s: neither %s.int8.onnx nor %s.onnx found in %s", base, base, base, s.ModelDir)
}

// Args builds the CLI arguments for one synthesis call. It is exported for the
// engine test, which checks the argument set without a model on disk.
func (s Sherpa) Args(id fwsync.TTSIdentity, wav string) ([]string, error) {
	if id.Engine != "supertonic" {
		return nil, fmt.Errorf("unsupported tts engine %q (supported: supertonic)", id.Engine)
	}

	var args []string
	for _, part := range []struct{ flag, base string }{
		{"--supertonic-duration-predictor=", "duration_predictor"},
		{"--supertonic-text-encoder=", "text_encoder"},
		{"--supertonic-vector-estimator=", "vector_estimator"},
		{"--supertonic-vocoder=", "vocoder"},
	} {
		p, err := s.modelFile(part.base)
		if err != nil {
			return nil, err
		}
		args = append(args, part.flag+p)
	}
	for _, part := range []struct{ flag, name string }{
		{"--supertonic-tts-json=", "tts.json"},
		{"--supertonic-unicode-indexer=", "unicode_indexer.bin"},
		{"--supertonic-voice-style=", "voice.bin"},
	} {
		args = append(args, part.flag+filepath.Join(s.ModelDir, part.name))
	}

	steps := s.NumSteps
	if steps <= 0 {
		steps = defaultNumSteps
	}
	args = append(args,
		"--lang="+id.Language,
		fmt.Sprintf("--num-steps=%d", steps),
		fmt.Sprintf("--speed=%.2f", id.Speed),
		fmt.Sprintf("--sid=%d", id.Sid),
		"--output-filename="+wav,
	)
	if s.Threads > 0 {
		args = append(args, fmt.Sprintf("--num-threads=%d", s.Threads))
	}
	return append(args, id.Text), nil
}

// Engine renders a text into a WAV file; the encoder then turns that into the
// firmware MP3 (16 kHz mono CBR, EBU R128). Sherpa (Supertonic) is the engine the
// shipped definitions use; Piper is available for auditioning with
// `soundctl say --engine piper`.
type Engine interface {
	// Available reports why the engine cannot be used (nil = it can).
	Available() error
	// Synthesize writes the spoken text to wav (16-bit PCM, the voice's native
	// rate) and returns how long the engine took.
	Synthesize(id fwsync.TTSIdentity, wav string) (time.Duration, error)
	// Describe renders the voice/settings of one synthesis for the log line.
	Describe(id fwsync.TTSIdentity) string
}

// Describe renders the identity for the log line ("supertonic sid=5 1.00x 32k
// loudnorm de").
func (s Sherpa) Describe(id fwsync.TTSIdentity) string { return id.String() }

// runEngine runs an engine binary that writes wav and returns the wall time it
// took. stdin is fed to the process ("" = none). Shared by all engines so the
// "no output / empty output / non-zero exit" handling is identical.
func runEngine(bin string, args []string, env []string, stdin, wav string) (time.Duration, error) {
	_ = os.Remove(wav)

	cmd := exec.Command(bin, args...)
	if env != nil {
		cmd.Env = env
	}
	if stdin != "" {
		cmd.Stdin = strings.NewReader(stdin)
	}
	start := time.Now()
	out, err := cmd.CombinedOutput()
	wall := time.Since(start)
	if err != nil {
		return wall, fmt.Errorf("%s: %v: %s", filepath.Base(bin), err, tail(string(out), 500))
	}
	if st, err := os.Stat(wav); err != nil {
		return wall, fmt.Errorf("%s produced no output: %w", filepath.Base(bin), err)
	} else if st.Size() == 0 {
		return wall, fmt.Errorf("%s produced an empty WAV", filepath.Base(bin))
	}
	return wall, nil
}

// Synthesize writes the spoken text to wav (a 16-bit PCM WAV at the model's
// native rate) and returns how long the engine took.
func (s Sherpa) Synthesize(id fwsync.TTSIdentity, wav string) (time.Duration, error) {
	args, err := s.Args(id, wav)
	if err != nil {
		return 0, err
	}
	env := os.Environ()
	if s.LibDir != "" {
		env = append(env, "LD_LIBRARY_PATH="+s.LibDir)
	}
	return runEngine(s.Bin, args, env, "", wav)
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
