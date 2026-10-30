package tts

import (
	"encoding/json"
	"fmt"
	"math"
	"os/exec"
	"strconv"
	"strings"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// TargetSampleRate is the only rate the firmware's I2S output runs at; the MP3s
// must match it because the firmware does not resample.
const TargetSampleRate = 16000

// Loudness target. -16 LUFS / -1.5 dBTP / 11 LU is the EBU R128 streaming
// profile: the same level the rest of the corpus has, with a true-peak limit
// that leaves the amplifier headroom. Measured: raw Supertonic output is ~9 dB
// quieter and varies by up to 3 LU between sentences.
const (
	LoudnessTargetI   = -16.0
	LoudnessTargetTP  = -1.5
	LoudnessTargetLRA = 11.0
)

// Encoder turns raw engine output into the firmware MP3 (16 kHz mono CBR).
type Encoder struct {
	FFmpeg  string
	FFprobe string
}

// loudnormFilter is the filter chain of both loudnorm passes.
const loudnormFilter = "loudnorm=I=-16:TP=-1.5:LRA=11"

// loudnormStats is what the measuring pass reports back. ffmpeg prints the
// numbers as strings ("-24.70"), so they are kept verbatim.
type loudnormStats struct {
	InputI       string `json:"input_i"`
	InputTP      string `json:"input_tp"`
	InputLRA     string `json:"input_lra"`
	InputThresh  string `json:"input_thresh"`
	TargetOffset string `json:"target_offset"`
}

// valid reports whether the measurement can be used for a linear second pass.
// Silence (and near silence) reports "-inf", which would make the pass fail.
func (s loudnormStats) valid() bool {
	for _, v := range []string{s.InputI, s.InputTP, s.InputLRA, s.InputThresh} {
		f, err := strconv.ParseFloat(v, 64)
		if err != nil || math.IsInf(f, 0) || math.IsNaN(f) {
			return false
		}
	}
	return true
}

// applyFilter is the second-pass filter: the measured values plus linear=true,
// which applies one constant gain instead of the adaptive dynamics of the
// single-pass mode. On 2 s announcements the adaptive mode audibly pumps.
func (s loudnormStats) applyFilter() string {
	offset := s.TargetOffset
	if offset == "" {
		offset = "0.00"
	}
	return fmt.Sprintf("%s:measured_I=%s:measured_TP=%s:measured_LRA=%s:measured_thresh=%s:offset=%s:linear=true",
		loudnormFilter, s.InputI, s.InputTP, s.InputLRA, s.InputThresh, offset)
}

// Measure runs the loudness measuring pass (first of the two loudnorm passes).
func (e Encoder) Measure(in string) (loudnormStats, error) {
	out, err := exec.Command(e.ffmpeg(), "-hide_banner", "-nostats", "-i", in,
		"-af", loudnormFilter+":print_format=json", "-f", "null", "-").CombinedOutput()
	if err != nil {
		return loudnormStats{}, fmt.Errorf("ffmpeg (loudnorm measure): %v: %s", err, tail(string(out), 400))
	}
	raw := lastJSON(string(out))
	if raw == "" {
		return loudnormStats{}, fmt.Errorf("ffmpeg (loudnorm measure): no report in the output")
	}
	var st loudnormStats
	if err := json.Unmarshal([]byte(raw), &st); err != nil {
		return loudnormStats{}, fmt.Errorf("ffmpeg (loudnorm measure): %w", err)
	}
	return st, nil
}

// Encode converts in to a 16 kHz mono CBR MP3 at kbps. With loudNorm it runs the
// two-pass linear loudness normalisation described above; when the measuring
// pass fails (silence, unreadable file) it falls back to a plain encode instead
// of failing the whole run.
func (e Encoder) Encode(in, out string, kbps int, loudNorm bool) (warnings []string, err error) {
	args := []string{"-hide_banner", "-nostats", "-loglevel", "error", "-y", "-i", in}
	if loudNorm {
		if st, mErr := e.Measure(in); mErr != nil {
			warnings = append(warnings, mErr.Error())
		} else if !st.valid() {
			warnings = append(warnings, "loudness measurement unusable (input too quiet?) — encoding without normalisation")
		} else {
			args = append(args, "-af", st.applyFilter())
		}
	}
	args = append(args,
		"-ac", "1",
		"-ar", strconv.Itoa(TargetSampleRate),
		"-c:a", "libmp3lame",
		"-b:a", fwsync.FormatBitrate(kbps),
		// -f is explicit because the caller writes to a ".tmp" file that
		// ffmpeg would otherwise refuse to map onto a muxer.
		"-f", "mp3",
		out,
	)
	if b, err := exec.Command(e.ffmpeg(), args...).CombinedOutput(); err != nil {
		return warnings, fmt.Errorf("ffmpeg (encode): %v: %s", err, tail(string(b), 400))
	}
	return warnings, nil
}

// Available reports why the encoder cannot be used (nil = it can).
func (e Encoder) Available() error {
	if _, err := exec.LookPath(e.ffmpeg()); err != nil {
		return fmt.Errorf("ffmpeg not found: %w", err)
	}
	return nil
}

func (e Encoder) ffmpeg() string {
	if e.FFmpeg == "" {
		return "ffmpeg"
	}
	return e.FFmpeg
}

// lastJSON returns the last JSON object in s (ffmpeg interleaves its report with
// log lines).
func lastJSON(s string) string {
	end := strings.LastIndex(s, "}")
	if end < 0 {
		return ""
	}
	start := strings.LastIndex(s[:end], "{")
	if start < 0 {
		return ""
	}
	return s[start : end+1]
}
