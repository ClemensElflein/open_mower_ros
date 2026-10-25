package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
)

// probe returns sample rate (Hz) and duration (s) of an audio file.
func probe(path string) (int, float64, error) {
	out, err := exec.Command("ffprobe", "-v", "error",
		"-select_streams", "a:0",
		"-show_entries", "stream=sample_rate",
		"-show_entries", "format=duration",
		"-of", "default=nw=1:nk=1", path).Output()
	if err != nil {
		return 0, 0, fmt.Errorf("ffprobe: %v", err)
	}
	fields := strings.Fields(string(out))
	if len(fields) < 2 {
		return 0, 0, fmt.Errorf("ffprobe: unexpected output %q", string(out))
	}
	rate, err := strconv.Atoi(fields[0])
	if err != nil {
		return 0, 0, err
	}
	dur, err := strconv.ParseFloat(fields[1], 64)
	if err != nil {
		return 0, 0, err
	}
	return rate, dur, nil
}

// loudNorm enables EBU R128 loudness normalisation in the MP3 conversion. It is
// off by default (raw engine output, like the existing files), but should be on
// for an A/B: the engines differ by >5 dB in level, which biases the judgement.
var loudNorm bool

// mp3Bitrate is the constant (CBR) MPEG-2 Layer III bitrate for the firmware
// MP3s. The corpus in sounds/ is CBR at 16/32/64 kbps, the firmware decoder
// (dr_mp3) does not care about the bitrate as long as it is a valid MPEG-2
// Layer III one and the stream is 16 kHz mono. 32 kbps is the corpus majority.
var mp3Bitrate = "32k"

// toFirmwareMP3 converts any input to the 16 kHz mono MP3 that the firmware
// expects (the same format as the existing files in soundfiles/).
func toFirmwareMP3(in, out string) error {
	args := []string{"-hide_banner", "-loglevel", "error", "-y", "-i", in}
	if loudNorm {
		args = append(args, "-af", "loudnorm=I=-16:TP=-1.5:LRA=11")
	}
	args = append(args, "-ac", "1", "-ar", "16000", "-c:a", "libmp3lame", "-b:a", mp3Bitrate, out)
	if b, err := exec.Command("ffmpeg", args...).CombinedOutput(); err != nil {
		return fmt.Errorf("ffmpeg: %v: %s", err, tail(string(b), 300))
	}
	return nil
}

func rel(base, p string) string {
	r, err := filepath.Rel(base, p)
	if err != nil {
		return p
	}
	return r
}

func shortErr(err error) string {
	s := strings.ReplaceAll(err.Error(), "\n", " ")
	if len(s) > 160 {
		s = s[:160]
	}
	return s
}

func fail(err error) {
	fmt.Fprintln(os.Stderr, "error:", err)
	os.Exit(1)
}
