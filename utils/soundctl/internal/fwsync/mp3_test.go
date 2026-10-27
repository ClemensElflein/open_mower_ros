package fwsync

import (
	"bytes"
	"slices"
	"testing"
)

func TestMPEGHeader16kMono(t *testing.T) {
	// MPEG2 Layer III, 16 kHz, mono.
	frame := []byte{0xFF, 0xF2, 0x88, 0xC0, 0, 0, 0, 0}
	rate, ch, err := mpegFrameHeader(bytes.NewReader(frame))
	if err != nil {
		t.Fatal(err)
	}
	if rate != 16000 {
		t.Fatalf("rate = %d, want 16000", rate)
	}
	if ch != 1 {
		t.Fatalf("channels = %d, want 1", ch)
	}
}

func TestMPEGHeader24kMono(t *testing.T) {
	// MPEG2 Layer III, 24 kHz, mono (matches the sample sounds).
	frame := []byte{0xFF, 0xF2, 0x84, 0xC0, 0, 0, 0, 0}
	rate, ch, err := mpegFrameHeader(bytes.NewReader(frame))
	if err != nil {
		t.Fatal(err)
	}
	if rate != 24000 {
		t.Fatalf("rate = %d, want 24000", rate)
	}
	if ch != 1 {
		t.Fatalf("channels = %d, want 1", ch)
	}
}

func TestMPEGHeaderStereo(t *testing.T) {
	// Same frame but channel mode = 00 (stereo).
	frame := []byte{0xFF, 0xF2, 0x88, 0x00, 0, 0, 0, 0}
	_, ch, err := mpegFrameHeader(bytes.NewReader(frame))
	if err != nil {
		t.Fatal(err)
	}
	if ch != 2 {
		t.Fatalf("channels = %d, want 2", ch)
	}
}

func TestMPEGHeaderSkipsID3v2(t *testing.T) {
	// 10-byte ID3v2 header with a 10-byte synchsafe size, then a 16 kHz mono frame.
	id3 := []byte{'I', 'D', '3', 4, 0, 0, 0, 0, 0, 10} // size=10, no extended header
	pad := make([]byte, 10)
	frame := []byte{0xFF, 0xF2, 0x88, 0xC0, 0, 0, 0, 0}
	data := append(append(append([]byte{}, id3...), pad...), frame...)

	rate, ch, err := mpegFrameHeader(bytes.NewReader(data))
	if err != nil {
		t.Fatal(err)
	}
	if rate != 16000 || ch != 1 {
		t.Fatalf("rate=%d ch=%d, want 16000/1", rate, ch)
	}
}

// synthFrame builds one synthetic MPEG Layer III frame: the header, an optional
// Xing/Info tag right after the side info and silence for the rest of the frame.
// version is 3 = MPEG-1, 2 = MPEG-2, 0 = MPEG-2.5; brIdx/srIdx are the raw
// bitrate/sample-rate table indices.
type synthFrame struct {
	version byte
	brIdx   byte
	srIdx   byte
	mono    bool
	tag     string // "Xing", "Info", "VBRI" or "" for a plain audio frame
}

func (s synthFrame) bytes() []byte {
	var bitrates []int
	var factor, rate int
	switch s.version {
	case 3:
		bitrates = []int{0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320}
		factor, rate = 144, [3]int{44100, 48000, 32000}[s.srIdx]
	default: // MPEG-2 and MPEG-2.5 share the factor and the bitrate table
		bitrates = []int{0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160}
		factor = 72
		if s.version == 2 {
			rate = [3]int{22050, 24000, 16000}[s.srIdx]
		} else {
			rate = [3]int{11025, 12000, 8000}[s.srIdx]
		}
	}
	n := factor * bitrates[s.brIdx] * 1000 / rate
	frame := make([]byte, n)
	frame[0] = 0xFF
	frame[1] = 0xE0 | s.version<<3 | 0x02 | 0x01 // sync, version, Layer III, no CRC
	frame[2] = s.brIdx<<4 | s.srIdx<<2           // bitrate, sample rate, no padding
	if s.mono {
		frame[3] = 0xC0
	}
	if s.tag != "" {
		side := 17 // MPEG-1 mono
		switch {
		case s.version == 3 && !s.mono:
			side = 32
		case s.version != 3 && s.mono:
			side = 9
		case s.version != 3 && !s.mono:
			side = 17
		}
		copy(frame[4+side:], s.tag)
	}
	return frame
}

// mpeg2Frame16k is the common case: a 16 kHz mono MPEG-2 Layer III frame.
func mpeg2Frame16k(bitrateKbps int, tag string) synthFrame {
	// encodeFrameHeader's MPEG-2/2.5 bitrate table starts with the "free"
	// index 0, so the table index is the position in mpeg2Layer3Bitrates + 1.
	idx := byte(slices.Index(mpeg2Layer3Bitrates, bitrateKbps) + 1)
	return synthFrame{version: 2, brIdx: idx, srIdx: 2, mono: true, tag: tag}
}

func joinFrames(frames ...[]byte) []byte {
	var out []byte
	for _, f := range frames {
		out = append(out, f...)
	}
	return out
}

// LAME writes a Xing/Info metadata frame as the very first frame of every
// stream, encoded at a different bitrate and carrying no audio. Counting it made
// every CBR file report as VBR - the bug this test pins down.
func TestParseMP3SkipsLeadingInfoFrame(t *testing.T) {
	// Info frame at 24 kbps, then four 32 kbps audio frames.
	data := joinFrames(
		mpeg2Frame16k(24, "Info").bytes(),
		mpeg2Frame16k(32, "").bytes(),
		mpeg2Frame16k(32, "").bytes(),
		mpeg2Frame16k(32, "").bytes(),
		mpeg2Frame16k(32, "").bytes(),
	)
	info, err := ParseMP3(data)
	if err != nil {
		t.Fatal(err)
	}
	if !info.CBR {
		t.Fatalf("CBR = false, want true (got %s)", info)
	}
	if info.BitrateKbps != 32 {
		t.Fatalf("bitrate = %d, want 32", info.BitrateKbps)
	}
	if info.Frames != 4 {
		t.Fatalf("frames = %d, want 4 (the metadata frame must not count)", info.Frames)
	}
	if info.SampleRate != 16000 || info.Channels != 1 {
		t.Fatalf("rate/ch = %d/%d, want 16000/1", info.SampleRate, info.Channels)
	}
	// 4 frames * 576 samples = 2304 samples = 144 ms at 16 kHz.
	if info.DurationMs != 144 {
		t.Fatalf("duration = %d ms, want 144", info.DurationMs)
	}
}

// The Xing tag sits after the side info, whose length depends on the MPEG
// version and the channel count - MPEG-1 mono is 17 bytes, not 9.
func TestParseMP3SkipsLeadingXingFrameMPEG1Mono(t *testing.T) {
	tag := synthFrame{version: 3, brIdx: 5, srIdx: 0, mono: true, tag: "Xing"} // 64 kbps 44.1 kHz
	frame := synthFrame{version: 3, brIdx: 9, srIdx: 0, mono: true}            // 128 kbps 44.1 kHz
	info, err := ParseMP3(joinFrames(tag.bytes(), frame.bytes(), frame.bytes()))
	if err != nil {
		t.Fatal(err)
	}
	if !info.CBR || info.BitrateKbps != 128 {
		t.Fatalf("got %s, want CBR 128 kbps", info)
	}
	if info.Frames != 2 {
		t.Fatalf("frames = %d, want 2", info.Frames)
	}
	if info.SampleRate != 44100 || info.Channels != 1 {
		t.Fatalf("rate/ch = %d/%d, want 44100/1", info.SampleRate, info.Channels)
	}
}

func TestParseMP3DetectsVBR(t *testing.T) {
	data := joinFrames(
		mpeg2Frame16k(32, "").bytes(),
		mpeg2Frame16k(64, "").bytes(),
		mpeg2Frame16k(32, "").bytes(),
	)
	info, err := ParseMP3(data)
	if err != nil {
		t.Fatal(err)
	}
	if info.CBR {
		t.Fatalf("CBR = true for a VBR stream (%s)", info)
	}
	if info.Frames != 3 || info.BitrateKbps != 32 {
		t.Fatalf("frames/bitrate = %d/%d, want 3/32 (first audio frame)", info.Frames, info.BitrateKbps)
	}
}

func TestParseMP3ToleratesTruncatedTail(t *testing.T) {
	frame := mpeg2Frame16k(32, "").bytes()
	data := joinFrames(frame, frame, frame[:len(frame)/2]) // last frame cut in half
	info, err := ParseMP3(data)
	if err != nil {
		t.Fatal(err)
	}
	if info.Frames != 2 || !info.CBR {
		t.Fatalf("got %s frames=%d, want 2 CBR frames", info, info.Frames)
	}
}

func TestParseMP3MPEG25SampleRate(t *testing.T) {
	// 8 kHz is MPEG-2.5, which the firmware can decode as well.
	frame := synthFrame{version: 0, brIdx: 4, srIdx: 2, mono: true}.bytes() // 32 kbps 8 kHz
	info, err := ParseMP3(joinFrames(frame, frame))
	if err != nil {
		t.Fatal(err)
	}
	if info.SampleRate != 8000 || info.Channels != 1 {
		t.Fatalf("rate/ch = %d/%d, want 8000/1", info.SampleRate, info.Channels)
	}
	if info.Is16kMono() {
		t.Fatal("Is16kMono() = true for an 8 kHz file")
	}
}

func TestParseMP3NoFrame(t *testing.T) {
	if _, err := ParseMP3(bytes.Repeat([]byte{0x00}, 4096)); err == nil {
		t.Fatal("ParseMP3 accepted 4 KiB of zeros")
	}
}
