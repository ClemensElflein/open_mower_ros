package fwsync

import (
	"bufio"
	"bytes"
	"fmt"
	"io"
	"os"
	"slices"
	"strconv"
	"strings"

	"github.com/hajimehoshi/go-mp3"
)

// MP3Info holds the result of an MP3 format check.
type MP3Info struct {
	SampleRate  int
	Channels    int  // 1 = mono, 2 = stereo
	BitrateKbps int  // of the first audio frame
	CBR         bool // every frame carries the same bitrate
	DurationMs  int
	Frames      int
}

// Is16kMono reports whether the MP3 meets the firmware requirement
// (16 kHz mono).
func (i MP3Info) Is16kMono() bool {
	return i.SampleRate == 16000 && i.Channels == 1
}

// HasBitrate reports whether the stream is CBR with the given bitrate.
func (i MP3Info) HasBitrate(kbps int) bool {
	return i.CBR && i.BitrateKbps == kbps
}

// String renders the format for logs and cache output.
func (i MP3Info) String() string {
	kind := "VBR"
	if i.CBR {
		kind = "CBR"
	}
	return fmt.Sprintf("%d Hz %d ch %s %d kbps %.2f s", i.SampleRate, i.Channels, kind, i.BitrateKbps, float64(i.DurationMs)/1000)
}

// mpeg2Layer3Bitrates are the bitrates a 16 kHz mono MPEG-2 Layer III stream can
// carry. The YAML value is validated against this list, so a bitrate that lame
// would refuse (e.g. 320k, which MPEG-2 does not define) fails at validate time
// instead of at render time.
var mpeg2Layer3Bitrates = []int{8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160}

// ParseBitrate parses a YAML bitrate ("64k", "64 kbps", "64") and validates it
// against the bitrates a 16 kHz mono MPEG-2 Layer III stream can use.
func ParseBitrate(s string) (int, error) {
	v := strings.ToLower(strings.TrimSpace(s))
	v = strings.TrimSuffix(strings.TrimSpace(v), "bps")
	v = strings.TrimSuffix(strings.TrimSpace(v), "k")
	n, err := strconv.Atoi(v)
	if err != nil {
		return 0, fmt.Errorf("invalid bitrate %q (use e.g. \"64k\")", s)
	}
	if !slices.Contains(mpeg2Layer3Bitrates, n) {
		return 0, fmt.Errorf("bitrate %q is not a 16 kHz mono MP3 bitrate (valid: %s)", s, BitrateList())
	}
	return n, nil
}

// FormatBitrate renders a bitrate the way the YAML writes it ("64k").
func FormatBitrate(kbps int) string { return strconv.Itoa(kbps) + "k" }

// BitrateList returns the valid bitrates as a human readable list.
func BitrateList() string {
	parts := make([]string, len(mpeg2Layer3Bitrates))
	for i, b := range mpeg2Layer3Bitrates {
		parts[i] = FormatBitrate(b)
	}
	return strings.Join(parts, ", ")
}

// CheckMP3 parses the MPEG frame chain (sample rate, channels, bitrate, CBR,
// duration) and cross-checks decodability with go-mp3.
func CheckMP3(path string) (MP3Info, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return MP3Info{}, err
	}
	info, err := ParseMP3(data)
	if err != nil {
		return MP3Info{}, err
	}

	dec, err := mp3.NewDecoder(bytes.NewReader(data))
	if err != nil {
		return MP3Info{}, fmt.Errorf("not a decodable MP3: %w", err)
	}
	if dec.SampleRate() != info.SampleRate {
		return MP3Info{}, fmt.Errorf("sample rate mismatch: header=%d go-mp3=%d", info.SampleRate, dec.SampleRate())
	}
	return info, nil
}

// ParseMP3 walks the whole MPEG frame chain of an MP3 file. Walking (instead of
// only reading the first frame) gives the duration that the sidecar records and
// a real CBR check — the firmware plays the stream without a bit reservoir, so
// we never ship VBR.
//
// The leading Xing/Info metadata frame is skipped: LAME writes one as the very
// first frame of every stream, encoded at a different bitrate and carrying no
// audio, so counting it would report every CBR file as VBR.
func ParseMP3(data []byte) (MP3Info, error) {
	var info MP3Info
	samples := 0
	junk := 0
	first := true

	off := id3v2Len(data)
	for off+4 <= len(data) {
		if len(data)-off >= 128 && string(data[off:off+3]) == "TAG" {
			break // ID3v1 tag
		}
		h, ok := decodeFrameHeader([4]byte(data[off : off+4]))
		if !ok {
			off++ // resync: tolerate junk between frames
			if junk++; junk > 1<<16 {
				return MP3Info{}, fmt.Errorf("no MPEG Layer III frame found")
			}
			continue
		}
		junk = 0
		if h.frameLen <= 0 || off+h.frameLen > len(data) {
			break // truncated tail
		}
		if info.SampleRate != 0 && h.sampleRate != info.SampleRate {
			return MP3Info{}, fmt.Errorf("sample rate changes at offset %d (%d -> %d)", off, info.SampleRate, h.sampleRate)
		}
		info.SampleRate = h.sampleRate
		if first && h.hasMetadataTag(data[off:off+h.frameLen]) {
			first = false
			off += h.frameLen
			continue
		}
		first = false
		if info.Frames == 0 {
			info.Channels, info.BitrateKbps = h.channels, h.bitrateKbps
			info.CBR = true
		} else if h.bitrateKbps != info.BitrateKbps || h.channels != info.Channels {
			info.CBR = false
		}
		info.Frames++
		samples += h.samples
		off += h.frameLen
	}
	if info.Frames == 0 {
		return MP3Info{}, fmt.Errorf("no MPEG Layer III frame found")
	}
	info.DurationMs = samples * 1000 / info.SampleRate
	return info, nil
}

// frameHeader is a decoded MPEG audio frame header.
type frameHeader struct {
	version     int // 3 = MPEG-1, 2 = MPEG-2, 0 = MPEG-2.5
	bitrateKbps int
	sampleRate  int
	channels    int
	frameLen    int // bytes, including padding
	samples     int // PCM samples per channel
}

// hasMetadataTag reports whether frame is a Xing/Info/VBRI metadata frame. LAME
// writes one as the first frame of every stream; it is encoded with a different
// bitrate and carries no audio, so it must not count towards the bitrate, the
// CBR flag or the duration.
func (h frameHeader) hasMetadataTag(frame []byte) bool {
	// The tag follows the header and the side info: 17 bytes in MPEG-1 mono,
	// 32 in MPEG-1 stereo, 9 in MPEG-2/2.5 mono and 17 in MPEG-2/2.5 stereo.
	side := 17
	switch {
	case h.version == 3 && h.channels == 2:
		side = 32
	case h.version != 3 && h.channels == 1:
		side = 9
	}
	if pos := 4 + side; pos+4 <= len(frame) {
		switch string(frame[pos : pos+4]) {
		case "Xing", "Info":
			return true
		}
	}
	// VBRI (Fraunhofer) always sits right after a 32 byte header block.
	return len(frame) >= 36 && string(frame[32:36]) == "VBRI"
}

// decodeFrameHeader decodes a 4-byte MPEG audio frame header. ok is false for
// anything that is not a valid MPEG-1/2/2.5 Layer III frame (the only format the
// firmware decodes).
func decodeFrameHeader(b [4]byte) (frameHeader, bool) {
	if b[0] != 0xFF || b[1]&0xE0 != 0xE0 {
		return frameHeader{}, false
	}
	// version: 0 = MPEG2.5, 1 = reserved, 2 = MPEG2, 3 = MPEG1.
	version := (b[1] >> 3) & 0x3
	// layer: 1 = Layer III, 2 = Layer II, 3 = Layer I, 0 = reserved.
	layer := (b[1] >> 1) & 0x3
	if version == 1 || layer != 1 {
		return frameHeader{}, false
	}
	bitrateIdx := (b[2] >> 4) & 0xF
	srIdx := (b[2] >> 2) & 0x3
	padding := int((b[2] >> 1) & 0x1)
	channelMode := (b[3] >> 6) & 0x3
	if bitrateIdx == 0 || bitrateIdx == 0xF || srIdx == 3 {
		return frameHeader{}, false
	}

	mpeg1 := version == 3
	var sampleRate int
	switch {
	case mpeg1:
		sampleRate = [4]int{44100, 48000, 32000, 0}[srIdx]
	case version == 2: // MPEG-2 LSF (16, 22.05, 24 kHz)
		sampleRate = [4]int{22050, 24000, 16000, 0}[srIdx]
	default: // MPEG-2.5 (8, 11.025, 12 kHz)
		sampleRate = [4]int{11025, 12000, 8000, 0}[srIdx]
	}
	if sampleRate == 0 {
		return frameHeader{}, false
	}

	var bitrateKbps, samples, factor int
	if mpeg1 {
		bitrateKbps = [...]int{0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0}[bitrateIdx]
		samples, factor = 1152, 144
	} else {
		bitrateKbps = [...]int{0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0}[bitrateIdx]
		samples, factor = 576, 72
	}
	if bitrateKbps == 0 {
		return frameHeader{}, false
	}

	channels := 2
	if channelMode == 3 { // 0b11 = mono
		channels = 1
	}
	frameLen := factor*bitrateKbps*1000/sampleRate + padding
	return frameHeader{version: int(version), bitrateKbps: bitrateKbps, sampleRate: sampleRate, channels: channels, frameLen: frameLen, samples: samples}, true
}

// id3v2Len returns the length of a leading ID3v2 tag (0 if there is none).
func id3v2Len(data []byte) int {
	if len(data) < 10 || string(data[:3]) != "ID3" {
		return 0
	}
	size := int(data[6]&0x7F)<<21 | int(data[7]&0x7F)<<14 | int(data[8]&0x7F)<<7 | int(data[9]&0x7F)
	total := 10 + size
	if total > len(data) {
		return len(data)
	}
	return total
}

// mpegFrameHeader locates the first MPEG audio frame and returns its sample
// rate and channel count.
func mpegFrameHeader(r io.Reader) (int, int, error) {
	br := bufio.NewReader(r)

	// Skip an optional ID3v2 tag.
	if head, _ := br.Peek(10); len(head) >= 10 && string(head[:3]) == "ID3" {
		size := int(head[6]&0x7F)<<21 | int(head[7]&0x7F)<<14 | int(head[8]&0x7F)<<7 | int(head[9]&0x7F)
		if _, err := br.Discard(10 + size); err != nil {
			return 0, 0, err
		}
	}

	// Scan for a valid frame header, one byte at a time.
	for {
		b, err := br.ReadByte()
		if err != nil {
			return 0, 0, fmt.Errorf("no MPEG frame found: %w", err)
		}
		if b != 0xFF {
			continue
		}
		rest, err := br.Peek(3)
		if err != nil {
			return 0, 0, fmt.Errorf("no MPEG frame found: %w", err)
		}
		if h, ok := decodeFrameHeader([4]byte{0xFF, rest[0], rest[1], rest[2]}); ok {
			return h.sampleRate, h.channels, nil
		}
	}
}
