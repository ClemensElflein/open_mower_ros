package sync

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"os"

	"github.com/hajimehoshi/go-mp3"
)

// MP3Info holds the result of an MP3 format check.
type MP3Info struct {
	SampleRate int
	Channels   int // 1 = mono, 2 = stereo
}

// Is16kMono reports whether the MP3 meets the firmware requirement
// (16 kHz mono).
func (i MP3Info) Is16kMono() bool {
	return i.SampleRate == 16000 && i.Channels == 1
}

// CheckMP3 parses the first MPEG frame header (sample rate + channel mode)
// and cross-checks decodability with go-mp3.
func CheckMP3(path string) (MP3Info, error) {
	f, err := os.Open(path)
	if err != nil {
		return MP3Info{}, err
	}
	defer f.Close()

	sampleRate, channels, err := mpegFrameHeader(f)
	if err != nil {
		return MP3Info{}, err
	}

	if _, err := f.Seek(0, io.SeekStart); err != nil {
		return MP3Info{}, err
	}
	dec, err := mp3.NewDecoder(f)
	if err != nil {
		return MP3Info{}, fmt.Errorf("not a decodable MP3: %w", err)
	}
	if dec.SampleRate() != sampleRate {
		return MP3Info{}, fmt.Errorf("sample rate mismatch: header=%d go-mp3=%d", sampleRate, dec.SampleRate())
	}

	return MP3Info{SampleRate: sampleRate, Channels: channels}, nil
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

	// Scan for the 11-bit frame sync (0xFF followed by 0b111xxxxx).
	var hdr [4]byte
	for {
		b, err := br.ReadByte()
		if err != nil {
			return 0, 0, fmt.Errorf("no MPEG frame found: %w", err)
		}
		if b != 0xFF {
			continue
		}
		b2, err := br.ReadByte()
		if err != nil {
			return 0, 0, fmt.Errorf("no MPEG frame found: %w", err)
		}
		if b2&0xE0 != 0xE0 {
			continue
		}
		hdr[0], hdr[1] = 0xFF, b2
		if _, err := io.ReadFull(br, hdr[2:]); err != nil {
			return 0, 0, fmt.Errorf("no MPEG frame found: %w", err)
		}
		break
	}

	v := binary.BigEndian.Uint32(hdr[:])
	version := (v >> 19) & 0x3
	srIdx := (v >> 10) & 0x3
	channelMode := (v >> 6) & 0x3

	sampleRates := map[uint32][4]int{
		3: {44100, 48000, 32000, 0}, // MPEG1
		2: {22050, 24000, 16000, 0}, // MPEG2
		0: {11025, 12000, 8000, 0},  // MPEG2.5
	}
	rates, ok := sampleRates[version]
	if !ok || srIdx >= 3 || rates[srIdx] == 0 {
		return 0, 0, fmt.Errorf("reserved/unsupported MPEG version %d sample rate index %d", version, srIdx)
	}

	channels := 2
	if channelMode == 3 { // 0b11 = mono
		channels = 1
	}
	return rates[srIdx], channels, nil
}
