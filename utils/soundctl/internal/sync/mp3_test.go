package sync

import (
	"bytes"
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
