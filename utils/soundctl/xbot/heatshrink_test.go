package xbot

import (
	"bytes"
	"testing"
)

// hsDecode mirrors the firmware's heatshrink decoder (literals + backrefs,
// MSB-first bits, window = 512). It exists only to verify the encoder in tests.
func hsDecode(data []byte) []byte {
	var out []byte
	window := make([]byte, hsWindowSize)
	head := 0

	bitPos := 0
	readBits := func(count int) (uint16, bool) {
		if bitPos+count > len(data)*8 {
			return 0, false
		}
		var v uint16
		for i := 0; i < count; i++ {
			b := data[bitPos>>3]
			shift := uint(7 - (bitPos & 7))
			v = (v << 1) | uint16((b>>shift)&1)
			bitPos++
		}
		return v, true
	}

	for {
		tag, ok := readBits(1)
		if !ok {
			break
		}
		if tag == hsLiteralMarker {
			b, ok := readBits(8)
			if !ok {
				break
			}
			c := byte(b)
			window[head&(hsWindowSize-1)] = c
			head++
			out = append(out, c)
			continue
		}
		// Backref; a trailing partial token (zero padding) is ignored.
		off, ok := readBits(hsWindowBits)
		if !ok {
			break
		}
		cnt, ok := readBits(hsLookaheadBits)
		if !ok {
			break
		}
		offset := int(off) + 1
		length := int(cnt) + 1
		for i := 0; i < length; i++ {
			c := window[(head-offset)&(hsWindowSize-1)]
			window[head&(hsWindowSize-1)] = c
			head++
			out = append(out, c)
		}
	}
	return out
}

func TestHeatshrinkRoundTrip(t *testing.T) {
	cases := [][]byte{
		nil,
		{},
		[]byte("a"),
		[]byte("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"),
		[]byte("abcabcabcabcabcabc"),
		[]byte("boot_ping boot_complete boot_ping boot_complete"),
		[]byte(`{"sounds":{"boot_complete":{"type":"mp3","volume":100,"file":"en_hi-i-am-steve.mp3"}}}`),
	}
	for i, c := range cases {
		enc := HeatshrinkEncode(c)
		if dec := hsDecode(enc); !bytes.Equal(c, dec) {
			t.Fatalf("case %d: round-trip mismatch\n in: %q\nout: %q", i, c, dec)
		}
	}
}

func TestHeatshrinkCompressesRepeats(t *testing.T) {
	data := bytes.Repeat([]byte("the quick brown fox "), 30)
	enc := HeatshrinkEncode(data)
	if len(enc) >= len(data) {
		t.Fatalf("expected compression: encoded %d >= raw %d", len(enc), len(data))
	}
	if dec := hsDecode(enc); !bytes.Equal(dec, data) {
		t.Fatalf("round-trip mismatch for repeated input")
	}
}
