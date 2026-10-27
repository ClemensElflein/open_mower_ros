package xbot

// Heatshrink stream parameters. These MUST match the firmware build
// (HEATSHRINK_STATIC_WINDOW_BITS=9, HEATSHRINK_STATIC_LOOKAHEAD_BITS=5).
const (
	hsWindowBits    = 9
	hsLookaheadBits = 5
	hsWindowSize    = 1 << hsWindowBits    // 512
	hsLookaheadSize = 1 << hsLookaheadBits // 32

	hsLiteralMarker = 1
	hsBackrefMarker = 0

	// A match is only worth a back-reference when it beats the +6 bit
	// overhead of the backref token (1 tag + 9 index + 5 count = 15 bits vs
	// 2*9 = 18 bits for two literals), i.e. length >= 2.
	hsMinMatch = 2
)

// HeatshrinkEncode compresses data into the heatshrink (LZSS) stream format
// that the firmware's HeatshrinkDataSource (window=9, lookahead=5) decodes.
//
// The stream is a sequence of tokens, MSB-first packed:
//
//	literal : 1 (tag)                     + 8 bits (byte)
//	backref : 0 (tag) + 9 bits (offset-1) + 5 bits (length-1)
func HeatshrinkEncode(data []byte) []byte {
	var out []byte
	var cur byte
	mask := byte(0x80)

	pushBit := func(bit uint8) {
		if bit != 0 {
			cur |= mask
		}
		mask >>= 1
		if mask == 0 {
			out = append(out, cur)
			cur = 0
			mask = 0x80
		}
	}
	pushBits := func(count int, bits uint16) {
		for i := count - 1; i >= 0; i-- {
			pushBit(uint8((bits >> uint(i)) & 1))
		}
	}

	for pos := 0; pos < len(data); {
		start := pos - hsWindowSize
		if start < 0 {
			start = 0
		}
		maxLen := hsLookaheadSize
		if pos+maxLen > len(data) {
			maxLen = len(data) - pos
		}

		bestLen, bestOff := 0, 0
		for i := start; i < pos; i++ {
			l := 0
			for l < maxLen && data[i+l] == data[pos+l] {
				l++
			}
			if l > bestLen {
				bestLen = l
				bestOff = pos - i
				if l == maxLen {
					break
				}
			}
		}

		if bestLen >= hsMinMatch {
			pushBits(1, hsBackrefMarker)
			pushBits(hsWindowBits, uint16(bestOff-1))
			pushBits(hsLookaheadBits, uint16(bestLen-1))
			pos += bestLen
		} else {
			pushBits(1, hsLiteralMarker)
			pushBits(8, uint16(data[pos]))
			pos++
		}
	}

	if mask != 0x80 {
		out = append(out, cur) // flush partial byte (zero-padded)
	}
	return out
}
