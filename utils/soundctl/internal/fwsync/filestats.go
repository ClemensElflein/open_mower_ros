package fwsync

import "fmt"

// FileStats is the LittleFS usage the low level reports for its file storage.
type FileStats struct {
	TotalBytes uint32
	UsedBytes  uint32
	// FreeBytes is what a caller may still write: one block is kept back, because
	// LittleFS needs a free block while it updates metadata.
	FreeBytes uint32
	BlockSize uint32
}

// ParseFileStats names the answer of the FileStats RPC: the schema declares
// uint32_t[4], so the values already carry their meaning (total, used, free, block
// size) and no byte layout has to be interpreted here. Only the count is checked.
func ParseFileStats(values []uint32) (FileStats, error) {
	const want = 4
	if len(values) != want {
		return FileStats{}, fmt.Errorf("FileStats returned %d values, want %d", len(values), want)
	}
	return FileStats{
		TotalBytes: values[0],
		UsedBytes:  values[1],
		FreeBytes:  values[2],
		BlockSize:  values[3],
	}, nil
}

// String renders the usage for a log line (binary units, one decimal).
func (s FileStats) String() string {
	usedPct := 0.0
	if s.TotalBytes > 0 {
		usedPct = 100.0 * float64(s.UsedBytes) / float64(s.TotalBytes)
	}
	return fmt.Sprintf("%s of %s used (%.1f%%), %s free",
		mib(s.UsedBytes), mib(s.TotalBytes), usedPct, mib(s.FreeBytes))
}

// mib formats a byte count as MiB (or KiB below one MiB).
func mib(bytes uint32) string {
	if bytes < 1<<20 {
		return fmt.Sprintf("%d KiB", bytes>>10)
	}
	return fmt.Sprintf("%.1f MiB", float64(bytes)/(1<<20))
}
