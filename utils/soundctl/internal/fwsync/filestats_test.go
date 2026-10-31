package fwsync

import (
	"strings"
	"testing"
)

func TestParseFileStats(t *testing.T) {
	// The order the low level fills uint32_t[4] with.
	values := []uint32{
		16 << 20, // total
		1 << 20,  // used
		14 << 20, // free (one block kept back)
		4096,     // block size
	}

	stats, err := ParseFileStats(values)
	if err != nil {
		t.Fatalf("ParseFileStats: %v", err)
	}
	if stats.TotalBytes != 16<<20 || stats.UsedBytes != 1<<20 || stats.FreeBytes != 14<<20 || stats.BlockSize != 4096 {
		t.Fatalf("stats = %+v", stats)
	}
	if got := stats.String(); !strings.Contains(got, "1.0 MiB of 16.0 MiB used") || !strings.Contains(got, "free") {
		t.Errorf("String() = %q", got)
	}
}

func TestParseFileStatsRejectsWrongCount(t *testing.T) {
	if _, err := ParseFileStats(make([]uint32, 3)); err == nil {
		t.Fatal("three values must be rejected")
	}
}
