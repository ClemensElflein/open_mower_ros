package fileservice

import (
	"encoding/binary"
	"strings"
	"testing"
)

func TestParseFileListPayload(t *testing.T) {
	// Payload: total=2, count=1, one entry.
	payload := make([]byte, 8+listEntrySize)
	binary.LittleEndian.PutUint32(payload[0:4], 2)
	binary.LittleEndian.PutUint32(payload[4:8], 1)
	copy(payload[8:8+listPathLen], "/sounds/en_hi-i-am-steve.mp3")
	binary.LittleEndian.PutUint32(payload[8+listPathLen:8+listEntrySize], 0xDEADBEEF)

	total, entries, err := parseList(payload)
	if err != nil {
		t.Fatal(err)
	}
	if total != 2 {
		t.Fatalf("total = %d, want 2", total)
	}
	if len(entries) != 1 {
		t.Fatalf("len(entries) = %d, want 1", len(entries))
	}
	if entries[0].Path != "/sounds/en_hi-i-am-steve.mp3" {
		t.Fatalf("path = %q, want /sounds/en_hi-i-am-steve.mp3", entries[0].Path)
	}
	if entries[0].Hash != 0xDEADBEEF {
		t.Fatalf("hash = %#x, want 0xDEADBEEF", entries[0].Hash)
	}
}

func TestParseFileListPayloadEmpty(t *testing.T) {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:4], 0)
	binary.LittleEndian.PutUint32(payload[4:8], 0)

	total, entries, err := parseList(payload)
	if err != nil {
		t.Fatal(err)
	}
	if total != 0 || len(entries) != 0 {
		t.Fatalf("empty dir: total=%d len=%d, want 0/0", total, len(entries))
	}
}

func TestParseFileListPayloadTruncated(t *testing.T) {
	// total=1 but the entry payload is cut off → must error, not panic.
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:4], 1)
	binary.LittleEndian.PutUint32(payload[4:8], 1)

	if _, _, err := parseList(payload); err == nil {
		t.Fatal("expected an error for a truncated payload, got nil")
	}
}

func TestParseFileListPayloadLongPath(t *testing.T) {
	// A path exactly at the 128-byte limit (no NUL padding).
	payload := make([]byte, 8+listEntrySize)
	binary.LittleEndian.PutUint32(payload[0:4], 1)
	binary.LittleEndian.PutUint32(payload[4:8], 1)
	longPath := "/sounds/" + strings.Repeat("x", listPathLen-len("/sounds/"))
	copy(payload[8:8+listPathLen], longPath)

	_, entries, err := parseList(payload)
	if err != nil {
		t.Fatal(err)
	}
	if len(entries) != 1 {
		t.Fatalf("len(entries) = %d, want 1", len(entries))
	}
	if entries[0].Path != longPath {
		t.Fatalf("path = %q, want %q", entries[0].Path, longPath)
	}
}
