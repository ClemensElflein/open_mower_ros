package fwsync

import (
	"hash/crc32"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func testIdentity() TTSIdentity {
	return TTSIdentity{
		Engine:   DefaultTTSEngine,
		Model:    DefaultTTSModel,
		Language: "en",
		Text:     "Robot operating system connected",
		Bitrate:  "32k",
		Sid:      5,
		Speed:    1.0,
		LoudNorm: true,
	}
}

func TestSidecarRoundTrip(t *testing.T) {
	dir := t.TempDir()
	mp3 := filepath.Join(dir, "en_ros_connected.mp3")
	data := []byte("not really an mp3, but the sidecar only cares about bytes")
	if err := os.WriteFile(mp3, data, 0o644); err != nil {
		t.Fatal(err)
	}

	id := testIdentity()
	sc := NewSidecar(id, MP3Info{SampleRate: 16000, Channels: 1, BitrateKbps: 32, DurationMs: 3460, CBR: true},
		int64(len(data)), crc32.ChecksumIEEE(data))
	sc.Stamped("soundctl test", 1234, time.Date(2026, 10, 25, 12, 0, 0, 0, time.UTC))
	if err := WriteSidecar(mp3, sc); err != nil {
		t.Fatal(err)
	}

	// No temp file may be left behind.
	if _, err := os.Stat(SidecarPath(mp3) + ".tmp"); err == nil {
		t.Fatal("WriteSidecar left the .tmp file behind")
	}

	got, err := ReadSidecar(mp3)
	if err != nil {
		t.Fatal(err)
	}
	if !got.Matches(id) {
		t.Fatalf("sidecar does not match its identity: %+v", got)
	}
	if !got.MatchesFile(data) {
		t.Fatal("sidecar does not match the file it was written for")
	}
	if got.DurationMs != 3460 || got.BitrateKbps != 32 || got.SampleRate != 16000 || got.Channels != 1 {
		t.Fatalf("audio fields lost in the round trip: %+v", got)
	}
	if got.RenderedAt != "2026-10-25T12:00:00Z" || got.RenderMs != 1234 || got.Generator != "soundctl test" {
		t.Fatalf("bookkeeping fields lost in the round trip: %+v", got)
	}
}

func TestSidecarDetectsChanges(t *testing.T) {
	id := testIdentity()
	sc := NewSidecar(id, MP3Info{SampleRate: 16000, Channels: 1, BitrateKbps: 32}, 10, 0)

	if !sc.Matches(id) {
		t.Fatal("fresh sidecar does not match its identity")
	}

	changed := id
	changed.Text = "Something else"
	if sc.Matches(changed) {
		t.Fatal("a changed text still matches")
	}

	changed = id
	changed.Speed = 1.1
	if sc.Matches(changed) {
		t.Fatal("a changed speed still matches")
	}

	changed = id
	changed.Sid = 7
	if sc.Matches(changed) {
		t.Fatal("a changed speaker id still matches")
	}

	changed = id
	changed.Bitrate = "64k"
	if sc.Matches(changed) {
		t.Fatal("a changed bitrate still matches")
	}

	// Bookkeeping fields must not take part in the cache key, otherwise every
	// checkout would re-render everything.
	changed = id
	sc.RenderedAt = "1999-01-01T00:00:00Z"
	sc.RenderMs = 999999
	sc.Generator = "other"
	if !sc.Matches(changed) {
		t.Fatal("bookkeeping fields influence Matches")
	}
}

func TestSidecarDetectsAlteredFile(t *testing.T) {
	data := []byte("0123456789")
	id := testIdentity()
	sc := NewSidecar(id, MP3Info{}, int64(len(data)), crc32.ChecksumIEEE(data))

	if !sc.MatchesFile(data) {
		t.Fatal("sidecar does not match the untouched file")
	}
	if sc.MatchesFile(data[:9]) {
		t.Fatal("a truncated file still matches")
	}
	other := []byte("012345678X")
	if sc.MatchesFile(other) {
		t.Fatal("a file with the same size but different content still matches")
	}
}

func TestReadSidecarRejectsUnknownVersion(t *testing.T) {
	dir := t.TempDir()
	mp3 := filepath.Join(dir, "x.mp3")
	// A future/foreign sidecar layout must be a cache miss, not a parse error
	// swallowed somewhere up the stack.
	if err := os.WriteFile(SidecarPath(mp3), []byte(`{"version": 99, "text": "x"}`), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, err := ReadSidecar(mp3); err == nil {
		t.Fatal("ReadSidecar accepted an unknown sidecar version")
	}
}

func TestReadSidecarMissing(t *testing.T) {
	if _, err := ReadSidecar(filepath.Join(t.TempDir(), "nope.mp3")); err == nil {
		t.Fatal("ReadSidecar returned no error for a missing sidecar")
	}
}
