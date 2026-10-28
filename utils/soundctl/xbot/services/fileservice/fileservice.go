// Package fileservice is a client for the OpenMower FileService (service id 12,
// defined in services/file_service.json): chunked file upload plus a paged
// directory listing. `soundctl sync` uses it to push the MP3s and to find
// orphaned files on the low level.
package fileservice

import (
	"context"
	"encoding/binary"
	"fmt"
	"strings"
	"time"

	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

// ServiceID is the FileService id (services/service_ids.h).
const ServiceID uint16 = 12

// RPC function ids (services/file_service.json -> "functions").
const (
	fnExists uint8 = 0
	fnRemove uint8 = 1
	fnWrite  uint8 = 2
	fnList   uint8 = 3
)

// ChunkSize is the fixed chunk size of the FileWrite RPC.
const ChunkSize = 256

// FileList wire format (see FileList in file_service.json):
//
//	[uint32 total][uint32 count][ count × (char[128] path + uint32 hash) ]
const (
	listPathLen   = 128
	listEntrySize = listPathLen + 4
)

// Client speaks the FileService RPCs over a claimed connection.
type Client struct {
	conn *xbot.Conn
}

// Dial discovers and claims the FileService. Claiming is intended: the upload
// replies (the number of bytes written per chunk) are sent to the owner.
func Dial(ctx context.Context, bindIP string, heartbeat, callTimeout time.Duration) (*Client, error) {
	conn, err := xbot.Dial(ctx, xbot.Options{
		ServiceID:   ServiceID,
		BindIP:      bindIP,
		Heartbeat:   heartbeat,
		CallTimeout: callTimeout,
	})
	if err != nil {
		return nil, err
	}
	return &Client{conn: conn}, nil
}

// Close closes the underlying connection.
func (c *Client) Close() error { return c.conn.Close() }

// Endpoint returns "ip:port" of the connected service.
func (c *Client) Endpoint() string { return c.conn.Endpoint() }

// Exists reports true only if the file exists *and* its hash matches, which is
// what makes an upload resumable: an unchanged file is skipped.
func (c *Client) Exists(ctx context.Context, path string, hash uint32) (bool, error) {
	payload, err := c.conn.Call(ctx, fnExists, xbot.String(0, path), xbot.Uint32(1, hash))
	if err != nil {
		return false, fmt.Errorf("fileservice: Exists(%s): %w", path, err)
	}
	if len(payload) < 1 {
		return false, fmt.Errorf("fileservice: Exists(%s) returned an empty payload", path)
	}
	return payload[0] == 1, nil
}

// Remove deletes a file. A file that is already gone counts as success
// (the firmware returns 0 for "removed or not there").
func (c *Client) Remove(ctx context.Context, path string) (bool, error) {
	payload, err := c.conn.Call(ctx, fnRemove, xbot.String(0, path))
	if err != nil {
		return false, fmt.Errorf("fileservice: Remove(%s): %w", path, err)
	}
	if len(payload) < 1 {
		return false, fmt.Errorf("fileservice: Remove(%s) returned an empty payload", path)
	}
	return payload[0] == 0, nil
}

// Write writes one chunk at offset. A hash != 0 marks the last chunk and commits
// the file (the firmware renames the temporary file and stores the sidecar). It
// returns the number of bytes the firmware accepted, or a negative error code.
func (c *Client) Write(ctx context.Context, path string, offset uint32, data []byte, hash uint32) (int32, error) {
	if len(data) > ChunkSize {
		return 0, fmt.Errorf("fileservice: chunk too large: %d > %d", len(data), ChunkSize)
	}
	// The RPC expects a full chunk; the tail of the file is zero padded.
	chunk := make([]byte, ChunkSize)
	copy(chunk, data)

	payload, err := c.conn.Call(ctx, fnWrite,
		xbot.String(0, path),
		xbot.Uint32(1, offset),
		xbot.Bytes(2, chunk),
		xbot.Uint32(3, hash),
	)
	if err != nil {
		return 0, fmt.Errorf("fileservice: Write(%s @%d): %w", path, offset, err)
	}
	if len(payload) < 4 {
		return 0, fmt.Errorf("fileservice: Write(%s @%d) returned a short payload (%d bytes)", path, offset, len(payload))
	}
	return int32(binary.LittleEndian.Uint32(payload[:4])), nil
}

// Entry is one directory entry returned by List.
type Entry struct {
	Path string
	Hash uint32
}

// List returns the number of files in dir plus the entries starting at
// startIndex. Callers page through until they have Total entries:
//
//	for off := uint32(0); ; off += uint32(len(entries)) {
//		total, entries, err := fs.List(ctx, "/sounds/", off)
//		if err != nil { ... }
//		if len(entries) == 0 || off >= total { break }
//	}
func (c *Client) List(ctx context.Context, dir string, startIndex uint32) (uint32, []Entry, error) {
	payload, err := c.conn.Call(ctx, fnList, xbot.String(0, dir), xbot.Uint32(1, startIndex))
	if err != nil {
		return 0, nil, fmt.Errorf("fileservice: List(%s @%d): %w", dir, startIndex, err)
	}
	return parseList(payload)
}

// parseList decodes the FileList response payload (see the wire format above).
// Each path is NUL terminated inside its 128-byte field; Hash is the opaque
// CRC32 sidecar value (0 if there is none).
func parseList(payload []byte) (total uint32, entries []Entry, err error) {
	if len(payload) < 8 {
		return 0, nil, fmt.Errorf("fileservice: List payload too small (%d bytes)", len(payload))
	}
	total = binary.LittleEndian.Uint32(payload[0:4])
	count := binary.LittleEndian.Uint32(payload[4:8])
	if count == 0 {
		return total, nil, nil
	}
	if int(count)*listEntrySize > len(payload)-8 {
		return 0, nil, fmt.Errorf("fileservice: List payload too small for %d entries", count)
	}
	offset := 8
	for i := uint32(0); i < count; i++ {
		path := string(payload[offset : offset+listPathLen])
		if idx := strings.IndexByte(path, 0); idx >= 0 {
			path = path[:idx]
		}
		hash := binary.LittleEndian.Uint32(payload[offset+listPathLen : offset+listEntrySize])
		entries = append(entries, Entry{Path: path, Hash: hash})
		offset += listEntrySize
	}
	return total, entries, nil
}
