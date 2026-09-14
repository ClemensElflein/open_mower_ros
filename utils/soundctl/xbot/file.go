package xbot

import (
	"context"
	"encoding/binary"
	"fmt"
	"strings"
	"time"
)

// FileService function IDs (see services/file_service.json).
const (
	fileFnExists = 0
	fileFnRemove = 1
	fileFnWrite  = 2
	fileFnList   = 3
)

// FileChunkSize is the fixed chunk size of the FileWrite RPC.
const FileChunkSize = 256

// FileList wire-format constants (see FileList in file_service.json):
//
//	[uint32 total][uint32 count][ count × (char[128] path + uint32 hash) ].
const (
	fileListPathLen   = 128
	fileListEntrySize = fileListPathLen + 4
)

// FileService is a thin wrapper around the FileService RPC (service id 12).
type FileService struct {
	c       *Conn
	timeout time.Duration
}

// NewFileService discovers and claims the FileService.
func NewFileService(ctx context.Context, bindIP string, heartbeat, timeout time.Duration) (*FileService, error) {
	c, err := Dial(ctx, ServiceFile, bindIP, heartbeat)
	if err != nil {
		return nil, err
	}
	return &FileService{c: c, timeout: timeout}, nil
}

// Close closes the underlying connection.
func (f *FileService) Close() error { return f.c.Close() }

// FileExists returns true only if the file exists AND its hash matches.
func (f *FileService) FileExists(path string, hash uint32) (bool, error) {
	status, payload, err := f.c.Call(fileFnExists, []Param{
		{ID: 0, Data: []byte(path)},
		{ID: 1, Data: packUint32(hash)},
	}, f.timeout)
	if err != nil {
		return false, err
	}
	if status != RpcSuccess {
		return false, fmt.Errorf("FileExists failed with status %d", status)
	}
	if len(payload) < 1 {
		return false, fmt.Errorf("FileExists returned empty payload")
	}
	return payload[0] == 1, nil
}

// FileRemove deletes a file. Returns true on success.
func (f *FileService) FileRemove(path string) (bool, error) {
	status, payload, err := f.c.Call(fileFnRemove, []Param{
		{ID: 0, Data: []byte(path)},
	}, f.timeout)
	if err != nil {
		return false, err
	}
	if status != RpcSuccess {
		return false, fmt.Errorf("FileRemove failed with status %d", status)
	}
	if len(payload) < 1 {
		return false, fmt.Errorf("FileRemove returned empty payload")
	}
	// The firmware returns 0 on success (including "already gone") and 1 on failure.
	return payload[0] == 0, nil
}

// FileWrite writes one chunk at offset. When hash != 0 the chunk is treated
// as the final one and commits the file (rename + sidecar). It returns the
// number of bytes written or a negative error code.
func (f *FileService) FileWrite(path string, offset uint32, data []byte, hash uint32) (int32, error) {
	if len(data) > FileChunkSize {
		return 0, fmt.Errorf("chunk too large: %d > %d", len(data), FileChunkSize)
	}
	chunk := make([]byte, FileChunkSize)
	copy(chunk, data)

	status, payload, err := f.c.Call(fileFnWrite, []Param{
		{ID: 0, Data: []byte(path)},
		{ID: 1, Data: packUint32(offset)},
		{ID: 2, Data: chunk},
		{ID: 3, Data: packUint32(hash)},
	}, f.timeout)
	if err != nil {
		return 0, err
	}
	if status != RpcSuccess {
		return 0, fmt.Errorf("FileWrite failed with status %d", status)
	}
	if len(payload) < 4 {
		return 0, fmt.Errorf("FileWrite returned short payload (%d bytes)", len(payload))
	}
	return int32(binary.LittleEndian.Uint32(payload[:4])), nil
}

// FileEntry is a single /sounds/ directory entry returned by FileList.
type FileEntry struct {
	Path string
	Hash uint32
}

// FileList returns the total number of files in the directory plus a slice
// holding at most one file (the one at startIndex). Callers page through using
// startIndex until they have collected total entries:
//
//	for off := uint32(0); ; off += uint32(len(entries)) {
//	    total, entries, err := fs.FileList("/sounds/", off)
//	    if err != nil { ... }
//	    if len(entries) == 0 || off >= total { break }
//	}
func (f *FileService) FileList(dir string, startIndex uint32) (total uint32, entries []FileEntry, err error) {
	status, payload, err := f.c.Call(fileFnList, []Param{
		{ID: 0, Data: []byte(dir)},
		{ID: 1, Data: packUint32(startIndex)},
	}, f.timeout)
	if err != nil {
		return 0, nil, err
	}
	if status != RpcSuccess {
		return 0, nil, fmt.Errorf("FileList failed with status %d", status)
	}
	if len(payload) < 8 {
		return 0, nil, fmt.Errorf("FileList returned short payload (%d bytes)", len(payload))
	}
	return parseFileListPayload(payload)
}

// parseFileListPayload decodes the FileList response payload:
//
//	[uint32 total][uint32 count][ count × (char[128] path + uint32 hash) ].
//
// each entry's path is NUL-terminated inside its 128-byte field; hash is the
// opaque CRC32 sidecar value (0 if none).
func parseFileListPayload(payload []byte) (total uint32, entries []FileEntry, err error) {
	if len(payload) < 8 {
		return 0, nil, fmt.Errorf("FileList payload too small (%d bytes)", len(payload))
	}
	total = binary.LittleEndian.Uint32(payload[0:4])
	count := binary.LittleEndian.Uint32(payload[4:8])
	if count > 0 {
		if int(count)*fileListEntrySize > len(payload)-8 {
			return 0, nil, fmt.Errorf("FileList payload too small for %d entries", count)
		}
		offset := 8
		for i := uint32(0); i < count; i++ {
			pathBytes := payload[offset : offset+fileListPathLen]
			path := string(pathBytes)
			if idx := strings.IndexByte(path, 0); idx >= 0 {
				path = path[:idx]
			}
			hash := binary.LittleEndian.Uint32(payload[offset+fileListPathLen : offset+fileListEntrySize])
			entries = append(entries, FileEntry{Path: path, Hash: hash})
			offset += fileListEntrySize
		}
	}
	return total, entries, nil
}

func packUint32(v uint32) []byte {
	b := make([]byte, 4)
	binary.LittleEndian.PutUint32(b, v)
	return b
}
