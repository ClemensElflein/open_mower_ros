package xbot

import (
	"context"
	"encoding/binary"
	"fmt"
	"time"
)

// FileService function IDs (see services/file_service.json).
const (
	fileFnExists = 0
	fileFnRemove = 1
	fileFnWrite  = 2
)

// FileChunkSize is the fixed chunk size of the FileWrite RPC.
const FileChunkSize = 256

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
	return payload[0] == 1, nil
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

func packUint32(v uint32) []byte {
	b := make([]byte, 4)
	binary.LittleEndian.PutUint32(b, v)
	return b
}
