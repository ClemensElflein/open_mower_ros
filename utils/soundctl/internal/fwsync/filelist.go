package fwsync

import (
	"encoding/binary"
	"fmt"
	"strings"
)

// fileServiceChunkSize is the fixed chunk size of the FileWrite RPC
// (services/file_service.json: Data is a uint8_t[256]).
const fileServiceChunkSize = 256

// fileEntry is one entry of the /sounds/ listing returned by FileList.
type fileEntry struct {
	Path string
	Hash uint32
}

// parseFileList decodes the payload of the FileList RPC:
//
// [uint32 total][uint32 count][ count × (char[128] path + uint32 hash) ]
//
// Each path is NUL terminated inside its 128-byte field; Hash is the opaque CRC32
// sidecar value (0 if there is none). The generated FileService client returns
// the payload as raw bytes, so the meaning of those bytes lives here, next to the
// code that uses it.
func parseFileList(payload []byte) (total uint32, entries []fileEntry, err error) {
	const pathLen = 128
	const entrySize = pathLen + 4

	if len(payload) < 8 {
		return 0, nil, fmt.Errorf("FileList payload too small (%d bytes)", len(payload))
	}
	total = binary.LittleEndian.Uint32(payload[0:4])
	count := binary.LittleEndian.Uint32(payload[4:8])
	if count == 0 {
		return total, nil, nil
	}
	if int(count)*entrySize > len(payload)-8 {
		return 0, nil, fmt.Errorf("FileList payload too small for %d entries", count)
	}
	offset := 8
	for i := uint32(0); i < count; i++ {
		path := string(payload[offset : offset+pathLen])
		if idx := strings.IndexByte(path, 0); idx >= 0 {
			path = path[:idx]
		}
		hash := binary.LittleEndian.Uint32(payload[offset+pathLen : offset+entrySize])
		entries = append(entries, fileEntry{Path: path, Hash: hash})
		offset += entrySize
	}
	return total, entries, nil
}
