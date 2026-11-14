package fwsync

import (
	"encoding/binary"
	"fmt"
	"strings"

	"github.com/xtech/xbot_framework/xbot_service_interface_go/xbot"
)

// maxFileWriteChunk is the largest Data payload a FileWrite call can carry for this
// path. One packet holds maxPacketSize - header bytes of RPC body; the call spends a
// descriptor per parameter (path, offset, data, hash), the offset and the hash
// themselves, and the path - which is a blob, so only its real length counts.
func maxFileWriteChunk(path string) int {
	const (
		descriptors   = 4 * 8 // one DataDescriptor per parameter
		offsetAndHash = 4 + 4
	)
	return xbot.MaxPacketSize - xbot.HeaderSize - descriptors - offsetAndHash - len(path)
}

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
