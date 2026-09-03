// Package xbot implements a minimal client for the xbot service protocol
// (the UDP wire protocol used by the OpenMower low-level services).
//
// Only the subset needed to talk to the FileService (service id 12) is
// implemented: multicast discovery, CLAIM, HEARTBEAT and RPC calls.
package xbot

import (
	"encoding/binary"
	"errors"
	"fmt"
	"time"
)

// Message types (see xbot/datatypes/XbotHeader.hpp).
const (
	MsgUnknown              byte = 0x00
	MsgData                 byte = 0x01
	MsgConfigurationRequest byte = 0x02
	MsgClaim                byte = 0x03
	MsgHeartbeat            byte = 0x04
	MsgTransaction          byte = 0x05
	MsgRPCCall              byte = 0x06
	MsgRPCResponse          byte = 0x07
	MsgLog                  byte = 0x7F
	MsgServiceAdvertisement byte = 0x80
	MsgServiceQuery         byte = 0x81
)

// RPC status codes.
const (
	RpcSuccess uint8 = 0
	RpcBusy    uint8 = 1
	RpcError   uint8 = 2
)

// Protocol constants (see xbot/config.hpp).
const (
	ProtocolVersion        = 1
	HeaderSize             = 24
	DescriptorSize         = 8
	ClaimSize              = 10
	MulticastPort          = 4242
	SDMulticastAddr        = "233.255.255.0"
	DefaultHeartbeatMicros = 1_000_000
	HeartbeatJitterMicros  = 100_000
	MaxPacketSize          = 1500
)

// Service IDs (see services/service_ids.h).
const (
	ServiceFile = 12
)

// Header is the 24-byte packed xbot protocol header.
type Header struct {
	ProtocolVersion uint8
	MessageType     byte
	Flags           uint8
	ServiceID       uint16
	Arg1            uint8
	Arg2            uint16
	SequenceNo      uint16
	Timestamp       uint64
	PayloadSize     uint32
}

// Marshal packs the header into 24 little-endian bytes.
func (h Header) Marshal() []byte {
	b := make([]byte, HeaderSize)
	b[0] = h.ProtocolVersion
	b[1] = h.MessageType
	b[2] = h.Flags
	// b[3] reserved
	binary.LittleEndian.PutUint16(b[4:6], h.ServiceID)
	b[6] = h.Arg1
	// b[7] reserved
	binary.LittleEndian.PutUint16(b[8:10], h.Arg2)
	binary.LittleEndian.PutUint16(b[10:12], h.SequenceNo)
	binary.LittleEndian.PutUint64(b[12:20], h.Timestamp)
	binary.LittleEndian.PutUint32(b[20:24], h.PayloadSize)
	return b
}

// UnmarshalHeader parses a 24-byte little-endian xbot header.
func UnmarshalHeader(data []byte) (Header, error) {
	var h Header
	if len(data) < HeaderSize {
		return h, fmt.Errorf("header too short: %d bytes", len(data))
	}
	h.ProtocolVersion = data[0]
	h.MessageType = data[1]
	h.Flags = data[2]
	h.ServiceID = binary.LittleEndian.Uint16(data[4:6])
	h.Arg1 = data[6]
	h.Arg2 = binary.LittleEndian.Uint16(data[8:10])
	h.SequenceNo = binary.LittleEndian.Uint16(data[10:12])
	h.Timestamp = binary.LittleEndian.Uint64(data[12:20])
	h.PayloadSize = binary.LittleEndian.Uint32(data[20:24])
	return h, nil
}

// PackDescriptor packs an 8-byte DataDescriptor (target_id + payload_size).
func PackDescriptor(targetID uint16, payloadSize uint32) []byte {
	b := make([]byte, DescriptorSize)
	binary.LittleEndian.PutUint16(b[0:2], targetID)
	binary.LittleEndian.PutUint32(b[4:8], payloadSize)
	return b
}

// PackClaimPayload packs a 10-byte ClaimPayload. targetIP must be the
// network-order (big-endian) uint32 of the local IP.
func PackClaimPayload(targetIP uint32, targetPort uint16, heartbeatMicros uint32) []byte {
	b := make([]byte, ClaimSize)
	binary.LittleEndian.PutUint32(b[0:4], targetIP)
	binary.LittleEndian.PutUint16(b[4:6], targetPort)
	binary.LittleEndian.PutUint32(b[6:10], heartbeatMicros)
	return b
}

func nowMicros() uint64 {
	return uint64(time.Now().UnixMicro())
}

var errPacketMismatch = errors.New("packet size mismatch")
