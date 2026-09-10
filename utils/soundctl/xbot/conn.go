package xbot

import (
	"context"
	"encoding/binary"
	"fmt"
	"net"
	"sync/atomic"
	"time"
)

// Param is a single RPC parameter (target id + raw serialized bytes).
type Param struct {
	ID   uint16
	Data []byte
}

// Conn is a claimed unicast connection to a single xbot service.
// It is not safe for concurrent use.
type Conn struct {
	udp             *net.UDPConn
	svcIP           string
	svcPort         int
	serviceID       uint16
	heartbeatMicros uint32

	seq    atomic.Uint32
	callID atomic.Uint32
}

// Dial discovers and claims the service with the given ID.
func Dial(ctx context.Context, serviceID uint16, bindIP string, heartbeat time.Duration) (*Conn, error) {
	if heartbeat <= 0 {
		heartbeat = DefaultHeartbeatMicros * time.Microsecond
	}

	udp, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.ParseIP(bindIP), Port: 0})
	if err != nil {
		return nil, fmt.Errorf("udp bind: %w", err)
	}

	ip, port, err := discover(ctx, bindIP, serviceID)
	if err != nil {
		udp.Close()
		return nil, err
	}

	c := &Conn{
		udp:             udp,
		svcIP:           ip,
		svcPort:         port,
		serviceID:       serviceID,
		heartbeatMicros: uint32(heartbeat / time.Microsecond),
	}

	localIP := bindIP
	if localIP == "" || localIP == "0.0.0.0" {
		localIP = primaryIP()
	}

	if err := c.claim(ctx, localIP); err != nil {
		udp.Close()
		return nil, err
	}
	return c, nil
}

// Close closes the underlying UDP socket.
func (c *Conn) Close() error {
	return c.udp.Close()
}

// claim sends CLAIM and waits for the CLAIM ack, retrying until ctx is done.
func (c *Conn) claim(ctx context.Context, localIP string) error {
	ipInt := binary.BigEndian.Uint32(net.ParseIP(localIP).To4())
	localPort := c.udp.LocalAddr().(*net.UDPAddr).Port
	payload := PackClaimPayload(ipInt, uint16(localPort), c.heartbeatMicros)

	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := c.send(MsgClaim, 0, 0, payload); err != nil {
			return err
		}
		hdr, _, err := c.readPacket(time.Second)
		if err == nil && hdr.MessageType == MsgClaim && hdr.Arg1 == 1 {
			return nil
		}
		if err != nil && !isTimeout(err) {
			return err
		}
	}
}

// Call performs a synchronous RPC call and returns the response status and
// raw return payload.
func (c *Conn) Call(functionID uint8, params []Param, timeout time.Duration) (uint8, []byte, error) {
	callID := uint16(c.callID.Add(1) & 0xFFFF)

	var body []byte
	for _, p := range params {
		body = append(body, PackDescriptor(p.ID, uint32(len(p.Data)))...)
		body = append(body, p.Data...)
	}

	if err := c.send(MsgRPCCall, functionID, callID, body); err != nil {
		return 0, nil, err
	}

	deadline := time.Now().Add(timeout)
	for {
		hdr, payload, err := c.readPacket(time.Until(deadline))
		if err != nil {
			if isTimeout(err) {
				return 0, nil, fmt.Errorf("RPC call %d timed out", callID)
			}
			return 0, nil, err
		}
		if hdr.MessageType == MsgRPCResponse && hdr.Arg2 == callID {
			return hdr.Arg1, payload, nil
		}
		// Ignore heartbeats and other unrelated packets.
	}
}

// send writes a single header+payload packet to the service.
func (c *Conn) send(msgType byte, arg1 uint8, arg2 uint16, payload []byte) error {
	hdr := Header{
		ProtocolVersion: ProtocolVersion,
		MessageType:     msgType,
		ServiceID:       c.serviceID,
		Arg1:            arg1,
		Arg2:            arg2,
		SequenceNo:      uint16(c.seq.Add(1) & 0xFFFF),
		Timestamp:       nowMicros(),
		PayloadSize:     uint32(len(payload)),
	}
	pkt := append(hdr.Marshal(), payload...)
	_, err := c.udp.WriteToUDP(pkt, &net.UDPAddr{IP: net.ParseIP(c.svcIP), Port: c.svcPort})
	return err
}

// readPacket reads one packet and splits header + payload.
func (c *Conn) readPacket(timeout time.Duration) (Header, []byte, error) {
	_ = c.udp.SetReadDeadline(time.Now().Add(timeout))
	buf := make([]byte, MaxPacketSize)
	n, _, err := c.udp.ReadFromUDP(buf)
	if err != nil {
		return Header{}, nil, err
	}
	data := buf[:n]
	hdr, err := UnmarshalHeader(data)
	if err != nil {
		return hdr, nil, err
	}
	if len(data) != HeaderSize+int(hdr.PayloadSize) {
		return hdr, nil, errPacketMismatch
	}
	return hdr, data[HeaderSize : HeaderSize+int(hdr.PayloadSize)], nil
}

func isTimeout(err error) bool {
	if ne, ok := err.(net.Error); ok {
		return ne.Timeout()
	}
	return false
}
