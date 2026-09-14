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

	udp, err := listenUDP(bindIP)
	if err != nil {
		return nil, err
	}

	ip, port, err := discover(ctx, bindIP, serviceID)
	if err != nil {
		_ = udp.Close()
		return nil, err
	}

	c := newServiceConn(udp, ip, port, serviceID)
	c.heartbeatMicros = uint32(heartbeat / time.Microsecond)

	localIP := bindIP
	if localIP == "" || localIP == "0.0.0.0" {
		localIP = primaryIP()
	}

	if err := c.claim(ctx, localIP); err != nil {
		_ = udp.Close()
		return nil, err
	}
	return c, nil
}

// listenUDP opens a UDP socket bound to bindIP ("" or "0.0.0.0" = any interface).
func listenUDP(bindIP string) (*net.UDPConn, error) {
	udp, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.ParseIP(bindIP), Port: 0})
	if err != nil {
		return nil, fmt.Errorf("udp bind: %w", err)
	}
	return udp, nil
}

// newServiceConn wraps an open socket for a known service endpoint.
func newServiceConn(udp *net.UDPConn, ip string, port int, serviceID uint16) *Conn {
	return &Conn{
		udp:             udp,
		svcIP:           ip,
		svcPort:         port,
		serviceID:       serviceID,
		heartbeatMicros: DefaultHeartbeatMicros,
	}
}

// Close closes the underlying UDP socket.
func (c *Conn) Close() error {
	return c.udp.Close()
}

// claim sends CLAIM and waits for the CLAIM ack, retrying until ctx is done.
func (c *Conn) claim(ctx context.Context, localIP string) error {
	ip := net.ParseIP(localIP).To4()
	if ip == nil {
		return fmt.Errorf("invalid IPv4 bind address %q", localIP)
	}
	ipInt := binary.BigEndian.Uint32(ip)
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

// buildRPCPayload serialises an RPC body: one 8-byte descriptor per parameter,
// each followed by that parameter's raw bytes.
func buildRPCPayload(params []Param) []byte {
	var body []byte
	for _, p := range params {
		body = append(body, PackDescriptor(p.ID, uint32(len(p.Data)))...)
		body = append(body, p.Data...)
	}
	return body
}

// startCall sends an RPC and returns the call id used, without waiting for a
// response.
func (c *Conn) startCall(functionID uint8, params []Param) (uint16, error) {
	callID := uint16(c.callID.Add(1) & 0xFFFF)
	return callID, c.send(MsgRPCCall, functionID, callID, buildRPCPayload(params))
}

// Call performs a synchronous RPC call and returns the response status and
// raw return payload.
func (c *Conn) Call(functionID uint8, params []Param, timeout time.Duration) (uint8, []byte, error) {
	callID, err := c.startCall(functionID, params)
	if err != nil {
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

// SendRPC sends an RPC call without waiting for the response ("fire and forget").
//
// Needed when the service is claimed by someone else (on the robot: the
// high-level system). The firmware still processes the call as long as the
// service is running — only the response goes to the claimed owner.
func (c *Conn) SendRPC(functionID uint8, params []Param) error {
	_, err := c.startCall(functionID, params)
	return err
}

// DialNoClaimTo connects to a known service endpoint: no discovery, no claim.
// Use it to skip the advertisement-driven discovery on repeat calls (the LL only
// advertises slowly once claimed) — see `soundctl play --addr`.
func DialNoClaimTo(ip string, port int, serviceID uint16) (*Conn, error) {
	udp, err := listenUDP("")
	if err != nil {
		return nil, err
	}
	return newServiceConn(udp, ip, port, serviceID), nil
}

// Endpoint returns the unicast address of the connected service.
func (c *Conn) Endpoint() (string, int) { return c.svcIP, c.svcPort }

// DialNoClaim discovers and connects to a service without claiming it.
//
// A claim would stop a running service and re-target its responses (see
// Service::HandleClaimMessage in the firmware), so clients that only want to
// push data or RPCs to a service owned by someone else must not claim.
func DialNoClaim(ctx context.Context, serviceID uint16, bindIP string) (*Conn, error) {
	udp, err := listenUDP(bindIP)
	if err != nil {
		return nil, err
	}

	ip, port, err := discover(ctx, bindIP, serviceID)
	if err != nil {
		_ = udp.Close()
		return nil, err
	}

	return newServiceConn(udp, ip, port, serviceID), nil
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
