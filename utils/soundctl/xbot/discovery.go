package xbot

import (
	"context"
	"fmt"
	"net"
	"strconv"
	"time"

	"github.com/fxamacker/cbor/v2"
)

// advertisement is the CBOR-decoded SERVICE_ADVERTISEMENT payload.
type advertisement struct {
	SID      int `cbor:"sid"`
	Endpoint struct {
		IP   string `cbor:"ip"`
		Port int    `cbor:"port"`
	} `cbor:"endpoint"`
}

// discover waits until the service with the given ID is advertised on the
// multicast group, then returns its unicast endpoint.
func discover(ctx context.Context, bindIP string, serviceID uint16) (string, int, error) {
	maddr, err := net.ResolveUDPAddr("udp4", net.JoinHostPort(SDMulticastAddr, strconv.Itoa(MulticastPort)))
	if err != nil {
		return "", 0, err
	}

	var ifi *net.Interface
	if bindIP != "" && bindIP != "0.0.0.0" {
		ifi = interfaceByIP(bindIP)
	}

	conn, err := net.ListenMulticastUDP("udp4", ifi, maddr)
	if err != nil {
		return "", 0, fmt.Errorf("multicast listen: %w", err)
	}
	defer conn.Close()

	buf := make([]byte, MaxPacketSize)
	for {
		if err := ctx.Err(); err != nil {
			return "", 0, err
		}
		_ = conn.SetReadDeadline(time.Now().Add(time.Second))
		n, _, err := conn.ReadFromUDP(buf)
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return "", 0, err
		}
		data := buf[:n]
		hdr, err := UnmarshalHeader(data)
		if err != nil || hdr.MessageType != MsgServiceAdvertisement {
			continue
		}
		if len(data) != HeaderSize+int(hdr.PayloadSize) {
			continue
		}
		var adv advertisement
		if err := cbor.Unmarshal(data[HeaderSize:HeaderSize+int(hdr.PayloadSize)], &adv); err != nil {
			continue
		}
		if adv.SID == int(serviceID) && adv.Endpoint.IP != "" && adv.Endpoint.Port != 0 {
			return adv.Endpoint.IP, adv.Endpoint.Port, nil
		}
	}
}

// interfaceByIP returns the interface that owns the given IPv4 address, or nil.
func interfaceByIP(ipStr string) *net.Interface {
	ip := net.ParseIP(ipStr)
	if ip == nil {
		return nil
	}
	ifaces, err := net.Interfaces()
	if err != nil {
		return nil
	}
	for i := range ifaces {
		addrs, err := ifaces[i].Addrs()
		if err != nil {
			continue
		}
		for _, a := range addrs {
			if ipn, ok := a.(*net.IPNet); ok && ipn.IP.Equal(ip) {
				return &ifaces[i]
			}
		}
	}
	return nil
}

// primaryIP returns the primary non-loopback, non-virtual IPv4 address,
// mirroring the Python/C++ helper that skips docker/veth/etc.
func primaryIP() string {
	ifaces, err := net.Interfaces()
	if err != nil {
		return "127.0.0.1"
	}
	skip := []string{"lo", "docker", "veth", "virbr", "br-", "wg", "tun", "tap"}
	for i := range ifaces {
		name := ifaces[i].Name
		skipped := false
		for _, p := range skip {
			if len(name) >= len(p) && name[:len(p)] == p {
				skipped = true
				break
			}
		}
		if skipped {
			continue
		}
		addrs, err := ifaces[i].Addrs()
		if err != nil {
			continue
		}
		for _, a := range addrs {
			if ipn, ok := a.(*net.IPNet); ok {
				if ip4 := ipn.IP.To4(); ip4 != nil && !ip4.IsLoopback() {
					return ip4.String()
				}
			}
		}
	}
	return "127.0.0.1"
}
