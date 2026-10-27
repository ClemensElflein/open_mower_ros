package xbot

import (
	"context"
	"fmt"
	"net"
	"sort"
	"strconv"
	"strings"
	"time"

	"github.com/fxamacker/cbor/v2"
)

// SlowAdvertisementInterval is how often a service advertises itself once it is
// claimed or has stayed unclaimed for sd_discovery_timeout (60 s): every 10 s
// (see xbot/config.hpp). While a service is unclaimed *and* younger than that
// timeout it advertises every second instead, so a discovery window shorter
// than this interval can miss a perfectly healthy service forever — on a robot
// the low level is long past the fast phase when the high level boots.
const SlowAdvertisementInterval = 10 * time.Second

// advertisement is the CBOR-decoded SERVICE_ADVERTISEMENT payload (see
// ServiceInfo::to_json in xbot_framework).
type advertisement struct {
	SID      int `cbor:"sid"`
	Endpoint struct {
		IP   string `cbor:"ip"`
		Port int    `cbor:"port"`
	} `cbor:"endpoint"`
	Desc struct {
		Type    string `cbor:"type"`
		Version int    `cbor:"version"`
	} `cbor:"desc"`
}

// ServiceAdvertisement is one service as advertised on the discovery group.
type ServiceAdvertisement struct {
	SID     uint16
	Type    string
	Version int
	IP      string
	Port    int
}

// Endpoint renders "ip:port".
func (a ServiceAdvertisement) Endpoint() string {
	return net.JoinHostPort(a.IP, strconv.Itoa(a.Port))
}

// String renders one line of the "soundctl discover" table.
func (a ServiceAdvertisement) String() string {
	return fmt.Sprintf("%-3d %-30s %-3d %s", a.SID, a.Type, a.Version, a.Endpoint())
}

// decodeAdvertisement decodes one discovery packet. It reports false for
// anything unusable: other message types, a wrong payload length, undecodable
// CBOR or a missing endpoint (the framework ignores those as well).
func decodeAdvertisement(data []byte) (ServiceAdvertisement, bool) {
	hdr, err := UnmarshalHeader(data)
	if err != nil || hdr.MessageType != MsgServiceAdvertisement {
		return ServiceAdvertisement{}, false
	}
	if len(data) != HeaderSize+int(hdr.PayloadSize) {
		return ServiceAdvertisement{}, false
	}
	var adv advertisement
	if err := cbor.Unmarshal(data[HeaderSize:HeaderSize+int(hdr.PayloadSize)], &adv); err != nil {
		return ServiceAdvertisement{}, false
	}
	if adv.SID <= 0 || adv.SID > 0xFFFF || adv.Endpoint.IP == "" || adv.Endpoint.Port == 0 {
		return ServiceAdvertisement{}, false
	}
	return ServiceAdvertisement{
		SID:     uint16(adv.SID),
		Type:    adv.Desc.Type,
		Version: adv.Desc.Version,
		IP:      adv.Endpoint.IP,
		Port:    adv.Endpoint.Port,
	}, true
}

// listen joins the discovery group and passes every usable advertisement to
// visit until visit returns true or ctx expires. It returns everything it saw
// (in arrival order) so a caller that timed out can report what *was* there.
func listen(ctx context.Context, bindIP string, visit func(ServiceAdvertisement) bool) ([]ServiceAdvertisement, error) {
	maddr, err := net.ResolveUDPAddr("udp4", net.JoinHostPort(SDMulticastAddr, strconv.Itoa(MulticastPort)))
	if err != nil {
		return nil, err
	}

	var ifi *net.Interface
	if bindIP != "" && bindIP != "0.0.0.0" {
		ifi = interfaceByIP(bindIP)
	}

	conn, err := net.ListenMulticastUDP("udp4", ifi, maddr)
	if err != nil {
		return nil, fmt.Errorf("multicast listen: %w", err)
	}
	defer conn.Close()

	var seen []ServiceAdvertisement
	buf := make([]byte, MaxPacketSize)
	for {
		if err := ctx.Err(); err != nil {
			return seen, err
		}
		_ = conn.SetReadDeadline(time.Now().Add(time.Second))
		n, _, err := conn.ReadFromUDP(buf)
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return seen, err
		}
		adv, ok := decodeAdvertisement(buf[:n])
		if !ok {
			continue
		}
		seen = append(seen, adv)
		if visit != nil && visit(adv) {
			return seen, nil
		}
	}
}

// discover waits until the service with the given ID is advertised on the
// multicast group, then returns its unicast endpoint. On timeout the error
// lists the service IDs that were advertised: that is what distinguishes "the
// LL is not on the network" from "this LL firmware has no such service".
func discover(ctx context.Context, bindIP string, serviceID uint16) (string, int, error) {
	var found *ServiceAdvertisement
	seen, err := listen(ctx, bindIP, func(adv ServiceAdvertisement) bool {
		if adv.SID != serviceID {
			return false
		}
		found = &adv
		return true
	})
	if found != nil {
		return found.IP, found.Port, nil
	}
	if err == nil {
		err = context.DeadlineExceeded
	}
	return "", 0, fmt.Errorf("%w (SID %d was not advertised; advertised: %s)", err, serviceID, summarizeSIDs(seen))
}

// Discover lists the services advertised on the discovery group until ctx
// expires, sorted by service ID. Used by "soundctl discover".
func Discover(ctx context.Context, bindIP string) ([]ServiceAdvertisement, error) {
	seen, err := listen(ctx, bindIP, nil)
	if err != nil && len(seen) == 0 {
		return nil, err
	}
	// One entry per service, the latest advertisement wins.
	byID := make(map[uint16]ServiceAdvertisement, len(seen))
	for _, adv := range seen {
		byID[adv.SID] = adv
	}
	out := make([]ServiceAdvertisement, 0, len(byID))
	for _, adv := range byID {
		out = append(out, adv)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].SID < out[j].SID })
	return out, nil
}

// summarizeSIDs lists the distinct service IDs of the seen advertisements, for
// error messages ("none" when nothing was on the bus at all).
func summarizeSIDs(seen []ServiceAdvertisement) string {
	ids := make(map[uint16]bool, len(seen))
	for _, adv := range seen {
		ids[adv.SID] = true
	}
	if len(ids) == 0 {
		return "none"
	}
	sorted := make([]int, 0, len(ids))
	for id := range ids {
		sorted = append(sorted, int(id))
	}
	sort.Ints(sorted)
	parts := make([]string, len(sorted))
	for i, id := range sorted {
		parts[i] = strconv.Itoa(id)
	}
	return strings.Join(parts, ",")
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
