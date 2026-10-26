package xbot

import (
	"strings"
	"testing"

	"github.com/fxamacker/cbor/v2"
)

// packAdvertisement builds a SERVICE_ADVERTISEMENT exactly the way
// xbot_framework serialises ServiceInfo (sid / endpoint / desc).
func packAdvertisement(t *testing.T, sid int, ip string, port int, typ string, version int) []byte {
	t.Helper()
	payload, err := cbor.Marshal(map[string]any{
		"sid":      sid,
		"endpoint": map[string]any{"ip": ip, "port": port},
		"desc":     map[string]any{"type": typ, "version": version},
	})
	if err != nil {
		t.Fatal(err)
	}
	hdr := Header{
		ProtocolVersion: ProtocolVersion,
		MessageType:     MsgServiceAdvertisement,
		ServiceID:       uint16(sid),
		PayloadSize:     uint32(len(payload)),
	}
	return append(hdr.Marshal(), payload...)
}

func TestDecodeAdvertisement(t *testing.T) {
	adv, ok := decodeAdvertisement(packAdvertisement(t, ServiceFile, "172.16.78.150", 49153, "FileService", 1))
	if !ok {
		t.Fatal("advertisement rejected")
	}
	if adv.SID != ServiceFile || adv.Type != "FileService" || adv.Version != 1 {
		t.Errorf("decoded %+v, want SID 12 / FileService / v1", adv)
	}
	if got, want := adv.Endpoint(), "172.16.78.150:49153"; got != want {
		t.Errorf("Endpoint() = %q, want %q", got, want)
	}
	if got, want := adv.String(), "12  FileService"; !strings.HasPrefix(got, want) {
		t.Errorf("String() = %q, want prefix %q", got, want)
	}
	if !strings.Contains(adv.String(), "172.16.78.150:49153") {
		t.Errorf("String() = %q, want it to contain the endpoint", adv.String())
	}
}

// TestDecodeAdvertisementRejects pins that packets the framework would ignore
// are ignored here too, instead of producing a bogus endpoint.
func TestDecodeAdvertisementRejects(t *testing.T) {
	good := packAdvertisement(t, ServiceSound, "172.16.78.150", 49152, "SoundService", 1)
	cases := map[string][]byte{
		"empty":          nil,
		"short header":   good[:HeaderSize-1],
		"trailing bytes": append(append([]byte{}, good...), 0),
		"not an advertisement": func() []byte {
			b := append([]byte{}, good...)
			b[1] = MsgData // message type
			return b
		}(),
		"garbage payload": func() []byte {
			b := append([]byte{}, good[:HeaderSize]...)
			return append(b, 0xFF, 0xFE, 0xFD, 0xFC)
		}(),
		"no port": packAdvertisement(t, ServiceFile, "172.16.78.150", 0, "FileService", 1),
		"no ip":   packAdvertisement(t, ServiceFile, "", 49153, "FileService", 1),
		"no sid":  packAdvertisement(t, 0, "172.16.78.150", 49153, "FileService", 1),
	}
	for name, packet := range cases {
		t.Run(name, func(t *testing.T) {
			if adv, ok := decodeAdvertisement(packet); ok {
				t.Fatalf("decoded %+v, want rejection", adv)
			}
		})
	}
}

func TestSummarizeSIDs(t *testing.T) {
	if got := summarizeSIDs(nil); got != "none" {
		t.Errorf("summarizeSIDs(nil) = %q, want \"none\"", got)
	}
	seen := []ServiceAdvertisement{{SID: 13}, {SID: 2}, {SID: 13}, {SID: 12}}
	if got, want := summarizeSIDs(seen), "2,12,13"; got != want {
		t.Errorf("summarizeSIDs = %q, want %q", got, want)
	}
}
