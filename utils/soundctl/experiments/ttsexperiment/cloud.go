package main

import (
	"crypto/rand"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"fmt"
	"html"
	"io"
	"net/http"
	"net/url"
	"os"
	"strings"
	"time"

	"github.com/gorilla/websocket"
)

const userAgent = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36 Edg/130.0.2849.68"

// synthGoogle uses the keyless Google Translate TTS endpoint. It returns an
// MP3 file, no account and no API key are required.
func synthGoogle(text, lang, dest string) (synthResult, error) {
	q := url.Values{}
	q.Set("ie", "UTF-8")
	q.Set("client", "tw-ob")
	q.Set("tl", lang)
	q.Set("q", text)
	u := "https://translate.google.com/translate_tts?" + q.Encode()

	start := time.Now()
	req, err := http.NewRequest(http.MethodGet, u, nil)
	if err != nil {
		return synthResult{}, err
	}
	req.Header.Set("User-Agent", userAgent)
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		return synthResult{}, err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return synthResult{}, fmt.Errorf("http %d", resp.StatusCode)
	}
	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return synthResult{}, err
	}
	if err := os.WriteFile(dest, body, 0o644); err != nil {
		return synthResult{}, err
	}
	return synthResult{Wall: time.Since(start)}, nil
}

// edgeTrustedToken is the public client token of the Edge "read aloud"
// feature. It is not an account and not an API key.
const edgeTrustedToken = "6A5AA1D4EAFF4E9FB37E23D68491D6F4"

// edgeSecMSGEC builds the Sec-MS-GEC token that Microsoft started to require
// in 2024: SHA256 of the current time (in 100 ns Windows ticks, truncated to
// 5 minutes) plus the trusted client token.
func edgeSecMSGEC() string {
	ticks := time.Now().Unix() + 11644473600 // seconds since 1601-01-01
	ticks -= ticks % 300
	sum := sha256.Sum256([]byte(fmt.Sprintf("%d%s", ticks*10_000_000, edgeTrustedToken)))
	return strings.ToUpper(hex.EncodeToString(sum[:]))
}

func edgeRequestID() string {
	var b [16]byte
	_, _ = rand.Read(b[:])
	b[6] = (b[6] & 0x0f) | 0x40
	b[8] = (b[8] & 0x3f) | 0x80
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:16])
}

// synthEdge uses the Edge "read aloud" WebSocket endpoint, which serves the
// Azure neural voices without any account. It returns an MP3 file.
func synthEdge(voice, text, dest string) (synthResult, error) {
	if len(text) > 4000 {
		text = text[:4000]
	}
	connID := edgeRequestID()
	u := fmt.Sprintf("wss://speech.platform.bing.com/consumer/speech/synthesize/readaloud/edge/v1"+
		"?TrustedClientToken=%s&Sec-MS-GEC=%s&Sec-MS-GEC-Version=1-130.0.2849.68&ConnectionId=%s",
		edgeTrustedToken, edgeSecMSGEC(), connID)

	header := http.Header{}
	header.Set("User-Agent", userAgent)
	header.Set("Origin", "chrome-extension://jdiccldimpdaibmpdkjnbmckianbfold")
	header.Set("Pragma", "no-cache")
	header.Set("Cache-Control", "no-cache")

	start := time.Now()
	dialer := websocket.Dialer{HandshakeTimeout: 30 * time.Second}
	conn, resp, err := dialer.Dial(u, header)
	if err != nil {
		detail := ""
		if resp != nil {
			body, _ := io.ReadAll(io.LimitReader(resp.Body, 400))
			detail = fmt.Sprintf(" (http %d: %s)", resp.StatusCode, strings.TrimSpace(string(body)))
		}
		return synthResult{}, fmt.Errorf("dial: %w%s", err, detail)
	}
	defer conn.Close()
	_ = conn.SetReadDeadline(time.Now().Add(90 * time.Second))

	stamp := time.Now().UTC().Format("Mon Jan 02 2006 15:04:05") + " GMT+0000 (Coordinated Universal Time)"
	cfg := `{"context":{"synthesis":{"audio":{"metadataoptions":{"sentenceBoundaryEnabled":"false","wordBoundaryEnabled":"false"},"outputFormat":"audio-24khz-48kbitrate-mono-mp3"}}}}`
	msgCfg := "X-Timestamp:" + stamp + "\r\nContent-Type:application/json; charset=utf-8\r\nPath:speech.config\r\n\r\n" + cfg
	if err := conn.WriteMessage(websocket.TextMessage, []byte(msgCfg)); err != nil {
		return synthResult{}, fmt.Errorf("write config: %w", err)
	}

	lang := "en-US"
	if i := strings.Index(voice, "-"); i > 0 {
		parts := strings.Split(voice, "-")
		if len(parts) >= 2 {
			lang = parts[0] + "-" + parts[1]
		}
	}
	ssml := fmt.Sprintf("<speak version='1.0' xmlns='http://www.w3.org/2001/10/synthesis' xml:lang='%s'>"+
		"<voice name='%s'><prosody pitch='+0Hz' rate='+0%%' volume='+0%%'>%s</prosody></voice></speak>",
		lang, voice, html.EscapeString(text))
	msgSSML := "X-RequestId:" + connID + "\r\nContent-Type:application/ssml+xml\r\nX-Timestamp:" +
		time.Now().UTC().Format("2006-01-02T15:04:05.000Z") + "Z\r\nPath:ssml\r\n\r\n" + ssml
	if err := conn.WriteMessage(websocket.TextMessage, []byte(msgSSML)); err != nil {
		return synthResult{}, fmt.Errorf("write ssml: %w", err)
	}

	var audio []byte
	for {
		mt, data, err := conn.ReadMessage()
		if err != nil {
			if len(audio) > 0 {
				break // some servers close before turn.end
			}
			return synthResult{}, fmt.Errorf("read: %w", err)
		}
		if mt == websocket.TextMessage {
			if strings.Contains(string(data), "Path:turn.end") {
				break
			}
			continue
		}
		// binary frame: uint16 header length (big endian) + header + payload
		if len(data) < 2 {
			continue
		}
		hlen := int(binary.BigEndian.Uint16(data[:2]))
		if len(data) < 2+hlen {
			continue
		}
		if strings.Contains(string(data[2:2+hlen]), "Path:audio") {
			audio = append(audio, data[2+hlen:]...)
		}
	}
	if len(audio) == 0 {
		return synthResult{}, fmt.Errorf("no audio received")
	}
	if err := os.WriteFile(dest, audio, 0o644); err != nil {
		return synthResult{}, err
	}
	return synthResult{Wall: time.Since(start)}, nil
}
