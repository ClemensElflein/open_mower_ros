package tts

import (
	"encoding/binary"
	"encoding/json"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

const testKey = "sk-test-not-a-real-key"

// testWAV returns a minimal 16 kHz mono 16-bit WAV with n silent samples.
func testWAV(n int) []byte {
	data := make([]byte, n*2)
	b := make([]byte, 44+len(data))
	copy(b[0:4], "RIFF")
	binary.LittleEndian.PutUint32(b[4:], uint32(36+len(data)))
	copy(b[8:12], "WAVE")
	copy(b[12:16], "fmt ")
	binary.LittleEndian.PutUint32(b[16:], 16)
	binary.LittleEndian.PutUint16(b[20:], 1)     // PCM
	binary.LittleEndian.PutUint16(b[22:], 1)     // mono
	binary.LittleEndian.PutUint32(b[24:], 16000) // sample rate
	binary.LittleEndian.PutUint32(b[28:], 32000) // byte rate
	binary.LittleEndian.PutUint16(b[32:], 2)     // block align
	binary.LittleEndian.PutUint16(b[34:], 16)    // bits
	copy(b[36:40], "data")
	binary.LittleEndian.PutUint32(b[40:], uint32(len(data)))
	copy(b[44:], data)
	return b
}

func testIdentityForEngine() fwsync.TTSIdentity {
	return fwsync.TTSIdentity{
		Engine:          "elevenlabs",
		Model:           "eleven_v4",
		Voice:           "5gaEGrWSk4v20HMfIhgC",
		Language:        "en",
		Text:            "The robot operating system has started.",
		Bitrate:         "32k",
		LoudNorm:        true,
		Stability:       0.5,
		SimilarityBoost: 0.75,
		Style:           0.0,
		UseSpeakerBoost: true,
	}
}

func TestSynthesizeRequest(t *testing.T) {
	var gotPath, gotQuery, gotKey, gotMethod string
	var gotBody elevenLabsRequest

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		gotMethod, gotPath, gotQuery, gotKey = r.Method, r.URL.Path, r.URL.RawQuery, r.Header.Get("xi-api-key")
		if err := json.NewDecoder(r.Body).Decode(&gotBody); err != nil {
			t.Errorf("decode request: %v", err)
		}
		w.Header().Set("Content-Type", "audio/wav")
		_, _ = w.Write(testWAV(1600))
	}))
	defer srv.Close()

	dst := filepath.Join(t.TempDir(), "speech.wav")
	took, err := ElevenLabs{APIKey: testKey, BaseURL: srv.URL}.Synthesize(testIdentityForEngine(), dst)
	if err != nil {
		t.Fatalf("Synthesize: %v", err)
	}
	if took <= 0 {
		t.Errorf("wall time = %v, want > 0", took)
	}

	if gotMethod != http.MethodPost {
		t.Errorf("method = %s, want POST", gotMethod)
	}
	if want := "/v1/text-to-speech/5gaEGrWSk4v20HMfIhgC"; gotPath != want {
		t.Errorf("path = %q, want %q", gotPath, want)
	}
	if !strings.Contains(gotQuery, "output_format="+ElevenLabsOutputFormat) {
		t.Errorf("query = %q, want output_format=%s", gotQuery, ElevenLabsOutputFormat)
	}
	if gotKey != testKey {
		t.Errorf("xi-api-key = %q, want the configured key", gotKey)
	}
	if gotBody.Text != "The robot operating system has started." || gotBody.ModelID != "eleven_v4" {
		t.Errorf("body = %+v, want text and model_id from the identity", gotBody)
	}
	if gotBody.VoiceSettings == nil {
		t.Fatal("voice_settings missing in the request")
	} else {
		vs := *gotBody.VoiceSettings
		if vs.Stability != 0.5 || vs.SimilarityBoost != 0.75 || vs.Style != 0 || !vs.UseSpeakerBoost {
			t.Errorf("voice_settings = %+v, want the identity values", vs)
		}
	}

	data, err := os.ReadFile(dst)
	if err != nil {
		t.Fatalf("read written wav: %v", err)
	}
	if !isWAV(data) {
		t.Errorf("written file is not a WAV (%d bytes)", len(data))
	}
}

func TestSynthesizeRetriesOn429(t *testing.T) {
	var calls int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if atomic.AddInt32(&calls, 1) == 1 {
			w.WriteHeader(http.StatusTooManyRequests)
			_, _ = io.WriteString(w, `{"detail":"rate limited"}`)
			return
		}
		_, _ = w.Write(testWAV(160))
	}))
	defer srv.Close()

	dst := filepath.Join(t.TempDir(), "speech.wav")
	eng := ElevenLabs{APIKey: testKey, BaseURL: srv.URL, RetryDelay: time.Millisecond}
	if _, err := eng.Synthesize(testIdentityForEngine(), dst); err != nil {
		t.Fatalf("Synthesize after a 429: %v", err)
	}
	if got := atomic.LoadInt32(&calls); got != 2 {
		t.Errorf("requests = %d, want 2 (one retry)", got)
	}
}

func TestSynthesizeDoesNotRetryAuthErrors(t *testing.T) {
	var calls int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		atomic.AddInt32(&calls, 1)
		w.WriteHeader(http.StatusUnauthorized)
		_, _ = io.WriteString(w, `{"detail":{"message":"invalid api key"}}`)
	}))
	defer srv.Close()

	dst := filepath.Join(t.TempDir(), "speech.wav")
	eng := ElevenLabs{APIKey: testKey, BaseURL: srv.URL, RetryDelay: time.Millisecond}
	err := func() error { _, err := eng.Synthesize(testIdentityForEngine(), dst); return err }()
	if err == nil {
		t.Fatal("expected an error for a 401")
	}
	if !strings.Contains(err.Error(), "401") {
		t.Errorf("error = %v, want the status code", err)
	}
	if strings.Contains(err.Error(), testKey) {
		t.Errorf("error leaks the API key: %v", err)
	}
	if got := atomic.LoadInt32(&calls); got != 1 {
		t.Errorf("requests = %d, want 1 (no retry on 401)", got)
	}
}

func TestSynthesizeRejectsNonWAV(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, "<html>error</html>")
	}))
	defer srv.Close()

	dst := filepath.Join(t.TempDir(), "speech.wav")
	eng := ElevenLabs{APIKey: testKey, BaseURL: srv.URL}
	if _, err := eng.Synthesize(testIdentityForEngine(), dst); err == nil {
		t.Fatal("expected an error for a non-WAV response")
	}
	if _, err := os.Stat(dst); err == nil {
		t.Fatal("a non-WAV response must not leave a file behind")
	}
}

func TestAvailableNeedsAKey(t *testing.T) {
	if err := (ElevenLabs{}).Available(); err == nil {
		t.Fatal("Available() without a key must fail")
	}
	if err := (ElevenLabs{APIKey: testKey}).Available(); err != nil {
		t.Fatalf("Available() with a key: %v", err)
	}
}

func TestVoicesListsAndPaginates(t *testing.T) {
	var calls int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/v2/voices" {
			t.Errorf("path = %q, want /v2/voices", r.URL.Path)
		}
		if got := r.Header.Get("xi-api-key"); got != testKey {
			t.Errorf("xi-api-key = %q, want the configured key", got)
		}
		if got := r.URL.Query().Get("voice_type"); got != "default" {
			t.Errorf("voice_type = %q, want default", got)
		}
		if got := r.URL.Query().Get("page_size"); got != "100" {
			t.Errorf("page_size = %q, want 100", got)
		}

		if atomic.AddInt32(&calls, 1) == 1 {
			if got := r.URL.Query().Get("next_page_token"); got != "" {
				t.Errorf("first request carries next_page_token %q", got)
			}
			_, _ = io.WriteString(w, `{"voices":[{"voice_id":"v1","name":"Rachel","category":"premade","labels":{"language":"en"}}],"has_more":true,"next_page_token":"p2"}`)
			return
		}
		if got := r.URL.Query().Get("next_page_token"); got != "p2" {
			t.Errorf("second request next_page_token = %q, want p2", got)
		}
		_, _ = io.WriteString(w, `{"voices":[{"voice_id":"v2","name":"Adam","category":"premade"}],"has_more":false}`)
	}))
	defer srv.Close()

	voices, err := ElevenLabs{APIKey: testKey, BaseURL: srv.URL}.Voices("default")
	if err != nil {
		t.Fatalf("Voices: %v", err)
	}
	if len(voices) != 2 {
		t.Fatalf("got %d voice(s), want 2 (both pages)", len(voices))
	}
	if voices[0].VoiceID != "v1" || voices[0].Name != "Rachel" || voices[0].Labels["language"] != "en" {
		t.Errorf("first voice = %+v", voices[0])
	}
	if voices[1].VoiceID != "v2" {
		t.Errorf("second voice = %+v", voices[1])
	}
}

func TestVoicesRejectsBadStatus(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusUnauthorized)
		_, _ = io.WriteString(w, `{"detail":"bad key"}`)
	}))
	defer srv.Close()

	if _, err := (ElevenLabs{APIKey: testKey, BaseURL: srv.URL}).Voices("default"); err == nil {
		t.Fatal("expected an error for a 401")
	} else if !strings.Contains(err.Error(), "401") {
		t.Errorf("error = %v, want the status code", err)
	}
}
