package tts

import (
	"encoding/json"
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

const testCartesiaKey = "sk_car_test_not_a_real_key"

func cartesiaIdentity() fwsync.TTSIdentity {
	return fwsync.TTSIdentity{
		Engine:   fwsync.TTSEngineCartesia,
		Model:    fwsync.DefaultCartesiaModel,
		Voice:    "b7187e84-fe22-4344-ba4a-bc013fcb533e",
		Language: "de",
		Text:     "Das Robot Operating System wurde gestartet.",
		Bitrate:  "32k",
		LoudNorm: true,
	}
}

func TestCartesiaSynthesizeRequest(t *testing.T) {
	var gotPath, gotAuth, gotVersion, gotContentType string
	var gotBody cartesiaRequest

	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		gotPath = r.URL.Path
		gotAuth = r.Header.Get("Authorization")
		gotVersion = r.Header.Get("Cartesia-Version")
		gotContentType = r.Header.Get("Content-Type")
		if err := json.NewDecoder(r.Body).Decode(&gotBody); err != nil {
			t.Errorf("decode body: %v", err)
		}
		w.Header().Set("Content-Type", "audio/wav")
		_, _ = w.Write(testWAV(160))
	}))
	defer srv.Close()

	wav := filepath.Join(t.TempDir(), "out.wav")
	engine := Cartesia{APIKey: testCartesiaKey, BaseURL: srv.URL}
	if _, err := engine.Synthesize(cartesiaIdentity(), wav); err != nil {
		t.Fatalf("Synthesize: %v", err)
	}

	if gotPath != "/tts/bytes" {
		t.Errorf("path = %q, want /tts/bytes", gotPath)
	}
	if gotAuth != "Bearer "+testCartesiaKey {
		t.Errorf("Authorization = %q", gotAuth)
	}
	if gotVersion != CartesiaAPIVersion {
		t.Errorf("Cartesia-Version = %q, want %q", gotVersion, CartesiaAPIVersion)
	}
	if gotContentType != "application/json" {
		t.Errorf("Content-Type = %q", gotContentType)
	}
	if gotBody.ModelID != fwsync.DefaultCartesiaModel || gotBody.Transcript != cartesiaIdentity().Text ||
		gotBody.Voice != cartesiaIdentity().Voice || gotBody.Language != "de" {
		t.Errorf("body = %+v", gotBody)
	}
	// 16 kHz keeps the resampling out of our pipeline (the firmware plays 16 kHz).
	if gotBody.OutputFormat.Container != "wav" || gotBody.OutputFormat.Encoding != "pcm_s16le" ||
		gotBody.OutputFormat.SampleRate != 16000 {
		t.Errorf("output_format = %+v", gotBody.OutputFormat)
	}
	if info, err := os.Stat(wav); err != nil || info.Size() == 0 {
		t.Errorf("wav not written: %v", err)
	}
}

func TestCartesiaRetriesOn429(t *testing.T) {
	var calls int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if atomic.AddInt32(&calls, 1) == 1 {
			http.Error(w, `{"error":"rate limited"}`, http.StatusTooManyRequests)
			return
		}
		w.Header().Set("Content-Type", "audio/wav")
		_, _ = w.Write(testWAV(160))
	}))
	defer srv.Close()

	wav := filepath.Join(t.TempDir(), "out.wav")
	engine := Cartesia{APIKey: testCartesiaKey, BaseURL: srv.URL, RetryDelay: time.Millisecond}
	if _, err := engine.Synthesize(cartesiaIdentity(), wav); err != nil {
		t.Fatalf("Synthesize after retry: %v", err)
	}
	if got := atomic.LoadInt32(&calls); got != 2 {
		t.Fatalf("calls = %d, want 2", got)
	}
}

func TestCartesiaDoesNotRetryPaymentRequired(t *testing.T) {
	var calls int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		atomic.AddInt32(&calls, 1)
		http.Error(w, `{"error":"insufficient credits"}`, http.StatusPaymentRequired)
	}))
	defer srv.Close()

	engine := Cartesia{APIKey: testCartesiaKey, BaseURL: srv.URL, RetryDelay: time.Millisecond}
	_, err := engine.Synthesize(cartesiaIdentity(), filepath.Join(t.TempDir(), "out.wav"))
	if err == nil {
		t.Fatal("expected an error for 402")
	}
	if !strings.Contains(err.Error(), "insufficient credits") {
		t.Errorf("error should carry the body: %v", err)
	}
	if got := atomic.LoadInt32(&calls); got != 1 {
		t.Fatalf("calls = %d, want 1 (no retry on 402)", got)
	}
}

func TestCartesiaRejectsNonWAV(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = w.Write([]byte(`{"error":"not audio"}`))
	}))
	defer srv.Close()

	engine := Cartesia{APIKey: testCartesiaKey, BaseURL: srv.URL}
	if _, err := engine.Synthesize(cartesiaIdentity(), filepath.Join(t.TempDir(), "out.wav")); err == nil {
		t.Fatal("a non-WAV body must be rejected")
	}
}

func TestCartesiaAvailableNeedsAKey(t *testing.T) {
	if err := (Cartesia{}).Available(); err == nil {
		t.Fatal("an empty key must be reported")
	}
	if err := (Cartesia{APIKey: testCartesiaKey}).Available(); err != nil {
		t.Fatalf("Available with a key: %v", err)
	}
}

func TestCartesiaVoicesPaginate(t *testing.T) {
	var calls int32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/voices" {
			t.Errorf("path = %q", r.URL.Path)
		}
		if r.Header.Get("Cartesia-Version") != CartesiaAPIVersion {
			t.Errorf("missing version header")
		}
		if atomic.AddInt32(&calls, 1) == 1 {
			if r.URL.Query().Get("starting_after") != "" {
				t.Errorf("first page must not carry a cursor")
			}
			_, _ = w.Write([]byte(`{"data":[{"id":"v1","name":"Sebastian","language":"de","description":"Warm male"}],"has_more":true,"next_page":"v1"}`))
			return
		}
		if r.URL.Query().Get("starting_after") != "v1" {
			t.Errorf("second page cursor = %q", r.URL.Query().Get("starting_after"))
		}
		_, _ = w.Write([]byte(`{"data":[{"id":"v2","name":"Anna","language":"de"}],"has_more":false}`))
	}))
	defer srv.Close()

	voices, err := Cartesia{APIKey: testCartesiaKey, BaseURL: srv.URL}.Voices()
	if err != nil {
		t.Fatalf("Voices: %v", err)
	}
	if len(voices) != 2 || voices[0].ID != "v1" || voices[1].Name != "Anna" {
		t.Fatalf("voices = %+v", voices)
	}
}

func TestCartesiaVoicesRejectBadStatus(t *testing.T) {
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		http.Error(w, `{"error":"unauthorized"}`, http.StatusUnauthorized)
	}))
	defer srv.Close()

	if _, err := (Cartesia{APIKey: testCartesiaKey, BaseURL: srv.URL}).Voices(); err == nil {
		t.Fatal("a 401 must be reported")
	}
}
