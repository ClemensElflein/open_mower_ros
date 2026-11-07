package tts

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"os"
	"strings"
	"time"

	"github.com/ClemensElflein/open_mower_ros/utils/soundctl/internal/fwsync"
)

// ElevenLabs renders with the ElevenLabs cloud API
// (POST /v1/text-to-speech/{voice_id}).
//
// The request asks for wav_16000, so ffmpeg only has to normalise and encode: the
// audio never has to be resampled by us because a voice happens to deliver
// 22.05 or 44.1 kHz, and there is only one lossy step (the MP3 the firmware
// plays).
type ElevenLabs struct {
	// APIKey is the xi-api-key header. Available() rejects an empty key.
	APIKey string
	// BaseURL is the API root ("" = DefaultElevenLabsURL). Tests point it at an
	// httptest server; the EU residency host (api.eu.residency.elevenlabs.io)
	// works the same way.
	BaseURL string
	// Client is the HTTP client (nil = one with DefaultElevenLabsTimeout).
	Client *http.Client
	// RetryDelay is the first backoff step (0 = DefaultRetryDelay); tests shrink
	// it so a retry case does not sleep for seconds.
	RetryDelay time.Duration
}

const (
	// DefaultElevenLabsURL is the global API root.
	DefaultElevenLabsURL = "https://api.elevenlabs.io"
	// ElevenLabsOutputFormat is 16 kHz PCM in a WAV container: exactly the rate
	// the firmware needs, so the encoder only has to normalise and encode.
	ElevenLabsOutputFormat = "wav_16000"
	// DefaultElevenLabsTimeout bounds one synthesis request. A short announcement
	// takes about a second; the timeout is generous for a slow uplink.
	DefaultElevenLabsTimeout = 60 * time.Second
	// DefaultRetryDelay is the first backoff step before a retry.
	DefaultRetryDelay = 2 * time.Second
	// maxElevenLabsAttempts bounds the retries on 429/5xx and network errors.
	maxElevenLabsAttempts = 3
)

// Available reports why the engine cannot be used (nil = it can). It never
// touches the network: it runs on every `tts` start, also on the mower at boot,
// where an offline first start must still be able to sync the sounds it has.
func (e ElevenLabs) Available() error {
	if strings.TrimSpace(e.APIKey) == "" {
		return errors.New("no ElevenLabs API key (--api-key, --api-key-file or " +
			"$ELEVENLABS_API_KEY)")
	}
	return nil
}

// Describe renders the voice/settings for the log line.
func (e ElevenLabs) Describe(id fwsync.TTSIdentity) string { return id.String() }

// elevenLabsRequest is the JSON body of a synthesis request. Only the settings
// that are part of the render identity are sent; the server defaults fill the
// rest, and a changed setting re-renders because it changes the identity.
type elevenLabsRequest struct {
	Text          string                   `json:"text"`
	ModelID       string                   `json:"model_id"`
	VoiceSettings *elevenLabsVoiceSettings `json:"voice_settings,omitempty"`
}

// elevenLabsVoiceSettings are the per-request voice settings.
type elevenLabsVoiceSettings struct {
	Stability       float64 `json:"stability"`
	SimilarityBoost float64 `json:"similarity_boost"`
	Style           float64 `json:"style"`
	UseSpeakerBoost bool    `json:"use_speaker_boost"`
}

// Synthesize requests the spoken text and writes the returned WAV to wav. It
// returns the wall time of the request (retries included).
func (e ElevenLabs) Synthesize(id fwsync.TTSIdentity, wav string) (time.Duration, error) {
	payload, err := json.Marshal(elevenLabsRequest{
		Text:    id.Text,
		ModelID: id.Model,
		VoiceSettings: &elevenLabsVoiceSettings{
			Stability:       id.Stability,
			SimilarityBoost: id.SimilarityBoost,
			Style:           id.Style,
			UseSpeakerBoost: id.UseSpeakerBoost,
		},
	})
	if err != nil {
		return 0, fmt.Errorf("elevenlabs: encode request: %w", err)
	}

	start := time.Now()
	attempts := maxElevenLabsAttempts
	var lastErr error
	for attempt := 0; attempt < attempts; attempt++ {
		if attempt > 0 {
			time.Sleep(e.retryDelay() << (attempt - 1))
		}
		err := e.request(id, payload, wav)
		if err == nil {
			return time.Since(start), nil
		}
		lastErr = err
		if !retryable(err) {
			break
		}
	}
	return time.Since(start), lastErr
}

// retryableError marks a failure that is worth another attempt (429, 5xx and
// transport errors); a 401/422 will not get better by retrying.
type retryableError struct{ error }

func retryable(err error) bool {
	var r retryableError
	return errors.As(err, &r)
}

func (e ElevenLabs) retryDelay() time.Duration {
	if e.RetryDelay > 0 {
		return e.RetryDelay
	}
	return DefaultRetryDelay
}

// request performs one synthesis call and writes the WAV.
func (e ElevenLabs) request(id fwsync.TTSIdentity, payload []byte, wav string) error {
	endpoint := strings.TrimRight(e.baseURL(), "/") + "/v1/text-to-speech/" +
		url.PathEscape(strings.TrimSpace(id.Voice)) + "?output_format=" + ElevenLabsOutputFormat

	req, err := http.NewRequest(http.MethodPost, endpoint, strings.NewReader(string(payload)))
	if err != nil {
		return fmt.Errorf("elevenlabs: build request: %w", err)
	}
	req.Header.Set("xi-api-key", strings.TrimSpace(e.APIKey))
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "audio/wav")

	resp, err := e.client().Do(req)
	if err != nil {
		return retryableError{fmt.Errorf("elevenlabs: request: %w", err)}
	}
	defer func() { _ = resp.Body.Close() }()

	body, readErr := io.ReadAll(io.LimitReader(resp.Body, 32<<20))
	if resp.StatusCode != http.StatusOK {
		err := fmt.Errorf("elevenlabs: %s: %s", resp.Status, errorText(body))
		if resp.StatusCode == http.StatusTooManyRequests || resp.StatusCode >= 500 {
			return retryableError{err}
		}
		return err
	}
	if readErr != nil {
		return retryableError{fmt.Errorf("elevenlabs: read response: %w", readErr)}
	}
	if !isWAV(body) {
		return fmt.Errorf("elevenlabs: response is not a WAV (%d bytes: %s)", len(body), errorText(body))
	}
	_ = os.Remove(wav)
	if err := os.WriteFile(wav, body, 0o644); err != nil {
		return fmt.Errorf("elevenlabs: write %s: %w", wav, err)
	}
	return nil
}

func (e ElevenLabs) baseURL() string {
	if strings.TrimSpace(e.BaseURL) != "" {
		return e.BaseURL
	}
	return DefaultElevenLabsURL
}

func (e ElevenLabs) client() *http.Client {
	if e.Client != nil {
		return e.Client
	}
	return &http.Client{Timeout: DefaultElevenLabsTimeout}
}

// Voice is one entry of GET /v2/voices.
type Voice struct {
	VoiceID  string            `json:"voice_id"`
	Name     string            `json:"name"`
	Category string            `json:"category"`
	Labels   map[string]string `json:"labels"`
}

// voicesPage is the paginated response of GET /v2/voices.
type voicesPage struct {
	Voices        []Voice `json:"voices"`
	HasMore       bool    `json:"has_more"`
	NextPageToken string  `json:"next_page_token"`
}

// Voices lists the voices of the account. voiceType filters server side:
// "default" are the built-in premade voices (the only ones a free plan may use
// through the API), "community" the Voice Library ones, "" everything.
func (e ElevenLabs) Voices(voiceType string) ([]Voice, error) {
	if err := e.Available(); err != nil {
		return nil, err
	}
	var all []Voice
	page := 0
	var token string
	for {
		query := url.Values{}
		query.Set("page_size", "100")
		if voiceType != "" {
			query.Set("voice_type", voiceType)
		}
		if page > 0 && token != "" {
			query.Set("next_page_token", token)
		}
		body, err := e.get("/v2/voices?" + query.Encode())
		if err != nil {
			return nil, err
		}
		var resp voicesPage
		if err := json.Unmarshal(body, &resp); err != nil {
			return nil, fmt.Errorf("elevenlabs: list voices: %w (%s)", err, errorText(body))
		}
		all = append(all, resp.Voices...)
		if !resp.HasMore || resp.NextPageToken == "" || resp.NextPageToken == token {
			return all, nil
		}
		token = resp.NextPageToken
		page++
	}
}

// get performs a GET against the API and returns the body.
func (e ElevenLabs) get(pathAndQuery string) ([]byte, error) {
	endpoint := strings.TrimRight(e.baseURL(), "/") + pathAndQuery
	req, err := http.NewRequest(http.MethodGet, endpoint, nil)
	if err != nil {
		return nil, fmt.Errorf("elevenlabs: build request: %w", err)
	}
	req.Header.Set("xi-api-key", strings.TrimSpace(e.APIKey))
	req.Header.Set("Accept", "application/json")

	resp, err := e.client().Do(req)
	if err != nil {
		return nil, fmt.Errorf("elevenlabs: request: %w", err)
	}
	defer func() { _ = resp.Body.Close() }()
	body, readErr := io.ReadAll(io.LimitReader(resp.Body, 8<<20))
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("elevenlabs: %s: %s", resp.Status, errorText(body))
	}
	if readErr != nil {
		return nil, fmt.Errorf("elevenlabs: read response: %w", readErr)
	}
	return body, nil
}

// isWAV checks the RIFF/WAVE magic: the encoder would fail on anything else, but
// with a much less helpful message.
func isWAV(b []byte) bool {
	return len(b) > 12 && string(b[0:4]) == "RIFF" && string(b[8:12]) == "WAVE"
}

// errorText shortens and flattens an error body for the log line. The API key is
// sent in a header, never in the body, so nothing secret can leak here.
func errorText(body []byte) string {
	s := strings.Join(strings.Fields(string(body)), " ")
	if s == "" {
		return "no body"
	}
	if len(s) > 300 {
		s = s[:300] + "..."
	}
	return s
}
