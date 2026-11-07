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

// Cartesia renders with the Cartesia cloud API (POST /tts/bytes).
//
// The request asks for 16 kHz PCM in a WAV container - exactly the rate the
// firmware plays - so the encoder only normalises the loudness and writes the
// MP3; nothing is resampled twice.
type Cartesia struct {
	// APIKey is the Bearer token (sk_car_...). Available() rejects an empty key.
	APIKey string
	// BaseURL is the API root ("" = DefaultCartesiaURL).
	BaseURL string
	// Client is the HTTP client (nil = one with DefaultCartesiaTimeout).
	Client *http.Client
	// RetryDelay is the first backoff step (0 = DefaultRetryDelay); tests shrink
	// it so a retry case does not sleep for seconds.
	RetryDelay time.Duration
}

const (
	// DefaultCartesiaURL is the global API root.
	DefaultCartesiaURL = "https://api.cartesia.ai"
	// CartesiaAPIVersion is the required Cartesia-Version header. The API is
	// versioned by date; this is the version the request format below is written
	// against.
	CartesiaAPIVersion = "2026-08-14"
	// CartesiaSampleRate is 16 kHz, the rate the firmware plays. Asking for it
	// keeps the resampling out of our pipeline (the encoder only normalises and
	// encodes).
	CartesiaSampleRate = 16000
	// DefaultCartesiaTimeout bounds one synthesis request.
	DefaultCartesiaTimeout = 60 * time.Second
	// maxCartesiaAttempts bounds the retries on 429/5xx and network errors.
	maxCartesiaAttempts = 3
)

// Available reports why the engine cannot be used (nil = it can). It never
// touches the network: it runs on every `tts` start, also on the mower at boot.
func (c Cartesia) Available() error {
	if strings.TrimSpace(c.APIKey) == "" {
		return errors.New("no Cartesia API key (--api-key, --api-key-file or " +
			"$CARTESIA_API_KEY)")
	}
	return nil
}

// Describe renders the voice/settings for the log line.
func (c Cartesia) Describe(id fwsync.TTSIdentity) string { return id.String() }

// cartesiaRequest is the JSON body of a synthesis request. Only what belongs to
// the render identity is sent; the server defaults fill the rest, and a changed
// setting re-renders because it changes the identity.
type cartesiaRequest struct {
	ModelID      string               `json:"model_id"`
	Transcript   string               `json:"transcript"`
	Voice        string               `json:"voice"`
	Language     string               `json:"language,omitempty"`
	OutputFormat cartesiaOutputFormat `json:"output_format"`
}

// cartesiaOutputFormat asks for WAV/pcm_s16le at the firmware's rate.
type cartesiaOutputFormat struct {
	Container  string `json:"container"`
	Encoding   string `json:"encoding"`
	SampleRate int    `json:"sample_rate"`
}

// Synthesize requests the spoken text and writes the returned WAV to wav. It
// returns the wall time of the request (retries included).
func (c Cartesia) Synthesize(id fwsync.TTSIdentity, wav string) (time.Duration, error) {
	payload, err := json.Marshal(cartesiaRequest{
		ModelID:    id.Model,
		Transcript: id.Text,
		Voice:      strings.TrimSpace(id.Voice),
		Language:   id.Language,
		OutputFormat: cartesiaOutputFormat{
			Container:  "wav",
			Encoding:   "pcm_s16le",
			SampleRate: CartesiaSampleRate,
		},
	})
	if err != nil {
		return 0, fmt.Errorf("cartesia: encode request: %w", err)
	}

	start := time.Now()
	var lastErr error
	for attempt := 0; attempt < maxCartesiaAttempts; attempt++ {
		if attempt > 0 {
			time.Sleep(c.retryDelay() << (attempt - 1))
		}
		err := c.request(payload, wav)
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

func (c Cartesia) retryDelay() time.Duration {
	if c.RetryDelay > 0 {
		return c.RetryDelay
	}
	return DefaultRetryDelay
}

// request performs one synthesis call and writes the WAV.
func (c Cartesia) request(payload []byte, wav string) error {
	endpoint := strings.TrimRight(c.baseURL(), "/") + "/tts/bytes"
	req, err := http.NewRequest(http.MethodPost, endpoint, strings.NewReader(string(payload)))
	if err != nil {
		return fmt.Errorf("cartesia: build request: %w", err)
	}
	req.Header.Set("Authorization", "Bearer "+strings.TrimSpace(c.APIKey))
	req.Header.Set("Cartesia-Version", CartesiaAPIVersion)
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "audio/wav")

	resp, err := c.client().Do(req)
	if err != nil {
		return retryableError{fmt.Errorf("cartesia: request: %w", err)}
	}
	defer func() { _ = resp.Body.Close() }()

	body, readErr := io.ReadAll(io.LimitReader(resp.Body, 32<<20))
	if resp.StatusCode != http.StatusOK {
		err := fmt.Errorf("cartesia: %s: %s", resp.Status, errorText(body))
		// 429 and 5xx may pass; 400/401/402 will not get better by retrying.
		if resp.StatusCode == http.StatusTooManyRequests || resp.StatusCode >= 500 {
			return retryableError{err}
		}
		return err
	}
	if readErr != nil {
		return retryableError{fmt.Errorf("cartesia: read response: %w", readErr)}
	}
	if !isWAV(body) {
		return fmt.Errorf("cartesia: response is not a WAV (%d bytes: %s)", len(body), errorText(body))
	}
	_ = os.Remove(wav)
	if err := os.WriteFile(wav, body, 0o644); err != nil {
		return fmt.Errorf("cartesia: write %s: %w", wav, err)
	}
	return nil
}

func (c Cartesia) baseURL() string {
	if strings.TrimSpace(c.BaseURL) != "" {
		return c.BaseURL
	}
	return DefaultCartesiaURL
}

func (c Cartesia) client() *http.Client {
	if c.Client != nil {
		return c.Client
	}
	return &http.Client{Timeout: DefaultCartesiaTimeout}
}

// CartesiaVoice is one entry of GET /voices.
type CartesiaVoice struct {
	ID          string `json:"id"`
	Name        string `json:"name"`
	Language    string `json:"language"`
	Gender      string `json:"gender"`
	Description string `json:"description"`
}

// cartesiaVoicesPage is the paginated response of GET /voices.
type cartesiaVoicesPage struct {
	Data     []CartesiaVoice `json:"data"`
	HasMore  bool            `json:"has_more"`
	NextPage string          `json:"next_page"`
}

// Voices lists the voices this key can use (id, name, language, description).
func (c Cartesia) Voices() ([]CartesiaVoice, error) {
	if err := c.Available(); err != nil {
		return nil, err
	}
	var all []CartesiaVoice
	cursor := ""
	for {
		query := url.Values{}
		query.Set("limit", "100")
		if cursor != "" {
			query.Set("starting_after", cursor)
		}
		body, err := c.get("/voices?" + query.Encode())
		if err != nil {
			return nil, err
		}
		var page cartesiaVoicesPage
		if err := json.Unmarshal(body, &page); err != nil {
			return nil, fmt.Errorf("cartesia: list voices: %w (%s)", err, errorText(body))
		}
		all = append(all, page.Data...)
		if !page.HasMore || page.NextPage == "" || page.NextPage == cursor {
			return all, nil
		}
		cursor = page.NextPage
	}
}

// get performs one GET against the API and returns the body.
func (c Cartesia) get(pathAndQuery string) ([]byte, error) {
	endpoint := strings.TrimRight(c.baseURL(), "/") + pathAndQuery
	req, err := http.NewRequest(http.MethodGet, endpoint, nil)
	if err != nil {
		return nil, fmt.Errorf("cartesia: build request: %w", err)
	}
	req.Header.Set("Authorization", "Bearer "+strings.TrimSpace(c.APIKey))
	req.Header.Set("Cartesia-Version", CartesiaAPIVersion)

	resp, err := c.client().Do(req)
	if err != nil {
		return nil, fmt.Errorf("cartesia: request: %w", err)
	}
	defer func() { _ = resp.Body.Close() }()

	body, readErr := io.ReadAll(io.LimitReader(resp.Body, 8<<20))
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("cartesia: %s: %s", resp.Status, errorText(body))
	}
	if readErr != nil {
		return nil, fmt.Errorf("cartesia: read response: %w", readErr)
	}
	return body, nil
}
