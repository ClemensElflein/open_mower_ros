package tts

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/url"
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

// api is the shared HTTP plumbing (api.go), configured for Cartesia: a Bearer
// token plus the dated API version header the request format is written against.
func (c Cartesia) api() apiClient {
	key := strings.TrimSpace(c.APIKey)
	return apiClient{
		Name:       "cartesia",
		BaseURL:    c.BaseURL,
		DefaultURL: DefaultCartesiaURL,
		Auth: func(req *http.Request) {
			req.Header.Set("Authorization", "Bearer "+key)
			req.Header.Set("Cartesia-Version", CartesiaAPIVersion)
		},
		Client:     c.Client,
		Timeout:    DefaultCartesiaTimeout,
		RetryDelay: c.RetryDelay,
	}
}

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
func (c Cartesia) Synthesize(ctx context.Context, id fwsync.TTSIdentity, wav string) (time.Duration, error) {
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
		return 0, c.api().errorf("encode request: %w", err)
	}

	start := time.Now()
	body, err := c.api().do(ctx, apiRequest{
		Method: http.MethodPost,
		Path:   "/tts/bytes",
		Body:   payload,
		Accept: "audio/wav",
	})
	if err != nil {
		return time.Since(start), err
	}
	if !isWAV(body) {
		return time.Since(start), c.api().errorf("response is not a WAV (%d bytes: %s)",
			len(body), errorText(body))
	}
	if err := writeWAV(c.api().Name, wav, body); err != nil {
		return time.Since(start), err
	}
	return time.Since(start), nil
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
func (c Cartesia) Voices(ctx context.Context) ([]CartesiaVoice, error) {
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
		body, err := c.api().do(ctx, apiRequest{
			Method:  http.MethodGet,
			Path:    "/voices?" + query.Encode(),
			Accept:  "application/json",
			MaxBody: 8 << 20,
		})
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
