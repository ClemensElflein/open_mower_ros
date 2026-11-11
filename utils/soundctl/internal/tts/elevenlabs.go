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

// api is the shared HTTP plumbing (api.go), configured for ElevenLabs: the key
// travels in the xi-api-key header.
func (e ElevenLabs) api() apiClient {
	key := strings.TrimSpace(e.APIKey)
	return apiClient{
		Name:       "elevenlabs",
		BaseURL:    e.BaseURL,
		DefaultURL: DefaultElevenLabsURL,
		Auth: func(req *http.Request) {
			req.Header.Set("xi-api-key", key)
		},
		Client:     e.Client,
		Timeout:    DefaultElevenLabsTimeout,
		RetryDelay: e.RetryDelay,
	}
}

// elevenLabsRequest is the JSON body of a synthesis request. Only the settings
// that are part of the render identity are sent; the server defaults fill the
// rest, and a changed setting re-renders because it changes the identity.
//
// language_code is the identity's Language (ISO 639-1). Without it the model
// guesses the language from the text, which is unreliable for the short
// announcements. The field is advisory: the API ignores a
// code the model does not support, and multilingual_v2 does not accept it at all.
type elevenLabsRequest struct {
	Text          string                   `json:"text"`
	ModelID       string                   `json:"model_id"`
	LanguageCode  string                   `json:"language_code,omitempty"`
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
func (e ElevenLabs) Synthesize(ctx context.Context, id fwsync.TTSIdentity, wav string) (time.Duration, error) {
	payload, err := json.Marshal(elevenLabsRequest{
		Text:         id.Text,
		ModelID:      id.Model,
		LanguageCode: id.Language,
		VoiceSettings: &elevenLabsVoiceSettings{
			Stability:       id.Stability,
			SimilarityBoost: id.SimilarityBoost,
			Style:           id.Style,
			UseSpeakerBoost: id.UseSpeakerBoost,
		},
	})
	if err != nil {
		return 0, e.api().errorf("encode request: %w", err)
	}

	start := time.Now()
	body, err := e.api().do(ctx, apiRequest{
		Method: http.MethodPost,
		Path: "/v1/text-to-speech/" + url.PathEscape(strings.TrimSpace(id.Voice)) +
			"?output_format=" + ElevenLabsOutputFormat,
		Body:   payload,
		Accept: "audio/wav",
	})
	if err != nil {
		return time.Since(start), err
	}
	if !isWAV(body) {
		return time.Since(start), e.api().errorf("response is not a WAV (%d bytes: %s)",
			len(body), errorText(body))
	}
	if err := writeWAV(e.api().Name, wav, body); err != nil {
		return time.Since(start), err
	}
	return time.Since(start), nil
}

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
func (e ElevenLabs) Voices(ctx context.Context, voiceType string) ([]Voice, error) {
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
		body, err := e.api().do(ctx, apiRequest{
			Method:  http.MethodGet,
			Path:    "/v2/voices?" + query.Encode(),
			Accept:  "application/json",
			MaxBody: 8 << 20,
		})
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
