package tts

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"strings"
	"time"
)

// This file is the HTTP plumbing the two hosted engines share. Everything engine
// specific (endpoint, payload, which header carries the key) stays in
// elevenlabs.go and cartesia.go; the retry policy, the status classification, the
// body limit and the error wording live here once.

const (
	// DefaultRetryDelay is the first backoff step before a retry.
	DefaultRetryDelay = 2 * time.Second
	// maxAttempts bounds the retries on 429/5xx and network errors.
	maxAttempts = 3
	// defaultMaxBody bounds a response when a request does not say otherwise.
	defaultMaxBody = 32 << 20
)

// apiClient is one hosted engine's HTTP client. The zero BaseURL falls back to
// DefaultURL so a test can point the engine at an httptest server.
type apiClient struct {
	// Name prefixes every error message ("elevenlabs: ...").
	Name string
	// BaseURL is the API root ("" = DefaultURL).
	BaseURL string
	// DefaultURL is the root of the hosted API.
	DefaultURL string
	// Auth sets the credential headers on a request.
	Auth func(req *http.Request)
	// Client is the HTTP client (nil = one with Timeout).
	Client *http.Client
	// Timeout bounds one request when Client is nil.
	Timeout time.Duration
	// RetryDelay is the first backoff step (0 = DefaultRetryDelay). Tests shrink
	// it so a retry case does not sleep for seconds.
	RetryDelay time.Duration
}

// apiRequest is one call. Path carries the query string already encoded, so the
// engine file stays the only place that knows the API's shape.
type apiRequest struct {
	Method  string
	Path    string
	Body    []byte
	Accept  string
	MaxBody int64
}

// do performs the call and returns the body, retrying transport errors, 429 and
// 5xx. The retry sleep is context aware, so Ctrl-C or a dead-line aborts a render
// instead of sitting out the backoff.
func (a apiClient) do(ctx context.Context, r apiRequest) ([]byte, error) {
	var lastErr error
	for attempt := 0; attempt < maxAttempts; attempt++ {
		if attempt > 0 {
			if err := sleep(ctx, a.retryDelay()<<(attempt-1)); err != nil {
				return nil, err
			}
		}
		body, err := a.once(ctx, r)
		if err == nil {
			return body, nil
		}
		lastErr = err
		if !retryable(err) {
			break
		}
	}
	return nil, lastErr
}

// once performs a single attempt.
func (a apiClient) once(ctx context.Context, r apiRequest) ([]byte, error) {
	endpoint := strings.TrimRight(a.baseURL(), "/") + r.Path
	var payload io.Reader
	if r.Body != nil {
		payload = bytes.NewReader(r.Body)
	}
	req, err := http.NewRequestWithContext(ctx, r.Method, endpoint, payload)
	if err != nil {
		return nil, a.errorf("build request: %w", err)
	}
	if a.Auth != nil {
		a.Auth(req)
	}
	if r.Body != nil {
		req.Header.Set("Content-Type", "application/json")
	}
	if r.Accept != "" {
		req.Header.Set("Accept", r.Accept)
	}

	resp, err := a.httpClient().Do(req)
	if err != nil {
		return nil, retryableError{a.errorf("request: %w", err)}
	}
	defer func() { _ = resp.Body.Close() }()

	limit := r.MaxBody
	if limit <= 0 {
		limit = defaultMaxBody
	}
	body, readErr := io.ReadAll(io.LimitReader(resp.Body, limit))
	if resp.StatusCode != http.StatusOK {
		// 429 and 5xx may pass; 400/401/402 will not get better by retrying.
		err := a.errorf("%s: %s", resp.Status, errorText(body))
		if resp.StatusCode == http.StatusTooManyRequests || resp.StatusCode >= 500 {
			return nil, retryableError{err}
		}
		return nil, err
	}
	if readErr != nil {
		return nil, retryableError{a.errorf("read response: %w", readErr)}
	}
	return body, nil
}

// errorf prefixes the engine name, so a failure names which API was called.
func (a apiClient) errorf(format string, args ...any) error {
	return fmt.Errorf(a.Name+": "+format, args...)
}

func (a apiClient) baseURL() string {
	if strings.TrimSpace(a.BaseURL) != "" {
		return a.BaseURL
	}
	return a.DefaultURL
}

func (a apiClient) httpClient() *http.Client {
	if a.Client != nil {
		return a.Client
	}
	return &http.Client{Timeout: a.Timeout}
}

func (a apiClient) retryDelay() time.Duration {
	if a.RetryDelay > 0 {
		return a.RetryDelay
	}
	return DefaultRetryDelay
}

// retryableError marks a failure that is worth another attempt (429, 5xx and
// transport errors); a 401/422 will not get better by retrying.
type retryableError struct{ error }

func retryable(err error) bool {
	var r retryableError
	return errors.As(err, &r)
}

// sleep waits d or until ctx is done.
func sleep(ctx context.Context, d time.Duration) error {
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}

// writeWAV stores a synthesised response. The destination is removed first: a
// leftover file from an earlier attempt must not survive as "the render".
func writeWAV(name, wav string, body []byte) error {
	_ = os.Remove(wav)
	if err := os.WriteFile(wav, body, 0o644); err != nil {
		return fmt.Errorf("%s: write %s: %w", name, wav, err)
	}
	return nil
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
