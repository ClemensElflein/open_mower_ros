package tts

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"time"
)

// DefaultModel is the Supertonic 3 model directory the image ships.
const DefaultModel = "sherpa-onnx-supertonic-3-tts-int8-2026-05-11"

// sherpaVersion is the sherpa-onnx release the binaries come from. It is pinned
// together with the checksums below, so an upstream change can never sneak into
// an image build unnoticed.
const sherpaVersion = "2.0.0"

const releaseBase = "https://github.com/k2-fsa/sherpa-onnx/releases/download/v" + sherpaVersion + "/"
const modelBase = "https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/"

// archive is one downloadable asset with its pinned checksum.
type archive struct {
	Name   string
	URL    string
	SHA256 string
}

// sherpaArchives maps GOARCH to the shared build of the TTS CLI. Shared (not
// static) keeps the download at ~28 MB instead of ~90 MB; Sherpa.LibDir makes
// the loader find the bundled libonnxruntime.
var sherpaArchives = map[string]archive{
	"amd64": {
		Name:   "sherpa-onnx-v" + sherpaVersion + "-linux-x64-shared.tar.bz2",
		URL:    releaseBase + "sherpa-onnx-v" + sherpaVersion + "-linux-x64-shared.tar.bz2",
		SHA256: "ee729caa387ede28fb62dd0c111e267da5ed436c9438ee502da9ee0a2e03ab7c",
	},
	"arm64": {
		Name:   "sherpa-onnx-v" + sherpaVersion + "-linux-aarch64-shared-cpu.tar.bz2",
		URL:    releaseBase + "sherpa-onnx-v" + sherpaVersion + "-linux-aarch64-shared-cpu.tar.bz2",
		SHA256: "0ee6e6a6de388f9db6f24c241ee6631c7509d47f3c8e5cd7b05259ff92f722b6",
	},
}

// modelArchive is the Supertonic 3 int8 model (one archive for all CPUs).
var modelArchive = archive{
	Name:   DefaultModel + ".tar.bz2",
	URL:    modelBase + DefaultModel + ".tar.bz2",
	SHA256: "82fa96f91c4ef8abaae3a14a3f4153facf88bed821d1f7331cec2700f432c427",
}

// Manifest is written to <dir>/install.json and describes what was installed.
type Manifest struct {
	Version   int    `json:"version"`
	Arch      string `json:"arch"`
	Binary    string `json:"binary"`
	SHA256    string `json:"sha256"`
	Model     string `json:"model"`
	ModelSHA  string `json:"model_sha256"`
	Installed string `json:"installed"`
}

// InstallOptions configures Install.
type InstallOptions struct {
	Dir   string // target directory ("/opt/open_mower_ros/tts" in the image)
	Arch  string // "x64"/"aarch64"/"amd64"/"arm64"; "" = runtime.GOARCH
	Model string // model directory name; "" = DefaultModel
	Force bool   // re-download and re-extract even if the files are present
	Log   Logf
	// BaseURL replaces the GitHub release host (mirror for air-gapped builds).
	BaseURL string
}

func (o InstallOptions) logf(format string, args ...any) {
	if o.Log != nil {
		o.Log(format, args...)
	}
}

// normalizeArch maps the many spellings of an architecture onto the two names
// sherpa uses in its asset file names.
func normalizeArch(arch string) (string, error) {
	switch arch {
	case "":
		arch = runtime.GOARCH
	case "x86_64", "x64":
		arch = "amd64"
	case "aarch64", "arm64":
		arch = "arm64"
	}
	switch arch {
	case "amd64", "arm64":
		return arch, nil
	}
	return "", fmt.Errorf("unsupported architecture %q (supported: amd64, arm64)", arch)
}

// Install downloads, verifies and unpacks the TTS engine and model into
// opts.Dir. It is what the image build runs; it only touches files below Dir.
func Install(opts InstallOptions) error {
	if opts.Dir == "" {
		return errors.New("no install directory")
	}
	arch, err := normalizeArch(opts.Arch)
	if err != nil {
		return err
	}
	model := opts.Model
	if model == "" {
		model = DefaultModel
	}

	binDir := filepath.Join(opts.Dir, "bin")
	modelDir := filepath.Join(opts.Dir, "models", model)
	binAsset := withBase(sherpaArchives[arch], opts.BaseURL)
	modelAsset := withBase(modelArchive, opts.BaseURL)

	// The archives are ~28 MB and ~129 MB; keep them in a cache directory next
	// to the target so a re-install (or a second arch) does not download again.
	cacheDir := filepath.Join(opts.Dir, ".cache")
	if err := os.MkdirAll(cacheDir, 0o755); err != nil {
		return err
	}

	if opts.Force || !fileExists(filepath.Join(binDir, "sherpa-onnx-offline-tts")) {
		opts.logf("installing %s (%s)", binAsset.Name, arch)
		path, err := fetch(binAsset, cacheDir, opts.logf)
		if err != nil {
			return err
		}
		// Only bin/ and lib/ are needed; the archive also carries headers.
		keep := func(rel string) bool {
			return strings.HasPrefix(rel, "bin/") || strings.HasPrefix(rel, "lib/")
		}
		if err := extractTarBz2(path, opts.Dir, keep); err != nil {
			return fmt.Errorf("unpack %s: %w", binAsset.Name, err)
		}
		_ = os.Remove(path)
	} else {
		opts.logf("engine present, skipping (%s)", binDir)
	}

	if opts.Force || !fileExists(filepath.Join(modelDir, "tts.json")) {
		opts.logf("installing %s (model)", modelAsset.Name)
		path, err := fetch(modelAsset, cacheDir, opts.logf)
		if err != nil {
			return err
		}
		if err := extractTarBz2(path, modelDir, nil); err != nil {
			return fmt.Errorf("unpack %s: %w", modelAsset.Name, err)
		}
		_ = os.Remove(path)
	} else {
		opts.logf("model present, skipping (%s)", modelDir)
	}

	man := Manifest{
		Version:   1,
		Arch:      arch,
		Binary:    binAsset.Name,
		SHA256:    binAsset.SHA256,
		Model:     model,
		ModelSHA:  modelAsset.SHA256,
		Installed: time.Now().UTC().Format(time.RFC3339),
	}
	raw, err := json.MarshalIndent(man, "", "  ")
	if err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(opts.Dir, "install.json"), append(raw, '\n'), 0o644); err != nil {
		return err
	}
	_ = os.RemoveAll(cacheDir)

	if err := Verify(opts.Dir, model); err != nil {
		return err
	}
	opts.logf("tts assets ready in %s", opts.Dir)
	return nil
}

// Verify checks that a directory holds a usable installation (no downloads, no
// engine run). It is used after Install, by "soundctl tts verify" and by the
// image build.
func Verify(dir, model string) error {
	if model == "" {
		model = DefaultModel
	}
	var errs []error
	for _, p := range []string{
		filepath.Join(dir, "bin", "sherpa-onnx-offline-tts"),
		filepath.Join(dir, "lib", "libsherpa-onnx-c-api.so"),
		filepath.Join(dir, "lib", "libonnxruntime.so"),
		filepath.Join(dir, "models", model, "tts.json"),
		filepath.Join(dir, "models", model, "voice.bin"),
		filepath.Join(dir, "models", model, "unicode_indexer.bin"),
	} {
		if !fileExists(p) {
			errs = append(errs, fmt.Errorf("missing %s", p))
		}
	}
	// The onnx files may be int8 or float; every stage needs one of the two.
	for _, base := range []string{"text_encoder", "vector_estimator", "vocoder", "duration_predictor"} {
		if !fileExists(filepath.Join(dir, "models", model, base+".int8.onnx")) &&
			!fileExists(filepath.Join(dir, "models", model, base+".onnx")) {
			errs = append(errs, fmt.Errorf("missing %s onnx", base))
		}
	}
	return errors.Join(errs...)
}

// fetch downloads an archive into dir (unless a verified copy is already there)
// and returns its path.
func fetch(a archive, dir string, logf Logf) (string, error) {
	dst := filepath.Join(dir, a.Name)
	if sum, err := fileSHA256(dst); err == nil && sum == a.SHA256 {
		if logf != nil {
			logf("%s cached", a.Name)
		}
		return dst, nil
	}

	tmp := dst + ".part"
	f, err := os.Create(tmp)
	if err != nil {
		return "", err
	}
	// Remove the partial file on every error path below.
	ok := false
	defer func() {
		f.Close()
		if !ok {
			_ = os.Remove(tmp)
		}
	}()

	client := &http.Client{Timeout: 15 * time.Minute}
	resp, err := client.Get(a.URL)
	if err != nil {
		return "", fmt.Errorf("download %s: %w", a.URL, err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return "", fmt.Errorf("download %s: %s", a.URL, resp.Status)
	}

	h := sha256.New()
	n, err := io.Copy(io.MultiWriter(f, h), resp.Body)
	if err != nil {
		return "", fmt.Errorf("download %s: %w", a.URL, err)
	}
	if err := f.Close(); err != nil {
		return "", err
	}
	got := hex.EncodeToString(h.Sum(nil))
	if got != a.SHA256 {
		return "", fmt.Errorf("%s: checksum mismatch (got %s, want %s)", a.Name, got, a.SHA256)
	}
	if err := os.Rename(tmp, dst); err != nil {
		return "", err
	}
	ok = true
	if logf != nil {
		logf("%s: %d MB, sha256 ok", a.Name, n>>20)
	}
	return dst, nil
}

// withBase rewrites the download host (mirror builds, tests).
func withBase(a archive, base string) archive {
	if base == "" {
		return a
	}
	a.URL = strings.TrimSuffix(base, "/") + "/" + a.Name
	return a
}

func fileExists(path string) bool {
	st, err := os.Stat(path)
	return err == nil && st.Mode().IsRegular()
}

func fileSHA256(path string) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer f.Close()
	h := sha256.New()
	if _, err := io.Copy(h, f); err != nil {
		return "", err
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
