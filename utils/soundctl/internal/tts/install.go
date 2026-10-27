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
	"sort"
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

// ---- piper -----------------------------------------------------------------

// piperVersion is the release the standalone bundles come from. The 2023 build
// is used on purpose: it needs no Python, is ~4x faster than the pip CLI and
// loads the current voices.
const piperVersion = "1.2.0"

const (
	piperReleaseBase = "https://github.com/rhasspy/piper/releases/download/v" + piperVersion + "/"
	piperVoiceBase   = "https://huggingface.co/rhasspy/piper-voices/resolve/main/"
)

// piperArchives maps GOARCH to the standalone bundle: the piper binary plus the
// espeak-ng data and shared libraries it loads from its own directory.
var piperArchives = map[string]archive{
	"amd64": {
		Name:   "piper_amd64.tar.gz",
		URL:    piperReleaseBase + "piper_amd64.tar.gz",
		SHA256: "467c17935d2a22dcce9dc9e08ba07485e29be813097e7cf08c5627aa09d32e42",
	},
	"arm64": {
		Name:   "piper_arm64.tar.gz",
		URL:    piperReleaseBase + "piper_arm64.tar.gz",
		SHA256: "34b298f6b3e55b55e81f05c6157310f9ec4df3fdd3d73e4c85eb80e218c54d2c",
	},
}

// DefaultPiperVoice is the voice sounds_en.yaml uses: en_US-bryce-medium, trained
// on public domain data.
const DefaultPiperVoice = "en_US-bryce-medium"

// piperVoices are the voices `tts install --engine piper --voice …` can fetch,
// pinned by SHA256. Only ones with a clean provenance are listed: public domain,
// CC0 or CC BY training data (see the MODEL_CARD in the voice repository).
var piperVoices = map[string][]archive{
	"en_US-bryce-medium": {
		{Name: "en_US-bryce-medium.onnx",
			URL:    piperVoiceBase + "en/en_US/bryce/medium/en_US-bryce-medium.onnx",
			SHA256: "dc9caa6c313199ffb5ac698b6e542fa6cba388aeaf2731e25262e33b9810aef1"},
		{Name: "en_US-bryce-medium.onnx.json",
			URL:    piperVoiceBase + "en/en_US/bryce/medium/en_US-bryce-medium.onnx.json",
			SHA256: "7ceb1bc4af6d4e41b6d1edbb86c67e91e01eaa71f66db4cd0ae92ac704d415be"},
	},
	"en_US-joe-medium": {
		{Name: "en_US-joe-medium.onnx",
			URL:    piperVoiceBase + "en/en_US/joe/medium/en_US-joe-medium.onnx",
			SHA256: "58afce0321b8d9c46d7cdf9c16500cc55a793b4220212dba6b70fb788b3baf06"},
		{Name: "en_US-joe-medium.onnx.json",
			URL:    piperVoiceBase + "en/en_US/joe/medium/en_US-joe-medium.onnx.json",
			SHA256: "3d6d5410b3795cb1950595247ef8f06190719e6fdbfa3a2356d8ec368e1aad33"},
	},
	"en_US-mike-medium": {
		{Name: "en_US-mike-medium.onnx",
			URL:    piperVoiceBase + "en/en_US/mike/medium/en_US-mike-medium.onnx",
			SHA256: "3157e69e2c580cab297483f4b301ee975a4590c17d35a9757e33347d0e90fae2"},
		{Name: "en_US-mike-medium.onnx.json",
			URL:    piperVoiceBase + "en/en_US/mike/medium/en_US-mike-medium.onnx.json",
			SHA256: "0a2538c0b1e06a6cab7c46b97c79abd5e6a34a6c05b74b9376874c7099d81dad"},
	},
	"en_US-norman-medium": {
		{Name: "en_US-norman-medium.onnx",
			URL:    piperVoiceBase + "en/en_US/norman/medium/en_US-norman-medium.onnx",
			SHA256: "b9739443232a80a59c7d18810dd856899bf16a7964725f5ab81ea49b1351cb71"},
		{Name: "en_US-norman-medium.onnx.json",
			URL:    piperVoiceBase + "en/en_US/norman/medium/en_US-norman-medium.onnx.json",
			SHA256: "6c2db7f558a4a8deb9fe822583c1c5105f6c4e834dd0f9de8ad17a888ee9fe1d"},
	},
	"de_DE-thorsten-high": {
		{Name: "de_DE-thorsten-high.onnx",
			URL:    piperVoiceBase + "de/de_DE/thorsten/high/de_DE-thorsten-high.onnx",
			SHA256: "9df1c43c61149ef9b39e618e2b861fbe41e1fcea9390b2dac62e8761573ea4f1"},
		{Name: "de_DE-thorsten-high.onnx.json",
			URL:    piperVoiceBase + "de/de_DE/thorsten/high/de_DE-thorsten-high.onnx.json",
			SHA256: "6de734444e4c3f9e33b7ebe2746dbc19b71e85f613e79c65acf623200b99a76a"},
	},
}

// PiperVoiceNames lists the voices InstallPiper can fetch, sorted.
func PiperVoiceNames() []string {
	names := make([]string, 0, len(piperVoices))
	for name := range piperVoices {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// PiperDir is where the piper assets live below the TTS directory (the CLI looks
// for <tts-dir>/piper/piper and <tts-dir>/piper/voices).
func PiperDir(dir string) string { return filepath.Join(dir, "piper") }

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
	// Voice is the piper voice to install ("" = DefaultPiperVoice). Ignored by
	// the Supertonic installer.
	Voice string
	Force bool // re-download and re-extract even if the files are present
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

// InstallPiper downloads, verifies and unpacks the piper bundle and one voice
// below <dir>/piper. Like Install it only touches files below dir.
func InstallPiper(opts InstallOptions) error {
	if opts.Dir == "" {
		return errors.New("no install directory")
	}
	arch, err := normalizeArch(opts.Arch)
	if err != nil {
		return err
	}
	voiceName := opts.Voice
	if voiceName == "" {
		voiceName = DefaultPiperVoice
	}
	files, ok := piperVoices[voiceName]
	if !ok {
		return fmt.Errorf("unknown piper voice %q (available: %v)", voiceName, PiperVoiceNames())
	}

	dst := PiperDir(opts.Dir)
	voicesDir := filepath.Join(dst, "voices")
	binAsset := withBase(piperArchives[arch], opts.BaseURL)
	cacheDir := filepath.Join(opts.Dir, ".cache")
	if err := os.MkdirAll(cacheDir, 0o755); err != nil {
		return err
	}
	if err := os.MkdirAll(voicesDir, 0o755); err != nil {
		return err
	}

	if opts.Force || !fileExists(filepath.Join(dst, "piper")) {
		opts.logf("installing %s (%s)", binAsset.Name, arch)
		path, err := fetch(binAsset, cacheDir, opts.logf)
		if err != nil {
			return err
		}
		if err := extractTarGz(path, dst); err != nil {
			return fmt.Errorf("unpack %s: %w", binAsset.Name, err)
		}
		_ = os.Remove(path)
	} else {
		opts.logf("piper binary present, skipping (%s)", dst)
	}

	for _, f := range files {
		a := withBase(f, opts.BaseURL)
		if _, err := fetch(a, voicesDir, opts.logf); err != nil {
			return err
		}
	}

	man := Manifest{
		Version:   1,
		Arch:      arch,
		Binary:    binAsset.Name,
		SHA256:    binAsset.SHA256,
		Model:     voiceName,
		ModelSHA:  files[0].SHA256,
		Installed: time.Now().UTC().Format(time.RFC3339),
	}
	raw, err := json.MarshalIndent(man, "", "  ")
	if err != nil {
		return err
	}
	if err := os.WriteFile(filepath.Join(dst, "install.json"), append(raw, '\n'), 0o644); err != nil {
		return err
	}
	_ = os.RemoveAll(cacheDir)

	if err := VerifyPiper(opts.Dir, voiceName); err != nil {
		return err
	}
	opts.logf("piper assets ready in %s", dst)
	return nil
}

// VerifyPiper checks that a directory holds a usable piper installation (no
// downloads, no engine run).
func VerifyPiper(dir, voice string) error {
	if voice == "" {
		voice = DefaultPiperVoice
	}
	dst := PiperDir(dir)
	var errs []error
	// The binary loads its libraries and the espeak-ng data from its own
	// directory, so all of it has to be there.
	if st, err := os.Stat(filepath.Join(dst, "espeak-ng-data")); err != nil || !st.IsDir() {
		errs = append(errs, fmt.Errorf("missing %s", filepath.Join(dst, "espeak-ng-data")))
	}
	for _, p := range []string{
		filepath.Join(dst, "piper"),
		filepath.Join(dst, "voices", voice+".onnx"),
		filepath.Join(dst, "voices", voice+".onnx.json"),
	} {
		if !fileExists(p) {
			errs = append(errs, fmt.Errorf("missing %s", p))
		}
	}
	return errors.Join(errs...)
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
