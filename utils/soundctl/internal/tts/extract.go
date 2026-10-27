package tts

import (
	"archive/tar"
	"compress/bzip2"
	"compress/gzip"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strings"
)

// extractTarBz2 unpacks a .tar.bz2 archive into dst. keep (optional) filters on
// the path *inside* the archive's top level directory.
func extractTarBz2(archivePath, dst string, keep func(rel string) bool) error {
	f, err := os.Open(archivePath)
	if err != nil {
		return err
	}
	defer f.Close()
	return extractTar(bzip2.NewReader(f), dst, keep)
}

// extractTarGz unpacks a .tar.gz archive (the piper bundles) into dst.
func extractTarGz(archivePath, dst string) error {
	f, err := os.Open(archivePath)
	if err != nil {
		return err
	}
	defer f.Close()
	gz, err := gzip.NewReader(f)
	if err != nil {
		return err
	}
	defer gz.Close()
	return extractTar(gz, dst, nil)
}

// extractTar unpacks a tar stream into dst. The sherpa archives wrap everything
// in a single top level directory; that directory is stripped so entries land in
// dst directly. Only regular files, directories and symlinks are extracted, and
// no entry may escape dst (zip-slip).
func extractTar(r io.Reader, dst string, keep func(rel string) bool) error {
	tr := tar.NewReader(r)
	prefix := ""
	first := true

	for {
		hdr, err := tr.Next()
		if errors.Is(err, io.EOF) {
			return nil
		}
		if err != nil {
			return err
		}

		name := strings.TrimPrefix(filepath.ToSlash(hdr.Name), "./")
		if first {
			first = false
			// A wrapper directory is the only thing at the top level in these
			// archives; without one, entries are used as they are.
			if hdr.Typeflag == tar.TypeDir {
				prefix = strings.TrimSuffix(name, "/") + "/"
				continue
			}
		}
		rel := strings.TrimPrefix(name, prefix)
		if rel == "" || rel == "." || rel == "/" {
			continue
		}
		if keep != nil && !keep(rel) {
			continue
		}

		target, err := safeJoin(dst, rel)
		if err != nil {
			return err
		}

		switch hdr.Typeflag {
		case tar.TypeDir:
			if err := os.MkdirAll(target, 0o755); err != nil {
				return err
			}
		case tar.TypeReg:
			if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
				return err
			}
			if err := writeFile(target, tr, os.FileMode(hdr.Mode).Perm()); err != nil {
				return fmt.Errorf("%s: %w", rel, err)
			}
		case tar.TypeSymlink:
			// The link target is resolved relative to the link's directory, so
			// it has to be checked against that directory, too.
			if _, err := safeJoin(filepath.Dir(target), hdr.Linkname); err != nil {
				return err
			}
			if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
				return err
			}
			_ = os.Remove(target)
			if err := os.Symlink(hdr.Linkname, target); err != nil {
				return err
			}
		case tar.TypeLink:
			linkRel := strings.TrimPrefix(strings.TrimPrefix(filepath.ToSlash(hdr.Linkname), "./"), prefix)
			linkPath, err := safeJoin(dst, linkRel)
			if err != nil {
				return err
			}
			if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
				return err
			}
			_ = os.Remove(target)
			if err := os.Link(linkPath, target); err != nil {
				return err
			}
		default:
			// Devices, fifos and the like are not part of these archives.
		}
	}
}

func writeFile(path string, r io.Reader, mode os.FileMode) error {
	if mode == 0 {
		mode = 0o644
	}
	f, err := os.OpenFile(path, os.O_CREATE|os.O_TRUNC|os.O_WRONLY, mode)
	if err != nil {
		return err
	}
	if _, err := io.Copy(f, r); err != nil {
		f.Close()
		return err
	}
	return f.Close()
}

// safeJoin joins rel onto dst and rejects anything that would leave dst.
func safeJoin(dst, rel string) (string, error) {
	rel = filepath.FromSlash(rel)
	if rel == "" || filepath.IsAbs(rel) || rel == ".." || strings.HasPrefix(rel, ".."+string(filepath.Separator)) {
		return "", fmt.Errorf("unsafe path %q in archive", rel)
	}
	target := filepath.Join(dst, rel)
	if target != dst && !strings.HasPrefix(target, strings.TrimSuffix(dst, string(filepath.Separator))+string(filepath.Separator)) {
		return "", fmt.Errorf("path %q escapes %s", rel, dst)
	}
	return target, nil
}
