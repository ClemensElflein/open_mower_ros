#!/bin/sh
# The naming rules of the TTS render store (ghcr.io/<repo>/tts-sounds), in one place.
#
# render-sounds.yaml writes the tags and prune-sound-store.yaml has to recognise them
# again: two copies of any rule here would drift, and the prune would then delete the
# stand of a live branch. What the tags mean is documented in sounds/README.md
# ("Where the renders live").
#
#   sound-store.sh languages      the shipped languages, space separated: one per
#                                 src/open_mower/params/sounds_<lang>.yaml
#   sound-store.sh owner          the owner-tag suffix of this run (GITHUB_* env): empty on
#                                 main, which owns the base tags <lang>; "-<suffix>" for every
#                                 other ref (a pull request, a branch, a release tag)
#   sound-store.sh suffix <ref>   the tag suffix of a branch or tag name
set -eu

root=$(cd "$(dirname "$0")/../.." && pwd)

# "/" and everything outside [A-Za-z0-9._-] become "-", because that is what an OCI tag
# allows; printf, not echo, or the trailing newline would become a "-" as well. A tag is at
# most 128 characters and "<lang>-" takes some of it, so a name longer than 80 is cut and
# gets a short hash of the full ref: two long names with a common prefix must not share a tag.
suffix() {
  clean=$(printf '%s' "$1" | tr '/' '-' | tr -c 'a-zA-Z0-9._-' '-')
  if [ "${#clean}" -gt 80 ]; then
    clean="$(printf '%s' "${clean}" | cut -c1-71 | sed 's/-*$//')-$(printf '%s' "$1" | sha256sum | cut -c1-8)"
  fi
  printf '%s\n' "${clean}"
}

case "${1:-}" in
  languages)
    langs=""
    for f in "${root}"/src/open_mower/params/sounds_*.yaml; do
      [ -e "${f}" ] || { echo "sound-store.sh: no sounds_*.yaml in src/open_mower/params" >&2; exit 1; }
      lang=${f##*/sounds_}
      langs="${langs:+${langs} }${lang%.yaml}"
    done
    printf '%s\n' "${langs}"
    ;;
  owner)
    if [ "${GITHUB_REF:-}" = "refs/heads/main" ]; then
      printf '\n'
    else
      # A pull_request's GITHUB_REF_NAME is "<number>/merge", which names the run and not
      # the branch, so prefer GITHUB_HEAD_REF (empty for a push or a dispatch).
      ref="${GITHUB_HEAD_REF:-${GITHUB_REF_NAME:-}}"
      [ -n "${ref}" ] || { echo "sound-store.sh: neither GITHUB_HEAD_REF nor GITHUB_REF_NAME is set" >&2; exit 2; }
      printf -- '-%s\n' "$(suffix "${ref}")"
    fi
    ;;
  suffix)
    [ -n "${2:-}" ] || { echo "usage: sound-store.sh suffix <ref>" >&2; exit 2; }
    suffix "$2"
    ;;
  *)
    echo "usage: sound-store.sh languages | owner | suffix <ref>" >&2
    exit 2
    ;;
esac
