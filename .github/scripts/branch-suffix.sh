#!/bin/sh
# The tag suffix of a branch: the sanitised ref name, without a leading "-".
#
# Shared by render-sounds.yaml (which writes the tag `<lang>-<suffix>`) and
# prune-sound-store.yaml (which has to recognise that tag again from the branch list). Two
# copies of this would silently drift, and the prune would then delete the stand of a live
# branch - so there is exactly one.
#
# Rules, in one place:
#   - "/" becomes "-" and everything outside [A-Za-z0-9._-] becomes "-", because that is what
#     an OCI tag allows.
#   - printf, not echo: a trailing newline would be sanitised into a "-" as well.
#   - An OCI tag is limited to 128 characters and "<language>-" takes some of it, so a name
#     longer than 80 is truncated and gets a short hash of the full ref appended: two long
#     branch names that share a prefix must not end up under the same tag.
set -eu

ref="${1:-}"
if [ -z "${ref}" ]; then
  echo "usage: branch-suffix.sh <ref>" >&2
  exit 2
fi

clean=$(printf '%s' "${ref}" | tr '/' '-' | tr -c 'a-zA-Z0-9._-' '-')
if [ "${#clean}" -gt 80 ]; then
  clean="$(printf '%s' "${clean}" | cut -c1-71 | sed 's/-*$//')-$(printf '%s' "${ref}" | sha256sum | cut -c1-8)"
fi
printf '%s\n' "${clean}"
