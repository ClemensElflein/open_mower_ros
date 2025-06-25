#!/bin/bash
set -euo pipefail

# Usage: utils/scripts/transfer_pr.sh https://github.com/ClemensElflein/<repo>/pull/<id>

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <GitHub PR URL>"
  exit 1
fi

PR_URL="$1"

# Validate and parse the GitHub PR URL
if [[ "$PR_URL" =~ ^https://github.com/([^/]+)/([^/]+)/pull/([0-9]+)$ ]]; then
  USER="${BASH_REMATCH[1]}"
  REPO="${BASH_REMATCH[2]}"
  PR_ID="${BASH_REMATCH[3]}"
  PREFIX="src/lib/$REPO"
  PATCH_URL="https://github.com/$USER/$REPO/pull/$PR_ID.patch"
else
  echo "❌ Invalid PR URL: $PR_URL"
  exit 1
fi

echo "🔗 PR URL:     $PR_URL"
echo "📦 Repo:       $REPO"
echo "📁 Prefix:     $PREFIX"
echo "🌐 Patch URL:  $PATCH_URL"

# Create temp dir and fetch the patch
TMPDIR=$(mktemp -d)
PATCH_ORIG="$TMPDIR/original.patch"
PATCH_MOD="$TMPDIR/modified.patch"

echo "🔽 Downloading patch..."
curl -sSL "$PATCH_URL" -o "$PATCH_ORIG"

echo "🛠️  Rewriting patch paths to include prefix: $PREFIX"

# Rewrite paths (--- a/… and +++ b/…) to include the prefix
sed -E \
  -e "s|^(--- a/)(.*)|\1$PREFIX/\2|" \
  -e "s|^(\\+\\+\\+ b/)(.*)|\1$PREFIX/\2|" \
  "$PATCH_ORIG" > "$PATCH_MOD"

echo "📥 Applying patch..."
git am --3way "$PATCH_MOD" || {
  echo "❌ Patch failed to apply cleanly."
  exit 1
}

echo "✅ Patch applied successfully into '$PREFIX/'"
