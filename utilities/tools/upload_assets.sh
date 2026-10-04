#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/../.." && pwd)"
repository="${SAC_GITHUB_REPOSITORY:-ITU-EMAV/sac_autonomy}"
release="${SAC_ASSET_RELEASE:-runtime-assets-v1}"
cache_dir="${SAC_ASSET_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/sac_autonomy/assets/$release}"
archive="$cache_dir/sac_runtime_assets.tar"
command -v gh >/dev/null || { echo 'GitHub CLI gerekli: sudo apt install gh' >&2; exit 1; }
python3 "$script_dir/runtime_assets.py" pack "$archive" --repo "$repo_dir"
if gh release view "$release" --repo "$repository" >/dev/null 2>&1; then
  echo "Release already exists: $release. Choose a new SAC_ASSET_RELEASE for updated assets." >&2
  exit 1
fi
notes_file="$cache_dir/release-notes.txt"
cat > "$notes_file" <<'NOTES'
Models, point-cloud maps, and geographic lookup data for sac_autonomy.

Download and install using utilities/tools/download_assets.sh. The archive keeps package paths and contains SHA-256 checksums for every file. Git stores code and map metadata; large binary assets are distributed here.
NOTES
gh release create "$release" "$archive" "$archive.sha256" \
  --repo "$repository" --title "Runtime assets $release" --notes-file "$notes_file"
echo "Release created. Pin the archive checksum in utilities/tools/runtime_assets.sha256 before pushing the corresponding code."
