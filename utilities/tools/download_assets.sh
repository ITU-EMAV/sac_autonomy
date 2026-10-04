#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/../.." && pwd)"
repository="${SAC_GITHUB_REPOSITORY:-ITU-EMAV/sac_autonomy}"
release="${SAC_ASSET_RELEASE:-runtime-assets-v1}"
cache_dir="${SAC_ASSET_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/sac_autonomy/assets/$release}"
archive="$cache_dir/sac_runtime_assets.tar"
checksum_file="${SAC_ASSET_CHECKSUM_FILE:-$script_dir/runtime_assets.sha256}"

command -v curl >/dev/null || { echo 'curl gerekli: sudo apt install curl' >&2; exit 1; }
mkdir -p "$cache_dir"
if [[ ! -f "$checksum_file" ]]; then
  echo "Release checksum missing: $checksum_file" >&2
  exit 1
fi
if ! (cd "$cache_dir" && sha256sum --check --status "$checksum_file") 2>/dev/null; then
  url="https://github.com/$repository/releases/download/$release/sac_runtime_assets.tar"
  if ! curl --fail --location --retry 3 --retry-delay 5 \
      --connect-timeout 15 --continue-at - --output "$archive.part" "$url"; then
    echo "Download failed; run this command again to resume." >&2
    exit 1
  fi
  (cd "$cache_dir" && expected="$(awk '{print $1}' "$checksum_file")" && \
    printf '%s  %s\n' "$expected" 'sac_runtime_assets.tar.part' | sha256sum --check)
  mv -- "$archive.part" "$archive"
fi
python3 "$script_dir/runtime_assets.py" extract "$archive" --repo "$repo_dir"
