#!/usr/bin/env bash
# Clone sac_autonomy and install its model/map release in one command.
set -euo pipefail
repository="${SAC_GITHUB_REPOSITORY:-ITU-EMAV/sac_autonomy}"
repository_url="${SAC_REPOSITORY_URL:-https://github.com/$repository.git}"
branch="${SAC_GITHUB_REF:-main}"
destination="${1:-sac_autonomy}"
for required in git curl python3 sha256sum; do
  command -v "$required" >/dev/null || { echo "Missing command: $required" >&2; exit 1; }
done
if [[ -e "$destination" ]]; then
  if [[ ! -d "$destination/.git" ]]; then
    echo "Destination exists and is not a Git checkout: $destination" >&2
    exit 1
  fi
  origin="$(git -C "$destination" remote get-url origin)"
  case "$origin" in
    "$repository_url"|"https://github.com/$repository.git"|"git@github.com:$repository.git") ;;
    *) echo "Destination belongs to a different repository: $destination" >&2; exit 1 ;;
  esac
else
  git clone --branch "$branch" "$repository_url" "$destination"
fi
bash "$destination/utilities/tools/download_assets.sh"
echo "sac_autonomy ready: $destination"
