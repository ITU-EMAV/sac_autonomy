#!/usr/bin/env bash
set -euo pipefail

readonly REPOSITORY="memre12/smart_car_ws"
readonly RELEASE_TAG="perception-models-v1.0.0"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly MODEL_DIR="${JETSON_PERCEPTION_MODEL_DIR:-${PACKAGE_DIR}/models}"
readonly MODEL_FILES=(
  yolopv2.pt
  traffic_sign_22cls.engine
  yolov8n.engine
)

command -v gh >/dev/null 2>&1 || {
  echo "ERROR: GitHub CLI (gh) is required because ${REPOSITORY} is private." >&2
  exit 1
}
gh auth status -h github.com >/dev/null

mkdir -p "${MODEL_DIR}"
rm -f -- "${MODEL_DIR}/SHA256SUMS"
gh release download "${RELEASE_TAG}" \
  --repo "${REPOSITORY}" \
  --dir "${MODEL_DIR}" \
  --pattern 'SHA256SUMS'

for model_file in "${MODEL_FILES[@]}"; do
  if (
    cd "${MODEL_DIR}"
    grep "  ${model_file}$" SHA256SUMS | sha256sum --check --status
  ); then
    echo "Already verified: ${model_file}"
    continue
  fi

  echo "Downloading: ${model_file}"
  rm -f -- "${MODEL_DIR}/${model_file}"
  gh release download "${RELEASE_TAG}" \
    --repo "${REPOSITORY}" \
    --dir "${MODEL_DIR}" \
    --pattern "${model_file}"
done

(
  cd "${MODEL_DIR}"
  sha256sum --check SHA256SUMS
)

echo "Perception models ready in ${MODEL_DIR}"
