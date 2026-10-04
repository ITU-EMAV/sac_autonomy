#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
mkdir -p "${PACKAGE_DIR}/models"
wget -c \
  -O "${PACKAGE_DIR}/models/yolopv2.pt" \
  https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt
test -s "${PACKAGE_DIR}/models/yolopv2.pt"
echo "YOLOPv2 weights ready: ${PACKAGE_DIR}/models/yolopv2.pt"
