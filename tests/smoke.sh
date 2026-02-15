#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 ]]; then
  echo "usage: $0 <gbaemu-bin> <bios> <rom>" >&2
  exit 2
fi

EMU_BIN="$1"
BIOS="$2"
ROM="$3"

OUTPUT="$(${EMU_BIN} --bios "${BIOS}" --rom "${ROM}" --headless-frames 120 --mute)"
echo "${OUTPUT}"

echo "${OUTPUT}" | rg -q '^HEADLESS_OK '
