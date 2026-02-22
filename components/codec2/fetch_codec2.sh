#!/usr/bin/env bash
# fetch_codec2.sh  –  Download the Codec2 source into components/codec2/codec2-src/
# Run from the project root directory:
#   bash components/codec2/fetch_codec2.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${SCRIPT_DIR}/codec2-src"
REPO_URL="https://github.com/drowe67/codec2.git"
# Use latest master (no stable release tags exist in this repo)
CODEC2_BRANCH="master"

echo "========================================="
echo "  Fetching Codec2 (${CODEC2_BRANCH})"
echo "  → ${TARGET_DIR}"
echo "========================================="

if [ -d "${TARGET_DIR}/.git" ]; then
    echo "Repository already cloned, pulling latest ..."
    git -C "${TARGET_DIR}" pull --ff-only
else
    echo "Cloning codec2 (shallow clone of ${CODEC2_BRANCH}) ..."
    git clone --depth 1 --branch "${CODEC2_BRANCH}" "${REPO_URL}" "${TARGET_DIR}"
fi

echo ""
echo "Applying ESP32 Codec2 memory/stability patch (3200-only + shared scratch) ..."
patch -d "${TARGET_DIR}" -p1 < "${SCRIPT_DIR}/esp32_codec2_3200_mem_stability.patch"

echo "Codec2 source is ready at: ${TARGET_DIR}"
echo ""
echo "Now rebuild the project:"
echo "  idf.py fullclean && idf.py build"
echo ""
