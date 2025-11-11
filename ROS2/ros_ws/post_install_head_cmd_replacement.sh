#!/bin/bash
set -e

INSTALL_DIR="pi-zero/install"

# Patch the specific line in _local_setup_util_sh.py
UTIL_FILE="$INSTALL_DIR/_local_setup_util_sh.py"
if [ -f "$UTIL_FILE" ]; then
    sed -i 's|head -c 1|printf "%.1s" "$name"|' "$UTIL_FILE"
fi
