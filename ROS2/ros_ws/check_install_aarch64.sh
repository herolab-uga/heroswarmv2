#!/usr/bin/env bash
# check_install_aarch64.sh
# Scan an install/ tree for binaries/libs that are NOT AArch64, and report suspicious dynamic deps.
# Usage: ./check_install_aarch64.sh [install_dir]
# Default install_dir = ./install

set -euo pipefail

INSTALL_DIR="${1:-./install}"
if [ ! -d "$INSTALL_DIR" ]; then
  echo "ERROR: install dir not found: $INSTALL_DIR" >&2
  exit 2
fi

# Tools required
for tool in file readelf awk sed; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "ERROR: required tool not found: $tool" >&2
    exit 2
  fi
done

# Patterns that indicate incorrect host arch or suspicious interpreter paths
BAD_ARCH_PATTERNS="x86_64|Intel|i386|i686|MS Windows"
GOOD_ARCH_PATTERN="ARM aarch64|AArch64|arm64"

echo "Scanning: $INSTALL_DIR"
echo

bad_count=0
ok_count=0
inconclusive_count=0

# find candidate files: ELF executables and shared objects
mapfile -t files < <(find "$INSTALL_DIR" -type f -print)

for f in "${files[@]}"; do
  # skip small non-ELF quickly
  if ! file "$f" | grep -q 'ELF'; then
    continue
  fi

  # Basic file(1) check
  fileout="$(file "$f")"

  # Check machine from readelf -h
  re_h="$(readelf -h "$f" 2>/dev/null || true)"
  machine_line="$(echo "$re_h" | awk -F: '/Machine:/ { print $2 }' | sed 's/^[ 	]*//')"

  # read program interpreter (dynamic linker) if present
  interp="$(readelf -l "$f" 2>/dev/null | awk '/Requesting program interpreter/ {print $NF; exit}')"

  # list DT_NEEDED libs
  needed="$(readelf -d "$f" 2>/dev/null | awk -F'[][]' '/NEEDED/ {print $2}' | tr '\n' ';')"

  # read ELF attributes (optional) and try to spot ASIMD/NEON mentions (may or may not be present)
  attrs="$(readelf -A "$f" 2>/dev/null || true)"
  has_asimd=0
  if echo "$attrs" | grep -qi -e 'ASIMD' -e 'neon' -e 'asimd'; then
    has_asimd=1
  fi

  # Determine verdict
  if echo "$fileout" | grep -Eiq "$BAD_ARCH_PATTERNS"; then
    echo "BAD ARCH: $f"
    echo "  file: $fileout"
    echo "  readelf Machine:${machine_line:-'(unknown)'}"
    echo "  interpreter: ${interp:-'(none)'}"
    echo "  NEEDED: ${needed:-'(none)'}"
    echo
    bad_count=$((bad_count+1))
    continue
  fi

  if echo "$fileout" | grep -Eiq "$GOOD_ARCH_PATTERN" || echo "$machine_line" | grep -qi 'AArch64'; then
    echo "OK      : $f"
    echo "  file: $fileout"
    echo "  readelf Machine:${machine_line:-'(unknown)'}"
    echo "  interpreter: ${interp:-'(none)'}"
    echo "  NEEDED: ${needed:-'(none)'}"
    if [ "$has_asimd" -eq 1 ]; then
      echo "  note: ELF attributes mention ASIMD/NEON-like features."
    else
      echo "  note: no explicit ASIMD/NEON attribute found — on AArch64 NEON (ASIMD) is baseline."
    fi
    echo
    ok_count=$((ok_count+1))
    continue
  fi

  # fallback: inconclusive
  echo "INCONCLUSIVE: $f"
  echo "  file: $fileout"
  echo "  readelf Machine:${machine_line:-'(unknown)'}"
  echo "  interpreter: ${interp:-'(none)'}"
  echo "  NEEDED: ${needed:-'(none)'}"
  echo
  inconclusive_count=$((inconclusive_count+1))
done

echo "Summary:"
echo "  OK binaries  : $ok_count"
echo "  BAD binaries : $bad_count"
echo "  Inconclusive : $inconclusive_count"
echo
if [ "$bad_count" -gt 0 ]; then
  echo "WARNING: One or more non-AArch64 (host) binaries found. Inspect above."
  exit 3
fi

echo "Done. Exit 0 = all scanned ELF files look AArch64 or inconclusive only."
exit 0
