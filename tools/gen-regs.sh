#!/usr/bin/env bash
# Regenerate src/regs/*.rs from the reference tree (reference/), checked by
# hash and patched exactly as the equivalence harness builds it. Run after
# moving the reference pin; commit the result together with the pin.
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
(cd "$repo/reference" && sha256sum -c --quiet SHA256SUMS)
tar -xzf "$repo"/reference/longmynd-*.tar.gz -C "$work"
src="$(echo "$work"/longmynd-*)"
patch -s -d "$src" -p1 --forward --fuzz=0 < "$repo/reference/longmynd-picotuner.patch"
python3 "$repo/tools/gen-regs.py" "$src" "$repo/src/regs"
