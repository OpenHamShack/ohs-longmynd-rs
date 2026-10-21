#!/usr/bin/env bash
# Regenerate src/charset_tables.rs from glibc's iconv, in the build
# container (tools/Containerfile). The C longmynd converts DVB service names
# with iconv; the Rust reproduces glibc's tables byte for byte instead of
# linking iconv, which Windows does not have.
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
image="${LONGMYND_RS_IMAGE:-localhost/longmynd-rs-build:1}"
podman run --rm -v "$repo/tools/charsets:/src:ro,Z" "$image" \
    sh -c 'gcc -O1 -o /tmp/enumerate /src/enumerate.c && /tmp/enumerate' \
    > "$repo/src/charset_tables.rs.new"
mv "$repo/src/charset_tables.rs.new" "$repo/src/charset_tables.rs"
