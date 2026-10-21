#!/usr/bin/env bash
# Run cargo in the build container (cargo is not installed on the hosts).
#   tools/cargo.sh test --workspace
# The crate registry is cached in a named volume so builds are not
# re-downloading it; target/ is the repository's own (gitignored).
set -euo pipefail
repo="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
image="${LONGMYND_RS_IMAGE:-localhost/longmynd-rs-build:1}"
if ! podman image exists "$image"; then
    podman build -t "$image" -f "$repo/tools/Containerfile" "$repo/tools"
fi
exec podman run --rm --init \
    -v "$repo:/src:Z" -w /src \
    -v longmynd-rs-cargo:/usr/local/cargo/registry \
    -e CARGO_TERM_COLOR=never ${LONGMYND_RS_BLESS:+-e LONGMYND_RS_BLESS=1} \
    "$image" cargo "$@"
