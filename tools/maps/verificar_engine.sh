#!/usr/bin/env bash
set -euo pipefail
map_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
map_build_dir="${MATTERENGINE_MAP_BUILD_DIR:-$map_repo_dir/build-linux}"
map_verifier="$map_build_dir/MatterMapVerify"
if [[ ! -x "$map_verifier" ]]; then
    echo "Verificador ausente: $map_verifier. Execute ./tools/build.sh Debug (ou configure MATTERENGINE_MAP_BUILD_DIR)." >&2
    exit 1
fi
exec "$map_verifier" "$@"
