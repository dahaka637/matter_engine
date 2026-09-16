#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile_executable="$root/build-profile/MatterEngine"
debug_executable="$root/build-linux/MatterEngine"
if [[ -x "$profile_executable" ]]; then
    executable="$profile_executable"
else
    executable="$debug_executable"
fi

if [[ ! -x "$executable" ]]; then
    echo "Executavel nao encontrado. Rode tools/build.sh primeiro." >&2
    exit 1
fi

cd "$root"
exec "$executable"
