#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile_executable="$root/build-profile/MatterEngine"
debug_executable="$root/build-linux/MatterEngine"
# O desenvolvimento alterna entre Debug e RelWithDebInfo. Escolher sempre o
# profile apenas porque ele existe pode iniciar por dias uma build obsoleta e
# fazer uma correcao recem-compilada parecer ausente. Quando ambas existem,
# execute a que foi ligada por ultimo.
if [[ -x "$profile_executable" && (! -x "$debug_executable" \
        || "$profile_executable" -nt "$debug_executable") ]]; then
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
