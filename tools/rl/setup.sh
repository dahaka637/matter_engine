#!/usr/bin/env bash
# Prepara o treino de RL: o ambiente Python (uv, Python 3.14 fixo e o PyTorch
# cu126, a variante que roda na GTX 1070 Ti), a ponte C++ (build-rl, Release)
# e confere tudo. Pode rodar de novo a qualquer hora (so refaz o que mudou).
set -Eeuo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"

if ! command -v uv >/dev/null 2>&1; then
    echo "o uv e necessario (sudo pacman -S uv)" >&2
    exit 2
fi
cd "$here"
uv python install 3.14 --managed-python
uv sync --managed-python --all-groups

python="$here/.venv/bin/python"
# As dependencias ja baixadas pela build normal sao reaproveitadas (sem clonar
# de novo); sem ela, o CMake baixa.
reuse=()
for source in "$root"/build-profile/_deps/*-src "$root"/build-linux/_deps/*-src; do
    [[ -d "$source" ]] || continue
    name="$(basename "$source")"
    name="${name%-src}"
    reuse+=("-DFETCHCONTENT_SOURCE_DIR_${name^^}=$source")
done
cmake -S "$root" -B "$root/build-rl" -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DMATTERENGINE_BUILD_RL_PYTHON=ON -DMATTERENGINE_BUILD_TESTS=OFF \
    -DPython_EXECUTABLE="$python" "${reuse[@]}"
cmake --build "$root/build-rl" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-6}" --target matter_rl_native

"$here/matter-rl" doctor
