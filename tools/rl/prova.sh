#!/usr/bin/env bash
# A prova bit a bit entre duas versoes do codigo (o "chave desligada = o de
# antes" de cada etapa): o build-rl usa -march=native e LTO, e o GCC funde
# multiplicacao e soma (FMA) conforme o inline - recompilar com outro codigo em
# volta muda o ultimo bit da fisica mesmo com a logica igual. A build de prova
# desliga a fusao (-ffp-contract=off) e o LTO: o resultado so muda se a conta
# mudar. Uso (os argumentos vao para o hash_locomocao.py):
#   tools/rl/prova.sh --gravar tools/rl/ouro/prova_<tarefa>_<commit>.json --tarefa <tarefa>   # no codigo antigo
#   tools/rl/prova.sh --conferir tools/rl/ouro/prova_<tarefa>_<commit>.json --tarefa <tarefa> # no novo
set -Eeuo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/../.." && pwd)"
python="$here/.venv/bin/python"
reuse=()
for source in "$root"/build-profile/_deps/*-src "$root"/build-linux/_deps/*-src; do
    [[ -d "$source" ]] || continue
    name="$(basename "$source")"
    name="${name%-src}"
    reuse+=("-DFETCHCONTENT_SOURCE_DIR_${name^^}=$source")
done
if [[ ! -f "$root/build-prova/build.ninja" ]]; then
    cmake -S "$root" -B "$root/build-prova" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DMATTERENGINE_BUILD_RL_PYTHON=ON -DMATTERENGINE_BUILD_TESTS=OFF \
        -DCMAKE_C_FLAGS="-ffp-contract=off" -DCMAKE_CXX_FLAGS="-ffp-contract=off" \
        -DPython_EXECUTABLE="$python" "${reuse[@]}" >/dev/null
fi
# (Uma build-prova que nao tenha as opcoes certas nao prova nada.)
cache="$root/build-prova/CMakeCache.txt"
if ! grep -q '^CMAKE_CXX_FLAGS:STRING=-ffp-contract=off$' "$cache" || grep -qE '^(CMAKE_)?INTERPROCEDURAL_OPTIMIZATION:BOOL=ON' "$cache"; then
    echo "build-prova sem -ffp-contract=off ou com LTO: apague a pasta e rode de novo" >&2
    exit 2
fi
cmake --build "$root/build-prova" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-10}" --target matter_locomotion_native >/dev/null
cd "$here"
MATTER_RL_NATIVE="$root/build-prova/python" exec "$python" hash_locomocao.py "$@"
