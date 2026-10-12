#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
configuration="${1:-RelWithDebInfo}"
skip_tests="${2:-}"

case "$configuration" in
    Debug|RelWithDebInfo|Release) ;;
    *)
        echo "Uso: $0 [Debug|RelWithDebInfo|Release] [--skip-tests]" >&2
        exit 2
        ;;
esac

case "$configuration" in
    Debug) build="$root/build-linux" ;;
    RelWithDebInfo) build="$root/build-profile" ;;
    Release) build="$root/build-release" ;;
esac

cmake -S "$root" -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE="$configuration"
# O CTest registra todos os executaveis de teste. Construir somente um
# subconjunto faz uma arvore limpa reportar "Not Run" para binarios ausentes,
# portanto a verificacao oficial precisa compilar todos os targets nao
# excluidos antes de iniciar a suite.
cmake --build "$build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-6}" \
    --target all

if [[ "$skip_tests" != "--skip-tests" ]]; then
    ctest --test-dir "$build" --output-on-failure
fi

echo "Build pronta: $build/MatterEngine"
