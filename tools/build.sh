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

# Backend de fisica: durante a migracao PhysX -> Jolt os dois convivem, e a
# mesma suite roda contra ambos. Sem a variavel, vale o default do CMake
# (cmake/PhysicsBackend.cmake). Uma vez configurado, o valor fica no cache do
# diretorio de build - exportar a variavel troca de backend no lugar.
physics_backend_arg=()
if [[ -n "${MATTERENGINE_PHYSICS_BACKEND:-}" ]]; then
    physics_backend_arg=(
        "-DMATTERENGINE_PHYSICS_BACKEND=${MATTERENGINE_PHYSICS_BACKEND}")
fi

cmake -S "$root" -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE="$configuration" "${physics_backend_arg[@]+"${physics_backend_arg[@]}"}"
cmake --build "$build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-6}" \
    --target MatterEngineApp MatterEngineTests MatterAudioTests

if [[ "$skip_tests" != "--skip-tests" ]]; then
    ctest --test-dir "$build" --output-on-failure
fi

echo "Build pronta: $build/MatterEngine"
