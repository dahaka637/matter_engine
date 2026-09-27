#!/usr/bin/env bash
# Script de compilação fácil do MatterEngine.
#
# Uso:
#   ./compilar.sh                 -> compila (RelWithDebInfo, incremental)
#   ./compilar.sh Debug           -> compila em Debug
#   ./compilar.sh Release         -> compila em Release
#   ./compilar.sh --run           -> compila e já roda o jogo
#   ./compilar.sh --testes        -> compila e roda a suíte de testes
#   ./compilar.sh --limpar        -> apaga a build anterior antes de compilar
#   ./compilar.sh Debug --run     -> as opções podem ser combinadas
#   ./compilar.sh --help          -> mostra esta ajuda
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root"

vermelho=$'\033[31m'; verde=$'\033[32m'; azul=$'\033[34m'; reset=$'\033[0m'
info()  { printf '%s==>%s %s\n' "$azul" "$reset" "$1"; }
ok()    { printf '%s✔%s %s\n' "$verde" "$reset" "$1"; }
erro()  { printf '%s✘ %s%s\n' "$vermelho" "$1" "$reset" >&2; }

mostrar_ajuda() {
    sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
}

configuration="RelWithDebInfo"
rodar_depois=0
rodar_testes=0
limpar=0

for arg in "$@"; do
    case "$arg" in
        Debug|RelWithDebInfo|Release) configuration="$arg" ;;
        --run|-r) rodar_depois=1 ;;
        --testes|-t) rodar_testes=1 ;;
        --limpar|-c) limpar=1 ;;
        -h|--help) mostrar_ajuda; exit 0 ;;
        *)
            erro "Argumento desconhecido: $arg"
            mostrar_ajuda
            exit 2
            ;;
    esac
done

for dep in cmake ninja; do
    if ! command -v "$dep" >/dev/null 2>&1; then
        erro "'$dep' não encontrado. Instale-o antes de compilar (ex.: sudo pacman -S $dep)."
        exit 1
    fi
done

case "$configuration" in
    Debug) build="$root/build-linux" ;;
    RelWithDebInfo) build="$root/build-profile" ;;
    Release) build="$root/build-release" ;;
esac

trap 'erro "A compilação falhou. Veja a mensagem acima para o erro exato."' ERR

if [[ "$limpar" == 1 && -d "$build" ]]; then
    info "Apagando build anterior em $build"
    rm -rf "$build"
fi

inicio=$(date +%s)

info "Configurando projeto ($configuration)"
cmake -S "$root" -B "$build" -G Ninja -DCMAKE_BUILD_TYPE="$configuration" >/dev/null

info "Compilando (isso pode demorar na primeira vez, baixando dependências)"
# Mesmos alvos de tools/build.sh. Os executaveis de teste antigos
# (AnimatedRagdoll, ContactFootwork, RagdollPoseMotor, ActiveRagdollV2) sairam
# do CMake e faziam este script falhar antes de compilar qualquer coisa.
cmake --build "$build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}" \
    --target MatterEngineApp MatterEngineTests MatterAudioTests

fim=$(date +%s)
ok "Build pronta em $((fim - inicio))s: $build/MatterEngine"

if [[ "$rodar_testes" == 1 ]]; then
    info "Rodando testes"
    ctest --test-dir "$build" --output-on-failure
    ok "Testes concluídos"
fi

if [[ "$rodar_depois" == 1 ]]; then
    info "Executando $build/MatterEngine"
    exec "$build/MatterEngine"
fi
