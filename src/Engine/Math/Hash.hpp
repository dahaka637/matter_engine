#pragma once

#include <cstdint>

namespace MatterEngine {

// Hash inteiro de alta qualidade (familia pcg - Jarzynski & Olano, "Hash
// Functions for GPU Rendering", 2020). Extraido para ca porque duas partes
// independentes da engine (WindSystem, do lado CPU, e ProceduralNoise, tambem
// CPU) precisavam da mesma familia de hash de boa qualidade - a primeira
// mantendo o mesmo tipo de ruido usado no shader do ceu
// (assets/shaders/scene3d_sky.frag, onde substituiu um hash mais fraco que
// causava correlacao em bloco visivel nas nuvens); duplicar a mesma funcao
// nos dois arquivos seria o tipo de divergencia silenciosa que este projeto
// evita.
[[nodiscard]] std::uint32_t pcgHash(std::uint32_t value);

// pcgHash() reescalado para o intervalo [0,1).
[[nodiscard]] float hashToUnitFloat(std::uint32_t seed);

} // namespace MatterEngine
