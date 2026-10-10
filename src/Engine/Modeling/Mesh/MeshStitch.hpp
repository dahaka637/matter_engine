#pragma once

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"

#include <cstdint>
#include <span>
#include <vector>

// Costuras: ligar pedacos da malha pelas bordas, vertice com vertice, sem
// sobrepor e sem fresta. A peca seguinte comeca da volta da anterior.
namespace MatterEngine::Modeling {

// Faixa de triangulos entre duas voltas fechadas (a de baixo "from" e a de
// cima "to" de um tubo), mesmo com quantidades diferentes de vertices. As
// duas no mesmo sentido do bridge(): anti-horarias vistas de cima (do lado
// de "to"). O casamento minimiza o comprimento das diagonais (programacao
// dinamica), comecando pelo par de vertices mais proximo.
void stitchLoops(MeshBuilder& builder, std::span<const std::uint32_t> from, std::span<const std::uint32_t> to);

// O mesmo para duas linhas abertas (pontas casadas: from[0] com to[0] e as
// ultimas entre si).
void stitchStrips(MeshBuilder& builder, std::span<const std::uint32_t> from, std::span<const std::uint32_t> to);

// A ordem dos triangulos de uma faixa entre duas linhas: a mais curta
// (diagonais menores), ou em leque a partir do comeco de "to" (todos os
// passos em "from" primeiro) ou do fim de "to" (os de "to" primeiro).
enum class StripOrder : std::uint8_t { Shortest, FromFirst, ToFirst };
void stitchStrips(MeshBuilder& builder, std::span<const std::uint32_t> from, std::span<const std::uint32_t> to,
    StripOrder order);

// Costura uma volta recortada (ImprintResult: "loop" com os cortes entre os
// pontos dados "inputs") num anel de cima ("to"), trecho a trecho: o trecho
// entre os pontos i e i+1 vai para o caminho do anel de to[map[i]] ate
// to[map[i+1]]. Sem map: um para um (mesma quantidade). Assim cada parede
// fica entre o lado certo da pegada e o lado certo do topo (sem dobra).
// loopFirst: a volta vem antes do anel no eixo da peca (o furo de onde ela
// sai); falso: depois (o furo onde ela entra, na outra ponta).
// orders: a ordem de cada trecho (vazio = a mais curta em todos).
void stitchCorresponding(MeshBuilder& builder, std::span<const std::uint32_t> loop,
    std::span<const std::uint32_t> inputs, std::span<const std::uint32_t> to, std::span<const std::uint32_t> map = {},
    bool loopFirst = true, std::span<const StripOrder> orders = {});

// Tampa uma volta (um furo) com triangulos no plano medio dela, sem pontos
// novos (Delaunay restrito). A volta no sentido de uma borda da superficie
// em volta (a superficie a esquerda): a tampa fica com a mesma frente.
// Devolve falso se a volta nao for plana o bastante para isso.
bool fillLoop(MeshBuilder& builder, std::span<const std::uint32_t> loop);

// Inverte a ordem de uma volta mantendo o primeiro vertice.
[[nodiscard]] std::vector<std::uint32_t> reversedLoop(std::span<const std::uint32_t> loop);

} // namespace MatterEngine::Modeling
