#pragma once

#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include <filesystem>
#include <string>
#include <vector>

// O jogador montado levado ao jogo (o personagem do laboratorio): o perfil
// fisico nas medidas do corpo novo, a pele e o atlas. O rig (as juntas e os
// eixos delas, os limites, os motores e as massas) e o do perfil atual - so
// as posicoes e os colisores mudam, entao as animacoes continuam valendo:
//
//  - o repouso da fisica e a pose T do esqueleto medido (a ligacao do jogo
//    com as medidas novas): as ancoras nas juntas e os links no mesmo lugar
//    relativo do perfil atual (na escala do osso);
//  - os colisores medidos na malha nessa pose (membros conicos, o tronco em
//    capsulas deitadas, a mao aberta, a caixa da chuteira com a sola no chao,
//    as coxas a 2,4 cm uma da outra, a folga do braco e das coxas nas
//    animacoes de locomocao), e a cabeca e o pescoco tambem
//    (docs/RAGDOLL_CHARACTER_PIPELINE.md);
//  - a pele (versao 2): a malha fica na pose A dela e cada osso guarda o
//    repouso e a ligacao da malha - o jogo deforma igual a inspecao; os dedos
//    levam os dados das maos do jogo (a mao solta e o punho aprovados), e os
//    auxiliares (joelho, cotovelo, pulso...) vao junto;
//  - as normais lisas por grupo (as quinas de "creaseDegrees"); o facetado
//    fica no manifesto (o shader do jogo mistura);
//  - o espaco do jogo tem a pelve na origem (o chao em -standingRootHeight,
//    como o codigo da locomocao conta): tudo sobe a altura que a pelve tem
//    abaixo da origem no espaco do modelo.
namespace MatterEngine::Modeling {

struct GameCharacterExportSettings {
    float creaseDegrees = 45.0f;
    float facet = 0.5f;
    std::string skinFile = "jogador.skin.json";
    std::string albedoFile = "jogador_atlas.png";
    // O template de superficie (com o modelo que o tem): os triangulos na ordem da
    // pele e as costuras nos vertices da modelagem ("source" de cada vertice da pele).
    std::string surfaceFile = "jogador.superficie.json";
    std::string displayName = "Jogador";
};

struct GameCharacterExportReport {
    // Os colisores e as medidas, uma linha por item.
    std::vector<std::string> lines;
    std::size_t vertices = 0;
    std::size_t triangles = 0;
    std::size_t bones = 0;
    // A altura da pelve do chao no repouso, no perfil de antes e no novo.
    float pelvisHeightBefore = 0.0f;
    float pelvisHeightAfter = 0.0f;
};

// "loaded": o modelo montado com o personagem atual do jogo (o molde do
// perfil e do manifesto). Escreve na pasta o manifesto (character.json), o
// perfil (com o nome do manifesto atual), a pele e o atlas.
[[nodiscard]] bool exportGameCharacter(const LoadedPlayerModel& loaded, const std::filesystem::path& outputDirectory,
    const GameCharacterExportSettings& settings, GameCharacterExportReport& report, std::string& error);

} // namespace MatterEngine::Modeling
