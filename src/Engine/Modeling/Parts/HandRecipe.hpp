#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// O tamanho de um dedo ao longo dele (meias medidas, metros): a largura e a
// espessura na base (MCP), na junta do meio (PIP), na da ponta (DIP) e
// perto da ponta. No polegar: no MCP, no IP, no meio da ultima falange e
// perto da ponta.
struct FingerSize {
    std::array<float, 4> width {};
    std::array<float, 4> thickness {};
};

// A mao em numeros (partes/mao.json): a palma do pulso ate os nos dos dedos,
// os quatro dedos e o polegar, uma superficie so com o antebraco. A palma no
// referencial da mao: ao longo (do pulso para o dedo medio), para o lado do
// polegar e para o dorso; estacoes em metros desde a junta da mao (o pulso).
struct HandRecipe {
    // A secao da palma por estacao: ate onde vai para o lado do polegar e
    // para o do minimo, para o dorso e para a palma (a partir do eixo), e o
    // expoente da superelipse. Na estacao 0 ela e o anel do pulso do braco.
    ProfileCurve radial;
    ProfileCurve ulnar;
    ProfileCurve back;
    ProfileCurve palm;
    ProfileCurve exponent;
    // Os aneis da palma antes do anel dos nos (que segue a linha das juntas
    // dos dedos: o dorso nos nos, a palma na prega da base dos dedos).
    std::vector<float> palmRings;
    // Os nos (MCP): o quanto o alto deles salta acima do dorso e a meia
    // largura em relacao a base do dedo.
    float knuckleBump = 0.0025f;
    float knuckleWidth = 1.12f;
    // O quanto as bordas da palma (do lado do indicador e do minimo) passam
    // dos nos.
    float palmEdge = 0.003f;
    // A membrana entre os dedos: a fracao da primeira falange onde fica a
    // forquilha (a prega da palma fica um pouco antes).
    float webDepth = 0.35f;
    // Indicador, medio, anelar e minimo; o polegar.
    std::array<FingerSize, 4> fingers {};
    FingerSize thumb;
    // O no das juntas do meio e da ponta (no dorso, metros), onde a ponta
    // arredondada comeca (fracao da ultima falange) e o quanto ela encolhe.
    float jointBump = 0.0012f;
    float tipStart = 0.8f;
    float tipScale = 0.8f;
    // O polegar: o primeiro dos tres aneis da palma da base dele (a janela
    // por onde ele sai), o quanto a eminencia tenar salta em volta e o giro
    // da unha em volta do eixo dele (radianos, a partir do dorso do osso).
    int thumbFromRing = 1;
    float thenarBulge = 0.007f;
    float thumbRoll = -0.6f;
    // O facetado (metros; nos dedos, a metade).
    float jitter = 0.0008f;
    unsigned seed = 71;

    Color skinColor = rgb(212, 152, 106);
    // A unha: a face de cima da ponta de cada dedo (um pouco mais clara).
    Color nailColor = rgb(222, 170, 134);

    [[nodiscard]] static HandRecipe defaults();
};

[[nodiscard]] std::optional<HandRecipe> parseHandRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
