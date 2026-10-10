#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// O braco em numeros (partes/braco.json): a pele da saida do punho da manga
// ate o pulso, o braco e o antebraco (aberto no pulso sem a mao). Ao longo
// do braco, distancias desde a junta do ombro; a secao em volta do osso.
struct ArmRecipe {
    // A secao por estacao: ate onde a pele vai para a frente, para tras,
    // para cima e para baixo do eixo, e o expoente da superelipse.
    ProfileCurve front;
    ProfileCurve back;
    ProfileCurve top;
    ProfileCurve bottom;
    ProfileCurve exponent;
    // O centro da secao a frente do osso (o braco nao gira em volta do
    // proprio centro), por estacao.
    ProfileCurve forward;
    // O olecrano (a ponta do cotovelo, atras): quanto sai, a meia altura
    // (ao longo do braco) e a meia abertura (radianos) em volta da junta.
    float olecranonProtrusion = 0.006f;
    float olecranonHalfLength = 0.02f;
    float olecranonHalfAngle = 0.7f;
    // Os aneis depois do punho (estacoes; o ultimo e o do pulso, na junta da
    // mao) e de onde a pele passa a ser do antebraco (a parte do menu).
    std::vector<float> rings;
    float forearmFrom = 0.345f;
    float jitter = 0.0018f;
    unsigned seed = 53;

    Color skinColor = rgb(212, 152, 106);

    [[nodiscard]] static ArmRecipe defaults();
};

[[nodiscard]] std::optional<ArmRecipe> parseArmRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
