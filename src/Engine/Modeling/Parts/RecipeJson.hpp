#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>

// A leitura das receitas das pecas (partes/*.json): campos que faltam ficam
// com o padrao; valores invalidos lancam std::invalid_argument com o nome do
// campo (a mensagem que o modelador ve).
namespace MatterEngine::Modeling {

// Curva [[t, valor], ...] com t crescente em [0, maximum] e valores finitos.
void readCurve(const nlohmann::json& document, const char* key, ProfileCurve& curve, float maximum = 1.0f);
void readColor(const nlohmann::json& colors, const char* key, Color& color);

template <typename T>
void readValue(const nlohmann::json& document, const char* key, T& value) {
    if (document.contains(key)) value = document.at(key).get<T>();
    if constexpr (std::is_floating_point_v<T>)
        if (!std::isfinite(value)) throw std::invalid_argument(std::string(key) + ": valor nao finito");
}

} // namespace MatterEngine::Modeling
