#include "Engine/Modeling/Parts/RecipeJson.hpp"

namespace MatterEngine::Modeling {

void readCurve(const nlohmann::json& document, const char* key, ProfileCurve& curve, float maximum) {
    if (!document.contains(key)) return;
    const auto& value = document.at(key);
    if (!value.is_array() || value.empty() || value.size() > 128)
        throw std::invalid_argument(std::string(key) + ": curva vazia ou longa demais");
    std::vector<std::pair<float, float>> points;
    for (const nlohmann::json& point : value) {
        if (!point.is_array() || point.size() != 2) throw std::invalid_argument(std::string(key) + ": esperado [t,valor]");
        const float t = point.at(0).get<float>(), y = point.at(1).get<float>();
        if (!std::isfinite(t) || !std::isfinite(y) || t < 0 || t > maximum || (!points.empty() && t <= points.back().first))
            throw std::invalid_argument(std::string(key) + ": t crescente em [0," + std::to_string(maximum) + "] e valores finitos");
        points.emplace_back(t, y);
    }
    curve.set(std::move(points));
}

void readColor(const nlohmann::json& colors, const char* key, Color& color) {
    if (!colors.contains(key)) return;
    const nlohmann::json& value = colors.at(key);
    if (!value.is_array() || value.size() != 3) throw std::invalid_argument(std::string(key) + ": esperado RGB");
    for (const auto& channel : value)
        if (!channel.is_number_integer() || channel.get<int>() < 0 || channel.get<int>() > 255)
            throw std::invalid_argument(std::string(key) + ": RGB fora de [0,255]");
    color = rgb(value.at(0).get<int>(), value.at(1).get<int>(), value.at(2).get<int>());
}

} // namespace MatterEngine::Modeling
