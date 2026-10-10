#pragma once

#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// As partes do corpo do menu de inspecao (cabeca, mao direita, canela...),
// descritas em dados (regions.json): nao cada osso, so as principais.
struct BodyRegion {
    std::string id;
    std::string label;
    Side side = Side::Center;
    // Juntas raiz: a regiao fica com elas e com os descendentes que nenhuma
    // outra regiao reivindica. Um osso pertence a regiao da junta de onde
    // ele sai (o antebraco e o osso do cotovelo ao pulso).
    std::vector<std::string> roots;
    // O corpo todo (a "Visao geral").
    bool overview = false;
    // O angulo do close-up (graus; ver OrbitCamera).
    float yawDegrees = 25.0f;
    float pitchDegrees = 10.0f;
    // Folga do enquadramento.
    float margin = 1.25f;
};

class BodyRegions {
public:
    std::vector<BodyRegion> regions;

    [[nodiscard]] static std::optional<BodyRegions> parse(std::string_view json, std::string* error = nullptr);
    [[nodiscard]] static std::optional<BodyRegions> load(const std::string& path, std::string* error = nullptr);

    [[nodiscard]] int find(std::string_view id) const;
    // A regiao de cada junta (-1 = nenhuma).
    [[nodiscard]] std::vector<int> assign(const Skeleton& skeleton) const;
    // Raizes que nao existem no esqueleto, ids repetidos.
    [[nodiscard]] std::vector<std::string> validate(const Skeleton& skeleton) const;
    // A caixa da regiao na pose: as juntas dela e os ossos que saem delas,
    // com o raio do corpo em volta.
    [[nodiscard]] Box3 bounds(int region, const Skeleton& skeleton, const SkeletonPose& pose,
        std::span<const int> assignment) const;
};

} // namespace MatterEngine::Modeling
