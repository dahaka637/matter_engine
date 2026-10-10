#include "Engine/Modeling/Skeleton/BodyRegions.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <set>
#include <sstream>

namespace MatterEngine::Modeling {

std::optional<BodyRegions> BodyRegions::parse(std::string_view json, std::string* error) {
    try {
        const nlohmann::json document = nlohmann::json::parse(json);
        BodyRegions result;
        for (const nlohmann::json& entry : document.at("regions")) {
            BodyRegion region;
            region.id = entry.at("id").get<std::string>();
            region.label = entry.value("label", region.id);
            const std::string side = entry.value("side", std::string("centro"));
            region.side = side == "esquerda" ? Side::Left : side == "direita" ? Side::Right : Side::Center;
            region.overview = entry.value("overview", false);
            if (entry.contains("roots")) region.roots = entry.at("roots").get<std::vector<std::string>>();
            if (entry.contains("view")) {
                region.yawDegrees = entry.at("view").value("yaw", region.yawDegrees);
                region.pitchDegrees = entry.at("view").value("pitch", region.pitchDegrees);
            }
            region.margin = entry.value("margin", region.margin);
            result.regions.push_back(std::move(region));
        }
        return result;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("regions.json invalido: ") + exception.what();
        return std::nullopt;
    }
}

std::optional<BodyRegions> BodyRegions::load(const std::string& path, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        if (error != nullptr) *error = "nao abriu " + path;
        return std::nullopt;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return parse(buffer.str(), error);
}

int BodyRegions::find(std::string_view id) const {
    for (std::size_t i = 0; i < regions.size(); ++i)
        if (regions[i].id == id) return static_cast<int>(i);
    return -1;
}

std::vector<int> BodyRegions::assign(const Skeleton& skeleton) const {
    std::vector<int> rootRegion(skeleton.joints.size(), -1);
    for (std::size_t r = 0; r < regions.size(); ++r)
        for (const std::string& root : regions[r].roots) {
            const int joint = skeleton.find(root);
            if (joint >= 0) rootRegion[static_cast<std::size_t>(joint)] = static_cast<int>(r);
        }
    // Pais antes dos filhos: cada junta herda a regiao do pai se nao e raiz.
    std::vector<int> result(skeleton.joints.size(), -1);
    for (std::size_t i = 0; i < skeleton.joints.size(); ++i) {
        const int parent = skeleton.joints[i].parent;
        result[i] = rootRegion[i] >= 0 ? rootRegion[i] : parent >= 0 ? result[static_cast<std::size_t>(parent)] : -1;
    }
    return result;
}

std::vector<std::string> BodyRegions::validate(const Skeleton& skeleton) const {
    std::vector<std::string> problems;
    std::set<std::string> ids;
    for (const BodyRegion& region : regions) {
        if (!ids.insert(region.id).second) problems.push_back("regiao repetida: " + region.id);
        for (const std::string& root : region.roots)
            if (skeleton.find(root) < 0) problems.push_back("regiao " + region.id + ": junta inexistente " + root);
        if (!region.overview && region.roots.empty()) problems.push_back("regiao " + region.id + " sem juntas");
    }
    return problems;
}

Box3 BodyRegions::bounds(int region, const Skeleton& skeleton, const SkeletonPose& pose,
    std::span<const int> assignment) const {
    Box3 box;
    if (region < 0 || region >= static_cast<int>(regions.size())) return box;
    const bool overview = regions[static_cast<std::size_t>(region)].overview;
    const auto position = [&](std::size_t joint) {
        return joint < pose.joints.size() ? pose.joints[joint].position : skeleton.joints[joint].bind.position;
    };
    for (std::size_t i = 0; i < skeleton.joints.size(); ++i) {
        const int owner = i < assignment.size() ? assignment[i] : -1;
        if (!overview && owner != region) continue;
        const float radius = skeleton.joints[i].radius;
        box.include(position(i), radius);
        // O osso que sai daqui ate cada filho.
        for (std::size_t child = i + 1; child < skeleton.joints.size(); ++child)
            if (skeleton.joints[child].parent == static_cast<int>(i)) box.include(position(child), radius);
    }
    return box;
}

} // namespace MatterEngine::Modeling
