#include "Engine/Modeling/Import/BlenderExchange.hpp"

#include "Engine/Modeling/Raster/ImageIO.hpp"

#include <nlohmann/json.hpp>

#include <fstream>

namespace MatterEngine::Modeling {

std::optional<BlenderExchange> loadBlenderExchange(const std::filesystem::path& path, const Skeleton& skeleton, std::string* error) {
    const auto fail = [&](const std::string& message) -> std::optional<BlenderExchange> {
        if (error != nullptr) *error = "blender: " + message;
        return std::nullopt;
    };
    std::ifstream file(path);
    if (!file) return fail("nao abriu " + path.string());
    nlohmann::json document;
    try {
        file >> document;
        if (document.value("schema", std::string()) != "matter.troca_blender.v1") return fail("schema diferente de matter.troca_blender.v1");
        Mesh mesh;
        for (const auto& p : document.at("positions")) mesh.positions.push_back({ p.at(0).get<float>(), p.at(1).get<float>(), p.at(2).get<float>() });
        for (const auto& t : document.at("triangles")) mesh.triangles.push_back({ t.at(0).get<std::uint32_t>(), t.at(1).get<std::uint32_t>(), t.at(2).get<std::uint32_t>() });
        mesh.trianglePart = document.at("trianglePart").get<std::vector<std::uint16_t>>();
        mesh.triangleMaterial = document.at("triangleMaterial").get<std::vector<std::uint16_t>>();
        if (document.contains("trianglePolygon")) mesh.trianglePolygon = document.at("trianglePolygon").get<std::vector<std::uint32_t>>();
        for (const auto& corners : document.at("triangleUv")) {
            std::array<Uv, 3> uv {};
            for (std::size_t k = 0; k < 3; ++k) uv[k] = { corners.at(k).at(0).get<float>(), corners.at(k).at(1).get<float>() };
            mesh.triangleUv.push_back(uv);
        }
        for (const auto& part : document.at("parts")) {
            MeshPart item;
            item.name = part.at("name").get<std::string>();
            item.region = part.value("region", std::string());
            item.connectedTo = part.value("connectedTo", std::vector<std::string>());
            item.openings = part.value("openings", std::vector<std::string>());
            item.root = part.value("root", false);
            mesh.parts.push_back(std::move(item));
        }
        for (const auto& material : document.at("materials")) {
            const auto color = material.at("color");
            mesh.materials.push_back({ material.at("name").get<std::string>(), Color { color.at(0).get<float>(), color.at(1).get<float>(), color.at(2).get<float>(), 1.0f },
                material.value("texture", -1) });
        }
        for (const auto& name : document.at("textures")) {
            std::string message;
            std::optional<Image> image = readImage((path.parent_path() / name.get<std::string>()).string(), &message);
            if (!image) return fail("textura " + name.get<std::string>() + ": " + message);
            mesh.textures.push_back({ name.get<std::string>(), std::move(*image) });
        }
        // As juntas do arquivo para as do esqueleto, pelo nome.
        std::vector<int> joint;
        for (const auto& item : document.at("joints")) {
            const std::string name = item.at("name").get<std::string>();
            const int found = skeleton.find(name);
            if (found < 0) return fail("a junta " + name + " nao existe no esqueleto do jogo");
            joint.push_back(found);
        }
        for (const auto& pairs : document.at("weights")) {
            SkinWeights weights {};
            std::size_t slot = 0;
            for (const auto& pair : pairs) {
                if (slot == 4) return fail("mais de quatro pesos num vertice");
                const std::size_t index = pair.at(0).get<std::size_t>();
                if (index >= joint.size()) return fail("peso numa junta fora da lista");
                weights.joints[slot] = static_cast<std::uint16_t>(joint[index]);
                weights.weights[slot] = pair.at(1).get<float>();
                ++slot;
            }
            mesh.skin.push_back(weights);
        }
        const std::vector<std::string> problems = mesh.validateLayout();
        if (!problems.empty()) return fail(problems.front());
        if (mesh.skin.size() != mesh.positions.size()) return fail("os pesos nao batem com os vertices");
        BlenderExchange exchange;
        if (document.contains("surface")) {
            std::string message;
            std::optional<Appearance::SurfaceTemplate> surface =
                Appearance::parseSurfaceTemplate(document, mesh.triangles, mesh.positions.size(), &message);
            if (!surface) return fail(message);
            exchange.surface = std::make_shared<const Appearance::SurfaceTemplate>(std::move(*surface));
        }
        exchange.mesh = std::move(mesh);
        return exchange;
    } catch (const std::exception& exception) {
        return fail(exception.what());
    }
}

} // namespace MatterEngine::Modeling
