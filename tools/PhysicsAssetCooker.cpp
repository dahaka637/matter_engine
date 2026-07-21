#include "Engine/Geometry/PhysicalAsset3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <array>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>

namespace {

constexpr std::array<std::string_view, 6> PropFiles {
    "wood_chair.glb",
    "anvil.glb",
    "plastic_barrel.glb",
    "soccer_ball.glb",
    "wood_crate.glb",
    "car_tire.glb"
};

} // namespace

int main(int argumentCount, char** arguments) {
    try {
        const std::string assetsDirectory = argumentCount > 1
            ? arguments[1] : MATTERENGINE_ASSETS_DIR;
        MatterEngine::MaterialLibrary materials;
        MatterEngine::PhysicsEngine3D physics;
        for (const std::string_view fileName : PropFiles) {
            const std::string path = assetsDirectory + "/models/props/"
                + std::string(fileName);
            const auto started = std::chrono::steady_clock::now();
            std::cout << "Cooking " << fileName << "...\n";
            const MatterEngine::PhysicalAsset3D asset =
                MatterEngine::loadPhysicalAsset3D(path, materials, physics);
            const float seconds = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - started).count();
            std::cout << "  " << asset.collision.shapes.size()
                << " shapes, " << asset.bodyTemplate.massKg << " kg, "
                << seconds << " s\n";
        }
        std::cout << "Caches fisicos atualizados.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Falha no cooking fisico: " << error.what() << '\n';
        return 1;
    }
}
