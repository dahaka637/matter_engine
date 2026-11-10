// Usa os carregadores e a fisica reais da engine, sem abrir Workbench ou RL.
#include "Engine/Geometry/GltfLoader.hpp"
#include "Engine/Geometry/GltfPhysicsMetadata3D.hpp"
#include "Engine/Core/TaskScheduler.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>

using namespace MatterEngine;

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main(int argc, char** argv) try {
    require(argc == 2 || (argc == 3 && std::string_view(argv[2]) == "--fechado"),
        "Uso: verificar_engine.sh mapa.glb [--fechado]");
    const LoadedGltfModel map = loadGltfModel(argv[1]);
    require(!map.parts.empty(), "Mapa sem geometria");
    MaterialLibrary materials;
    auto scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings{1});
    PhysicsEngine3D physics(scheduler);
    PhysicsSceneSettings3D settings;
    settings.workerThreadCount = 1;
    settings.runJobsInline = true;
    auto scene = physics.createScene(settings, materials);
    std::size_t bodyId = 0, spawns = 0;
    for (const auto& image : map.images)
        require(image.width > 0 && image.width <= 4096 && image.height > 0 && image.height <= 4096 && !image.rgbaPixels.empty(),
            "Textura de mapa ausente ou dimensoes inesperadas");
    for (const auto& part : map.parts) {
        const auto metadata = parseGltfPhysicsMetadata(part.extras);
        require(materials.find(metadata.materialId) != nullptr, "Material fisico desconhecido");
        require(metadata.collisionMode == ImportedCollisionMode3D::StaticTriangleMesh,
            "Cenario deve usar colisao estatica");
        require(part.materialIndex >= 0 && map.materials.at(part.materialIndex).imageIndex >= 0,
            "Material sem textura");
        const auto& mesh = part.mesh;
        for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
            const auto& a = mesh.vertices.at(mesh.indices.at(i));
            const auto& b = mesh.vertices.at(mesh.indices.at(i + 1));
            const auto& c = mesh.vertices.at(mesh.indices.at(i + 2));
            const auto ab = b.position - a.position, ac = c.position - a.position;
            const Vec3 n{ab.y*ac.z-ab.z*ac.y, ab.z*ac.x-ab.x*ac.z, ab.x*ac.y-ab.y*ac.x};
            require(n.x*a.normal.x+n.y*a.normal.y+n.z*a.normal.z > 1e-7f,
                "Triangulo degenerado ou winding invertido");
        }
        for (const auto& v : mesh.vertices)
            require(std::isfinite(v.position.x) && std::isfinite(v.position.y)
                && std::isfinite(v.position.z) && std::isfinite(v.uv.x) && std::isfinite(v.uv.y),
                "Vertice nao finito");
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::TriangleMesh;
        shape.mesh = physics.cookStaticTriangleMesh(mesh, metadata.materialId);
        shape.materialId = metadata.materialId;
        PhysicsBodyDefinition3D body;
        body.motionType = PhysicsMotionType3D::Static;
        body.entityId = ++bodyId;
        body.materialId = metadata.materialId;
        const auto handle = scene->createBody(body, std::span(&shape, 1));
        require(scene->contains(handle), "Jolt nao criou o cenario");
    }
    for (const auto& entity : map.entities) {
        if (entity.name != "info_player_start") continue;
        ++spawns;
        require(entity.hasOrientation, "Spawn sem orientacao");
        PhysicsRayHit3D hit;
        require(scene->raycastStatic({entity.position + Vec3{0, 0, 0.2f}, {0, 0, -1}}, 4.0f, hit),
            "Nascimento sem piso nos 4 metros abaixo dos pes");
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Sphere;
        shape.radius = 0.1f;
        shape.materialId = "rubber";
        PhysicsBodyDefinition3D body;
        body.entityId = 100000 + spawns;
        body.position = hit.position + Vec3{0, 0, 1};
        body.materialId = "rubber";
        const auto ball = scene->createBody(body, std::span(&shape, 1));
        for (int step = 0; step < 240; ++step) scene->simulate(1.0f / 120.0f);
        const float height = scene->bodyState(ball).position.z - hit.position.z;
        require(height > 0.05f && height < 0.25f, "Corpo de prova atravessou o piso");
        scene->destroyBody(ball);
    }
    require(spawns > 0, "Nenhum nascimento no GLB");
    if (argc == 3) {
        const Vec3 directions[]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        for (int x=-20; x<=20; x+=4) for (int y=-10; y<=10; y+=4) {
            for (const auto direction : directions) {
                PhysicsRayHit3D hit;
                require(scene->raycastStatic({{float(x),float(y),1.7f},direction},120,hit),
                    "Amostra do interior escapou: parede, piso ou cobertura com vao");
            }
        }
        std::cout << "OK: 396 raios do volume da quadra encontram a casca fechada\n";
    }
    std::cout << "OK: GltfLoader, texturas, UV, winding, materiais e colisao Jolt; "
              << map.parts.size() << " partes, " << spawns << " nascimentos\n";
    return 0;
} catch (const std::exception& error) {
    std::cerr << "Mapa rejeitado pela engine: " << error.what() << '\n';
    return 1;
}
