// Os objetos procedurais no jogo (Engine/Objects): a malha da GPU (os
// vertices por face, o mapa, a textura), o corpo fisico (a massa, as formas,
// o material) e o laboratorio de fisica com o Jolt de verdade - a caixa cai e
// para numa face, quica pouco (madeira no concreto), escorrega na rampa
// ingreme e para na suave, a pilha fica de pe e a bola de futebol a empurra.

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Objects/ObjectAsset3D.hpp"
#include "Engine/Objects/ObjectPhysicsLab3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace MatterEngine;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

Modeling::ProceduralObject crate() {
    std::string error;
    const auto recipe = Modeling::readObjectRecipe(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/objects/wood_crate.json", &error);
    require(recipe.has_value(), error);
    auto object = Modeling::generateObject(*recipe, &error);
    require(object.has_value(), error);
    return *object;
}

void testRenderData(const Modeling::ProceduralObject& object) {
    const ObjectRenderData3D data = objectRenderData3D(object);
    // As 6 faces planas: 4 vertices cada (a quina separa a normal e o mapa).
    require(data.mesh.vertices.size() == 24 && data.mesh.indices.size() == 36, "24 vertices e 36 indices");
    for (const MeshVertex3D& vertex : data.mesh.vertices) {
        require(std::abs(vertex.normal.length() - 1.0f) < 1e-3f, "normal unitaria");
        // A normal da face: um eixo so.
        const float largest = std::max({ std::abs(vertex.normal.x), std::abs(vertex.normal.y), std::abs(vertex.normal.z) });
        require(largest > 0.999f, "a normal e a da face");
        require(vertex.uv.x > 0.0f && vertex.uv.x < 1.0f && vertex.uv.y > 0.0f && vertex.uv.y <= 1.0f, "o UV no atlas");
        require(vertex.weights[0] == 0.0f, "estatica (sem pesos)");
    }
    require(std::abs(data.mesh.boundsMax.x - 0.3f) < 1e-5f && std::abs(data.mesh.boundsMin.z + 0.3f) < 1e-5f, "os limites");
    // Cada canto de triangulo na GPU com a posicao e o mapa do da modelagem
    // (o mesmo atlas, a mesma origem em cima: a caixa no jogo e a da tela).
    require(data.mesh.indices.size() == object.mesh.triangles.size() * 3, "um triangulo da GPU por triangulo da modelagem");
    for (std::size_t t = 0; t < object.mesh.triangles.size(); ++t)
        for (std::size_t k = 0; k < 3; ++k) {
            const MeshVertex3D& vertex = data.mesh.vertices[data.mesh.indices[t * 3 + k]];
            const Vec3 position = object.mesh.positions[object.mesh.triangles[t][k]];
            const Modeling::Uv uv = object.mesh.triangleUv[t][k];
            require((vertex.position - position).length() < 1e-6f, "o canto no mesmo lugar");
            require(std::abs(vertex.uv.x - uv.u) < 1e-6f && std::abs(vertex.uv.y - uv.v) < 1e-6f, "o canto com o mesmo mapa");
        }
    require(data.width == static_cast<std::uint32_t>(object.mesh.textures.front().image.width()) && data.rgba.size() == data.width * data.height * 4,
        "a textura RGBA do tamanho da imagem");
}

void testBody(const Modeling::ProceduralObject& object, const MaterialLibrary& materials) {
    const ObjectBodyData3D body = objectBodyData3D(object, materials);
    require(std::abs(body.body.massKg - 14.0f) < 1e-3f && body.body.materialId == "wood", "a massa e o material da receita");
    require(body.shapes.size() == 1 && body.shapes.front().type == PhysicsShapeType3D::Box
            && std::abs(body.shapes.front().halfExtents.x - 0.3f) < 1e-6f,
        "a caixa de colisao");
    require(body.body.motionType == PhysicsMotionType3D::Dynamic, "dinamica");
    Modeling::ProceduralObject unknown = object;
    unknown.physics.material = "nao_existe";
    require(objectBodyData3D(unknown, materials).body.materialId == "default", "material desconhecido vira o default");
}

void testLab(const Modeling::ProceduralObject& object, PhysicsEngine3D& engine, const MaterialLibrary& materials) {
    ObjectPhysicsLab3D lab(engine, materials);
    // A queda: toca o chao, quica pouco e para numa face, em cima do chao.
    ObjectTestSettings3D drop;
    drop.kind = ObjectTestKind3D::Drop;
    drop.height = 1.5f;
    const ObjectTestMetrics3D fall = lab.run(object, drop);
    std::cout << "queda: toque " << fall.firstImpactSpeed << " m/s, quique " << fall.bounceHeight << " m, parou em " << fall.settleTime
              << " s na face " << fall.restFace << "\n";
    // De 1,5 m (mais a inclinacao): ~5,6 m/s no toque.
    require(fall.firstImpactSpeed > 4.5f && fall.firstImpactSpeed < 7.0f, "a queda toca o chao a ~5,6 m/s");
    require(fall.bounceHeight < 0.25f, "madeira no concreto quica pouco");
    require(fall.settled && fall.settleTime < 5.0f, "para");
    require(!lab.bodies().empty(), "o corpo para desenhar");
    const Vec3 rest = lab.bodies().front().position;
    require(std::abs(rest.z - 0.3f) < 0.02f, "parada numa face: o centro a meia altura do chao (" + std::to_string(rest.z) + ")");
    // A rampa: na ingreme escorrega, na suave fica.
    ObjectTestSettings3D steep;
    steep.kind = ObjectTestKind3D::Ramp;
    steep.rampDegrees = 40.0f;
    const ObjectTestMetrics3D slid = lab.run(object, steep, 3.0f);
    ObjectTestSettings3D gentle = steep;
    gentle.rampDegrees = 12.0f;
    const ObjectTestMetrics3D held = lab.run(object, gentle, 3.0f);
    std::cout << "rampa: 40 graus desceu " << slid.slide << " m, 12 graus " << held.slide << " m\n";
    require(slid.slide > 0.5f, "na rampa de 40 graus a caixa escorrega");
    require(std::abs(held.slide) < 0.05f, "na de 12 graus ela fica");
    // A pilha de 3 fica de pe.
    ObjectTestSettings3D stack;
    stack.kind = ObjectTestKind3D::Stack;
    stack.count = 3;
    const ObjectTestMetrics3D pile = lab.run(object, stack, 4.0f);
    std::cout << "pilha: desvio " << pile.stackDrift << " m, caiu " << pile.toppled << "\n";
    require(!pile.toppled && pile.stackDrift < 0.08f, "a pilha de 3 fica de pe");
    // A bola a 15 m/s: empurra a caixa e volta mais devagar.
    ObjectTestSettings3D shot;
    shot.kind = ObjectTestKind3D::BallShot;
    shot.speed = 15.0f;
    const ObjectTestMetrics3D hit = lab.run(object, shot, 4.0f);
    std::cout << "bola: volta a " << hit.ballSpeedAfter << " m/s, caixa ate " << hit.objectSpeedAfter << " m/s\n";
    require(hit.ballSpeedAfter >= 0.0f && hit.ballSpeedAfter < 15.0f, "a bola bate e perde velocidade");
    require(hit.objectSpeedAfter > 0.1f, "a caixa sente a bola");
    // O arremesso anda e para.
    ObjectTestSettings3D toss;
    toss.kind = ObjectTestKind3D::Throw;
    const ObjectTestMetrics3D thrown = lab.run(object, toss, 6.0f);
    std::cout << "arremesso: andou " << thrown.travel << " m, parou em " << thrown.settleTime << " s\n";
    require(thrown.travel > 1.0f && thrown.settled, "o arremesso anda e para");
}

} // namespace

int main() {
    try {
        PhysicsEngine3D engine;
        const MaterialLibrary materials;
        const Modeling::ProceduralObject object = crate();
        testRenderData(object);
        testBody(object, materials);
        testLab(object, engine, materials);
    } catch (const std::exception& exception) {
        std::cerr << "MatterObjectPhysics test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterObjectPhysics tests passed (" << checks << " checagens)\n";
    return 0;
}
