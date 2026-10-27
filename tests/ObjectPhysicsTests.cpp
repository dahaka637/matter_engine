// Os objetos procedurais no jogo (Engine/Objects): a malha da GPU (os
// vertices por face, o mapa, a textura), o corpo fisico (a massa, as formas,
// o material) e o laboratorio de fisica com o Jolt de verdade - a caixa cai e
// para numa face, quica pouco (madeira no concreto), escorrega na rampa
// ingreme e para na suave, a pilha fica de pe e a bola de futebol a empurra.

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Objects/ObjectAsset3D.hpp"
#include "Engine/Objects/ObjectNetLab3D.hpp"
#include "Engine/Objects/ObjectPhysicsLab3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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

Modeling::ProceduralObject goal() {
    std::string error;
    const auto recipe = Modeling::readObjectRecipe(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/objects/trave.json", &error);
    require(recipe.has_value(), error);
    auto object = Modeling::generateObject(*recipe, &error);
    require(object.has_value(), error);
    return *object;
}

// Roda o laboratorio da trave ate a condicao ou o tempo (s).
template <typename Done>
void runUntil(ObjectNetLab3D& lab, float seconds, Done done) {
    for (int t = 0; t < static_cast<int>(seconds * 120.0f) && !done(); ++t) lab.step();
}

// A trave com a rede na cena com o Jolt de verdade e a bola inflada: a rede
// assentada dorme sem custo; o chute no meio entra, estufa a rede e para na
// frente dela (a freada e a da rede: ela pode devolver a bola pela boca, nunca
// deixa passar), e a rede volta a dormir; o giro da bola nao some no contato
// (so o atrito o muda); na trave, a bola volta para o campo; largada no teto da
// rede, ela fica apoiada (tambem com a rede dormindo); e a caixa atirada nao
// atravessa a rede.
void testGoalNet(const Modeling::ProceduralObject& crateObject, PhysicsEngine3D& engine, const MaterialLibrary& materials) {
    const Modeling::ProceduralObject object = goal();
    ObjectNetLab3D lab(engine, materials);
    std::string error;
    const auto startBegin = std::chrono::steady_clock::now();
    require(lab.start(object, &error), "o laboratorio da trave: " + error);
    std::printf("trave: a cena (a rede assentando) montou em %.0f ms\n",
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startBegin).count());
    require(lab.netStatus().sleeping, "a rede assentada dorme");
    require(std::abs(lab.goalLineX() + 0.12f) < 1e-3f, "a linha do gol no fundo dos postes");
    // Dormindo: quase sem custo.
    float asleep = 0.0f;
    for (int t = 0; t < 120; ++t) {
        lab.step();
        asleep += lab.metrics().netMilliseconds;
    }
    asleep /= 120.0f;
    // O chute no meio, a 25 m/s, da marca do penalti.
    NetShotSettings3D shot;
    shot.speed = 25.0f;
    shot.distance = 11.0f;
    shot.aimHeight = 0.45f;
    lab.shoot(shot);
    runUntil(lab, 6.0f, [&] { return lab.metrics().stopTime >= 0.0f; });
    const NetShotMetrics3D metrics = lab.metrics();
    const Vec3 rest = lab.bodies().back().position;
    std::printf("trave: dormindo %.4f ms/tick; chute 25 m/s: na linha %.1f m/s, gol %d, estufou %.2f m, parou em %.2f s em "
                "(%.2f %.2f %.2f), fio no maximo %.1f%%, rede %.3f ms/tick media e %.3f de pico\n",
        asleep, metrics.speedAtLine, metrics.goal ? 1 : 0, metrics.bulge, metrics.stopTime, rest.x, rest.y, rest.z,
        metrics.peakStrain * 100.0f, metrics.netAverageMilliseconds, metrics.netPeakMilliseconds);
    require(asleep < 0.05f, "a rede dormindo custa quase nada");
    require(metrics.crossed && metrics.goal, "o chute no meio e gol");
    require(metrics.speedAtLine > 19.0f, "a bola chega a linha forte");
    require(metrics.bulge > 0.15f && metrics.bulge < 1.6f, "a rede estufa");
    require(metrics.stopTime >= 0.0f, "a bola para");
    require(rest.x > -object.size.x - 0.15f && rest.z > 0.0f, "a bola para na frente da rede (no gol ou fora, pela boca)");
    require(metrics.peakStrain < 0.16f, "nenhum fio passa do limite");
    runUntil(lab, 8.0f, [&] { return lab.netStatus().sleeping; });
    require(lab.netStatus().sleeping, "a rede volta a dormir");
    require(lab.bodies().back().position.z > 0.0f, "e a bola continua no chao");
    // O giro: com efeito por cima, a bola que encosta na rede continua girando. Antes um freio o
    // zerava em 2 ticks e ele ficava zero o contato inteiro, sem atrito nem reacao. Agora so o atrito
    // o muda: no impacto ele vira rolamento rede acima (a bola sobe encostada) e volta.
    lab.reset();
    NetShotSettings3D spinning;
    spinning.speed = 25.0f;
    spinning.distance = 11.0f;
    spinning.aimHeight = 0.45f;
    spinning.topSpin = 30.0f;
    lab.shoot(spinning);
    float spinBefore = 0.0f, spinAfter = -1.0f;
    for (int t = 0, touched = -1; t < 240 && spinAfter < 0.0f; ++t) {
        const float spin = lab.scene()->inflatedBallState(lab.balls().back()).angularVelocity.length();
        if (touched < 0) spinBefore = spin;
        lab.step();
        if (touched < 0 && lab.netStatus().contacts > 0) touched = t;
        if (touched >= 0 && t == touched + 10) spinAfter = lab.scene()->inflatedBallState(lab.balls().back()).angularVelocity.length();
    }
    std::printf("trave: o giro da bola na rede, %.1f rad/s antes do contato e %.1f 10 ticks depois\n", spinBefore, spinAfter);
    require(spinBefore > 20.0f && spinAfter > 0.2f * spinBefore, "o giro da bola nao some no contato com a rede");
    // O giro em espiral (em volta do rumo do chute): contra a rede do fundo, de frente, o atrito quase
    // nao tem braco para ele, entao ele se mantem; um freio de giro o zeraria.
    {
        lab.reset();
        PhysicsScene3D& spinScene = *lab.scene();
        InflatedBallDefinition3D spiral;
        spiral.position = { 9.0f, 0.0f, 1.0f };
        const Vec3 spin { -30.0f, 0.0f, 0.0f };
        spiral.linearVelocity = ObjectNetLab3D::launchVelocity(spiral.position, { lab.goalLineX(), 0.0f, 1.0f }, 20.0f, spin);
        spiral.angularVelocity = spin;
        const InflatedBallHandle3D spiralBall = spinScene.createInflatedBall(spiral);
        float before = 0.0f, after = -1.0f;
        for (int t = 0, touched = -1; t < 240 && after < 0.0f; ++t) {
            if (touched < 0) before = -spinScene.inflatedBallState(spiralBall).angularVelocity.x;
            lab.step();
            if (touched < 0 && lab.netStatus().contacts > 0) touched = t;
            if (touched >= 0 && t == touched + 10) after = -spinScene.inflatedBallState(spiralBall).angularVelocity.x;
        }
        std::printf("trave: o giro em espiral, %.1f rad/s antes do contato e %.1f 10 ticks depois\n", before, after);
        require(before > 25.0f && after > 0.6f * before, "o giro em espiral se mantem no contato com a rede");
    }
    // Na trave: a bola volta para o campo.
    lab.reset();
    PhysicsScene3D& scene = *lab.scene();
    InflatedBallDefinition3D ball;
    ball.position = { 9.0f, 3.0f, ball.profile.radiusMeters };
    const Vec3 post { lab.goalLineX() + 0.06f, 0.5f * object.size.y - 0.06f, 1.3f };
    ball.linearVelocity = ObjectNetLab3D::launchVelocity(ball.position, post, 22.0f, {});
    const InflatedBallHandle3D postBall = scene.createInflatedBall(ball);
    for (int t = 0; t < 120; ++t) lab.step();
    const InflatedBallState3D after = scene.inflatedBallState(postBall);
    std::printf("trave: chute na trave volta a (%.2f %.2f %.2f) m/s\n", after.linearVelocity.x, after.linearVelocity.y,
        after.linearVelocity.z);
    require(after.position.x > 0.5f && after.linearVelocity.x > 2.0f, "na trave a bola volta para o campo");
    // A rede acordada sozinha anda 4 subpassos por tick, mesmo com uma bola rolando longe (que pede
    // 16 a cena): andava nos 16 da bola, 4 vezes o custo.
    lab.reset();
    InflatedBallDefinition3D rolling;
    rolling.position = { lab.goalLineX() + 8.0f, 0.0f, rolling.profile.radiusMeters };
    rolling.linearVelocity = { 0.0f, 5.0f, 0.0f };
    lab.scene()->createInflatedBall(rolling);
    lab.scene()->wakeNet(lab.net());
    lab.step();
    require(!lab.netStatus().sleeping && lab.netStatus().substeps == 4,
        "a rede sozinha, com a bola rolando longe, anda 4 subpassos: " + std::to_string(lab.netStatus().substeps));
    // Com uma bola de 18 subpassos, os da rede dividem os 18 (cobrem o tick inteiro).
    lab.reset();
    rolling.profile.substeps = 18;
    lab.scene()->createInflatedBall(rolling);
    lab.scene()->wakeNet(lab.net());
    lab.step();
    require(lab.netStatus().substeps > 0 && 18 % lab.netStatus().substeps == 0,
        "com 18 subpassos na cena, a rede cobre o tick inteiro: " + std::to_string(lab.netStatus().substeps));
    // No teto: a bola pousada fica em cima da rede (e a rede dormindo a segura). (Largada de 1 m, o
    // teto esticado a devolve como uma cama elastica e ela rola para fora pela borda de tras, por
    // cima do fundo: fisica, nao travessia; aqui ela e pousada de 20 cm, no meio do teto.)
    lab.reset();
    PhysicsScene3D& roofScene = *lab.scene();
    InflatedBallDefinition3D dropped;
    dropped.position = { lab.goalLineX() - 0.45f, 0.3f, 2.75f };
    const InflatedBallHandle3D roofBall = roofScene.createInflatedBall(dropped);
    for (int t = 0; t < 120; ++t) lab.step();
    runUntil(lab, 8.0f, [&] { return lab.netStatus().sleeping; });
    const float settledHeight = roofScene.inflatedBallState(roofBall).position.z;
    for (int t = 0; t < 240; ++t) lab.step();
    const float laterHeight = roofScene.inflatedBallState(roofBall).position.z;
    std::printf("trave: bola no teto parou a %.2f m, rede dormindo %d, 2 s depois %.2f m\n", settledHeight,
        lab.netStatus().sleeping ? 1 : 0, laterHeight);
    require(settledHeight > 1.9f && settledHeight < 2.7f, "a bola fica no teto da rede");
    require(std::abs(laterHeight - settledHeight) < 0.02f, "dormindo, a rede continua segurando a bola");
    // A grade de chutes (cantos, rente ao chao, alto, perto dos postes, com
    // angulo), fraco e forte: a bola nunca acaba atras da rede, nem o fio passa
    // do limite.
    float worstStrain = 0.0f, deepest = 0.0f;
    int shots = 0, goals = 0;
    for (const float speed : { 15.0f, 32.0f })
        for (const float across : { -0.95f, -0.5f, 0.0f, 0.6f, 0.95f })
            for (const float height : { 0.05f, 0.5f, 0.95f }) {
                lab.reset();
                NetShotSettings3D grid;
                grid.speed = speed;
                grid.distance = 11.0f;
                grid.angleDegrees = across * 25.0f;
                grid.aimAcross = across;
                grid.aimHeight = height;
                grid.sideSpin = across * 6.0f;
                lab.shoot(grid);
                float lowestX = 1e9f;
                for (int t = 0; t < 360; ++t) {
                    lab.step();
                    lowestX = std::min(lowestX, lab.bodies().back().position.x);
                }
                const Vec3 end = lab.bodies().back().position;
                require(std::isfinite(end.x) && std::isfinite(end.z), "a bola continua finita");
                require(lowestX > -object.size.x - 0.75f && end.x > -object.size.x - 0.3f, "a bola nunca passa da rede");
                worstStrain = std::max(worstStrain, lab.metrics().peakStrain);
                deepest = std::min(deepest, lowestX);
                goals += lab.metrics().goal ? 1 : 0;
                ++shots;
            }
    std::printf("trave: grade de %d chutes, %d gols, a bola mais funda em x = %.2f, fio no maximo %.1f%%\n", shots, goals, deepest,
        worstStrain * 100.0f);
    require(worstStrain < 0.16f, "na grade, nenhum fio passa do limite");
    require(goals >= shots / 2, "a maior parte da grade entra");
    // De perto, na diagonal, no canto do poste (25/10): a bola espremia a rede
    // contra o tubo do lado, um no passava para o outro lado dele e a rede
    // cuspia a bola a 40-60 m/s pela rede do lado.
    {
        const float close[3][7] { { 30.0f, 4.0f, 0.95f, 0.5f, 28.0f, 10.0f, 0.0f }, { 35.0f, 4.0f, 0.95f, 0.5f, 28.0f, -10.0f, 15.0f },
            { 45.0f, 4.0f, -0.95f, 0.5f, -28.0f, -10.0f, 15.0f } };
        float widest = 0.0f, strongest = 0.0f;
        for (const auto& c : close) {
            lab.reset();
            NetShotSettings3D shot;
            shot.speed = c[0];
            shot.distance = c[1];
            shot.aimAcross = c[2];
            shot.aimHeight = c[3];
            shot.angleDegrees = c[4];
            shot.sideSpin = c[5];
            shot.topSpin = c[6];
            lab.shoot(shot);
            for (int t = 0; t < 360; ++t) {
                lab.step();
                const Vec3 p = lab.bodies().back().position;
                if (p.x < -0.3f) widest = std::max(widest, std::abs(p.y));
            }
            strongest = std::max(strongest, lab.metrics().peakStrain);
        }
        std::printf("trave: chutes de perto no canto do poste (30-45 m/s, 4 m): a bola ate |y| = %.2f (o lado %.2f), fio no maximo %.1f%%\n",
            widest, 0.5f * object.size.y, strongest * 100.0f);
        require(widest < 0.5f * object.size.y + 0.3f, "de perto no canto do poste, a bola nao sai pela rede do lado");
        require(strongest < 0.16f, "de perto no canto do poste, nenhum fio passa do limite");
    }
    // A caixa atirada contra a rede: nao atravessa.
    lab.reset();
    NetShotSettings3D toss;
    toss.speed = 9.0f;
    toss.distance = 6.0f;
    toss.aimHeight = 0.4f;
    lab.throwObject(crateObject, toss);
    for (int t = 0; t < 480; ++t) lab.step();
    const Vec3 crate = lab.bodies().back().position;
    std::printf("trave: caixa termina em (%.2f %.2f %.2f), fio no maximo %.1f%%\n", crate.x, crate.y, crate.z,
        lab.metrics().peakStrain * 100.0f);
    require(std::isfinite(crate.x) && crate.x > -object.size.x - 0.6f && crate.z > 0.0f, "a caixa nao atravessa a rede");
}

// A rede desenhada na GPU pela pele dos nos (sem a GPU): a conta do
// scene3d_mesh.vert (soma dos pesos x matriz do no, aplicada ao vertice do
// repouso) com os nos mexidos poe cada ponta de fio no no desenhado de
// drawnNetKnots, e o tubo em volta dele.
void testGoalNetSkin() {
    std::string error;
    const auto recipe = Modeling::readObjectRecipe(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/objects/trave.json", &error);
    const auto object = recipe ? Modeling::generateObject(*recipe, &error) : std::nullopt;
    require(object.has_value() && object->net.has_value(), "a trave com rede: " + error);
    const Vec3 position { 3.0f, -19.05f, 1.28f };
    const Quaternion orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 1.5707963f);
    const ObjectNetRenderData3D render = objectNetRenderData3D(*object, position, orientation);
    const Modeling::ObjectNet& net = *object->net;
    require(render.restKnots.size() == net.knots.size(), "o esqueleto com um no por no da fisica");
    require(render.mesh.vertices.size() == net.drawnSegments.size() * 6 && render.mesh.indices.size() == net.drawnSegments.size() * 18,
        "um tubo de 3 lados por fio desenhado");
    // Os nos mexidos (uma estufada qualquer) e a conta do shader.
    std::vector<Vec3> moved(render.restKnots);
    for (std::size_t i = 0; i < moved.size(); ++i)
        if (net.pinned[i] == 0)
            moved[i] = moved[i] + Vec3 { 0.3f * std::sin(0.7f * static_cast<float>(i)), -0.4f * std::cos(0.3f * static_cast<float>(i)), 0.1f };
    std::vector<Mat4> palette;
    objectNetSkin3D(moved, render.restKnots, palette);
    // (A matriz de coluna, como no shader: a quarta coluna e a translacao.)
    const auto apply = [](const Mat4& m, Vec3 p) {
        return Vec3 { m.at(0, 0) * p.x + m.at(0, 1) * p.y + m.at(0, 2) * p.z + m.at(0, 3),
            m.at(1, 0) * p.x + m.at(1, 1) * p.y + m.at(1, 2) * p.z + m.at(1, 3), m.at(2, 0) * p.x + m.at(2, 1) * p.y + m.at(2, 2) * p.z + m.at(2, 3) };
    };
    std::vector<Vec3> drawn;
    Modeling::drawnNetKnots(net, moved, drawn);
    float worst = 0.0f;
    for (std::size_t s = 0; s < net.drawnSegments.size(); ++s) {
        for (int end = 0; end < 2; ++end) {
            Vec3 center;
            for (int k = 0; k < 3; ++k) {
                const MeshVertex3D& v = render.mesh.vertices[s * 6 + static_cast<std::size_t>(end * 3 + k)];
                Vec3 skinned;
                for (int j = 0; j < 4; ++j) skinned = skinned + apply(palette[v.joints[static_cast<std::size_t>(j)]], v.position) * v.weights[static_cast<std::size_t>(j)];
                center = center + skinned * (1.0f / 3.0f);
            }
            const std::uint32_t knot = end == 0 ? net.drawnSegments[s][0] : net.drawnSegments[s][1];
            worst = std::max(worst, std::sqrt((center - drawn[knot]).lengthSquared()));
        }
    }
    std::printf("trave: a pele da rede na GPU cai no no desenhado com erro de %.2e m\n", worst);
    require(worst < 1e-4f, "a pele da rede (a conta do shader) poe o fio no no desenhado");
}

// A rede forte, na cena de verdade (o Jolt, a bola inflada, o boneco): nada
// passa dela - o boneco fisico atirado, a caixa a 30 m/s, a caixa de 100 kg,
// a bola a 40 m/s - e o boneco guiado (o modo padrao) recebe o empurrao dela
// como empurrao de fora: de leve, quase nada; contra a rede esticada, o de
// parar o corpo (o atleta vira fisico e ela o segura). Tambem o custo da
// rede com um boneco parado dentro do gol, encostado nela e correndo ao lado.
void testGoalNetStrength(PhysicsEngine3D& engine, const MaterialLibrary& materials) {
    std::string error;
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const auto goalRecipe = Modeling::readObjectRecipe(assets + "/objects/trave.json", &error);
    const auto goal = goalRecipe ? Modeling::generateObject(*goalRecipe, &error) : std::nullopt;
    const auto crateRecipe = Modeling::readObjectRecipe(assets + "/objects/wood_crate.json", &error);
    const auto crate = crateRecipe ? Modeling::generateObject(*crateRecipe, &error) : std::nullopt;
    require(goal && crate, "a trave e a caixa: " + error);
    const RagdollProfile3D profile = loadRagdollProfile3D(assets + "/characters/football_player/FootballPlayerV1.ragdoll.json");
    // Atras da estrutura, alem de qualquer estufada: atravessou.
    const float behind = -goal->size.x - 0.6f;

    // O boneco fisico atirado contra o fundo, a 9 m/s.
    {
        ObjectNetLab3D lab(engine, materials);
        require(lab.start(*goal, &error), "o laboratorio da trave: " + error);
        PhysicsScene3D& scene = *lab.scene();
        RagdollSpawnDefinition3D spawn;
        spawn.pelvisPosition = { 1.0f, 0.3f, profile.standingRootHeightMeters + 0.1f };
        spawn.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 3.14159265f);
        const RagdollHandle3D doll = scene.createRagdoll(profile, spawn);
        std::vector<PhysicsBodyState3D> links = scene.ragdollState(doll).links;
        for (PhysicsBodyState3D& link : links) link.linearVelocity = { -9.0f, 0.0f, 1.0f };
        scene.resetRagdollState(doll, links);
        float deepest = 1e9f, strain = 0.0f;
        for (int t = 0; t < 360; ++t) {
            lab.step();
            for (const PhysicsBodyState3D& link : scene.ragdollState(doll).links) deepest = std::min(deepest, link.position.x);
            strain = std::max(strain, lab.netStatus().maximumStrain);
        }
        // Depois, caido/apoiado na rede por 3 s: ela o segura sem contar como
        // pancada (o boneco fisico leva o impulso comum, nao o empurrao de
        // fora, senao o peso dele o deixaria preso no impacto).
        float restingPush = 0.0f;
        for (int t = 0; t < 360; ++t) {
            lab.step();
            restingPush = std::max(restingPush, scene.ragdollState(doll).externalPushNewtonSeconds);
            for (const PhysicsBodyState3D& link : scene.ragdollState(doll).links) deepest = std::min(deepest, link.position.x);
        }
        std::printf("trave: boneco fisico a 9 m/s no fundo: foi ate x = %.2f (atras = %.2f), fio no maximo %.1f%%; apoiado 3 s, "
                    "empurrao de fora %.2f N s\n",
            deepest, behind, strain * 100.0f, restingPush);
        require(deepest > behind, "o boneco fisico atirado nao atravessa a rede");
        require(strain < 0.3f, "a rede segura o boneco sem estourar o fio");
        require(restingPush == 0.0f, "o boneco fisico apoiado na rede nao conta como empurrado");
    }
    // A barreira dos jogadores (as placas no lugar da rede): a capsula contra
    // o fundo 4 s e de lado 4 s para nele e desliza ate o canto; o boneco
    // fisico atirado a 9 m/s contra o lado para na placa; a bola passa por
    // ela e estufa a rede.
    {
        ObjectNetLab3D lab(engine, materials);
        require(lab.start(*goal, &error), "o laboratorio da trave: " + error);
        PhysicsScene3D& scene = *lab.scene();
        std::vector<ObjectBodyData3D> barrier = objectNetBarrier3D(*goal);
        require(barrier.size() == 2 && barrier[0].shapes.size() == 4 && barrier[1].shapes.size() == 1,
            "a barreira da trave: o teto, a parede de tras e os lados para todos; o fundo so para a capsula");
        for (ObjectBodyData3D& part : barrier) {
            part.body.position = lab.objectPosition();
            static_cast<void>(scene.createBody(part.body, part.shapes));
        }
        // A face de dentro da placa do fundo (os pontos pares da casca: os de
        // dentro; a normal para fora) e a das placas do lado.
        const std::vector<Vec3>& back = barrier[1].shapes[0].hullPoints;
        const Vec3 backInner = lab.objectPosition() + back[0];
        const Vec3 outward = (back[1] - back[0]).normalized();
        const auto insideBack = [&](Vec3 p) { return dot(backInner - p, outward); };
        float sideInner = 1e9f;
        for (std::size_t s = 2; s < 4; ++s)
            for (const Vec3& p : barrier[0].shapes[s].hullPoints) sideInner = std::min(sideInner, std::abs(p.y));
        const CharacterMotorSettings3D settings;
        scene.createCharacter({ 2.0f, 0.0f, 0.0f }, settings);
        CharacterMotorCommand3D command;
        command.navigationProxy = true;
        const float axis = 0.5f * settings.standingHeight - settings.radius;
        float clearance = 1e9f, widest = 0.0f;
        for (int t = 0; t < 960; ++t) {
            command.moveDirection = t < 480 ? Vec3 { -1.0f, 0.0f, 0.0f } : Vec3 { -0.3f, 1.0f, 0.0f };
            scene.moveCharacter(command, settings, 1.0f / 120.0f);
            lab.step();
            const Vec3 center = scene.characterState().position;
            clearance = std::min({ clearance, insideBack(center + Vec3 { 0.0f, 0.0f, axis }), insideBack(center - Vec3 { 0.0f, 0.0f, axis }) });
            widest = std::max(widest, std::abs(center.y));
        }
        scene.destroyCharacter();
        std::printf("trave: a capsula na barreira: o eixo a %.3f m da placa do fundo (raio %.2f), |y| ate %.3f (a placa do lado %.3f - o raio)\n",
            clearance, settings.radius, widest, sideInner);
        require(clearance >= settings.radius - 0.01f, "a capsula para na placa do fundo");
        require(widest <= sideInner - settings.radius + 0.01f, "a capsula desliza no fundo e para na placa do lado");
        require(widest > sideInner - settings.radius - 0.1f, "a capsula desliza no fundo ate o canto (nao gruda)");
        // A bola passa pela barreira: o chute no meio estufa a rede como sem ela.
        NetShotSettings3D shot;
        shot.speed = 25.0f;
        shot.distance = 11.0f;
        shot.aimHeight = 0.45f;
        lab.shoot(shot);
        runUntil(lab, 6.0f, [&] { return lab.metrics().stopTime >= 0.0f; });
        std::printf("trave: chute 25 m/s com a barreira: gol %d, estufou %.2f m\n", lab.metrics().goal ? 1 : 0, lab.metrics().bulge);
        require(lab.metrics().goal && lab.metrics().bulge > 0.6f, "a bola passa pela barreira e estufa a rede");
        lab.clearBodies();
        // O boneco fisico a 9 m/s contra o lado, de dentro: os links param na
        // placa.
        RagdollSpawnDefinition3D spawn;
        spawn.pelvisPosition = { lab.objectPosition().x, sideInner - 2.0f, profile.standingRootHeightMeters + 0.1f };
        spawn.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 0.5f * 3.14159265f);
        const RagdollHandle3D doll = scene.createRagdoll(profile, spawn);
        std::vector<PhysicsBodyState3D> links = scene.ragdollState(doll).links;
        for (PhysicsBodyState3D& link : links) link.linearVelocity = { 0.0f, 9.0f, 1.0f };
        scene.resetRagdollState(doll, links);
        float farthest = 0.0f;
        for (int t = 0; t < 360; ++t) {
            lab.step();
            for (const PhysicsBodyState3D& link : scene.ragdollState(doll).links) farthest = std::max(farthest, link.position.y);
        }
        std::printf("trave: boneco fisico a 9 m/s no lado: o centro do link mais fundo a %.3f m da face de dentro\n", farthest - sideInner);
        require(farthest < sideInner + 0.05f, "o boneco fisico para na placa do lado (no maximo 5 cm dentro dela)");
    }
    // A caixa (14 kg) a 30 m/s, a caixa de 100 kg a 15 m/s, a bola a 40 m/s.
    {
        ObjectNetLab3D lab(engine, materials);
        require(lab.start(*goal, &error), "o laboratorio da trave: " + error);
        NetShotSettings3D toss;
        toss.speed = 30.0f;
        toss.distance = 6.0f;
        toss.aimHeight = 0.45f;
        lab.throwObject(*crate, toss);
        float deepest = 1e9f, strain = 0.0f;
        for (int t = 0; t < 360; ++t) {
            lab.step();
            deepest = std::min(deepest, lab.bodies().back().position.x);
            strain = std::max(strain, lab.netStatus().maximumStrain);
        }
        std::printf("trave: caixa a 30 m/s: foi ate x = %.2f, fio no maximo %.1f%%\n", deepest, strain * 100.0f);
        require(deepest > behind, "a caixa a 30 m/s nao atravessa a rede");
        lab.reset();
        PhysicsScene3D& scene = *lab.scene();
        PhysicsBodyDefinition3D heavy;
        heavy.position = { 3.0f, -0.4f, 1.0f };
        heavy.linearVelocity = { -15.0f, 0.0f, 1.5f };
        heavy.massKg = 100.0f;
        heavy.materialId = "default";
        PhysicsShape3D box;
        box.type = PhysicsShapeType3D::Box;
        box.halfExtents = { 0.3f, 0.3f, 0.3f };
        const PhysicsBodyHandle3D block = scene.createBody(heavy, std::span<const PhysicsShape3D>(&box, 1));
        deepest = 1e9f;
        strain = 0.0f;
        for (int t = 0; t < 360; ++t) {
            lab.step();
            deepest = std::min(deepest, scene.bodyState(block).position.x);
            strain = std::max(strain, lab.netStatus().maximumStrain);
        }
        std::printf("trave: caixa de 100 kg a 15 m/s: foi ate x = %.2f, fio no maximo %.1f%%\n", deepest, strain * 100.0f);
        require(deepest > behind, "a caixa de 100 kg nao atravessa a rede");
        scene.destroyBody(block);
        lab.reset();
        NetShotSettings3D shot;
        shot.speed = 40.0f;
        shot.aimHeight = 0.5f;
        lab.shoot(shot);
        deepest = 1e9f;
        for (int t = 0; t < 360; ++t) {
            lab.step();
            deepest = std::min(deepest, lab.bodies().back().position.x);
        }
        std::printf("trave: bola a 40 m/s: foi ate x = %.2f, estufou %.2f m\n", deepest, lab.metrics().bulge);
        require(deepest > behind, "a bola a 40 m/s nao atravessa a rede");
    }
    // O boneco guiado: o empurrao da rede chega a ele como empurrao de fora.
    {
        ObjectNetLab3D lab(engine, materials);
        require(lab.start(*goal, &error), "o laboratorio da trave: " + error);
        PhysicsScene3D& scene = *lab.scene();
        RagdollSpawnDefinition3D spawn;
        spawn.pelvisPosition = { -0.7f, 0.2f, profile.standingRootHeightMeters };
        spawn.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 3.14159265f);
        const RagdollHandle3D doll = scene.createRagdoll(profile, spawn);
        scene.setRagdollGuided(doll, true);
        const RagdollState3D first = scene.ragdollState(doll);
        std::vector<Vec3> base;
        std::vector<Quaternion> turns;
        for (const PhysicsBodyState3D& link : first.links) {
            base.push_back(link.position);
            turns.push_back(link.orientation);
        }
        std::vector<Vec3> pose(base.size());
        const auto drive = [&](Vec3 offset) {
            for (std::size_t i = 0; i < base.size(); ++i) pose[i] = base[i] + offset;
            scene.setRagdollGuidePose(doll, pose, turns);
            const auto begin = std::chrono::steady_clock::now();
            lab.step();
            return std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - begin).count();
        };
        // Parado dentro do gol, sem encostar (a rede dorme): nada, e o custo.
        float idlePush = 0.0f, idleCost = 0.0f;
        for (int t = 0; t < 120; ++t) {
            drive({});
            idlePush = std::max(idlePush, scene.ragdollState(doll).externalPushNewtonSeconds);
            idleCost += lab.netStatus().stepMilliseconds;
        }
        idleCost /= 120.0f;
        // Andando para o fundo a 1,5 m/s: encosta de leve, depois estica.
        float lightPush = 0.0f, tautPush = 0.0f, walkCost = 0.0f;
        int walkTicks = 0;
        for (int t = 1; t <= 150; ++t) {
            const Vec3 offset { -1.5f * static_cast<float>(t) / 120.0f, 0.0f, 0.0f };
            drive(offset);
            const float push = scene.ragdollState(doll).externalPushNewtonSeconds;
            const float pelvis = base.front().x + offset.x;
            if (pelvis > -1.45f) lightPush = std::max(lightPush, push);
            if (pelvis < -1.9f) tautPush = std::max(tautPush, push);
            walkCost += lab.netStatus().stepMilliseconds;
            ++walkTicks;
        }
        walkCost /= static_cast<float>(std::max(1, walkTicks));
        // Correndo a 9 m/s de um lado ao outro por dentro do gol, rente a
        // linha, passando pelas duas laterais (o pior caso: um corpo rapido
        // empurrando a rede sem parar, que no jogo viraria fisico).
        float runCost = 0.0f;
        for (int t = 1; t <= 120; ++t) {
            drive({ 0.4f, -4.0f + 9.0f * static_cast<float>(t) / 120.0f, 0.0f });
            runCost += lab.netStatus().stepMilliseconds;
        }
        runCost /= 120.0f;
        std::printf("trave: boneco guiado correndo a 9 m/s pelas laterais: rede %.3f ms/tick\n", runCost);
        std::printf("trave: boneco guiado: parado no gol empurrao %.3f N s (rede %.4f ms/tick); encostando de leve %.2f N s; "
                    "contra a rede esticada %.1f N s (rede %.3f ms/tick)\n",
            idlePush, idleCost, lightPush, tautPush, walkCost);
        require(idlePush == 0.0f, "parado sem encostar, a rede nao empurra o boneco");
        require(tautPush > 5.0f && tautPush > 4.0f * lightPush, "a rede esticada empurra o boneco guiado (o atleta reage)");
        // (Dormindo, o boneco guiado parado dentro do gol, sem encostar, nao
        // conta: era 0,06 ms por tick, a lista e os subpassos para nada.)
        require(idleCost < 0.03f, "o boneco parado dentro do gol custa pouco a rede");
    }
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
        testGoalNet(object, engine, materials);
        testGoalNetSkin();
        testGoalNetStrength(engine, materials);
    } catch (const std::exception& exception) {
        std::cerr << "MatterObjectPhysics test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterObjectPhysics tests passed (" << checks << " checagens)\n";
    return 0;
}
