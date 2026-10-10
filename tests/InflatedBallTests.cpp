// Bola inflada (a bola de futebol): o modelo e a bola na cena do Jolt, contra
// os numeros oficiais (FIFA Quality Pro) e medidas publicadas.
#include "Engine/Character/BallOwnership3D.hpp"
#include "Engine/Geometry/ProceduralSoccerBall3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

constexpr float Dt = 1.0f / 120.0f;

// (XBOLARAIO: a bola de outro tamanho com o mesmo jeito - o tamanho do
// laboratorio, InflatedBallModel3D::scaledProfile.)
InflatedBallProfile3D testProfile() {
    const InflatedBallProfile3D base;
    if (const char* r = std::getenv("XBOLARAIO")) return InflatedBallModel3D::scaledProfile(base, std::strtof(r, nullptr));
    return base;
}

struct Bench {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    explicit Bench(const char* floorMaterial = "steel") {
        // Calibracao: XB_<material>_Z / _R0 / _R1 trocam a resposta da
        // superficie a bola (amortecimento normal, braco de rolagem, braco
        // por m/s).
        for (const char* id : { "concrete", "steel", "wood", "grass", "soil", "sand", "rubber", "default" }) {
            const SurfaceMaterial* found = materials.find(id);
            if (!found) continue;
            SurfaceMaterial material = *found;
            bool changed = false;
            const auto take = [&](const char* suffix, float& field) {
                const std::string key = std::string("XB_") + id + suffix;
                if (const char* v = std::getenv(key.c_str())) { field = std::strtof(v, nullptr); changed = true; }
            };
            take("_Z", material.ballSurface.normalDampingRatio);
            take("_R0", material.ballSurface.rollingResistanceMeters);
            take("_R1", material.ballSurface.rollingResistancePerSpeed);
            if (changed) materials.registerMaterial(material);
        }
        scene = engine.createScene({}, materials);
        PhysicsBodyDefinition3D floor;
        floor.motionType = PhysicsMotionType3D::Static;
        floor.position = { 0, 0, -0.5f };
        floor.materialId = floorMaterial;
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 60, 60, 0.5f };
        shape.materialId = floorMaterial;
        require(static_cast<bool>(scene->createBody(floor, std::span<const PhysicsShape3D>(&shape, 1))), "Sem piso");
    }
    InflatedBallHandle3D ball(Vec3 position, Vec3 velocity = {}, Vec3 spin = {}) {
        InflatedBallDefinition3D definition;
        definition.profile = testProfile();
        definition.position = position;
        definition.linearVelocity = velocity;
        definition.angularVelocity = spin;
        return scene->createInflatedBall(definition);
    }
    PhysicsBodyHandle3D box(Vec3 position, Vec3 half, float mass, Vec3 velocity, bool dynamic, const char* material,
        Quaternion orientation = {}) {
        PhysicsBodyDefinition3D body;
        body.motionType = dynamic ? PhysicsMotionType3D::Dynamic : PhysicsMotionType3D::Static;
        body.position = position;
        body.orientation = orientation;
        body.linearVelocity = velocity;
        body.massKg = mass;
        body.materialId = material;
        body.linearDamping = 0.0f;
        body.angularDamping = 0.0f;
        body.collisionMode = PhysicsCollisionMode3D::Continuous;
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = half;
        shape.materialId = material;
        return scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
    }
};

float radius() { return testProfile().radiusMeters; }

void modelChecks() {
    const InflatedBallProfile3D profile;
    const float v0 = InflatedBallModel3D::restVolume(profile);
    require(std::abs(v0 - 0.005545f) < 0.0001f, "Volume de repouso fora (raio 10,98 cm)");
    require(std::abs(InflatedBallModel3D::gaugePressure(profile, v0) - 80000.0f) < 1.0f, "Pressao de repouso nao e 0,8 bar");
    require(InflatedBallModel3D::gaugePressure(profile, 0.9f * v0) > 80000.0f, "Comprimida, a pressao nao subiu");
    require(InflatedBallModel3D::normalForce(profile, 0.0f, 5.0f, v0, 0.0f) == 0.0f, "Forca sem compressao");
    // ~1 cm de mancha: pressao x area ~ 0,5 kN (rigidez da bola inflada).
    const float f = InflatedBallModel3D::normalForce(profile, 0.01f, 0.0f, v0 - InflatedBallModel3D::capVolume(profile, 0.01f), 0.0f);
    std::printf("MODELO forca a 1 cm %.0f N\n", f);
    require(f > 450.0f && f < 650.0f, "Rigidez da bola fora (~0,5 kN a 1 cm)");
}

// FIFA Quality Pro: solta de 2,00 m (da base) sobre aco, quica 135-155 cm.
void dropTest() {
    Bench bench("steel");
    const float r = radius();
    auto ball = bench.ball({ 0, 0, 2.0f + r });
    float apex = -1.0f, contactStart = -1.0f, contactEnd = -1.0f;
    bool contact = false, rising = false;
    for (int i = 0; i < 480; ++i) {
        bench.scene->simulate(Dt);
        const auto state = bench.scene->inflatedBallState(ball);
        const float t = (i + 1) * Dt;
        const bool now = state.contactCount > 0;
        if (now && !contact && contactStart < 0) contactStart = t;
        if (!now && contact && contactEnd < 0) contactEnd = t;
        contact = now;
        if (contactEnd > 0 && state.linearVelocity.z > 0) rising = true;
        if (rising && apex < 0 && state.linearVelocity.z <= 0) apex = state.position.z - r;
    }
    const auto state = bench.scene->inflatedBallState(ball);
    std::printf("QUEDA 2 m -> quique %.3f m (FIFA Pro 1,35-1,55) | compressao max %.1f mm\n", apex, state.peakCompressionMeters * 1000);
    require(apex > 1.35f && apex < 1.55f, "Quique fora da faixa FIFA Quality Pro");
    require(state.peakCompressionMeters > 0.008f && state.peakCompressionMeters < 0.03f, "Compressao da queda fora (1-3 cm)");
}

// Calibracao (MATTERENGINE_TEST_FILTER=calibrar): mede, sem reprovar, o que o
// estudo de calibracao pede (brain/30_Projects/matter_engine/
// ESTUDO_CALIBRACAO_FISICA_BOLA_FUTEBOL_SOCCER_FALL.md).
struct DropResult {
    std::vector<float> apex;       // alturas dos quiques (base da bola, m)
    float contactMs = 0.0f;        // primeiro contato
    float peakCompressionMm = 0.0f;
    float peakForce = 0.0f;
    float restSeconds = -1.0f;     // ate parar de quicar (sem sair do chao)
    int bounces = 0;
};

DropResult dropOn(const char* material, float heightBottom, const InflatedBallProfile3D& profile,
    float dt = 0.001f, float seconds = 8.0f) {
    Bench bench(material);
    const float r = profile.radiusMeters;
    InflatedBallDefinition3D definition;
    definition.profile = profile;
    definition.position = { 0, 0, heightBottom + r };
    auto ball = bench.scene->createInflatedBall(definition);
    DropResult out;
    bool contact = false, rising = false, firstDone = false;
    float contactStart = -1.0f, top = 0.0f, lastAir = 0.0f;
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        bench.scene->simulate(dt);
        const auto state = bench.scene->inflatedBallState(ball);
        const float t = (i + 1) * dt;
        const bool now = state.contactCount > 0;
        if (now) {
            for (std::uint32_t c = 0; c < state.contactCount; ++c)
                if (!firstDone) out.peakForce = std::max(out.peakForce, state.contacts[c].normalForceNewtons);
            if (!firstDone) out.peakCompressionMm = std::max(out.peakCompressionMm, state.compressionMeters * 1000.0f);
        }
        if (now && !contact) {
            if (contactStart < 0.0f) contactStart = t;
            if (rising && top > 0.0005f) { out.apex.push_back(top); ++out.bounces; }
            rising = false;
            top = 0.0f;
        }
        if (!now && contact) {
            if (!firstDone) { out.contactMs = (t - contactStart) * 1000.0f; firstDone = true; }
            rising = true;
            lastAir = t;
        }
        if (!now) top = std::max(top, state.position.z - r);
        contact = now;
    }
    if (rising && top > 0.0005f) out.apex.push_back(top);
    out.restSeconds = lastAir;
    return out;
}

float rollOn(const char* material, float speed, const InflatedBallProfile3D& profile, float* seconds = nullptr) {
    Bench bench(material);
    const float r = profile.radiusMeters;
    InflatedBallDefinition3D definition;
    definition.profile = profile;
    // Em repouso apoiada: assenta antes de rolar.
    definition.position = { 0, 0, r - 0.004f };
    auto ball = bench.scene->createInflatedBall(definition);
    for (int i = 0; i < 120; ++i) bench.scene->simulate(Dt);
    auto start = bench.scene->inflatedBallState(ball);
    bench.scene->setInflatedBallMotion(ball, start.position, { speed, 0, 0 }, { 0, speed / r, 0 });
    const float x0 = start.position.x;
    float t = 0.0f;
    for (int i = 0; i < 120 * 60; ++i) {
        bench.scene->simulate(Dt);
        t += Dt;
        const auto state = bench.scene->inflatedBallState(ball);
        if (std::hypot(state.linearVelocity.x, state.linearVelocity.y) < 0.02f && t > 0.2f) break;
    }
    if (seconds) *seconds = t;
    return bench.scene->inflatedBallState(ball).position.x - x0;
}

struct ImpactResult { float contactMs = 0, peakForce = 0, peakCompressionMm = 0, exitSpeed = 0; };

ImpactResult wallImpact(float speed, const InflatedBallProfile3D& profile) {
    Bench bench("concrete");
    bench.box({ 1.0f, 0.0f, 1.5f }, { 0.5f, 2.0f, 1.5f }, 0.0f, {}, false, "concrete");
    const float r = profile.radiusMeters;
    InflatedBallDefinition3D definition;
    definition.profile = profile;
    definition.position = { 0.5f - r - 0.01f, 0, 1.5f };
    definition.linearVelocity = { speed, 0, 0 };
    auto ball = bench.scene->createInflatedBall(definition);
    ImpactResult out;
    const float dt = 0.0002f;
    bool contact = false, done = false;
    float start = -1.0f;
    for (int i = 0; i < 2000 && !done; ++i) {
        bench.scene->simulate(dt);
        const auto state = bench.scene->inflatedBallState(ball);
        const bool now = state.contactCount > 0;
        if (now && !contact) start = (i + 1) * dt;
        if (now) {
            out.peakCompressionMm = std::max(out.peakCompressionMm, state.compressionMeters * 1000.0f);
            for (std::uint32_t c = 0; c < state.contactCount; ++c)
                out.peakForce = std::max(out.peakForce, state.contacts[c].normalForceNewtons);
        }
        if (!now && contact) {
            out.contactMs = ((i + 1) * dt - start) * 1000.0f;
            out.exitSpeed = -state.linearVelocity.x;
            done = true;
        }
        contact = now;
    }
    return out;
}

// Parada no chao: quieta, sem tremer nem afundar.
void restTest() {
    Bench bench("grass");
    const float r = radius();
    auto ball = bench.ball({ 0, 0, r + 0.002f });
    float worstSpeed = 0.0f;
    for (int i = 0; i < 600; ++i) {
        bench.scene->simulate(Dt);
        if (i > 240) worstSpeed = std::max(worstSpeed, bench.scene->inflatedBallState(ball).linearVelocity.length());
    }
    const auto state = bench.scene->inflatedBallState(ball);
    std::printf("PARADA altura %.4f m velocidade max %.5f m/s compressao %.3f mm\n", state.position.z, worstSpeed, state.compressionMeters * 1000);
    require(worstSpeed < 0.002f, "A bola parada treme");
    require(state.compressionMeters < 0.0005f && state.position.z > r - 0.001f, "A bola parada afunda");
}

// Pe rigido de 1,2 kg a 20 m/s na bola no ar: a bola sai ~1,2-1,4x o pe
// (chutes de peito do pe medidos) e o momento se conserva.
void kickTest() {
    Bench bench("grass");
    const float r = radius();
    auto ball = bench.ball({ 0, 0, 1.5f });
    // Massa efetiva de quem bate: pe e canela (~2,5 kg num chute de peito do
    // pe; so o pe, 1,2 kg, e leve demais para a perna travada). O pe cede
    // (restituicao pe-bola ~0,5) e a bola sai a 1,2-1,35x o pe (medido em
    // chutes).
    constexpr float FootMass = 2.5f;
    auto foot = bench.box({ -0.30f, 0, 1.5f }, { 0.05f, 0.045f, 0.035f }, FootMass, { 20, 0, 0 }, true, "shoe_sole");
    // So o choque: o pe e a bola, sem gravidade no pe (o tempo e curto).
    float ballSpeed = 0.0f;
    for (int i = 0; i < 6; ++i) bench.scene->simulate(Dt);
    const auto state = bench.scene->inflatedBallState(ball);
    const auto footState = bench.scene->bodyState(foot);
    ballSpeed = state.linearVelocity.x;
    const float momentumBefore = FootMass * 20.0f;
    const float momentumAfter = FootMass * footState.linearVelocity.x + testProfile().massKg * state.linearVelocity.x;
    std::printf("CHUTE pe 20 m/s 2,5 kg -> bola %.2f m/s (razao %.2f) pe %.2f | momento %.2f -> %.2f | compressao max %.1f mm\n",
        ballSpeed, ballSpeed / 20.0f, footState.linearVelocity.x, momentumBefore, momentumAfter, state.peakCompressionMeters * 1000);
    require(ballSpeed / 20.0f > 1.15f && ballSpeed / 20.0f < 1.45f, "Velocidade de saida do chute fora (1,15-1,45x o pe)");
    require(std::abs(momentumAfter - momentumBefore) < 0.08f * momentumBefore, "O chute nao conservou o momento");
    (void)r;
}

// Contato fora do centro da giro (sem comando especial).
void spinKickTest() {
    Bench bench("grass");
    auto ball = bench.ball({ 0, 0, 1.5f });
    bench.box({ -0.30f, 0.07f, 1.5f }, { 0.05f, 0.045f, 0.035f }, 1.2f, { 20, 0, 0 }, true, "shoe_sole");
    for (int i = 0; i < 6; ++i) bench.scene->simulate(Dt);
    const auto state = bench.scene->inflatedBallState(ball);
    const float rps = state.angularVelocity.length() / 6.2832f;
    std::printf("CHUTE DE LADO -> bola %.2f m/s, giro %.1f rot/s eixo z %.2f, saida lateral %.2f m/s\n",
        state.linearVelocity.length(), rps, state.angularVelocity.z / std::max(1e-3f, state.angularVelocity.length()), state.linearVelocity.y);
    require(rps > 1.5f, "Contato fora do centro nao deu giro");
    require(std::abs(state.angularVelocity.z) > 0.7f * state.angularVelocity.length(), "O giro do chute de lado nao e em torno da vertical");
}

// Trave (cilindro estatico de 6 cm) a 30 m/s: sem atravessar.
void postTest() {
    Bench bench("grass");
    PhysicsBodyDefinition3D post;
    post.motionType = PhysicsMotionType3D::Static;
    post.position = { 1.0f, 0, 1.2f };
    // A capsula e ao longo de Y: de pe, girada 90 graus em X.
    post.orientation = Quaternion::fromAxisAngle({ 1, 0, 0 }, 1.5707963f);
    post.materialId = "steel";
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Capsule;
    shape.radius = 0.06f;
    shape.capsuleHalfHeight = 1.1f;
    shape.materialId = "steel";
    bench.scene->createBody(post, std::span<const PhysicsShape3D>(&shape, 1));
    auto ball = bench.ball({ -0.5f, 0, 1.2f }, { 30, 0, 0 });
    for (int i = 0; i < 24; ++i) bench.scene->simulate(Dt);
    const auto state = bench.scene->inflatedBallState(ball);
    std::printf("TRAVE 30 m/s -> x %.2f v %.2f m/s compressao max %.1f mm\n", state.position.x, state.linearVelocity.x, state.peakCompressionMeters * 1000);
    require(state.position.x < 1.0f && state.linearVelocity.x < -10.0f, "A bola atravessou a trave ou nao voltou");
}

// Rolando: sem deslizar (v = w R) e desacelerando pelo ar e pelo gramado.
void rollTest() {
    Bench bench("grass");
    const float r = radius();
    auto ball = bench.ball({ 0, 0, r }, { 5, 0, 0 });
    for (int i = 0; i < 240; ++i) bench.scene->simulate(Dt);
    const auto state = bench.scene->inflatedBallState(ball);
    const float rolling = state.angularVelocity.y * r;
    std::printf("ROLANDO 5 m/s -> em 2 s v %.2f m/s, giro*R %.2f, altura %.4f\n", state.linearVelocity.x, rolling, state.position.z);
    require(std::abs(rolling - state.linearVelocity.x) < 0.1f, "A bola nao passou a rolar sem deslizar");
    require(state.linearVelocity.x > 0.5f && state.linearVelocity.x < 4.8f, "Rolando, desacelerou de menos ou demais");
}

// Rampa da FIFA (bola solta de 1 m numa rampa, ~3,43 m/s rolando na saida):
// gramado natural excelente, 4-10 m ate parar.
void fifaRollTest() {
    const InflatedBallProfile3D profile;
    float seconds = 0.0f;
    const float grass = rollOn("grass", 3.43f, profile, &seconds);
    std::printf("RAMPA FIFA grama %.1f m em %.1f s (FIFA 4-10 m)\n", grass, seconds);
    require(grass > 4.0f && grass < 10.0f, "Rolagem na grama fora da faixa da FIFA (4-10 m)");
}

// Queda de 2 m na grama: a mesma bola que quica ~1,4 m no concreto quica
// 0,60-1,00 m num gramado excelente (FIFA); parada perto do chao, poucos quiques.
void surfaceDropTest() {
    const InflatedBallProfile3D profile;
    const DropResult grass = dropOn("grass", 2.0f, profile);
    const DropResult low = dropOn("concrete", 0.10f, profile);
    std::printf("GRAMA 2 m -> %.3f m | concreto de 10 cm: %d quiques, quieta em %.2f s\n",
        grass.apex.empty() ? 0.0f : grass.apex[0], low.bounces, low.restSeconds);
    require(!grass.apex.empty() && grass.apex[0] > 0.6f && grass.apex[0] < 1.0f, "Quique na grama fora da faixa da FIFA (0,6-1,0 m)");
    require(low.bounces <= 6 && low.restSeconds < 1.2f, "Quiques pequenos demais (de 10 cm a bola nao para)");
}

// Efeito no ar: 26 m/s com 8 rot/s em torno da vertical, 16 graus para cima;
// em 20 m desvia alguns metros (Asai et al. 2007: ~4 m/s de deriva lateral).
void curveTest() {
    Bench bench("grass");
    const float r = radius();
    const float elevation = 16.0f * 3.14159265f / 180.0f;
    auto ball = bench.ball({ 0, 0, r + 0.01f }, { 26.0f * std::cos(elevation), 0, 26.0f * std::sin(elevation) }, { 0, 0, 8.0f * 6.2832f });
    auto straight = bench.ball({ 0, 3.0f, r + 0.01f }, { 26.0f * std::cos(elevation), 0, 26.0f * std::sin(elevation) });
    InflatedBallState3D state, other;
    for (int i = 0; i < 360; ++i) {
        bench.scene->simulate(Dt);
        state = bench.scene->inflatedBallState(ball);
        if (state.position.x >= 20.0f) break;
    }
    other = bench.scene->inflatedBallState(straight);
    std::printf("CURVA 26 m/s 8 rot/s -> em x %.1f m lateral %.2f m (sem giro %.2f) v %.2f m/s\n",
        state.position.x, state.position.y, other.position.y - 3.0f, state.linearVelocity.length());
    require(state.position.y > 1.5f && state.position.y < 5.0f, "O efeito no ar fora do esperado");
    require(std::abs(other.position.y - 3.0f) < 0.05f, "A bola sem giro desviou");
    require(state.linearVelocity.length() > 15.0f && state.linearVelocity.length() < 24.0f, "Arrasto fora (26 m/s em 20 m)");
}


// Bola contra bola no ar, 5 m/s cada uma: as duas cascas achatam, o momento
// se conserva e elas voltam com a restituicao de uma bola cheia.
void ballPairTest() {
    Bench bench("grass");
    auto left = bench.ball({ -0.4f, 0, 2.0f }, { 5, 0, 0 });
    auto right = bench.ball({ 0.4f, 0, 2.0f }, { -5, 0, 0 });
    for (int i = 0; i < 24; ++i) bench.scene->simulate(Dt);
    const auto a = bench.scene->inflatedBallState(left);
    const auto b = bench.scene->inflatedBallState(right);
    const float restitution = (b.linearVelocity.x - a.linearVelocity.x) / 10.0f;
    std::printf("BOLA x BOLA 5+5 m/s -> %.2f e %.2f m/s (restituicao %.2f) | momento %.3f | compressao max %.1f mm\n",
        a.linearVelocity.x, b.linearVelocity.x, restitution, 0.43f * (a.linearVelocity.x + b.linearVelocity.x),
        a.peakCompressionMeters * 1000);
    require(a.linearVelocity.x < 0.0f && b.linearVelocity.x > 0.0f, "As bolas se atravessaram");
    require(restitution > 0.6f && restitution < 0.95f, "Restituicao bola-bola fora");
    require(std::abs(a.linearVelocity.x + b.linearVelocity.x) < 0.1f, "Bola-bola nao conservou o momento");
    require(a.peakCompressionMeters > 0.002f && std::abs(a.peakCompressionMeters - b.peakCompressionMeters) < 1e-4f,
        "As duas cascas nao achataram igual");
}

// Malha procedural: 32 gomos ligados a gaiola; sem contato o skinning
// reproduz a bola rigida e, achatada contra o chao, nada passa do plano.
void meshTest() {
    const SoccerBallMesh3D ball = buildProceduralSoccerBall3D();
    const auto& vertices = ball.mesh.vertices;
    require(vertices.size() > 10000 && ball.mesh.indices.size() % 3 == 0 && ball.lods.size() == 2, "Malha da bola incompleta");
    require(ball.lods[1].size() < ball.lods[0].size() && ball.lods[0].size() < ball.mesh.indices.size(), "Niveis de detalhe fora de ordem");
    for (const auto& vertex : vertices) {
        const float sum = vertex.weights[0] + vertex.weights[1] + vertex.weights[2] + vertex.weights[3];
        require(std::abs(sum - 1.0f) < 1e-4f, "Pesos da gaiola nao somam 1");
        for (const auto joint : vertex.joints) require(joint < ball.cageDirections.size(), "Osso fora da gaiola");
    }
    const auto skinned = [&](const std::vector<Mat4>& palette, const MeshVertex3D& vertex) {
        Vec3 p {};
        for (std::size_t k = 0; k < 4; ++k) {
            const Mat4& m = palette[vertex.joints[k]];
            const Vec3 v = vertex.position;
            p += Vec3 { m.at(0, 0) * v.x + m.at(0, 1) * v.y + m.at(0, 2) * v.z + m.at(0, 3),
                m.at(1, 0) * v.x + m.at(1, 1) * v.y + m.at(1, 2) * v.z + m.at(1, 3),
                m.at(2, 0) * v.x + m.at(2, 1) * v.y + m.at(2, 2) * v.z + m.at(2, 3) } * vertex.weights[k];
        }
        return p;
    };
    const float r = ball.radiusMeters;
    const Quaternion orientation = Quaternion::fromAxisAngle({ 0.3f, 1.0f, 0.2f }, 0.7f);
    std::vector<Mat4> palette;
    buildSoccerBallSkin3D(ball.cageDirections, r, { 1, 2, 3 }, orientation, {}, palette);
    float rigidError = 0.0f;
    for (const auto& vertex : vertices) {
        rigidError = std::max(rigidError, (skinned(palette, vertex) - (Vec3 { 1, 2, 3 } + orientation.rotate(vertex.position))).length());
    }
    // Achatada 16 mm (a queda da FIFA) contra o chao z = 0.
    const float compression = 0.016f;
    const SoccerBallContactPlane3D floor { { 0, 0, 0 }, { 0, 0, 1 } };
    buildSoccerBallSkin3D(ball.cageDirections, r, { 0, 0, r - compression }, orientation,
        std::span<const SoccerBallContactPlane3D>(&floor, 1), palette);
    float lowest = 1.0f, highest = -1.0f, widest = 0.0f;
    for (const auto& vertex : vertices) {
        const Vec3 p = skinned(palette, vertex);
        lowest = std::min(lowest, p.z);
        highest = std::max(highest, p.z);
        if (p.z < 0.0005f) widest = std::max(widest, std::sqrt(p.x * p.x + p.y * p.y));
    }
    const float patch = std::sqrt(2.0f * r * compression - compression * compression);
    std::printf("MALHA %zu vertices %zu triangulos (LOD %zu/%zu) gaiola %zu | rigida erro %.4f mm | achatada 16 mm: "
                "fundo %.2f mm topo %.1f mm mancha %.1f mm (esfera %.1f)\n",
        vertices.size(), ball.mesh.indices.size() / 3, ball.lods[0].size() / 3, ball.lods[1].size() / 3,
        ball.cageDirections.size(), rigidError * 1000, lowest * 1000, highest * 1000, widest * 1000, patch * 1000);
    require(rigidError < 1e-5f, "Sem contato, o skinning nao reproduz a bola rigida");
    require(lowest > -0.0004f && lowest < 0.0005f, "A bola achatada passa do chao ou nao encosta");
    require(std::abs(highest - (2.0f * r - compression)) < 0.001f, "O topo da bola achatada saiu do lugar");
    require(widest > 0.8f * patch && widest < 1.25f * patch, "Mancha achatada fora do tamanho da calota");
}

void calibration() {
    InflatedBallProfile3D profile;
    const auto take = [](const char* key, float& field) {
        if (const char* v = std::getenv(key)) field = std::strtof(v, nullptr);
    };
    take("XB_PRESSURE", profile.gaugePressurePascals);
    take("XB_ZETA", profile.dampingRatio);
    take("XB_ZLOW", profile.lowSpeedDampingRatio);
    take("XB_VLOW", profile.lowSpeedDampingSpeed);
    take("XB_ROLL", profile.rollingResistanceMeters);
    // Rigidez quase estatica (N/mm) a 1, 2 e 3 cm.
    {
        const float v0 = InflatedBallModel3D::restVolume(profile);
        std::printf("RIGIDEZ");
        for (const float d : { 0.01f, 0.02f, 0.03f }) {
            const float f = InflatedBallModel3D::normalForce(profile, d, 0.0f,
                v0 - InflatedBallModel3D::capVolume(profile, d), 0.0f);
            std::printf(" | %.0f mm: %.0f N (%.1f N/mm)", d * 1000, f, f / (d * 1000));
        }
        std::printf("\n");
    }
    for (const char* material : { "steel", "concrete", "wood", "grass", "soil", "sand", "rubber", "shoe_sole", "body" }) {
        const DropResult r = dropOn(material, 2.0f, profile);
        std::printf("QUEDA 2 m %-9s quiques", material);
        for (std::size_t i = 0; i < std::min<std::size_t>(r.apex.size(), 7); ++i) std::printf(" %.3f", r.apex[i]);
        std::printf(" (%d) | contato %.1f ms compressao %.1f mm forca %.0f N | quieta em %.2f s\n",
            r.bounces, r.contactMs, r.peakCompressionMm, r.peakForce, r.restSeconds);
    }
    for (const float hz : { 60.0f, 120.0f, 240.0f }) {
        const DropResult r = dropOn("concrete", 2.0f, profile, 1.0f / hz);
        std::printf("PASSO %.0f Hz: quiques %.3f %.3f %.3f | quieta em %.2f s\n", hz,
            r.apex.size() > 0 ? r.apex[0] : 0.0f, r.apex.size() > 1 ? r.apex[1] : 0.0f,
            r.apex.size() > 2 ? r.apex[2] : 0.0f, r.restSeconds);
    }
    for (const float h : { 0.10f, 0.05f, 0.02f }) {
        const DropResult r = dropOn("concrete", h, profile);
        std::printf("REPOUSO concreto de %.0f cm: %d quiques, quieta em %.2f s\n", h * 100, r.bounces, r.restSeconds);
    }
    for (const char* material : { "concrete", "wood", "grass", "soil" }) {
        float seconds = 0.0f;
        const float d = rollOn(material, 3.43f, profile, &seconds);
        std::printf("RAMPA FIFA (3,43 m/s rolando) %-9s %.1f m em %.1f s\n", material, d, seconds);
    }
    for (const float v : { 1.0f, 2.0f, 5.0f, 10.0f }) {
        float s1 = 0, s2 = 0;
        const float a = rollOn("concrete", v, profile, &s1);
        const float b = rollOn("grass", v, profile, &s2);
        std::printf("ROLAR %.0f m/s: concreto %.1f m (%.1f s) grama %.1f m (%.1f s)\n", v, a, s1, b, s2);
    }
    for (const float psi : { 8.5f, 10.0f, 11.6f, 14.0f, 15.6f }) {
        InflatedBallProfile3D p = profile;
        p.gaugePressurePascals = psi * 6894.76f;
        const DropResult r = dropOn("concrete", 2.0f, p);
        const float v0 = InflatedBallModel3D::restVolume(p);
        const float k = InflatedBallModel3D::normalForce(p, 0.01f, 0.0f, v0 - InflatedBallModel3D::capVolume(p, 0.01f), 0.0f) / 10.0f;
        std::printf("PRESSAO %.1f psi: quique %.3f m | contato %.1f ms compressao %.1f mm | rigidez 1 cm %.1f N/mm\n",
            psi, r.apex.empty() ? 0.0f : r.apex[0], r.contactMs, r.peakCompressionMm, k);
    }
    for (const float v : { 6.26f, 9.0f, 14.0f, 20.0f, 30.0f }) {
        const ImpactResult r = wallImpact(v, profile);
        std::printf("IMPACTO PAREDE %.1f m/s: contato %.1f ms forca %.0f N compressao %.1f mm saida %.2f m/s (e %.3f)\n",
            v, r.contactMs, r.peakForce, r.peakCompressionMm, r.exitSpeed, r.exitSpeed / v);
    }
}

} // namespace

// Dono da bola (BallOwnership3D): interagir faz o dono; outro boneco tocando
// a bola o troca; o dono longe por 3 s (ou sumido) a solta - longe por
// menos que isso, nao; o toque do proprio dono nao muda nada.
void ownershipTest() {
    BallOwnership3D ownership;
    InflatedBallState3D ball;
    ball.position = { 0.0f, 0.0f, 0.11f };
    const Vec3 near { 0.5f, 0.0f, 0.95f };
    ownership.update(ball, nullptr, 0.1f);
    require(ownership.owner() == 0, "Sem ninguem, a bola ja tinha dono");
    ownership.interact(11);
    require(ownership.owner() == 11, "Interagindo, o boneco nao virou o dono");
    ownership.update(ball, &near, 0.1f);
    require(ownership.owner() == 11, "Perto e sem toque, o dono mudou");
    ball.contactCount = 2;
    ball.contacts[0].otherRagdoll = true;
    ball.contacts[0].otherEntityId = 11;
    ball.contacts[0].normalForceNewtons = 900.0f;
    ball.contacts[1].otherRagdoll = true;
    ball.contacts[1].otherEntityId = 22;
    ball.contacts[1].normalForceNewtons = 40.0f;
    ownership.update(ball, &near, 0.1f);
    require(ownership.owner() == 22, "Outro boneco tocou a bola e o dono nao mudou");
    ball.contactCount = 0;
    ownership.interact(33);
    require(ownership.owner() == 33, "Outro boneco interagiu e o dono nao mudou");
    const Vec3 far { 5.5f, 0.0f, 0.95f };
    for (int tick = 0; tick < 25; ++tick) ownership.update(ball, &far, 0.1f);
    require(ownership.owner() == 33, "Longe por 2,5 s (menos que 3) o dono ja soltou a bola");
    ownership.update(ball, &near, 0.1f);
    for (int tick = 0; tick < 25; ++tick) ownership.update(ball, &far, 0.1f);
    require(ownership.owner() == 33, "Voltou perto e o tempo longe nao recomecou");
    for (int tick = 0; tick < 6; ++tick) ownership.update(ball, &far, 0.1f);
    require(ownership.owner() == 0, "Longe por 3,1 s o dono nao soltou a bola");
    ownership.interact(33);
    ownership.update(ball, nullptr, 0.1f);
    require(ownership.owner() == 0, "O dono sumiu e a bola seguiu com ele");
}

int main() {
    try {
        const char* only = std::getenv("MATTERENGINE_TEST_FILTER");
        const auto run = [&](const char* name, const std::function<void()>& test) {
            if (only && std::string(only) != name) return;
            test();
        };
        if (only && std::string(only) == "calibrar") {
            calibration();
            return 0;
        }
        run("modelo", modelChecks);
        run("queda", dropTest);
        run("parada", restTest);
        run("chute", kickTest);
        run("giro", spinKickTest);
        run("trave", postTest);
        run("rolar", rollTest);
        run("rampa", fifaRollTest);
        run("superficie", surfaceDropTest);
        run("curva", curveTest);
        run("bolas", ballPairTest);
        run("malha", meshTest);
        run("dono", ownershipTest);
        std::printf("Bola inflada: ok\n");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Bola inflada: %s\n", e.what());
        return 1;
    }
}
