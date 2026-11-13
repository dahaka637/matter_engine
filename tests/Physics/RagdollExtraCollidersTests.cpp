// Os colisores a mais de um link (11/11, a mao: a palma no collider e os dedos em "extraColliders", presos aos ossos dos dedos em
// "posableBones"): o perfil le, a validacao confere cada um, a variacao de corpo escala junto, o corpo fisico do Jolt e o composto (um
// raio acerta o extra longe do principal; sem extras, nada muda), os dedos ficam onde o perfil diz e acompanham a pose dos ossos
// (setRagdollBoneRotations), e seis quedas soltas nao explodem nem enterram a mao.
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Math/Ray3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/PhysicsTypes3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Physics/RagdollProfileBody3D.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <memory>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace MatterEngine;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::size_t linkOf(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    throw std::runtime_error("sem o link " + id);
}

// (Um caminho no argumento troca o perfil: o candidato de tools/fisica/colisores_mao.py antes de ir para os assets.)
std::string AssetProfile = std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json";

// O perfil do jogador com um extra conico e uma casca no LeftHand (escrito num arquivo temporario).
RagdollProfile3D profileWithExtras(const RagdollProfile3D& plain) {
    // O perfil que ja tem os extras da mao (o do jogador depois de tools/fisica/colisores_mao.py) e testado como esta.
    for (const RagdollLinkDefinition3D& link : plain.links)
        if (link.id == "LeftHand" && !link.extraColliders.empty()) return plain;
    std::ifstream in(AssetProfile);
    std::stringstream text;
    text << in.rdbuf();
    std::string json = text.str();
    const std::string key = "\"id\": \"LeftHand\",";
    const std::size_t at = json.find(key);
    require(at != std::string::npos && json.find(key, at + 1) == std::string::npos, "um LeftHand no perfil");
    json.insert(at, "\"extraColliders\": [ { \"capsule\": { \"position\": [0.0, 0.30, 0.0], \"orientation\": [0, 0, 0, 1],"
                        " \"length\": 0.10, \"radius\": 0.02, \"radiusAtPositiveX\": 0.012, \"material\": \"body\" } },"
                        " { \"hull\": { \"position\": [0.0, 0.0, 0.0], \"orientation\": [0, 0, 0, 1], \"points\": [[0.30, 0, 0],"
                        " [0.34, 0, 0], [0.30, 0.04, 0], [0.30, 0, 0.04]], \"faces\": [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],"
                        " \"convexRadius\": 0.002 } } ],\n   ");
    const std::filesystem::path path = std::filesystem::temp_directory_path() / "matter_extra_colliders_test.ragdoll.json";
    std::ofstream(path) << json;
    RagdollProfile3D profile = loadRagdollProfile3D(path.string());
    std::filesystem::remove(path);
    return profile;
}

// O ponto mais baixo (z) dos colisores de um link posto no mundo: os pontos da casca, os cantos da caixa, as tampas da capsula.
float lowestColliderZ(const RagdollLinkDefinition3D& link, const PhysicsBodyState3D& state) {
    float lowest = 1e9f;
    const auto add = [&](const RagdollCapsuleDefinition3D& c) {
        const Quaternion frame = state.orientation * c.localOrientation;
        const Vec3 center = state.position + state.orientation.rotate(c.localPosition);
        if (c.shape == RagdollColliderShape3D::Capsule) {
            const float half = ragdollCapsuleHalfSegment3D(c);
            lowest = std::min({ lowest, (center + frame.rotate({ -half, 0.0f, 0.0f })).z - c.radiusMeters,
                (center + frame.rotate({ half, 0.0f, 0.0f })).z - ragdollCapsuleRadiusAtPositiveX3D(c) });
        } else {
            for (const Vec3& corner : ragdollColliderCorners3D(c)) lowest = std::min(lowest, (center + frame.rotate(corner)).z);
        }
    };
    add(link.collider);
    const std::vector<RagdollBoneFrame3D> frames = ragdollPosableBoneFrames3D(link);
    for (std::size_t k = 0; k < link.extraColliders.size(); ++k) add(ragdollExtraColliderInLink3D(link, k, frames));
    return lowest;
}

struct Scene {
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    std::unique_ptr<PhysicsScene3D> scene;
    RagdollHandle3D ragdoll;
    RagdollSpawnDefinition3D spawn;
};

void makeScene(Scene& world, const RagdollProfile3D& profile, bool gravity, float height = 1.0f, float yaw = 0.0f) {
    PhysicsSceneSettings3D settings;
    settings.gravity = { 0.0f, 0.0f, gravity ? -9.81f : 0.0f };
    settings.workerThreadCount = 1;
    world.scene = world.engine.createScene(settings, world.materials);
    if (gravity) {
        PhysicsShape3D ground;
        ground.type = PhysicsShapeType3D::Box;
        ground.halfExtents = { 20.0f, 20.0f, 0.25f };
        PhysicsBodyDefinition3D body;
        body.motionType = PhysicsMotionType3D::Static;
        body.position = { 0.0f, 0.0f, -0.25f };
        (void)world.scene->createBody(body, std::span<const PhysicsShape3D>(&ground, 1));
    }
    world.spawn.entityId = 1;
    world.spawn.pelvisPosition = { 0.0f, 0.0f, height };
    world.spawn.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
    world.spawn.active = true;
    world.ragdoll = world.scene->createRagdoll(profile, world.spawn);
}

// O toque do raio vertical (de cima ou de baixo) na mao, relativo ao ponto do link; NAN sem toque na mao.
float touch(const Scene& world, const RagdollLinkDefinition3D& link, std::size_t hand, Vec3 local, bool fromAbove) {
    const Vec3 point = world.spawn.pelvisPosition + link.modelPosition + link.modelOrientation.rotate(local);
    const float side = fromAbove ? 1.0f : -1.0f;
    PhysicsRagdollRayHit3D hit;
    const bool any = world.scene->raycastRagdoll(Ray3D { point + Vec3 { 0.0f, 0.0f, 0.5f * side }, { 0.0f, 0.0f, -side } }, 1.0f, hit);
    return any && hit.linkIndex == hand ? hit.position.z - point.z : std::nanf("");
}

Vec3 capsuleTip(const RagdollCapsuleDefinition3D& c) {
    return c.shape == RagdollColliderShape3D::Capsule ? c.localPosition + c.localOrientation.rotate({ ragdollCapsuleHalfSegment3D(c), 0.0f, 0.0f })
                                                      : c.localPosition;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1) AssetProfile = argv[1];
    try {
        const RagdollProfile3D plain = loadRagdollProfile3D(AssetProfile);
        const RagdollProfile3D profile = profileWithExtras(plain);
        const std::size_t hand = linkOf(profile, "LeftHand");
        const RagdollLinkDefinition3D& link = profile.links[hand];
        const auto& extras = link.extraColliders;
        require(!extras.empty(), "os extras da mao lidos");
        validateRagdollProfileOrThrow3D(profile);
        if (plain.links[hand].extraColliders.empty()) {
            require(extras.size() == 2, "dois extras de teste");
            require(extras[0].shape == RagdollColliderShape3D::Capsule && std::abs(extras[0].radiusAtPositiveXMeters - 0.012f) < 1e-6f
                    && std::abs(extras[0].localPosition.y - 0.30f) < 1e-6f,
                "a capsula conica do extra");
            require(extras[1].shape == RagdollColliderShape3D::ConvexHull && extras[1].hullPlanes.size() == 4,
                "a casca do extra preparada (os planos)");
            RagdollProfile3D broken = profile;
            broken.links[hand].extraColliders[0].radiusAtPositiveXMeters = 0.001f;
            bool caught = false;
            try {
                validateRagdollProfileOrThrow3D(broken);
            } catch (const std::exception& error) {
                caught = std::string(error.what()).find("extraColliders[0]") != std::string::npos;
            }
            require(caught, "a validacao acusa o extra invalido pelo caminho dele");
        } else {
            std::cout << "perfil com " << extras.size() << " extras na mao esquerda (" << link.posableBones.size() << " ossos), principal "
                      << (link.collider.shape == RagdollColliderShape3D::ConvexHull ? "casca" : "outro") << '\n';
        }

        // A variacao de corpo leva os extras e os ossos na escala (1,98 m: s = 1,1).
        const RagdollProfile3D tall = bodyRagdollProfile3D(profile, PlayerBody3D { 1.98f, 80.0f });
        const float s = tall.links[hand].modelPosition.length() / profile.links[hand].modelPosition.length();
        require(std::abs(tall.links[hand].extraColliders[0].radiusMeters / extras[0].radiusMeters - s) < 1e-3f
                && std::abs(tall.links[hand].extraColliders[0].localPosition.length() / extras[0].localPosition.length() - s) < 1e-3f,
            "os extras escalam com o corpo");
        if (!link.posableBones.empty())
            require(std::abs(tall.links[hand].posableBones[0].localPosition.length() / link.posableBones[0].localPosition.length() - s) < 1e-3f,
                "os ossos escalam com o corpo");

        // O composto no Jolt: sem gravidade, o raio de cima acerta o principal e a ponta do extra mais longe dele; sem os extras, o
        // mesmo raio na ponta passa.
        const std::vector<RagdollBoneFrame3D> rest = ragdollPosableBoneFrames3D(link);
        std::size_t far = 0;
        float farthest = -1.0f;
        for (std::size_t k = 0; k < extras.size(); ++k) {
            const float d = (capsuleTip(ragdollExtraColliderInLink3D(link, k, rest)) - link.collider.localPosition).length();
            if (d > farthest) {
                farthest = d;
                far = k;
            }
        }
        const Vec3 tip = capsuleTip(ragdollExtraColliderInLink3D(link, far, rest));
        {
            Scene world;
            makeScene(world, profile, false);
            require(!std::isnan(touch(world, link, hand, link.collider.localPosition, true)), "o raio acerta o colisor principal da mao");
            require(!std::isnan(touch(world, link, hand, tip, true)) || !std::isnan(touch(world, link, hand, tip, false)),
                "o raio acerta a ponta do extra (o composto no Jolt)");
            RagdollProfile3D without = profile;
            without.links[hand].extraColliders.clear();
            Scene bare;
            makeScene(bare, without, false);
            require(std::isnan(touch(bare, without.links[hand], hand, tip, true)) && std::isnan(touch(bare, without.links[hand], hand, tip, false)),
                "sem os extras o mesmo raio passa");
        }

        // Os dedos presos aos ossos: esticados (sem rotacao), o topo de cada capsula de dedo fica onde o perfil diz (o polegar fica
        // sob a palma e fica de fora); com 60 graus a mais de flexao em cada osso, a ponta sai do lugar e o raio a acha na pose nova.
        if (extras[far].bone >= 0) {
            Scene world;
            makeScene(world, profile, false);
            std::vector<std::string> names;
            for (const RagdollPosableBone3D& bone : link.posableBones) names.push_back(bone.name);
            require(!std::isnan(touch(world, link, hand, tip, true)) || !std::isnan(touch(world, link, hand, tip, false)),
                "sem pose informada, a ponta do dedo esta no repouso");
            const std::vector<Quaternion> straight(names.size());
            world.scene->setRagdollBoneRotations(world.ragdoll, names, straight);
            const std::vector<RagdollBoneFrame3D> flat = ragdollPosableBoneFrames3D(link, straight);
            float worst = 0.0f;
            for (std::size_t k = 0; k < extras.size(); ++k) {
                if (extras[k].bone < 0 || link.posableBones[static_cast<std::size_t>(extras[k].bone)].name.find("Thumb") != std::string::npos)
                    continue;
                const RagdollCapsuleDefinition3D c = ragdollExtraColliderInLink3D(link, k, flat);
                const float top = touch(world, link, hand, c.localPosition, true);
                require(!std::isnan(top), "o raio acerta a capsula do dedo esticado");
                worst = std::max(worst, std::abs(top - 0.5f * (c.radiusMeters + ragdollCapsuleRadiusAtPositiveX3D(c))));
            }
            require(worst < 0.003f, "cada dedo onde o perfil diz (a menos de 3 mm)");
            std::vector<Quaternion> bent;
            for (const RagdollPosableBone3D& bone : link.posableBones)
                bent.push_back((Quaternion::fromAxisAngle({ 0.0f, -1.0f, 0.0f }, 1.0472f) * bone.restRotation).normalized());
            world.scene->setRagdollBoneRotations(world.ragdoll, names, bent);
            const Vec3 moved = capsuleTip(ragdollExtraColliderInLink3D(link, far, ragdollPosableBoneFrames3D(link, bent)));
            require((moved - tip).length() > 0.03f, "a ponta do dedo sai do lugar com a flexao");
            require(!std::isnan(touch(world, link, hand, moved, true)) || !std::isnan(touch(world, link, hand, moved, false)),
                "com os ossos girados, o raio acerta a ponta na pose nova");
            require(std::isnan(touch(world, link, hand, tip, true)) && std::isnan(touch(world, link, hand, tip, false)),
                "e nao mais na de repouso");
            std::cout << "dedos que acompanham: esticados, o topo de cada capsula a ate " << worst * 1000.0f
                      << " mm do previsto; com 60 graus a ponta andou " << (moved - tip).length() * 100.0f << " cm\n";
        }

        // Seis quedas soltas de 3 s (alturas e rumos diferentes): nada explode e o corpo para. A queda e caotica e as vezes a mao fica
        // presa entre o chao e o proprio corpo (o peso a afunda ~1,7 cm, com os dedos fixos ou nao): a mediana do ponto mais baixo da
        // mao no fim fica no chao. No impacto a penetracao passageira e a do contato a 60 Hz (so registrada).
        std::vector<float> rests, peaks;
        float slowest = 0.0f;
        for (int v = 0; v < 6; ++v) {
            Scene world;
            makeScene(world, profile, true, 0.9f + 0.1f * static_cast<float>(v), 0.5f * static_cast<float>(v));
            float low = 1e9f;
            for (int k = 0; k < 180; ++k) {
                world.scene->simulate(1.0f / 60.0f);
                const RagdollState3D state = world.scene->ragdollState(world.ragdoll);
                for (const PhysicsBodyState3D& part : state.links)
                    require(std::isfinite(part.position.x) && std::isfinite(part.position.z) && part.linearVelocity.length() < 30.0f,
                        "a queda nao explode");
                low = std::min(low, lowestColliderZ(link, state.links[hand]));
            }
            const RagdollState3D state = world.scene->ragdollState(world.ragdoll);
            float speed = 0.0f;
            for (const PhysicsBodyState3D& part : state.links) speed = std::max(speed, part.linearVelocity.length());
            rests.push_back(lowestColliderZ(link, state.links[hand]));
            peaks.push_back(low);
            slowest = std::max(slowest, speed);
        }
        std::vector<float> sorted = rests;
        std::sort(sorted.begin(), sorted.end());
        require(0.5f * (sorted[2] + sorted[3]) > -0.005f, "a mao parada nao fica enterrada (a mediana acima de -5 mm)");
        require(slowest < 0.5f, "o corpo para");
        std::cout << "seis quedas: o ponto mais baixo da mao no fim (cm)";
        for (const float r : rests) std::cout << ' ' << r * 100.0f;
        std::cout << "; o pico no impacto (cm)";
        for (const float l : peaks) std::cout << ' ' << l * 100.0f;
        std::cout << "; a maior velocidade no fim " << slowest << " m/s\ncolisores extras: ok\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FALHOU: " << error.what() << '\n';
        return 1;
    }
}
