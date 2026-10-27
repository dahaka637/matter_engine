// O corpo variavel do laboratorio (26/10): a cache dos corpos (o personagem e
// os clipes de cada corpo, pela receita do AthleteBundle3D) e a capsula do
// corpo. A troca dentro do WorkbenchApp (os bonecos, a malha na GPU) precisa
// da janela e fica fora daqui.
#include "Workbench/Laboratory/LaboratoryBody.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
    if (condition) return;
    std::fprintf(stderr, "LaboratoryBody: %s\n", message.c_str());
    ++failures;
}

bool near(float a, float b, float tolerance) { return std::abs(a - b) <= tolerance; }

} // namespace

int main() {
    const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
    const RagdollCharacter3D base = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const std::vector<AnimationClip3D> clips = loadAthleteClipLibrary3D(assets + "/animations/clips", base.profile);
    expect(!clips.empty(), "a biblioteca de clipes");

    // A capsula do laboratorio ate 26/10 (a copia em WorkbenchApp.hpp, antes de
    // vir de athleteAvatarMotorSettings3D): os mesmos bits.
    CharacterMotorSettings3D laboratory;
    laboratory.radius = 0.34f;
    laboratory.standingHeight = 1.83f;
    laboratory.crouchedHeight = 1.22f;
    laboratory.walkSpeed = 3.0f;
    laboratory.sprintSpeed = 7.5f;
    laboratory.crouchedSpeed = 0.95f;
    laboratory.groundAcceleration = 14.0f;
    laboratory.groundDeceleration = 18.0f;
    laboratory.airAcceleration = 4.5f;
    laboratory.jumpSpeed = 3.6f;
    laboratory.jumpTapSeconds = 0.06f;
    laboratory.jumpFullChargeSeconds = 0.55f;
    laboratory.jumpTapSpeedScale = 0.72f;
    laboratory.jumpChargedSpeedScale = 1.16f;
    laboratory.gravityScale = 1.0f;
    laboratory.maximumFallSpeed = 48.0f;
    laboratory.maximumSlopeDegrees = 50.0f;
    laboratory.maximumStepHeight = 0.19f;
    laboratory.skinWidth = 0.018f;
    laboratory.coyoteTime = 0.10f;
    laboratory.jumpBufferTime = 0.12f;
    laboratory.flightSpeed = 12.0f;
    laboratory.fastFlightSpeed = 28.0f;
    laboratory.swimSpeed = 3.4f;
    laboratory.fastSwimSpeed = 5.0f;
    laboratory.swimAcceleration = 9.0f;
    laboratory.maximumPushSpeedMetersPerSecond = 2.2f;
    laboratory.pushSaturationPenetrationMeters = 0.25f;
    const CharacterMotorSettings3D athlete = athleteAvatarMotorSettings3D();
    const CharacterMotorSettings3D today = athleteAvatarMotorSettings3D(PlayerBody3D {});
    expect(std::memcmp(&laboratory, &athlete, sizeof laboratory) == 0, "a capsula do atleta e a do laboratorio");
    expect(std::memcmp(&laboratory, &today, sizeof laboratory) == 0, "a capsula do corpo de hoje e a do laboratorio");

    Workbench::LaboratoryBodyCache cache;
    // O corpo de hoje: os originais (nada feito).
    expect(cache.select(base, clips, 180.0f, 80.0f) == nullptr, "180/80 devolve os originais");
    expect(cache.select(base, clips, 180.4f, 79.6f) == nullptr, "180/80 arredondado devolve os originais");

    const Workbench::LaboratoryBody* tall = cache.select(base, clips, 200.0f, 100.0f);
    expect(tall != nullptr, "o corpo 200/100");
    if (tall == nullptr) return 1;
    const float s = 2.0f / 1.8f;
    expect(tall->body.heightMeters == 2.0f && tall->body.massKg == 100.0f, "o corpo pedido");
    float mass = 0.0f;
    for (const auto& link : tall->character.profile.links) mass += link.massFraction * tall->character.profile.totalMassKg;
    expect(tall->character.profile.totalMassKg == 100.0f && near(mass, 100.0f, 1e-3f), "a massa dos bonecos: 100 kg");
    expect(near(tall->character.profile.standingRootHeightMeters, base.profile.standingRootHeightMeters * s, 1e-6f),
        "a altura de pe x s");
    expect(near(tall->character.mesh.boundsMax.z - tall->character.mesh.boundsMin.z,
               (base.mesh.boundsMax.z - base.mesh.boundsMin.z) * s, 0.01f),
        "a malha x s");
    // A malha no tonus (a do encorpado, nao so a de hoje em escala).
    const RagdollCharacter3D scaledOnly = bodyRagdollCharacter3D(base, tall->body, nullptr);
    bool toned = false;
    for (std::size_t v = 0; v < scaledOnly.mesh.vertices.size() && !toned; ++v)
        toned = (tall->character.mesh.vertices[v].position - scaledOnly.mesh.vertices[v].position).length() > 1e-3f;
    expect(toned, "a malha no tonus");
    expect(tall->character.mesh.indices == base.mesh.indices, "os mesmos indices da malha de hoje");
    expect(tall->clips.size() == clips.size(), "um clipe do corpo por clipe");
    const CharacterMotorSettings3D capsule = athleteAvatarMotorSettings3D(tall->body);
    expect(near(capsule.radius, today.radius * s, 1e-6f) && near(capsule.standingHeight, today.standingHeight * s, 1e-6f)
            && near(capsule.crouchedHeight, today.crouchedHeight * s, 1e-6f),
        "a capsula x s");

    // Os corpos ficam vivos: o mesmo pedido, o mesmo objeto; outro corpo, outro
    // endereco (as tabelas por endereco nunca veem um corpo no lugar de outro).
    const Workbench::LaboratoryBody* light = cache.select(base, clips, 170.0f, 60.0f);
    expect(light != nullptr && light != tall && &light->character.profile != &tall->character.profile
            && light->clips.data() != tall->clips.data(),
        "outro corpo, outros objetos");
    expect(cache.select(base, clips, 200.0f, 100.0f) == tall, "a volta ao 200/100 e o mesmo objeto");
    expect(cache.select(base, clips, 180.0f, 80.0f) == nullptr, "a volta ao 180/80 devolve os originais");

    if (failures == 0) std::printf("LaboratoryBody: ok\n");
    return failures == 0 ? 0 : 1;
}
