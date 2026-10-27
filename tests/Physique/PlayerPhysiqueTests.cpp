// O corpo variavel (26/10): a lei (Engine/Physique), o perfil fisico do corpo
// (RagdollProfileBody3D) e o personagem do corpo (RagdollCharacterBody3D). O
// corpo de hoje sai bit a bit igual; nos cantos (1,70/2,00 m x 65/95 kg) o
// perfil e valido, pesa o que deve e nao cria par sobreposto, e a pele do
// corpo (a malha no tonus e x s) e a mesma da GPU (a malha de hoje com os
// deltas e a paleta x S(s)).
#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Animation/RagdollCharacterBody3D.hpp"
#include "Engine/Physics/RagdollProfileBody3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"
#include "Engine/Gesture/GestureChoice3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

int g_checks = 0;

void expect(bool condition, const std::string& what) {
    ++g_checks;
    if (!condition) throw std::runtime_error(what);
}

bool same(Vec3 a, Vec3 b) { return std::memcmp(&a, &b, sizeof a) == 0; }
bool near(Vec3 a, Vec3 b, float tolerance) { return (a - b).length() <= tolerance; }
std::string text(const PlayerBody3D& body) {
    return std::to_string(static_cast<int>(std::lround(body.heightMeters * 100.0f))) + " cm / "
        + std::to_string(static_cast<int>(std::lround(body.massKg))) + " kg";
}

const PlayerBody3D Base {};
const PlayerBody3D Corners[] = { { 1.70f, 60.0f }, { 1.70f, 100.0f }, { 2.00f, 60.0f }, { 2.00f, 100.0f }, { 1.87f, 72.0f } };

void lawTests() {
    expect(isBaseBody3D(Base) && bodyScale3D(Base) == 1.0f && bodyTone3D(Base) == 0.0f, "o corpo de hoje: escala 1 e tonus 0");
    const BodyMotorScale3D unit = bodyMotorScale3D(Base);
    expect(unit.torque == 1.0f && unit.stiffness == 1.0f && unit.damping == 1.0f, "os motores do corpo de hoje intactos");
    expect(std::abs(bodyScale3D({ 2.0f, 80.0f }) - 2.0f / 1.8f) < 1e-6f, "a escala e a altura sobre 1,80");
    // O volume da malha medido: a franzina da folha pesa 70 kg; a encorpada, ~99.
    expect(std::abs(bodyTone3D({ 1.8f, 70.0f }) + 1.0f) < 0.02f && std::abs(BaseBodyMassKg * bodyToneVolume3D(1.0f) - 98.9f) < 0.3f
            && std::abs(bodyTone3D({ 1.8f, 98.9f }) - 0.95f) < 0.02f,
        "o tonus pelo volume: -1 a 70 kg; a forma +1 pesa ~99 kg e, com o joelho em +0,9, 99 kg da +0,95");
    expect(bodyTone3D({ 2.0f, 60.0f }) > -3.0f && bodyTone3D({ 2.0f, 60.0f }) < bodyTone3D({ 2.0f, 61.0f }) - 0.02f,
        "o joelho do franzino: o canto para em ~-3,0 e cada quilo ainda muda a forma");
    expect(bodyTone3D({ 1.7f, 100.0f }) < 1.45f && bodyTone3D({ 1.7f, 100.0f }) > bodyTone3D({ 1.7f, 99.0f }) + 0.02f
            && bodyTone3D({ 1.8f, 100.0f }) < 1.0f,
        "o joelho do encorpado (+0,9): o canto para em ~+1,4, o de 1,80 m e 100 kg abaixo de +1, e cada quilo ainda muda a forma");
    for (float tone = -2.25f; tone <= 0.9f; tone += 0.25f) {
        const float kg = BaseBodyMassKg * bodyToneVolume3D(tone);
        expect(std::abs(bodyTone3D({ 1.8f, kg }) - tone) < 1e-3f, "o tonus e o volume se invertem em " + std::to_string(tone));
    }
    expect(bodyTone3D({ 2.0f, 90.0f }) < bodyTone3D({ 1.7f, 90.0f }) - 1.0f, "o alto do mesmo peso e mais magro");
    // O peso equivalente: peso / s^3 nos baixos e peso / s^2 nos altos (o ajuste dele, 26/10).
    const float shortSame = bodyTone3D({ 1.7f, 80.0f * (1.7f / 1.8f) * (1.7f / 1.8f) * (1.7f / 1.8f) });
    const float tallSame = bodyTone3D({ 2.0f, 80.0f * (2.0f / 1.8f) * (2.0f / 1.8f) });
    expect(std::abs(shortSame) < 1e-3f && std::abs(tallSame) < 1e-3f, "o mesmo fisico: peso x s^3 nos baixos, peso x s^2 nos altos");
    expect(bodyTone3D({ 2.0f, 100.0f }) > -0.1f && bodyTone3D({ 2.0f, 100.0f }) > bodyTone3D({ 1.9f, 90.0f }),
        "o alto pesado nao fica franzino (2,00 m e 100 kg: o fisico do de hoje)");
    const auto parsed = parseBody3D("200,100");
    expect(parsed && parsed->heightMeters == 2.0f && parsed->massKg == 100.0f, "--corpo 200,100");
    expect(parseBody3D("1.75x70").has_value() && !parseBody3D("250,80") && !parseBody3D("180") && !parseBody3D("180,80,1"),
        "--corpo: metros, separadores e fora da faixa");
    for (int r = 0; r < static_cast<int>(PhysiqueRegion3D::Count); ++r) {
        const auto region = static_cast<PhysiqueRegion3D>(r);
        float previous = 0.0f;
        for (int k = -15; k <= 15; ++k) {
            const float girth = bodyGirth3D(region, static_cast<float>(k) / 10.0f);
            expect(girth >= previous && girth > 0.8f && girth < 1.45f && (k == -15 || girth > previous || region == PhysiqueRegion3D::Foot || (region == PhysiqueRegion3D::Hand && k <= 0)), "a grossura cresce com o peso, sem salto");
            previous = girth;
        }
    }
    expect(bodyGirth3D(PhysiqueRegion3D::Foot, 1.5f) == 1.0f && bodyGirth3D(PhysiqueRegion3D::Head, 1.0f) < 1.06f, "a chuteira e a cabeca");
    expect(physiqueRegionOfLink3D("LeftUpperArm") == PhysiqueRegion3D::UpperArm && !physiqueRegionOfLink3D("Tail"), "a regiao pelo nome");
}

void profileTests(const RagdollProfile3D& profile) {
    const RagdollProfile3D same0 = bodyRagdollProfile3D(profile, Base);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D &a = profile.links[i], &b = same0.links[i];
        expect(same(a.modelPosition, b.modelPosition) && same(a.collider.localPosition, b.collider.localPosition)
                && a.collider.radiusMeters == b.collider.radiusMeters && a.collider.lengthMeters == b.collider.lengthMeters
                && a.massFraction == b.massFraction && a.inboundJoint.axes[0].maximumTorque == b.inboundJoint.axes[0].maximumTorque,
            "o perfil do corpo de hoje igual ao original: " + a.id);
    }
    const auto index = [&](const char* id) {
        for (std::size_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        throw std::runtime_error(std::string("sem o link ") + id);
    };
    const std::size_t head = index("Head"), thighL = index("LeftThigh"), thighR = index("RightThigh");
    // Pai e filho, e os pares que o perfil ja declara sem colisao.
    const auto related = [&](std::size_t a, std::size_t b) {
        for (const RagdollIgnoredCollisionPair3D& pair : profile.selfCollisionIgnoredPairs)
            if ((pair.firstLinkId == profile.links[a].id && pair.secondLinkId == profile.links[b].id)
                || (pair.firstLinkId == profile.links[b].id && pair.secondLinkId == profile.links[a].id))
                return true;
        return profile.links[a].parentIndex == static_cast<int>(b) || profile.links[b].parentIndex == static_cast<int>(a);
    };
    for (const PlayerBody3D& body : Corners) {
        const RagdollProfile3D v = bodyRagdollProfile3D(profile, body);
        const float s = bodyScale3D(body);
        expect(validateRagdollProfile3D(v).empty(), "o perfil valido em " + text(body));
        float fractions = 0.0f;
        for (const auto& link : v.links) fractions += link.massFraction;
        expect(std::abs(fractions - 1.0f) < 1e-5f && v.totalMassKg == body.massKg, "a massa do jogador em " + text(body));
        expect(std::abs(v.standingRootHeightMeters - profile.standingRootHeightMeters * s) < 1e-6f, "a altura de pe x s");
        for (std::size_t i = 0; i < v.links.size(); ++i) {
            expect(near(v.links[i].modelPosition, profile.links[i].modelPosition * s, 1e-6f), "o link x s: " + v.links[i].id);
            for (std::size_t a = 0; a < 3; ++a)
                expect(v.links[i].inboundJoint.axes[a].minimumRadians == profile.links[i].inboundJoint.axes[a].minimumRadians,
                    "os limites das juntas nao mudam");
        }
        expect(ragdollRestCapsuleGap3D(v.links[thighL], v.links[thighR]) >= 0.024f * s - 1e-4f,
            "as coxas afastadas no repouso em " + text(body));
        // Nenhum par novo sobreposto no repouso (o backend o desligaria calado).
        for (std::size_t a = 0; a < v.links.size(); ++a)
            for (std::size_t b = a + 1; b < v.links.size(); ++b) {
                if (related(a, b) || v.links[a].collider.shape != RagdollColliderShape3D::Capsule
                    || v.links[b].collider.shape != RagdollColliderShape3D::Capsule)
                    continue;
                if (ragdollRestCapsuleGap3D(profile.links[a], profile.links[b]) > 0.0f)
                    expect(ragdollRestCapsuleGap3D(v.links[a], v.links[b]) > 0.0f,
                        "par novo sobreposto em " + text(body) + ": " + v.links[a].id + " x " + v.links[b].id);
            }
    }
    // Na mesma altura, cada link pesa mais com o peso (a cabeca inclusive).
    for (float height : { 1.70f, 1.80f, 2.00f }) {
        const RagdollProfile3D light = bodyRagdollProfile3D(profile, { height, 60.0f });
        const RagdollProfile3D middle = bodyRagdollProfile3D(profile, { height, 80.0f });
        const RagdollProfile3D heavy = bodyRagdollProfile3D(profile, { height, 100.0f });
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            const float a = light.links[i].massFraction * 60.0f, b = middle.links[i].massFraction * 80.0f,
                        c = heavy.links[i].massFraction * 100.0f;
            expect(a <= b + 1e-4f && b <= c + 1e-4f, "a massa de " + profile.links[i].id + " cresce com o peso");
        }
        const float headHeavy = heavy.links[head].massFraction * 100.0f, headBase = middle.links[head].massFraction * 80.0f;
        expect(headHeavy - headBase < 0.05f * 20.0f, "a cabeca leva pouco do ganho");
    }
}

void characterTests(const RagdollCharacter3D& character, const std::string& assets) {
    const RagdollToneDeltas3D tone = computeRagdollToneDeltas3D(character);
    expect(tone.heavyPosition.size() == character.mesh.vertices.size(), "um delta por vertice da pele");
    const RagdollCharacter3D same0 = bodyRagdollCharacter3D(character, Base, &tone);
    for (std::size_t v = 0; v < character.mesh.vertices.size(); ++v)
        expect(same(same0.mesh.vertices[v].position, character.mesh.vertices[v].position)
                && same(same0.mesh.vertices[v].normal, character.mesh.vertices[v].normal),
            "a malha do corpo de hoje igual a original");
    // A chuteira (so os ossos do pe) nao muda com o peso.
    std::size_t footOnly = 0;
    for (std::size_t v = 0; v < character.mesh.vertices.size(); ++v) {
        const MeshVertex3D& vertex = character.mesh.vertices[v];
        bool foot = true;
        for (std::size_t k = 0; k < 4; ++k) {
            if (vertex.weights[k] <= 0.0f) continue;
            const std::size_t bone = vertex.joints[k];
            foot = foot && bone < character.boneLinkIndices.size()
                && character.profile.links[character.boneLinkIndices[bone]].id.find("Foot") != std::string::npos;
        }
        if (!foot) continue;
        ++footOnly;
        expect(tone.heavyPosition[v].lengthSquared() == 0.0f && tone.lightPosition[v].lengthSquared() == 0.0f, "a chuteira rigida");
    }
    expect(footOnly > 100, "ha vertices so da chuteira");

    const AnimationClip3D sprint = loadAnimationClip3D(assets + "/animations/clips/sprint.matteranim.json");
    for (const PlayerBody3D& body : Corners) {
        const RagdollCharacter3D v = bodyRagdollCharacter3D(character, body, &tone);
        const float s = bodyScale3D(body), w = bodyTone3D(body);
        const AnimationClip3D clip = scaledAnimationClip3D(sprint, s);
        for (const auto& [name, pose] : { std::pair { "repouso", sampleRagdollAnimationPose3D(v.profile, nullptr, 0.0f, true) },
                 std::pair { "sprint", sampleRagdollAnimationPose3D(v.profile, &clip, 0.15f, true) } }) {
            const std::vector<Mat4> palette = buildRagdollSkinMatrices3D(v, pose.linkPositions, pose.linkOrientations);
            std::vector<Mat4> gpu = palette;
            scaleRagdollSkinPalette3D(gpu, s);
            float worst = 0.0f, lowest = 1e9f;
            for (std::size_t g = 0; g < v.mesh.vertices.size(); ++g) {
                MeshVertex3D toned = character.mesh.vertices[g];
                toned.position = ragdollTonePosition3D(tone, g, toned.position, w);
                const Vec3 cpu = skinVertexPosition3D(v.mesh.vertices[g], palette);
                worst = std::max(worst, (cpu - skinVertexPosition3D(toned, gpu)).length());
                lowest = std::min(lowest, cpu.z);
            }
            expect(worst < 2e-5f * s, std::string("a GPU (deltas e paleta x S) igual a malha do corpo em ") + name + ", "
                    + text(body) + ": " + std::to_string(worst * 1000.0f) + " mm");
            if (std::string(name) == "repouso")
                expect(std::abs(lowest + v.profile.standingRootHeightMeters) < 0.003f * s,
                    "a sola no chao do corpo " + text(body) + ": " + std::to_string((lowest + v.profile.standingRootHeightMeters) * 1000.0f) + " mm");
        }
    }
    // O tonus 0 em outra altura e o corpo de hoje x s.
    const PlayerBody3D tall { 1.7f, 80.0f * (1.7f / 1.8f) * (1.7f / 1.8f) * (1.7f / 1.8f) };
    const RagdollCharacter3D v = bodyRagdollCharacter3D(character, tall, &tone);
    const auto rest = sampleRagdollAnimationPose3D(character.profile, nullptr, 0.0f, true);
    const auto restTall = sampleRagdollAnimationPose3D(v.profile, nullptr, 0.0f, true);
    const std::vector<Mat4> a = buildRagdollSkinMatrices3D(character, rest.linkPositions, rest.linkOrientations);
    const std::vector<Mat4> b = buildRagdollSkinMatrices3D(v, restTall.linkPositions, restTall.linkOrientations);
    float worst = 0.0f;
    const float s = bodyScale3D(tall);
    for (std::size_t g = 0; g < v.mesh.vertices.size(); ++g)
        worst = std::max(worst, (skinVertexPosition3D(character.mesh.vertices[g], a) * s - skinVertexPosition3D(v.mesh.vertices[g], b)).length());
    expect(worst < 2e-5f, "1,70 m e 67,4 kg (o mesmo volume em escala): o corpo de hoje x s (" + std::to_string(worst * 1000.0f) + " mm)");
}

// A folga nas poses dos clipes de locomocao (a do exportador: o braco a 4 mm do
// tronco, as coxas a 8 mm): a pior de cada par.
struct Clearance {
    float arm = 1e9f, thigh = 1e9f;
};
Clearance clipClearance(const RagdollProfile3D& profile, const std::vector<AnimationClip3D>& clips, float s, float armOpen, float legOpen) {
    const auto part = [](const std::string& id) {
        return id.rfind("Left", 0) == 0 ? id.substr(4) : id.rfind("Right", 0) == 0 ? id.substr(5) : id;
    };
    Clearance out;
    for (const AnimationClip3D& source : clips) {
        const AnimationClip3D clip = bodyAnimationClip3D(source, profile, s, armOpen, legOpen);
        for (int k = 0; k <= 40; ++k) {
            const auto pose = sampleRagdollAnimationPose3D(profile, &clip, clip.durationSeconds * static_cast<float>(k) / 40.0f, true);
            for (std::size_t a = 0; a < profile.links.size(); ++a)
                for (std::size_t b = a + 1; b < profile.links.size(); ++b) {
                    const std::string pa = part(profile.links[a].id), pb = part(profile.links[b].id);
                    const bool armA = pa == "UpperArm" || pa == "Forearm" || pa == "Hand", armB = pb == "UpperArm" || pb == "Forearm" || pb == "Hand";
                    const bool torsoA = pa == "Pelvis" || pa == "Abdomen" || pa == "Chest" || pa == "UpperChest";
                    const bool torsoB = pb == "Pelvis" || pb == "Abdomen" || pb == "Chest" || pb == "UpperChest";
                    const bool armTorso = (armA && torsoB) || (armB && torsoA), thighs = pa == "Thigh" && pb == "Thigh";
                    if (!armTorso && !thighs) continue;
                    if (profile.links[a].parentIndex == static_cast<int>(b) || profile.links[b].parentIndex == static_cast<int>(a)) continue;
                    const float gap = ragdollPosedCapsuleGap3D(profile.links[a], pose.linkPositions[a], pose.linkOrientations[a],
                        profile.links[b], pose.linkPositions[b], pose.linkOrientations[b]);
                    float& worst = armTorso ? out.arm : out.thigh;
                    if (armTorso && gap < out.arm && std::getenv("XFOLGA"))
                        std::cout << "  " << profile.links[a].id << " x " << profile.links[b].id << " " << clip.id << "@" << k
                                  << ": " << gap * 1000.0f << " mm\n";
                    worst = std::min(worst, gap);
                }
        }
    }
    return out;
}

std::vector<AnimationClip3D> locomotionClips(const RagdollCharacter3D& character, const std::string& assets) {
    std::vector<std::string> ids { character.idleClipId, character.walkClipId, character.walkBackwardClipId, character.sprintClipId,
        character.sprintBackwardClipId, character.strafeLeftClipId, character.strafeRightClipId, character.sprintStrafeLeftClipId,
        character.sprintStrafeRightClipId, character.runForwardArcLeftClipId, character.runForwardArcRightClipId };
    for (const auto& [role, id] : character.defaultLocomotionClipIds) ids.push_back(id);
    std::vector<AnimationClip3D> clips;
    for (const std::string& id : ids)
        if (!id.empty()) clips.push_back(loadAnimationClip3D(assets + "/animations/clips/" + id + ".matteranim.json"));
    return clips;
}

void clearanceTests(const RagdollCharacter3D& character, const std::string& assets) {
    const std::vector<AnimationClip3D> library = locomotionClips(character, assets);
    std::vector<const AnimationClip3D*> clips;
    for (const AnimationClip3D& clip : library) clips.push_back(&clip);
    const Clearance base = clipClearance(character.profile, library, 1.0f, 0.0f, 0.0f);
    for (const PlayerBody3D& body : { PlayerBody3D { 1.80f, 100.0f }, Corners[0], Corners[1], Corners[2], Corners[3], Corners[4] }) {
        const RagdollProfile3D profile = bodyRagdollProfileForClips3D(character.profile, body, clips);
        const float s = bodyScale3D(body);
        const Clearance c = clipClearance(profile, library, s, bodyArmOpening3D(bodyTone3D(body)), 0.0f);
        std::cout << "folga do corpo " << text(body) << ": braco " << c.arm * 1000.0f << " mm, coxas " << c.thigh * 1000.0f << " mm\n";
        expect(c.arm >= std::min(base.arm, 0.004f * s) - 1e-3f && c.thigh >= std::min(base.thigh, 0.008f * s) - 1e-3f,
            "a folga do exportador nos clipes em " + text(body));
        expect(validateRagdollProfile3D(profile).empty(), "o perfil com a folga valido em " + text(body));
    }
}

void clearanceReport(const RagdollCharacter3D& character, const std::string& assets) {
    const std::vector<AnimationClip3D> clips = locomotionClips(character, assets);
    const Clearance base = clipClearance(character.profile, clips, 1.0f, 0.0f, 0.0f);
    std::cout << "folga de hoje: braco " << base.arm * 1000.0f << " mm, coxas " << base.thigh * 1000.0f << " mm\n";
    for (float kg : { 90.0f, 100.0f }) {
        const PlayerBody3D body { 1.80f, kg };
        const RagdollProfile3D profile = bodyRagdollProfile3D(character.profile, body);
        float found[2] = {};
        for (int which = 0; which < 2; ++which) {
            float low = 0.0f, high = 0.5f;
            for (int it = 0; it < 14; ++it) {
                const float mid = 0.5f * (low + high);
                const Clearance c = clipClearance(profile, clips, 1.0f, which == 0 ? mid : 0.0f, which == 1 ? mid : 0.0f);
                ((which == 0 ? c.arm >= base.arm : c.thigh >= base.thigh) ? high : low) = mid;
            }
            found[which] = high;
        }
        std::cout << "abertura para a folga de hoje, " << kg << " kg (tonus " << bodyTone3D(body) << "): braco "
                  << found[0] * 57.29578f << " graus, coxa " << found[1] * 57.29578f << " graus\n";
    }
}

void reachTests() {
    const GestureReach3D base;
    const GestureReach3D same0 = gestureReachForBody3D(base, 1.0f, 0.0f);
    expect(same0.front == base.front && same0.headTop == base.headTop && same0.bodySide == base.bodySide, "o alcance de hoje intacto");
    for (const PlayerBody3D& body : Corners) {
        const float s = bodyScale3D(body);
        const GestureReach3D r = gestureReachForBody3D(base, s, bodyGirth3D(PhysiqueRegion3D::Chest, bodyTone3D(body)) - 1.0f);
        expect(std::abs(r.front - base.front * s) < 1e-6f && std::abs(r.highBall - base.highBall * s) < 1e-6f
                && std::abs(r.reachFar - base.reachFar * s) < 1e-6f,
            "os pes na proporcao da altura em " + text(body));
        expect(std::abs((r.headTop - r.chestTop) - (base.headTop - base.chestTop)) < 1e-6f && r.headFront == base.headFront
                && r.headSide == base.headSide && r.headBack == base.headBack,
            "a cabeca do mesmo tamanho em " + text(body));
        expect(std::abs(r.chestTop - base.chestTop * s) < 1e-6f, "a cabeca sobe com o pescoco em " + text(body));
    }
}

void clipTests(const std::string& assets) {
    const AnimationClip3D clip = loadAnimationClip3D(assets + "/animations/clips/jog.matteranim.json");
    const AnimationClip3D one = scaledAnimationClip3D(clip, 1.0f);
    const AnimationClip3D tall = scaledAnimationClip3D(clip, 1.1f);
    expect(one.nominalSpeedMetersPerSecond == clip.nominalSpeedMetersPerSecond, "o clipe x 1 igual");
    expect(std::abs(tall.nominalSpeedMetersPerSecond - 1.1f * clip.nominalSpeedMetersPerSecond) < 1e-5f, "a velocidade nominal x s");
    for (std::size_t t = 0; t < clip.tracks.size(); ++t)
        for (std::size_t k = 0; k < clip.tracks[t].keyframes.size(); ++k) {
            const AnimationKeyframe3D &a = clip.tracks[t].keyframes[k], &b = tall.tracks[t].keyframes[k];
            expect(same(a.jointPositionRadians, b.jointPositionRadians), "as juntas do clipe nao mudam");
            if (clip.tracks[t].space == AnimationTrackSpace3D::Root)
                expect(near(b.translationOffsetMeters, a.translationOffsetMeters * 1.1f, 1e-6f), "a raiz x s");
        }
}

} // namespace

int main() {
    try {
        const std::string assets = MATTERENGINE_TEST_ASSETS_DIR;
        const RagdollCharacter3D character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
        lawTests();
        profileTests(character.profile);
        characterTests(character, assets);
        clipTests(assets);
        reachTests();
        clearanceTests(character, assets);
        if (std::getenv("XFOLGA")) clearanceReport(character, assets);
    } catch (const std::exception& error) {
        std::cerr << "corpo: " << error.what() << "\n";
        return 1;
    }
    std::cout << "corpo: " << g_checks << " checagens, tudo certo\n";
    return 0;
}
