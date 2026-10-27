// Sonda da locomocao padrao (04/10): o boneco pela capsula, sem fisica
// (DefaultLocomotion3D), em roteiros de teclas - arrancar, sprint, lados,
// inverter, zigue-zague, girar a camera, pular. Mede o que o usuario pediu
// ("rapido e perfeito, sem tremer, sem desequilibrar"):
//   - pes: o quanto a sola entra no chao ou flutua apoiada;
//   - deslize: a velocidade do pe apoiado (a capsula anda numa velocidade e
//     o clipe noutra);
//   - suavidade: o maior pico de aceleracao de um link (tremor, salto de
//     pose) e o maior giro de um link num tick;
//   - a pelve em relacao a capsula.
//
// uso: MatterLocomotionProbe [--cenario nome] [--acel 50] [--desacel 60]
//      [--dump arquivo] [--cada 4]
//      [--gestos] [--tempo-manejo 0.35] [--tempo-chute 0.55]
//      [--corpo cm,kg] (o perfil e os clipes de outro corpo; o dump ganha "corpo <m> <kg>")
// --gestos: a bateria dos gestos de bola (24/10) - os 36 manejos e os 6
// chutes em 3 forcas, parado, trotando e correndo; mede o tranco, o joelho
// invertido, o pe no chao, as pernas cruzando, o apoio escorregando e o voo
// no gesto (nomes g_<marcha>_toque_<perna><alcance><frente-tras>_<dentro|fora>
// e g_<marcha>_chute_<perna><alcance>_<forca>).
// Sem --cenario roda todos. O dump vai para MatterModelInspector quadros.
#include "Engine/Animation/RagdollVisualPose3D.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"
#include "Engine/Gesture/GestureCatalog3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <tuple>
#include <vector>

using namespace MatterEngine;

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Dt = 1.0f / 120.0f;

struct Keys {
    // Teclas no referencial da camera (x frente, y esquerda).
    float forward = 0.0f;
    float left = 0.0f;
    bool sprint = false;
    // O sprint maximo (o Shift duplo do laboratorio): pede maxSprintSpeed.
    bool maxSprint = false;
    float cameraYaw = 0.0f;
    bool jump = false;
    // A velocidade do pulo (m/s; a do laboratorio e 4,6, carregado 5,3).
    // (A do jogo: 3,0 m/s, 24/10.)
    float jumpSpeed = 3.0f;
    // Carrinho (a borda do agachar correndo).
    bool tackle = false;
    // Carga do pulo parado (0..1): o agachar de preparo.
    float jumpCrouch = 0.0f;
};

struct Scenario {
    const char* name;
    float seconds;
    std::function<Keys(float)> keys;
    // O gesto de bola pedido em gestureAt (s; < 0 nenhum).
    GestureSpec3D gesture {};
    float gestureAt = -1.0f;
};

// A bateria dos gestos (--gestos).
std::vector<Scenario> gestureScenarios(float touchSeconds, float shotSeconds) {
    static std::deque<std::string> names;
    std::vector<Scenario> list;
    const char* gaits[3] = { "parado", "trote", "corre" };
    for (int gait = 0; gait < 3; ++gait) {
        // Andando, para 0,3 s depois do gesto e fica parado (os pes tem que
        // voltar ao lugar: o "perna deslocada" depois do gesto).
        // Parado, a camera girando (o corpo vira com ela): durante o gesto
        // (comeca 0,1 s antes) e na volta dele, 70 graus para cada lado.
        const auto addTurns = [&](const GestureSpec3D& spec, float seconds) {
            const std::string base = names.back();
            for (const float from : { 0.9f, 1.0f + 0.6f * seconds })
                for (const float angle : { 1.22f, -1.22f }) {
                    names.push_back(base + (from < 0.95f ? "_gira" : "_giradepois") + (angle > 0.0f ? "E" : "D"));
                    const auto keys = [from, angle](float t) {
                        Keys k;
                        k.cameraYaw = angle * std::clamp((t - from) / 0.5f, 0.0f, 1.0f);
                        return k;
                    };
                    list.push_back({ names.back().c_str(), 1.0f + seconds + 2.2f, keys, spec, 1.0f });
                }
        };
        const auto keysFor = [gait](float stopAt) {
            return [gait, stopAt](float t) {
                Keys k;
                if (gait > 0 && t > 0.2f && t < stopAt) k.forward = 1.0f;
                k.sprint = gait == 2;
                return k;
            };
        };
        for (int side = 0; side < 2; ++side) {
            for (int reach = 0; reach < 3; ++reach) {
                for (int depth = 0; depth < 3; ++depth)
                    for (int move = 0; move < 2; ++move) {
                        GestureSpec3D spec;
                        spec.kind = GestureKind3D::Touch;
                        spec.side = side;
                        spec.reach = reach;
                        spec.depth = depth;
                        spec.move = move == 0 ? GestureMove3D::In : GestureMove3D::Out;
                        spec.power = 0.4f;
                        spec.seconds = touchSeconds;
                        spec.seed = static_cast<std::uint32_t>(1 + list.size());
                        names.push_back(std::string("g_") + gaits[gait] + "_toque_" + (side == 0 ? "E" : "D")
                            + std::to_string(reach) + std::to_string(depth) + (move == 0 ? "_dentro" : "_fora"));
                        list.push_back({ names.back().c_str(), 1.0f + touchSeconds + 2.2f, keysFor(1.3f + touchSeconds), spec, 1.0f });
                        if (gait == 0) addTurns(spec, touchSeconds);
                    }
                for (const float power : { 0.2f, 0.6f, 1.0f }) {
                    GestureSpec3D spec;
                    spec.kind = GestureKind3D::Shot;
                    spec.side = side;
                    spec.reach = reach;
                    spec.power = power;
                    spec.seconds = shotSeconds;
                    spec.seed = static_cast<std::uint32_t>(1 + list.size());
                    names.push_back(std::string("g_") + gaits[gait] + "_chute_" + (side == 0 ? "E" : "D")
                        + std::to_string(reach) + "_" + std::to_string(static_cast<int>(power * 10.0f + 0.5f)));
                    list.push_back({ names.back().c_str(), 1.0f + shotSeconds + 2.2f, keysFor(1.3f + shotSeconds), spec, 1.0f });
                    if (gait == 0) addTurns(spec, shotSeconds);
                }
            }
        }
    }
    return list;
}

bool pulse(float t, float at) { return t >= at && t < at + 0.008f; }

std::vector<Scenario> scenarios() {
    return {
        { "arranca", 3.5f, [](float t) {
            Keys k; if (t > 0.5f && t < 2.5f) k.forward = 1.0f; return k; } },
        // Trote reto, longo (a amplitude do trote, 06/10).
        { "trote", 4.0f, [](float t) {
            Keys k; if (t > 0.5f && t < 3.8f) k.forward = 1.0f; return k; } },
        { "sprint", 4.0f, [](float t) {
            Keys k; if (t > 0.5f && t < 3.0f) { k.forward = 1.0f; k.sprint = true; } return k; } },
        { "lados", 6.0f, [](float t) {
            Keys k;
            if (t > 0.5f && t < 1.8f) k.left = 1.0f;
            else if (t < 3.1f && t >= 1.8f) k.left = -1.0f;
            else if (t < 4.4f && t >= 3.1f) k.forward = -1.0f;
            else if (t < 5.5f && t >= 4.4f) k.forward = 1.0f;
            return k; } },
        { "inverte", 4.0f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 1.5f) k.forward = 1.0f;
            else if (t >= 1.5f && t < 2.7f) k.forward = -1.0f;
            else if (t >= 2.7f && t < 3.6f) k.forward = 1.0f;
            return k; } },
        { "diagonais", 5.0f, [](float t) {
            Keys k;
            const int part = static_cast<int>(t / 0.6f) % 8;
            const float a = part * Pi * 0.25f;
            if (t > 0.3f && t < 4.6f) { k.forward = std::cos(a); k.left = std::sin(a); }
            return k; } },
        { "zigue", 4.5f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 4.0f) {
                k.sprint = true;
                const float a = (static_cast<int>(t / 0.45f) % 2 == 0 ? 1.0f : -1.0f) * Pi * 0.25f;
                k.forward = std::cos(a); k.left = std::sin(a);
            }
            return k; } },
        { "costas", 4.0f, [](float t) {
            // Trote de costas (Jog Backward).
            Keys k; if (t > 0.3f && t < 3.6f) k.forward = -1.0f; return k; } },
        { "costasprint", 4.0f, [](float t) {
            // Sprint de costas (Running Backward).
            Keys k; if (t > 0.3f && t < 3.6f) { k.forward = -1.0f; k.sprint = true; } return k; } },
        { "ladosprint", 5.0f, [](float t) {
            // Sprint de lado: esquerda, depois direita.
            Keys k;
            if (t > 0.3f && t < 2.3f) { k.left = 1.0f; k.sprint = true; }
            else if (t >= 2.3f && t < 4.3f) { k.left = -1.0f; k.sprint = true; }
            return k; } },
        { "curvadireita", 5.0f, [](float t) {
            // Sprint em curva para a direita, a camera girando (Running Right Turn).
            Keys k;
            if (t > 0.3f && t < 4.6f) { k.forward = 1.0f; k.sprint = true; }
            k.cameraYaw = t > 1.2f ? -(t - 1.2f) * 1.1f : 0.0f;
            return k; } },
        { "curvasuave", 5.0f, [](float t) {
            // Sprint numa curva leve (a camera a 0,3 rad/s: raio ~25 m).
            Keys k;
            if (t > 0.3f && t < 4.6f) { k.forward = 1.0f; k.sprint = true; }
            k.cameraYaw = t > 1.5f ? (t - 1.5f) * 0.3f : 0.0f;
            return k; } },
        { "curvamedia", 5.0f, [](float t) {
            // Sprint numa curva media (0,6 rad/s: raio ~13 m).
            Keys k;
            if (t > 0.3f && t < 4.6f) { k.forward = 1.0f; k.sprint = true; }
            k.cameraYaw = t > 1.5f ? (t - 1.5f) * 0.6f : 0.0f;
            return k; } },
        { "trotesuave", 5.0f, [](float t) {
            // Trote mudando um pouco de direcao (0,5 rad/s por 1 s).
            Keys k;
            if (t > 0.3f && t < 4.6f) k.forward = 1.0f;
            k.cameraYaw = t > 1.5f ? std::min(t - 1.5f, 1.0f) * 0.5f : 0.0f;
            return k; } },
        { "circulo", 5.0f, [](float t) {
            // Correndo para a frente com a camera girando (curva).
            Keys k;
            if (t > 0.3f && t < 4.6f) { k.forward = 1.0f; k.sprint = t > 1.5f; }
            k.cameraYaw = t > 0.8f ? (t - 0.8f) * 1.4f : 0.0f;
            return k; } },
        { "gira", 3.5f, [](float t) {
            // Parado, a camera vira 180 graus rapido e volta.
            Keys k;
            if (t > 0.5f && t < 0.9f) k.cameraYaw = (t - 0.5f) / 0.4f * Pi;
            else if (t >= 0.9f && t < 2.0f) k.cameraYaw = Pi;
            else if (t >= 2.0f && t < 2.3f) k.cameraYaw = Pi * (1.0f - (t - 2.0f) / 0.3f);
            return k; } },
        { "pulo", 3.5f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.0f) k.forward = 1.0f;
            k.jump = t > 1.2f && t < 1.22f;
            return k; } },
        { "pulinho", 3.5f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.0f) k.forward = 1.0f;
            k.jump = t > 1.2f && t < 1.22f;
            k.jumpSpeed = 2.2f;
            return k; } },
        { "pulo alto", 3.5f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.0f) k.forward = 1.0f;
            k.jump = t > 1.2f && t < 1.22f;
            k.jumpSpeed = 3.5f;
            return k; } },
        // O pulo instantaneo parado (24/10): sai ao apertar, sem o agachar de
        // preparo, na altura media.
        { "pulo instantaneo", 3.0f, [](float t) {
            Keys k;
            k.jump = t >= 1.2f && t < 1.22f;
            k.jumpSpeed = 3.0f;
            return k; } },
        { "pulo parado", 3.5f, [](float t) {
            Keys k;
            k.jump = t > 1.2f && t < 1.22f;
            return k; } },
        { "pulo sprint", 3.5f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.0f) { k.forward = 1.0f; k.sprint = true; }
            k.jump = t > 1.6f && t < 1.62f;
            return k; } },
        { "carrinho", 4.0f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.5f) { k.forward = 1.0f; k.sprint = true; }
            k.tackle = pulse(t, 1.4f);
            return k; } },
        { "carrinho dir", 4.0f, [](float t) {
            // Sprint girando a camera para a direita e o carrinho.
            Keys k;
            if (t > 0.3f && t < 3.5f) { k.forward = 1.0f; k.sprint = true; }
            k.cameraYaw = t > 1.0f ? -(std::min(t, 1.45f) - 1.0f) * 1.5f : 0.0f;
            k.tackle = pulse(t, 1.4f);
            return k; } },
        { "carrinho esq", 4.0f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.5f) { k.forward = 1.0f; k.sprint = true; }
            k.cameraYaw = t > 1.0f ? (std::min(t, 1.45f) - 1.0f) * 1.5f : 0.0f;
            k.tackle = pulse(t, 1.4f);
            return k; } },
        { "carrinho diag", 4.0f, [](float t) {
            // Correndo na diagonal para a direita (W + D), camera parada.
            Keys k;
            if (t > 0.3f && t < 3.5f) { k.forward = 1.0f; k.left = -1.0f; k.sprint = true; }
            k.tackle = pulse(t, 1.4f);
            return k; } },
        { "voadora", 4.0f, [](float t) {
            // Sprint, pulo e o carrinho no meio do voo.
            Keys k;
            if (t > 0.3f && t < 3.5f) { k.forward = 1.0f; k.sprint = true; }
            k.jump = t > 1.6f && t < 1.62f;
            k.tackle = pulse(t, 1.85f);
            return k; } },
        { "voadora tarde", 4.0f, [](float t) {
            // O carrinho quase pousando.
            Keys k;
            if (t > 0.3f && t < 3.5f) { k.forward = 1.0f; k.sprint = true; }
            k.jump = t > 1.6f && t < 1.62f;
            k.tackle = pulse(t, 2.45f);
            return k; } },
        { "preparo pulo", 3.0f, [](float t) {
            // Parado, segurando o espaco ate a carga cheia e soltando.
            Keys k;
            if (t > 0.8f && t < 1.6f) {
                const float x = std::clamp((t - 0.86f) / 0.49f, 0.0f, 1.0f);
                k.jumpCrouch = x * x * (3.0f - 2.0f * x);
            }
            k.jump = t >= 1.6f && t < 1.62f;
            k.jumpSpeed = 3.5f;
            return k; } },
        { "voadora parado", 4.0f, [](float t) {
            Keys k;
            k.jump = t > 1.0f && t < 1.02f;
            k.tackle = pulse(t, 1.3f);
            return k; } },
        { "carrinho trote", 4.0f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.5f) k.forward = 1.0f;
            k.tackle = pulse(t, 1.4f);
            return k; } },
        { "olhar", 4.0f, [](float t) {
            // Parado: a camera vai 40 graus devagar (so coluna e cabeca), depois
            // 120 graus rapido (a pelve vai junto no fim).
            Keys k;
            if (t > 0.4f && t < 1.4f) k.cameraYaw = (t - 0.4f) * 0.7f;
            else if (t >= 1.4f && t < 2.2f) k.cameraYaw = 0.7f;
            else if (t >= 2.2f && t < 2.6f) k.cameraYaw = 0.7f + (t - 2.2f) / 0.4f * 2.1f;
            else if (t >= 2.6f) k.cameraYaw = 2.8f;
            return k; } },
        // O sprint maximo reto (24/10).
        { "sprintmax", 4.5f, [](float t) {
            Keys k; if (t > 0.5f && t < 4.0f) { k.forward = 1.0f; k.sprint = k.maxSprint = true; } return k; } },
    };
}

// Os gestos com a bola para a revisao visual (rework de 06/10): parado, nas
// oito direcoes, passe, chutes pela forca, sola, bola alta, bola longe e
// andando. O pedido em 0,6 s.

void dumpFrame(FILE* file, const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose, float t,
    const DefaultLocomotionOutput3D& out) {
    std::fprintf(file, "F %.4f %d %d\n", t, out.airborne ? 5 : 0, 0);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const auto& collider = profile.links[i].collider;
        const Vec3 center = pose.linkPositions[i] + pose.linkOrientations[i].rotate(collider.localPosition);
        const Quaternion q = (pose.linkOrientations[i] * collider.localOrientation).normalized();
        if (ragdollColliderHasBox3D(collider)) {
            std::fprintf(file, "B %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w,
                collider.boxHalfExtents.x, collider.boxHalfExtents.y, collider.boxHalfExtents.z);
        } else {
            std::fprintf(file, "C %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w, ragdollCapsuleHalfSegment3D(collider),
                collider.radiusMeters, ragdollCapsuleRadiusAtPositiveX3D(collider));
        }
    }
}

// A sola: o canto mais baixo da caixa do pe.
float soleHeight(const RagdollLinkDefinition3D& link, Vec3 position, Quaternion orientation) {
    const auto& c = link.collider;
    if (!ragdollColliderHasBox3D(c)) return position.z;
    const Vec3 center = position + orientation.rotate(c.localPosition);
    const Quaternion q = (orientation * c.localOrientation).normalized();
    float lowest = 1e9f;
    for (int i = 0; i < 8; ++i) {
        const Vec3 corner { (i & 1 ? 1.0f : -1.0f) * c.boxHalfExtents.x, (i & 2 ? 1.0f : -1.0f) * c.boxHalfExtents.y,
            (i & 4 ? 1.0f : -1.0f) * c.boxHalfExtents.z };
        lowest = std::min(lowest, (center + q.rotate(corner)).z);
    }
    return lowest;
}

// Velocidade angular (rad/s, mundo) de a para b em dt.
Vec3 angularVelocity(Quaternion a, Quaternion b, float dt) {
    Quaternion d = (b * a.conjugate()).normalized();
    if (d.w < 0.0f) d = { -d.x, -d.y, -d.z, -d.w };
    const Vec3 v { d.x, d.y, d.z };
    const float n = v.length();
    if (n < 1e-7f) return {};
    return v * (2.0f * std::atan2(n, d.w) / n / dt);
}

float angleBetween(Quaternion a, Quaternion b) {
    const float d = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
    return 2.0f * std::acos(std::clamp(d, 0.0f, 1.0f));
}

} // namespace

// Os pontos de reference_capture.py (gait_metrics.py mede os dois do mesmo
// jeito): articulacoes pelas ancoras das juntas, pe por pontos na sola.
std::map<std::string, Vec3> landmarks(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose) {
    const auto index = [&](const char* id) {
        for (std::size_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == id) return i;
        return std::size_t { 0 };
    };
    const auto anchor = [&](const char* id) {
        const std::size_t i = index(id);
        const RagdollLinkDefinition3D& link = profile.links[i];
        const Vec3 local = link.modelOrientation.conjugate().rotate(
            link.inboundJoint.anchorModelPosition - link.modelPosition);
        return pose.linkPositions[i] + pose.linkOrientations[i].rotate(local);
    };
    const auto sole = [&](const char* id, float forward) {
        const std::size_t i = index(id);
        return pose.linkPositions[i] + pose.linkOrientations[i].rotate({ 0.24f * forward, 0.0f, -0.13f });
    };
    std::map<std::string, Vec3> out;
    const std::size_t head = index("Head");
    out["Hips"] = pose.linkPositions[0];
    out["Chest"] = anchor("UpperChest");
    out["Neck"] = anchor("Neck");
    out["Head"] = anchor("Head");
    out["HeadTop"] = out["Head"] + (pose.linkOrientations[head] * profile.links[head].modelOrientation.conjugate())
        .rotate({ 0.0f, 0.0f, 0.2f });
    for (const char* side : { "Left", "Right" }) {
        const std::string s(side);
        out[s + "Hip"] = anchor((s + "Thigh").c_str());
        out[s + "Knee"] = anchor((s + "Shin").c_str());
        out[s + "Ankle"] = anchor((s + "Foot").c_str());
        out[s + "Ball"] = sole((s + "Foot").c_str(), 0.7f);
        out[s + "Toe"] = sole((s + "Foot").c_str(), 1.0f);
        out[s + "Shoulder"] = anchor((s + "UpperArm").c_str());
        out[s + "Elbow"] = anchor((s + "Forearm").c_str());
        out[s + "Wrist"] = anchor((s + "Hand").c_str());
    }
    return out;
}

void writeCapture(const std::string& path, const char* name, const RagdollProfile3D& profile,
    const std::vector<std::map<std::string, Vec3>>& frames) {
    FILE* file = std::fopen(path.c_str(), "w");
    if (file == nullptr || frames.empty()) return;
    RagdollAnimationPose3D bind;
    buildRagdollLocalPose3D(profile, std::vector<Vec3>(profile.links.size()), bind);
    const auto rest = landmarks(profile, bind);
    const auto point = [&](Vec3 v) { std::fprintf(file, "[%.5f, %.5f, %.5f]", v.x, v.y, v.z); };
    std::fprintf(file, "{\"source\": \"%s\", \"fps\": %.1f, \"frames\": %zu, \"positions\": {", name, 1.0f / Dt,
        frames.size());
    bool firstKey = true;
    for (const auto& [key, value] : rest) {
        (void)value;
        std::fprintf(file, "%s\"%s\": [", firstKey ? "" : ", ", key.c_str());
        for (std::size_t f = 0; f < frames.size(); ++f) {
            if (f) std::fprintf(file, ", ");
            point(frames[f].at(key));
        }
        std::fprintf(file, "]");
        firstKey = false;
    }
    std::fprintf(file, "}, \"rest\": {");
    firstKey = true;
    for (const auto& [key, value] : rest) {
        std::fprintf(file, "%s\"%s\": ", firstKey ? "" : ", ", key.c_str());
        point(value);
        firstKey = false;
    }
    std::fprintf(file, "}}\n");
    std::fclose(file);
}

int main(int argc, char** argv) {
    std::string only, dumpPath, capturePath;
    bool gestures = false;
    float touchSeconds = 0.35f, shotSeconds = 0.55f;
    float captureFrom = 1.5f;
    float acceleration = 16.0f, deceleration = 22.0f, turnRate = 10.0f, brake = 30.0f;
    int every = 4;
    std::vector<std::pair<std::string, float>> tweaks;
    PlayerBody3D body;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto need = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--cenario") only = need();
        else if (arg == "--corpo") {
            const auto parsed = parseBody3D(need());
            if (!parsed) { std::fprintf(stderr, "--corpo cm,kg (170 a 200 cm, 60 a 100 kg)\n"); return 2; }
            body = *parsed;
        }
        else if (arg == "--acel") acceleration = std::strtof(need(), nullptr);
        else if (arg == "--desacel") deceleration = std::strtof(need(), nullptr);
        else if (arg == "--giro") turnRate = std::strtof(need(), nullptr);
        else if (arg == "--dump") dumpPath = need();
        else if (arg == "--gestos") gestures = true;
        else if (arg == "--tempo-manejo") touchSeconds = std::strtof(need(), nullptr);
        else if (arg == "--tempo-chute") shotSeconds = std::strtof(need(), nullptr);
        else if (arg == "--cada") every = std::max(1, std::atoi(need()));
        // Um ciclo inteiro (de um pouso do pe esquerdo ao seguinte) a partir
        // de --captura-de s, no formato de reference_capture.py.
        else if (arg == "--captura") capturePath = need();
        else if (arg == "--captura-de") captureFrom = std::strtof(need(), nullptr);
        else if (arg == "--ajuste") {
            const std::string pair = need();
            const auto at = pair.find('=');
            if (at == std::string::npos) { std::fprintf(stderr, "--ajuste nome=valor\n"); return 2; }
            tweaks.push_back({ pair.substr(0, at), std::strtof(pair.c_str() + at + 1, nullptr) });
        }
        else { std::fprintf(stderr, "argumento desconhecido: %s\n", arg.c_str()); return 2; }
    }
    const std::shared_ptr<const AthleteBundle3D> bundle = AthleteBundle3D::shared(MATTERENGINE_ASSETS_DIR, body);
    const RagdollProfile3D& profile = bundle->profile();
    if (!isBaseBody3D(body))
        std::printf("corpo %.2f m %.1f kg (escala %.4f, perfil %.1f kg)\n", body.heightMeters, body.massKg,
            bodyScale3D(body), profile.totalMassKg);
    // Os ossos visuais (07/10): a mao relaxada tem de ser a exportada.
    {
        RagdollVisualPose3D check;
        RagdollVisualPoseInput3D input;
        std::vector<Vec3> positions;
        std::vector<Quaternion> orientations;
        for (const auto& link : profile.links) {
            positions.push_back(link.modelPosition);
            orientations.push_back(link.modelOrientation);
        }
        input.linkPositions = positions;
        input.linkOrientations = orientations;
        input.handPose = { RagdollHandPose3D::Relaxed, RagdollHandPose3D::Relaxed };
        input.groundHeight = { -profile.standingRootHeightMeters, -profile.standingRootHeightMeters };
        const auto& rotations = check.update(bundle->character(), input);
        float worst = 0.0f;
        for (std::size_t k = 0; k < rotations.size(); ++k) {
            const Quaternion a = rotations[k], b = bundle->character().visualBones[k].restRotation;
            const float d = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
            worst = std::max(worst, 2.0f * std::acos(std::min(1.0f, d)) * 180.0f / Pi);
        }
        std::printf("ossos visuais: %zu (mao relaxada difere da exportada %.4f graus; dedos do pe na ligacao %.2f graus)\n",
            rotations.size(), worst, std::max(check.toeBend(0), check.toeBend(1)) * 180.0f / Pi);
    }
    // Os clipes do modo padrao (os do manifesto por cima dos do fisico), como
    // no laboratorio.
    const CharacterLocomotionAnimations3D animations = *bundle->assets().defaultClips;
    std::size_t feet[2] = { 0, 0 };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    std::printf("%-10s | %-17s | %-23s | %-27s | %-14s | %s\n", "cenario", "sola min/apoio max", "deslize apoio med/p95/max",
        "pico acel tronco/bracos (t)", "pico pernas", "giro/tick max | pelve-capsula max");
    std::vector<Scenario> all = gestures ? gestureScenarios(touchSeconds, shotSeconds) : scenarios();
    std::size_t shins[2] = { 0, 0 }, thighs[2] = { 0, 0 };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftShin") shins[0] = i;
        if (profile.links[i].id == "RightShin") shins[1] = i;
        if (profile.links[i].id == "LeftThigh") thighs[0] = i;
        if (profile.links[i].id == "RightThigh") thighs[1] = i;
    }
    // O pior de cada medida da bateria dos gestos (e o cenario).
    struct Worst { float value = -1e9f; std::string name; void take(float v, const char* n) { if (v > value) { value = v; name = n; } } };
    Worst worstLegPeak, worstLegTurn, worstKnee, worstSole, worstSupportSlip, worstCrossing, worstFlight, worstPop, worstStance;
    int gestureRuns = 0;
    bool dumpOpened = false;
    for (const Scenario& scenario : all) {
        if (!only.empty() && only != scenario.name
            && !(only.back() == '*' && std::string(scenario.name).rfind(only.substr(0, only.size() - 1), 0) == 0))
            continue;
        DefaultLocomotion3D locomotion;
        locomotion.reset(0.0f);
        DefaultLocomotionSettings3D& settings = locomotion.settings();
        float variety = 1.0f;
        for (const auto& [key, value] : tweaks) {
            if (key == "giro") settings.turnSeconds = value;
            else if (key == "mistura") settings.gaitBlendSeconds = value;
            else if (key == "parado") settings.idleBlendSeconds = value;
            else if (key == "curva") settings.curveLeanMaximum = value;
            else if (key == "filtrocurva") settings.pathTurnFilterSeconds = value;
            else if (key == "adianta") settings.pelvisCurveLeadMaximum = value;
            else if (key == "girar") settings.turnSeconds = value;
            else if (key == "frente") settings.forwardLeanShare = value;
            else if (key == "passomin") settings.minimumStrideScale = value;
            else if (key == "passomax") settings.maximumStrideScale = value;
            else if (key == "ar") settings.airBlendSeconds = value;
            else if (key == "ritmomin") settings.minimumPlayRate = value;
            else if (key == "ritmomax") settings.maximumPlayRate = value;
            else if (key == "adapta") settings.adaptiveTransitions = value > 0.5f;
            else if (key == "zona") settings.directionBlendDeadZone = value;
            else if (key == "suave") settings.smoothClipSampling = value > 0.5f;
            else if (key == "trava") settings.footLock = value > 0.5f;
            else if (key == "vida") variety = value;
            else if (key == "pecurva") settings.footCurveYawMaximum = value;
            else if (key == "pouso") settings.landingPrep = value > 0.5f;
            else if (key == "absorve") settings.landingAbsorb = value > 0.5f;
            else if (key == "zonacurva") settings.curveLeanDeadZone = value;
            else if (key == "carrinhoigual") settings.tackleEqualSlide = value > 0.5f;
            else if (key == "tronco") settings.trunkMotionAmount = value;
            else if (key == "amplitude") settings.gaitAmplitude = value > 0.5f;
            else if (key == "ampmax") settings.gaitAmplitudeMaximum = value;
            else if (key == "amplado") settings.sprintAmplitudeSide = value;
            // Teste de eixo: tudo zero menos um canal, na classe trote-frente.
            else if (key == "t_alt") settings.trunkMotions[0].heightCos2 = value;
            else if (key == "t_frente") settings.trunkMotions[0].pitchMean = value;
            else if (key == "t_cabfrente") settings.trunkMotions[0].headPitchMean = value;
            else if (key == "t_cablado") settings.trunkMotions[0].headRollCos1 = value;
            else if (key == "cima") settings.upperGaitBlendSeconds = value;
            else if (key == "cimaparado") settings.upperIdleBlendSeconds = value;
            else if (key == "parada") settings.settleStop = value > 0.5f;
            else if (key == "giroacel") settings.maximumTurnAcceleration = value;
            else if (key == "giromola") settings.turnSpringSeconds = value;
            else if (key == "giromax") settings.maximumTurnRateRadiansPerSecond = value;
            else { std::fprintf(stderr, "ajuste desconhecido: %s\n", key.c_str()); return 2; }
        }
        Vec3 position, velocity;
        bool grounded = true;
        // Os ossos visuais (07/10): os dedos do pe no chao (z 0) e a mao
        // (XMAOS: relaxada, aberta, fechada; senao a automatica).
        RagdollVisualPose3D visualPose;
        RagdollHandPose3D handPose = RagdollHandPose3D::Automatic;
        if (const char* hands = std::getenv("XMAOS")) {
            const std::string h = hands;
            handPose = h == "relaxada" ? RagdollHandPose3D::Relaxed : h == "aberta" ? RagdollHandPose3D::Open
                : h == "fechada" ? RagdollHandPose3D::Fist : RagdollHandPose3D::Automatic;
        }
        FILE* dump = nullptr;
        if (!dumpPath.empty()) {
            // (Um arquivo para todos os cenarios: o primeiro abre, os outros somam.)
            dump = std::fopen(dumpPath.c_str(), dumpOpened ? "a" : "w");
            dumpOpened = true;
            std::fprintf(dump, "S padrao %s\n", scenario.name);
            // (O corpo so fora do de hoje: o dump dele fica o de sempre.)
            if (!isBaseBody3D(body)) std::fprintf(dump, "corpo %.4f %.2f\n", body.heightMeters, body.massKg);
        }
        std::vector<std::vector<Vec3>> history;
        int captureState = 0;
        float previousCapturePhase = 0.0f;
        std::vector<std::map<std::string, Vec3>> captureFrames;
        std::vector<std::vector<Quaternion>> rotations;
        float soleMin = 1e9f, plantedMax = -1e9f, peak = 0.0f, peakTime = 0.0f, turnPeak = 0.0f, offsetPeak = 0.0f;
        float landTouch = -1.0f, landPeak = 0.0f, landLegPeak = 0.0f;
        bool tackleWas = false;
        Vec3 tackleFrom, tacklePrevious;
        float tackleEntry = 0.0f, tackleLow = 0.0f, tackleYaw = 0.0f, tackleTurn = 0.0f;
        std::string tackleName;
        // Inclinacao de curva (graus): a maior e o tempo acima de 3 graus.
        float curveLeanPeak = 0.0f, curveLeanSeconds = 0.0f;
        std::string landPeakLink;
        float legPeak = 0.0f;
        // Solavanco de rotacao: o pico da aceleracao angular da pelve e do
        // peito (graus/s2) - o "brusco" das trocas de clipe e de rumo.
        float pelvisSpin = 0.0f, chestSpin = 0.0f, pelvisSpinTime = 0.0f, chestSpinTime = 0.0f;
        // O rumo de cada pe apoiado em relacao ao rumo da pelve (graus, +
        // para a esquerda): a media do giro do pe na curva e o maior.
        std::array<float, 2> footTurnSum {}, footTurnPeak {};
        std::array<int, 2> footTurnCount {};
        // Balanco: a cabeca de lado (no referencial do rumo, em volta da
        // media) e a inclinacao do peito (graus da vertical), a partir de 0,5 s.
        std::vector<float> headSide, chestTilt;
        std::size_t headIndex = 0, chestIndex = 0;
        for (std::size_t i = 0; i < profile.links.size(); ++i) {
            if (profile.links[i].id == "Head") headIndex = i;
            if (profile.links[i].id == "UpperChest") chestIndex = i;
        }
        std::string peakLink;
        std::vector<float> slides;
        // O gesto: a bola de referencia e as medidas na janela dele.
        Vec3 nominalBall;
        const float gestureEnd = scenario.gestureAt >= 0.0f ? scenario.gestureAt + scenario.gesture.seconds + 0.05f : -1.0f;
        float gLegPeak = 0.0f, gLegTurn = 0.0f, gKneeMin = 10.0f, gSoleMin = 10.0f, gSupportSlip = 0.0f, gFlight = 0.0f,
              gFlightRun = 0.0f, gPop = 0.0f;
        int gCrossing = 0;
        // O pe de apoio: onde estava no comando e o quanto saiu dali (m, no
        // plano; parado ele nao sai); e o tranco por grupo (a perna do gesto,
        // a de apoio e o corpo de cima, m/s2).
        Vec3 gSupportStart;
        bool gSupportStartValid = false;
        float gSupportDrift = 0.0f, gPeakGestureLeg = 0.0f, gPeakSupportLeg = 0.0f, gPeakUpper = 0.0f;
        // Os pes parados (no plano, em relacao a pelve, no rumo): no comeco
        // (antes de andar) e no fim - o quanto ficaram fora do lugar.
        std::array<Vec3, 2> stanceStart {}, stanceEnd {};
        const int steps = static_cast<int>(scenario.seconds / Dt);
        for (int step = 0; step < steps; ++step) {
            const float t = step * Dt;
            const Keys keys = scenario.keys(t);
            // A capsula: velocidade definida, aceleracao alta (Unreal).
            Vec3 move { keys.forward * std::cos(keys.cameraYaw) - keys.left * std::sin(keys.cameraYaw),
                keys.forward * std::sin(keys.cameraYaw) + keys.left * std::cos(keys.cameraYaw), 0.0f };
            Vec3 wanted;
            if (move.lengthSquared() > 1e-4f) {
                move = move.normalized();
                const float relative = std::atan2(move.y, move.x) - keys.cameraYaw;
                wanted = move * (keys.maxSprint ? settings.maxSprintSpeed
                    : defaultLocomotionSpeed3D(settings, relative, keys.sprint));
            }
            if (grounded) {
                // O mesmo motor da capsula no modo padrao (JoltCharacter3D com
                // groundTurnRate): o rumo gira rapido, a velocidade vai aos
                // poucos; invertendo, freia antes.
                Vec3 planarVelocity { velocity.x, velocity.y, 0.0f };
                const float current = planarVelocity.length(), target = wanted.length();
                if (target < 1e-4f) {
                    planarVelocity = current > 1e-4f
                        ? planarVelocity * (std::max(0.0f, current - deceleration * Dt) / current) : Vec3 {};
                } else if (current < 0.3f) {
                    const Vec3 change = wanted - planarVelocity;
                    const float length = change.length(), limit = acceleration * Dt;
                    planarVelocity += length > limit ? change * (limit / length) : change;
                } else {
                    const Vec3 from = planarVelocity * (1.0f / current), to = wanted * (1.0f / target);
                    const float angle = std::atan2(from.x * to.y - from.y * to.x, dot(from, to));
                    if (std::abs(angle) > 120.0f * Pi / 180.0f) {
                        planarVelocity = planarVelocity * (std::max(0.0f, current - brake * Dt) / current);
                    } else {
                        const float turn = std::clamp(angle, -turnRate * Dt, turnRate * Dt);
                        const Vec3 direction { from.x * std::cos(turn) - from.y * std::sin(turn),
                            from.x * std::sin(turn) + from.y * std::cos(turn), 0.0f };
                        const float rate = target > current ? acceleration : deceleration;
                        planarVelocity = direction * (current + std::clamp(target - current, -rate * Dt, rate * Dt));
                    }
                }
                velocity.x = planarVelocity.x;
                velocity.y = planarVelocity.y;
                if (keys.jump) { velocity.z = keys.jumpSpeed; grounded = false; }
            } else {
                velocity.z -= 9.81f * Dt;
            }
            position += velocity * Dt;
            if (!grounded && position.z <= 0.0f && velocity.z < 0.0f) {
                position.z = 0.0f;
                velocity.z = 0.0f;
                grounded = true;
            }
            DefaultLocomotionInput3D input;
            input.requestedVelocityWorld = wanted;
            const bool gestureTick = scenario.gestureAt >= 0.0f && t >= scenario.gestureAt && t < scenario.gestureAt + 0.5f * Dt;
            // (XSEMGESTO: a mesma janela sem o pedido - a regua da passada sozinha.)
            if (gestureTick && !std::getenv("XSEMGESTO")) {
                input.gesture = scenario.gesture;
                // (XGESTOSEMENTE: o mesmo gesto com outro jeito sorteado.)
                if (const char* seed = std::getenv("XGESTOSEMENTE")) input.gesture.seed = static_cast<std::uint32_t>(std::atoi(seed));
                // (XGESTOFORCA: a barra, 0..1.)
                if (const char* power = std::getenv("XGESTOFORCA")) input.gesture.power = std::strtof(power, nullptr);
                // (XGESTOGIRO: o rumo da bola no chute, graus para a esquerda.)
                if (const char* turn = std::getenv("XGESTOGIRO")) input.gesture.turn = std::strtof(turn, nullptr) * Pi / 180.0f;
                input.gestureRequested = true;
                // A bola de referencia do balde, parada no mundo onde estava no
                // comando (so o desenho).
                const Vec3 local = gestureNominalBall3D(scenario.gesture);
                const float yaw = locomotion.output().valid ? locomotion.output().facingYawRadians : 0.0f;
                nominalBall = { position.x + std::cos(yaw) * local.x - std::sin(yaw) * local.y,
                    position.y + std::sin(yaw) * local.x + std::cos(yaw) * local.y, position.z + local.z };
            }
            input.tackleRequest = keys.tackle;
            // (A voadora: o Ctrl com Shift + W no ar.)
            input.tackleHeld = keys.tackle;
            input.jumpCrouch = keys.jumpCrouch;
            input.airTackleCombo = keys.sprint && keys.forward > 0.5f;
            input.feetPositionWorld = position;
            // O chao plano em z = 0 (o preparo do pouso mede a queda ate ele).
            input.groundAt = [](Vec3 at) {
                GroundProbeResult3D ground;
                ground.hasSurface = true;
                ground.pointWorld = { at.x, at.y, 0.0f };
                ground.normalWorld = { 0.0f, 0.0f, 1.0f };
                return ground;
            };
            input.velocityWorld = velocity;
            input.grounded = grounded;
            input.facingYawRadians = keys.cameraYaw;
            input.deltaTime = Dt;
            // A "vida" do movimento como no laboratorio (100%, semente do
            // primeiro boneco); --ajuste vida=0 desliga.
            input.motionVariety = variety;
            input.varietySeed = static_cast<std::uint32_t>(1ull * 0x9E3779B97F4A7C15ull >> 32);
            locomotion.update(profile, animations, input);
            const DefaultLocomotionOutput3D& out = locomotion.output();
            if (!out.valid) { std::fprintf(stderr, "pose invalida\n"); return 1; }
            if (std::getenv("XPROBEFEET")) {
                // Trocas de apoio de cada pe (a passada: balanco e apoio).
                static std::array<int, 2> was { -1, -1 };
                static std::array<float, 2> since { 0.0f, 0.0f };
                for (std::size_t side = 0; side < 2; ++side) {
                    const int now = out.footPlanted[side] ? 1 : 0;
                    if (was[side] >= 0 && now != was[side]) {
                        std::printf("  pe %zu %s em %.3f (fase anterior %.3f s) v %.2f\n", side, now ? "pousa" : "sai",
                            t, t - since[side], std::hypot(velocity.x, velocity.y));
                        since[side] = t;
                    }
                    was[side] = now;
                }
            }
            // Pouso: no ultimo tick no ar antes do toque, o peito (graus da
            // vertical, + para a frente do rumo) e o pe mais baixo a frente da
            // pelve ao longo do rumo (m).
            if (!grounded && velocity.z < 0.0f && position.z + velocity.z * Dt <= 0.0f && chestIndex != 0) {
                const Vec3 flat { velocity.x, velocity.y, 0.0f };
                const float speedFlat = flat.length();
                const Vec3 along = speedFlat > 1e-3f ? flat * (1.0f / speedFlat) : Vec3 { 1.0f, 0.0f, 0.0f };
                const Vec3 up = (out.pose.linkOrientations[chestIndex]
                    * profile.links[chestIndex].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                const float chest = std::atan2(dot(up, along), up.z) * 180.0f / Pi;
                const std::size_t low = out.pose.linkPositions[feet[0]].z < out.pose.linkPositions[feet[1]].z ? feet[0] : feet[1];
                const float ahead = dot(out.pose.linkPositions[low] - out.pose.linkPositions[0], along);
                std::printf("           pouso em %.2f s: v %.2f m/s, peito %+.1f graus do rumo, pe mais baixo %+.2f m a frente da pelve\n",
                    t, speedFlat, chest, ahead);
                landTouch = t;
            }
            // Carrinho: a capsula vai embaixo da pelve do clipe; no fim, com o
            // embalo dela.
            if (out.tackling && out.rootScripted) {
                if (!tackleWas) {
                    tackleFrom = position;
                    tackleEntry = std::hypot(velocity.x, velocity.y);
                    tackleLow = 0.0f;
                    const auto state = locomotion.tackle();
                    tackleName = state.clip != nullptr ? state.clip->id : "?";
                    tackleYaw = state.yaw;
                    tackleTurn = 0.0f;
                    tacklePrevious = out.tackleRootWorld;
                }
                const Vec3 step { out.tackleRootWorld.x - tacklePrevious.x, out.tackleRootWorld.y - tacklePrevious.y, 0.0f };
                tacklePrevious = out.tackleRootWorld;
                if (out.tackleRootWorld.z - position.z < 0.55f) tackleLow += step.length();
                const Vec3 forward = (out.rootOrientation * profile.links.front().modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                const float turn = std::remainder(std::atan2(forward.y, forward.x) - tackleYaw, 2.0f * Pi);
                if (std::abs(turn) > std::abs(tackleTurn)) tackleTurn = turn;
                position.x = out.tackleRootWorld.x;
                position.y = out.tackleRootWorld.y;
                velocity.x = out.tackleVelocity.x;
                velocity.y = out.tackleVelocity.y;
            }
            if (tackleWas && !(out.tackling && out.rootScripted))
                std::printf("           carrinho %s: entrada %.1f m/s, deslize (pelve baixa) %.2f m, total %.2f m, corpo virado %+.0f graus (%s)\n",
                    tackleName.c_str(), tackleEntry, tackleLow, std::hypot(position.x - tackleFrom.x, position.y - tackleFrom.y),
                    tackleTurn * 180.0f / Pi, tackleTurn > 0.0f ? "esquerda" : "direita");
            tackleWas = out.tackling && out.rootScripted;
            if (out.tackleEnded) {
                velocity.x = out.tackleVelocity.x;
                velocity.y = out.tackleVelocity.y;
            }
            if (std::getenv("XPROBETRACE") && step % 12 == 0)
                std::printf("  t %.2f rumo %+.2f camera %+.2f v %.2f carrinho %d fase %.2f\n", t, out.facingYawRadians,
                    keys.cameraYaw, std::hypot(velocity.x, velocity.y), out.tackling ? 1 : 0, out.cyclePhase);
            const RagdollAnimationPose3D& pose = out.pose;
            if (scenario.gestureAt >= 0.0f && ((t >= 0.15f && t < 0.15f + 0.5f * Dt) || step == steps - 1)) {
                const float c = std::cos(out.facingYawRadians), s = std::sin(out.facingYawRadians);
                for (std::size_t side = 0; side < 2; ++side) {
                    const Vec3 d = pose.linkPositions[feet[side]] - pose.linkPositions[0];
                    (step == steps - 1 ? stanceEnd : stanceStart)[side] = { c * d.x + s * d.y, -s * d.x + c * d.y, 0.0f };
                }
            }
            history.push_back(pose.linkPositions);
            rotations.push_back(pose.linkOrientations);
            if (std::getenv("XGESTORASTRO") && gestureEnd > 0.0f && t >= scenario.gestureAt - 0.05f && t <= gestureEnd) {
                const Vec3 l = pose.linkPositions[feet[0]], r = pose.linkPositions[feet[1]], p0 = pose.linkPositions[0];
                std::printf("  t %.3f pelve %.3f %.3f %.3f | E %.3f %.3f %.3f | D %.3f %.3f %.3f | contato %.2f %.2f fase %.3f\n",
                    t, p0.x - position.x, p0.y - position.y, p0.z, l.x - position.x, l.y - position.y, l.z,
                    r.x - position.x, r.y - position.y, r.z, out.footContact[0], out.footContact[1], out.cyclePhase);
            }
            if (gestureEnd > 0.0f && t >= scenario.gestureAt - 0.5f * Dt && t <= gestureEnd && history.size() >= 3) {
                const int g = scenario.gesture.side == 1 ? 1 : 0, support = 1 - g;
                if (std::getenv("XGESTOJUNTAS") && t <= scenario.gestureAt + 0.45f) {
                    const Vec3 th = out.coordinates[thighs[g]], sh = out.coordinates[shins[g]], ft = out.coordinates[feet[g]];
                    const Vec3 fp = pose.linkPositions[feet[g]];
                    std::printf("  J %.3f coxa %+.3f %+.3f %+.3f joelho %+.3f pe %+.3f %+.3f | pe %.3f %.3f %.3f | fase %.3f\n",
                        t - scenario.gestureAt, th.x, th.y, th.z, sh.x, ft.x, ft.y, fp.x - position.x, fp.y - position.y, fp.z,
                        out.cyclePhase);
                }
                const std::size_t k = history.size() - 1;
                // O tranco das pernas (aceleracao dos links, m/s2), o giro da
                // coxa e da canela (graus/s) e o salto de pose (m: a posicao
                // contra a prevista pela velocidade do tick anterior).
                for (const std::size_t i : { thighs[0], thighs[1], shins[0], shins[1], feet[0], feet[1] }) {
                    const Vec3 a = history[k - 2][i], b = history[k - 1][i], c = history[k][i];
                    const float accel = (c - b * 2.0f + a).length() / (Dt * Dt);
                    gLegPeak = std::max(gLegPeak, accel);
                    gPop = std::max(gPop, (c - b * 2.0f + a).length());
                    const float turn = angleBetween(rotations[k - 1][i], rotations[k][i]) / Dt * 180.0f / Pi;
                    if (std::getenv("XGIRO") && turn > std::atof(std::getenv("XGIRO")))
                        std::printf("  giro %s %.0f graus/s em %.3f s do gesto\n", profile.links[i].id.c_str(), turn,
                            t - scenario.gestureAt);
                    gLegTurn = std::max(gLegTurn, turn);
                    const bool gestureLink = i == thighs[g] || i == shins[g] || i == feet[g];
                    if (std::getenv("XGESTOPERNA") && accel > (gestureLink ? gPeakGestureLeg : gPeakSupportLeg))
                        std::printf("  perna %s %s %.0f m/s2 em %.3f s do gesto (fim %.3f)\n", gestureLink ? "gesto" : "apoio",
                            profile.links[i].id.c_str(), accel, t - scenario.gestureAt, gestureEnd - scenario.gestureAt);
                    (gestureLink ? gPeakGestureLeg : gPeakSupportLeg)
                        = std::max(gestureLink ? gPeakGestureLeg : gPeakSupportLeg, accel);
                }
                for (std::size_t i = 1; i < pose.linkPositions.size(); ++i) {
                    if (i == thighs[0] || i == thighs[1] || i == shins[0] || i == shins[1] || i == feet[0] || i == feet[1])
                        continue;
                    const Vec3 a = history[k - 2][i], b = history[k - 1][i], c = history[k][i];
                    const float accel = (c - b * 2.0f + a).length() / (Dt * Dt);
                    if (accel > gPeakUpper && std::getenv("XGESTOCIMA"))
                        std::printf("  cima %s %.0f m/s2 em %.3f s (gesto %.3f)\n", profile.links[i].id.c_str(), accel, t,
                            t - scenario.gestureAt);
                    gPeakUpper = std::max(gPeakUpper, accel);
                }
                {
                    const Vec3 foot = pose.linkPositions[feet[support]];
                    if (!gSupportStartValid) { gSupportStart = foot; gSupportStartValid = true; }
                    gSupportDrift = std::max(gSupportDrift, std::hypot(foot.x - gSupportStart.x, foot.y - gSupportStart.y));
                }
                for (int side = 0; side < 2; ++side)
                    gKneeMin = std::min(gKneeMin, out.coordinates[shins[side]].x * 180.0f / Pi);
                const float gestureSole = soleHeight(profile.links[feet[g]], pose.linkPositions[feet[g]],
                    pose.linkOrientations[feet[g]]) - position.z;
                gSoleMin = std::min(gSoleMin, gestureSole);
                const float supportSole = soleHeight(profile.links[feet[support]], pose.linkPositions[feet[support]],
                    pose.linkOrientations[feet[support]]) - position.z;
                if (supportSole < 0.02f) {
                    const Vec3 a = history[k - 1][feet[support]], b = history[k][feet[support]];
                    gSupportSlip = std::max(gSupportSlip, std::hypot(b.x - a.x, b.y - a.y) / Dt);
                }
                gFlightRun = gestureSole > 0.03f && supportSole > 0.03f ? gFlightRun + Dt : 0.0f;
                gFlight = std::max(gFlight, gFlightRun);
                // As pernas cruzando: o pe esquerdo a direita do direito (no rumo
                // da pelve) com os dois lado a lado.
                const float yaw = out.facingYawRadians;
                const Vec3 fwd { std::cos(yaw), std::sin(yaw), 0.0f }, lft { -fwd.y, fwd.x, 0.0f };
                const Vec3 apart = pose.linkPositions[feet[0]] - pose.linkPositions[feet[1]];
                if (dot(apart, lft) < 0.0f && std::abs(dot(apart, fwd)) < 0.20f) ++gCrossing;
            }
            for (std::size_t side = 0; side < 2; ++side) {
                const float sole = soleHeight(profile.links[feet[side]], pose.linkPositions[feet[side]],
                    pose.linkOrientations[feet[side]]) - position.z;
                if (grounded) soleMin = std::min(soleMin, sole);
                if (grounded && out.footContact[side] > 0.6f) {
                    if (std::getenv("XPROBEHIGH") && sole > 0.03f)
                        std::printf("  apoio alto %s %.3f m em %.3f s (contato %.2f, pouso %.2f afunda %.3f)\n", side == 0 ? "esq" : "dir",
                            sole, t, out.footContact[side], out.landingIntensity, out.landingSink);
                    plantedMax = std::max(plantedMax, sole);
                    if (history.size() >= 2) {
                        const Vec3 a = history[history.size() - 2][feet[side]], b = pose.linkPositions[feet[side]];
                        slides.push_back(std::hypot(b.x - a.x, b.y - a.y) / Dt);
                        if (std::getenv("XPROBESLIDE") && slides.back() > 3.0f)
                            std::printf("  desliza %s %.2f m/s em %.3f s (v %.2f fase %.2f contato %.2f)\n",
                                side == 0 ? "esq" : "dir", slides.back(), t, std::hypot(velocity.x, velocity.y),
                                out.cyclePhase, out.footContact[side]);
                    }
                }
            }
            if (history.size() >= 3 && step > 12) {
                const auto& p0 = history[history.size() - 3];
                const auto& p1 = history[history.size() - 2];
                const auto& p2 = history[history.size() - 1];
                for (std::size_t i = 0; i < p2.size(); ++i) {
                    const float a = (p2[i] - p1[i] * 2.0f + p0[i]).length() / (Dt * Dt);
                    const std::string& id = profile.links[i].id;
                    const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                        || id.find("Foot") != std::string::npos;
                    if (landTouch >= 0.0f && t > landTouch && t - landTouch < 0.6f) {
                        if (leg) landLegPeak = std::max(landLegPeak, a);
                        else if (a > landPeak) { landPeak = a; landPeakLink = id; }
                    }
                    if (leg) {
                        if (a > legPeak && std::getenv("XPROBELEG"))
                            std::printf("  perna %.0f m/s2 em %.3f s (%s)\n", a, t, id.c_str());
                        legPeak = std::max(legPeak, a);
                    }
                    else if (a > peak) { peak = a; peakTime = t; peakLink = id; }
                    {
                        const float turn = angleBetween(rotations[rotations.size() - 2][i], pose.linkOrientations[i]);
                        if (std::getenv("XPROBEGIRO") && turn > std::strtof(std::getenv("XPROBEGIRO"), nullptr) * Pi / 180.0f)
                            std::printf("           GIRO %.1f graus em %.3f s no %s\n", turn * 180.0f / Pi, t,
                                profile.links[i].id.c_str());
                        turnPeak = std::max(turnPeak, turn);
                    }
                }
            }
            if (rotations.size() >= 3 && step > 12) {
                const std::size_t k = rotations.size() - 1;
                for (std::size_t which = 0; which < 2; ++which) {
                    const std::size_t i = which == 0 ? 0 : chestIndex;
                    const Vec3 w1 = angularVelocity(rotations[k - 1][i], rotations[k][i], Dt);
                    const Vec3 w0 = angularVelocity(rotations[k - 2][i], rotations[k - 1][i], Dt);
                    const float alpha = (w1 - w0).length() / Dt * 180.0f / Pi;
                    float& spin = which == 0 ? pelvisSpin : chestSpin;
                    float& when = which == 0 ? pelvisSpinTime : chestSpinTime;
                    if (alpha > spin) { spin = alpha; when = t; }
                }
            }
            for (std::size_t side = 0; side < 2; ++side) {
                if (!grounded || out.footContact[side] < 0.6f || t < 0.5f) continue;
                const std::size_t f = feet[side];
                const Vec3 ahead = (pose.linkOrientations[f] * profile.links[f].modelOrientation.conjugate())
                    .rotate({ 1.0f, 0.0f, 0.0f });
                const float turn = std::remainder(std::atan2(ahead.y, ahead.x) - out.facingYawRadians, 2.0f * Pi)
                    * 180.0f / Pi;
                footTurnSum[side] += turn;
                ++footTurnCount[side];
                if (std::abs(turn) > std::abs(footTurnPeak[side])) footTurnPeak[side] = turn;
            }
            const Vec3 offset = pose.linkPositions[0] - position;
            offsetPeak = std::max(offsetPeak, std::hypot(offset.x, offset.y));
            if (t > 0.5f && headIndex != 0 && chestIndex != 0) {
                const float yawNow = keys.cameraYaw;
                const Vec3 leftAxis { -std::sin(yawNow), std::cos(yawNow), 0.0f };
                headSide.push_back(dot(pose.linkPositions[headIndex] - position, leftAxis));
                const Vec3 up = (pose.linkOrientations[chestIndex]
                    * profile.links[chestIndex].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                chestTilt.push_back(std::acos(std::clamp(up.z, -1.0f, 1.0f)) * 180.0f / Pi);
            }
            {
                RagdollVisualPoseInput3D visualInput;
                visualInput.linkPositions = pose.linkPositions;
                visualInput.linkOrientations = pose.linkOrientations;
                visualInput.handPose = { handPose, handPose };
                visualInput.toesFollowGround = !std::getenv("XSEMDEDOSPE");
                visualInput.deltaTime = Dt;
                visualPose.update(bundle->character(), visualInput);
            }
            if (const char* path = std::getenv("XPROBEPOSES")) {
                // Poses dos links (origem, orientacao xyzw) de XPROBEDE a
                // XPROBEATE, uma por linha (previa renderizada).
                const float from = std::getenv("XPROBEDE") ? std::strtof(std::getenv("XPROBEDE"), nullptr) : 0.0f;
                const float to = std::getenv("XPROBEATE") ? std::strtof(std::getenv("XPROBEATE"), nullptr) : 1e9f;
                if (t >= from && t <= to) {
                    if (FILE* file = std::fopen(path, "a")) {
                        std::fprintf(file, "{\"t\": %.4f, \"z\": %.4f, \"p\": [", t, position.z);
                        for (std::size_t i = 0; i < pose.linkPositions.size(); ++i)
                            std::fprintf(file, "%s[%.5f, %.5f, %.5f]", i ? ", " : "", pose.linkPositions[i].x,
                                pose.linkPositions[i].y, pose.linkPositions[i].z);
                        std::fprintf(file, "], \"q\": [");
                        for (std::size_t i = 0; i < pose.linkOrientations.size(); ++i)
                            std::fprintf(file, "%s[%.6f, %.6f, %.6f, %.6f]", i ? ", " : "", pose.linkOrientations[i].x,
                                pose.linkOrientations[i].y, pose.linkOrientations[i].z, pose.linkOrientations[i].w);
                        std::fprintf(file, "], \"v\": [");
                        const auto& visual = visualPose.rotations();
                        for (std::size_t i = 0; i < visual.size(); ++i)
                            std::fprintf(file, "%s[%.6f, %.6f, %.6f, %.6f]", i ? ", " : "", visual[i].x, visual[i].y,
                                visual[i].z, visual[i].w);
                        std::fprintf(file, "], \"dp\": [%.4f, %.4f], \"mao\": [%.3f, %.3f]}\n", visualPose.toeBend(0),
                            visualPose.toeBend(1), visualPose.handOpen(0), visualPose.handOpen(1));
                        std::fclose(file);
                    }
                }
            }
            curveLeanPeak = std::max(curveLeanPeak, std::abs(out.lean.x) * 180.0f / Pi);
            if (std::abs(out.lean.x) * 180.0f / Pi > 3.0f) curveLeanSeconds += Dt;
            if (std::getenv("XPROBETRACE") && step % (std::getenv("XPROBEEVERY") ? std::atoi(std::getenv("XPROBEEVERY")) : 6) == 0) {
                const Vec3 pelvisForward = (out.rootOrientation * profile.links.front().modelOrientation.conjugate())
                    .rotate({ 1.0f, 0.0f, 0.0f });
                const Vec3 up = (pose.linkOrientations[chestIndex]
                    * profile.links[chestIndex].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                std::printf("  t %.2f chao %d z %.2f vz %+.2f rumo %+.1f pelve %+.1f pelvez %.3f peito %.1f pouso %.2f afunda %.3f\n", t,
                    grounded ? 1 : 0, position.z, velocity.z, out.facingYawRadians * 180.0f / Pi,
                    std::atan2(pelvisForward.y, pelvisForward.x) * 180.0f / Pi, out.rootPosition.z - position.z,
                    std::acos(std::clamp(up.z, -1.0f, 1.0f)) * 180.0f / Pi, out.landingIntensity, out.landingSink);
                for (std::size_t side = 0; side < 2; ++side)
                    std::printf("      pe %zu sola %.3f contato %.2f\n", side, soleHeight(profile.links[feet[side]],
                        pose.linkPositions[feet[side]], pose.linkOrientations[feet[side]]) - position.z, out.footContact[side]);
            }
            if (!capturePath.empty() && captureState < 2) {
                const bool wrapped = out.cyclePhase < previousCapturePhase - 0.5f;
                previousCapturePhase = out.cyclePhase;
                if (captureState == 0 && wrapped && t >= captureFrom) captureState = 1;
                if (captureState == 1) {
                    captureFrames.push_back(landmarks(profile, pose));
                    if (wrapped && captureFrames.size() > 1) {
                        writeCapture(capturePath, scenario.name, profile, captureFrames);
                        captureState = 2;
                    }
                }
            }
            if (dump != nullptr && step % every == 0) {
                dumpFrame(dump, profile, pose, t, out);
                if (gestureEnd > 0.0f && t >= scenario.gestureAt - 0.5f * Dt && t <= gestureEnd)
                    std::fprintf(dump, "K %.4f %.4f %.4f\n", nominalBall.x, nominalBall.y, nominalBall.z);
                if (gestureTick) std::fprintf(dump, "T\n");
            }
        }
        if (dump != nullptr) std::fclose(dump);
        if (gestureEnd > 0.0f) {
            ++gestureRuns;
            const float stanceError = std::max((stanceEnd[0] - stanceStart[0]).length(), (stanceEnd[1] - stanceStart[1]).length());
            std::printf("           PES %-28s parado no fim: fora do lugar %.3f m (E %+.3f %+.3f -> %+.3f %+.3f | D %+.3f %+.3f -> %+.3f %+.3f)\n",
                scenario.name, stanceError, stanceStart[0].x, stanceStart[0].y, stanceEnd[0].x, stanceEnd[0].y,
                stanceStart[1].x, stanceStart[1].y, stanceEnd[1].x, stanceEnd[1].y);
            worstStance.take(stanceError, scenario.name);
            std::printf("           GESTO %-28s pernas %5.0f m/s2 giro %5.0f graus/s salto %.3f m | joelho min %+5.1f graus |"
                " pe do gesto %+.3f m | apoio escorrega %.2f m/s | cruzou %d | voo %.3f s | apoio saiu %.3f m |"
                " tranco gesto %.0f apoio %.0f cima %.0f\n", scenario.name, gLegPeak,
                gLegTurn, gPop, gKneeMin, gSoleMin, gSupportSlip, gCrossing, gFlight, gSupportDrift, gPeakGestureLeg,
                gPeakSupportLeg, gPeakUpper);
            worstLegPeak.take(gLegPeak, scenario.name);
            worstLegTurn.take(gLegTurn, scenario.name);
            worstPop.take(gPop, scenario.name);
            worstKnee.take(-gKneeMin, scenario.name);
            worstSole.take(-gSoleMin, scenario.name);
            worstSupportSlip.take(gSupportSlip, scenario.name);
            worstCrossing.take(static_cast<float>(gCrossing), scenario.name);
            worstFlight.take(gFlight, scenario.name);
        }
        if (landTouch >= 0.0f)
            std::printf("           no pouso (0,6 s): pico tronco/bracos %.1f m/s2 (%s), pernas %.1f m/s2\n", landPeak,
                landPeakLink.c_str(), landLegPeak);
        std::printf("           curva: inclinacao max %.1f graus, %.2f s acima de 3 graus\n", curveLeanPeak, curveLeanSeconds);
        std::sort(slides.begin(), slides.end());
        float mean = 0.0f;
        for (float v : slides) mean += v;
        if (!slides.empty()) mean /= static_cast<float>(slides.size());
        const float p95 = slides.empty() ? 0.0f : slides[static_cast<std::size_t>(0.95f * (slides.size() - 1))];
        const float maximum = slides.empty() ? 0.0f : slides.back();
        std::printf("%-10s | %+6.3f / %+6.3f m | %5.2f / %5.2f / %5.2f m/s | %6.1f (%.2f s, %-12s) | %6.1f m/s2 | %5.1f graus | %.2f m\n",
            scenario.name, soleMin, plantedMax, mean, p95, maximum, peak, peakTime, peakLink.c_str(), legPeak,
            turnPeak * 180.0f / Pi, offsetPeak);
        {
            float mean = 0.0f, rms = 0.0f, tiltMean = 0.0f, tiltMax = 0.0f;
            for (float v : headSide) mean += v;
            if (!headSide.empty()) mean /= static_cast<float>(headSide.size());
            for (float v : headSide) rms += (v - mean) * (v - mean);
            if (!headSide.empty()) rms = std::sqrt(rms / static_cast<float>(headSide.size()));
            for (float v : chestTilt) { tiltMean += v; tiltMax = std::max(tiltMax, v); }
            if (!chestTilt.empty()) tiltMean /= static_cast<float>(chestTilt.size());
            std::printf("           balanco: cabeca de lado %.1f cm rms | peito inclinado media %.1f max %.1f graus\n",
                rms * 100.0f, tiltMean, tiltMax);
            std::printf("           solavanco de giro: pelve %.0f graus/s2 (%.2f s) | peito %.0f graus/s2 (%.2f s)\n",
                pelvisSpin, pelvisSpinTime, chestSpin, chestSpinTime);
            std::printf("           pe apoiado x pelve: esq media %+.1f max %+.1f | dir media %+.1f max %+.1f graus\n",
                footTurnCount[0] ? footTurnSum[0] / footTurnCount[0] : 0.0f, footTurnPeak[0],
                footTurnCount[1] ? footTurnSum[1] / footTurnCount[1] : 0.0f, footTurnPeak[1]);
        }
    }
    if (gestureRuns > 0) {
        std::printf("\nGESTOS (%d): pior tranco das pernas %.0f m/s2 (%s) | giro %.0f graus/s (%s) | salto %.3f m (%s)\n",
            gestureRuns, worstLegPeak.value, worstLegPeak.name.c_str(), worstLegTurn.value, worstLegTurn.name.c_str(),
            worstPop.value, worstPop.name.c_str());
        std::printf("        joelho min %+.1f graus (%s) | pe do gesto mais baixo %+.3f m (%s) | apoio escorrega %.2f m/s (%s)\n",
            -worstKnee.value, worstKnee.name.c_str(), -worstSole.value, worstSole.name.c_str(), worstSupportSlip.value,
            worstSupportSlip.name.c_str());
        std::printf("        cruzou %.0f ticks (%s) | voo %.3f s (%s)\n", worstCrossing.value, worstCrossing.name.c_str(),
            worstFlight.value, worstFlight.name.c_str());
        std::printf("        pe fora do lugar parado no fim %.3f m (%s)\n", worstStance.value, worstStance.name.c_str());
    }
    return 0;
}
