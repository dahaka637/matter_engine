// MatterBallPreview: o visualizador de toque na bola na linha de comando
// (para analise e correcoes). Roda a mesma simulacao do Workbench
// (BallTouchPreview3D), imprime o resumo do toque e grava os quadros no
// formato do XPOSEDUMP (com a bola, o alvo do pe e o pe de apoio), que
// tools/analysis/ball_touch_preview.py desenha.
//
//   MatterBallPreview [--marcha parado] [--fase 0.25] [--bola 0.55,0.12,0.11]
//       [--parada] [--acao drible|chute|nada] [--direcao 0] [--forca 0.3] [--fator 0.5]
//       [--altura 0] [--efeito 0] [--area] [--variacao] [--antes 0.5]
//       [--depois 1.6] [--dump arquivo] [--cada 1] [--trace] [--cheia] [--livre]
//       [--varrer direcao|fase|ambos]
//
// Marchas: parado, trote, sprint, lado-esq, lado-dir, recuo, sprint-esq,
// sprint-dir. Direcao em graus em relacao ao corpo (+ esquerda). A bola: x a
// frente e y a esquerda da pelve, z a altura do centro.

#include "Engine/Character/BallTouchPreview3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

// O caminho de cada pe depois do pedido (m, no plano): o quanto o movimento
// andou (laco em volta da bola, voltar longe, passos a mais).
std::array<float, 2> footPaths(const BallTouchPreviewResult3D& r) {
    std::array<float, 2> length {};
    for (std::size_t i = r.requestFrame + 1; i < r.frames.size(); ++i)
        for (std::size_t side = 0; side < 2; ++side) {
            const Vec3 d = r.frames[i].feet[side] - r.frames[i - 1].feet[side];
            length[side] += std::hypot(d.x, d.y);
        }
    return length;
}

const char* surfaceName(BallTouchSurface3D surface) {
    switch (surface) {
    case BallTouchSurface3D::Laces: return "peito do pe";
    case BallTouchSurface3D::Inside: return "parte de dentro";
    case BallTouchSurface3D::Outside: return "parte de fora";
    case BallTouchSurface3D::Sole: return "sola";
    case BallTouchSurface3D::Heel: return "calcanhar";
    }
    return "?";
}

const char* missName(int reason) {
    switch (reason) {
    case 0: return "-";
    case 1: return "bola longe";
    case 2: return "o pe nao chegou a tempo";
    case 3: return "acabou sem toque";
    }
    return "?";
}

const char* phaseName(BallControlPhase3D phase) {
    switch (phase) {
    case BallControlPhase3D::Idle: return "livre";
    case BallControlPhase3D::Waiting: return "esperando";
    case BallControlPhase3D::Windup: return "armando";
    case BallControlPhase3D::Reaching: return "indo";
    case BallControlPhase3D::FollowThrough: return "acompanhando";
    }
    return "?";
}

void dumpFrame(FILE* file, const RagdollProfile3D& profile, const BallTouchPreviewFrame3D& frame) {
    std::fprintf(file, "F %.4f %d %d\n", frame.timeSeconds, frame.locomotionState, static_cast<int>(frame.phase));
    for (std::size_t i = 0; i < profile.links.size() && i < frame.linkPositions.size(); ++i) {
        const auto& collider = profile.links[i].collider;
        const Vec3 center = frame.linkPositions[i] + frame.linkOrientations[i].rotate(collider.localPosition);
        const Quaternion q = (frame.linkOrientations[i] * collider.localOrientation).normalized();
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
    if (frame.ballPresent)
        std::fprintf(file, "K %.4f %.4f %.4f %.3f %.3f %.3f\n", frame.ballPosition.x, frame.ballPosition.y,
            frame.ballPosition.z, frame.ballVelocity.x, frame.ballVelocity.y, frame.ballVelocity.z);
    for (int side = 0; side < 2; ++side)
        std::fprintf(file, "E %d %.4f %.4f %.4f %d\n", side, frame.feet[side].x, frame.feet[side].y,
            frame.feet[side].z, frame.planted[side] ? 1 : 0);
    // Pernas: a origem do link fisico (V) e a da pose alvo (W) - o quanto o
    // corpo segue o alvo.
    for (std::size_t i = 0; i < profile.links.size() && i < frame.linkPositions.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id.find("Thigh") == std::string::npos && id.find("Shin") == std::string::npos
            && id.find("Foot") == std::string::npos && id != "Pelvis") continue;
        std::fprintf(file, "V %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f\n", id.c_str(), frame.linkPositions[i].x,
            frame.linkPositions[i].y, frame.linkPositions[i].z, frame.linkOrientations[i].x, frame.linkOrientations[i].y,
            frame.linkOrientations[i].z, frame.linkOrientations[i].w);
        if (i < frame.targetLinkPositions.size() && i < frame.targetLinkOrientations.size()) {
            const Quaternion q = frame.targetLinkOrientations[i];
            std::fprintf(file, "W %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f\n", id.c_str(), frame.targetLinkPositions[i].x,
                frame.targetLinkPositions[i].y, frame.targetLinkPositions[i].z, q.x, q.y, q.z, q.w);
        }
    }
    if (frame.footGoal.active)
        std::fprintf(file, "Q %d %.4f %.4f %.4f %.3f\n", frame.footGoal.side, frame.footGoal.position.x,
            frame.footGoal.position.y, frame.footGoal.position.z, frame.footGoal.weight);
    if (frame.plant.active)
        std::fprintf(file, "L %d %.4f %.4f %.4f %.3f\n", frame.plant.side, frame.plant.position.x,
            frame.plant.position.y, frame.plant.position.z, frame.plant.weight);
    if (frame.touched) std::fprintf(file, "T\n");
}

std::string label(const BallTouchPreviewSettings3D& s) {
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), "%s %s fase %.2f bola (%.2f %.2f %.2f) dir %+.0f forca %.2f%s%s",
        ballPreviewGaitKey(s.gait), s.kind == BallActionKind3D::Shot ? "chute" : "drible", s.requestPhase,
        s.ballOffset.x, s.ballOffset.y, s.ballOffset.z, s.directionDegrees, s.power,
        s.areaTouch ? " area" : "", s.ballRolling ? "" : " parada");
    return buffer;
}

// Medidas do gesto (analise da beleza e da precisao, 04/10): a menor
// distancia entre as canelas (joelho-tornozelo) do pedido ate o fim da acao
// (embananar as pernas), a velocidade do pe que toca na direcao pedida e a
// folga dele no instante do toque (o pe de verdade na bola), a pelve em
// relacao a bola nessa direcao (+ atras dela) e o joelho que toca (graus;
// 180 esticado).
struct ActionMetrics {
    bool valid = false;
    float shinMinimum = 9.0f;
    float footSpeed = 0.0f;
    float gap = 0.0f;
    float pelvisBehind = 0.0f;
    float knee = 0.0f;
    float plantAside = 0.0f;
};

Vec3 jointWorld(const RagdollProfile3D& profile, const BallTouchPreviewFrame3D& frame, std::size_t link) {
    const auto& def = profile.links[link];
    const Vec3 local = def.modelOrientation.conjugate().rotate(def.inboundJoint.anchorModelPosition - def.modelPosition);
    return frame.linkPositions[link] + frame.linkOrientations[link].rotate(local);
}

float segmentDistance(Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
    float best = 1e9f;
    for (int i = 0; i <= 10; ++i) {
        const Vec3 p = a + (b - a) * (i / 10.0f);
        for (int j = 0; j <= 10; ++j) best = std::min(best, (p - (c + (d - c) * (j / 10.0f))).length());
    }
    return best;
}

ActionMetrics actionMetrics(const RagdollProfile3D& profile, const BallTouchPreviewResult3D& r) {
    ActionMetrics m;
    std::size_t shin[2] {}, foot[2] {}, thigh[2] {};
    const char* names[2] = { "Left", "Right" };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        for (int s = 0; s < 2; ++s) {
            if (profile.links[i].id == std::string(names[s]) + "Shin") shin[s] = i;
            if (profile.links[i].id == std::string(names[s]) + "Foot") foot[s] = i;
            if (profile.links[i].id == std::string(names[s]) + "Thigh") thigh[s] = i;
        }
    }
    if (!shin[0] || !shin[1] || !foot[0] || !foot[1] || !thigh[0] || !thigh[1] || r.frames.empty()) return m;
    // Do pedido ate a acao acabar (o manejo livre de novo) ou 1,2 s.
    std::size_t end = r.frames.size();
    for (std::size_t i = r.requestFrame + 1; i < r.frames.size(); ++i) {
        if (r.frames[i].timeSeconds > 1.2f) { end = i; break; }
        if (r.frames[i].phase == BallControlPhase3D::Idle && r.frames[i].timeSeconds > 0.1f
            && (r.touchFrame < 0 || static_cast<int>(i) > r.touchFrame)) { end = i; break; }
    }
    for (std::size_t i = r.requestFrame; i < end; ++i) {
        const auto& f = r.frames[i];
        m.shinMinimum = std::min(m.shinMinimum, segmentDistance(jointWorld(profile, f, shin[0]),
            jointWorld(profile, f, foot[0]), jointWorld(profile, f, shin[1]), jointWorld(profile, f, foot[1])));
    }
    m.valid = true;
    if (r.touchFrame < 1 || r.side < 0 || static_cast<std::size_t>(r.touchFrame) + 1 >= r.frames.size()) return m;
    const std::size_t t = static_cast<std::size_t>(r.touchFrame);
    const int k = r.side;
    const auto& f = r.frames[t];
    const Vec3 dir { r.directionWorld.x, r.directionWorld.y, 0.0f };
    const Vec3 velocity = (r.frames[t].linkPositions[foot[k]] - r.frames[t - 1].linkPositions[foot[k]])
        * (1.0f / std::max(1e-4f, r.frames[t].timeSeconds - r.frames[t - 1].timeSeconds));
    m.footSpeed = dot(velocity, dir);
    m.gap = f.footGap;
    const Vec3 ball = r.frames[t - 1].ballPresent ? r.frames[t - 1].ballPosition : f.ballPosition;
    const Vec3 pelvis = f.linkPositions.front();
    m.pelvisBehind = dot(Vec3 { ball.x - pelvis.x, ball.y - pelvis.y, 0.0f }, dir);
    const Vec3 hip = jointWorld(profile, f, thigh[k]), knee = jointWorld(profile, f, shin[k]), ankle = jointWorld(profile, f, foot[k]);
    const Vec3 a = (hip - knee).normalized(), b = (ankle - knee).normalized();
    m.knee = std::acos(std::clamp(dot(a, b), -1.0f, 1.0f)) * 57.2958f;
    const Vec3 left { -dir.y, dir.x, 0.0f };
    const Vec3 plant = jointWorld(profile, f, foot[1 - k]);
    m.plantAside = std::abs(dot(Vec3 { plant.x - ball.x, plant.y - ball.y, 0.0f }, left));
    return m;
}

void printSummary(const BallTouchPreviewSettings3D& s, const BallTouchPreviewResult3D& r) {
    std::printf("== %s\n", label(s).c_str());
    if (!r.valid) {
        std::printf("   falhou: %s\n", r.error.c_str());
        return;
    }
    const auto& request = r.frames[std::min(r.requestFrame, r.frames.size() - 1)];
    const float clipFrame = r.cyclePhaseAtRequest * r.gaitClipSeconds * r.gaitClipFramesPerSecond;
    std::printf("   pedido: fase %.3f (quadro %.0f de %s), pes plantados E %d D %d\n", r.cyclePhaseAtRequest,
        clipFrame, r.gaitClipId.c_str(), request.planted[0] ? 1 : 0, request.planted[1] ? 1 : 0);
    if (r.touchFrame >= 0) {
        const Vec3 v = r.ballVelocityAfter;
        const float speed = v.length();
        const float elevation = std::atan2(v.z, std::hypot(v.x, v.y)) * 57.2958f;
        const float yaw = std::atan2(v.y, v.x) * 57.2958f;
        std::printf("   toque: %.3f s depois, pe %s, %s; bola %.2f m/s rumo %+.0f graus elevacao %.1f graus, giro %.1f rad/s\n",
            r.touchDelaySeconds, r.side == 0 ? "esquerdo" : r.side == 1 ? "direito" : "?", surfaceName(r.surface),
            speed, yaw, elevation, r.ballSpinAfter.length());
    } else {
        std::printf("   sem toque: %s\n", missName(r.missReason));
    }
    // A sequencia de etapas do manejo depois do pedido.
    std::printf("   etapas:");
    BallControlPhase3D last = BallControlPhase3D::Idle;
    bool first = true;
    for (std::size_t i = r.requestFrame; i < r.frames.size(); ++i) {
        const auto& f = r.frames[i];
        if (first || f.phase != last) {
            std::printf(" %s@%.2f(pe %d)", phaseName(f.phase), f.timeSeconds, f.side);
            last = f.phase;
            first = false;
        }
    }
    std::printf("\n");
    if (r.fell) std::printf("   CAIU\n");
    std::printf("   simulado %.2f s em %.2f s\n", r.simulatedSeconds, r.wallSeconds);
}

void printMetrics(const ActionMetrics& m) {
    if (!m.valid) return;
    std::printf("   gesto: canelas min %.2f m | no toque: pe %.1f m/s, folga %.3f m, pelve %.2f m atras da bola, joelho %.0f graus, apoio %.2f m de lado\n",
        m.shinMinimum, m.footSpeed, m.gap, m.pelvisBehind, m.knee, m.plantAside);
}

void printTrace(const BallTouchPreviewResult3D& r, int every) {
    for (std::size_t i = 0; i < r.frames.size(); i += static_cast<std::size_t>(std::max(1, every))) {
        const auto& f = r.frames[i];
        std::printf("   t %+.3f %-12s pe %2d gap %.3f peso %.2f ritmo %.2f | pes E (%.2f %.2f %.2f)%s D (%.2f %.2f %.2f)%s",
            f.timeSeconds, phaseName(f.phase), f.side, f.footGap, f.footGoal.weight, f.gaitRate,
            f.feet[0].x, f.feet[0].y, f.feet[0].z, f.planted[0] ? "*" : " ",
            f.feet[1].x, f.feet[1].y, f.feet[1].z, f.planted[1] ? "*" : " ");
        std::printf(" | fase E %d%s%s D %d%s%s", f.footPhase[0], f.touching[0] ? "t" : "", f.supporting[0] ? "s" : "",
            f.footPhase[1], f.touching[1] ? "t" : "", f.supporting[1] ? "s" : "");
        if (f.footGoal.active)
            std::printf(" | alvo (%.2f %.2f %.2f)", f.footGoal.position.x, f.footGoal.position.y, f.footGoal.position.z);
        if (f.ballPresent)
            std::printf(" | bola (%.2f %.2f %.2f)", f.ballPosition.x, f.ballPosition.y, f.ballPosition.z);
        std::printf("%s\n", f.touched ? " TOQUE" : "");
    }
}

bool parseVec3(const char* text, Vec3& value) {
    return std::sscanf(text, "%f,%f,%f", &value.x, &value.y, &value.z) == 3;
}

} // namespace

int main(int argc, char** argv) {
    BallTouchPreviewSettings3D settings;
    std::string dumpPath;
    std::string sweep;
    int every = 1;
    bool trace = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const char* next = i + 1 < argc ? argv[i + 1] : nullptr;
        auto need = [&]() -> const char* {
            if (next == nullptr) {
                std::fprintf(stderr, "falta o valor de %s\n", arg.c_str());
                std::exit(2);
            }
            ++i;
            return next;
        };
        if (arg == "--marcha") {
            const std::string key = need();
            if (!parseBallPreviewGait(key, settings.gait)) {
                std::fprintf(stderr, "marcha desconhecida: %s\n", key.c_str());
                return 2;
            }
        } else if (arg == "--fase") settings.requestPhase = std::strtof(need(), nullptr);
        else if (arg == "--bola") {
            if (!parseVec3(need(), settings.ballOffset)) {
                std::fprintf(stderr, "--bola x,y,z\n");
                return 2;
            }
        } else if (arg == "--parada") settings.ballRolling = false;
        else if (arg == "--bolav") {
            Vec3 v;
            if (std::sscanf(need(), "%f,%f", &v.x, &v.y) != 2) {
                std::fprintf(stderr, "--bolav vx,vy (referencial do corpo)\n");
                return 2;
            }
            settings.ballVelocity = v;
            settings.ballVelocitySet = true;
        }
        else if (arg == "--acao") {
            const std::string kind = need();
            settings.kind = kind == "chute" ? BallActionKind3D::Shot
                : kind == "nada" ? BallActionKind3D::None : BallActionKind3D::Dribble;
        } else if (arg == "--direcao") settings.directionDegrees = std::strtof(need(), nullptr);
        else if (arg == "--forca") settings.power = std::strtof(need(), nullptr);
        else if (arg == "--fator") settings.contactTimingFactor = std::strtof(need(), nullptr);
        else if (arg == "--altura") settings.shotLift = std::strtof(need(), nullptr);
        else if (arg == "--efeito") settings.shotCurve = std::strtof(need(), nullptr);
        else if (arg == "--area") settings.areaTouch = true;
        else if (arg == "--variacao") settings.motionVariety = true;
        else if (arg == "--antes") settings.recordBeforeSeconds = std::strtof(need(), nullptr);
        else if (arg == "--depois") settings.recordAfterSeconds = std::strtof(need(), nullptr);
        else if (arg == "--dump") dumpPath = need();
        else if (arg == "--cada") every = std::atoi(need());
        else if (arg == "--trace") trace = true;
        else if (arg == "--varrer") sweep = need();
        else if (arg == "-h" || arg == "--help") {
            std::printf("veja o comentario no topo de tools/BallTouchPreview.cpp\n");
            return 0;
        } else {
            std::fprintf(stderr, "argumento desconhecido: %s\n", arg.c_str());
            return 2;
        }
    }

    if (settings.kind == BallActionKind3D::Shot)
        settings.directionDegrees = std::clamp(settings.directionDegrees, -90.0f, 90.0f);
    try {
        const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_ASSETS_DIR);
        BallTouchPreview3D preview(assets.character, assets.clips);
        if (!sweep.empty()) {
            // Grade: direcoes de 45 em 45 graus e/ou quatro fases.
            std::vector<float> directions { settings.directionDegrees };
            std::vector<float> phases { settings.requestPhase };
            if (sweep == "direcao" || sweep == "ambos") {
                // O chute vai para a frente: ate 90 graus para cada lado.
                directions = settings.kind == BallActionKind3D::Shot ? std::vector<float> { 0, 45, 90, -90, -45 }
                    : std::vector<float> { 0, 45, 90, 135, 180, -135, -90, -45 };
            }
            if (sweep == "fase" || sweep == "ambos") phases = { 0.0f, 0.25f, 0.5f, 0.75f };
            std::printf("%-6s %-6s | %-7s %-4s %-16s %-7s %-6s | caminho E/D | canela pe-v folga pelve joelho | %s\n", "dir", "fase",
                "toque", "pe", "superficie", "v bola", "rumo", "falha");
            for (const float phase : phases) {
                for (const float direction : directions) {
                    BallTouchPreviewSettings3D s = settings;
                    s.requestPhase = phase;
                    s.directionDegrees = direction;
                    const BallTouchPreviewResult3D r = preview.run(s);
                    if (!r.valid) {
                        std::printf("%+6.0f %6.2f | falhou: %s\n", direction, phase, r.error.c_str());
                        continue;
                    }
                    const auto path = footPaths(r);
                    const ActionMetrics m = actionMetrics(preview.profile(), r);
                    if (r.touchFrame >= 0) {
                        const Vec3 v = r.ballVelocityAfter;
                        std::printf("%+6.0f %6.2f | %6.3fs %-4s %-16s %5.2f   %+5.0f  | %.2f/%.2f m | %.2f %5.1f %.3f %5.2f %4.0f | -%s\n",
                            direction, phase, r.touchDelaySeconds, r.side == 0 ? "E" : "D", surfaceName(r.surface), v.length(),
                            std::atan2(v.y, v.x) * 57.2958f, path[0], path[1], m.shinMinimum, m.footSpeed, m.gap,
                            m.pelvisBehind, m.knee, r.fell ? " CAIU" : "");
                    } else {
                        std::printf("%+6.0f %6.2f | %-7s %-4s %-16s %-7s %-6s | %.2f/%.2f m | %.2f | %s%s\n", direction, phase, "-",
                            "-", "-", "-", "-", path[0], path[1], m.shinMinimum, missName(r.missReason), r.fell ? " CAIU" : "");
                    }
                    std::fflush(stdout);
                }
            }
            return 0;
        }
        const BallTouchPreviewResult3D result = preview.run(settings);
        printSummary(settings, result);
        if (result.valid) printMetrics(actionMetrics(preview.profile(), result));
        if (trace && result.valid) printTrace(result, every);
        if (!dumpPath.empty() && result.valid) {
            FILE* file = std::fopen(dumpPath.c_str(), "w");
            if (file == nullptr) {
                std::fprintf(stderr, "nao abriu %s\n", dumpPath.c_str());
                return 1;
            }
            std::fprintf(file, "S %s\n", label(settings).c_str());
            std::fprintf(file, "I pedido %zu toque %d pe %d superficie %d falha %d rumo %.4f direcao %.4f %.4f "
                "pelve %.4f %.4f %.4f bola %.4f %.4f %.4f\n", result.requestFrame, result.touchFrame, result.side,
                static_cast<int>(result.surface), result.missReason, result.facingYawRadians, result.directionWorld.x,
                result.directionWorld.y, result.pelvisAtRequest.x, result.pelvisAtRequest.y, result.pelvisAtRequest.z,
                result.ballAtRequest.x, result.ballAtRequest.y, result.ballAtRequest.z);
            for (std::size_t i = 0; i < result.frames.size(); i += static_cast<std::size_t>(std::max(1, every)))
                dumpFrame(file, preview.profile(), result.frames[i]);
            // O quadro do toque sempre entra.
            if (result.touchFrame >= 0 && every > 1 && result.touchFrame % every != 0)
                dumpFrame(file, preview.profile(), result.frames[static_cast<std::size_t>(result.touchFrame)]);
            std::fclose(file);
            std::printf("   quadros em %s\n", dumpPath.c_str());
        }
        return result.valid ? 0 : 1;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "erro: %s\n", error.what());
        return 1;
    }
}
