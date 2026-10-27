// Os gestos de bola (Engine/Gesture, 24/10): a perna de dois ossos no
// esqueleto do jogador (o tornozelo cai onde o catalogo pede, o joelho para o
// lado certo), o tocador sem salto (a pose e a velocidade continuas, a volta
// termina na locomocao) e a escolha dos baldes.
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"
#include "Engine/Gesture/GestureCatalog3D.hpp"
#include "Engine/Gesture/GestureChoice3D.hpp"
#include "Engine/Gesture/GesturePlayer3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace MatterEngine;

namespace {

int g_checks = 0;

void expect(bool condition, const std::string& what) {
    ++g_checks;
    if (!condition) throw std::runtime_error(what);
}

std::size_t linkIndex(const RagdollProfile3D& profile, const char* id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return i;
    throw std::runtime_error(std::string("link ausente: ") + id);
}

// O tornozelo e o joelho (no referencial da pelve) com a perna nesta pose e o
// resto em repouso.
struct LegPoint { Vec3 ankle; Vec3 knee; Vec3 toe; };
LegPoint legPoint(const RagdollProfile3D& profile, int side, const GestureLegPose3D& leg) {
    const char* thighs[2] = { "LeftThigh", "RightThigh" };
    const char* shins[2] = { "LeftShin", "RightShin" };
    const char* feet[2] = { "LeftFoot", "RightFoot" };
    const std::size_t thigh = linkIndex(profile, thighs[side]), shin = linkIndex(profile, shins[side]),
                      foot = linkIndex(profile, feet[side]);
    std::vector<Vec3> coordinates(profile.links.size());
    coordinates[thigh] = leg.thigh();
    coordinates[shin] = { leg.knee(), 0.0f, 0.0f };
    coordinates[foot] = leg.foot();
    RagdollAnimationPose3D pose;
    buildRagdollLocalPose3D(profile, coordinates, pose);
    const RagdollLinkDefinition3D& pelvis = profile.links.front();
    const auto anchorOf = [&](std::size_t index) {
        const RagdollLinkDefinition3D& link = profile.links[index];
        const Vec3 local = link.modelOrientation.conjugate().rotate(link.inboundJoint.anchorModelPosition - link.modelPosition);
        const Vec3 world = pose.linkPositions[index] + pose.linkOrientations[index].rotate(local);
        return pelvis.modelOrientation.conjugate().rotate(world - pose.linkPositions.front());
    };
    LegPoint point;
    point.ankle = anchorOf(foot);
    point.knee = anchorOf(shin);
    // A ponta do pe: a frente do pe em repouso (x do modelo) girada.
    point.toe = (pose.linkOrientations[foot] * profile.links[foot].modelOrientation.conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
    return point;
}

void legTests(const RagdollProfile3D& profile) {
    const GestureRig3D rig = defaultGestureRig3D(profile);
    expect(rig.valid, "o esqueleto dos gestos sai do perfil");
    for (int side = 0; side < 2; ++side) {
        const GestureLegRig3D& leg = rig.legs[static_cast<std::size_t>(side)];
        expect(leg.thigh > 0.3f && leg.thigh < 0.6f && leg.shin > 0.3f && leg.shin < 0.6f, "coxa e canela plausiveis");
        expect(leg.ankleHeight > 0.05f && leg.ankleHeight < 0.2f, "tornozelo sobre a sola");
        expect((side == 0 ? leg.hip.y : -leg.hip.y) > 0.05f, "o quadril do lado certo");
        // O tornozelo cai onde se pede (alcancavel), para varios alvos.
        const float out = side == 0 ? 1.0f : -1.0f;
        const Vec3 targets[] = { { 0.0f, 0.09f * out, -0.82f }, { 0.25f, 0.15f * out, -0.70f },
            { -0.20f, 0.12f * out, -0.72f }, { 0.10f, 0.40f * out, -0.68f }, { 0.55f, 0.05f * out, -0.20f },
            { 0.30f, -0.02f * out, -0.65f } };
        for (const Vec3& target : targets) {
            for (const float twist : { 0.0f, 0.5f, -0.3f }) {
                const GestureLegPose3D pose = gestureLegPose3D(leg, side, target, twist, 0.3f, 0.1f);
                const LegPoint point = legPoint(profile, side, pose);
                const float error = (point.ankle - target).length();
                char what[200];
                std::snprintf(what, sizeof what, "perna %d: o tornozelo no alvo (%.2f %.2f %.2f) erro %.3f m (giro %.1f)",
                    side, target.x, target.y, target.z, error, twist);
                expect(error < 0.01f, what);
                // O joelho dobra para a frente (com a perna embaixo do corpo) e o
                // giro para fora leva a ponta do pe para fora.
                if (target.x < 0.4f) {
                    const Vec3 mid = leg.hip + (target - leg.hip) * 0.5f;
                    std::snprintf(what, sizeof what, "perna %d: o joelho a frente da linha quadril-tornozelo (%.3f)", side,
                        point.knee.x - mid.x);
                    expect(point.knee.x - mid.x > -0.005f, what);
                }
            }
        }
        const GestureLegPose3D straight = gestureLegPose3D(leg, side, { 0.0f, leg.hip.y, -0.83f }, 0.0f, 0.0f, 0.0f);
        const GestureLegPose3D outward = gestureLegPose3D(leg, side, { 0.0f, leg.hip.y, -0.83f }, 0.5f, 0.0f, 0.0f);
        const float toeOut = legPoint(profile, side, outward).toe.y * out, toeStraight = legPoint(profile, side, straight).toe.y * out;
        char what[160];
        std::snprintf(what, sizeof what, "perna %d: girar para fora leva a ponta do pe para fora (%.2f > %.2f)", side, toeOut,
            toeStraight);
        expect(toeOut > toeStraight + 0.2f, what);
        // A ponta para baixo com o tornozelo plantar.
        const GestureLegPose3D pointed = gestureLegPose3D(leg, side, { 0.0f, leg.hip.y, -0.80f }, 0.0f, 0.6f, 0.0f);
        expect(legPoint(profile, side, pointed).toe.z < -0.4f, "plantar leva a ponta para baixo");
    }
}

void playerTests(const RagdollProfile3D& profile) {
    const GestureRig3D rig = defaultGestureRig3D(profile);
    constexpr float dt240 = 1.0f / 240.0f;
    for (const GestureKind3D kind : { GestureKind3D::Touch, GestureKind3D::Shot }) {
        for (int reach = 0; reach < 3; ++reach) {
            GestureSpec3D spec;
            spec.kind = kind;
            spec.side = reach % 2;
            spec.reach = reach;
            spec.depth = reach;
            spec.move = reach == 1 ? GestureMove3D::In : GestureMove3D::Out;
            spec.power = 0.8f;
            spec.seconds = kind == GestureKind3D::Shot ? 0.55f : 0.35f;
            spec.seed = 7u + static_cast<std::uint32_t>(reach);
            GestureLegPose3D start, live;
            for (std::size_t i = 0; i < 7; ++i) start.value[i] = live.value[i] = 0.05f * static_cast<float>(i);
            // Rodado em dois passos: um salto de velocidade de verdade fica do
            // mesmo tamanho com o passo menor; a aceleracao (continua) cai junto.
            struct Run { float jump = 0.0f, rateJump = 0.0f, lastError = 1.0f, seconds = 0.0f; };
            const auto run = [&](float dt) {
                GesturePlayer3D player;
                player.start(spec, rig, start, {});
                GestureLegPose3D previous = start, previousRate {};
                Run result;
                int ticks = 0;
                while (player.active() && ticks < 20000) {
                    const GestureFrame3D frame = player.update(dt, live);
                    ++ticks;
                    if (!frame.active) break;
                    GestureLegPose3D rate;
                    for (std::size_t i = 0; i < 7; ++i) {
                        rate.value[i] = (frame.leg.value[i] - previous.value[i]) / dt;
                        result.jump = std::max(result.jump, std::abs(frame.leg.value[i] - previous.value[i]));
                        if (ticks > 1) result.rateJump = std::max(result.rateJump, std::abs(rate.value[i] - previousRate.value[i]));
                    }
                    previous = frame.leg;
                    previousRate = rate;
                    result.lastError = 0.0f;
                    for (std::size_t i = 0; i < 7; ++i)
                        result.lastError = std::max(result.lastError, std::abs(frame.leg.value[i] - live.value[i]));
                }
                result.seconds = static_cast<float>(ticks) * dt;
                return result;
            };
            const Run coarse = run(1.0f / 240.0f), fine = run(1.0f / 960.0f);
            const char* name = kind == GestureKind3D::Shot ? "chute" : "toque";
            char what[200];
            std::snprintf(what, sizeof what, "tocador (%s %d): sem salto de pose (%.3f rad num tick a 240 Hz, %.3f a 960 Hz)",
                name, reach, coarse.jump, fine.jump);
            expect(fine.jump < 0.5f * coarse.jump + 0.002f, what);
            std::snprintf(what, sizeof what, "tocador (%s %d): a junta mais rapida a %.1f rad/s", name, reach,
                coarse.jump * 240.0f);
            expect(coarse.jump * 240.0f < (kind == GestureKind3D::Shot ? 34.0f : 22.0f), what);
            std::snprintf(what, sizeof what, "tocador (%s %d): velocidade continua (salto %.2f rad/s a 240 Hz, %.2f a 960 Hz)",
                name, reach, coarse.rateJump, fine.rateJump);
            expect(fine.rateJump < 0.5f * coarse.rateJump + 0.05f, what);
            std::snprintf(what, sizeof what, "tocador (%s %d): termina na pose da locomocao (%.4f rad) em %.3f s", name, reach,
                coarse.lastError, coarse.seconds);
            expect(coarse.lastError < 0.01f && std::abs(coarse.seconds - spec.seconds) < 0.02f, what);
            // O que o usuario pediu (24/10): a perna passa pela bola andando,
            // num movimento so (nunca para nela), e o chute arma para tras
            // antes (a pegada de impulso, pela barra).
            GesturePlayer3D player;
            player.start(spec, rig, start, {});
            const GestureKeys3D keys = player.keys();
            const float hipX = legPoint(profile, spec.side, start).ankle.x;
            float behind = 1e9f, before = 0.0f, at = 0.0f, fastest = 0.0f;
            GestureLegPose3D previous = start;
            for (float t = dt240; t < keys.duration; t += dt240) {
                const GestureFrame3D frame = player.update(dt240, live);
                if (!frame.active) break;
                float speed = 0.0f;
                for (std::size_t i = 0; i < 4; ++i)
                    speed += (frame.leg.value[i] - previous.value[i]) * (frame.leg.value[i] - previous.value[i]);
                speed = std::sqrt(speed) / dt240;
                previous = frame.leg;
                fastest = std::max(fastest, speed);
                if (std::abs(t - keys.contactTime) < 0.5f * dt240) at = speed;
                if (t < keys.contactTime - 0.02f) before = std::max(before, speed);
                if (t <= keys.windupTime + 0.5f * dt240) behind = std::min(behind, legPoint(profile, spec.side, frame.leg).ankle.x);
            }
            std::snprintf(what, sizeof what, "tocador (%s %d): passa pela bola andando (%.1f rad/s no contato, pico %.1f)", name,
                reach, at, fastest);
            expect(at > (kind == GestureKind3D::Shot ? 6.0f : 2.0f), what);
            if (kind == GestureKind3D::Shot) {
                std::snprintf(what, sizeof what, "tocador (chute %d): arma para tras (o tornozelo %.2f m atras de onde estava)",
                    reach, hipX - behind);
                expect(keys.windupTime > 0.0f && hipX - behind > 0.15f, what);
            }
            // Outro gesto na outra perna: este termina em 0,1 s, sem salto, na
            // pose da locomocao.
            {
                GesturePlayer3D cut;
                cut.start(spec, rig, start, {});
                GestureLegPose3D last = start;
                float jump = 0.0f, elapsed = 0.0f;
                bool ended = false;
                for (int tick = 0; tick < 2000; ++tick) {
                    if (tick == 30) cut.finishSoon(0.1f);
                    const GestureFrame3D frame = cut.update(dt240, live);
                    elapsed += dt240;
                    if (!frame.active) { ended = true; break; }
                    for (std::size_t i = 0; i < 7; ++i) jump = std::max(jump, std::abs(frame.leg.value[i] - last.value[i]));
                    last = frame.leg;
                }
                float error = 0.0f;
                for (std::size_t i = 0; i < 7; ++i) error = std::max(error, std::abs(last.value[i] - live.value[i]));
                std::snprintf(what, sizeof what,
                    "tocador (%s %d): terminado antes, acaba em %.3f s na locomocao (%.3f rad), sem salto (%.3f rad/tick)", name,
                    reach, elapsed, error, jump);
                expect(ended && std::abs(elapsed - (30.0f * dt240 + 0.1f)) < 0.01f && error < 0.02f
                        && jump < (kind == GestureKind3D::Shot ? 34.0f : 22.0f) * dt240, what);
            }
        }
    }
}

void choiceTests() {
    const GestureReach3D reach;
    const GestureChoice3D left = chooseGesture3D(GestureKind3D::Touch, { 0.10f, 0.20f, 0.11f }, { 0.0f, -1.0f, 0.0f }, -1, reach);
    expect(left.inReach && left.spec.side == 0 && left.spec.reach == 0 && left.spec.depth == 1
            && left.spec.move == GestureMove3D::In,
        "bola a esquerda indo para a direita: perna esquerda, curto, no meio, para dentro");
    const GestureChoice3D right = chooseGesture3D(GestureKind3D::Touch, { 0.40f, -0.50f, 0.11f }, { 0.0f, -1.0f, 0.0f }, -1, reach);
    expect(right.inReach && right.spec.side == 1 && right.spec.reach == 2 && right.spec.depth == 2
            && right.spec.move == GestureMove3D::Out,
        "bola longe a direita a frente indo para a direita: perna direita, longe, a frente, para fora");
    const GestureChoice3D back = chooseGesture3D(GestureKind3D::Touch, { -0.20f, 0.30f, 0.11f }, { 1.0f, 0.0f, 0.0f }, -1, reach);
    expect(back.inReach && back.spec.depth == 0 && back.spec.reach == 1 && back.spec.move == GestureMove3D::In,
        "bola atras trazida para a frente: atras, medio, para dentro");
    const GestureChoice3D middle = chooseGesture3D(GestureKind3D::Touch, { 0.30f, 0.0f, 0.11f }, { 1.0f, 0.0f, 0.0f }, 1, reach);
    expect(middle.inReach && middle.spec.side == 1, "bola no meio: a perna livre");
    expect(!chooseGesture3D(GestureKind3D::Shot, { 1.5f, 0.0f, 0.11f }, { 1.0f, 0.0f, 0.0f }, -1, reach).inReach,
        "bola longe: fora do alcance");
    // Conduzindo: a bola a frente um pouco a esquerda vai a esquerda; em
    // sequencia, a perna da vez (a direita), no alcance curto.
    const GestureChoice3D single = chooseGesture3D(GestureKind3D::Touch, { 0.40f, 0.12f, 0.11f }, { 1.0f, 0.0f, 0.0f }, -1, reach);
    const GestureChoice3D next = chooseGesture3D(GestureKind3D::Touch, { 0.40f, 0.12f, 0.11f }, { 1.0f, 0.0f, 0.0f }, -1, reach, 1);
    expect(single.spec.side == 0 && next.spec.side == 1 && next.spec.reach == 0,
        "conduzindo: em sequencia as pernas se alternam (a bola a frente, perto do meio)");
    expect(chooseGesture3D(GestureKind3D::Touch, { 0.40f, 0.40f, 0.11f }, { 1.0f, 0.0f, 0.0f }, -1, reach, 1).spec.side == 0,
        "conduzindo: a bola longe de um lado fica com a perna daquele lado");
    expect(!chooseGesture3D(GestureKind3D::Touch, { 0.3f, 0.1f, 1.2f }, { 1.0f, 0.0f, 0.0f }, -1, reach).inReach,
        "bola alta: fora do alcance");
    // A bola alta (24/10): de pouco abaixo do joelho ate o teto, o gesto alto;
    // atras dele, o do meio.
    const GestureChoice3D high = chooseGesture3D(GestureKind3D::Touch, { -0.15f, 0.2f, 0.5f }, { 1.0f, 0.0f, 0.0f }, -1, reach);
    expect(high.inReach && high.spec.high && high.spec.depth == 1, "bola na altura do joelho: o gesto alto, atras vira o meio");
    expect(!chooseGesture3D(GestureKind3D::Touch, { 0.3f, 0.1f, 0.3f }, { 1.0f, 0.0f, 0.0f }, -1, reach).spec.high,
        "bola abaixo do joelho: o gesto rente");
    // (O teto quase na linha da cintura: 24/10, pedido do usuario.)
    const GestureChoice3D waist = chooseGesture3D(GestureKind3D::Touch, { 0.3f, 0.1f, 0.9f }, { 1.0f, 0.0f, 0.0f }, -1, reach);
    expect(waist.inReach && waist.spec.high, "bola a 0,9 m: no alcance, o gesto alto");
    expect(!chooseGesture3D(GestureKind3D::Touch, { 0.3f, 0.1f, 1.0f }, { 1.0f, 0.0f, 0.0f }, -1, reach).inReach,
        "bola acima da cintura: fora do alcance");
    // A recepcao: o meio mais largo vai a perna preferida (parado, o pe da frente).
    const GestureChoice3D received
        = chooseGesture3D(GestureKind3D::Touch, { 0.5f, -0.12f, 0.11f }, { 1.0f, 0.0f, 0.0f }, 0, reach, -1, true);
    expect(received.spec.receive && received.spec.side == 0
            && chooseGesture3D(GestureKind3D::Touch, { 0.5f, -0.12f, 0.11f }, { 1.0f, 0.0f, 0.0f }, 0, reach).spec.side == 1,
        "recepcao: a bola perto do meio vai ao pe preferido; o toque, ao do lado dela");
}

// A perna nos limites das juntas (como a locomocao a mostra).
GestureLegPose3D limitedLeg(const RagdollProfile3D& profile, int side, GestureLegPose3D leg) {
    const char* names[3][2] = { { "LeftThigh", "RightThigh" }, { "LeftShin", "RightShin" }, { "LeftFoot", "RightFoot" } };
    const std::size_t offsets[3] = { 0, 3, 4 }, counts[3] = { 3, 1, 3 };
    for (std::size_t part = 0; part < 3; ++part) {
        const RagdollLinkDefinition3D& link = profile.links[linkIndex(profile, names[part][side])];
        for (std::size_t axis = 0; axis < counts[part]; ++axis) {
            const RagdollAxisDefinition3D& limit = link.inboundJoint.axes[axis];
            float& value = leg.value[offsets[part] + axis];
            value = limit.enabled ? std::clamp(value, limit.minimumRadians, limit.maximumRadians) : 0.0f;
        }
    }
    return leg;
}

// A bola alta e a recepcao (24/10): gestos prontos - a bola so escolhe o
// balde e a superficie (o pe nunca vai ate ela: ele via as pernas malucas). O
// contato na bola de referencia do balde (nos limites das juntas), o pe da
// bola alta na altura do joelho, o pe subindo antes, a mesma bola mexida dentro
// do balde dando o mesmo gesto e a recepcao encostando na bola de referencia,
// sem entrar, e cedendo no rumo da superficie.
void highAndReceiveTests(const RagdollProfile3D& profile) {
    GestureRig3D rig = defaultGestureRig3D(profile);
    rig.pelvisHeight = 0.914f;
    const GestureReach3D reach;
    struct Case { const char* name; GestureKind3D kind; bool receive; Vec3 ball, incoming; float near; };
    const Case cases[] = {
        { "toque alto a esquerda", GestureKind3D::Touch, false, { 0.32f, 0.20f, 0.60f }, {}, 0.16f },
        { "toque alto a direita", GestureKind3D::Touch, false, { 0.25f, -0.25f, 0.50f }, {}, 0.16f },
        { "voleio", GestureKind3D::Shot, false, { 0.35f, -0.15f, 0.55f }, {}, 0.18f },
        { "recepcao de sola", GestureKind3D::Touch, true, { 0.50f, 0.10f, 0.11f }, { -6.0f, 0.0f, 0.0f }, 0.21f },
        { "recepcao de lado", GestureKind3D::Touch, true, { 0.25f, 0.45f, 0.11f }, { 0.0f, -6.0f, 0.0f }, 0.21f },
        { "recepcao alta", GestureKind3D::Touch, true, { 0.50f, 0.08f, 0.50f }, { -5.0f, 0.0f, 0.0f }, 0.21f },
    };
    const GestureLegRig3D& leftLeg = rig.legs[0];
    const float knee = leftLeg.ankleHeight + leftLeg.shin;
    for (const Case& c : cases) {
        const auto keysFor = [&](Vec3 ball, GestureSpec3D* used) {
            const GestureChoice3D choice = chooseGesture3D(c.kind, ball, { 1.0f, 0.0f, 0.0f }, -1, reach, -1, c.receive);
            expect(choice.inReach, std::string(c.name) + ": no alcance");
            GestureSpec3D spec = choice.spec;
            spec.incoming = c.incoming;
            spec.power = 0.3f;
            spec.seconds = c.kind == GestureKind3D::Shot ? 0.58f : 0.45f;
            if (used != nullptr) *used = spec;
            return gestureKeys3D(spec, rig);
        };
        GestureSpec3D spec;
        const GestureKeys3D keys = keysFor(c.ball, &spec);
        const int side = spec.side;
        const GestureBodyPeaks3D& b = keys.body;
        const auto inBody = [&](const GestureLegPose3D& leg) {
            const Vec3 ankle = legPoint(profile, side, limitedLeg(profile, side, leg)).ankle;
            return ankle + Vec3 { b.forward, b.left, rig.pelvisHeight - b.drop };
        };
        const Vec3 contact = inBody(keys.contact);
        char what[220];
        // (A mesma bola mexida dentro do balde: o mesmo gesto.)
        const GestureKeys3D moved = keysFor(c.ball + Vec3 { 0.02f, 0.0f, spec.high ? 0.10f : 0.0f }, nullptr);
        bool same = true;
        for (std::size_t i = 0; i < keys.contact.value.size(); ++i) same = same && keys.contact.value[i] == moved.contact.value[i];
        expect(same, std::string(c.name) + ": a bola mexida dentro do balde da o mesmo gesto (o pe nao segue a bola)");
        // A bola de referencia do balde (a alta, na altura do joelho).
        Vec3 nominal = gestureNominalBall3D(spec);
        if (spec.high) nominal.z = knee + 0.05f - 0.06f;
        const float error = (contact - nominal).length();
        std::snprintf(what, sizeof what, "%s: o tornozelo do contato %s (%.2f %.2f %.2f, referencia %.2f %.2f %.2f: %.3f m)",
            c.name, c.receive && !spec.high ? "encosta na referencia sem entrar" : "perto da referencia", contact.x, contact.y,
            contact.z, nominal.x, nominal.y, nominal.z, error);
        expect(c.receive && !spec.high ? error > 0.13f && error < c.near : spec.high && c.receive ? contact.z < knee + 0.10f
                                                                                                    : error < c.near, what);
        if (c.kind == GestureKind3D::Touch) {
            expect(keys.liftValid, std::string(c.name) + ": o pe sobe antes de ir a bola");
            const Vec3 lift = inBody(keys.lift);
            std::snprintf(what, sizeof what, "%s: a subida acima do chao e atras do contato (z %.2f, x %.2f contra %.2f)", c.name,
                lift.z, lift.x, contact.x);
            expect(lift.z > 0.2f && lift.x < contact.x, what);
        }
        if (c.receive) {
            // Cede no rumo da superficie: a sola e o peito do pe para tras (o do
            // peito descendo), a parte de fora para dentro.
            const Vec3 give = inBody(keys.follow) - contact;
            const float outward = side == 0 ? give.y : -give.y;
            const float withOut = (side == 0 ? 1.0f : -1.0f) * c.incoming.y / std::max(1e-6f, c.incoming.length());
            const bool outside = !spec.high && !(spec.reach == 0 && spec.depth == 2) && withOut < -0.4f;
            std::snprintf(what, sizeof what, "%s: cede no rumo da superficie (x %.3f, fora %.3f, z %.3f m)", c.name, give.x,
                outward, give.z);
            expect(outside ? outward < -0.04f : give.x < -0.04f && (!spec.high || give.z < -0.10f), what);
        }
    }
    // A altura do joelho: a bola mais alta nao leva o pe acima dele.
    GestureSpec3D tall = chooseGesture3D(GestureKind3D::Touch, { 0.30f, 0.20f, 0.90f }, { 1.0f, 0.0f, 0.0f }, -1, reach).spec;
    tall.seconds = 0.45f;
    const Vec3 tallContact = legPoint(profile, 0, limitedLeg(profile, 0, gestureKeys3D(tall, rig).contact)).ankle;
    char what[160];
    std::snprintf(what, sizeof what, "bola a 0,90 m: o tornozelo ate a altura do joelho (%.2f, joelho %.2f)",
        tallContact.z + rig.pelvisHeight, knee);
    expect(tallContact.z + rig.pelvisHeight < knee + 0.10f, what);
}

// O peito e a cabeca (24/10): gestos so do corpo - a perna fica na
// locomocao; o cabeceio arma o tronco antes de bater, acena mais na bola para
// baixo e leva a cabeca ao rumo da bola.
void bodyPartTests(const RagdollProfile3D& profile) {
    GestureRig3D rig = defaultGestureRig3D(profile);
    GestureSpec3D chest;
    chest.part = GesturePart3D::Chest;
    chest.seconds = 0.45f;
    const GestureKeys3D trap = gestureKeys3D(chest, rig);
    expect(trap.legWeight == 0.0f && trap.body.spinePitch < 0.0f, "peito: so o corpo, o peito cede para tras");
    GesturePlayer3D player;
    player.start(chest, rig, GestureLegPose3D {}, GestureLegPose3D {});
    expect(player.active() && !player.movesLeg(), "peito: o gesto nao mexe na perna");
    GestureSpec3D head;
    head.part = GesturePart3D::Head;
    head.kind = GestureKind3D::Shot;
    head.power = 1.0f;
    head.turn = 0.5f;
    head.seconds = 0.58f;
    head.lift = -1.0f;
    const GestureKeys3D down = gestureKeys3D(head, rig);
    head.lift = 1.0f;
    const GestureKeys3D up = gestureKeys3D(head, rig);
    expect(down.legWeight == 0.0f && down.windupTime > 0.0f && down.contactTime > down.windupTime
            && down.body.windupBack > 0.0f && down.body.spinePitch > 0.0f,
        "cabeca: arma o tronco para tras e bate a frente, so o corpo");
    expect(down.body.headYaw > 0.0f && down.body.headPitch > up.body.headPitch,
        "cabeca: vai ao rumo da bola (a esquerda) e acena mais na bola para baixo");
}

} // namespace

int main() {
    try {
        const RagdollCharacter3D character
            = loadRagdollCharacter3D(std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/character.json");
        legTests(character.profile);
        playerTests(character.profile);
        choiceTests();
        highAndReceiveTests(character.profile);
        bodyPartTests(character.profile);
    } catch (const std::exception& error) {
        std::cerr << "gestos: " << error.what() << "\n";
        return 1;
    }
    std::cout << "gestos: " << g_checks << " checagens, tudo certo\n";
    return 0;
}
