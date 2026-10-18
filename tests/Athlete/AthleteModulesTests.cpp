// Os testes dos modulos novos do atleta (a reescrita da locomocao; vault:
// 04_locomocao/reescrita_da_locomocao.md). Modulos puros: unidade, a tabela de
// trocas linha a linha e propriedades com sinais aleatorios.
#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/Athlete/Modes/ContactClasses3D.hpp"
#include "Engine/Athlete/Modes/ModeRules3D.hpp"
#include "Engine/Athlete/Physical/DynamicTurnMove3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Athlete/Legacy/LegacyReference3D.hpp"
#include "Engine/Athlete/Reference/PhaseLock3D.hpp"
#include "Engine/Athlete/Reference/ReferenceFrame3D.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
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

// O corpo do jogador do jogo (80 kg; as fracoes do perfil).
ContactBody3D playerBody() {
    RagdollProfile3D profile;
    profile.totalMassKg = 80.0f;
    const std::pair<const char*, float> links[] = { { "Pelvis", 0.142f }, { "Abdomen", 0.102f }, { "Chest", 0.124f },
        { "UpperChest", 0.086f }, { "Neck", 0.02f }, { "Head", 0.081f }, { "LeftUpperArm", 0.027f },
        { "LeftForearm", 0.016f }, { "LeftHand", 0.006f }, { "RightUpperArm", 0.027f }, { "RightForearm", 0.016f },
        { "RightHand", 0.006f }, { "LeftThigh", 0.112f }, { "LeftShin", 0.047f }, { "LeftFoot", 0.0145f },
        { "RightThigh", 0.112f }, { "RightShin", 0.047f }, { "RightFoot", 0.0145f } };
    for (const auto& [id, fraction] : links) {
        RagdollLinkDefinition3D link;
        link.id = id;
        link.massFraction = fraction;
        profile.links.push_back(link);
    }
    return contactBody3D(profile);
}

constexpr std::uint32_t Pelvis = 0, LeftHand = 8, LeftShin = 13, LeftFoot = 14, LeftThigh = 12, Chest = 2;

// Um contato de interacao: a normal da superficie do outro (para o link) e o
// link vindo contra ela a `approach` m/s.
RagdollInteraction3D touch(std::uint32_t link, Vec3 normal, float height, float impulse, float approach,
    RagdollContactMotion3D motion = RagdollContactMotion3D::Static) {
    RagdollInteraction3D c;
    c.linkIndex = link;
    c.otherMotion = motion;
    c.pointCount = 1;
    c.positionWorld = { 0.0f, 0.0f, height };
    c.normalWorld = normal;
    c.relativeVelocityWorld = normal * -approach;
    c.normalImpulseNewtonSeconds = impulse;
    c.velocityBeforeSolve = true;
    return c;
}

RagdollWorldContact3D poke(std::uint32_t link, Vec3 normal, float height, float depth) {
    RagdollWorldContact3D c;
    c.linkIndex = link;
    c.normalWorld = normal;
    c.pointWorld = { 0.0f, 0.0f, height };
    c.depthMeters = depth;
    return c;
}

const Vec3 Up { 0.0f, 0.0f, 1.0f };
const Vec3 Side { 1.0f, 0.0f, 0.0f };

void contactClassTests() {
    const ContactBody3D body = playerBody();
    const ContactClassParams3D p;
    const ContactDeclarations3D none;
    expect(p.valid(), "os parametros padrao sao validos");
    expect(body.valid() && std::abs(body.massKg - 80.0f) < 1e-4f, "o corpo de 80 kg");
    expect(body.regions[7] == BodyRegion3D::Arm && body.regions[6] == BodyRegion3D::Arm,
        "o antebraco (Forearm, com o arm minusculo) e o braco sao bracos");
    expect(body.regions[LeftShin] == BodyRegion3D::Leg && body.regions[LeftFoot] == BodyRegion3D::Foot
            && body.regions[LeftHand] == BodyRegion3D::Arm && body.regions[Chest] == BodyRegion3D::Trunk,
        "as regioes do corpo pelo nome do link");

    // O pe: chao e degrau sao apoio; a ponta na parede acima do degrau e impacto.
    expect(classifyInteraction3D(touch(LeftFoot, Up, 0.0f, 20.0f, 2.0f), body, none, 0.0f, p) == ContactClass3D::Support,
        "o pe no chao e apoio");
    expect(classifyInteraction3D(touch(LeftFoot, Side, 0.2f, 20.0f, 2.0f), body, none, 0.0f, p)
            == ContactClass3D::Support,
        "o pe no espelho de um degrau baixo e apoio (subir escada nao vai ao fisico)");
    expect(classifyInteraction3D(touch(LeftFoot, Side, 0.6f, 20.0f, 2.0f), body, none, 0.0f, p)
            == ContactClass3D::Impact,
        "o pe batendo numa parede acima da faixa do degrau e impacto");

    // O chao embaixo da base tocado por outra parte (carrinho, levantar).
    expect(classifyInteraction3D(touch(Pelvis, Up, 0.02f, 30.0f, 3.0f), body, none, 0.0f, p) == ContactClass3D::Ground,
        "o quadril no chao embaixo da base e chao (o carrinho nao vira impacto pelo chao)");

    // O topo de um obstaculo acima da base nao e chao (o buraco de hoje).
    expect(classifyInteraction3D(touch(LeftShin, Up, 0.5f, 10.0f, 2.0f), body, none, 0.0f, p) == ContactClass3D::Impact,
        "a canela no topo de uma caixa de 50 cm e impacto");

    // Os limiares de hoje: 4 N.s com o que se move e 2,5 N.s com o mundo parado
    // (corpo de 80 kg); ~3,8 N.s na canela (3,76 kg).
    constexpr auto Moving = RagdollContactMotion3D::Dynamic;
    expect(classifyInteraction3D(touch(LeftHand, Side, 1.0f, 4.2f, 1.0f, Moving), body, none, 0.0f, p)
            == ContactClass3D::Impact,
        "a mao acima de 0,05 m/s do corpo (4 N.s) com algo que se move e impacto");
    expect(classifyInteraction3D(touch(LeftHand, Side, 1.0f, 2.0f, 1.0f, Moving), body, none, 0.0f, p)
            == ContactClass3D::Graze,
        "a mao abaixo do limiar do corpo e raspao (o Dv do link nao conta para bracos)");
    expect(classifyInteraction3D(touch(LeftShin, Side, 0.5f, 3.9f, 1.0f, Moving), body, none, 0.0f, p)
            == ContactClass3D::Impact,
        "a canela com Dv do link >= 1 m/s e impacto mesmo abaixo de 4 N.s");
    expect(classifyInteraction3D(touch(LeftShin, Side, 0.5f, 3.0f, 1.0f, Moving), body, none, 0.0f, p)
            == ContactClass3D::Graze,
        "a canela abaixo dos dois limiares e raspao");
    expect(classifyInteraction3D(touch(Chest, Side, 1.2f, 2.6f, 1.0f), body, none, 0.0f, p) == ContactClass3D::Impact,
        "o peito na parede acima de 2,5 N.s e impacto (o limiar do mundo parado de hoje)");
    expect(classifyInteraction3D(touch(Chest, Side, 1.2f, 2.0f, 1.0f), body, none, 0.0f, p) == ContactClass3D::Graze,
        "o peito na parede abaixo de 2,5 N.s e raspao");
    expect(classifyInteraction3D(touch(LeftThigh, Side, 0.7f, 30.0f, 0.1f), body, none, 0.0f, p)
            == ContactClass3D::Graze,
        "aproximacao lenta (encostado) e raspao, mesmo com impulso");

    // O contato da acao.
    ContactDeclarations3D tackle;
    tackle.intentionalLinks = 1ull << LeftShin;
    expect(classifyInteraction3D(touch(LeftShin, Side, 0.3f, 30.0f, 4.0f), body, tackle, 0.0f, p)
            == ContactClass3D::Intentional,
        "a perna do carrinho declarada e intencional");

    // As penetracoes (o corpo guiado nao colide com superficies para cima).
    expect(classifyPenetration3D(poke(LeftShin, Up, 0.5f, 0.03f), body, 0.0f, p) == ContactClass3D::Impact,
        "a canela 3 cm dentro do topo de uma caixa e impacto");
    expect(classifyPenetration3D(poke(LeftShin, Up, 0.5f, 0.01f), body, 0.0f, p) == ContactClass3D::Graze,
        "1 cm dentro e raspao (abaixo de 1,5 cm)");
    expect(classifyPenetration3D(poke(LeftShin, Up, 0.03f, 0.05f), body, 0.0f, p) == ContactClass3D::Ground,
        "o joelho no chao embaixo da base e chao");
    expect(classifyPenetration3D(poke(LeftHand, Side, 1.0f, 0.10f), body, 0.0f, p) == ContactClass3D::Graze,
        "a mao entrando na parede nao vira impacto");
    expect(classifyPenetration3D(poke(LeftFoot, Up, 0.3f, 0.05f), body, 0.0f, p) == ContactClass3D::Support,
        "o pe no degrau e apoio");

    // A base acompanha o terreno: uma caixa de 50 cm com a base em cima dela e chao.
    expect(classifyPenetration3D(poke(LeftShin, Up, 0.52f, 0.05f), body, 0.5f, p) == ContactClass3D::Ground,
        "em cima da caixa, o topo dela e o chao");

    // O resumo do tick.
    const std::vector<RagdollInteraction3D> hits { touch(LeftFoot, Up, 0.0f, 10.0f, 1.0f),
        touch(LeftShin, Side, 0.5f, 6.0f, 2.0f), touch(Chest, Side, 1.2f, 9.0f, 2.0f) };
    const std::vector<RagdollWorldContact3D> pokes { poke(LeftThigh, Up, 0.6f, 0.04f) };
    const ContactAssessment3D a = assessContacts3D(hits, pokes, body, none, 0.0f, p);
    expect(a.impact && a.link == Chest && !a.fromPenetration, "o impacto mais forte do tick e o do peito");
    expect(std::abs(a.impactImpulse - 15.0f) < 1e-4f, "a soma dos impulsos de impacto (sem o apoio)");
    expect(std::abs(a.penetrationMeters - 0.04f) < 1e-6f, "a maior penetracao registrada");
    const ContactAssessment3D only = assessContacts3D({}, pokes, body, none, 0.0f, p);
    expect(only.impact && only.fromPenetration && only.link == LeftThigh, "so a penetracao tambem e impacto");
    // A pancada espalhada: cada parte leva pouco, a soma do tick passa (como hoje).
    const std::vector<RagdollInteraction3D> spread { touch(Chest, Side, 1.2f, 1.5f, 2.0f, Moving),
        touch(LeftHand, Side, 1.1f, 1.5f, 2.0f, Moving), touch(5, Side, 1.6f, 1.5f, 2.0f, Moving) };
    const ContactAssessment3D spreadHit = assessContacts3D(spread, {}, body, none, 0.0f, p);
    expect(spreadHit.impact && std::abs(spreadHit.impactImpulse - 4.5f) < 1e-4f,
        "a caixa batendo no peito, no braco e na cabeca (1,5 N.s cada) soma 4,5 N.s: impacto");
    const ContactAssessment3D twoLight = assessContacts3D(std::vector { touch(Chest, Side, 1.2f, 1.5f, 2.0f, Moving),
        touch(LeftHand, Side, 1.1f, 1.5f, 2.0f, Moving) }, {}, body, none, 0.0f, p);
    expect(!twoLight.impact && twoLight.grazes == 2, "3 N.s somados com o que se move ficam abaixo de 4: raspoes");
    const ContactAssessment3D quiet = assessContacts3D(std::vector { touch(LeftFoot, Up, 0.0f, 40.0f, 3.0f) }, {}, body,
        none, 0.0f, p);
    expect(!quiet.impact && quiet.impactImpulse == 0.0f, "andando: so apoio, nenhum impacto");
}

ModeSignals3D calm() {
    ModeSignals3D s;
    s.supported = true;
    s.assistLevel = 1.0f;
    return s;
}

ModeSignals3D withImpact(float bodyDeltaV = 0.1f) {
    ModeSignals3D s = calm();
    s.contacts.impact = true;
    s.contacts.bodyDeltaV = bodyDeltaV;
    s.contacts.impactImpulse = bodyDeltaV * 80.0f;
    return s;
}

void modeRuleTests() {
    const ModeRuleParams3D p;
    expect(p.valid(), "os parametros padrao das trocas sao validos");
    constexpr float Dt = 1.0f / 120.0f;

    // Cada linha da tabela.
    {
        ModeRules3D rules;
        const ModeDecision3D d = rules.decide(BodyModeKind3D::Animated, withImpact(), p, Dt);
        expect(d.change && d.to == BodyModeKind3D::Physical && d.event == ModeEvent3D::Impact,
            "animado + impacto -> fisico");
    }
    {
        ModeRules3D rules;
        const ModeDecision3D d = rules.decide(BodyModeKind3D::Bridge, withImpact(0.11f), p, Dt);
        expect(d.change && d.event == ModeEvent3D::Impact,
            "ponte + impacto medio (9 N.s) -> fisico (antes so acima de 25 N.s)");
    }
    {
        ModeRules3D rules;
        const ModeDecision3D d = rules.decide(BodyModeKind3D::Returning, withImpact(), p, Dt);
        expect(d.change && d.event == ModeEvent3D::Impact, "volta + impacto -> fisico");
    }
    {
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.contacts.impact = true;
        s.contacts.fromPenetration = true;
        s.contacts.penetrationMeters = 0.03f;
        const ModeDecision3D d = rules.decide(BodyModeKind3D::Animated, s, p, Dt);
        expect(d.change && d.event == ModeEvent3D::Penetration, "animado + perna no topo de algo -> fisico");
    }
    {
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.capsuleWallImpact = 2.0f;
        expect(rules.decide(BodyModeKind3D::Animated, s, p, Dt).event == ModeEvent3D::CapsuleWall,
            "a capsula batendo a 2 m/s -> fisico");
        s.capsuleWallImpact = 1.0f;
        expect(!rules.decide(BodyModeKind3D::Animated, s, p, Dt).change, "a capsula a 1 m/s nao troca");
    }
    {
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.assistLevel = 0.7f;
        const ModeDecision3D d = rules.decide(BodyModeKind3D::Bridge, s, p, Dt);
        expect(d.change && d.event == ModeEvent3D::BridgeAssist && d.threshold == p.bridgeAssist,
            "ponte + a ajuda caindo abaixo de 0,8 -> fisico");
        expect(!rules.decide(BodyModeKind3D::Animated, s, p, Dt).change,
            "no animado a ajuda baixa sozinha nao troca de modo");
    }
    {
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.chestTiltRate = 4.0f;
        expect(!rules.decide(BodyModeKind3D::Bridge, s, p, Dt).change,
            "o criterio do tronco tombando vem desligado (limiar 0)");
        ModeRuleParams3D tilt = p;
        tilt.bridgeTiltRate = 3.0f;
        expect(tilt.valid() && rules.decide(BodyModeKind3D::Bridge, s, tilt, Dt).event == ModeEvent3D::BridgeTilt,
            "ligado (3 rad/s): ponte + o peito tombando a 4 rad/s fora da pose -> fisico");
    }
    {
        // O desvio da pose precisa durar bridgePoseSeconds.
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.poseDeviationDegrees = 25.0f;
        s.pendulumDegrees = 30.0f;
        int ticks = 0;
        ModeDecision3D d;
        while (!(d = rules.decide(BodyModeKind3D::Bridge, s, p, Dt)).change && ticks < 120) ++ticks;
        expect(d.event == ModeEvent3D::BridgePose && ticks + 1 >= static_cast<int>(p.bridgePoseSeconds / Dt) - 1
                && ticks + 1 <= static_cast<int>(p.bridgePoseSeconds / Dt) + 2,
            "ponte + 25 graus fora da pose por 0,1 s -> fisico (e nao antes)");
        ModeRules3D fresh;
        s.poseDeviationDegrees = 15.0f;
        for (int i = 0; i < 120; ++i)
            expect(!fresh.decide(BodyModeKind3D::Bridge, s, p, Dt).change, "15 graus fora da pose nao aborta");
        ModeRules3D rising;
        s.poseDeviationDegrees = 27.0f;
        s.pendulumDegrees = 10.0f;
        for (int i = 0; i < 120; ++i)
            expect(!rising.decide(BodyModeKind3D::Bridge, s, p, Dt).change,
                "fora da pose com o corpo em pe (pendulo de 10 graus: o fim do levantar) nao aborta");
    }
    {
        // Sem apoio: o voo do sprint (0,14 s) nao aborta; 0,31 s aborta; o pulo de proposito nao conta.
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.supported = false;
        bool aborted = false;
        for (int i = 0; i < 17; ++i) aborted |= rules.decide(BodyModeKind3D::Bridge, s, p, Dt).change;
        expect(!aborted, "o voo de uma passada de sprint nao aborta a ponte");
        for (int i = 0; i < 24 && !aborted; ++i)
            aborted = rules.decide(BodyModeKind3D::Bridge, s, p, Dt).event == ModeEvent3D::BridgeSupport;
        expect(aborted, "mais de 0,3 s sem apoio aborta a ponte");
        ModeRules3D jump;
        s.airborneByDesign = true;
        bool any = false;
        for (int i = 0; i < 120; ++i) any |= jump.decide(BodyModeKind3D::Bridge, s, p, Dt).change;
        expect(!any, "no ar de proposito (pulo) nao aborta");
    }
    {
        ModeRules3D rules;
        ModeSignals3D s = withImpact();
        s.grabbed = true;
        expect(rules.decide(BodyModeKind3D::Animated, s, p, Dt).event == ModeEvent3D::Grab,
            "agarrado (prioridade 2) vence o impacto (3)");
        ModeSignals3D both = withImpact();
        both.jumped = true;
        expect(rules.decide(BodyModeKind3D::Animated, both, p, Dt).event == ModeEvent3D::Impact,
            "o impacto (3) vence o pulo (4)");
    }
    {
        ModeRules3D rules;
        ModeSignals3D s = calm();
        s.actionContact = true;
        expect(rules.decide(BodyModeKind3D::Animated, s, p, Dt).event == ModeEvent3D::ActionContact,
            "o contato do carrinho -> fisico");
    }
    {
        // A trava depois de abortar e o impacto recente seguram a volta.
        ModeRules3D rules;
        const ModeDecision3D d = rules.decide(BodyModeKind3D::Bridge, withImpact(), p, Dt);
        rules.noteTransition(BodyModeKind3D::Bridge, d, p);
        expect(!rules.returnAllowed(p), "logo depois de abortar a ponte nao volta");
        int ticks = 0;
        while (!rules.returnAllowed(p) && ticks < 240) {
            rules.decide(BodyModeKind3D::Physical, calm(), p, Dt);
            ++ticks;
        }
        expect(ticks * Dt >= p.abortLockoutSeconds - Dt && ticks * Dt <= p.abortLockoutSeconds + 2 * Dt,
            "a trava dura abortLockoutSeconds");
        ModeRules3D recent;
        recent.decide(BodyModeKind3D::Physical, withImpact(), p, Dt);
        expect(!recent.returnAllowed(p), "com um impacto agora nao volta");
    }
    {
        ModeRules3D rules;
        expect(!rules.decide(BodyModeKind3D::Physical, withImpact(), p, Dt).change,
            "no fisico a tabela nao troca (a volta e decidida pela calma)");
        expect(!rules.decide(BodyModeKind3D::Active, withImpact(), p, Dt).change,
            "o Ativo (experimental) fica fora da tabela");
    }
}

void pendulumTests() {
    const Vec3 base { 0.0f, 0.0f, 0.0f };
    expect(pendulumDeviationDegrees3D({ 0.0f, 0.0f, 1.0f }, base, { 0.0f, 0.0f, 1.0f }, base) < 1e-4f,
        "em pe na pose: desvio zero");
    // O sprint inclinado 30 graus de proposito: a pose tambem esta, desvio zero.
    const Vec3 lean30 { std::tan(0.5236f), 0.0f, 1.0f };
    expect(pendulumDeviationDegrees3D(lean30, base, lean30, base) < 1e-3f,
        "o pendulo cru de 30 graus igual ao da pose nao e desvio");
    const float d = pendulumDeviationDegrees3D(lean30, base, { 0.0f, 0.0f, 1.0f }, base);
    expect(std::abs(d - 30.0f) < 0.1f, "30 graus inclinado contra a pose em pe: 30 graus de desvio");
    expect(std::isfinite(pendulumDeviationDegrees3D({ 0.0f, 0.0f, 0.0f }, base, { 0.0f, 0.0f, 0.0f }, base)),
        "caido (centro de massa no chao): finito");
    expect(std::abs(pendulumDegrees3D(lean30, base) - 30.0f) < 0.1f, "o pendulo de 30 graus");
}

void referenceFrameTests() {
    BodyForecast3D forecast;
    DefaultLocomotionOutput3D motion;
    ReferenceFrameInput3D input;
    expect(!buildReferenceFrame3D(legacyReferenceMotion3D(motion), input, forecast).valid, "sem o padrao valido, a referencia nao vale");

    // Correndo em linha reta a 4 m/s (o pedido igual a velocidade): o pe
    // esquerdo apoiado, o direito no balanco pousando em 0,12 s.
    motion.valid = true;
    motion.cycling = true;
    motion.cyclePhase = 0.3f;
    motion.movingShare = 1.0f;
    motion.cycleSeconds = 0.7f;
    motion.forecastStep = 0.04f;
    motion.cyclePlanted = { true, false };
    motion.cycleSecondsToTouchdown = { -1.0f, 0.12f };
    motion.secondsToLiftoff = { 0.2f, -1.0f };
    motion.footForecast[1][3] = { 0.3f, -0.1f, 0.0f };
    input.pelvisGround = { 0.0f, 0.0f, 0.0f };
    input.velocity = { 4.0f, 0.0f, 0.0f };
    input.requested = { 4.0f, 0.0f, 0.0f };
    const ReferenceFrame3D frame = buildReferenceFrame3D(legacyReferenceMotion3D(motion), input, forecast);
    expect(frame.valid && frame.pose == &motion.pose && frame.coordinates == &motion.coordinates,
        "a pose de estilo e a do padrao, sem copia");
    expect(frame.feet[0].planted && !frame.feet[0].touchdownValid, "o pe apoiado nao tem meta de pouso");
    expect(!frame.feet[1].planted && frame.feet[1].touchdownValid
            && std::abs(frame.feet[1].touchdownWorld.x - (0.48f + 0.3f)) < 0.02f
            && std::abs(frame.feet[1].touchdownWorld.y + 0.1f) < 0.02f,
        "o pe no balanco pousa onde a previsao do pe diz, levada pelo corpo a 4 m/s (0,78; -0,1)");
    for (int i = 0; i < ReferenceFrame3D::PlanCount; ++i) {
        const float t = frame.planTimes[static_cast<std::size_t>(i)];
        expect(std::abs(frame.planPosition[static_cast<std::size_t>(i)].x - 4.0f * t) < 0.02f
                && std::abs(frame.planVelocity[static_cast<std::size_t>(i)].x - 4.0f) < 0.05f,
            "em linha reta, o plano a frente e a velocidade constante");
    }

    // Arrancando do parado: o perfil decide o quanto o plano anda.
    input.velocity = {};
    input.profile = { 4.0f, 6.0f, 8.0f, 3.0f };
    const ReferenceFrame3D slow = buildReferenceFrame3D(legacyReferenceMotion3D(motion), input, forecast);
    input.profile = {};
    const ReferenceFrame3D fast = buildReferenceFrame3D(legacyReferenceMotion3D(motion), input, forecast);
    expect(slow.planVelocity[2].x < fast.planVelocity[2].x && std::abs(slow.planVelocity[2].x - 2.0f) < 0.1f,
        "com o perfil do fisico (4 m/s2) o plano chega a 2 m/s em 0,5 s; o do padrao (16 m/s2), mais");
    input.profile = { -1.0f, 6.0f, 8.0f, 3.0f };
    const ReferenceFrame3D fallback = buildReferenceFrame3D(legacyReferenceMotion3D(motion), input, forecast);
    expect(std::abs(fallback.planVelocity[2].x - fast.planVelocity[2].x) < 1e-4f,
        "um perfil invalido cai no do padrao (falha segura)");
}

void correctionTests() {
    PhysicalCorrections3D c;
    expect(!c.active(), "sem cabeca ligada, as correcoes nao estao ativas (o classico exato)");
    const Vec3 goal { 1.0f, 2.0f, 0.1f };
    c.frame = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 1.5707963f);
    c.feet[1].touchdownOffset = { 0.2f, 0.0f, 0.0f };
    c.feet[1].swingHeight = 0.04f;
    Vec3 out = correctedFootGoal3D(goal, false, false, 0.5f, &c, 1);
    expect((out - goal).length() < 1e-6f, "com a cabeca dos pes desligada, o alvo do pe nao muda");
    c.footwork = true;
    expect(c.active(), "uma cabeca ligada liga as correcoes");
    out = correctedFootGoal3D(goal, false, false, 0.5f, &c, 1);
    expect(std::abs(out.x - 1.0f) < 1e-4f && std::abs(out.y - 2.2f) < 1e-4f && std::abs(out.z - 0.14f) < 1e-4f,
        "no meio do balanco: o pouso inteiro no referencial do rumo (0,2 m a frente virado 90 graus) e a altura no pico");
    out = correctedFootGoal3D(goal, false, false, 0.0f, &c, 1);
    expect((out - goal).length() < 1e-6f, "na saida do pe (progresso 0) nada: o pouso entra ao longo do balanco");
    out = correctedFootGoal3D(goal, false, false, 1.0f, &c, 1);
    expect(std::abs(out.y - 2.2f) < 1e-4f && std::abs(out.z - 0.1f) < 1e-4f, "no pouso: o deslocamento inteiro, a altura a zero");
    out = correctedFootGoal3D(goal, false, true, 0.5f, &c, 1);
    expect(std::abs(out.y - 2.0f) < 1e-6f && std::abs(out.z - 0.14f) < 1e-4f,
        "no passo do planejador so a altura: o pouso vai no plano dele");
    const Vec3 planned = plannerLandingOffset3D(&c, 1);
    expect(std::abs(planned.x) < 1e-6f && std::abs(planned.y - 0.2f) < 1e-6f, "o pouso do planejador no mundo");
    expect((correctedFootGoal3D(goal, true, false, 0.5f, &c, 1) - goal).length() < 1e-6f
            && (correctedFootGoal3D(goal, false, false, -1.0f, &c, 1) - goal).length() < 1e-6f,
        "o pe apoiado (ou sem balanco) nunca e arrastado pela correcao");
    expect((correctedFootGoal3D(goal, false, false, 0.5f, nullptr, 1) - goal).length() < 1e-6f, "sem correcoes, o alvo e o mesmo");

    c.cadenceOctaves = 0.5f;
    c.strideOctaves = -1.0f;
    expect(std::abs(cadenceFactor3D(&c) - std::sqrt(2.0f)) < 1e-6f && std::abs(strideFactor3D(&c) - 0.5f) < 1e-6f
            && cadenceFactor3D(nullptr) == 1.0f,
        "o ritmo e o comprimento em oitavas");
    c.feet[0].phaseLead = -0.04f;
    expect(std::abs(legPhaseLead3D(&c, 0, false) + 0.04f) < 1e-6f && legPhaseLead3D(&c, 0, true) == 0.0f,
        "o atraso de uma perna, cortado na perna que espera o toque");
    c.feet[0].recoveryUrgency = 3.0f;
    expect(recoveryUrgency3D(&c, 0) == 1.0f && recoveryUrgency3D(&c, 1) == 0.0f, "a urgencia de recuperacao fica em 0..1");

    c.forwardLean = 0.1f;
    c.lateralLean = -0.05f;
    c.comOffset = { 0.1f, 0.0f, 0.3f };
    c.pelvisHeight = -0.03f;
    c.contactAcceleration = { 0.0f, 2.0f, 5.0f };
    expect(correctedForwardLean3D(0.2f, &c) == 0.2f && comOffsetWorld3D(&c).lengthSquared() == 0.0f
            && pelvisHeightOffset3D(&c) == 0.0f,
        "com o equilibrio desligado, nada muda");
    c.balance = true;
    expect(std::abs(correctedForwardLean3D(0.2f, &c) - 0.3f) < 1e-6f
            && std::abs(correctedLateralLean3D(0.0f, &c) + 0.05f) < 1e-6f,
        "o equilibrio soma a inclinacao");
    const Vec3 com = comOffsetWorld3D(&c), push = contactAccelerationWorld3D(&c);
    expect(std::abs(com.y - 0.1f) < 1e-5f && std::abs(com.x) < 1e-5f && com.z == 0.0f
            && std::abs(push.x + 2.0f) < 1e-5f && push.z == 0.0f && pelvisHeightOffset3D(&c) == -0.03f,
        "o centro de massa e o contato no plano, no referencial do rumo; a altura da pelve direto");

    RagdollProfile3D profile;
    for (const char* id : { "Pelvis", "LeftThigh", "LeftHand" }) {
        RagdollLinkDefinition3D link;
        link.id = id;
        profile.links.push_back(link);
    }
    std::vector<RagdollDriveTarget3D> targets(3);
    for (std::uint32_t i = 0; i < 3; ++i) {
        targets[i].linkIndex = i;
        targets[i].axis = RagdollAxis3D::Swing1;
        targets[i].positionRadians = 0.5f;
    }
    const std::vector<RagdollDriveTarget3D> before = targets;
    c.jointResiduals.assign(9, 0.0f);
    c.jointResiduals[1 * 3 + 1] = 0.1f;
    c.gainScale = { 2.0f, 1.0f, 0.5f };
    correctDriveTargets3D(targets, profile, &c);
    bool same = true;
    for (std::size_t i = 0; i < 3; ++i)
        same &= targets[i].positionRadians == before[i].positionRadians && targets[i].stiffnessScale == before[i].stiffnessScale;
    expect(same, "com a pose e a rigidez desligadas, os alvos dos motores nao mudam");
    c.pose = true;
    c.gain = true;
    correctDriveTargets3D(targets, profile, &c);
    expect(std::abs(targets[1].positionRadians - 0.6f) < 1e-6f && targets[0].positionRadians == 0.5f,
        "a pose soma a correcao do eixo (coxa, swing1: 0,1) e so dele");
    expect(std::abs(targets[1].stiffnessScale - 2.0f) < 1e-6f && std::abs(targets[2].stiffnessScale - 0.5f) < 1e-6f
            && std::abs(targets[0].stiffnessScale - 1.0f) < 1e-6f,
        "a rigidez escala por grupo: pernas x2, bracos x0,5, tronco igual");
    expect(physicalGainGroup3D("RightToe") == 0 && physicalGainGroup3D("Head") == 1
            && physicalGainGroup3D("RightForearm") == 2,
        "os grupos de ganho pelo nome do link");
}

// O contrato da rede num corpo de 18 links como o do jogo.
RagdollProfile3D contractProfile() {
    RagdollProfile3D profile;
    profile.id = "teste";
    profile.totalMassKg = 80.0f;
    const char* ids[] = { "Pelvis", "Abdomen", "Chest", "UpperChest", "Neck", "Head", "LeftUpperArm", "LeftForearm",
        "LeftHand", "RightUpperArm", "RightForearm", "RightHand", "LeftThigh", "LeftShin", "LeftFoot", "RightThigh",
        "RightShin", "RightFoot" };
    for (const char* id : ids) {
        RagdollLinkDefinition3D link;
        link.id = id;
        link.parentIndex = profile.links.empty() ? -1 : 0;
        link.modelPosition = { 0.0f, 0.0f, 1.0f };
        const std::string name(id);
        if (!profile.links.empty()) {
            link.inboundJoint.axes[0].enabled = true;
            link.inboundJoint.axes[1].enabled = name.find("Hand") == std::string::npos;
            link.inboundJoint.axes[2].enabled = name.find("Thigh") != std::string::npos
                || name.find("UpperArm") != std::string::npos;
        }
        if (name == "LeftThigh") link.inboundJoint.anchorModelPosition = { 0.0f, 0.1f, 0.9f };
        if (name == "RightThigh") link.inboundJoint.anchorModelPosition = { 0.0f, -0.1f, 0.9f };
        profile.links.push_back(link);
    }
    return profile;
}

RagdollState3D contractBody(const RagdollProfile3D& profile, Quaternion turn) {
    RagdollState3D body;
    body.links.resize(profile.links.size());
    body.joints.resize(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        body.links[i].position = Vec3 { 0.0f, 0.0f, 1.0f } + turn.rotate({ 0.01f * static_cast<float>(i), 0.0f, 0.0f });
        body.links[i].orientation = turn;
    }
    return body;
}

std::size_t channelOf(const LocomotionContract3D& contract, const std::string& name) {
    const auto& channels = contract.observationLayout().channels;
    for (std::size_t i = 0; i < channels.size(); ++i)
        if (channels[i].name == name) return i;
    throw std::runtime_error("canal sumiu: " + name);
}

void contractTests() {
    const RagdollProfile3D profile = contractProfile();
    const LocomotionContract3D contract(profile);
    std::size_t axes = 0;
    for (std::size_t i = 1; i < profile.links.size(); ++i)
        for (const RagdollAxisDefinition3D& axis : profile.links[i].inboundJoint.axes) axes += axis.enabled ? 1 : 0;
    expect(contract.actuatedAxes().size() == axes, "os eixos atuados sao os habilitados no perfil");
    const std::size_t links = profile.links.size();
    expect(contract.actionSize() == 12 + 7 + axes + 3 + 4,
        "a saida: os pes (12), o equilibrio (7), um residuo por eixo, os ganhos e o ganho da virada (4)");
    expect(contract.observationSize()
            == 1 + links * 12 + (links - 1) * 3 + 2 * axes + 4 + 7 + 50 + 28 + 9 + 53 + 13
                + LocomotionContract3D::HistoryDecisions * contract.actionSize(),
        "a entrada: o tamanho de cada bloco (o executor com 53, a virada com 13)");

    // O que observe escreve e o layout, na mesma conta (sem deriva).
    std::vector<float> out(contract.observationSize());
    BodyForecast3D forecast;
    AthleteCommand3D command;
    AthleteControlView3D view;
    view.groundHeight = 0.0f;
    const auto observe = [&](const RagdollState3D& body) {
        return contract.observe(body, command, view, {}, forecast, out);
    };
    LocomotionObservation3D result = observe(contractBody(profile, {}));
    expect(result.written == contract.observationSize(), "observe escreve exatamente o layout");
    expect(result.nonFinite == 0, "de pe, nada nao finito");
    expect(std::abs(out[channelOf(contract, "corpo.Pelvis.frente.x")] - 1.0f) < 1e-5f
            && std::abs(out[channelOf(contract, "corpo.Pelvis.frente.y")]) < 1e-5f
            && std::abs(out[channelOf(contract, "corpo.pelve.altura")] - 1.0f) < 1e-5f,
        "de pe na pose de ligacao: a frente da pelve e +x, a 1 m do chao");

    // Girado: o referencial gira junto (a entrada nao depende do rumo no mundo).
    const float yaw = 1.1f;
    const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
    command.facingYawRadians = yaw;
    command.intent.requestedVelocityWorld = turn.rotate({ 3.0f, 0.0f, 0.0f });
    std::vector<float> turned(contract.observationSize());
    (void)contract.observe(contractBody(profile, turn), command, view, {}, forecast, turned);
    command.facingYawRadians = 0.0f;
    command.intent.requestedVelocityWorld = { 3.0f, 0.0f, 0.0f };
    (void)observe(contractBody(profile, {}));
    bool same = true;
    for (std::size_t i = 0; i < out.size(); ++i) same &= std::abs(out[i] - turned[i]) < 1e-4f;
    expect(same, "girar o corpo e o comando juntos nao muda a entrada");

    // Deitado de costas (a frente da pelve para cima): o rumo vem do lado.
    const Quaternion back = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -1.5707963f);
    result = observe(contractBody(profile, back));
    expect(result.nonFinite == 0 && std::abs(out[channelOf(contract, "corpo.Pelvis.frente.z")] - 1.0f) < 1e-4f,
        "deitado de costas: finito, a frente da pelve para cima");

    // Um numero estragado no corpo vira zero e e contado.
    RagdollState3D broken = contractBody(profile, {});
    broken.links[3].linearVelocity.x = std::numeric_limits<float>::quiet_NaN();
    result = observe(broken);
    bool finite = true;
    for (float v : out) finite &= std::isfinite(v);
    expect(result.nonFinite > 0 && finite, "um NaN no corpo: contado, e a entrada segue finita");

    // A saida: a escala maxima, o teto da fase e a autoridade da cabeca.
    const auto actionIndex = [&](const std::string& name) {
        const auto& channels = contract.actionLayout().channels;
        for (std::size_t i = 0; i < channels.size(); ++i)
            if (channels[i].name == name) return i;
        throw std::runtime_error("canal da saida sumiu: " + name);
    };
    const LocomotionAuthority3D full = LocomotionAuthority3D::uniform(1.0f, LocomotionCaps3D::full());
    const std::vector<float> none = contract.physical(std::vector<float>(contract.actionSize(), 0.0f), full);
    bool neutral = true;
    for (float v : none) neutral &= v == 0.0f;
    PhysicalCorrections3D zero = contract.corrections(none, full);
    neutral &= zero.footwork && zero.balance && zero.pose && zero.gain && zero.jointResiduals.size() == links * 3;
    for (float r : zero.jointResiduals) neutral &= r == 0.0f;
    for (float g : zero.gainScale) neutral &= g == 1.0f;
    expect(neutral, "a acao zero e a correcao zero (as cabecas com autoridade ligadas)");
    const PhysicalCorrections3D off = contract.corrections(none, LocomotionAuthority3D {});
    expect(!off.active(), "sem autoridade e sem valor: nenhuma cabeca ligada (o classico exato)");
    {
        // O ganho da virada: 1 + a acao (-1 descarta, 0 mantem, +1 dobra); a
        // acao zero e sem autoridade deixam o professor inteiro; o ganho nao
        // liga cabeca nenhuma.
        bool unit = true;
        for (float g : zero.turnGate) unit &= g == 1.0f;
        for (float g : off.turnGate) unit &= g == 1.0f;
        expect(unit, "a acao zero e sem autoridade: o ganho da virada e 1 (o professor inteiro)");
        std::vector<float> gates(contract.actionSize(), 0.0f);
        gates[actionIndex("turno.ganho.equilibrio")] = -1.0f;
        gates[actionIndex("turno.ganho.pes")] = 0.5f;
        gates[actionIndex("turno.ganho.pose")] = 1.0f;
        gates[actionIndex("turno.ganho.velocidade")] = -0.25f;
        const PhysicalCorrections3D gated = contract.corrections(contract.physical(gates, full), full);
        expect(std::abs(gated.turnGate[0] - 0.0f) < 1e-6f && std::abs(gated.turnGate[1] - 1.5f) < 1e-6f
                && std::abs(gated.turnGate[2] - 2.0f) < 1e-6f && std::abs(gated.turnGate[3] - 0.75f) < 1e-6f,
            "o ganho da virada: 1 + a acao, de 0 (descarta) a 2");
        LocomotionAuthority3D nobalance = full;
        nobalance.balance = 0.0f;
        const PhysicalCorrections3D closed = contract.corrections(contract.physical(gates, nobalance), nobalance);
        expect(closed.turnGate[0] == 1.0f && closed.turnGate[2] == 1.0f,
            "sem a autoridade do equilibrio (a rampa), o ganho fica em 1: a rede ainda nao mexe no professor");
        std::vector<float> onlyGate(contract.actionSize(), 0.0f);
        onlyGate[actionIndex("turno.ganho.pes")] = 1.0f;
        expect(!contract.corrections(contract.physical(onlyGate, LocomotionAuthority3D::uniform(0.0f)),
                   LocomotionAuthority3D::uniform(0.0f)).active(),
            "o ganho da virada sozinho nao liga cabeca nenhuma");
        const LocomotionMirror3D mirror = contract.mirror();
        bool signs = true;
        for (const char* name : { "turno.lado", "turno.prior.inclina_lado" }) {
            const std::size_t at = channelOf(contract, name);
            signs &= mirror.observation.source[at] == at && mirror.observation.scale[at] == -1.0f;
        }
        for (const char* name : { "turno.angulo", "turno.prior.freada", "turno.estado.corte", "turno.prior.altura" }) {
            const std::size_t at = channelOf(contract, name);
            signs &= mirror.observation.scale[at] == 1.0f;
        }
        for (const char* name : { "turno.ganho.equilibrio", "turno.ganho.pes", "turno.ganho.pose", "turno.ganho.velocidade" }) {
            const std::size_t at = actionIndex(name);
            signs &= mirror.action.source[at] == at && mirror.action.scale[at] == 1.0f;
        }
        expect(signs, "o espelho: o lado e a inclinacao do professor trocam o sinal; o resto e os ganhos ficam");
        // A entrada: ativa, o estado e o que o professor pede; desligada, zeros.
        AthleteControlView3D view;
        view.turn.active = true;
        view.turn.state = 3;
        view.turn.side = -1.0f;
        view.turn.remainingDegrees = 90.0f;
        view.turn.priorBrake = 0.4f;
        RagdollState3D body;
        body.links.resize(profile.links.size());
        for (std::size_t i = 0; i < body.links.size(); ++i) {
            body.links[i].orientation = profile.links[i].modelOrientation;
            body.links[i].position = profile.links[i].modelPosition;
        }
        BodyForecast3D forecast;
        AthleteCommand3D command;
        std::vector<float> in(contract.observationSize(), 0.0f);
        (void)contract.observe(body, command, view, {}, forecast, in);
        expect(in[channelOf(contract, "turno.ativo")] == 1.0f && in[channelOf(contract, "turno.estado.freando")] == 1.0f
                && in[channelOf(contract, "turno.estado.corte")] == 0.0f && in[channelOf(contract, "turno.lado")] == -1.0f
                && std::abs(in[channelOf(contract, "turno.angulo")] - 0.5f) < 1e-6f
                && std::abs(in[channelOf(contract, "turno.prior.freada")] - 0.4f) < 1e-6f,
            "a entrada da virada: o estado, o lado, o que falta virar e a freada pedida");
        view.turn.active = false;
        (void)contract.observe(body, command, view, {}, forecast, in);
        expect(in[channelOf(contract, "turno.ativo")] == 0.0f && in[channelOf(contract, "turno.lado")] == 0.0f
                && in[channelOf(contract, "turno.prior.freada")] == 0.0f,
            "sem a virada, as entradas dela sao zero");
    }
    std::vector<float> big(contract.actionSize(), 5.0f);
    big[actionIndex("pes.esq.pouso.x")] = std::numeric_limits<float>::quiet_NaN();
    LocomotionAuthority3D partial;
    partial.footwork = 0.5f;
    partial.balance = 0.25f;
    partial.pose = 1.0f;
    partial.gain = 0.75f;
    std::vector<float> most = contract.physical(big, partial);
    const auto near = [](float a, float b) { return std::abs(a - b) < 1e-6f; };
    expect(most[actionIndex("pes.esq.pouso.x")] == 0.0f && near(most[actionIndex("pes.esq.pouso.y")], 0.06f * 0.5f)
            && near(most[actionIndex("pes.dir.altura")], 0.05f * 0.5f)
            && near(most[actionIndex("pes.esq.recupera")], 0.5f * 0.5f),
        "os pes: NaN vira zero; o teto da fase corta (pouso 0,06 m, altura 0,05 m, urgencia 0,5) e a autoridade multiplica");
    expect(near(most[actionIndex("equilibrio.inclina.frente")], 0.12f * 0.25f)
            && near(most[actionIndex("ganho.tronco")], 0.3f * 0.75f),
        "o equilibrio e os ganhos: o teto e a autoridade de cada cabeca");
    bool onAxes = true;
    for (std::uint32_t key : contract.actuatedAxes())
        onAxes &= near(most[actionIndex("pose." + profile.links[key / 3].id + "."
            + (key % 3 == 0 ? "twist" : key % 3 == 1 ? "swing1" : "swing2"))], 0.10f);
    expect(onAxes, "a pose: o residuo de cada eixo atuado ate o teto (0,10 rad)");
    const PhysicalCorrections3D mostCorrections = contract.corrections(most, partial);
    expect(near(mostCorrections.gainScale[1], std::exp2(0.225f)) && mostCorrections.jointResiduals.size() == links * 3,
        "as correcoes: o ganho em oitavas vira escala");
    std::vector<float> low(contract.actionSize(), -5.0f);
    most = contract.physical(low, partial);
    expect(near(most[actionIndex("equilibrio.altura")], -0.04f * 0.25f)
            && near(most[actionIndex("pes.esq.altura")], -0.02f * 0.5f) && most[actionIndex("pes.dir.recupera")] == 0.0f,
        "os canais assimetricos: o lado de baixo com o teto dele; a urgencia so na metade positiva");
    partial.caps = LocomotionCaps3D::full();
    most = contract.physical(big, partial);
    expect(near(most[actionIndex("pes.esq.pouso.y")], 0.30f * 0.5f), "com os tetos abertos, a escala maxima");

    // A decisao, o historico, a rampa e o limite de taxa.
    AthleteState3D athlete;
    std::vector<float> first(contract.actionSize(), 0.8f), second(contract.actionSize(), -2.0f);
    contract.decide(athlete, first);
    const std::uint32_t key = contract.actuatedAxes().front();
    const float dt = 1.0f / 120.0f;
    for (int tick = 1; tick <= LocomotionContract3D::TicksPerDecision; ++tick) {
        contract.follow(athlete, full, dt);
        expect(near(athlete.corrections.feet[0].recoveryUrgency, 0.8f * static_cast<float>(tick) / 4.0f),
            "a rampa vai da acao de antes (zero) a nova em 4 ticks (a urgencia, sem limite de taxa)");
        expect(std::abs(athlete.corrections.jointResiduals[key] - LocomotionContract3D::JointRate * dt * static_cast<float>(tick))
                < 1e-5f,
            "o residuo da junta anda no maximo o limite de taxa por tick");
    }
    contract.decide(athlete, second);
    const std::size_t a = contract.actionSize();
    expect(athlete.policyHistory.size() == LocomotionContract3D::HistoryDecisions * a
            && athlete.policyHistory[0] == -1.0f && std::abs(athlete.policyHistory[a] - 0.8f) < 1e-6f
            && athlete.policyHistory[2 * a] == 0.0f,
        "o historico: a mais nova primeiro, guardada no -1..1");
    contract.follow(athlete, full, dt);
    expect(near(athlete.corrections.feet[0].recoveryUrgency, 0.8f + (-1.0f - 0.8f) * 0.25f),
        "a rampa nova parte de onde a anterior estava");
    // Sem autoridade de repente: a cabeca segue ligada ate o valor voltar a
    // zero pelo limite de taxa (desligada de uma vez, saltava).
    for (int tick = 0; tick < 3; ++tick) contract.follow(athlete, full, dt);
    const float leanBefore = athlete.corrections.forwardLean;
    contract.follow(athlete, LocomotionAuthority3D {}, dt);
    expect(athlete.corrections.balance && std::abs(athlete.corrections.forwardLean)
            >= std::abs(leanBefore) - LocomotionContract3D::LeanRate * dt - 1e-6f,
        "a autoridade a zero: o equilibrio volta pelo limite de taxa, ainda ligado");
    for (int tick = 0; tick < 240; ++tick) contract.follow(athlete, LocomotionAuthority3D {}, dt);
    expect(!athlete.corrections.active(), "depois, tudo a zero: nenhuma cabeca ligada");
    contract.decide(athlete, first);
    contract.follow(athlete, full, dt);
    LocomotionContract3D::clear(athlete);
    expect(!athlete.corrections.active() && athlete.policyApplied.empty() && athlete.policyHistory.empty(),
        "clear: a politica esquecida de uma vez (o classico exato)");

    // O espelho: duas vezes e a identidade, e o espelho da entrada de um corpo
    // torto e a entrada do corpo refletido de verdade.
    const LocomotionMirror3D mirror = contract.mirror();
    bool involution = true;
    for (const LocomotionMirrorMap3D* map : { &mirror.observation, &mirror.action })
        for (std::size_t i = 0; i < map->source.size(); ++i)
            involution &= map->source[map->source[i]] == i && map->scale[i] * map->scale[map->source[i]] == 1.0f;
    expect(involution && mirror.observation.source.size() == contract.observationSize()
            && mirror.action.source.size() == contract.actionSize(),
        "o espelho duas vezes e a identidade");
    std::uint64_t seed = 0x2545F4914F6CDD1Dull;
    const auto random = [&]() {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        return static_cast<float>(seed >> 40) / static_cast<float>(1ull << 24) * 2.0f - 1.0f;
    };
    const auto mirrorVector = [](Vec3 v) { return Vec3 { v.x, -v.y, v.z }; };
    const auto mirrorPseudo = [](Vec3 v) { return Vec3 { -v.x, v.y, -v.z }; };
    std::vector<int> side(links);
    for (std::size_t i = 0; i < links; ++i) {
        std::string id = profile.links[i].id;
        if (id.rfind("Left", 0) == 0) id = "Right" + id.substr(4);
        else if (id.rfind("Right", 0) == 0) id = "Left" + id.substr(5);
        for (std::size_t j = 0; j < links; ++j)
            if (profile.links[j].id == id) side[i] = static_cast<int>(j);
    }
    RagdollState3D crooked = contractBody(profile, Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 0.4f));
    RagdollState3D reflected = crooked;
    for (std::size_t i = 0; i < links; ++i) {
        PhysicsBodyState3D& link = crooked.links[i];
        link.position += Vec3 { 0.3f * random(), 0.3f * random(), 0.2f * random() };
        link.orientation = (link.orientation * Quaternion::fromAxisAngle({ random(), random(), random() + 0.01f },
            0.7f * random())).normalized();
        link.linearVelocity = { 3.0f * random(), 3.0f * random(), random() };
        link.angularVelocity = { 4.0f * random(), 4.0f * random(), 4.0f * random() };
    }
    for (std::size_t i = 0; i < links; ++i) {
        const PhysicsBodyState3D& from = crooked.links[i];
        PhysicsBodyState3D& to = reflected.links[static_cast<std::size_t>(side[i])];
        to.position = mirrorVector(from.position);
        to.orientation = { -from.orientation.x, from.orientation.y, -from.orientation.z, from.orientation.w };
        to.linearVelocity = mirrorVector(from.linearVelocity);
        to.angularVelocity = mirrorPseudo(from.angularVelocity);
    }
    RagdollInteraction3D step;
    step.linkIndex = 14;
    step.normalWorld = { 0.0f, 0.0f, 1.0f };
    step.normalImpulseNewtonSeconds = 3.0f;
    crooked.interactions.push_back(step);
    step.linkIndex = static_cast<std::uint32_t>(side[14]);
    reflected.interactions.push_back(step);
    AthleteCommand3D asked;
    asked.intent.requestedVelocityWorld = { 2.0f, 1.5f, 0.0f };
    asked.facingYawRadians = 0.6f;
    asked.lookPitchRadians = -0.2f;
    asked.sprinting = true;
    AthleteCommand3D askedReflected = asked;
    askedReflected.intent.requestedVelocityWorld = mirrorVector(asked.intent.requestedVelocityWorld);
    askedReflected.facingYawRadians = -asked.facingYawRadians;
    AthleteControlView3D assist;
    assist.pelvisForceWorld = { 100.0f, -250.0f, 400.0f };
    assist.pelvisTorqueWorld = { 30.0f, 12.0f, -8.0f };
    // O executor: um passo de lado em andamento (os dois pes diferentes).
    AthleteControlView3D::Executor& exec = assist.executor;
    exec.state = static_cast<int>(CharacterLocomotionState3D::Walking);
    exec.gaitDirection = static_cast<int>(CharacterGaitDirection3D::Left);
    exec.cyclePhase = 0.13f;
    exec.legPhase = { 0.21f, 0.68f };
    exec.swingProgress = { 0.4f, -1.0f };
    exec.clipContact = { 0.1f, 0.9f };
    exec.classicLead = { 0.03f, 0.0f };
    exec.waitingLeg = 1;
    exec.playbackRate = 1.2f;
    exec.strideScale = 0.8f;
    exec.plannerPhase = { 2, 0 };
    exec.plannerOwns = { true, false };
    exec.recoveryUrgency = 0.3f;
    exec.capturePointWorld = { 0.4f, 0.3f, 0.0f };
    exec.captureOutside = 0.05f;
    exec.referenceRootWorld = { 0.1f, -0.2f, 0.95f };
    exec.legShare = 0.7f;
    exec.balanceShare = 0.6f;
    exec.comAccelerationRequested = { 2.0f, -1.0f, 0.0f };
    exec.comAccelerationAllocated = { 1.5f, -0.5f, 0.0f };
    exec.comAccelerationResidual = { 0.5f, -0.5f, 0.0f };
    exec.balanceMargin = 0.04f;
    AthleteControlView3D assistReflected = assist;
    assistReflected.pelvisForceWorld = mirrorVector(assist.pelvisForceWorld);
    assistReflected.pelvisTorqueWorld = mirrorPseudo(assist.pelvisTorqueWorld);
    {
        // No espelho: os lados trocam, as fases andam meio ciclo, os vetores
        // trocam o y.
        AthleteControlView3D::Executor& r = assistReflected.executor;
        const auto swap = [](auto pair) { return decltype(pair) { pair[1], pair[0] }; };
        r.gaitDirection = static_cast<int>(CharacterGaitDirection3D::Right);
        r.cyclePhase = exec.cyclePhase + 0.5f;
        r.legPhase = { exec.legPhase[1] + 0.5f, exec.legPhase[0] + 0.5f };
        r.swingProgress = swap(exec.swingProgress);
        r.clipContact = swap(exec.clipContact);
        r.classicLead = swap(exec.classicLead);
        r.waitingLeg = 1 - exec.waitingLeg;
        r.plannerPhase = swap(exec.plannerPhase);
        r.plannerOwns = swap(exec.plannerOwns);
        r.capturePointWorld = mirrorVector(exec.capturePointWorld);
        r.referenceRootWorld = mirrorVector(exec.referenceRootWorld);
        r.comAccelerationRequested = mirrorVector(exec.comAccelerationRequested);
        r.comAccelerationAllocated = mirrorVector(exec.comAccelerationAllocated);
        r.comAccelerationResidual = mirrorVector(exec.comAccelerationResidual);
    }
    std::vector<float> original(contract.observationSize()), mirrored(contract.observationSize());
    (void)contract.observe(crooked, asked, assist, {}, forecast, original);
    (void)contract.observe(reflected, askedReflected, assistReflected, {}, forecast, mirrored);
    std::size_t worst = 0;
    float worstError = 0.0f;
    for (std::size_t i = 0; i < original.size(); ++i) {
        const float predicted = original[mirror.observation.source[i]] * mirror.observation.scale[i];
        const float error = std::abs(predicted - mirrored[i]);
        if (error > worstError) {
            worstError = error;
            worst = i;
        }
    }
    expect(worstError < 1e-3f, "o espelho da entrada e a entrada do corpo refletido (pior canal: "
            + contract.observationLayout().channels[worst].name + ", erro " + std::to_string(worstError) + ")");

    // A impressao: a mesma para o mesmo corpo, outra se o corpo muda.
    expect(LocomotionContract3D(profile).hash() == contract.hash(), "a impressao e estavel");
    RagdollProfile3D other = profile;
    other.links[14].id = "LeftFootX";
    expect(LocomotionContract3D(other).hash() != contract.hash(), "outro corpo, outra impressao");
}

// O laco de fase: a referencia alcanca a fase do corpo pelos pousos.
void phaseLockTests() {
    const float dt = 1.0f / 120.0f;
    const float cycle = 0.7f;
    const auto wrap = [](float x) { return x - std::floor(x + 0.5f); };
    // O corpo pousa o pe esquerdo em t = 0,7k e o direito em 0,7k + 0,35
    // (cada pe 0,2 s no chao); a referencia anda no mesmo ritmo, 0,3 de fase
    // atrasada, e o laco a puxa.
    PhaseLock3D lock;
    float reference = 0.7f;
    float worstLate = 0.0f;
    for (int tick = 0; tick < static_cast<int>(4.0f / dt); ++tick) {
        const float t = static_cast<float>(tick) * dt;
        const float left = std::fmod(t, cycle), right = std::fmod(t + cycle * 0.5f, cycle);
        const std::array<bool, 2> touching { left < 0.2f, right < 0.2f };
        const float rate = lock.update(touching, reference, 1.0f, true, true, dt);
        expect(rate >= 0.5f && rate <= 1.5f, "o ritmo fica entre 0,5 e 1,5");
        reference = reference + rate * dt / cycle - std::floor(reference + rate * dt / cycle);
        const float body = std::fmod(t, cycle) / cycle;
        if (t > 2.5f) worstLate = std::max(worstLate, std::abs(wrap(reference - body)));
    }
    expect(lock.valid(), "com pousos, a fase do corpo vale");
    expect(worstLate < 0.05f, "a referencia alcanca o corpo (menos de 5% do ciclo depois de 2,5 s)");

    // Os dois pes pousando juntos (um pulo): ambiguo, nao define a fase.
    PhaseLock3D jump;
    for (int tick = 0; tick < 30; ++tick) (void)jump.update({ false, false }, 0.0f, 1.0f, true, true, dt);
    (void)jump.update({ true, true }, 0.0f, 1.0f, true, true, dt);
    expect(!jump.valid(), "os dois pes pousando juntos nao definem a fase");

    // Um pisca de contato (um tick sem contato) nao e pouso.
    PhaseLock3D flicker;
    for (int tick = 0; tick < 10; ++tick) (void)flicker.update({ true, true }, 0.0f, 1.0f, true, true, dt);
    (void)flicker.update({ false, true }, 0.0f, 1.0f, true, true, dt);
    (void)flicker.update({ true, true }, 0.0f, 1.0f, true, true, dt);
    expect(!flicker.valid(), "um tick sem contato nao vira pouso");

    // Parado (a referencia nao anda) ou o laco desligado: o ritmo e 1.
    PhaseLock3D still;
    for (int tick = 0; tick < 30; ++tick) (void)still.update({ false, true }, 0.0f, 1.0f, true, true, dt);
    expect(still.update({ true, true }, 0.3f, 0.2f, true, true, dt) == 1.0f, "parado, o laco nao puxa");
    expect(still.update({ true, true }, 0.3f, 1.0f, true, false, dt) == 1.0f, "desligado, o laco nao puxa");
    still.reset();
    expect(!still.valid() && still.bodyPhase() == 0.0f, "recomecar zera a fase do corpo");
}

// Propriedades: sequencias aleatorias de sinais em todos os modos.
void modeRulePropertyTests() {
    const ModeRuleParams3D p;
    std::uint64_t seed = 0x9E3779B97F4A7C15ull;
    const auto next = [&]() {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        return static_cast<float>(seed >> 40) / static_cast<float>(1ull << 24);
    };
    const auto table = modeRuleTable3D();
    for (int run = 0; run < 200; ++run) {
        ModeRules3D rules;
        BodyModeKind3D mode = static_cast<BodyModeKind3D>(static_cast<int>(next() * 5.0f) % 5);
        for (int tick = 0; tick < 600; ++tick) {
            ModeSignals3D s;
            s.grabbed = next() < 0.01f;
            s.contacts.impact = next() < 0.03f;
            s.contacts.fromPenetration = s.contacts.impact && next() < 0.3f;
            s.contacts.bodyDeltaV = next() * 0.3f;
            s.actionContact = next() < 0.01f;
            s.airTackleStart = next() < 0.005f;
            s.jumped = next() < 0.005f;
            s.capsuleWallImpact = next() * 2.0f;
            s.poseDeviationDegrees = next() * 40.0f;
            s.pendulumDegrees = next() * 40.0f;
            s.chestTiltRate = next() * 4.0f;
            s.supported = next() < 0.9f;
            s.airborneByDesign = next() < 0.1f;
            s.assistLevel = 0.5f + next() * 0.5f;
            const ModeDecision3D d = rules.decide(mode, s, p, 1.0f / 120.0f);
            if (d.change) {
                bool inTable = false;
                for (const ModeRule3D& rule : table)
                    inTable |= rule.event == d.event && rule.to == d.to && (rule.from & modeBit3D(mode)) != 0;
                expect(inTable, "toda troca vem de uma linha da tabela para o modo atual");
                expect(d.event != ModeEvent3D::None, "toda troca tem um evento (o motivo)");
                expect(std::isfinite(d.value) && std::isfinite(d.threshold), "o motivo tem valores finitos");
                rules.noteTransition(mode, d, p);
                mode = d.to;
            }
            const bool guided = mode == BodyModeKind3D::Animated || mode == BodyModeKind3D::Returning
                || mode == BodyModeKind3D::Bridge;
            if (guided && (s.contacts.impact || s.grabbed))
                expect(d.change, "impacto ou agarrar sempre tira o corpo de um modo guiado");
            // O "fisico" volta sozinho pela calma (fora desta tabela): simula.
            if (mode == BodyModeKind3D::Physical && rules.returnAllowed(p) && next() < 0.05f)
                mode = BodyModeKind3D::Bridge;
            if (mode == BodyModeKind3D::Bridge && next() < 0.02f) mode = BodyModeKind3D::Animated;
            expect(rules.lockoutLeft() >= 0.0f && rules.lockoutLeft() <= p.abortLockoutSeconds,
                "a trava fica entre 0 e o maximo");
        }
    }
}

void turnMoveTests() {
    const float dt = 1.0f / 120.0f;
    // Reto: nunca dispara.
    {
        DynamicTurnMove3D move;
        for (int i = 0; i < 240; ++i) move.update({ { 4.0f, 0.0f, 0.0f }, { 4.0f, 0.0f, 0.0f }, dt });
        expect(move.output().state == TurnMoveState3D::Idle && move.output().weight == 0.0f && !move.corrections().active(),
            "sem mudar de direcao, a manobra nao dispara e nao ha correcao");
    }
    // Devagar: nao dispara (abaixo da velocidade minima).
    {
        DynamicTurnMove3D move;
        for (int i = 0; i < 240; ++i) move.update({ { 0.0f, 0.9f, 0.0f }, { 0.9f, 0.0f, 0.0f }, dt });
        expect(move.output().state == TurnMoveState3D::Idle, "andando devagar a virada nao dispara");
    }
    // Uma virada para a esquerda e a espelhada, com o corpo girando para o
    // rumo pedido a 3 rad/s (como o corpo real se realinha).
    struct Run {
        TurnMoveOutput3D early;
        PhysicalCorrections3D earlyCorrections;
        float peakLean = 0.0f, minScale = 1.0f, firstWeightAbove = -1.0f;
        bool sawPlant = false, sawCut = false, sawExit = false, finished = false;
    };
    const auto simulate = [&](float side, float speed, float degrees) {
        DynamicTurnMove3D move;
        // (O pe de fora e um ajuste desligado por padrao: ligado aqui.)
        move.settings().outsideSideBase = 0.14f;
        move.settings().outsideShort = 0.10f;
        Run run;
        const float target = side * degrees * 3.14159265f / 180.0f;
        float heading = 0.0f;
        for (int i = 0; i < 480; ++i) {
            const float t = static_cast<float>(i) * dt;
            if (i > 20) heading += std::clamp(target - heading, -3.0f * dt, 3.0f * dt);
            const Vec3 body { speed * std::cos(heading), speed * std::sin(heading), 0.0f };
            const Vec3 request { speed * std::cos(target), speed * std::sin(target), 0.0f };
            move.update({ i < 20 ? Vec3 { speed, 0.0f, 0.0f } : request, body, dt });
            if (std::abs(move.corrections().lateralLean) > std::abs(run.peakLean)) run.peakLean = move.corrections().lateralLean;
            run.minScale = std::min(run.minScale, move.output().speedScale);
            run.sawPlant = run.sawPlant || move.output().state == TurnMoveState3D::Plant;
            run.sawCut = run.sawCut || move.output().state == TurnMoveState3D::Cut;
            run.sawExit = run.sawExit || move.output().state == TurnMoveState3D::Exit;
            if (run.firstWeightAbove < 0.0f && move.output().weight > 0.5f) run.firstWeightAbove = t - 20.0f * dt;
            if (i == 40) { run.early = move.output(); run.earlyCorrections = move.corrections(); }
        }
        run.finished = move.output().state == TurnMoveState3D::Idle && move.output().weight == 0.0f;
        return run;
    };
    const Run left = simulate(1.0f, 4.0f, 90.0f);
    const Run right = simulate(-1.0f, 4.0f, 90.0f);
    expect(left.early.side > 0.0f && right.early.side < 0.0f, "o lado da virada: esquerda positivo, direita negativo");
    expect(left.peakLean > 0.05f && right.peakLean < -0.05f && std::abs(left.peakLean + right.peakLean) < 1e-4f,
        "inclina para dentro da curva, espelhado entre esquerda e direita");
    expect(left.minScale < 0.8f && left.minScale > 0.3f, "a freada do plantio baixa a velocidade pedida, sem zerar");
    expect(left.sawPlant && left.sawCut && left.sawExit && left.finished, "passa por plantio, corte e saida e termina");
    expect(left.firstWeightAbove >= 0.0f && left.firstWeightAbove < 0.20f,
        "a manobra entra logo (peso acima de 0,5 em menos de 0,2 s): em corrida o corpo se realinha em ~0,5 s");
    expect(left.earlyCorrections.footwork && left.earlyCorrections.feet[1].touchdownOffset.y < 0.0f
            && left.earlyCorrections.feet[0].touchdownOffset.lengthSquared() < 1e-9f,
        "virando a esquerda o pe de fora (o direito) pousa mais aberto, o outro nao muda");
    expect(right.earlyCorrections.feet[0].touchdownOffset.y > 0.0f && right.earlyCorrections.feet[1].touchdownOffset.lengthSquared() < 1e-9f,
        "virando a direita o pe de fora e o esquerdo");
    // Sprint: tambem dispara, e termina.
    const Run sprint = simulate(1.0f, 8.0f, 135.0f);
    expect(sprint.peakLean > 0.05f && sprint.finished, "o sprint tambem dispara e termina");
    // Intensidade zero: nunca dispara.
    {
        DynamicTurnMove3D move;
        move.settings().amount = 0.0f;
        for (int i = 0; i < 120; ++i) move.update({ { 0.0f, 4.0f, 0.0f }, { 4.0f, 0.0f, 0.0f }, dt });
        expect(move.output().state == TurnMoveState3D::Idle, "intensidade zero desliga a manobra");
    }
}

} // namespace

int main() {
    try {
        contactClassTests();
        modeRuleTests();
        pendulumTests();
        referenceFrameTests();
        correctionTests();
        turnMoveTests();
        contractTests();
        phaseLockTests();
        modeRulePropertyTests();
        std::cout << "athlete modules: " << g_checks << " checagens, tudo certo\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "athlete modules: FALHA: " << error.what() << '\n';
        return 1;
    }
}
