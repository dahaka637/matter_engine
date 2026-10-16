// Os testes dos modulos novos do atleta (a reescrita da locomocao; vault:
// 04_locomocao/reescrita_da_locomocao.md). Modulos puros: unidade, a tabela de
// trocas linha a linha e propriedades com sinais aleatorios.
#include "Engine/Athlete/Modes/ContactClasses3D.hpp"
#include "Engine/Athlete/Modes/ModeRules3D.hpp"
#include "Engine/Athlete/Legacy/LegacyReference3D.hpp"
#include "Engine/Athlete/Reference/ReferenceFrame3D.hpp"

#include <cmath>
#include <cstdint>
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

} // namespace

int main() {
    try {
        contactClassTests();
        modeRuleTests();
        pendulumTests();
        referenceFrameTests();
        modeRulePropertyTests();
        std::cout << "athlete modules: " << g_checks << " checagens, tudo certo\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "athlete modules: FALHA: " << error.what() << '\n';
        return 1;
    }
}
