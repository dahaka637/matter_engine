// Sonda dos atletas (rework de 10/10/2026): o MESMO runtime do laboratorio
// (AthleteRuntime3D) com a fisica de verdade (Jolt), sem janela. Roteiros de
// teclas, bola e empurroes; mede o que importa para o novo controle e grava o
// dump que o MatterModelInspector desenha com o jogador de verdade:
//   MatterModelInspector quadros --dump <arquivo> --cenario <nome>
//
// uso: MatterAthleteProbe [--cenario nome] [--dump arquivo] [--cada N] [--lista]
//      [--fisico] (comeca no modo fisico, como ao assumir no laboratorio)
//      [--ativo 0|1] (o Corpo Ativo; 1 padrao)  [--pegadas 0|1] (locomocao por
//      pegadas; 0 padrao)  [--passos 0|1] (passos de equilibrio no Ativo)
//      [--resposta %] (a resposta da capsula no padrao)  [--teste] (os contratos)
//      [--total] (o modo Fisico total)  [--pes %] (a ajuda nos pes)  [--guia %] (a guiagem
//      de ossos do Fisico total)  [--espaco 0|1] (as pernas nunca se atravessam; 0 padrao)
//      [--volta-suave 0|1] (a volta do fisico ao padrao suave, plano v4; 1 padrao)
//      [--tabela 0|1] (as trocas de modo pela tabela e as classes de contato, a reescrita R3; 1 padrao)
//      [--velocidades-padrao 0|1] (no fisico, a tabela de velocidades do padrao; a reescrita R5)
//      [--sem-capsula] (o atleta comandado pelo slot, sem a capsula do jogador: o caminho do treino)
//      [--corpo cm,kg] (o atleta de outro corpo: 170 a 200 cm, 60 a 100 kg; o dump ganha "corpo <m> <kg>")
//      XDESLOCA=m: a largada da capsula deslocada em x (a sensibilidade caotica)
//      XCUSTO=1: o custo do runtime por tick (p50/p95/p99, o pior tick)
//      [--paridade [N]] (a sonda contra os mundos de treino, bit a bit; N roteiros, 0 todos)
//      [--linha-de-base [threads]] (o classico por nivel de ajuda num lote de mundos, e a vazao)
//      [--ajuda-parado %] [--ajuda-andando %]
//      [--contato-ms ms] [--forca-ms ms] [--gesto-ms ms] (os tempos do toque: 350, 400 e 700 padrao)
// Sem --cenario roda todos.
#include "athlete_probe/ProbeShared.hpp"
#include "Engine/Athlete/Policy/LocomotionPolicy3D.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/Athlete/Training/LocomotionBatch3D.hpp"
#include "Engine/Athlete/Training/LocomotionWorld3D.hpp"
#include "Engine/Athlete/Reference/BodyForecast3D.hpp"
#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Objects/ObjectAsset3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace MatterEngine;
using namespace MatterEngine::Probe;

namespace MatterEngine::Probe {

std::vector<Scenario> scenarios() {
    std::vector<Scenario> list;
    list.push_back({ "parado", 3.0f, [](float) { return Keys {}; } });
    list.push_back({ "arranca", 3.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } });
    // (O analogico pela metade e por um terco: o trote e o andar.)
    list.push_back({ "anda", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 0.33f : 0.0f; return k; } });
    list.push_back({ "trota", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 0.5f : 0.0f; return k; } });
    list.push_back({ "sprint", 4.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = true; return k; } });
    list.push_back({ "inverte", 4.0f, [](float t) {
        Keys k;
        k.forward = t < 0.3f ? 0.0f : t < 2.0f ? 1.0f : -1.0f;
        return k; } });
    list.push_back({ "zigue", 4.5f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.left = t < 0.3f ? 0.0f : std::fmod(t, 1.0f) < 0.5f ? 1.0f : -1.0f;
        k.sprint = true;
        return k; } });
    list.push_back({ "lados", 4.0f, [](float t) { Keys k; k.left = t < 0.3f ? 0.0f : t < 2.0f ? 1.0f : -1.0f; return k; } });
    // Os roteiros do efeito dos ganchos do RL (so com XEFEITO: a lista de
    // sempre nao muda): andar 6 s para a frente e de lado, a passo.
    if (std::getenv("XEFEITO") != nullptr) {
        list.push_back({ "efeito_frente", 6.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 0.33f : 0.0f; return k; } });
        list.push_back({ "efeito_lado", 6.0f, [](float t) { Keys k; k.left = t > 0.3f ? 0.33f : 0.0f; return k; } });
        list.push_back({ "efeito_parado", 6.0f, [](float) { return Keys {}; } });
    }
    list.push_back({ "corte", 4.0f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.sprint = true;
        if (t > 2.0f) { k.forward = 0.0f; k.left = 1.0f; }
        return k; } });
    list.push_back({ "para", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f && t < 1.8f ? 1.0f : 0.0f; return k; } });
    list.push_back({ "para_sprint", 3.5f, [](float t) {
        Keys k;
        k.forward = t > 0.3f && t < 2.0f ? 1.0f : 0.0f;
        k.sprint = true;
        return k; } });
    list.push_back({ "diagonal", 3.0f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.left = t > 0.3f ? 1.0f : 0.0f;
        return k; } });
    list.push_back({ "costas", 3.0f, [](float t) { Keys k; k.forward = t > 0.3f ? -1.0f : 0.0f; return k; } });
    list.push_back({ "gira", 3.0f, [](float t) { Keys k; k.cameraYaw = std::min(1.0f, std::max(0.0f, t - 0.5f)) * Pi * 0.75f; return k; } });
    list.push_back({ "pulo", 3.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.jump = t > 1.2f && t < 1.22f; return k; } });
    list.push_back({ "carrinho", 3.5f, [](float t) {
        Keys k;
        k.forward = t > 0.3f ? 1.0f : 0.0f;
        k.sprint = true;
        k.crouch = t > 1.6f && t < 1.7f;
        return k; } });
    {
        Scenario s { "caixa_leve", 3.0f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 6.0f, { 0.15f, 0.15f, 0.15f }, { 0.0f, 1.2f, 1.2f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    // Os gestos de bola (24/10, a reescrita: a forca no pedido, o gesto so
    // animacao): a bola posta em relacao a pelve (x a frente, y a esquerda no
    // rumo do corpo) e um pedido na mesma hora - parado, trotando e correndo,
    // o toque e o chute; conduzindo (o toque de novo sempre que a bola chega
    // perto); e a bola fora do alcance (nada sai).
    {
        struct Gesture { const char* name; float forward; bool sprint; BallActionKind3D kind; float power; Vec3 ball; };
        static const Gesture gestures[] = {
            { "gesto_parado_toque", 0.0f, false, BallActionKind3D::Dribble, 0.4f, { 0.15f, -0.32f, 0.11f } },
            { "gesto_parado_toque_tras", 0.0f, false, BallActionKind3D::Dribble, 0.4f, { -0.15f, 0.20f, 0.11f } },
            { "gesto_parado_chute", 0.0f, false, BallActionKind3D::Shot, 0.8f, { 0.25f, 0.18f, 0.11f } },
            { "gesto_parado_chute_longe", 0.0f, false, BallActionKind3D::Shot, 0.6f, { 0.25f, -0.55f, 0.11f } },
            { "gesto_trote_toque", 1.0f, false, BallActionKind3D::Dribble, 0.4f, { 0.40f, 0.12f, 0.11f } },
            { "gesto_trote_chute", 1.0f, false, BallActionKind3D::Shot, 0.9f, { 0.45f, -0.15f, 0.11f } },
            { "gesto_corre_toque", 1.0f, true, BallActionKind3D::Dribble, 0.4f, { 0.50f, -0.10f, 0.11f } },
            { "gesto_corre_chute", 1.0f, true, BallActionKind3D::Shot, 1.0f, { 0.55f, 0.15f, 0.11f } },
        };
        for (const Gesture& g : gestures) {
            const float forward = g.forward;
            const bool sprint = g.sprint;
            const BallActionKind3D kind = g.kind;
            const float power = g.power;
            Scenario s { g.name, 2.6f, [=](float t) {
                Keys k;
                k.forward = t > 0.2f ? forward : 0.0f;
                k.sprint = sprint;
                if (t >= 1.2f && t < 1.2f + Dt) { k.ball = kind; k.power = power; }
                return k; } };
            s.ball = true;
            s.ballStart = { 0.0f, 8.0f, 0.11f };
            s.placeAt = 1.2f;
            s.placeOffset = g.ball;
            list.push_back(s);
            // E parando 0,6 s depois do gesto, ate assentar (os pes voltam ao
            // lugar?).
            Scenario stop = s;
            stop.name = std::string(g.name) + "_para";
            stop.seconds = 4.4f;
            stop.keys = [=](float t) {
                Keys k;
                k.forward = t > 0.2f && t < 1.8f ? forward : 0.0f;
                k.sprint = sprint;
                if (t >= 1.2f && t < 1.2f + Dt) { k.ball = kind; k.power = power; }
                return k; };
            list.push_back(stop);
        }
        Scenario far { "gesto_fora", 2.0f, [](float t) {
            Keys k;
            if (t >= 1.0f && t < 1.0f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; }
            return k; } };
        far.ball = true;
        far.ballStart = { 1.6f, 0.0f, 0.11f };
        list.push_back(far);
        Scenario drive { "gesto_conduz", 6.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        drive.ball = true;
        drive.ballStart = { 0.55f, 0.0f, 0.11f };
        drive.autoDistance = 0.8f;
        drive.autoKind = BallActionKind3D::Dribble;
        drive.autoCooldown = 0.45f;
        drive.autoPower = 0.2f;
        list.push_back(drive);
        // O teste sorteado (24/10, o "perna deslocada" do usuario): teclas,
        // corrida, giros da camera, toques e chutes sorteados por 30 s, com
        // pausas paradas longas - em cada pausa os pes tem que voltar ao lugar.
        for (std::uint32_t seed = 1; seed <= 12; ++seed) {
            std::uint32_t state = seed * 2654435761u + 12345u;
            const auto next = [&state]() {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                return static_cast<float>(state >> 8) * (1.0f / 16777216.0f);
            };
            struct Segment { float at, yawFrom, yawTo; float forward, left; bool sprint; };
            struct Request { float at; BallActionKind3D kind; float power; };
            std::vector<Segment> plan;
            std::vector<Request> requests;
            Scenario fuzz { "gesto_sorteio_" + std::to_string(seed), 30.0f, nullptr };
            float time = 0.3f, yaw = 0.0f;
            int count = 0;
            while (time < 27.0f) {
                Segment segment { time, yaw, yaw, 0.0f, 0.0f, false };
                const bool pause = ++count % 4 == 0;
                if (!pause && next() > 0.35f) {
                    const float angle = static_cast<float>(static_cast<int>(next() * 8.0f)) * 0.25f * Pi;
                    segment.forward = std::round(std::cos(angle));
                    segment.left = std::round(std::sin(angle));
                    segment.sprint = next() < 0.3f;
                }
                if (!pause) {
                    yaw += (next() - 0.5f) * 2.4f;
                    segment.yawTo = yaw;
                }
                plan.push_back(segment);
                const float length = pause ? 2.2f : 0.5f + next();
                if (next() < 0.75f) {
                    const float when = time + 0.1f + next() * 0.3f;
                    const Vec3 offset { -0.15f + 0.6f * next(), -0.5f + next(), 0.11f };
                    const BallActionKind3D kind = next() < 0.3f ? BallActionKind3D::Shot : BallActionKind3D::Dribble;
                    fuzz.placements.push_back({ when, offset });
                    requests.push_back({ when + Dt, kind, kind == BallActionKind3D::Shot ? 0.3f + 0.7f * next() : 0.2f + 0.4f * next() });
                }
                time += length;
            }
            plan.push_back({ time, yaw, yaw, 0.0f, 0.0f, false });
            fuzz.keys = [plan, requests](float t) {
                Keys k;
                const Segment* now = &plan.front();
                for (const Segment& segment : plan)
                    if (segment.at <= t) now = &segment;
                if (t < plan.front().at) return k;
                k.forward = now->forward;
                k.left = now->left;
                k.sprint = now->sprint;
                k.cameraYaw = now->yawFrom + (now->yawTo - now->yawFrom) * std::clamp((t - now->at) / 0.4f, 0.0f, 1.0f);
                for (const Request& request : requests)
                    if (t >= request.at && t < request.at + Dt) { k.ball = request.kind; k.power = request.power; }
                return k; };
            fuzz.ball = true;
            fuzz.ballStart = { 0.0f, 8.0f, 0.11f };
            list.push_back(fuzz);
        }
        // A gravidade contra a real (24/10, o usuario achou baixa): a bola solta
        // de 2 m (queda livre: 0,639 s; quique FIFA 1,35 a 1,55 m), a bola
        // lancada a 20 m/s a 30 graus (no vacuo: 2,04 s e 35,3 m) e os pulos
        // parados (toque, medio, carregado) - a cápsula Jolt e a bola inflada.
        // (XGRAVIDADE imprime as medidas.)
        {
            Scenario drop { "gravidade_bola", 3.0f, [](float) { return Keys {}; } };
            drop.ball = true;
            drop.ballStart = { 3.0f, 0.0f, 2.0f + 0.1098f };
            list.push_back(drop);
            Scenario thrown { "gravidade_chute", 3.5f, [](float) { return Keys {}; } };
            thrown.ball = true;
            thrown.ballStart = { 3.0f, 0.0f, 0.1098f + 0.01f };
            thrown.ballVelocity = { 17.32f, 0.0f, 10.0f };
            list.push_back(thrown);
            list.push_back({ "gravidade_caixa", 2.0f, [](float) { return Keys {}; } });
            for (const float hold : { 0.03f, 0.30f, 0.60f }) {
                Scenario jump { std::string("gravidade_pulo_") + (hold < 0.1f ? "toque" : hold < 0.5f ? "medio" : "cheio"), 2.6f,
                    [hold](float t) { Keys k; k.jump = t > 1.0f && t < 1.0f + hold; return k; } };
                list.push_back(jump);
            }
        }
        // A bola alta e a recepcao (24/10, pedidos do usuario): a bola posta na
        // altura do joelho com o pedido na hora (o pe vai la), e a bola vindo
        // para ele - rolando de frente, mais devagar, de lado, com ele trotando,
        // o chute fraco e a bola alta vindo -, com o pedido quando ela chega a
        // 0,6 m (a forca a para junto do corpo). RECEPCAO imprime as medidas.
        {
            struct High { const char* name; float forward; bool sprint; BallActionKind3D kind; float power; Vec3 ball; };
            static const High highs[] = {
                { "gesto_alta_parado_toque", 0.0f, false, BallActionKind3D::Dribble, 0.4f, { 0.30f, 0.20f, 0.65f } },
                { "gesto_alta_parado_chute", 0.0f, false, BallActionKind3D::Shot, 0.8f, { 0.35f, -0.15f, 0.70f } },
                { "gesto_alta_trote_toque", 1.0f, false, BallActionKind3D::Dribble, 0.4f, { 0.40f, -0.18f, 0.55f } },
                { "gesto_alta_corre_chute", 1.0f, true, BallActionKind3D::Shot, 0.9f, { 0.50f, 0.15f, 0.60f } },
                { "gesto_alta_parado_toque_longe", 0.0f, false, BallActionKind3D::Dribble, 0.4f, { 0.45f, 0.16f, 0.60f } },
                // Levantar a bola (24/10, o botao do meio): parado sobe reta; andando,
                // a frente (o chapeu).
                { "gesto_levantar_parado", 0.0f, false, BallActionKind3D::Raise, 0.6f, { 0.30f, 0.10f, 0.11f } },
                { "gesto_levantar_andando", 0.6f, false, BallActionKind3D::Raise, 0.7f, { 0.40f, -0.08f, 0.11f } },
            };
            for (const High& g : highs) {
                const float forward = g.forward;
                const bool sprint = g.sprint;
                const BallActionKind3D kind = g.kind;
                const float power = g.power;
                Scenario s { g.name, 2.6f, [=](float t) {
                    Keys k;
                    k.forward = t > 0.2f ? forward : 0.0f;
                    k.sprint = sprint;
                    if (t >= 1.2f && t < 1.2f + Dt) { k.ball = kind; k.power = power; }
                    return k; } };
                s.ball = true;
                s.ballStart = { 0.0f, 8.0f, 0.11f };
                s.placeAt = 1.2f;
                s.placeOffset = g.ball;
                list.push_back(s);
            }
            struct Incoming { const char* name; float forward; BallActionKind3D kind; float power; Vec3 from, velocity; };
            static const Incoming incoming[] = {
                { "recepcao_frente", 0.0f, BallActionKind3D::Dribble, 0.2f, { 3.0f, 0.10f, 0.11f }, { -8.0f, 0.0f, 0.0f } },
                { "recepcao_frente_lenta", 0.0f, BallActionKind3D::Dribble, 0.2f, { 2.0f, -0.12f, 0.11f }, { -5.0f, 0.0f, 0.0f } },
                { "recepcao_lado", 0.0f, BallActionKind3D::Dribble, 0.2f, { 0.25f, 3.0f, 0.11f }, { 0.0f, -7.0f, 0.0f } },
                { "recepcao_trote", 1.0f, BallActionKind3D::Dribble, 0.2f, { 5.0f, 0.12f, 0.11f }, { -6.0f, 0.0f, 0.0f } },
                { "recepcao_chute_fraco", 0.0f, BallActionKind3D::Shot, 0.15f, { 3.0f, -0.10f, 0.11f }, { -8.0f, 0.0f, 0.0f } },
                { "recepcao_alta", 0.0f, BallActionKind3D::Dribble, 0.2f, { 1.6f, 0.08f, 0.80f }, { -5.0f, 0.0f, 0.5f } },
                // O peito e a cabeca (24/10, pedido do usuario): a bola chegando na
                // altura do tronco (~1,3 m no aperto) e da cabeca (~1,95 m).
                { "peito_dominio", 0.0f, BallActionKind3D::Dribble, 0.3f, { 1.8f, 0.05f, 1.30f }, { -6.0f, 0.0f, 1.0f } },
                { "peito_passe", 0.0f, BallActionKind3D::Shot, 0.6f, { 1.8f, 0.05f, 1.30f }, { -6.0f, 0.0f, 1.0f } },
                { "cabeceio_toque", 0.0f, BallActionKind3D::Dribble, 0.5f, { 1.8f, -0.05f, 1.95f }, { -6.0f, 0.0f, 1.0f } },
                { "cabeceio_chute", 0.0f, BallActionKind3D::Shot, 1.0f, { 1.8f, -0.05f, 1.95f }, { -6.0f, 0.0f, 1.0f } },
                { "cabeceio_correndo", 1.0f, BallActionKind3D::Shot, 0.7f, { 5.0f, 0.0f, 1.95f }, { -6.0f, 0.0f, 1.2f } },
            };
            // A bola em balao caindo na frente dele (o teste natural sozinho no
            // laboratorio): solta a 1,5 m quase parada, o pedido com ela a ~0,6 m
            // de altura e meio metro a frente. (Colada nele, a 22 cm, ela ja
            // raspa a coxa caindo e qualquer gesto de pe a atravessa.)
            {
                Scenario lob { "recepcao_balao", 3.0f, [](float t) {
                    Keys k;
                    if (t >= 1.43f && t < 1.43f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.2f; }
                    return k; } };
                lob.ball = true;
                lob.ballStart = { 0.0f, 8.0f, 0.11f };
                lob.placements.push_back({ 1.0f, { 0.75f, 0.05f, 1.5f } });
                lob.placeVelocity = { -0.6f, 0.0f, -0.01f };
                list.push_back(lob);
            }
            // (A varredura da recepcao no ambiente: XRECEP_POT a barra, XRECEP_TECLA "frente,lado" seguradas,
            // XRECEP_VEL a escala da bola e XRECEP_LADO o quanto ela passa de lado, em m.)
            const auto knob = [](const char* name, float fallback) {
                const char* value = std::getenv(name);
                return value != nullptr && *value != '\0' ? static_cast<float>(std::atof(value)) : fallback;
            };
            float keyForward = 0.0f, keyLeft = 0.0f;
            if (const char* keys = std::getenv("XRECEP_TECLA"); keys != nullptr)
                std::sscanf(keys, "%f,%f", &keyForward, &keyLeft);
            const bool keysGiven = std::getenv("XRECEP_TECLA") != nullptr;
            for (const Incoming& g : incoming) {
                const float forward = keysGiven ? keyForward : g.forward;
                const float left = keyLeft;
                Scenario s { g.name, 3.0f, [=](float t) { Keys k; k.forward = t > 0.2f ? forward : 0.0f;
                    k.left = t > 0.2f ? left : 0.0f; return k; } };
                s.ball = true;
                s.ballStart = { 0.0f, 8.0f, 0.11f };
                s.placements.push_back({ 1.0f, g.from + Vec3 { 0.0f, knob("XRECEP_LADO", 0.0f), 0.0f } });
                s.placeVelocity = g.velocity * knob("XRECEP_VEL", 1.0f);
                s.autoDistance = 0.6f;
                s.autoKind = g.kind;
                s.autoPower = knob("XRECEP_POT", g.power);
                list.push_back(s);
            }
        }
        // Toques alternados rapidos (esquerda, direita a cada 0,15 s): o gesto
        // da outra perna termina logo (as duas no ar, nenhum pe apoiava).
        {
            Scenario quick { "gesto_rapido", 3.0f, [](float t) {
                Keys k;
                for (int i = 0; i < 8; ++i)
                    if (t >= 1.0f + 0.15f * static_cast<float>(i) + Dt && t < 1.0f + 0.15f * static_cast<float>(i) + 2.0f * Dt) {
                        k.ball = BallActionKind3D::Dribble;
                        k.power = 0.3f;
                    }
                return k; } };
            quick.ball = true;
            quick.ballStart = { 0.0f, 8.0f, 0.11f };
            for (int i = 0; i < 8; ++i)
                quick.placements.push_back({ 1.0f + 0.15f * static_cast<float>(i), { 0.15f, i % 2 == 0 ? 0.25f : -0.25f, 0.11f } });
            list.push_back(quick);
        }
        // Brincando com a bola (o uso do usuario no laboratorio): toques
        // seguidos sempre que ela chega perto, a camera girando para os dois
        // lados, e no fim parado ate assentar (os pes tem que voltar ao lugar).
        for (int variant = 0; variant < 4; ++variant) {
            const bool walk = variant % 2 == 1, shots = variant >= 2;
            Scenario play { std::string("gesto_brinca") + (shots ? "_chute" : "") + (walk ? "_anda" : ""), 9.0f,
                [walk](float t) {
                    Keys k;
                    if (walk && t > 0.3f && t < 6.0f) k.forward = 0.6f;
                    if (t > 0.5f && t < 6.0f) k.cameraYaw = 1.1f * std::sin(1.3f * (t - 0.5f));
                    else if (t >= 6.0f) k.cameraYaw = 1.1f * std::sin(1.3f * 5.5f);
                    return k; } };
            play.ball = true;
            play.ballStart = { 0.35f, 0.15f, 0.11f };
            play.autoDistance = 0.8f;
            play.autoKind = shots ? BallActionKind3D::Shot : BallActionKind3D::Dribble;
            play.autoCooldown = shots ? 0.9f : 0.5f;
            play.autoPower = shots ? 0.3f : 0.15f;
            list.push_back(play);
        }
    }
    {
        // Os casos do teste do usuario (10/10, noite): a caixa forte derruba,
        // ele levanta - parado, e com as teclas apertadas no fim do levantar
        // (andando e correndo). O encostao leve andando e correndo.
        Scenario parado { "queda_levanta", 10.0f, [](float) { return Keys {}; } };
        parado.throws.push_back({ 1.0f, 140.0f, { 0.3f, 0.3f, 0.3f }, { 0.0f, 1.5f, 0.9f }, { 0.0f, -9.0f, 0.0f } });
        list.push_back(parado);
        Scenario anda { "queda_levanta_anda", 10.0f, [](float t) { Keys k; k.forward = t > 2.5f ? 0.5f : 0.0f; return k; } };
        anda.throws = parado.throws;
        list.push_back(anda);
        Scenario corre { "queda_levanta_corre", 10.0f, [](float t) { Keys k; k.forward = t > 2.5f ? 1.0f : 0.0f; return k; } };
        corre.throws = parado.throws;
        list.push_back(corre);
        // O levantar de cada jeito de cair (plano v2, fase D): a caixa no
        // quadril de lado (cai de lado), nas costas (de frente, de brucos) e
        // no peito (de costas).
        for (const int newtons : { 700, 1000, 1400, 1900 }) {
            char name[48];
            std::snprintf(name, sizeof(name), "queda_lado_%d", newtons);
            Scenario lado { name, 11.0f, [](float) { return Keys {}; } };
            lado.pushes.push_back({ 0.5f, 0.25f, { 0.0f, -static_cast<float>(newtons), 0.0f } });
            lado.startPhysical = true;
            list.push_back(lado);
        }
        // Quedas em oito direcoes (a caixa no peito vindo de cada lado),
        // parado e correndo: o jeito do corpo no comeco do levantar.
        for (int running = 0; running < 2; ++running)
            for (int k = 0; k < 8; ++k) {
                char name[48];
                std::snprintf(name, sizeof(name), "%s_%03d", running ? "quedacorre" : "queda", k * 45);
                Scenario q { name, 11.0f, [running](float t) { Keys keys; keys.forward = running && t > 0.3f && t < 1.4f ? 1.0f : 0.0f; return keys; } };
                const float angle = static_cast<float>(k) * Pi * 0.25f;
                const Vec3 from { std::cos(angle), std::sin(angle), 0.0f };
                const Vec3 at = running ? Vec3 { 4.6f, 0.0f, 0.0f } : Vec3 {};
                q.throws.push_back({ running ? 1.5f : 1.0f, 140.0f, { 0.3f, 0.3f, 0.3f },
                    at + from * 1.4f + Vec3 { 0.0f, 0.0f, 1.0f }, from * -9.0f });
                list.push_back(q);
            }
        // Terreno (plano v2, fase E): um degrau de 10, 18, 25 e 35 cm (uma
        // plataforma a partir de x = 3 m), a escada (8 degraus de 17 por 30
        // cm) e a rampa de 12 graus, andando e correndo.
        {
            struct Gait { const char* key; float forward; };
            static const Gait gaits[2] = { { "anda", 0.5f }, { "corre", 1.0f } };
            for (const Gait& gait : gaits) {
                const float forward = gait.forward;
                const auto walk = [forward](float t) { Keys k; k.forward = t > 0.3f ? forward : 0.0f; return k; };
                for (const int centimeters : { 10, 18, 25, 35 }) {
                    char name[48];
                    std::snprintf(name, sizeof(name), "degrau_%02d_%s", centimeters, gait.key);
                    Scenario step { name, 4.0f, walk };
                    const float h = 0.01f * static_cast<float>(centimeters);
                    step.statics.push_back({ { 13.0f, 0.0f, 0.5f * h }, { 10.0f, 3.0f, 0.5f * h }, {} });
                    list.push_back(step);
                }
                {
                    char name[48];
                    std::snprintf(name, sizeof(name), "escada_%s", gait.key);
                    Scenario stairs { name, 5.0f, walk };
                    for (int k = 0; k < 8; ++k) {
                        const float from = 3.0f + 0.30f * static_cast<float>(k);
                        stairs.statics.push_back({ { 0.5f * (from + 23.0f), 0.0f, 0.17f * static_cast<float>(k) + 0.085f },
                            { 0.5f * (23.0f - from), 3.0f, 0.085f }, {} });
                    }
                    list.push_back(stairs);
                }
                {
                    char name[48];
                    std::snprintf(name, sizeof(name), "rampa_%s", gait.key);
                    Scenario ramp { name, 5.0f, walk };
                    const float angle = 12.0f * Pi / 180.0f, length = 40.0f, half = 0.25f;
                    const Vec3 along { std::cos(angle), 0.0f, std::sin(angle) };
                    const Vec3 normal { -std::sin(angle), 0.0f, std::cos(angle) };
                    const Vec3 top = Vec3 { 3.0f, 0.0f, 0.0f } + along * (0.5f * length);
                    ramp.statics.push_back({ top - normal * half, { 0.5f * length, 3.0f, half },
                        Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -angle) });
                    list.push_back(ramp);
                }
            }
        }
        // A parede: correndo contra ela, e pulando contra ela (cair de pe e
        // recobrar a postura).
        Scenario paredeCorre { "parede_corre", 5.0f, [](float t) { Keys k; k.forward = t > 0.3f && t < 2.5f ? 1.0f : 0.0f; return k; } };
        paredeCorre.wallCenter = { 5.5f, 0.0f, 1.5f };
        paredeCorre.wallHalf = { 0.2f, 3.0f, 1.5f };
        list.push_back(paredeCorre);
        Scenario paredeEmpurra { "parede_empurra", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        paredeEmpurra.wallCenter = paredeCorre.wallCenter;
        paredeEmpurra.wallHalf = paredeCorre.wallHalf;
        list.push_back(paredeEmpurra);
        // A rede (25/10): correndo no sprint para o fundo de uma trave com
        // rede (a boca para o atleta, a linha a 3,5 m): a rede esticada o
        // empurra, ele vira fisico, ela o segura sem deixar passar e ele volta
        // ao padrao; solta as teclas aos 2 s.
        Scenario redeCorre { "rede_corre", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f && t < 2.0f ? 1.0f : 0.0f; k.sprint = true; return k; } };
        list.push_back(redeCorre);
        // Andando e correndo contra a rede sem soltar a tecla (o jogador
        // empurrando a rede): se cai, se fica trocando de modo.
        list.push_back({ "rede_anda", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 0.5f : 0.0f; return k; } });
        list.push_back({ "rede_empurra", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } });
        // Na diagonal para o canto (a rede do lado e a do fundo), e por tras
        // (a trave de costas: a rede do fundo por fora).
        list.push_back({ "rede_canto", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.cameraYaw = 0.75f; return k; } });
        list.push_back({ "rede_atras", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } });
        // Com forca (25/10, o usuario atravessava): no sprint maximo para o
        // fundo, de fora contra a rede do lado (correndo e no sprint maximo) e
        // de carrinho para o fundo.
        list.push_back({ "rede_max", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = k.maxSprint = true; return k; } });
        list.push_back({ "rede_lado_fora", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = true; return k; } });
        list.push_back({ "rede_lado_max", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = k.maxSprint = true; return k; } });
        // De fora pela lateral perto do canto de tras (onde a placa do lado
        // so vai ate a diagonal): a 1,4 e 1,8 m do poste, no sprint maximo, na
        // diagonal para o canto e de carrinho.
        list.push_back({ "rede_lado_fundo", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = true; return k; } });
        list.push_back({ "rede_lado_canto", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = true; return k; } });
        list.push_back({ "rede_lado_canto_max", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = k.maxSprint = true; return k; } });
        list.push_back({ "rede_lado_diag", 10.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; k.sprint = true; k.cameraYaw = -0.35f; return k; } });
        list.push_back({ "rede_lado_carrinho", 6.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f ? 1.0f : 0.0f;
            k.sprint = true;
            const float at = std::getenv("XCARRINHO") ? static_cast<float>(std::atof(std::getenv("XCARRINHO"))) : 0.7f;
            k.crouch = t > at && t < at + 0.1f;
            return k; } });
        list.push_back({ "rede_carrinho", 6.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f ? 1.0f : 0.0f;
            k.sprint = true;
            const float at = std::getenv("XCARRINHO") ? static_cast<float>(std::atof(std::getenv("XCARRINHO"))) : 0.9f;
            k.crouch = t > at && t < at + 0.1f;
            return k; } });
        Scenario paredePulo { "parede_pulo", 5.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f && t < 2.5f ? 1.0f : 0.0f;
            k.jump = t > 1.05f && t < 1.25f;
            return k; } };
        paredePulo.wallCenter = { 5.5f, 0.0f, 1.5f };
        paredePulo.wallHalf = { 0.2f, 3.0f, 1.5f };
        list.push_back(paredePulo);
        // Cai de brucos e ja sai correndo (as teclas desde o chao).
        Scenario frenteCorre { "queda_frente_corre", 11.0f, [](float t) { Keys k; k.forward = t > 3.5f ? 1.0f : 0.0f; return k; } };
        frenteCorre.throws.push_back({ 1.0f, 140.0f, { 0.3f, 0.3f, 0.3f }, { -1.5f, 0.0f, 1.1f }, { 9.0f, 0.0f, 0.0f } });
        list.push_back(frenteCorre);
        // A pancada e as teclas logo depois (andando e correndo): quanto tempo
        // ele anda no fisico antes de voltar ao padrao.
        for (int k = 0; k < 2; ++k) {
            const float forward = k == 0 ? 0.5f : 1.0f;
            Scenario hitMove { k == 0 ? "pancada_anda" : "pancada_corre", 5.0f, [forward](float t) {
                Keys keys;
                keys.forward = t > 1.25f ? forward : 0.0f;
                return keys; } };
            hitMove.throws.push_back({ 1.0f, 40.0f, { 0.25f, 0.25f, 0.25f }, { 0.0f, 1.4f, 1.1f }, { 0.0f, -7.0f, 0.0f } });
            list.push_back(hitMove);
        }
        Scenario frente { "queda_frente", 11.0f, [](float) { return Keys {}; } };
        frente.throws.push_back({ 1.0f, 140.0f, { 0.3f, 0.3f, 0.3f }, { -1.5f, 0.0f, 1.1f }, { 9.0f, 0.0f, 0.0f } });
        list.push_back(frente);
        Scenario costas { "queda_costas", 11.0f, [](float) { return Keys {}; } };
        costas.throws.push_back({ 1.0f, 140.0f, { 0.3f, 0.3f, 0.3f }, { 1.5f, 0.0f, 1.1f }, { -9.0f, 0.0f, 0.0f } });
        list.push_back(costas);
        Scenario encostaAnda { "encostao_andando", 4.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 0.5f : 0.0f; return k; } };
        encostaAnda.throws.push_back({ 1.5f, 6.0f, { 0.15f, 0.15f, 0.15f }, { 3.15f, 1.2f, 1.2f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(encostaAnda);
        Scenario encostaCorre { "encostao_correndo", 4.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        encostaCorre.throws.push_back({ 1.5f, 6.0f, { 0.15f, 0.15f, 0.15f }, { 5.5f, 1.2f, 1.2f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(encostaCorre);
    }
    // Pegadas novas (plano v3, etapa 2): os casos do teste do usuario. O pulo
    // que pousa numa plataforma curta e sai dela correndo, o pulo que raspa o
    // pe numa caixa baixa, a escada com parada e com volta descendo, e os
    // giros bruscos (parado, andando, correndo, no sprint, de lado; a volta
    // de 180 graus correndo).
    {
        const auto jumpRun = [](float jumpAt) {
            return [jumpAt](float t) {
                Keys k;
                k.forward = t > 0.3f ? 1.0f : 0.0f;
                k.jump = t > jumpAt && t < jumpAt + 0.05f;
                return k;
            };
        };
        // O pulo correndo contra caixas de 40 a 70 cm em varias distancias (o
        // caso do usuario: a perna batendo no mundo no pulo).
        for (int hi = 0; hi < 4; ++hi)
            for (int xi = 0; xi < 5; ++xi) {
                const float h = 0.40f + 0.10f * static_cast<float>(hi);
                const float x = 3.4f + 0.3f * static_cast<float>(xi);
                char name[32];
                std::snprintf(name, sizeof(name), "topo_%02d_%02d", static_cast<int>(h * 100.0f + 0.5f),
                    static_cast<int>(x * 10.0f + 0.5f));
                Scenario s { name, 3.0f, jumpRun(0.95f) };
                s.statics.push_back({ { x, 0.0f, 0.5f * h }, { 0.25f, 2.0f, 0.5f * h }, {} });
                list.push_back(s);
            }
        Scenario plataforma { "pulo_plataforma", 3.5f, jumpRun(0.95f) };
        plataforma.statics.push_back({ { 4.4f, 0.0f, 0.2f }, { 0.45f, 2.0f, 0.2f }, {} });
        list.push_back(plataforma);
        Scenario raspa { "pulo_raspa", 3.5f, jumpRun(0.95f) };
        raspa.statics.push_back({ { 3.9f, 0.0f, 0.14f }, { 0.12f, 2.0f, 0.14f }, {} });
        list.push_back(raspa);
        Scenario alta { "pulo_alta", 3.5f, jumpRun(0.95f) };
        alta.statics.push_back({ { 4.6f, 0.0f, 0.3f }, { 0.35f, 2.0f, 0.3f }, {} });
        list.push_back(alta);
        const auto stairs = [](Scenario& scenario) {
            for (int k = 0; k < 8; ++k) {
                const float from = 3.0f + 0.30f * static_cast<float>(k);
                scenario.statics.push_back({ { 0.5f * (from + 23.0f), 0.0f, 0.17f * static_cast<float>(k) + 0.085f },
                    { 0.5f * (23.0f - from), 3.0f, 0.085f }, {} });
            }
        };
        Scenario para { "escada_para", 5.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f && t < 1.9f ? 0.5f : 0.0f;
            return k; } };
        stairs(para);
        list.push_back(para);
        Scenario volta { "escada_volta", 8.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f ? 0.5f : 0.0f;
            k.cameraYaw = std::clamp((t - 4.0f) / 0.6f, 0.0f, 1.0f) * Pi;
            return k; } };
        stairs(volta);
        list.push_back(volta);
        list.push_back({ "gira_rapido", 3.0f, [](float t) {
            Keys k;
            k.cameraYaw = t < 0.5f ? 0.0f : t < 0.75f ? (t - 0.5f) / 0.25f * Pi : t < 1.8f ? Pi
                : t < 2.05f ? Pi * (1.0f - (t - 1.8f) / 0.25f) : 0.0f;
            return k; } });
        // Giros parados extremos (21/10, pedido do usuario: girando muito rapido as
        // pernas embolavam): duas voltas a 720 graus/s e o vaivem de 90 graus.
        list.push_back({ "gira_continuo", 3.0f, [](float t) {
            Keys k;
            k.cameraYaw = std::clamp(t - 0.5f, 0.0f, 1.0f) * 4.0f * Pi;
            return k; } });
        list.push_back({ "gira_vaivem", 3.0f, [](float t) {
            Keys k;
            k.cameraYaw = t < 0.5f ? 0.0f : 0.5f * Pi * std::sin((t - 0.5f) * 2.0f * Pi * 2.0f);
            return k; } });
        struct Turn { const char* name; float forward; float left; bool sprint; float rate; };
        static const Turn turns[5] = { { "anda_gira", 0.5f, 0.0f, false, 3.0f }, { "corre_gira", 1.0f, 0.0f, false, 3.0f },
            { "sprint_gira", 1.0f, 0.0f, true, 2.5f }, { "lado_gira", 0.0f, 1.0f, false, 3.0f },
            { "diagonal_gira", 0.7f, 0.7f, false, -3.0f } };
        for (const Turn& turn : turns) {
            const Turn c = turn;
            list.push_back({ c.name, 4.0f, [c](float t) {
                Keys k;
                k.forward = t > 0.3f ? c.forward : 0.0f;
                k.left = t > 0.3f ? c.left : 0.0f;
                k.sprint = c.sprint;
                k.cameraYaw = std::max(0.0f, t - 1.0f) * c.rate;
                return k; } });
        }
        list.push_back({ "corre_vira180", 4.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f ? 1.0f : 0.0f;
            k.cameraYaw = t > 1.6f ? Pi : 0.0f;
            return k; } });
        list.push_back({ "anda_vira180", 4.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f ? 0.5f : 0.0f;
            k.cameraYaw = t > 1.6f ? Pi : 0.0f;
            return k; } });
        // As viradas bruscas em movimento (XVIRADAS=1; o experimento
        // DynamicTurnMove3D): a camera salta o angulo a 2,2 s, ja na marcha.
        if (std::getenv("XVIRADAS")) {
            // A troca de direcao correndo de lado (a esquerda e logo a direita) e para a frente e para tras.
            list.push_back({ "lados_corre", 4.0f, [](float t) {
                Keys k;
                k.left = t < 0.3f ? 0.0f : t < 2.0f ? 1.0f : -1.0f;
                k.sprint = true;
                return k; } });
            list.push_back({ "inverte_corre", 4.0f, [](float t) {
                Keys k;
                k.forward = t < 0.3f ? 0.0f : t < 2.0f ? 1.0f : -1.0f;
                k.sprint = true;
                return k; } });
            struct Gait { const char* name; float forward; bool sprint; };
            static const Gait gaits[3] = { { "trota", 0.5f, false }, { "corre", 1.0f, false }, { "sprint", 1.0f, true } };
            for (const Gait& gait : gaits)
                for (const int degrees : { 45, 90, 135, 180 })
                    for (const float sign : { 1.0f, -1.0f }) {
                        const Gait g = gait;
                        const float angle = sign * static_cast<float>(degrees) * Pi / 180.0f;
                        char name[64];
                        std::snprintf(name, sizeof name, "virada_%s_%d%s", g.name, degrees, sign > 0.0f ? "e" : "d");
                        list.push_back({ name, 4.5f, [g, angle](float t) {
                            Keys k;
                            k.forward = t > 0.3f ? g.forward : 0.0f;
                            k.sprint = g.sprint;
                            k.cameraYaw = t > 2.2f ? angle : 0.0f;
                            return k; } });
                    }
        }
    }
    {
        Scenario s { "caixa_media", 3.0f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 20.0f, { 0.2f, 0.2f, 0.2f }, { 0.0f, 1.3f, 1.1f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "caixa_forte", 3.5f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 60.0f, { 0.25f, 0.25f, 0.25f }, { 0.0f, 1.4f, 1.1f }, { 0.0f, -6.0f, 0.0f } });
        list.push_back(s);
    }
    // A pancada dupla (a reescrita, R3; o problema 2 do usuario): a caixa forte
    // derruba no fisico e a media chega em instantes diferentes - na volta (a
    // ponte) ela era engolida (so acima de 25 N.s ia ao fisico).
    for (int k = 0; k < 12; ++k) {
        const float at = 2.0f + 0.15f * static_cast<float>(k);
        char name[32];
        std::snprintf(name, sizeof(name), "dupla_%02d", k);
        Scenario s { name, 7.0f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 60.0f, { 0.25f, 0.25f, 0.25f }, { 0.0f, 1.4f, 1.1f }, { 0.0f, -6.0f, 0.0f } });
        s.throws.push_back({ at, 20.0f, { 0.2f, 0.2f, 0.2f }, { 0.0f, 1.3f, 1.1f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "caixa_correndo", 4.2f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.throws.push_back({ 1.5f, 15.0f, { 0.2f, 0.2f, 0.2f }, { 5.5f, 1.3f, 1.1f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_leve", 3.0f, [](float) { return Keys {}; } };
        s.pushes.push_back({ 1.0f, 0.1f, { 0.0f, 350.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_medio", 3.0f, [](float) { return Keys {}; } };
        s.pushes.push_back({ 1.0f, 0.1f, { 0.0f, 900.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_forte", 3.5f, [](float) { return Keys {}; } };
        s.pushes.push_back({ 1.0f, 0.12f, { 0.0f, 2200.0f, 0.0f } });
        list.push_back(s);
    }
    {
        Scenario s { "empurrao_correndo", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.pushes.push_back({ 1.6f, 0.1f, { 0.0f, 500.0f, 0.0f } });
        list.push_back(s);
    }
    return list;
}


// A menor distancia entre dois segmentos (a0-a1 e b0-b1).
float segmentDistance(Vec3 a0, Vec3 a1, Vec3 b0, Vec3 b1) {
    const Vec3 d1 = a1 - a0, d2 = b1 - b0, r = a0 - b0;
    const float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    float s = 0.0f, t = 0.0f;
    if (a <= 1e-9f && e <= 1e-9f) return (a0 - b0).length();
    if (a <= 1e-9f) {
        t = std::clamp(f / e, 0.0f, 1.0f);
    } else {
        const float c = dot(d1, r);
        if (e <= 1e-9f) {
            s = std::clamp(-c / a, 0.0f, 1.0f);
        } else {
            const float b = dot(d1, d2), denom = a * e - b * b;
            s = denom > 1e-9f ? std::clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = std::clamp(-c / a, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = std::clamp((b - c) / a, 0.0f, 1.0f); }
        }
    }
    return ((a0 + d1 * s) - (b0 + d2 * t)).length();
}

// Um pedaco da perna como segmento com raio (a capsula; a caixa do pe pelo
// eixo comprido, com o menor meio-lado de raio).
struct LegPiece {
    Vec3 a, b;
    float radius = 0.05f;
};

LegPiece legPiece(const RagdollProfile3D& profile, const PhysicsBodyState3D& link, std::size_t index) {
    const auto& collider = profile.links[index].collider;
    const Vec3 center = link.position + link.orientation.rotate(collider.localPosition);
    const Quaternion q = (link.orientation * collider.localOrientation).normalized();
    const Vec3 axis = q.rotate({ 1.0f, 0.0f, 0.0f });
    LegPiece piece;
    if (ragdollColliderHasBox3D(collider)) {
        const Vec3 h = collider.boxHalfExtents;
        const float half = std::max(0.0f, h.x - std::min(h.y, h.z));
        piece = { center - axis * half, center + axis * half, std::min(h.y, h.z) };
    } else {
        const float half = ragdollCapsuleHalfSegment3D(collider);
        piece = { center - axis * half, center + axis * half,
            0.5f * (collider.radiusMeters + ragdollCapsuleRadiusAtPositiveX3D(collider)) };
    }
    return piece;
}

float percentile(std::vector<float> values, float q) {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    const std::size_t index = std::min(values.size() - 1, static_cast<std::size_t>(q * static_cast<float>(values.size() - 1)));
    return values[index];
}

void dumpFrame(FILE* file, const RagdollProfile3D& profile, const RagdollState3D& state, float t, int mode) {
    std::fprintf(file, "F %.4f %d 0\n", t, mode);
    for (std::size_t i = 0; i < profile.links.size() && i < state.links.size(); ++i) {
        const auto& collider = profile.links[i].collider;
        const Vec3 center = state.links[i].position + state.links[i].orientation.rotate(collider.localPosition);
        const Quaternion q = (state.links[i].orientation * collider.localOrientation).normalized();
        if (ragdollColliderHasBox3D(collider)) {
            std::fprintf(file, "B %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w, collider.boxHalfExtents.x, collider.boxHalfExtents.y,
                collider.boxHalfExtents.z);
        } else {
            std::fprintf(file, "C %s %.4f %.4f %.4f %.5f %.5f %.5f %.5f %.4f %.4f %.4f\n", profile.links[i].id.c_str(),
                center.x, center.y, center.z, q.x, q.y, q.z, q.w, ragdollCapsuleHalfSegment3D(collider),
                collider.radiusMeters, ragdollCapsuleRadiusAtPositiveX3D(collider));
        }
    }
}


} // namespace MatterEngine::Probe

namespace MatterEngine::Probe {



// (trace: a impressao digital de cada tick do roteiro, para a paridade.)
Metrics runScenario(const ProbeContext& context, const Scenario& scenario, const AthleteRuntimeSettings3D& settings,
    bool startPhysicalArgument, FILE* dump, int every, bool suppressRequests,
    std::vector<AthleteTickDigest3D>* trace, ContractHooks* hooks) {
    const bool startPhysical = startPhysicalArgument || scenario.startPhysical;
    const RagdollProfile3D& profile = *context.profile;
    const AthleteFootGeometry3D footGeometry = resolveAthleteFootGeometry3D(profile, resolveAthleteLegs3D(profile), 0.08f);
    const CharacterGaitSpeeds3D& gaitSpeeds = context.gaitSpeeds;
    const CharacterMotorSettings3D& avatar = context.avatar;
    const AthleteRuntimeAssets3D& runtimeAssets = context.runtimeAssets;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    // (O chao, o corpo e o comeco sao os dos mundos de treino: a paridade.)
    createLocomotionGround3D(*scene);
    {
        std::vector<StaticBox> statics = scenario.statics;
        if (scenario.wallHalf.lengthSquared() > 0.0f) statics.push_back({ scenario.wallCenter, scenario.wallHalf, {} });
        for (const StaticBox& box : statics) createLocomotionStaticBox3D(*scene, { box.center, box.half, box.orientation });
    }
    // A trave com rede (os roteiros rede_*): a boca para -x (o atleta), a
    // linha a 3,5 m, a rede assentada.
    float netBehind = 1e9f, capsuleBehind = 1e9f;
    NetHandle3D goalNet;
    // A linha do gol e a boca: o dentro do gol (atras da linha, dentro dos
    // lados e do teto, ate a barra de tras: com a cunha debaixo do fundo
    // inclinado), para os roteiros de fora.
    Vec3 goalLine;
    Quaternion goalFacing;
    const auto insideGoal = [&](Vec3 p) {
        const Vec3 q = goalFacing.conjugate().rotate(p - goalLine);
        return q.x < -0.1f && q.x > -1.97f + 0.05f && std::abs(q.y) < 3.69f - 0.05f && q.z < 2.44f;
    };
    const bool goalScenario = scenario.name.rfind("rede_", 0) == 0;
    if (goalScenario) {
        std::string error;
        const auto recipe = Modeling::readObjectRecipe(std::string(MATTERENGINE_ASSETS_DIR) + "/objects/trave.json", &error);
        const auto goal = recipe ? Modeling::generateObject(*recipe, &error) : std::nullopt;
        if (goal && goal->net) {
            // (rede_atras: a boca para +x, o quadro de tras a 3,5 m; rede_lado_*:
            // a boca para +y, o lado de fora a 3,5 m, o atleta no alto da rede
            // do lado.)
            const bool behind = scenario.name == "rede_atras";
            const bool outside = scenario.name.rfind("rede_lado", 0) == 0;
            const Quaternion facing = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, behind ? 0.0f : outside ? 0.5f * Pi : Pi);
            // (Ao lado: o atleta passa a 0,6 m do poste; rede_lado_fundo a 1,4,
            // rede_lado_canto* a 1,8, rede_lado_carrinho a 1,6.)
            const float alongSide = scenario.name == "rede_lado_fundo" ? 1.4f
                : scenario.name.rfind("rede_lado_canto", 0) == 0 ? 1.8f : scenario.name == "rede_lado_carrinho" ? 1.6f : 0.6f;
            const Vec3 line = outside ? Vec3 { 3.5f + 0.5f * goal->size.y, alongSide, 0.0f }
                                      : Vec3 { behind ? 3.5f + goal->size.x : 3.5f, 0.0f, 0.0f };
            goalLine = line;
            goalFacing = facing;
            const Vec3 center = line - facing.rotate({ 0.5f * goal->size.x, 0.0f, 0.0f }) + Vec3 { 0.0f, 0.0f, 0.5f * goal->size.z };
            ObjectBodyData3D body = objectBodyData3D(*goal, materials);
            body.body.motionType = PhysicsMotionType3D::Static;
            body.body.position = center;
            body.body.orientation = facing;
            static_cast<void>(scene->createBody(body.body, body.shapes));
            const NetHandle3D net = scene->createNet(objectNetDescription3D(*goal, center, facing, 0.0f));
            goalNet = net;
            scene->settleNet(net, 8.0f);
            for (ObjectBodyData3D& barrier : objectNetBarrier3D(*goal)) {
                barrier.body.position = center;
                barrier.body.orientation = facing;
                static_cast<void>(scene->createBody(barrier.body, barrier.shapes));
            }
            // Alem de qualquer estufada atras da barra de tras: atravessou (por
            // tras: 1,2 m para dentro do quadro de tras).
            // (Um link alem da rede do lado, ou do fundo estufado; a capsula
            // alem do quadro de tras ou dentro do gol pelo lado ou por tras.)
            // (De fora - pelo lado e por tras - vale o dentro do gol, abaixo:
            // perto do canto o pe atras do gol passa da linha do lado.)
            netBehind = outside || behind ? 1e9f : 3.5f + goal->size.x + 0.6f;
            capsuleBehind = outside || behind ? 1e9f : 3.5f + goal->size.x;
        } else {
            std::printf("  %s: sem a trave (%s)\n", scenario.name.c_str(), error.c_str());
        }
    }
    // (--sem-capsula: nenhuma capsula na cena, como num mundo de treino.)
    if (!context.noCapsule)
        scene->createCharacter(
            { std::getenv("XDESLOCA") ? static_cast<float>(std::atof(std::getenv("XDESLOCA"))) : 0.0f, 0.0f, 0.0f },
            avatar);
    const RagdollHandle3D handle = spawnLocomotionAthlete3D(*scene, profile);
    RagdollState3D bodyState = scene->ragdollState(handle);
    std::size_t leftFoot = profile.links.size(), rightFoot = profile.links.size();
    std::size_t chestLink = profile.links.size();
    std::array<std::size_t, 2> thighs { profile.links.size(), profile.links.size() };
    std::array<std::size_t, 2> shins { profile.links.size(), profile.links.size() };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") leftFoot = i;
        if (profile.links[i].id == "RightFoot") rightFoot = i;
        if (profile.links[i].id == "LeftThigh") thighs[0] = i;
        if (profile.links[i].id == "RightThigh") thighs[1] = i;
        if (profile.links[i].id == "Chest") chestLink = i;
        if (profile.links[i].id == "LeftShin") shins[0] = i;
        if (profile.links[i].id == "RightShin") shins[1] = i;
    }
    // A dobra do joelho (graus): o giro da canela em relacao a coxa, a partir
    // da pose de ligacao (pernas retas).
    const auto kneeFlexion = [&](std::size_t side, Quaternion thigh, Quaternion shin) {
        const Quaternion a = (thigh * profile.links[thighs[side]].modelOrientation.conjugate()).normalized();
        const Quaternion b = (shin * profile.links[shins[side]].modelOrientation.conjugate()).normalized();
        const Quaternion relative = (a.conjugate() * b).normalized();
        return 2.0f * std::acos(std::clamp(std::abs(relative.w), 0.0f, 1.0f)) * 57.29578f;
    };
    std::array<float, 2> kneeBefore { -1.0f, -1.0f }, kneeTargetBefore { -1.0f, -1.0f }, kneeRateBefore {};
    // Pegadas novas: o alvo e o modo de cada pe no tick anterior, a
    // velocidade vertical dele (alvo e corpo) e o tempo no ar.
    std::array<FootMode3D, 2> footModeBefore { FootMode3D::Free, FootMode3D::Free };
    std::array<float, 2> footTargetZBefore {}, footTargetVzBefore {}, footBodyVzBefore {};
    // (O tremor: a velocidade vertical trocando de sentido em ticks seguidos -
    // o rolar do pe no apoio troca uma vez, devagar.)
    std::array<int, 2> targetFlipBefore {}, bodyFlipBefore {};
    float airborneSeconds = 0.0f;
    AthleteState3D athlete;
    LegacyAthleteState3D& legacy = athlete.legacy;
    // O modo pela telemetria neutra (a reescrita, R0): a mesma bateria vale
    // para o caminho legado e para as pecas novas.
    const auto mode = [&]() { return athleteTelemetry3D(athlete).mode; };
    const auto isActive = [&]() {
        const BodyModeKind3D now = mode();
        return now == BodyModeKind3D::Active || now == BodyModeKind3D::Bridge;
    };
    // Como "Restaurar pose" do laboratorio: ja no modo padrao. (--fisico:
    // como ao assumir o boneco, no fisico ate ele assentar.)
    athleteStart3D(*scene, handle, profile, bodyState, athlete,
        startPhysical ? AthleteStartMode3D::Physical : AthleteStartMode3D::Default);
    InflatedBallHandle3D ballHandle;
    InflatedBallState3D ballState;
    BallOwnership3D ownership;
    if (dump != nullptr) std::fprintf(dump, "S %s\n", scenario.name.c_str());
    // (O corpo so fora do de hoje: o dump dele fica o de sempre.)
    if (dump != nullptr && !isBaseBody3D(context.body))
        std::fprintf(dump, "corpo %.4f %.2f\n", context.body.heightMeters, context.body.massKg);
    // XCUSTO=1: o custo do runtime por tick (o passo do atleta; us, p50/p95/p99).
    std::vector<double> runtimeCost;
    struct {
        long ticks = 0;
        double slider = 0.0, pelvis = 0.0, feet = 0.0, getUp = 0.0, outside = 0.0, vertical = 0.0, planar = 0.0,
            couple = 0.0;
        float sliderMax = 0.0f, torqueMax = 0.0f, correctionMax = 0.0f, handoverTorqueMax = 0.0f;
        long handover = 0;
    } auxStats;
    // XVIRADA=1: a manobra da virada brusca (DynamicTurnMove3D).
    struct {
        long ticks = 0;
        float firstAt = -1.0f, lastAt = -1.0f, minScale = 1.0f, maxLean = 0.0f, degrees = 0.0f, entry = 0.0f;
        float minSpeed = 1.0e9f, endSpeed = 0.0f;
    } turnStats;

    AthleteRuntime3D runtime;
    // XREDE=<.mrlpolicy>: a rede da locomocao pelo runtime do jogo (como a
    // chave "Rede de locomocao" do laboratorio).
    AthleteRuntimeSettings3D policySettings = settings;
    if (const char* rede = std::getenv("XREDE"); rede != nullptr && *rede != '\0') {
        runtime.setLocomotionPolicy(std::make_shared<LocomotionPolicy3D>(*context.profile, rede));
        policySettings.locomotionPolicy = true;
    }
    Vec3 followVelocity {};
    bool controlledAnimated = !startPhysical;
    float capsuleSpeedScale = 1.0f;
    Vec3 capsuleMoved {};
    float capsuleRose = 0.0f;
    float wallImpact = 0.0f;
    Metrics metrics;
    // XAJUDA=1: os canais da ajuda no fisico (medias por tick; fracoes do peso).
    struct AssistTally {
        int ticks = 0;
        double rootUp = 0.0, rootPlanar = 0.0, rootTorquePerKg = 0.0, feetUp = 0.0, feetPlanar = 0.0;
        double linkTorquePerKg = 0.0, feetDelivered = 0.0;
    } assistTally;
    // (O modo Ativo por dentro da ponte fisico -> padrao: "ponte".)
    const auto modeOf = [&]() {
        return bodyModeName3D(mode());
    };
    metrics.trail = modeOf();
    BodyModeKind3D previousMode = mode();
    float transitionStart = -1.0f, transitionGuided = -1.0f;
    std::vector<float> transitionChest;
    std::vector<PhysicsBodyState3D> previousLinks = bodyState.links;
    std::vector<Vec3> previousVelocities(bodyState.links.size());
    bool previousJump = false;
    float jumpHeld = 0.0f;
    float autoCooldown = 0.0f;
    // Conduzindo: o rumo pedido do tick anterior e o tempo que ainda vale a
    // virada (o toque da virada).
    Vec3 autoLastDirection {};
    float autoTurnLeft = 0.0f;
    // O rumo pedido virado depois do pedido automatico (o corte).
    float headingTurn = 0.0f;
    const auto turned = [](Vec3 v, float angle) {
        const float c = std::cos(angle), s = std::sin(angle);
        return Vec3 { c * v.x - s * v.y, s * v.x + c * v.y, v.z };
    };
    const int ticks = static_cast<int>(scenario.seconds / Dt);
    // Aquecimento (0,1 s antes do roteiro, sem teclas e sem pegadas): o corpo
    // nasce na pose de ligacao e assenta na do parado - com as pegadas, os
    // pes travados viriam dela. (Na volta do fisico, nao.)
    const int warmTicks = startPhysical ? 0 : 12;
    AthleteRuntimeSettings3D warmSettings = settings;
    warmSettings.footstepLocomotion = false;
    // XEFEITO=1: o efeito de um gancho do RL (com XCORRECAO), depois de 1 s:
    // a velocidade da pelve no plano (media, e a de lado no rumo dela), os
    // pousos por segundo, o pico do pe no ar sobre o chao, a altura media da
    // pelve e os passos do planejador.
    struct EffectTally {
        double speed = 0.0, lateral = 0.0, pelvis = 0.0;
        int samples = 0;
        int touchdowns = 0;
        float footPeak = 0.0f;
        std::array<bool, 2> planted { true, true };
    } effect;
    const bool effectWanted = std::getenv("XEFEITO") != nullptr;
    float lastCameraYaw = 0.0f;
    std::array<bool, 2> gestureWas {};
    // A recepcao e a bola alta (RECEPCAO): o toque, a bola antes dele, o pe
    // mais alto no gesto e se ja imprimiu.
    const bool receptionReport = scenario.name.rfind("recepcao", 0) == 0 || scenario.name.rfind("gesto_alta", 0) == 0
        || scenario.name.rfind("gesto_levantar", 0) == 0
        || scenario.name.rfind("peito", 0) == 0 || scenario.name.rfind("cabeceio", 0) == 0;
    float receptionAt = -1.0f, receptionFootPeak = 0.0f;
    Vec3 receptionBallBefore {}, receptionBodyVelocity {};
    bool receptionAfterShown = false, receptionHalfShown = false;
    for (int step = -warmTicks; step < ticks; ++step) {
        const float t = static_cast<float>(step) * Dt;
        Keys keys = step < 0 ? Keys {} : scenario.keys(t);
        if (suppressRequests) keys.ball = BallActionKind3D::None;
        // (Manejo v2: a bola posta em relacao a pelve, andando com o corpo.)
        const auto placeBall = [&](Vec3 offset) {
            const Vec3 pelvis = bodyState.links.front().position;
            const Vec3 forward { std::cos(keys.cameraYaw), std::sin(keys.cameraYaw), 0.0f };
            const Vec3 left { -forward.y, forward.x, 0.0f };
            const Vec3 position = Vec3 { pelvis.x, pelvis.y, offset.z } + forward * offset.x + left * offset.y;
            const Vec3 incoming = forward * scenario.placeVelocity.x + left * scenario.placeVelocity.y
                + Vec3 { 0.0f, 0.0f, scenario.placeVelocity.z };
            const Vec3 velocity = scenario.placeVelocity.lengthSquared() > 1e-6f ? incoming
                : Vec3 { scene->characterState().velocity.x, scene->characterState().velocity.y, 0.0f };
            const float radius = scene->inflatedBallProfile(ballHandle).radiusMeters;
            scene->setInflatedBallMotion(ballHandle, position, velocity, cross(Vec3 { 0.0f, 0.0f, 1.0f }, velocity) * (1.0f / radius));
            ballState = scene->inflatedBallState(ballHandle);
        };
        if (step >= 0 && scenario.ball && scenario.placeAt >= 0.0f && t >= scenario.placeAt && t < scenario.placeAt + Dt
            && !bodyState.links.empty())
            placeBall(scenario.placeOffset);
        if (step >= 0 && scenario.ball && !bodyState.links.empty())
            for (const auto& [at, offset] : scenario.placements)
                if (t >= at && t < at + Dt) placeBall(offset);
        if (step == 0 && scenario.ball) {
            InflatedBallDefinition3D definition;
            definition.position = scenario.ballStart;
            definition.linearVelocity = scenario.ballVelocity;
            definition.entityId = 2;
            ballHandle = scene->createInflatedBall(definition);
            ballState = scene->inflatedBallState(ballHandle);
        }
        // A capsula como no laboratorio (o personagem controlado no chao).
        const PhysicsCharacterState3D before = scene->characterState();
        if (std::getenv("XGRAVIDADE") && step >= 0) {
            static float takeoff = -1.0f, apex = 0.0f, baseZ = 0.0f, ballFirst = -1.0f, ballApex = 0.0f, ballLaunch = 0.0f;
            static bool wasGrounded = true, ballDown = false, ballBounced = false;
            if (step == 0) {
                takeoff = -1.0f; apex = 0.0f; baseZ = before.position.z; ballFirst = -1.0f; ballApex = 0.0f;
                wasGrounded = true; ballDown = false; ballBounced = false;
                ballLaunch = scenario.ball ? ballState.position.x : 0.0f;
            }
            static PhysicsBodyHandle3D box;
            static bool boxDown = false;
            if (scenario.name == "gravidade_caixa") {
                if (step == 0) {
                    PhysicsShape3D shape;
                    shape.type = PhysicsShapeType3D::Box;
                    shape.halfExtents = { 0.2f, 0.2f, 0.2f };
                    PhysicsBodyDefinition3D definition;
                    definition.position = { 3.0f, 0.0f, 2.0f + 0.2f };
                    definition.massKg = 10.0f;
                    box = scene->createBody(definition, std::span<const PhysicsShape3D>(&shape, 1));
                    boxDown = false;
                }
                const PhysicsBodyState3D state = scene->bodyState(box);
                if (!boxDown && state.position.z - 0.2f <= 0.005f) {
                    boxDown = true;
                    std::printf("  GRAVIDADE gravidade_caixa: caixa solta de 2 m toca o chao em %.3f s (queda livre 0,639 s), vel %.2f m/s (livre 6,26)\n",
                        t, -state.linearVelocity.z);
                }
            }
            // A queda do corpo derrubado (a caixa pesada): a maior aceleracao da
            // pelve para baixo (livre: 9,81) e o tempo ate ela chegar a 0,35 m.
            if (scenario.name == "queda_levanta" && !bodyState.links.empty()) {
                static float lastVz = 0.0f, worstDown = 0.0f, hitTime = -1.0f;
                static bool reported = false;
                const PhysicsBodyState3D& pelvis = bodyState.links.front();
                if (step == 0) { worstDown = 0.0f; hitTime = -1.0f; reported = false; lastVz = pelvis.linearVelocity.z; }
                const float az = (pelvis.linearVelocity.z - lastVz) / Dt;
                lastVz = pelvis.linearVelocity.z;
                if (t > 1.0f && !reported) {
                    if (hitTime < 0.0f) hitTime = 1.0f;
                    worstDown = std::min(worstDown, az);
                    if (pelvis.position.z < 0.35f * bodyScale3D(context.body)) {
                        std::printf("  GRAVIDADE queda: a pelve chega a 0,35 m em %.3f s depois da pancada; maior aceleracao para baixo %.2f m/s2 (livre 9,81)\n",
                            t - hitTime, -worstDown);
                        reported = true;
                    }
                }
            }
            if (scenario.name.rfind("gravidade_pulo", 0) == 0) {
                if (wasGrounded && !before.grounded && takeoff < 0.0f) takeoff = t;
                if (takeoff >= 0.0f) apex = std::max(apex, before.position.z - baseZ);
                if (!wasGrounded && before.grounded && takeoff >= 0.0f) {
                    std::printf("  GRAVIDADE %s: subida da capsula %.3f m, no ar %.3f s (velocidade de saida pela subida %.2f m/s)\n",
                        scenario.name.c_str(), apex, t - takeoff, std::sqrt(2.0f * 9.81f * apex));
                    takeoff = 1e9f;
                }
                wasGrounded = before.grounded;
            }
            if (scenario.ball && scenario.name.rfind("gravidade_", 0) == 0 && scenario.name.rfind("gravidade_pulo", 0) != 0) {
                const float radius = scene->inflatedBallProfile(ballHandle).radiusMeters;
                const float bottom = ballState.position.z - radius;
                if (!ballDown && bottom <= 0.005f && step > 0) {
                    ballDown = true;
                    ballFirst = t;
                    std::printf("  GRAVIDADE %s: primeiro toque no chao em %.3f s, a %.2f m (vel %.2f %.2f m/s)\n",
                        scenario.name.c_str(), t, ballState.position.x - ballLaunch, ballState.linearVelocity.x,
                        ballState.linearVelocity.z);
                }
                if (ballDown && !ballBounced) {
                    ballApex = std::max(ballApex, bottom);
                    if (ballState.linearVelocity.z < -0.05f && ballApex > 0.02f) {
                        ballBounced = true;
                        std::printf("  GRAVIDADE %s: quique ate %.3f m (embaixo da bola)\n", scenario.name.c_str(), ballApex);
                    }
                }
            }
        }
        const Vec3 forward { std::cos(keys.cameraYaw), std::sin(keys.cameraYaw), 0.0f };
        const Vec3 right { forward.y, -forward.x, 0.0f };
        // Caido, levantando ou no carrinho, a capsula fica sem teclas (como no
        // laboratorio: o corpo e quem a leva).
        const CharacterLocomotionState3D lastState = legacy.locomotion.telemetry().state;
        const bool down = controlledAnimated ? legacy.defaultLocomotion.output().tackling
            : lastState == CharacterLocomotionState3D::Fallen || lastState == CharacterLocomotionState3D::GettingUp
                || lastState == CharacterLocomotionState3D::Tackling;
        CharacterMotorCommand3D command;
        command.ignoreRagdolls = true;
        // A intencao, como no laboratorio, existe tambem caido e levantando
        // (o levantar e a volta a usam); so a capsula fica sem teclas.
        const Vec3 wantedDirection = turned(forward * keys.forward - right * keys.left, headingTurn);
        if (!down) command.moveDirection = wantedDirection;
        command.sprint = keys.sprint;
        CharacterIntent3D intent;
        intent.requestedFacingYawRadians = keys.cameraYaw;
        intent.effort = keys.sprint ? 1.0f : 0.0f;
        intent.crouch = keys.crouch;
        if (wantedDirection.lengthSquared() > 1e-4f) {
            const Vec3 direction = wantedDirection.normalized();
            const float travelRelative = std::atan2(direction.y, direction.x) - keys.cameraYaw;
            const float wanted = controlledAnimated && keys.sprint && keys.maxSprint ? settings.defaultMaxSprintSpeed
                : controlledAnimated || settings.physicalUsesDefaultSpeeds
                ? athleteDefaultTravelSpeed3D(settings, travelRelative, keys.sprint)
                : characterGaitSpeed3D(gaitSpeeds, travelRelative, keys.sprint);
            const float nominal = keys.sprint ? avatar.sprintSpeed : avatar.walkSpeed;
            if (wanted > 0.0f && nominal > 0.0f) command.speedScale = wanted / nominal;
            if (controlledAnimated) command.speedScale *= capsuleSpeedScale;
            if (!controlledAnimated && wanted > 0.0f)
                command.speedScale *= std::clamp(legacy.locomotion.telemetry().terrainSpeedLimit / wanted, 0.0f, 1.0f);
            intent.requestedDirectionWorld = direction;
            // (O analogico pela metade pede metade da velocidade, como a
            // capsula anda.)
            float speed = wanted * std::min(1.0f, wantedDirection.length());
            if (!controlledAnimated) speed = std::min(speed, legacy.locomotion.telemetry().terrainSpeedLimit);
            intent.requestedVelocityWorld = direction * speed;
        }
        // Pulo ao soltar: segurar o espaco carrega.
        if (keys.jump) jumpHeld += Dt;
        command.jumpPressed = previousJump && !keys.jump;
        command.jumpChargeSeconds = jumpHeld;
        if (!keys.jump && previousJump) jumpHeld = 0.0f;
        previousJump = keys.jump;
        CharacterMotorSettings3D active = avatar;
        if (controlledAnimated) {
            const AthleteCapsuleResponse3D response = athleteCapsuleResponse3D(settings.capsuleResponsePercent,
                settings.capsuleReversalPercent);
            active.groundAcceleration = response.acceleration;
            active.groundDeceleration = response.deceleration;
            active.groundTurnRate = response.turnRate;
            active.groundBrakeDeceleration = response.brake;
            active.groundReverseBrakeScale = response.reverseBrakeScale;
            active.groundReverseAccelerationScale = response.reverseAccelerationScale;
        } else {
            active = adaptiveCharacterMotorSettings3D(active, before, command,
                physicalNavigationAcceleration3D(settings.e5WalkingAssistPercent * 0.01f));
        }
        command.followVelocity = followVelocity;
        command.navigationProxy = true;
        scene->moveCharacter(command, active, Dt);
        const PhysicsCharacterState3D& capsule = scene->characterState();
        {
            const Vec3 moved { (capsule.position.x - before.position.x) / Dt,
                (capsule.position.y - before.position.y) / Dt, 0.0f };
            const float braking = std::max({ active.groundAcceleration, active.groundDeceleration,
                active.groundBrakeDeceleration }) * Dt;
            const float rise = capsule.position.z - before.position.z;
            wallImpact = athleteCapsuleWallImpact3D(capsuleMoved, moved, braking, capsule.grounded,
                std::max(rise, capsuleRose));
            capsuleRose = rise;
            capsuleMoved = moved;
        }
        AthleteControlInput3D control;
        control.controlledEntityId = 1;
        control.intent = intent;
        control.facingYawRadians = keys.cameraYaw;
        control.lookPitchRadians = -0.2f;
        control.sprinting = keys.sprint && Vec3 { capsule.velocity.x, capsule.velocity.y, 0.0f }.lengthSquared() > 0.01f;
        control.proxyVelocityWorld = { capsule.velocity.x, capsule.velocity.y, 0.0f };
        control.capsuleWallImpact = wallImpact;
        control.capsuleFollowVelocityWorld = followVelocity;
        control.jumpCharge = characterJumpCharge3D(active, command.jumpChargeSeconds);
        control.ballRequest = keys.ball;
        control.ballRequestPower = keys.power;
        control.ballRequestAimYaw = keys.cameraYaw;
        autoCooldown -= Dt;
        // (Conduzindo, o jogador toca tambem na virada e na parada: o rumo
        // pedido mudando mais de 30 graus, ou as teclas soltas, com a bola ate
        // 1,6 m.)
        if (scenario.autoCooldown < 5.0f) {
            const Vec3 now = wantedDirection;
            const bool moving = now.lengthSquared() > 0.04f, wasMoving = autoLastDirection.lengthSquared() > 0.04f;
            if ((moving && wasMoving && dot(now.normalized(), autoLastDirection.normalized()) < 0.87f) || (wasMoving && !moving))
                autoTurnLeft = 0.3f;
            autoLastDirection = now;
            autoTurnLeft -= Dt;
        }
        // (XAUTODIST: o pedido automatico a outra distancia - a recepcao apertada cedo.)
        const float autoDistance = std::getenv("XAUTODIST") && scenario.autoDistance > 0.0f
            ? static_cast<float>(std::atof(std::getenv("XAUTODIST"))) : scenario.autoDistance;
        if (step >= 0 && autoDistance > 0.0f && scenario.ball && autoCooldown <= 0.0f && !bodyState.links.empty()) {
            const Vec3 pelvis = bodyState.links.front().position;
            const Vec3 offset = ballState.position - pelvis;
            const float reach = autoTurnLeft > 0.0f ? std::max(autoDistance, 1.6f) : autoDistance;
            if (std::hypot(offset.x, offset.y) < reach) {
                autoTurnLeft = 0.0f;
                // (Sem pedido: so a virada do rumo, para comparar.)
                if (!suppressRequests) control.ballRequest = scenario.autoKind;
                control.ballRequestPower = scenario.autoPower >= 0.0f ? scenario.autoPower
                    : scenario.autoKind == BallActionKind3D::Shot ? 0.8f : 0.45f;
                autoCooldown = scenario.autoCooldown;
                if (scenario.autoTurn != 0.0f) {
                    headingTurn = scenario.autoTurn;
                    control.intent.requestedDirectionWorld = turned(control.intent.requestedDirectionWorld, headingTurn);
                    control.intent.requestedVelocityWorld = turned(control.intent.requestedVelocityWorld, headingTurn);
                }
            }
        }
        if (control.ballRequest != BallActionKind3D::None) {
            {
                char item[32];
                std::snprintf(item, sizeof(item), "%sp@%.2f", metrics.gestureSequence.empty() ? "" : " ", t);
                metrics.gestureSequence += item;
            }
            ++metrics.requests;
            if (metrics.requestTime < 0.0f) metrics.requestTime = t;
            metrics.pendingRequests.push_back(t);
        }
        if (std::getenv("XPAREDE")) {
            float wallImpulse = 0.0f;
            // (XPAREDE=2: tambem o que se move - a caixa, outro boneco.)
            const bool dynamic = std::getenv("XPAREDE")[0] == '2';
            for (const RagdollInteraction3D& contact : bodyState.interactions)
                if ((dynamic || contact.otherMotion != RagdollContactMotion3D::Dynamic) && contact.normalWorld.z < 0.6f) {
                    wallImpulse += contact.normalImpulseNewtonSeconds;
                    if (contact.normalImpulseNewtonSeconds > 0.2f)
                        std::printf("  t %.3f link %s motion %d normal %.2f %.2f %.2f impulso %.2f\n", t,
                            profile.links[contact.linkIndex].id.c_str(), static_cast<int>(contact.otherMotion),
                            contact.normalWorld.x, contact.normalWorld.y, contact.normalWorld.z,
                            contact.normalImpulseNewtonSeconds);
                }
            if (wallImpact > 1.0f) std::printf("  t %.3f capsula bateu %.2f\n", t, wallImpact);
        }
        // XCORRECAO=<gancho> (XVALOR=<valor fisico>): um teste dos ganchos do
        // RL, a correcao fixa numa cabeca (sem teto nem filtro): inclinacao
        // (rad), passo (m, os pes abrindo), joelho (rad), cm_x/cm_y (m),
        // altura (m), contato_x/contato_y (m/s2), cadencia/passada (oitavas),
        // ritmo_esq (ciclo), balanco (m), recupera_esq (0..1).
        if (const char* hook = std::getenv("XCORRECAO"); hook != nullptr && step == 0) {
            const std::string which(hook);
            const char* given = std::getenv("XVALOR");
            const auto value = [&](float fallback) { return given != nullptr ? static_cast<float>(std::atof(given)) : fallback; };
            PhysicalCorrections3D& c = athlete.corrections;
            if (which == "inclinacao") {
                c.balance = true;
                c.forwardLean = value(0.15f);
            } else if (which == "passo") {
                c.footwork = true;
                c.feet[0].touchdownOffset = { 0.0f, value(0.12f), 0.0f };
                c.feet[1].touchdownOffset = { 0.0f, -value(0.12f), 0.0f };
            } else if (which == "joelho") {
                c.pose = true;
                c.jointResiduals.assign(profile.links.size() * 3, 0.0f);
                for (std::size_t i = 0; i < profile.links.size(); ++i)
                    if (profile.links[i].id.find("Shin") != std::string::npos)
                        for (std::size_t axis = 0; axis < 3; ++axis) c.jointResiduals[i * 3 + axis] = value(0.25f);
            } else if (which == "cm_x" || which == "cm_y") {
                c.balance = true;
                (which == "cm_x" ? c.comOffset.x : c.comOffset.y) = value(0.08f);
            } else if (which == "altura") {
                c.balance = true;
                c.pelvisHeight = value(-0.04f);
            } else if (which == "contato_x" || which == "contato_y") {
                c.balance = true;
                (which == "contato_x" ? c.contactAcceleration.x : c.contactAcceleration.y) = value(1.0f);
            } else if (which == "cadencia" || which == "passada") {
                c.footwork = true;
                (which == "cadencia" ? c.cadenceOctaves : c.strideOctaves) = value(0.2f);
            } else if (which == "ritmo_esq") {
                c.footwork = true;
                c.feet[0].phaseLead = value(0.05f);
            } else if (which == "balanco") {
                c.footwork = true;
                c.feet[0].swingHeight = c.feet[1].swingHeight = value(0.05f);
            } else if (which == "recupera_esq") {
                c.footwork = true;
                c.feet[0].recoveryUrgency = value(0.5f);
            }
        }
        AthleteSlot3D slot { 1, handle, &bodyState, true, false, &athlete };
        // --sem-capsula: ninguem e o controlado; o atleta recebe o comando
        // pelo slot (a capsula virtual dele anda no pedido).
        AthleteCommand3D slotCommand;
        if (context.noCapsule) {
            control.controlledEntityId = 0;
            AthleteStickInput3D stick;
            stick.moveWorld = wantedDirection;
            stick.cameraYawRadians = keys.cameraYaw;
            stick.lookPitchRadians = -0.2f;
            stick.sprint = keys.sprint;
            stick.crouch = keys.crouch;
            slotCommand = athleteStickCommand3D(stick, settings, gaitSpeeds,
                athleteTelemetry3D(athlete).terrainSpeedLimit);
            slot.command = &slotCommand;
        }
        std::vector<AthleteBall3D> balls;
        if (scenario.ball && step >= 0) balls.push_back({ ballHandle, &ballState, &ownership });
        AthleteRuntimeOutput3D output;
        if (step >= 0) contractBefore(hooks, athlete, step);
        const auto runtimeBegin = std::chrono::steady_clock::now();
        runtime.update(*scene, runtimeAssets, std::span<AthleteSlot3D>(&slot, 1), balls, step < 0 ? warmSettings : policySettings,
            control, Dt, output);
        // (XREACH: a base das areas de alcance dos pes e a altura da pelve sobre ela,
        // a cada 0,05 s - no pulo, a base sobe com ele.)
        if (output.reachValid && step >= 0) metrics.reachBaseMax = std::max(metrics.reachBaseMax, output.reachOrigin.z);
        if (std::getenv("XREACH") && output.reachValid && step >= 0 && step % 6 == 0)
            std::printf("  REACH t %.2f base %.3f pelve sobre a base %.3f\n", t, output.reachOrigin.z, output.reachPelvisHeight);
        if (step >= 0)
            runtimeCost.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - runtimeBegin).count());
        if (step >= 0) {
            // XAUX=1: a ajuda auxiliar por canal (a medida do atleta).
            const AthleteExternalAssist3D& aux = athlete.externalAssist;
            auxStats.ticks += 1;
            if (std::getenv("XTROCA") && step % 6 == 0 && t > 1.4f && t < 3.4f) {
                const Vec3 v = scene->characterState().velocity;
                std::printf("    t %.2f capsula %.2f %.2f (%.2f m/s)\n", t, v.x, v.y, std::hypot(v.x, v.y));
            }
            {
                const TurnMoveOutput3D& move = legacy.turnMove.output();
                const float planarSpeed = std::hypot(bodyState.links.front().linearVelocity.x,
                    bodyState.links.front().linearVelocity.y);
                if (std::getenv("XVIRADA") && std::string(std::getenv("XVIRADA")) == "2" && step % 6 == 0 && t > 2.0f && t < 4.0f) {
                    const Vec3 bodyV = bodyState.links.front().linearVelocity;
                    const Vec3 req = control.intent.requestedVelocityWorld;
                    const float ang = (std::hypot(bodyV.x, bodyV.y) > 0.05f && std::hypot(req.x, req.y) > 0.05f)
                        ? std::acos(std::clamp((bodyV.x * req.x + bodyV.y * req.y) / (std::hypot(bodyV.x, bodyV.y) * std::hypot(req.x, req.y)), -1.0f, 1.0f)) * 57.2958f
                        : 0.0f;
                    std::printf("    t %.2f estado %d peso %.2f | pedido %.2f corpo %.2f m/s angulo %.0f | escala %.2f lean %.3f\n", t,
                        static_cast<int>(move.state), move.weight, std::hypot(req.x, req.y), std::hypot(bodyV.x, bodyV.y), ang,
                        move.speedScale, legacy.turnCorrections.lateralLean);
                }
                if (move.weight > 0.0f) {
                    ++turnStats.ticks;
                    if (turnStats.firstAt < 0.0f) turnStats.firstAt = t;
                    turnStats.lastAt = t;
                    turnStats.minScale = std::min(turnStats.minScale, move.speedScale);
                    turnStats.degrees = move.turnDegrees;
                    turnStats.entry = move.entrySpeed;
                    turnStats.minSpeed = std::min(turnStats.minSpeed, planarSpeed);
                }
                turnStats.maxLean = std::max(turnStats.maxLean, std::abs(legacy.turnCorrections.lateralLean));
                turnStats.endSpeed = planarSpeed;
            }
            if (std::getenv("XGRAVIDADE") && scenario.name == "queda_levanta" && t >= 0.95f && t <= 1.85f && step % 6 == 0
                && !bodyState.links.empty())
                std::printf("  QUEDA t %.2f pelve z %.2f vz %+.2f | ajuda: regua %.0f pelve %.0f pes %.0f levantar %.0f fora %.0f N | modo %s\n", t,
                    bodyState.links.front().position.z, bodyState.links.front().linearVelocity.z, aux.sliderNewtons(),
                    aux.pelvisNewtons, aux.feetNewtons, aux.getUpNewtons, aux.tackleNewtons + aux.activeNewtons,
                    bodyModeName3D(mode()));
            auxStats.handover += aux.handover ? 1 : 0;
            // (A volta ao padrao sobe a ajuda ate a cheia: fora da regua.)
            if (aux.handover) {
                auxStats.outside += aux.sliderNewtons() + aux.tackleNewtons + aux.activeNewtons;
                auxStats.handoverTorqueMax = std::max(auxStats.handoverTorqueMax, aux.netTorque);
            } else {
            auxStats.slider += aux.sliderNewtons();
            auxStats.sliderMax = std::max(auxStats.sliderMax, aux.sliderNewtons());
            auxStats.pelvis += aux.pelvisNewtons;
            auxStats.feet += aux.feetNewtons;
            auxStats.getUp += aux.getUpNewtons;
            auxStats.torqueMax = std::max(auxStats.torqueMax, aux.netTorque);
            auxStats.outside += aux.tackleNewtons + aux.activeNewtons;
            auxStats.vertical += aux.pelvisVerticalNewtons;
            auxStats.planar += aux.pelvisPlanarNewtons;
            auxStats.correctionMax = std::max(auxStats.correctionMax, aux.correctionTorque);
            auxStats.couple += aux.postureCoupleTorque;
            }
        }
        if (output.followVelocitySet) followVelocity = output.followVelocityWorld;
        if (output.controlledAnimatedSet) controlledAnimated = output.controlledAnimated;
        capsuleSpeedScale = output.controlledSpeedScale;
        for (std::size_t side = 0; side < 2; ++side)
            if (legacy.defaultLocomotion.output().gestureLegs[side]) ++metrics.gestureTicks[side];
        if (legacy.defaultLocomotion.output().gestureLegs[0] && legacy.defaultLocomotion.output().gestureLegs[1])
            ++metrics.bothGestureTicks;
        for (std::size_t side = 0; side < 2; ++side)
            if (legacy.defaultLocomotion.output().gestureLegs[side] && !gestureWas[side]) {
                char item[32];
                std::snprintf(item, sizeof(item), "%s%c@%.2f", metrics.gestureSequence.empty() ? "" : " ", side == 0 ? 'E' : 'D', t);
                metrics.gestureSequence += item;
            }
        for (std::size_t side = 0; side < 2; ++side) gestureWas[side] = legacy.defaultLocomotion.output().gestureLegs[side];
        if (output.ballTouched && receptionReport && receptionAt < 0.0f && !bodyState.links.empty()) {
            receptionAt = t;
            receptionBallBefore = ballState.linearVelocity;
            const Vec3 capsule = scene->characterState().velocity;
            receptionBodyVelocity = { capsule.x, capsule.y, 0.0f };
            const Vec3 offset = ballState.position - bodyState.links.front().position;
            const Vec3 toward = Vec3 { -offset.x, -offset.y, 0.0f } * (1.0f / std::max(1e-4f, std::hypot(offset.x, offset.y)));
            std::printf("  RECEPCAO %s: pedido %.3f s, pelve %.3f, bola a %.2f m (altura %.2f), vindo %.2f m/s, relativa %.2f m/s\n",
                scenario.name.c_str(), t,
                legacy.defaultLocomotion.output().pose.linkPositions.empty() ? 0.0f
                    : legacy.defaultLocomotion.output().pose.linkPositions[0].z,
                std::hypot(offset.x, offset.y), ballState.position.z,
                dot(Vec3 { ballState.linearVelocity.x, ballState.linearVelocity.y, 0.0f }, toward),
                (Vec3 { ballState.linearVelocity.x, ballState.linearVelocity.y, 0.0f } - receptionBodyVelocity).length());
        }
        if (output.ballTouched && std::getenv("XQUALRECEP") && !bodyState.links.empty()) {
            const Vec3 offset = ballState.position - bodyState.links.front().position;
            const Vec3 toward = Vec3 { -offset.x, -offset.y, 0.0f } * (1.0f / std::max(1e-4f, std::hypot(offset.x, offset.y)));
            const Vec3 capsule = scene->characterState().velocity;
            const Vec3 flat { ballState.linearVelocity.x, ballState.linearVelocity.y, 0.0f };
            const float world = dot(flat, toward), relative = dot(flat - Vec3 { capsule.x, capsule.y, 0.0f }, toward);
            std::printf("  QUAL t %.3f mundo %.2f relativa %.2f | bola %+.2f %+.2f %.2f v %+.2f %+.2f %+.2f | capsula %+.2f %+.2f\n", t,
                world, relative, offset.x, offset.y, ballState.position.z, ballState.linearVelocity.x,
                ballState.linearVelocity.y, ballState.linearVelocity.z, capsule.x, capsule.y);
        }
        if (output.ballTouched) {
            ++metrics.touches;
            if (dump != nullptr) std::fprintf(dump, "T\n");
            // (O atraso do pedido mais antigo ate a forca: zero, a forca sai no
            // pedido.)
            while (!metrics.pendingRequests.empty() && t - metrics.pendingRequests.front() > 1.2f) {
                metrics.pendingRequests.erase(metrics.pendingRequests.begin());
                ++metrics.lostRequests;
            }
            if (!metrics.pendingRequests.empty()) {
                metrics.latencies.push_back(t - metrics.pendingRequests.front());
                metrics.pendingRequests.erase(metrics.pendingRequests.begin());
            }
            if (metrics.forceTime < 0.0f) metrics.forceTime = t;
        }
        if (step >= 0) metrics.capsulePath.push_back(scene->characterState().position);
        if (step >= 0 && scenario.ball && scenario.autoCooldown < 5.0f && t > 0.3f && !bodyState.links.empty()) {
            const Vec3 offset = ballState.position - bodyState.links.front().position;
            const float distance = std::hypot(offset.x, offset.y);
            if (distance > 2.5f) metrics.ballFarSeconds += Dt;
            metrics.ballDistanceSum += distance;
            ++metrics.ballDistanceCount;
            if (std::getenv("XCONDUZ") && step % 12 == 0)
                std::printf("  t %.2f bola a %.2f m (x %+.2f y %+.2f) v bola %.2f %.2f corpo %.2f %.2f\n", t, distance,
                    offset.x, offset.y, ballState.linearVelocity.x, ballState.linearVelocity.y,
                    scene->characterState().velocity.x, scene->characterState().velocity.y);
        }
        if (step >= 0 && t > 0.35f && !bodyState.links.empty()) {
            metrics.capsuleUp = std::max(metrics.capsuleUp, scene->characterState().velocity.z);
            metrics.pelvisUp = std::max(metrics.pelvisUp, bodyState.links.front().linearVelocity.z);
            metrics.pelvisOverCapsule = std::max(metrics.pelvisOverCapsule,
                bodyState.links.front().position.z - scene->characterState().position.z);
            if (std::getenv("XTERRENO") && step % 6 == 0)
                std::printf("  t %.3f capsula %.2f %.3f vz %.2f | pelve %.3f vz %.2f | modo %s\n", t,
                    scene->characterState().position.x, scene->characterState().position.z,
                    scene->characterState().velocity.z, bodyState.links.front().position.z,
                    bodyState.links.front().linearVelocity.z, modeOf());
        }
        if (mode() == BodyModeKind3D::Physical && !bodyState.links.empty()) {
            const auto& telemetry = legacy.locomotion.telemetry();
            if (metrics.riseAt < 0.0f && telemetry.getUpPhase == CharacterGetUpPhase3D::Rising) {
                std::size_t chest = 0;
                for (std::size_t i = 0; i < profile.links.size(); ++i)
                    if (profile.links[i].id == "Chest") chest = i;
                metrics.riseAt = t;
                metrics.riseChest = bodyState.links[chest].orientation.rotate({ 1.0f, 0.0f, 0.0f }).z;
                metrics.risePelvis = (bodyState.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                                         .rotate({ 1.0f, 0.0f, 0.0f }).z;
                metrics.riseSide = static_cast<int>(telemetry.fallOrientation);
            }
        }
        if (metrics.riseAt >= 0.0f && metrics.risenAt < 0.0f
            && (mode() != BodyModeKind3D::Physical
                || legacy.locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::None))
            metrics.risenAt = t;
        if (std::getenv("XQUEDA") && step % 15 == 0 && mode() == BodyModeKind3D::Physical && !bodyState.links.empty()) {
            std::size_t chest = 0;
            for (std::size_t i = 0; i < profile.links.size(); ++i)
                if (profile.links[i].id == "Chest") chest = i;
            const Vec3 chestForward = bodyState.links[chest].orientation.rotate({ 1.0f, 0.0f, 0.0f });
            const Vec3 pelvisUp = (bodyState.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                                      .rotate({ 0.0f, 0.0f, 1.0f });
            const auto& telemetry = legacy.locomotion.telemetry();
            const Vec3 pelvisForward = (bodyState.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                                           .rotate({ 1.0f, 0.0f, 0.0f });
            std::printf("  t %.2f estado %d fase %d lado %d peito frente z %+.2f pelve frente z %+.2f cima z %+.2f altura %.2f\n",
                t, static_cast<int>(telemetry.state), static_cast<int>(telemetry.getUpPhase),
                static_cast<int>(telemetry.fallOrientation), chestForward.z, pelvisForward.z, pelvisUp.z,
                bodyState.links.front().position.z);
        }
        for (const Throw& item : scenario.throws)
            if (t >= item.at && t < item.at + Dt) {
                createLocomotionThrow3D(*scene, { item.start, item.velocity, item.massKg, item.halfExtents });
                if (dump != nullptr) std::fprintf(dump, "E caixa\n");
            }
        for (const Push& push : scenario.pushes)
            if (t >= push.at && t < push.at + push.seconds) {
                scene->applyRagdollLinkForce(handle, 0, push.force * 0.20f, {});
                scene->applyRagdollLinkForce(handle, 2, push.force * 0.32f, {});
                scene->applyRagdollLinkForce(handle, 3, push.force * 0.48f, {});
            }
        scene->simulate(Dt);
        bodyState = scene->ragdollState(handle);
        if (goalScenario && !bodyState.links.empty()) {
            static float deepest = 0.0f, rebound = 0.0f, firstPhysical = -1.0f, firstPush = -1.0f, peakPush = 0.0f, widest = 0.0f, pelvisDeepest = 0.0f;
            static float capsuleDeepest = 0.0f, netMilliseconds = 0.0f, capsuleRise = 0.0f;
            static int entered = 0;
            static int netAsleep = 0, netTouched = 0;
            if (step == 0) { deepest = 0.0f; rebound = 0.0f; firstPhysical = -1.0f; firstPush = -1.0f; peakPush = 0.0f; widest = 0.0f; pelvisDeepest = 0.0f; capsuleDeepest = 0.0f; netMilliseconds = 0.0f; netAsleep = 0; netTouched = 0; capsuleRise = 0.0f; entered = 0; }
            const bool fromOutside = scenario.name.rfind("rede_lado", 0) == 0 || scenario.name == "rede_atras";
            if (fromOutside) {
                bool in = scene->hasCharacter() && insideGoal(scene->characterState().position);
                for (const PhysicsBodyState3D& link : bodyState.links) in = in || insideGoal(link.position);
                entered += in ? 1 : 0;
            }
            // (A metade da capsula de pe mais 5 mm: 0,92 no corpo de hoje.)
            const float capsuleHalf = isBaseBody3D(context.body) ? 0.92f : 0.5f * context.avatar.standingHeight + 0.005f;
            if (scene->hasCharacter()) capsuleRise = std::max(capsuleRise, scene->characterState().position.z - capsuleHalf);
            // (A rede nos ultimos 5 s: dormindo, e o custo medio.)
            if (t >= scenario.seconds - 5.0f) {
                const NetStatus3D net = scene->netStatus(goalNet);
                netAsleep += net.sleeping ? 1 : 0;
                netTouched += net.contacts > 0 ? 1 : 0;
                netMilliseconds += net.stepMilliseconds;
            }
            pelvisDeepest = std::max(pelvisDeepest, bodyState.links.front().position.x);
            if (scene->hasCharacter()) capsuleDeepest = std::max(capsuleDeepest, scene->characterState().position.x);
            for (const PhysicsBodyState3D& link : bodyState.links) {
                deepest = std::max(deepest, link.position.x);
                widest = std::max(widest, std::abs(link.position.y));
            }
            if (bodyState.externalPushNewtonSeconds > 0.5f && firstPush < 0.0f) firstPush = t;
            peakPush = std::max(peakPush, bodyState.externalPushNewtonSeconds);
            if (mode() == BodyModeKind3D::Physical && firstPhysical < 0.0f) firstPhysical = t;
            if (firstPhysical >= 0.0f && t < firstPhysical + 1.0f) rebound = std::max(rebound, -bodyState.links.front().linearVelocity.x);
            if (step + 1 == ticks)
                std::printf("  REDE %s: empurrao da rede aos %.2f s (pico %.1f N s), fisico aos %.2f s, a pelve ate x = %.2f, "
                            "o link mais fundo x = %.2f (atravessar: alem de %.2f)%s, a capsula ate x = %.2f (alem de %.2f)%s, |y| ate %.2f, "
                            "volta para tras ate %.2f m/s, termina %s; a rede nos ultimos 5 s: dormindo %.0f%%, tocada %.0f%%, %.3f ms/tick; de fora, ticks com um link ou a capsula dentro do gol: %d%s; a capsula subiu %.2f m%s\n",
                    scenario.name.c_str(), firstPush, peakPush, firstPhysical, pelvisDeepest, deepest, netBehind,
                    deepest > netBehind ? " ATRAVESSOU" : "", capsuleDeepest, capsuleBehind, capsuleDeepest > capsuleBehind ? " CAPSULA PASSOU" : "",
                    widest, rebound, bodyModeName3D(mode()), 100.0f * static_cast<float>(netAsleep) / (5.0f / Dt),
                    100.0f * static_cast<float>(netTouched) / (5.0f / Dt),
                    netMilliseconds / (5.0f / Dt), entered, entered > 0 ? " ENTROU" : "", capsuleRise, capsuleRise > 0.1f ? " SUBIU" : "");
        }
        if (scenario.name.rfind("gesto_", 0) == 0 && leftFoot < bodyState.links.size() && rightFoot < bodyState.links.size()) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            const float c = std::cos(o.facingYawRadians), s = std::sin(o.facingYawRadians);
            const auto local = [&](Vec3 p, Vec3 pelvis) {
                const Vec3 d = p - pelvis;
                return Vec3 { c * d.x + s * d.y, -s * d.x + c * d.y, 0.0f };
            };
            const std::size_t footLinks[2] = { leftFoot, rightFoot };
            const bool start = t >= 0.15f && t < 0.15f + 0.5f * Dt;
            for (std::size_t side = 0; side < 2; ++side) {
                const Vec3 body = local(bodyState.links[footLinks[side]].position, bodyState.links[0].position);
                const Vec3 animated = o.pose.linkPositions.size() > footLinks[side]
                    ? local(o.pose.linkPositions[footLinks[side]], o.pose.linkPositions[0]) : Vec3 {};
                (start ? metrics.stanceStart : metrics.stanceEnd)[side] = body;
                (start ? metrics.animatedStart : metrics.animatedEnd)[side] = animated;
            }
            metrics.stanceValid = true;
            // Parado de verdade ha mais de 1 s: o pe tem que estar no lugar.
            const Vec3 capsule = scene->characterState().velocity;
            const bool still = std::hypot(capsule.x, capsule.y) < 0.05f && keys.forward == 0.0f && keys.left == 0.0f
                && std::abs(keys.cameraYaw - lastCameraYaw) < 1e-6f && !o.gestureLegs[0] && !o.gestureLegs[1];
            metrics.stillSeconds = still ? metrics.stillSeconds + Dt : 0.0f;
            if (std::getenv("XPESTRACO") && step % 6 == 0 && t >= std::atof(std::getenv("XPESTRACO"))) {
                const Vec3 l = local(o.pose.linkPositions[leftFoot], o.pose.linkPositions[0]);
                const Vec3 r = local(o.pose.linkPositions[rightFoot], o.pose.linkPositions[0]);
                std::printf("  PT %.2f teclas %+.0f %+.0f %d cam %+.2f v %.2f | gesto %d%d | marcha %d mov %.2f | contato %.2f %.2f"
                    " apoio %d%d | E %+.2f %+.2f D %+.2f %+.2f\n", t, keys.forward, keys.left, keys.sprint ? 1 : 0,
                    keys.cameraYaw, std::hypot(capsule.x, capsule.y), o.gestureLegs[0] ? 1 : 0, o.gestureLegs[1] ? 1 : 0,
                    o.cycling ? 1 : 0, o.movingShare, o.footContact[0], o.footContact[1], o.cyclePlanted[0] ? 1 : 0,
                    o.cyclePlanted[1] ? 1 : 0, l.x, l.y, r.x, r.y);
            }
            if (metrics.stillSeconds > 1.0f) {
                const std::array<Vec3, 2> now { local(bodyState.links[leftFoot].position, bodyState.links[0].position),
                    local(bodyState.links[rightFoot].position, bodyState.links[0].position) };
                const float off = std::max((now[0] - metrics.stanceStart[0]).length(), (now[1] - metrics.stanceStart[1]).length());
                if (off > metrics.stillWorst) {
                    metrics.stillWorst = off;
                    metrics.stillWorstTime = t;
                    metrics.stillWorstFeet = now;
                }
            }
        }
        lastCameraYaw = keys.cameraYaw;
        if (trace != nullptr && step >= 0) trace->push_back(athleteTickDigest3D(athlete, bodyState));
        if (step >= 0 && context.noCapsule) contractAfter(hooks, athlete, bodyState, slotCommand);
        if (scenario.ball && step >= 0) ballState = scene->inflatedBallState(ballHandle);
        if (receptionReport && step >= 0 && !bodyState.links.empty()) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            for (std::size_t side = 0; side < 2; ++side) {
                const std::size_t foot = side == 0 ? leftFoot : rightFoot;
                if (o.gestureLegs[side] && foot < bodyState.links.size()) {
                    // (O tornozelo - o ponto que o catalogo mira -, nao o centro do pe.)
                    const RagdollLinkDefinition3D& footLink = profile.links[foot];
                    const Vec3 ankle = bodyState.links[foot].position + bodyState.links[foot].orientation.rotate(
                        footLink.modelOrientation.conjugate().rotate(footLink.inboundJoint.anchorModelPosition - footLink.modelPosition));
                    receptionFootPeak = std::max(receptionFootPeak, ankle.z);
                    if (std::getenv("XPEGESTO"))
                        std::printf("  PG t %.3f %c tornozelo z %.3f x %+.3f v %.2f (vx %+.2f vy %+.2f vz %+.2f) bola %.2f %.2f %.2f\n", t,
                            side == 0 ? 'E' : 'D', ankle.z, ankle.x - bodyState.links.front().position.x,
                            bodyState.links[foot].linearVelocity.length(), bodyState.links[foot].linearVelocity.x,
                            bodyState.links[foot].linearVelocity.y, bodyState.links[foot].linearVelocity.z,
                            ballState.position.x - bodyState.links.front().position.x,
                            ballState.position.y - bodyState.links.front().position.y, ballState.position.z);
                }
            }
            const Vec3 capsule = scene->characterState().velocity;
            const Vec3 relative = Vec3 { ballState.linearVelocity.x - capsule.x, ballState.linearVelocity.y - capsule.y, 0.0f };
            const Vec3 offset = ballState.position - bodyState.links.front().position;
            if (receptionAt >= 0.0f && !receptionAfterShown) {
                receptionAfterShown = true;
                std::printf("  RECEPCAO %s: depois da forca, bola %.2f m/s (antes %.2f), relativa ao corpo %.2f m/s, vertical %.2f\n",
                    scenario.name.c_str(), std::hypot(ballState.linearVelocity.x, ballState.linearVelocity.y),
                    std::hypot(receptionBallBefore.x, receptionBallBefore.y), relative.length(), ballState.linearVelocity.z);
            }
            if (receptionAt >= 0.0f && !receptionHalfShown && t >= receptionAt + 0.5f) {
                receptionHalfShown = true;
                std::printf("  RECEPCAO %s: 0,5 s depois, bola a %.2f m da pelve (x %+.2f), relativa %.2f m/s, altura %.2f; tornozelo do gesto no alto %.2f m\n",
                    scenario.name.c_str(), std::hypot(offset.x, offset.y), offset.x, relative.length(), ballState.position.z,
                    receptionFootPeak);
            }
        }
        if (step < 0) {
            previousLinks = bodyState.links;
            for (std::size_t i = 0; i < bodyState.links.size() && i < previousVelocities.size(); ++i)
                previousVelocities[i] = bodyState.links[i].linearVelocity;
            previousMode = mode();
            continue;
        }
        if (std::getenv("XALVO") && !legacy.animationPose.linkPositions.empty()) {
            static std::vector<Vec3> lastTarget;
            const auto& pose = legacy.animationPose;
            if (lastTarget.size() != pose.linkPositions.size()) lastTarget = pose.linkPositions;
            std::printf("  t %.3f erro %.3f |", t, bodyState.guideErrorMeters);
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                const std::string& id = profile.links[i].id;
                const std::string wanted = std::getenv("XALVO");
                if (wanted.size() > 1 ? id != wanted : (id != "RightThigh" && id != "RightShin" && id != "RightFoot"))
                    continue;
                const Vec3 a = pose.linkPositions[i];
                const Vec3 b = bodyState.links[i].position;
                std::printf(" %s alvo %.3f %.3f %.3f d %.3f fis-alvo %.3f v %.2f |", id.c_str(), a.x, a.y, a.z,
                    (a - lastTarget[i]).length(), (b - a).length(), bodyState.links[i].linearVelocity.length());
            }
            {
                const Vec3 p0 = pose.linkPositions[0];
                const Vec3 o0 = legacy.defaultLocomotion.output().pose.linkPositions.empty() ? Vec3 {}
                    : legacy.defaultLocomotion.output().pose.linkPositions[0];
                const auto cs = scene->characterState();
                std::printf(" pelve %.3f %.3f %.3f loc %.3f %.3f %.3f cap %.3f %.3f %.3f |", p0.x, p0.y, p0.z, o0.x, o0.y,
                    o0.z, cs.position.x, cs.position.y, cs.position.z);
            }
            std::printf("\n");
            lastTarget = pose.linkPositions;
        }

        if (std::getenv("XATIVO") && isActive() && step % (std::getenv("XATIVO")[0] == '2' ? 1 : 6) == 0) {
            const ActiveBodyOutput3D& a = athlete.activeOutput;
            std::printf("  t %.2f captura fora %.3f | raiz %.3f m giro %.3f rad link %.3f m | apoio %d complac %.2f | base %.3f%s | pedido %.0f/%.0f/%.0f entregue %.0f/%.0f/%.0f ajuda %.0f/%.0f%s%s\n",
                t, athlete.stance.captureOutside(), a.rootError, a.orientationError, a.linkError, a.supportCount, a.compliance,
                athlete.stance.mismatch(), athlete.stance.stepping() ? " PASSO" : "", a.requestedForce.x,
                a.requestedForce.y, a.requestedForce.z, a.deliveredForce.x, a.deliveredForce.y, a.deliveredForce.z,
                a.assistForce.x, a.assistForce.y, a.knockout ? " NOCAUTE" : "", a.readyToAnimate ? " PRONTO" : "");
            if (std::getenv("XGEO")) {
                const BodyObservation3D o = athlete.observer.observe(profile, bodyState, Dt);
                const Vec3 mid = (o.feet[0].soleCenter + o.feet[1].soleCenter) * 0.5f;
                const Vec3 cap = scene->characterState().position;
                std::printf("      geo cm %+.3f %+.3f v %+.2f %+.2f | pelve %+.3f %+.3f | alvo %+.3f %+.3f (anc %+.3f %+.3f) z %.3f/%.3f | pes E %+.3f %+.3f %s%.0fN D %+.3f %+.3f %s%.0fN | capsula %+.3f %+.3f\n",
                    o.centerOfMass.x - mid.x, o.centerOfMass.y - mid.y, o.centerOfMassVelocity.x, o.centerOfMassVelocity.y,
                    o.pelvisPosition.x - mid.x, o.pelvisPosition.y - mid.y, a.targetRoot.x - mid.x, a.targetRoot.y - mid.y,
                    athlete.stance.baseShift().x, athlete.stance.baseShift().y, a.targetRoot.z, o.pelvisPosition.z,
                    o.feet[0].soleCenter.x - mid.x, o.feet[0].soleCenter.y - mid.y, o.feet[0].supporting ? "*" : "-", o.feet[0].loadNewtons,
                    o.feet[1].soleCenter.x - mid.x, o.feet[1].soleCenter.y - mid.y, o.feet[1].supporting ? "*" : "-", o.feet[1].loadNewtons,
                    cap.x - mid.x, cap.y - mid.y);
            }
        }
        if (std::getenv("XPES") && step % 12 == 0) {
            const PhysicalFeetAssistOutput3D& f = athlete.feetAssistOutput;
            std::printf("  t %.2f pes: apoio %d servo %.0f %.0f %.0f | aos pes %.0f %.0f %.0f | o chao deu %.0f %.0f %.0f\n", t,
                f.supportCount, f.requestedForce.x, f.requestedForce.y, f.requestedForce.z, f.routedForce.x,
                f.routedForce.y, f.routedForce.z, f.deliveredForce.x, f.deliveredForce.y, f.deliveredForce.z);
        }
        if (std::getenv("XMARCHA")) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            std::printf("  t %.3f marcha %d mov %.2f | E %c sai %.3f pousa %.3f balanco %.3f | D %c sai %.3f pousa %.3f balanco %.3f\n",
                t, o.cycling ? 1 : 0, o.movingShare, o.cyclePlanted[0] ? 'A' : 'B', o.secondsToLiftoff[0],
                o.cycleSecondsToTouchdown[0], o.swingSeconds[0], o.cyclePlanted[1] ? 'A' : 'B', o.secondsToLiftoff[1],
                o.cycleSecondsToTouchdown[1], o.swingSeconds[1]);
        }
        if (std::getenv("XALVOPE")) {
            const int side = std::atoi(std::getenv("XALVOPE"));
            const FootTarget3D& f = athlete.footTargets[static_cast<std::size_t>(side)];
            static Vec3 last;
            std::printf("  t %.3f pe %d %c alvo %.3f %.3f %.3f (passo %.4f) | corpo %.3f %.3f %.3f\n", t, side,
                "FLSPD"[static_cast<int>(f.mode)], f.position.x, f.position.y, f.position.z, (f.position - last).length(),
                bodyState.links[side == 0 ? leftFoot : rightFoot].position.x,
                bodyState.links[side == 0 ? leftFoot : rightFoot].position.y,
                bodyState.links[side == 0 ? leftFoot : rightFoot].position.z);
            last = f.position;
        }
        if (std::getenv("XPEG") && settings.footstepLocomotion
            && (mode() == BodyModeKind3D::Animated || mode() == BodyModeKind3D::Returning)) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            const AthleteLegs3D& legs = athlete.footsteps.legs();
            std::printf("  t %.3f mov %.2f v %.2f |", t, o.movingShare, std::hypot(scene->characterState().velocity.x,
                scene->characterState().velocity.y));
            for (std::size_t side = 0; side < 2; ++side) {
                const char* mode = "LSWP" + static_cast<int>(athlete.footTargets[side].mode);
                const Vec3 stylePos = o.pose.linkPositions[legs.foot[side]];
                const Vec3 target = athlete.footTargets[side].position;
                std::printf(" %c %c%s d %.3f z %.3f alc %.3f lo %.2f td %.2f |", side == 0 ? 'E' : 'D',
                    "FLSPD"[static_cast<int>(athlete.footTargets[side].mode)], o.footPlanted[side] ? "*" : " ",
                    std::hypot(target.x - stylePos.x, target.y - stylePos.y), target.z - stylePos.z,
                    athlete.legSynthesis.reachError[side], o.secondsToLiftoff[side], o.secondsToTouchdown[side]);
                (void)mode;
            }
            std::printf("\n");
        }
        // Medidas.
        // O joelho no modo padrao (no corpo e no alvo).
        if (thighs[0] < bodyState.links.size() && thighs[1] < bodyState.links.size()) {
            const bool animatedMode = mode() == BodyModeKind3D::Animated;
            for (std::size_t side = 0; side < 2; ++side) {
                const float flexion = kneeFlexion(side, bodyState.links[thighs[side]].orientation,
                    bodyState.links[shins[side]].orientation);
                const bool havePose = legacy.animationPose.linkOrientations.size() == profile.links.size();
                const float target = havePose ? kneeFlexion(side, legacy.animationPose.linkOrientations[thighs[side]],
                    legacy.animationPose.linkOrientations[shins[side]]) : -1.0f;
                const float rate = kneeBefore[side] >= 0.0f ? (flexion - kneeBefore[side]) / Dt : 0.0f;
                const float targetRate = kneeTargetBefore[side] >= 0.0f && target >= 0.0f
                    ? (target - kneeTargetBefore[side]) / Dt : 0.0f;
                const float acceleration = (rate - kneeRateBefore[side]) / Dt;
                if (animatedMode && t > 0.5f && kneeBefore[side] >= 0.0f) {
                    metrics.peakKneeExtension = std::max(metrics.peakKneeExtension, -rate);
                    metrics.peakKneeExtensionTarget = std::max(metrics.peakKneeExtensionTarget, -targetRate);
                    metrics.peakKneeAcceleration = std::max(metrics.peakKneeAcceleration, std::abs(acceleration));
                }
                if (std::getenv("XJOELHO") && animatedMode && (std::string(std::getenv("XJOELHO")) == "todos"
                        || -rate > std::atof(std::getenv("XJOELHO")) || -targetRate > std::atof(std::getenv("XJOELHO")))) {
                    const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
                    const float reachNow = athlete.legSynthesis.reachError[side];
                    // (O alcance do alvo do pe e o do pe do estilo: quadril ao
                    // tornozelo / comprimento da perna.)
                    float reachTarget = 0.0f, reachStyle = 0.0f, behind = 0.0f;
                    const AthleteLegs3D& legs = athlete.footsteps.legs();
                    if (legs.valid && !o.pose.linkPositions.empty()) {
                        const Vec3 hip = o.rootPosition + o.rootOrientation.rotate(legs.hipInPelvis[side]);
                        const float length = legs.thighLength[side] + legs.shinLength[side];
                        const Vec3 ankleTarget = athlete.footTargets[side].position
                            + athlete.footTargets[side].orientation.rotate(legs.ankleInFoot[side]);
                        const Vec3 ankleStyle = o.pose.linkPositions[legs.foot[side]]
                            + o.pose.linkOrientations[legs.foot[side]].rotate(legs.ankleInFoot[side]);
                        reachTarget = (ankleTarget - hip).length() / length;
                        reachStyle = (ankleStyle - hip).length() / length;
                        const Vec3 forward = o.rootOrientation.rotate({ 1.0f, 0.0f, 0.0f });
                        behind = (ankleStyle.x - ankleTarget.x) * forward.x + (ankleStyle.y - ankleTarget.y) * forward.y;
                    }
                    std::printf("  t %.3f %c %c%s joelho %.1f alvo %.1f | vel %.0f alvo %.0f graus/s | sai %.3f pousa %.3f | erro %.3f | alcance alvo %.3f estilo %.3f | atras do estilo %.3f\n",
                        t, side == 0 ? 'E' : 'D', settings.footstepLocomotion
                            ? "FLSPD"[static_cast<int>(athlete.footTargets[side].mode)] : (o.footPlanted[side] ? 'A' : 'B'),
                        o.footPlanted[side] ? "*" : " ", flexion, target, rate, targetRate, o.secondsToLiftoff[side],
                        o.secondsToTouchdown[side], reachNow, reachTarget, reachStyle, behind);
                }
                kneeRateBefore[side] = kneeBefore[side] >= 0.0f ? rate : 0.0f;
                kneeBefore[side] = flexion;
                kneeTargetBefore[side] = target;
            }
        }
        if (settings.footstepLocomotion && t > 0.35f)
            for (const float reach : athlete.legSynthesis.reachError)
                metrics.worstReach = std::max(metrics.worstReach, reach);
        // Pegadas novas: as pernas nunca se cruzam nem se atravessam, o pe
        // apoiado nao treme, so trava no chao e no ar segue o estilo.
        const bool defaultMode = mode() == BodyModeKind3D::Animated
            || mode() == BodyModeKind3D::Returning;
        if (t > 0.35f && defaultMode && leftFoot < bodyState.links.size() && rightFoot < bodyState.links.size()
            && shins[0] < bodyState.links.size() && shins[1] < bodyState.links.size()) {
            const Vec3 forward = [&] {
                const Vec3 f = (bodyState.links.front().orientation * profile.links.front().modelOrientation.conjugate())
                                   .rotate({ 1.0f, 0.0f, 0.0f });
                const Vec3 flat { f.x, f.y, 0.0f };
                return flat.lengthSquared() > 1e-6f ? flat.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
            }();
            const Vec3 leftward { -forward.y, forward.x, 0.0f };
            const Vec3 apart = bodyState.links[leftFoot].position - bodyState.links[rightFoot].position;
            if (dot(apart, leftward) < -0.02f && std::abs(dot(apart, forward)) < 0.25f) ++metrics.legCrossTicks;
            if (dot(apart, leftward) < 0.0f) ++metrics.legOrderTicks;
            if (dot(bodyState.links[shins[0]].position - bodyState.links[shins[1]].position, leftward) < 0.0f)
                ++metrics.kneeOrderTicks;
            if (std::getenv("XCRUZA")) {
                const float yaw = legacy.defaultLocomotion.output().facingYawRadians;
                const Vec3 f2 { std::cos(yaw), std::sin(yaw), 0.0f };
                const Vec3 l2 { -f2.y, f2.x, 0.0f };
                const Vec3 targets = athlete.footTargets[0].position - athlete.footTargets[1].position;
                std::printf("  t %.3f corpo lado %+.3f frente %+.3f | alvo (rumo do estilo) lado %+.3f frente %+.3f | pelve x estilo %.0f graus | modos %c %c\n",
                    t, dot(apart, leftward), dot(apart, forward), dot(targets, l2), dot(targets, f2),
                    std::atan2(forward.x * f2.y - forward.y * f2.x, dot(forward, f2)) * 57.3f,
                    "FLSPD"[static_cast<int>(athlete.footTargets[0].mode)], "FLSPD"[static_cast<int>(athlete.footTargets[1].mode)]);
            }
            const LegPiece pieces[2][2] = {
                { legPiece(profile, bodyState.links[shins[0]], shins[0]), legPiece(profile, bodyState.links[leftFoot], leftFoot) },
                { legPiece(profile, bodyState.links[shins[1]], shins[1]), legPiece(profile, bodyState.links[rightFoot], rightFoot) } };
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j) {
                    const LegPiece& a = pieces[0][i];
                    const LegPiece& b = pieces[1][j];
                    const float overlap = a.radius + b.radius - segmentDistance(a.a, a.b, b.a, b.b);
                    metrics.legOverlap = std::max(metrics.legOverlap, overlap);
                    if (std::getenv("XSOBRE") && overlap > 0.005f)
                        std::printf("  t %.3f sobreposicao %.3f m: %s E x %s D | empurrao E %.3f D %.3f | pes lado %+.3f frente %+.3f | altura E %.3f D %.3f\n", t,
                            overlap, i == 0 ? "canela" : "pe", j == 0 ? "canela" : "pe",
                            athlete.legClearance.applied(0).length(), athlete.legClearance.applied(1).length(),
                            dot(apart, leftward), dot(apart, forward), bodyState.links[leftFoot].position.z,
                            bodyState.links[rightFoot].position.z);
                }
        }
        if (settings.footstepLocomotion && athlete.footsteps.legs().valid) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            airborneSeconds = o.airborne ? airborneSeconds + Dt : 0.0f;
            for (std::size_t side = 0; side < 2; ++side) {
                const FootTarget3D& target = athlete.footTargets[side];
                const bool stance = defaultMode && target.mode == FootMode3D::Stance;
                const float vz = (target.position.z - footTargetZBefore[side]) / Dt;
                const std::size_t footLink = side == 0 ? leftFoot : rightFoot;
                const float bodyVz = footLink < bodyState.links.size() ? bodyState.links[footLink].linearVelocity.z : 0.0f;
                if (t > 0.35f && stance && footModeBefore[side] == FootMode3D::Stance) {
                    const bool targetFlip = vz * footTargetVzBefore[side] < 0.0f && std::abs(vz) > 0.03f
                        && std::abs(footTargetVzBefore[side]) > 0.03f;
                    const bool bodyFlip = bodyVz * footBodyVzBefore[side] < 0.0f && std::abs(bodyVz) > 0.06f
                        && std::abs(footBodyVzBefore[side]) > 0.06f;
                    if (targetFlip && targetFlipBefore[side] > 0) ++metrics.stanceTremor;
                    if (bodyFlip && bodyFlipBefore[side] > 0) ++metrics.bodyTremor;
                    targetFlipBefore[side] = targetFlip ? 2 : std::max(0, targetFlipBefore[side] - 1);
                    bodyFlipBefore[side] = bodyFlip ? 2 : std::max(0, bodyFlipBefore[side] - 1);
                } else {
                    targetFlipBefore[side] = bodyFlipBefore[side] = 0;
                }
                if (t > 0.35f && stance && footModeBefore[side] != FootMode3D::Stance) {
                    const float sole = athlete.footsteps.soleHeight(static_cast<int>(side), target.position, target.orientation);
                    const float gap = sole - athleteGroundHeight3D(*scene, profile, target.position);
                    if (std::getenv("XTRAVA"))
                        std::printf("  t %.3f trava %c em %.2f %.2f %.3f folga %+.3f\n", t, side == 0 ? 'E' : 'D',
                            target.position.x, target.position.y, target.position.z, gap);
                    if (std::abs(gap) > std::abs(metrics.lockGap)) metrics.lockGap = gap;
                }
                if (defaultMode && airborneSeconds > 0.15f && !o.pose.linkPositions.empty()) {
                    const Vec3 style = o.pose.linkPositions[athlete.footsteps.legs().foot[side]];
                    metrics.airPull = std::max(metrics.airPull, (target.position - style).length());
                }
                footModeBefore[side] = defaultMode ? target.mode : FootMode3D::Free;
                footTargetVzBefore[side] = vz;
                footTargetZBefore[side] = target.position.z;
                footBodyVzBefore[side] = bodyVz;
            }
        }
        // A troca fisico -> padrao (plano v4, etapa C): da saida do fisico ate
        // 0,6 s depois do guiado, a pelve contra a capsula, a postura e os
        // picos.
        if (previousMode == BodyModeKind3D::Physical && mode() != BodyModeKind3D::Physical) {
            transitionStart = t;
            transitionGuided = -1.0f;
        }
        if (transitionStart >= 0.0f && transitionGuided < 0.0f && mode() != BodyModeKind3D::Physical
            && !isActive())
            transitionGuided = t;
        if (mode() == BodyModeKind3D::Physical) {
            transitionStart = -1.0f;
            transitionChest.clear();
        }
        if (transitionStart >= 0.0f && (transitionGuided < 0.0f || t - transitionGuided < 0.6f)
            && !bodyState.links.empty()) {
            const PhysicsBodyState3D& pelvisLink = bodyState.links.front();
            const Vec3 capsuleVelocity = scene->characterState().velocity;
            const float pelvisSpeed = std::hypot(pelvisLink.linearVelocity.x, pelvisLink.linearVelocity.y);
            const float capsuleSpeed = std::hypot(capsuleVelocity.x, capsuleVelocity.y);
            metrics.transitionSpeedGap = std::max(metrics.transitionSpeedGap, std::abs(pelvisSpeed - capsuleSpeed));
            float worst = 0.0f;
            for (std::size_t i = 0; i < bodyState.links.size() && i < previousVelocities.size(); ++i)
                worst = std::max(worst, ((bodyState.links[i].linearVelocity - previousVelocities[i]) * (1.0f / Dt)).length());
            metrics.transitionPeak = std::max(metrics.transitionPeak, worst);
            const Vec3 chest = chestLink < bodyState.links.size()
                ? (bodyState.links[chestLink].orientation * profile.links[chestLink].modelOrientation.conjugate())
                      .rotate({ 1.0f, 0.0f, 0.0f })
                : Vec3 {};
            // (A postura: o quanto o peito inclina em 0,1 s.)
            transitionChest.push_back(chest.z);
            if (transitionChest.size() > 12) {
                metrics.transitionTrunkRate = std::max(metrics.transitionTrunkRate,
                    std::abs(transitionChest.back() - transitionChest[transitionChest.size() - 13]));
            }
            metrics.transitionSpeedMax = std::max(metrics.transitionSpeedMax, pelvisSpeed);
            if (std::getenv("XTROCA")) {
                const float poseRoot = legacy.animationPose.linkPositions.empty() ? 0.0f : legacy.animationPose.linkPositions.front().z;
                static Vec3 lastPoseRoot;
                const Vec3 poseRootNow = legacy.animationPose.linkPositions.empty() ? Vec3 {} : legacy.animationPose.linkPositions.front();
                const float poseSpeed = std::hypot(poseRootNow.x - lastPoseRoot.x, poseRootNow.y - lastPoseRoot.y) / Dt;
                lastPoseRoot = poseRootNow;
                std::printf("  t %.3f troca %-7s | pelve %.2f m/s pose %.2f capsula %.2f pedido %.2f | pelve z %.3f pose z %.3f | peito %+.2f | pico %.0f\n",
                    t, modeOf(), pelvisSpeed, poseSpeed, capsuleSpeed,
                    std::hypot(control.intent.requestedVelocityWorld.x, control.intent.requestedVelocityWorld.y),
                    pelvisLink.position.z, poseRoot, chest.z, worst);
            }
        }
        if (std::getenv("XAJUDA") && mode() == BodyModeKind3D::Physical) {
            const AdaptivePhysicalOutput3D& a = legacy.adaptivePhysics.output();
            const double weight = profile.totalMassKg * 9.81;
            if (a.valid) {
                ++assistTally.ticks;
                assistTally.rootUp += a.rootForceWorld.z / weight;
                assistTally.rootPlanar += std::hypot(a.rootForceWorld.x, a.rootForceWorld.y) / weight;
                assistTally.rootTorquePerKg += a.rootTorqueWorld.length() / profile.totalMassKg;
                for (const Vec3& f : a.footForceWorld) {
                    assistTally.feetUp += f.z / weight;
                    assistTally.feetPlanar += std::hypot(f.x, f.y) / weight;
                }
                for (const Vec3& torque : a.linkTorqueWorld) assistTally.linkTorquePerKg += torque.length() / profile.totalMassKg;
                assistTally.feetDelivered += athlete.feetAssistOutput.deliveredForce.length() / weight;
            }
        }
        // XVELOCIDADE=1: a cada 0,25 s, a velocidade pedida, a da capsula e a da
        // pelve (no plano) - onde o fisico perde velocidade.
        if (std::getenv("XVELOCIDADE") && step % 30 == 0 && !bodyState.links.empty()) {
            const Vec3 asked = control.intent.requestedVelocityWorld;
            // (Sem a capsula: a do plano do comandado.)
            const Vec3 capsuleVelocity = context.noCapsule ? athlete.commandPlanVelocity : scene->characterState().velocity;
            const Vec3 pelvisVelocity = bodyState.links.front().linearVelocity;
            const float bodyHeight = scene->characterState().crouched ? avatar.crouchedHeight : avatar.standingHeight;
            std::printf("  t %.2f %s pedida %.2f capsula %.2f pelve %.2f m/s (pes da capsula z %.3f)\n", t, modeOf(),
                std::hypot(asked.x, asked.y), std::hypot(capsuleVelocity.x, capsuleVelocity.y),
                std::hypot(pelvisVelocity.x, pelvisVelocity.y), scene->characterState().position.z - bodyHeight * 0.5f);
        }
        if (effectWanted && t > 1.0f && !bodyState.links.empty()) {
            const PhysicsBodyState3D& pelvis = bodyState.links.front();
            const float yaw = locomotionFrameYaw3D(profile, pelvis.orientation);
            effect.speed += std::hypot(pelvis.linearVelocity.x, pelvis.linearVelocity.y);
            effect.lateral += -std::sin(yaw) * pelvis.linearVelocity.x + std::cos(yaw) * pelvis.linearVelocity.y;
            effect.pelvis += pelvis.position.z - athlete.groundHeight;
            ++effect.samples;
            const CharacterLocomotionTelemetry3D& lt = legacy.locomotion.telemetry();
            for (std::size_t side = 0; side < 2; ++side) {
                if (lt.footPlanted[side] && !effect.planted[side]) ++effect.touchdowns;
                effect.planted[side] = lt.footPlanted[side];
            }
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                const bool left = profile.links[i].id == "LeftFoot", right = profile.links[i].id == "RightFoot";
                if ((left || right) && !lt.footPlanted[left ? 0 : 1] && i < bodyState.links.size())
                    effect.footPeak = std::max(effect.footPeak, bodyState.links[i].position.z - athlete.groundHeight);
            }
        }
        // XPRONTO=1: as condicoes da volta do fisico ao padrao, a cada 0,1 s.
        if (std::getenv("XPRONTO") && mode() == BodyModeKind3D::Physical && step % 12 == 0) {
            const AthleteTelemetry3D telemetry = athleteTelemetry3D(athlete);
            std::printf("  t %.2f estado %d calma %.2f ajuda %.2f desde-impacto %.2f trava %.2f perturbacao %.2f cambaleio %.2f\n",
                t, telemetry.fallen ? 9 : telemetry.gettingUp ? 10 : static_cast<int>(legacy.locomotion.telemetry().state),
                telemetry.calmSeconds, telemetry.assistLevel, athlete.modeRules.secondsSinceImpact(),
                athlete.modeRules.lockoutLeft(), telemetry.perturbation, telemetry.stagger);
        }
        if (mode() != previousMode) {
            if (mode() == BodyModeKind3D::Physical && metrics.trail.find("volta>animado") != std::string::npos)
                ++metrics.relapses;
            ++metrics.modeSwitches;
            metrics.trail += std::string(">") + modeOf();
            if (dump != nullptr) std::fprintf(dump, "E %s\n", modeOf());
            // XTROCAS=1: cada troca com o instante e, pela tabela (R3), o motivo,
            // o valor, o limiar e o que bateu no tick.
            if (std::getenv("XTROCAS")) {
                const AthleteTelemetry3D telemetry = athleteTelemetry3D(athlete);
                const ContactAssessment3D& c = telemetry.contacts;
                const ModeDecision3D& d = telemetry.lastDecision;
                std::printf("  t %.3f -> %s | ultima decisao: %s %.3f (limiar %.3f) | contato: link %s impulso %.2f N.s "
                            "Dv corpo %.3f link %.2f penetracao %.3f%s\n",
                    t, modeOf(), modeEventName3D(d.event), d.value, d.threshold,
                    c.link < profile.links.size() ? profile.links[c.link].id.c_str() : "?", c.impactImpulse,
                    c.bodyDeltaV, c.linkDeltaV, c.penetrationMeters, c.fromPenetration ? " (penetracao)" : "");
            }
        }
        previousMode = mode();
        (mode() == BodyModeKind3D::Animated ? metrics.animatedSeconds
            : mode() == BodyModeKind3D::Physical ? metrics.physicalSeconds
            : isActive() ? metrics.activeSeconds : metrics.returningSeconds) += Dt;
        metrics.assistImpulse = athlete.activeOutput.assistImpulseTotal;
        for (std::size_t i = 0; i < bodyState.links.size() && i < profile.links.size(); ++i) {
            const PhysicsBodyState3D& link = bodyState.links[i];
            const std::string& id = profile.links[i].id;
            const Vec3 acceleration = (link.linearVelocity - previousVelocities[i]) * (1.0f / Dt);
            if (t > 0.35f) {
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (std::getenv("XLINK") && id == std::getenv("XLINK"))
                    std::printf("  t %.3f %s pos %.4f %.4f %.4f vel %.3f %.3f %.3f acel %.0f\n", t, id.c_str(), link.position.x,
                        link.position.y, link.position.z, link.linearVelocity.x, link.linearVelocity.y, link.linearVelocity.z,
                        acceleration.length());
                if (std::getenv("XPICOTODOS") && acceleration.length() > std::atof(std::getenv("XPICOTODOS")))
                    std::printf("  t %.3f %s acel %.0f salto %.3f\n", t, id.c_str(), acceleration.length(),
                        (link.position - (previousLinks[i].position + link.linearVelocity * Dt)).length());
                if (std::getenv("XPICO") && leg && acceleration.length() > std::atof(std::getenv("XPICO"))) {
                    const auto mode = [&](std::size_t side) { return "FLSPD"[static_cast<int>(athlete.footTargets[side].mode)]; };
                    std::printf("  t %.3f %s acel %.0f | pes %c %c alcance %.3f %.3f\n", t, id.c_str(), acceleration.length(),
                        mode(0), mode(1), athlete.legSynthesis.reachError[0], athlete.legSynthesis.reachError[1]);
                }
                (leg ? metrics.peakLegAcceleration : metrics.peakUpperAcceleration) = std::max(
                    leg ? metrics.peakLegAcceleration : metrics.peakUpperAcceleration, acceleration.length());
                // Salto de pose: o link andou mais do que a velocidade dele explica.
                const Vec3 expected = previousLinks[i].position + link.linearVelocity * Dt;
                metrics.worstPoseJump = std::max(metrics.worstPoseJump, (link.position - expected).length());
            }
            previousVelocities[i] = link.linearVelocity;
            if ((id == "LeftFoot" || id == "RightFoot") && footGeometry.valid) {
                // O ponto que toca o chao (o mais baixo: calcanhar ou base dos
                // dedos) - o pe que rola gira em volta dele; a origem do link
                // anda ate com o pe parado no chao.
                const std::size_t side = id == "LeftFoot" ? 0 : 1;
                const Vec3 heel = link.position + link.orientation.rotate(footGeometry.heel[side]);
                const Vec3 toe = link.position + link.orientation.rotate(footGeometry.toeBase[side]);
                const Vec3 contact = heel.z < toe.z ? heel : toe;
                const Vec3 before = previousLinks[i].position
                    + previousLinks[i].orientation.rotate(heel.z < toe.z ? footGeometry.heel[side] : footGeometry.toeBase[side]);
                const bool planted = settings.footstepLocomotion
                    ? athlete.footTargets[side].mode == FootMode3D::Stance
                    : legacy.defaultLocomotion.output().footPlanted[side] && !legacy.defaultLocomotion.output().airborne;
                if (t > 0.35f && planted
                    && (mode() == BodyModeKind3D::Animated || mode() == BodyModeKind3D::Returning)) {
                    const float slide = std::hypot(contact.x - before.x, contact.y - before.y) / Dt;
                    metrics.plantedSlide.push_back(slide);
                    if (std::getenv("XDESLIZE") && slide > 0.3f)
                        std::printf("  t %.3f %s desliza %.2f m/s (%s) alcance %.3f\n", t, id.c_str(), slide,
                            heel.z < toe.z ? "calcanhar" : "dedos", athlete.legSynthesis.reachError[side]);
                }
            }
            if (id == "LeftFoot" || id == "RightFoot") {
                const auto& collider = profile.links[i].collider;
                const Vec3 center = link.position + link.orientation.rotate(collider.localPosition);
                if (t > 0.35f && center.z < collider.boxHalfExtents.z + 0.03f) {
                    metrics.footSlide.push_back(std::hypot(link.linearVelocity.x, link.linearVelocity.y));
                    if (std::getenv("XPERAPIDO") && metrics.footSlide.back() > std::atof(std::getenv("XPERAPIDO")))
                        std::printf("  t %.3f %s rente ao chao a %.1f m/s (%s), centro %.3f m\n", t, id.c_str(),
                            metrics.footSlide.back(), modeOf(), center.z);
                }
            }
        }
        // Plano v5 (E2): a capsula prevista 0,3 s antes contra a de verdade.
        if (t > 0.5f && legacy.defaultLocomotion.output().valid) {
            const PhysicsCharacterState3D& now = scene->characterState();
            while (!metrics.bodyForecasts.empty() && metrics.bodyForecasts.front().first <= t + 1e-4f) {
                const Vec3 predicted = metrics.bodyForecasts.front().second;
                metrics.bodyForecastErrors.push_back(std::hypot(predicted.x - now.position.x, predicted.y - now.position.y));
                metrics.bodyForecasts.erase(metrics.bodyForecasts.begin());
            }
            const AthleteCapsuleResponse3D response = athleteCapsuleResponse3D(settings.capsuleResponsePercent);
            BodyForecastInput3D in;
            in.pelvisGround = { now.position.x, now.position.y, 0.0f };
            in.velocity = { now.velocity.x, now.velocity.y, 0.0f };
            in.requested = { control.intent.requestedVelocityWorld.x, control.intent.requestedVelocityWorld.y, 0.0f };
            in.acceleration = response.acceleration;
            in.deceleration = response.deceleration;
            in.brake = response.brake;
            in.turnRate = response.turnRate;
            const float yawNow = legacy.defaultLocomotion.output().facingYawRadians;
            in.facingYaw = yawNow;
            in.facingRate = std::remainder(yawNow - metrics.lastFacingYaw, 6.2831853f) / Dt;
            metrics.lastFacingYaw = yawNow;
            in.horizon = 0.35f;
            BodyForecast3D forecast;
            forecast.build(in);
            if (step % 6 == 0) metrics.bodyForecasts.push_back({ t + 0.3f, forecast.pelvisGround(0.3f) });
        }
        // Plano v5 (E1): a previsao do pouso contra o de verdade.
        if (t > 0.35f && mode() == BodyModeKind3D::Animated && legacy.defaultLocomotion.output().valid) {
            const DefaultLocomotionOutput3D& style = legacy.defaultLocomotion.output();
            const Vec3 pelvis = bodyState.links.front().position;
            const float yaw = style.facingYawRadians;
            const Vec3 velocity { scene->characterState().velocity.x, scene->characterState().velocity.y, 0.0f };
            const AthleteLegs3D legs = resolveAthleteLegs3D(profile);
            for (std::size_t side = 0; side < 2 && legs.valid; ++side) {
                const bool planted = style.cyclePlanted[side];
                if (style.cycling && !planted && metrics.forecastAt[side] < 0.0f) {
                    const float end = style.cycleSecondsToTouchdown[side];
                    if (end > 0.2f && end <= 0.3f) {
                        const int k = std::clamp(static_cast<int>(std::lround(end / style.forecastStep)), 0,
                            DefaultLocomotionOutput3D::ForecastCount - 1);
                        const Vec3 local = style.footForecast[side][static_cast<std::size_t>(k)];
                        const float c = std::cos(yaw), s = std::sin(yaw);
                        metrics.forecastAt[side] = t + end;
                        metrics.forecastFoot[side] = Vec3 { pelvis.x, pelvis.y, 0.0f }
                            + Vec3 { c * local.x - s * local.y, s * local.x + c * local.y, local.z } + velocity * end;
                    }
                }
                if (planted && !metrics.forecastWasPlanted[side] && metrics.forecastAt[side] >= 0.0f) {
                    const Vec3 actual = bodyState.links[legs.foot[side]].position;
                    metrics.forecastTimeErrors.push_back(std::abs(t - metrics.forecastAt[side]));
                    metrics.forecastPositionErrors.push_back(std::hypot(actual.x - metrics.forecastFoot[side].x,
                        actual.y - metrics.forecastFoot[side].y));
                    if (std::getenv("XPREVISAO"))
                        std::printf("  t %.3f pouso %c previsto %.3f | pe %.3f %.3f previsto %.3f %.3f\n", t, side == 0 ? 'E' : 'D',
                            metrics.forecastAt[side], actual.x, actual.y, metrics.forecastFoot[side].x, metrics.forecastFoot[side].y);
                    metrics.forecastAt[side] = -1.0f;
                }
                if (planted) metrics.forecastAt[side] = planted && metrics.forecastWasPlanted[side] ? -1.0f : metrics.forecastAt[side];
                metrics.forecastWasPlanted[side] = planted;
            }
        }
        // Plano v5 (E0): a "perna louca" (coxa ou canela girando a mais de 30
        // rad/s; o joelho virando de lado mais de 60 graus num tick) e o voo
        // (os dois pes no ar).
        if (t > 0.35f && (scenario.ball || std::getenv("XLOUCA")) && mode() == BodyModeKind3D::Animated) {
            const AthleteLegs3D legs = resolveAthleteLegs3D(profile);
            if (legs.valid) {
                bool crazy = false, flipped = false;
                for (std::size_t side = 0; side < 2; ++side) {
                    for (const std::size_t link : { legs.thigh[side], legs.shin[side] }) {
                        if (link >= bodyState.links.size() || link >= previousLinks.size()) continue;
                        const Quaternion change = (previousLinks[link].orientation.conjugate() * bodyState.links[link].orientation).normalized();
                        const float angle = 2.0f * std::acos(std::clamp(std::abs(change.w), 0.0f, 1.0f));
                        metrics.legSpinMax = std::max(metrics.legSpinMax, angle / Dt);
                        if (angle / Dt > 30.0f) crazy = true;
                    }
                    const auto pole = [&](const std::vector<PhysicsBodyState3D>& links, Vec3& out) {
                        const Vec3 hip = links.front().position + links.front().orientation.rotate(legs.hipInPelvis[side]);
                        const Vec3 knee = links[legs.shin[side]].position;
                        const Vec3 ankle = links[legs.foot[side]].position + links[legs.foot[side]].orientation.rotate(legs.ankleInFoot[side]);
                        const Vec3 axis = ankle - hip;
                        if (axis.lengthSquared() < 1e-6f) return false;
                        const Vec3 unit = axis.normalized();
                        const Vec3 off = (knee - hip) - unit * dot(knee - hip, unit);
                        if (off.length() < 0.04f) return false;
                        out = off.normalized();
                        return true;
                    };
                    Vec3 before, now;
                    if (pole(previousLinks, before) && pole(bodyState.links, now) && dot(before, now) < 0.5f) flipped = true;
                    if (std::getenv("XLOUCA")) {
                        // (Os giros da coxa e da canela e o joelho deste lado.)
                        float spin[2] {};
                        int k = 0;
                        for (const std::size_t link : { legs.thigh[side], legs.shin[side] }) {
                            const Quaternion change = (previousLinks[link].orientation.conjugate() * bodyState.links[link].orientation).normalized();
                            spin[k++] = 2.0f * std::acos(std::clamp(std::abs(change.w), 0.0f, 1.0f)) / Dt;
                        }
                        if (spin[0] > std::atof(std::getenv("XLOUCA")) || spin[1] > std::atof(std::getenv("XLOUCA"))) {
                            const Vec3 hip = bodyState.links.front().position + bodyState.links.front().orientation.rotate(legs.hipInPelvis[side]);
                            const Vec3 knee = bodyState.links[legs.shin[side]].position;
                            const Vec3 ankle = bodyState.links[legs.foot[side]].position
                                + bodyState.links[legs.foot[side]].orientation.rotate(legs.ankleInFoot[side]);
                            const float a = (knee - hip).length(), b = (ankle - knee).length(), c = (ankle - hip).length();
                            const float bend = std::acos(std::clamp((a * a + b * b - c * c) / (2.0f * a * b), -1.0f, 1.0f));
                            std::printf("  t %.3f LOUCA %c coxa %.1f canela %.1f rad/s | joelho %.0f graus alcance %.3f | polo %.2f %.2f %.2f | tornozelo %.3f %.3f %.3f\n",
                                t, side == 0 ? 'E' : 'D', spin[0], spin[1], 180.0f - bend * 57.2958f, c / (a + b), now.x, now.y, now.z,
                                ankle.x, ankle.y, ankle.z);
                        }
                    }
                }
                if (crazy) ++metrics.crazyLegTicks;
                if (flipped) ++metrics.kneeFlipTicks;
                if (scenario.ball)
                    for (std::size_t side = 0; side < 2; ++side) {
                        // (O pe entrando mais de 2 cm na bola.)
                        const std::size_t link = legs.foot[side];
                        const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
                        if (!ragdollColliderHasBox3D(collider)) continue;
                        const PhysicsBodyState3D& foot = bodyState.links[link];
                        const Quaternion q = (foot.orientation * collider.localOrientation).normalized();
                        const Vec3 center = foot.position + foot.orientation.rotate(collider.localPosition);
                        const Vec3 local = q.conjugate().rotate(ballState.position - center);
                        const float radius = scene->inflatedBallProfile(ballHandle).radiusMeters;
                        const float inside = radius - ragdollColliderClosestPoint3D(collider, local).distance;
                        if (inside > 0.02f) {
                            ++metrics.footOnBallTicks;
                            if (std::getenv("XPEBOLA"))
                                std::printf("  t %.3f PE NA BOLA %c entra %.3f m | pe z %.3f\n", t, side == 0 ? 'E' : 'D',
                                    inside, foot.position.z);
                        }
                    }
                const float left = bodyState.links[legs.foot[0]].position.z, right = bodyState.links[legs.foot[1]].position.z;
                const float flightBefore = metrics.flightRun;
                metrics.flightRun = left > 0.10f && right > 0.10f ? metrics.flightRun + Dt : 0.0f;
                if (std::getenv("XVOO") && flightBefore > 0.05f && metrics.flightRun == 0.0f)
                    std::printf("  t %.3f fim do voo de %.3f s\n", t, flightBefore);
                metrics.longestFlight = std::max(metrics.longestFlight, metrics.flightRun);
            }
        }
        previousLinks = bodyState.links;
        metrics.lowestPelvis = std::min(metrics.lowestPelvis, bodyState.links.front().position.z);
        if (bodyState.links.front().position.z < 0.5f) metrics.fell = true;
        if (scenario.ball) metrics.ballSpeedAfter = std::max(metrics.ballSpeedAfter, ballState.linearVelocity.length());
        if (dump != nullptr && step % every == 0) {
            dumpFrame(dump, profile, bodyState, t, athleteDumpModeCode3D(mode()));
            if (scenario.ball)
                std::fprintf(dump, "K %.4f %.4f %.4f %.3f %.3f %.3f\n", ballState.position.x, ballState.position.y,
                    ballState.position.z, ballState.linearVelocity.x, ballState.linearVelocity.y,
                    ballState.linearVelocity.z);
        }
    }
    metrics.finalMode = mode();
    if (std::getenv("XAJUDA") && assistTally.ticks > 0) {
        const double n = assistTally.ticks;
        std::printf("  AJUDA (%d ticks no fisico): pelve vertical %.3f peso, pelve no plano %.3f peso, torque pelve %.2f N.m/kg | "
                    "pes vertical %.3f, no plano %.3f | torque nos ossos %.2f N.m/kg | ajuda nos pes entregue %.3f peso\n",
            assistTally.ticks, assistTally.rootUp / n, assistTally.rootPlanar / n, assistTally.rootTorquePerKg / n,
            assistTally.feetUp / n, assistTally.feetPlanar / n, assistTally.linkTorquePerKg / n, assistTally.feetDelivered / n);
    }
    if (!bodyState.links.empty())
        metrics.travel = std::hypot(bodyState.links.front().position.x, bodyState.links.front().position.y);
    metrics.worstLiftCorrection = athlete.footsteps.telemetry().worstLiftCorrection;
    metrics.settleSteps = athlete.footsteps.telemetry().settleSteps;
    metrics.earlyLifts = athlete.footsteps.telemetry().earlyLifts;
    metrics.catchSteps = athlete.footsteps.telemetry().catchSteps;
    metrics.startSteps = athlete.footsteps.telemetry().startSteps;
    if (effectWanted && effect.samples > 0) {
        const double n = effect.samples;
        std::printf("  EFEITO %-14s vel %.3f lado %+.3f m/s | pousos/s %.2f | pe no ar max %.3f m | pelve %.3f m | "
                    "passos do planejador %u | %s\n", scenario.name.c_str(), effect.speed / n, effect.lateral / n,
            effect.touchdowns / (n * Dt), effect.footPeak, effect.pelvis / n, legacy.locomotion.contactPlan().stepsStarted,
            metrics.fell ? "CAIU" : "de pe");
    }
    if (std::getenv("XVIRADA") && auxStats.ticks > 0)
        std::printf("  VIRADA: %s | manobra %ld ticks de %.2f a %.2f s | angulo %.0f entrada %.2f m/s | escala min %.2f | inclinacao max %.3f rad | vel min %.2f fim %.2f m/s\n",
            metrics.fell ? "CAIU" : "de pe", turnStats.ticks, turnStats.firstAt, turnStats.lastAt, turnStats.degrees,
            turnStats.entry, turnStats.minScale, turnStats.maxLean, turnStats.minSpeed < 1.0e8f ? turnStats.minSpeed : 0.0f,
            turnStats.endSpeed);
    if (std::getenv("XAUX") && auxStats.ticks > 0) {
        const double weight = 9.81 * std::max(1.0f, profile.totalMassKg);
        const double n = static_cast<double>(auxStats.ticks);
        std::printf("  AJUDA (fracao do peso, media/max): regua %.4f / %.4f | pelve %.4f pes %.4f levantar %.4f | torque liquido max %.4f N.m | fora da regua %.4f\n",
            auxStats.slider / n / weight, auxStats.sliderMax / weight, auxStats.pelvis / n / weight,
            auxStats.feet / n / weight, auxStats.getUp / n / weight, auxStats.torqueMax, auxStats.outside / n / weight);
        std::printf("  AJUDA (detalhe): pelve vertical %.4f plano %.4f | correcao max %.2f N.m | par de postura medio %.1f N.m | volta ao padrao (fora) %ld ticks, torque max %.1f N.m\n",
            auxStats.vertical / n / weight, auxStats.planar / n / weight, auxStats.correctionMax, auxStats.couple / n,
            auxStats.handover, auxStats.handoverTorqueMax);
    }
    if (std::getenv("XCUSTO") && !runtimeCost.empty()) {
        const std::size_t worst = static_cast<std::size_t>(
            std::max_element(runtimeCost.begin(), runtimeCost.end()) - runtimeCost.begin());
        std::printf("  CUSTO: o pior tick e o %zu (%.2f s)\n", worst, static_cast<double>(worst) * Dt);
        std::sort(runtimeCost.begin(), runtimeCost.end());
        const auto at = [&](double p) {
            return runtimeCost[std::min(runtimeCost.size() - 1, static_cast<std::size_t>(p * runtimeCost.size()))] * 1.0e6;
        };
        std::printf("  CUSTO do runtime por tick (us): p50 %.0f p95 %.0f p99 %.0f max %.0f\n", at(0.5), at(0.95), at(0.99),
            runtimeCost.back() * 1.0e6);
    }
    return metrics;
}

void printMetrics(const Scenario& scenario, const Metrics& metrics, bool footsteps) {
    std::printf("%-18s | %5.2f / %5.2f / %5.2f | %7.1f / %7.1f m/s2 | %5.3f m | %d; %.1f/%.1f/%.1f/%.1f; %.0f | %s%d pedidos, %d toques, bola max %.1f m/s\n",
        scenario.name.c_str(), percentile(metrics.footSlide, 0.5f), percentile(metrics.footSlide, 0.95f),
        percentile(metrics.footSlide, 1.0f), metrics.peakUpperAcceleration, metrics.peakLegAcceleration,
        metrics.worstPoseJump, metrics.modeSwitches, metrics.animatedSeconds, metrics.activeSeconds,
        metrics.physicalSeconds, metrics.returningSeconds, metrics.assistImpulse, metrics.fell ? "CAIU, " : "",
        metrics.requests, metrics.touches, metrics.ballSpeedAfter);
    if (metrics.gestureTicks[0] + metrics.gestureTicks[1] > 0)
        std::printf("%-18s   gestos: perna esquerda %d ticks, direita %d (as duas juntas %d) | sequencia %s\n", "",
            metrics.gestureTicks[0], metrics.gestureTicks[1], metrics.bothGestureTicks, metrics.gestureSequence.c_str());
    if (metrics.stillWorstTime >= 0.0f)
        std::printf("%-18s   PARADO (mais de 1 s): pior pe fora do lugar %.3f m em %.2f s (E %+.2f %+.2f D %+.2f %+.2f) | as duas pernas em gesto %d ticks\n", "",
            metrics.stillWorst, metrics.stillWorstTime, metrics.stillWorstFeet[0].x, metrics.stillWorstFeet[0].y,
            metrics.stillWorstFeet[1].x, metrics.stillWorstFeet[1].y, metrics.bothGestureTicks);
    if (metrics.stanceValid) {
        const auto off = [](const std::array<Vec3, 2>& a, const std::array<Vec3, 2>& b) {
            return std::max((a[0] - b[0]).length(), (a[1] - b[1]).length());
        };
        std::printf("%-18s   PES no fim: corpo fora do lugar %.3f m, animacao %.3f m, corpo x animacao %.3f m (corpo E %+.2f %+.2f D %+.2f %+.2f | comeco E %+.2f %+.2f D %+.2f %+.2f)\n",
            "", off(metrics.stanceEnd, metrics.stanceStart), off(metrics.animatedEnd, metrics.animatedStart),
            off(metrics.stanceEnd, metrics.animatedEnd), metrics.stanceEnd[0].x, metrics.stanceEnd[0].y,
            metrics.stanceEnd[1].x, metrics.stanceEnd[1].y, metrics.stanceStart[0].x, metrics.stanceStart[0].y,
            metrics.stanceStart[1].x, metrics.stanceStart[1].y);
    }
    std::printf("%-18s   andou %.2f m, recaidas %d, %s, fim %s (%s)\n", "", metrics.travel, metrics.relapses,
        metrics.fell ? "CAIU" : "de pe", bodyModeName3D(metrics.finalMode), metrics.trail.c_str());
    std::printf("%-18s   joelho esticando max %.0f graus/s (alvo %.0f), acel max %.0f graus/s2\n", "",
        metrics.peakKneeExtension, metrics.peakKneeExtensionTarget, metrics.peakKneeAcceleration);
    std::printf("%-18s   apoio %5.2f / %5.2f / %5.2f m/s%s", "", percentile(metrics.plantedSlide, 0.5f),
        percentile(metrics.plantedSlide, 0.95f), percentile(metrics.plantedSlide, 1.0f), footsteps ? "" : "\n");
    if (metrics.ballDistanceCount > 0)
        std::printf("%-18s   conduzir: bola longe (>2,5 m) %.2f s, distancia media %.2f m\n", "", metrics.ballFarSeconds,
            metrics.ballDistanceSum / static_cast<float>(metrics.ballDistanceCount));
    if (metrics.requests > 0) {
        // (O atraso do clique a forca, a perna louca e o voo.)
        float latencyMax = 0.0f;
        for (const float value : metrics.latencies) latencyMax = std::max(latencyMax, value);
        std::printf("%-18s   bola: pedidos %d toques %d | atraso max %.3f s | perna louca %d joelho virando %d giro max %.1f rad/s | voo max %.3f s | perdidos %d | pe na bola %d\n",
            "", metrics.requests, metrics.touches, latencyMax, metrics.crazyLegTicks, metrics.kneeFlipTicks,
            metrics.legSpinMax, metrics.longestFlight, metrics.lostRequests + static_cast<int>(metrics.pendingRequests.size()),
            metrics.footOnBallTicks);
    }
    if (!metrics.bodyForecastErrors.empty())
        std::printf("%-18s   previsao do corpo (0,3 s antes): %zu | med %.3f p95 %.3f max %.3f m\n", "",
            metrics.bodyForecastErrors.size(), percentile(metrics.bodyForecastErrors, 0.5f),
            percentile(metrics.bodyForecastErrors, 0.95f), percentile(metrics.bodyForecastErrors, 1.0f));
    if (!metrics.forecastTimeErrors.empty())
        std::printf("%-18s   previsao do pouso (0,2-0,3 s antes): %zu pousos | instante med %.3f max %.3f s | pe med %.3f max %.3f m\n",
            "", metrics.forecastTimeErrors.size(), percentile(metrics.forecastTimeErrors, 0.5f),
            percentile(metrics.forecastTimeErrors, 1.0f), percentile(metrics.forecastPositionErrors, 0.5f),
            percentile(metrics.forecastPositionErrors, 1.0f));
    if (metrics.capsuleUp > 0.0f || metrics.pelvisUp > 0.0f)
        std::printf("%-18s   subida max: capsula %.2f m/s, pelve %.2f m/s, pelve sobre a capsula %.2f m\n", "",
            metrics.capsuleUp, metrics.pelvisUp, metrics.pelvisOverCapsule);
    if (metrics.riseAt >= 0.0f)
        std::printf("%-18s   levantar: comeca %.2f s (peito %+.2f pelve %+.2f, %s), de pe em %.2f s\n", "", metrics.riseAt,
            metrics.riseChest, metrics.risePelvis, metrics.riseSide == 1 ? "de costas" : metrics.riseSide == 2 ? "de brucos" : "?",
            metrics.risenAt >= 0.0f ? metrics.risenAt - metrics.riseAt : -1.0f);
    if (metrics.forceTime >= 0.0f)
        std::printf("%-18s   bola: pedido %.3f forca %.3f\n", "", metrics.requestTime, metrics.forceTime);
    if (footsteps)
        std::printf(" | saida do apoio max %.3f m | alcance max %.3f m | arranca %d, acomoda %d, antes %d, no ar %d\n",
            metrics.worstLiftCorrection, metrics.worstReach, metrics.startSteps, metrics.settleSteps, metrics.earlyLifts,
            metrics.catchSteps);
    if (metrics.transitionPeak > 0.0f)
        std::printf("%-18s   troca fisico>padrao: pelve max %.2f m/s, pelve x capsula max %.2f m/s, pico %.0f m/s2, peito %.3f em 0,1 s\n",
            "", metrics.transitionSpeedMax, metrics.transitionSpeedGap, metrics.transitionPeak, metrics.transitionTrunkRate);
    std::printf("%-18s   pernas: cruzadas %d ticks, sobreposicao %.3f m, ordem trocada pes %d joelhos %d", "",
        metrics.legCrossTicks, metrics.legOverlap, metrics.legOrderTicks, metrics.kneeOrderTicks);
    if (footsteps)
        std::printf(" | apoio tremendo %d (corpo %d) | trava longe do chao %+.3f m | no ar longe do estilo %.3f m",
            metrics.stanceTremor, metrics.bodyTremor, metrics.lockGap, metrics.airPull);
    std::printf("\n");
}

// Os contratos do runtime (ctest MatterEngine.Athlete): o que cada etapa do
// rework garante, medido na fisica de verdade.
int runChecks(const ProbeContext& context) {
    int failures = 0;
    const auto run = [&](const char* name, const AthleteRuntimeSettings3D& settings, bool startPhysical) {
        for (const Scenario& scenario : scenarios())
            if (scenario.name == name) {
                const Metrics metrics = runScenario(context, scenario, settings, startPhysical, nullptr, 1);
                printMetrics(scenario, metrics, settings.footstepLocomotion);
                return metrics;
            }
        std::fprintf(stderr, "cenario sumiu: %s\n", name);
        ++failures;
        return Metrics {};
    };
    const auto expect = [&](bool ok, const char* what, const Metrics& metrics) {
        std::printf("  %s %s (%s)\n", ok ? "ok  " : "FALHOU", what, metrics.trail.c_str());
        if (!ok) ++failures;
    };
    const auto has = [](const Metrics& metrics, const char* mode) {
        return metrics.trail.find(mode) != std::string::npos;
    };
    const AthleteRuntimeSettings3D settings;
    // O Ativo (desligado por padrao desde 10/10, noite) segue na chave.
    AthleteRuntimeSettings3D activeOn;
    activeOn.activeBody = true;
    {
        const Metrics m = run("parado", settings, false);
        expect(m.modeSwitches == 0 && m.peakLegAcceleration < 5.0f, "parado: fica no padrao, quieto", m);
    }
    {
        const Metrics m = run("sprint", settings, false);
        expect(m.modeSwitches == 0, "sprint: fica no padrao", m);
    }
    {
        const Metrics m = run("caixa_leve", settings, false);
        expect(has(m, "fisico") && !has(m, "ativo")
                && (m.finalMode == BodyModeKind3D::Animated || m.finalMode == BodyModeKind3D::Returning),
            "caixa leve: o fisico e a volta ao padrao", m);
    }
    {
        const Metrics m = run("caixa_forte", settings, false);
        expect(has(m, "fisico"), "caixa forte: vai ao fisico pleno", m);
    }
    // Os casos do teste do usuario (10/10, noite): derrubado, ele levanta e
    // volta ao padrao - parado e com as teclas apertadas desde o fim do
    // levantar (andando e correndo) - e segue sem cair de novo; o encostao
    // andando e correndo nao derruba.
    for (const char* name : { "queda_levanta", "queda_levanta_anda", "queda_levanta_corre" }) {
        const Metrics m = run(name, settings, false);
        const float travel = std::string(name) == "queda_levanta_anda" ? 10.0f
            : std::string(name) == "queda_levanta_corre" ? 20.0f : 0.0f;
        char what[192];
        std::snprintf(what, sizeof(what), "%s: levanta, volta ao padrao e segue (andou %.1f m, recaidas %d)", name,
            m.travel, m.relapses);
        expect(m.finalMode == BodyModeKind3D::Animated && m.relapses == 0 && m.travel >= travel, what, m);
    }
    for (const char* name : { "encostao_andando", "encostao_correndo", "caixa_media", "caixa_correndo" }) {
        const Metrics m = run(name, settings, false);
        char what[192];
        std::snprintf(what, sizeof(what), "%s: nao derruba, volta ao padrao (pelve min %.2f m)", name, m.lowestPelvis);
        expect(!m.fell && m.relapses == 0
                && (m.finalMode == BodyModeKind3D::Animated || m.finalMode == BodyModeKind3D::Returning),
            what, m);
    }
    {
        const Metrics m = run("caixa_leve", activeOn, false);
        expect(has(m, "ativo") && !has(m, "fisico"), "Ativo ligado, caixa leve: segura sem fisico pleno", m);
        expect(m.finalMode == BodyModeKind3D::Animated, "Ativo ligado, caixa leve: volta ao padrao", m);
    }
    // O terreno (plano v2, fase E): degrau baixo, escada e rampa sem ir ao
    // fisico e sem o lancamento (a pelve longe acima da capsula), andando ate o
    // fim; o degrau alto (35 cm, parede para a capsula) pode ir ao fisico, sem
    // lancamento; encostado na parede volta ao padrao. (As pegadas sairam do
    // jogo em 11/10.)
    for (const AthleteRuntimeSettings3D* config : { &settings }) {
        const bool steps = config->footstepLocomotion;
        for (const char* name : { "degrau_10_anda", "degrau_18_anda", "degrau_25_anda", "degrau_10_corre", "degrau_18_corre",
                 "escada_anda", "escada_corre", "rampa_anda", "rampa_corre" }) {
            const Metrics m = run(name, *config, false);
            const bool running = std::string(name).find("corre") != std::string::npos;
            const float travel = running ? 12.0f : std::string(name).find("escada") != std::string::npos ? 5.0f : 6.0f;
            char what[200];
            std::snprintf(what, sizeof(what), "terreno%s, %s: sem fisico, sem lancamento (pelve %.2f m sobre a capsula), andou %.1f m",
                steps ? " (pegadas)" : "", name, m.pelvisOverCapsule, m.travel);
            expect(!has(m, "fisico") && m.pelvisOverCapsule < 0.15f * bodyScale3D(context.body) && m.travel >= travel && !m.fell, what, m);
        }
        for (const char* name : { "degrau_35_anda", "degrau_35_corre" }) {
            const Metrics m = run(name, *config, false);
            char what[160];
            std::snprintf(what, sizeof(what), "terreno%s, %s: sem lancamento (pelve %.2f m sobre a capsula)",
                steps ? " (pegadas)" : "", name, m.pelvisOverCapsule);
            expect(m.pelvisOverCapsule < 0.3f * bodyScale3D(context.body), what, m);
        }
    }
    {
        const Metrics m = run("parede_corre", settings, false);
        expect(m.finalMode != BodyModeKind3D::Physical, "terreno, parede_corre: encostado na parede volta ao padrao", m);
    }
    // A ponte fisico -> padrao (plano v2, fase D): toda volta do fisico passa
    // por ela e acaba no padrao, sem salto nos pes na troca; o levantar comeca
    // com o tronco alinhado ao lado dele e acaba de pe.
    for (const char* name : { "caixa_leve", "caixa_media", "encostao_andando", "encostao_correndo", "parede_pulo",
             "queda_levanta_corre" }) {
        const Metrics m = run(name, settings, false);
        char what[192];
        std::snprintf(what, sizeof(what), "ponte, %s: volta pela ponte ao padrao, sem salto (pe max %.1f m/s, salto %.3f m)",
            name, percentile(m.footSlide, 1.0f), m.worstPoseJump);
        expect(has(m, "ponte") && (m.finalMode == BodyModeKind3D::Animated || m.finalMode == BodyModeKind3D::Returning)
                && percentile(m.footSlide, 1.0f) < 10.0f && m.worstPoseJump < 0.03f,
            what, m);
    }
    for (const char* name : { "queda_frente", "queda_costas", "queda_lado_1900", "queda_000", "queda_180", "queda_225",
             "queda_315", "quedacorre_000", "quedacorre_180", "queda_frente_corre" }) {
        const Metrics m = run(name, settings, false);
        const float sign = m.riseSide == 2 ? -1.0f : 1.0f;
        char what[224];
        std::snprintf(what, sizeof(what), "levantar, %s: alinhado (peito %+.2f, pelve %+.2f), de pe em %.2f s, pela ponte", name,
            m.riseChest, m.risePelvis, m.risenAt >= 0.0f ? m.risenAt - m.riseAt : -1.0f);
        // (A fisica nao e deterministica entre rodadas: as vezes a pancada so
        // faz tropecar - sem queda, basta voltar pela ponte.)
        const bool rose = m.riseAt >= 0.0f && sign * m.riseChest > 0.5f && sign * m.risePelvis > 0.35f
            && m.risenAt >= 0.0f && m.risenAt - m.riseAt < 4.5f;
        expect((rose || (!m.fell && m.riseAt < 0.0f)) && has(m, "ponte") && m.finalMode != BodyModeKind3D::Physical,
            what, m);
    }
    // Os gestos de bola (24/10, a reescrita): com a bola no alcance a forca sai
    // no tick do pedido (nenhuma espera), uma vez; fora dele, nada; o gesto
    // nunca mexe no caminho da capsula nem segura o corpo.
    {
        AthleteRuntimeSettings3D gestures;
        int total = 0, failed = 0;
        for (const Scenario& scenario : scenarios()) {
            const std::string& name = scenario.name;
            // (So os de uma bola posta e o fora do alcance: conduzir, brincar e o
            // sorteado tem outros contratos.)
            if (name.rfind("gesto_", 0) != 0 || (scenario.placeAt < 0.0f && name != "gesto_fora")) continue;
            const Metrics m = runScenario(context, scenario, gestures, false, nullptr, 1);
            ++total;
            std::vector<std::string> problems;
            const bool outside = name == "gesto_fora";
            if (m.requests != 1) problems.push_back("pedidos " + std::to_string(m.requests));
            if (m.touches != (outside ? 0 : 1)) problems.push_back("toques " + std::to_string(m.touches));
            if (!outside && (m.forceTime < 0.0f || std::abs(m.forceTime - m.requestTime) > 0.5f * Dt))
                problems.push_back("a forca nao saiu no pedido");
            // O gesto toca de fato, na perna do lado da bola (a bola posta a
            // esquerda, + y, e a perna esquerda), e nada fora do alcance.
            const int expected = scenario.placeOffset.y > 0.0f ? 0 : 1;
            const int played = m.gestureTicks[0] + m.gestureTicks[1];
            if (outside && played != 0) problems.push_back("gesto fora do alcance");
            if (!outside && (m.gestureTicks[static_cast<std::size_t>(expected)] < 12
                    || m.gestureTicks[static_cast<std::size_t>(1 - expected)] != 0))
                problems.push_back("gesto na perna errada ou parado (E " + std::to_string(m.gestureTicks[0]) + ", D "
                    + std::to_string(m.gestureTicks[1]) + ")");
            if (m.fell) problems.push_back("caiu");
            if (!problems.empty()) {
                ++failed;
                std::string text;
                for (const std::string& problem : problems) text += (text.empty() ? "" : ", ") + problem;
                std::printf("  FALHOU gesto, %s: %s\n", name.c_str(), text.c_str());
            }
        }
        char what[160];
        std::snprintf(what, sizeof(what), "gestos de bola: %d de %d cenarios com a forca no tick do pedido (e nada fora do alcance)",
            total - failed, total);
        Metrics none;
        none.trail = "gestos";
        expect(failed == 0 && total > 0, what, none);
        for (const char* name : { "gesto_trote_toque", "gesto_corre_chute" }) {
            for (const Scenario& scenario : scenarios()) {
                if (scenario.name != name) continue;
                const Metrics with = runScenario(context, scenario, gestures, false, nullptr, 1);
                const Metrics without = runScenario(context, scenario, gestures, false, nullptr, 1, true);
                float apart = 0.0f;
                for (std::size_t i = 0; i < with.capsulePath.size() && i < without.capsulePath.size(); ++i)
                    apart = std::max(apart, std::hypot(with.capsulePath[i].x - without.capsulePath[i].x,
                        with.capsulePath[i].y - without.capsulePath[i].y));
                char line[160];
                std::snprintf(line, sizeof(line), "gestos de bola, %s: a capsula faz o mesmo caminho (%.4f m)", name, apart);
                expect(apart < 0.01f, line, with);
            }
        }
        // A recepcao espera a bola chegar (24/10, pedido do usuario: o pedido
        // armado, so na recepcao): apertada a 0,6 m ou bem antes (1,5 m), a
        // forca sai uma vez, depois do aperto e antes de 0,3 s, e ele fica de pe.
        for (const float distance : { 0.6f, 1.5f }) {
            for (const Scenario& scenario : scenarios()) {
                if (scenario.name != "recepcao_frente") continue;
                Scenario early = scenario;
                early.autoDistance = distance;
                const Metrics m = runScenario(context, early, gestures, false, nullptr, 1);
                const float waited = m.forceTime - m.requestTime;
                char line[200];
                std::snprintf(line, sizeof(line), "recepcao apertada a %.1f m: %d toque de %d pedido, a forca %.3f s depois do aperto",
                    distance, m.touches, m.requests, waited);
                expect(m.requests == 1 && m.touches == 1 && m.forceTime >= 0.0f && waited >= -0.5f * Dt && waited < 0.3f
                        && (distance < 1.0f || waited > 0.05f) && !m.fell, line, m);
            }
        }
        // Pulando, as areas dos pes sobem com ele (24/10, o usuario viu elas no chao).
        {
            const Metrics jump = run("gravidade_pulo_medio", gestures, false);
            char text[160];
            std::snprintf(text, sizeof(text), "pulando, a area de alcance dos pes sobe com ele (%.2f m no alto)", jump.reachBaseMax);
            expect(jump.reachBaseMax > 0.3f, text, jump);
        }
        const Metrics drive = run("gesto_conduz", gestures, false);
        char line[160];
        std::snprintf(line, sizeof(line), "gestos de bola, conduzindo: %d toques de %d pedidos, de pe", drive.touches, drive.requests);
        expect(drive.touches >= 4 && drive.touches == drive.requests && !drive.fell, line, drive);
        // O pe volta ao lugar (24/10, o "perna deslocada" do usuario: o empurrao
        // de nao cruzar ficava para sempre com o pe apoiado e parado): no teste
        // sorteado - teclas, giros, toques e chutes por 30 s -, em cada pausa
        // parada os pes ficam na pose parada e as pernas sem tranco.
        float worstStance = 0.0f, worstLeg = 0.0f;
        std::string stanceWhere;
        Metrics worstMetrics;
        for (const Scenario& scenario : scenarios()) {
            if (scenario.name.rfind("gesto_sorteio_", 0) != 0) continue;
            const Metrics m = runScenario(context, scenario, gestures, false, nullptr, 1);
            if (m.stillWorst >= worstStance) {
                worstStance = m.stillWorst;
                stanceWhere = scenario.name;
                worstMetrics = m;
            }
            worstLeg = std::max(worstLeg, m.peakLegAcceleration);
        }
        std::snprintf(line, sizeof(line), "gestos de bola, sorteado: parado, pe fora do lugar %.3f m (%s); pernas ate %.0f m/s2",
            worstStance, stanceWhere.c_str(), worstLeg);
        expect(worstStance < 0.05f && worstLeg < 1500.0f, line, worstMetrics);
    }
    // A reescrita (L0): o atleta comandado sem a capsula do jogador (o caminho
    // do treino, muitos atletas na cena) anda no fisico total como o
    // controlado: de pe e perto da mesma distancia (a fisica e caotica: so
    // mover a largada da capsula 3 mm muda o sprint em 5%, XDESLOCA).
    {
        ProbeContext commanded = context;
        commanded.noCapsule = true;
        AthleteRuntimeSettings3D physical;
        physical.defaultLocomotion = false;
        const struct { const char* name; float travel; } cases[] = {
            { "trota", 6.0f }, { "sprint", 21.5f }, { "costas", 10.0f }, { "diagonal", 10.0f }, { "zigue", 17.0f } };
        for (const auto& item : cases)
            for (const Scenario& scenario : scenarios()) {
                if (scenario.name != item.name) continue;
                const Metrics m = runScenario(commanded, scenario, physical, true, nullptr, 1);
                char what[160];
                std::snprintf(what, sizeof(what), "comandado sem capsula, %s: de pe, andou %.2f m (>= %.1f)", item.name,
                    m.travel, item.travel);
                expect(!m.fell && m.travel >= item.travel, what, m);
            }
    }
    // A paridade (R1, o portao da L0): a sonda e um mundo de treino iguais bit
    // a bit, os recomecos de um instante iguais e a cena inline igual a com
    // threads (--paridade roda tudo).
    {
        const bool same = runParity(context, 4, 2) == 0;
        std::printf("  %s paridade: sonda e mundo de treino bit a bit, recomecos e cena inline iguais\n",
            same ? "ok  " : "FALHOU");
        if (!same) ++failures;
    }
    std::printf(failures == 0 ? "atleta: tudo certo\n" : "atleta: %d falha(s)\n", failures);
    return failures == 0 ? 0 : 1;
}


} // namespace MatterEngine::Probe

int main(int argc, char** argv) {
    std::string only, dumpPath;
    int every = 3;
    bool startPhysical = false;
    bool activeBody = AthleteRuntimeSettings3D {}.activeBody;
    bool footsteps = false;
    bool balanceSteps = AthleteRuntimeSettings3D {}.activeStanceSettings.balanceSteps;
    bool checks = false;
    int parity = -1;
    int baseline = -1;
    float response = AthleteRuntimeSettings3D {}.capsuleResponsePercent;
    float reversal = AthleteRuntimeSettings3D {}.capsuleReversalPercent;
    float startGesture = AthleteRuntimeSettings3D {}.defaultStartGesturePercent;
    bool noRequests = false;
    bool bridge = AthleteRuntimeSettings3D {}.physicalBridge;
    // O modo Fisico total (sempre pelo fisico) e as ajudas dele (%).
    bool totalPhysical = false;
    float feetAssist = AthleteRuntimeSettings3D {}.physicalFeetAssistPercent;
    float boneGuide = AthleteRuntimeSettings3D {}.physicalBoneGuidePercent;
    bool legClearance = AthleteRuntimeSettings3D {}.legClearance.enabled;
    bool returnSmoothing = AthleteRuntimeSettings3D {}.returnSmoothing;
    bool modeTable = AthleteRuntimeSettings3D {}.modeTable;
    bool defaultSpeeds = AthleteRuntimeSettings3D {}.physicalUsesDefaultSpeeds;
    bool noCapsule = false;
    float standingAssist = AthleteRuntimeSettings3D {}.e5StandingAssistPercent;
    float walkingAssist = AthleteRuntimeSettings3D {}.e5WalkingAssistPercent;
    float getUpAssist = AthleteRuntimeSettings3D {}.getUpAssistPercent;
    int zeroProof = -1;
    bool postureCouple = AthleteRuntimeSettings3D {}.postureCoupleFollowsAssist;
    float turnMove = 0.0f;
    PlayerBody3D body;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto need = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--cenario") only = need();
        else if (arg == "--dump") dumpPath = need();
        else if (arg == "--cada") every = std::max(1, std::atoi(need()));
        else if (arg == "--fisico") startPhysical = true;
        else if (arg == "--ativo") activeBody = std::atoi(need()) != 0;
        else if (arg == "--pegadas") footsteps = std::atoi(need()) != 0;
        else if (arg == "--passos") balanceSteps = std::atoi(need()) != 0;
        else if (arg == "--resposta") response = static_cast<float>(std::atof(need()));
        else if (arg == "--troca-direcao") reversal = static_cast<float>(std::atof(need()));
        else if (arg == "--arrancada") startGesture = static_cast<float>(std::atof(need()));
        else if (arg == "--sem-pedido") noRequests = true;
        else if (arg == "--ponte") bridge = std::atoi(need()) != 0;
        else if (arg == "--teste") checks = true;
        else if (arg == "--paridade") parity = i + 1 < argc && argv[i + 1][0] != '-' ? std::atoi(argv[++i]) : 0;
        else if (arg == "--prova-zero") zeroProof = i + 1 < argc && argv[i + 1][0] != '-' ? std::atoi(argv[++i]) : 0;
        else if (arg == "--linha-de-base") baseline = i + 1 < argc && argv[i + 1][0] != '-' ? std::atoi(argv[++i]) : 0;
        else if (arg == "--total") totalPhysical = true;
        else if (arg == "--pes") feetAssist = static_cast<float>(std::atof(need()));
        else if (arg == "--guia") boneGuide = static_cast<float>(std::atof(need()));
        else if (arg == "--espaco") legClearance = std::atoi(need()) != 0;
        else if (arg == "--volta-suave") returnSmoothing = std::atoi(need()) != 0;
        else if (arg == "--tabela") modeTable = std::atoi(need()) != 0;
        else if (arg == "--velocidades-padrao") defaultSpeeds = std::atoi(need()) != 0;
        else if (arg == "--sem-capsula") noCapsule = true;
        else if (arg == "--ajuda-parado") standingAssist = static_cast<float>(std::atof(need()));
        else if (arg == "--ajuda-andando") walkingAssist = static_cast<float>(std::atof(need()));
        else if (arg == "--ajuda-levantar") getUpAssist = static_cast<float>(std::atof(need()));
        else if (arg == "--par-na-regua") postureCouple = true;
        else if (arg == "--par-fixo") postureCouple = false;
        else if (arg == "--virada-brusca") turnMove = static_cast<float>(std::atof(need()));
        else if (arg == "--corpo") {
            const auto parsed = parseBody3D(need());
            if (!parsed) {
                std::fprintf(stderr, "--corpo cm,kg (170 a 200 cm, 60 a 100 kg)\n");
                return 2;
            }
            body = *parsed;
        }
        else if (arg == "--ajuda") {
            // A regua unica do laboratorio (Ajuda auxiliar): as quatro juntas.
            const float all = static_cast<float>(std::atof(need()));
            standingAssist = walkingAssist = feetAssist = getUpAssist = all;
        }
        else if (arg == "--lista") {
            for (const Scenario& scenario : scenarios()) std::printf("%s\n", scenario.name.c_str());
            return 0;
        } else {
            std::fprintf(stderr, "argumento desconhecido: %s\n", arg.c_str());
            return 2;
        }
    }
    // O atleta do corpo pelo pacote (o do corpo de hoje e o de sempre: o mesmo objeto).
    const std::string assets = MATTERENGINE_ASSETS_DIR;
    const std::shared_ptr<const AthleteBundle3D> bundle = AthleteBundle3D::shared(assets, body);
    if (isBaseBody3D(body) && bundle != AthleteBundle3D::shared(assets)) {
        std::fprintf(stderr, "o pacote do corpo de hoje nao e o de sempre\n");
        return 1;
    }
    if (!isBaseBody3D(body)) {
        // (Os mesmos clipes do corpo de hoje: um papel sem clipe estragaria as medidas.)
        const CharacterMotorSettings3D& capsule = bundle->assets().avatarSettings;
        const bool sameClips = bundle->clipErrors() == AthleteBundle3D::shared(assets)->clipErrors();
        std::printf("corpo %.2f m %.1f kg (escala %.4f, perfil %.1f kg): capsula raio %.3f de pe %.3f agachado %.3f"
            " degrau %.3f pulo %.2f m/s | clipes: trote %.2f sprint %.2f m/s, %zu nao carregaram (%s)\n",
            body.heightMeters, body.massKg, bodyScale3D(body), bundle->profile().totalMassKg, capsule.radius,
            capsule.standingHeight, capsule.crouchedHeight, capsule.maximumStepHeight, capsule.jumpSpeed,
            capsule.walkSpeed, capsule.sprintSpeed, bundle->clipErrors().size(),
            sameClips ? "os mesmos do corpo de hoje" : "DIFERENTES dos do corpo de hoje");
    }
    AthleteRuntimeSettings3D settings;
    settings.activeBody = activeBody;
    settings.footstepLocomotion = footsteps;
    settings.activeStanceSettings.balanceSteps = balanceSteps;
    settings.capsuleResponsePercent = response;
    settings.capsuleReversalPercent = reversal;
    settings.defaultStartGesturePercent = startGesture;
    settings.physicalBridge = bridge;
    settings.physicalFeetAssistPercent = feetAssist;
    settings.physicalBoneGuidePercent = boneGuide;
    settings.legClearance.enabled = legClearance;
    settings.returnSmoothing = returnSmoothing;
    settings.modeTable = modeTable;
    settings.physicalUsesDefaultSpeeds = defaultSpeeds;
    settings.e5StandingAssistPercent = standingAssist;
    settings.e5WalkingAssistPercent = walkingAssist;
    settings.getUpAssistPercent = getUpAssist;
    settings.postureCoupleFollowsAssist = postureCouple;
    settings.dynamicTurnMove = turnMove > 0.0f;
    settings.dynamicTurnMoveSettings.amount = turnMove;
    // XTM="campo=valor,campo=valor": os ajustes da manobra (nomes do TurnMoveSettings3D).
    if (const char* overrides = std::getenv("XTM")) {
        TurnMoveSettings3D& tm = settings.dynamicTurnMoveSettings;
        const std::pair<const char*, float*> fields[] = { { "lateralLeanBase", &tm.lateralLeanBase },
            { "lateralLeanPerSpeed", &tm.lateralLeanPerSpeed }, { "lateralLeanMaximum", &tm.lateralLeanMaximum },
            { "brakeLean", &tm.brakeLean }, { "exitLean", &tm.exitLean }, { "comInsideBase", &tm.comInsideBase },
            { "comInsidePerSpeed", &tm.comInsidePerSpeed }, { "comInsideMaximum", &tm.comInsideMaximum },
            { "crouchBase", &tm.crouchBase }, { "crouchPerSpeed", &tm.crouchPerSpeed }, { "crouchMaximum", &tm.crouchMaximum },
            { "cadenceOctaves", &tm.cadenceOctaves }, { "strideOctaves", &tm.strideOctaves },
            { "outsideSideBase", &tm.outsideSideBase }, { "outsideSidePerSin", &tm.outsideSidePerSin },
            { "outsideShort", &tm.outsideShort }, { "cutSpeedFraction", &tm.cutSpeedFraction },
            { "cutSpeedFloor", &tm.cutSpeedFloor }, { "triggerDegrees", &tm.triggerDegrees },
            { "minRequestedSpeed", &tm.minRequestedSpeed } };
        std::string text(overrides);
        std::size_t begin = 0;
        while (begin < text.size()) {
            std::size_t end = text.find(',', begin);
            if (end == std::string::npos) end = text.size();
            const std::string item = text.substr(begin, end - begin);
            const std::size_t eq = item.find('=');
            if (eq != std::string::npos)
                for (const auto& [name, target] : fields)
                    if (item.compare(0, eq, name) == 0 && std::strlen(name) == eq) *target = std::strtof(item.c_str() + eq + 1, nullptr);
            begin = end + 1;
        }
    }
    if (totalPhysical) {
        settings.defaultLocomotion = false;
        startPhysical = true;
    }
    ProbeContext context;
    context.profile = &bundle->profile();
    context.gaitSpeeds = bundle->gaitSpeeds();
    context.noCapsule = noCapsule;
    context.avatar = bundle->assets().avatarSettings;
    context.runtimeAssets = bundle->assets();
    context.body = bundle->body();
    if (checks) return runChecks(context);
    if (parity >= 0) return runParity(context, parity, 0);
    if (baseline >= 0) return runBaseline(context, baseline);
    if (zeroProof >= 0) return runZeroProof(context, zeroProof);

    FILE* dump = dumpPath.empty() ? nullptr : std::fopen(dumpPath.c_str(), "w");
    std::printf("cenario            | deslize pe no chao med/p95/max m/s | pico acel tronco+bracos / pernas | salto pose | modos (trocas; animado/ativo/fisico/volta s; ajuda N.s) | bola\n");
    for (const Scenario& scenario : scenarios()) {
        if (!only.empty() && scenario.name != only && !(only.back() == '*' && scenario.name.rfind(only.substr(0, only.size() - 1), 0) == 0)) continue;
        printMetrics(scenario, runScenario(context, scenario, settings, startPhysical, dump, every, noRequests),
            settings.footstepLocomotion);
    }
    if (dump != nullptr) std::fclose(dump);
    return 0;
}
