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
//      XDESLOCA=m: a largada da capsula deslocada em x (a sensibilidade caotica)
//      XCUSTO=1: o custo do runtime por tick (p50/p95/p99, o pior tick)
//      [--paridade [N]] (a sonda contra os mundos de treino, bit a bit; N roteiros, 0 todos)
//      [--linha-de-base [threads]] (o classico por nivel de ajuda num lote de mundos, e a vazao)
//      [--ajuda-parado %] [--ajuda-andando %]
// Sem --cenario roda todos.
#include "athlete_probe/ProbeShared.hpp"

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Contract/LocomotionContract3D.hpp"
#include "Engine/Athlete/Training/LocomotionBatch3D.hpp"
#include "Engine/Athlete/Training/LocomotionWorld3D.hpp"
#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"
#include "Engine/Athlete/Dribble/ContactPose3D.hpp"
#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

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
        Scenario s { "toque_parado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.45f, -0.12f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "toque_lado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.15f, -0.45f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "toque_corre", 4.0f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.ball = true;
        s.ballStart = { 3.0f, -0.1f, 0.11f };
        s.autoDistance = 0.9f;
        list.push_back(s);
    }
    {
        // O corte correndo: no pedido as teclas viram 50 graus para a direita
        // (o toque sai para la, de lado de fora do pe direito) e seguem.
        Scenario s { "corte_corre", 4.0f, [](float t) {
            Keys k;
            k.forward = t > 0.3f ? 1.0f : 0.0f;
            return k; } };
        s.ball = true;
        s.ballStart = { 3.0f, -0.1f, 0.11f };
        s.autoDistance = 0.9f;
        s.autoTurn = -0.87f;
        list.push_back(s);
    }
    {
        Scenario s { "chute_parado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Shot; k.power = 0.8f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.55f, -0.15f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "chute_corre", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.ball = true;
        s.ballStart = { 3.2f, -0.12f, 0.11f };
        s.autoDistance = 1.0f;
        s.autoKind = BallActionKind3D::Shot;
        list.push_back(s);
    }
    {
        // (O chute carregado antes: o pedido com a bola a 1,8 m, a tempo de a
        // passada e o freio acertarem o apoio.)
        Scenario s { "chute_corre_cedo", 3.5f, [](float t) { Keys k; k.forward = t > 0.3f ? 1.0f : 0.0f; return k; } };
        s.ball = true;
        s.ballStart = { 3.6f, -0.12f, 0.11f };
        s.autoDistance = 1.8f;
        s.autoKind = BallActionKind3D::Shot;
        list.push_back(s);
    }
    // A bola parada a 0,5 m em oito angulos em volta (0 a frente, 90 a
    // esquerda), o toque pedido para a frente; a bola vindo de frente.
    for (int k = 0; k < 8; ++k) {
        static const char* names[8] = { "bola_000", "bola_045", "bola_090", "bola_135", "bola_180", "bola_225",
            "bola_270", "bola_315" };
        Scenario s { names[k], 2.2f, [](float t) {
            Keys keys;
            if (t >= 0.6f && t < 0.6f + Dt) { keys.ball = BallActionKind3D::Dribble; keys.power = 0.4f; }
            return keys; } };
        const float angle = static_cast<float>(k) * Pi * 0.25f;
        s.ball = true;
        s.ballStart = { 0.5f * std::cos(angle), 0.5f * std::sin(angle), 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "bola_vindo", 3.0f, [](float) { return Keys {}; } };
        s.ball = true;
        s.ballStart = { 3.5f, -0.15f, 0.11f };
        s.ballVelocity = { -3.0f, 0.0f, 0.0f };
        // (O pedido com a bola a 1,2 m da pelve: dentro dos prazos.)
        s.autoDistance = 1.2f;
        list.push_back(s);
    }
    {
        // A bola vindo pelo ar e chegando na altura da cintura (~0,6 m).
        Scenario s { "bola_alta", 2.5f, [](float) { return Keys {}; } };
        s.ball = true;
        s.ballStart = { 3.6f, -0.15f, 0.5f };
        s.ballVelocity = { -4.5f, 0.0f, 3.6f };
        s.autoDistance = 1.2f;
        list.push_back(s);
    }
    {
        // (Mais alta: chegando perto do quadril.)
        Scenario s { "bola_quadril", 2.5f, [](float) { return Keys {}; } };
        s.ball = true;
        s.ballStart = { 3.6f, -0.15f, 0.5f };
        s.ballVelocity = { -4.5f, 0.0f, 3.95f };
        s.autoDistance = 1.2f;
        list.push_back(s);
    }
    {
        // A bola atras e a direita, tocada para a direita (o lado de fora, a
        // perna para tras e para fora) e atras, tocada para tras (calcanhar).
        Scenario s { "atras_fora", 2.2f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; k.left = -1.0f; }
            return k; } };
        s.ball = true;
        s.ballStart = { -0.34f, -0.38f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "atras_tras", 2.2f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; k.forward = -1.0f; }
            return k; } };
        s.ball = true;
        s.ballStart = { -0.50f, -0.08f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "chute_lado", 2.5f, [](float t) {
            Keys k;
            if (t >= 0.6f && t < 0.6f + Dt) { k.ball = BallActionKind3D::Shot; k.power = 0.7f; }
            return k; } };
        s.ball = true;
        s.ballStart = { 0.35f, -0.35f, 0.11f };
        list.push_back(s);
    }
    {
        Scenario s { "caixa_leve", 3.0f, [](float) { return Keys {}; } };
        s.throws.push_back({ 1.0f, 6.0f, { 0.15f, 0.15f, 0.15f }, { 0.0f, 1.2f, 1.2f }, { 0.0f, -5.0f, 0.0f } });
        list.push_back(s);
    }
    {
        // O toque no Ativo: a caixa leve bate e o toque e pedido logo depois,
        // com o corpo ainda no Ativo.
        Scenario s { "toque_ativo", 3.0f, [](float t) {
            Keys k;
            if (t >= 1.30f && t < 1.30f + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.4f; }
            return k; } };
        s.throws.push_back({ 1.0f, 6.0f, { 0.15f, 0.15f, 0.15f }, { 0.0f, 1.2f, 1.2f }, { 0.0f, -5.0f, 0.0f } });
        s.ball = true;
        s.ballStart = { 0.45f, -0.12f, 0.11f };
        list.push_back(s);
    }
    // Conduzir (o teste do usuario, 11/10): andando, correndo, no sprint, de
    // lado e na diagonal, o jogador toca de novo (clique rapido: forca baixa)
    // sempre que a bola esta a ate 1,1 m, com 0,3 s entre os cliques.
    {
        struct Drive { const char* key; float forward; float left; bool sprint; Vec3 ball; };
        static const Drive drives[6] = { { "conduz_anda", 0.5f, 0.0f, false, { 0.55f, 0.0f, 0.11f } },
            { "conduz_corre", 1.0f, 0.0f, false, { 0.55f, 0.0f, 0.11f } },
            { "conduz_sprint", 1.0f, 0.0f, true, { 0.55f, 0.0f, 0.11f } },
            { "conduz_lado", 0.0f, 1.0f, false, { 0.0f, 0.55f, 0.11f } },
            { "conduz_lado_anda", 0.0f, 0.5f, false, { 0.0f, 0.55f, 0.11f } },
            { "conduz_diagonal", 0.71f, 0.71f, false, { 0.4f, 0.4f, 0.11f } } };
        // (Com troca de rumo: zigue-zague, o corte de 90 graus e parar.)
        {
            const auto add = [&list](const char* name, std::function<Keys(float)> keys) {
                Scenario s { name, 8.0f, std::move(keys) };
                s.ball = true;
                s.ballStart = { 0.55f, 0.0f, 0.11f };
                s.autoDistance = 1.1f;
                s.autoKind = BallActionKind3D::Dribble;
                s.autoCooldown = 0.3f;
                s.autoPower = 0.15f;
                list.push_back(s);
            };
            add("conduz_zigue", [](float t) {
                Keys k;
                if (t > 0.3f) {
                    k.forward = 0.8f;
                    k.left = std::fmod(t, 2.0f) < 1.0f ? 0.6f : -0.6f;
                }
                return k; });
            add("conduz_corte", [](float t) {
                Keys k;
                if (t > 0.3f) {
                    k.forward = t < 3.0f ? 1.0f : 0.0f;
                    k.left = t < 3.0f ? 0.0f : 1.0f;
                }
                return k; });
            add("conduz_inverte", [](float t) {
                Keys k;
                k.forward = t > 0.3f ? (t < 3.0f ? 1.0f : -1.0f) : 0.0f;
                return k; });
            add("conduz_para", [](float t) {
                Keys k;
                k.forward = t > 0.3f && t < 3.5f ? 1.0f : t >= 5.0f ? 1.0f : 0.0f;
                return k; });
        }
        for (const Drive& d : drives) {
            const Drive drive = d;
            Scenario s { drive.key, 8.0f, [drive](float t) {
                Keys k;
                if (t > 0.3f) {
                    k.forward = drive.forward;
                    k.left = drive.left;
                    k.sprint = drive.sprint;
                }
                return k; } };
            s.ball = true;
            s.ballStart = drive.ball;
            s.autoDistance = 1.1f;
            s.autoKind = BallActionKind3D::Dribble;
            s.autoCooldown = 0.3f;
            s.autoPower = 0.15f;
            list.push_back(s);
        }
    }
    // Plano v5 (E0): o manejo como o usuario joga (11/10 noite: "manuseios
    // consecutivos rapidos com mudanca de direcao", "manejando rapidamente a
    // bola para a direita", intercalando). As teclas dao o movimento e o rumo
    // do toque (como no laboratorio); o clique (forca baixa, o clique rapido)
    // sai sempre que a bola esta ao alcance e o intervalo passou.
    {
        const auto add = [&list](const char* name, float seconds, float cooldown, float distance, Vec3 ball,
                             std::function<Keys(float)> keys) {
            Scenario s { name, seconds, std::move(keys) };
            s.ball = true;
            s.ballStart = ball;
            s.autoDistance = distance;
            s.autoKind = BallActionKind3D::Dribble;
            s.autoCooldown = cooldown;
            s.autoPower = 0.15f;
            list.push_back(s);
        };
        // O rumo das teclas (rad, 0 frente, + esquerda) e o tamanho.
        const auto heading = [](float angle, float amount) {
            Keys k;
            k.forward = std::cos(angle) * amount;
            k.left = std::sin(angle) * amount;
            return k;
        };
        constexpr float Quarter = 1.5707963f;
        // Parado (quase): o rumo trocando a cada 0,18 s - direita, esquerda,
        // frente, tras.
        add("rajada_parado", 4.0f, 0.16f, 1.0f, { 0.30f, 0.0f, 0.11f }, [heading](float t) {
            static const float angles[4] = { -Quarter, Quarter, 0.0f, 2.0f * Quarter };
            if (t < 0.3f) return Keys {};
            return heading(angles[static_cast<int>((t - 0.3f) / 0.18f) % 4], 0.3f); });
        add("rajada_anda", 5.0f, 0.18f, 1.1f, { 0.40f, 0.0f, 0.11f }, [heading](float t) {
            if (t < 0.3f) return Keys {};
            return heading((static_cast<int>((t - 0.3f) / 0.2f) % 2 == 0 ? 1.0f : -1.0f) * 1.05f, 0.4f); });
        add("rajada_corre", 5.0f, 0.2f, 1.2f, { 0.55f, 0.0f, 0.11f }, [heading](float t) {
            if (t < 0.3f) return Keys {};
            return heading((static_cast<int>((t - 0.3f) / 0.22f) % 2 == 0 ? 1.0f : -1.0f) * 0.785f, 1.0f); });
        // Correndo, o rumo das teclas girando 90 graus a cada 0,4 s.
        add("rajada_vira_corre", 5.0f, 0.2f, 1.3f, { 0.55f, 0.0f, 0.11f }, [heading](float t) {
            if (t < 0.3f) return Keys {};
            return heading(static_cast<float>(static_cast<int>((t - 0.3f) / 0.4f) % 4) * Quarter, 1.0f); });
        // Manejando para a direita, sem parar.
        add("continuo_direita_parado", 4.0f, 0.25f, 1.0f, { 0.10f, -0.30f, 0.11f }, [heading](float t) {
            return t < 0.3f ? Keys {} : heading(-Quarter, 0.25f); });
        add("continuo_direita_anda", 5.0f, 0.25f, 1.1f, { 0.10f, -0.35f, 0.11f }, [heading](float t) {
            return t < 0.3f ? Keys {} : heading(-Quarter, 0.5f); });
        add("continuo_direita_corre", 5.0f, 0.22f, 1.2f, { 0.10f, -0.45f, 0.11f }, [heading](float t) {
            return t < 0.3f ? Keys {} : heading(-Quarter, 1.0f); });
        // Intercalando: frente, frente-direita, frente, frente-esquerda...
        add("intercala_corre", 5.0f, 0.22f, 1.2f, { 0.55f, 0.0f, 0.11f }, [heading](float t) {
            static const float angles[4] = { 0.0f, -0.785f, 0.0f, 0.785f };
            if (t < 0.3f) return Keys {};
            return heading(angles[static_cast<int>((t - 0.3f) / 0.25f) % 4], 1.0f); });
        // Parado: frente, tras (a sola), direita, esquerda.
        add("intercala_parado", 4.0f, 0.28f, 1.0f, { 0.30f, 0.0f, 0.11f }, [heading](float t) {
            static const float angles[4] = { 0.0f, 2.0f * Quarter, -Quarter, Quarter };
            if (t < 0.3f) return Keys {};
            return heading(angles[static_cast<int>((t - 0.3f) / 0.3f) % 4], 0.25f); });
        // Com a camera (o olhar) girando, como o mouse no laboratorio: o rumo do
        // corpo vira junto e as teclas sao relativas a ela.
        add("rajada_camera_corre", 5.0f, 0.2f, 1.2f, { 0.55f, 0.0f, 0.11f }, [heading](float t) {
            if (t < 0.3f) return Keys {};
            Keys k = heading((static_cast<int>((t - 0.3f) / 0.22f) % 2 == 0 ? 1.0f : -1.0f) * 0.785f, 1.0f);
            k.cameraYaw = 1.2f * std::sin(5.2f * (t - 0.3f));
            return k; });
        add("rajada_camera_parado", 4.0f, 0.16f, 1.0f, { 0.30f, 0.0f, 0.11f }, [heading](float t) {
            static const float angles[4] = { -Quarter, Quarter, 0.0f, 2.0f * Quarter };
            if (t < 0.3f) return Keys {};
            Keys k = heading(angles[static_cast<int>((t - 0.3f) / 0.18f) % 4], 0.3f);
            k.cameraYaw = 3.0f * (t - 0.3f);
            return k; });
        add("rajada_camera_anda", 5.0f, 0.18f, 1.1f, { 0.40f, 0.0f, 0.11f }, [heading](float t) {
            if (t < 0.3f) return Keys {};
            Keys k = heading((static_cast<int>((t - 0.3f) / 0.2f) % 2 == 0 ? 1.0f : -1.0f) * 1.05f, 0.4f);
            k.cameraYaw = 0.9f * std::sin(7.0f * (t - 0.3f));
            return k; });
        // Parado, sai correndo, para - tocando o tempo todo.
        add("para_toca_sai", 5.0f, 0.3f, 1.1f, { 0.35f, 0.0f, 0.11f }, [](float t) {
            Keys k;
            k.forward = t >= 1.3f && t < 3.5f ? 1.0f : 0.0f;
            return k; });
    }
    // Manejo v2 (plano v2, fase C): os casos do teste do usuario. A bola fora
    // do lugar (a 0,35, 0,65 e 0,9 m da pelve, em oito angulos) parado,
    // andando e correndo, o toque pedido no instante em que ela e posta; o
    // chute nos oito angulos parado e correndo; o pedido em varias fases da
    // passada; e o movimento mudando logo depois do pedido.
    {
        struct Gait { const char* key; float forward; float placeAt; };
        static const Gait gaits[3] = { { "parado", 0.0f, 0.6f }, { "anda", 0.33f, 1.6f }, { "corre", 1.0f, 1.6f } };
        const auto request = [](float placeAt, float forward, BallActionKind3D kind, float power) {
            return [=](float t) {
                Keys k;
                k.forward = t > 0.3f ? forward : 0.0f;
                if (t >= placeAt && t < placeAt + Dt) { k.ball = kind; k.power = power; }
                return k;
            };
        };
        for (const Gait& gait : gaits) {
            for (const int centimeters : { 35, 65, 90 }) {
                for (int k = 0; k < 8; ++k) {
                    char name[64];
                    std::snprintf(name, sizeof(name), "v2_%s_%02d_%03d", gait.key, centimeters, k * 45);
                    Scenario s { name, gait.placeAt + 1.2f, request(gait.placeAt, gait.forward, BallActionKind3D::Dribble, 0.45f) };
                    const float angle = static_cast<float>(k) * Pi * 0.25f;
                    const float d = 0.01f * static_cast<float>(centimeters);
                    s.ball = true;
                    s.ballStart = { 0.0f, 8.0f, 0.11f };
                    s.placeAt = gait.placeAt;
                    s.placeOffset = { d * std::cos(angle), d * std::sin(angle), 0.11f };
                    list.push_back(s);
                }
            }
            if (std::string(gait.key) == "anda") continue;
            for (int k = 0; k < 8; ++k) {
                char name[64];
                std::snprintf(name, sizeof(name), "v2_chute_%s_%03d", gait.key, k * 45);
                Scenario s { name, gait.placeAt + 1.4f, request(gait.placeAt, gait.forward, BallActionKind3D::Shot, 0.8f) };
                const float angle = static_cast<float>(k) * Pi * 0.25f;
                s.ball = true;
                s.ballStart = { 0.0f, 8.0f, 0.11f };
                s.placeAt = gait.placeAt;
                s.placeOffset = { 0.5f * std::cos(angle), 0.5f * std::sin(angle), 0.11f };
                list.push_back(s);
            }
        }
        // O pedido em oito fases da passada (correndo, a bola a frente e a
        // direita).
        for (int k = 0; k < 8; ++k) {
            char name[64];
            std::snprintf(name, sizeof(name), "v2_fase_%d", k);
            const float at = 1.5f + 0.05f * static_cast<float>(k);
            Scenario s { name, at + 1.2f, request(at, 1.0f, BallActionKind3D::Dribble, 0.45f) };
            s.ball = true;
            s.ballStart = { 0.0f, 8.0f, 0.11f };
            s.placeAt = at;
            s.placeOffset = { 0.6f, -0.12f, 0.11f };
            list.push_back(s);
        }
        // O movimento muda 0,05 s depois do pedido: arranca (de parado),
        // para, vira 90 graus e inverte (correndo).
        struct Change { const char* key; float placeAt; float before; float after; float leftAfter; };
        static const Change changes[5] = { { "arranca", 0.6f, 0.0f, 1.0f, 0.0f }, { "para", 1.6f, 1.0f, 0.0f, 0.0f },
            { "vira", 1.6f, 1.0f, 0.0f, 1.0f }, { "inverte", 1.6f, 1.0f, -1.0f, 0.0f }, { "anda_para", 1.6f, 0.33f, 0.0f, 0.0f } };
        for (const Change& change : changes) {
            char name[64];
            std::snprintf(name, sizeof(name), "v2_muda_%s", change.key);
            const Change c = change;
            Scenario s { name, c.placeAt + 1.4f, [c](float t) {
                Keys k;
                const bool after = t >= c.placeAt + 0.05f;
                k.forward = t > 0.3f ? (after ? c.after : c.before) : 0.0f;
                k.left = after ? c.leftAfter : 0.0f;
                if (t >= c.placeAt && t < c.placeAt + Dt) { k.ball = BallActionKind3D::Dribble; k.power = 0.45f; }
                return k; } };
            s.ball = true;
            s.ballStart = { 0.0f, 8.0f, 0.11f };
            s.placeAt = c.placeAt;
            s.placeOffset = { 0.55f, -0.14f, 0.11f };
            s.changeAt = c.placeAt + 0.05f;
            list.push_back(s);
        }
    }
    // Os movimentos padronizados (plano v3, etapa 3): andando em cada direcao
    // (parado, e as 8 andando e correndo), as teclas viram para a direcao do
    // toque no clique (o toque vai para onde as teclas apontam) e seguem; a
    // bola posta a 0,4 m nessa direcao.
    {
        struct Gait { const char* key; float speed; float placeAt; };
        static const Gait gaits[3] = { { "parado", 0.0f, 0.6f }, { "anda", 0.5f, 1.6f }, { "corre", 1.0f, 1.6f } };
        for (const Gait& gait : gaits) {
            for (int m = 0; m < (gait.speed > 0.0f ? 8 : 1); ++m) {
                for (int k = 0; k < 8; ++k) {
                    char name[64];
                    if (gait.speed > 0.0f) std::snprintf(name, sizeof(name), "mov_%s_%03d_%03d", gait.key, m * 45, k * 45);
                    else std::snprintf(name, sizeof(name), "mov_parado_%03d", k * 45);
                    const float moveAngle = static_cast<float>(m) * Pi * 0.25f;
                    const float touchAngle = static_cast<float>(k) * Pi * 0.25f;
                    const Gait g = gait;
                    Scenario sc { name, g.placeAt + 1.0f, [g, moveAngle, touchAngle](float t) {
                        Keys keys;
                        const bool after = t >= g.placeAt;
                        const float angle = after ? touchAngle : moveAngle;
                        const float amount = after ? std::max(0.5f, g.speed) : (t > 0.3f ? g.speed : 0.0f);
                        keys.forward = std::cos(angle) * amount;
                        keys.left = std::sin(angle) * amount;
                        if (t >= g.placeAt && t < g.placeAt + Dt) { keys.ball = BallActionKind3D::Dribble; keys.power = 0.3f; }
                        return keys; } };
                    sc.ball = true;
                    sc.ballStart = { 0.0f, 8.0f, 0.11f };
                    sc.placeAt = g.placeAt;
                    sc.placeOffset = { 0.4f * std::cos(touchAngle), 0.4f * std::sin(touchAngle), 0.11f };
                    list.push_back(sc);
                }
            }
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
    // XCUSTO=1: o custo do runtime por tick (o passo do atleta; us, p50/p95/p99).
    std::vector<double> runtimeCost;

    AthleteRuntime3D runtime;
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
    for (int step = -warmTicks; step < ticks; ++step) {
        const float t = static_cast<float>(step) * Dt;
        Keys keys = step < 0 ? Keys {} : scenario.keys(t);
        if (suppressRequests) keys.ball = BallActionKind3D::None;
        // (Manejo v2: a bola posta em relacao a pelve, andando com o corpo.)
        if (step >= 0 && scenario.ball && scenario.placeAt >= 0.0f && t >= scenario.placeAt && t < scenario.placeAt + Dt
            && !bodyState.links.empty()) {
            const Vec3 pelvis = bodyState.links.front().position;
            const Vec3 forward { std::cos(keys.cameraYaw), std::sin(keys.cameraYaw), 0.0f };
            const Vec3 left { -forward.y, forward.x, 0.0f };
            const Vec3 position = Vec3 { pelvis.x, pelvis.y, scenario.placeOffset.z } + forward * scenario.placeOffset.x
                + left * scenario.placeOffset.y;
            const Vec3 velocity { scene->characterState().velocity.x, scene->characterState().velocity.y, 0.0f };
            const float radius = scene->inflatedBallProfile(ballHandle).radiusMeters;
            scene->setInflatedBallMotion(ballHandle, position, velocity, cross(Vec3 { 0.0f, 0.0f, 1.0f }, velocity) * (1.0f / radius));
            ballState = scene->inflatedBallState(ballHandle);
        }
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
            const float wanted = controlledAnimated || settings.physicalUsesDefaultSpeeds
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
            const AthleteCapsuleResponse3D response = athleteCapsuleResponse3D(settings.capsuleResponsePercent);
            active.groundAcceleration = response.acceleration;
            active.groundDeceleration = response.deceleration;
            active.groundTurnRate = response.turnRate;
            active.groundBrakeDeceleration = response.brake;
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
        if (step >= 0 && scenario.autoDistance > 0.0f && scenario.ball && autoCooldown <= 0.0f && !bodyState.links.empty()) {
            const Vec3 pelvis = bodyState.links.front().position;
            const Vec3 offset = ballState.position - pelvis;
            const float reach = autoTurnLeft > 0.0f ? std::max(scenario.autoDistance, 1.6f) : scenario.autoDistance;
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
        if (settings.dribbleV5) {
            metrics.abortedTouches += athlete.dribbleOutput.aborted;
            metrics.missedTouches += athlete.dribbleOutput.missed;
            metrics.mergedRequests += athlete.dribbleOutput.merged;
            // (O clique juntado - o pendente mais novo que virou o clique novo -
            // sai da fila do atraso: sem isso o toque pareava com ele.)
            for (int k = 0; k < athlete.dribbleOutput.merged && metrics.pendingRequests.size() >= 2; ++k)
                metrics.pendingRequests.erase(metrics.pendingRequests.end() - 2);
        }
        if (control.ballRequest != BallActionKind3D::None) {
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
        runtime.update(*scene, runtimeAssets, std::span<AthleteSlot3D>(&slot, 1), balls, step < 0 ? warmSettings : settings,
            control, Dt, output);
        if (step >= 0)
            runtimeCost.push_back(std::chrono::duration<double>(std::chrono::steady_clock::now() - runtimeBegin).count());
        if (output.followVelocitySet) followVelocity = output.followVelocityWorld;
        if (output.controlledAnimatedSet) controlledAnimated = output.controlledAnimated;
        capsuleSpeedScale = output.controlledSpeedScale;
        if (legacy.ballLastOutput.touched) {
            if (std::getenv("XCONDUZ") && settings.newBallHandling)
                std::printf("  t %.3f TOQUE movimento %d setor %d lado %d superficie %d | resultado %.2f %.2f %.2f | pedido %.2f %.2f | corpo %.2f %.2f | bola antes %.2f %.2f\n", t,
                    static_cast<int>(athlete.ballPlay.move().choice().touch), athlete.ballPlay.move().choice().sector,
                    athlete.ballPlay.move().side(), static_cast<int>(athlete.ballPlay.move().choice().surface),
                    athlete.ballOutput.outcomeVelocity.x, athlete.ballOutput.outcomeVelocity.y,
                    athlete.ballOutput.outcomeVelocity.z, control.intent.requestedVelocityWorld.x,
                    control.intent.requestedVelocityWorld.y, scene->characterState().velocity.x,
                    scene->characterState().velocity.y, ballState.linearVelocity.x, ballState.linearVelocity.y);
            ++metrics.touches;
            if (dump != nullptr) std::fprintf(dump, "T\n");
            if (std::getenv("XV5") && settings.dribbleV5)
                std::printf("  t %.3f TOQUE v5 perna %d superficie %d garantia %d pendentes %d | bola %.2f %.2f %.2f a %.2f m da pelve\n", t,
                    athlete.dribbleOutput.touchSide, static_cast<int>(athlete.dribbleOutput.touchSurface),
                    athlete.dribbleOutput.touchGuaranteed ? 1 : 0, athlete.dribbleOutput.pending, ballState.position.x,
                    ballState.position.y, ballState.position.z,
                    std::hypot(ballState.position.x - bodyState.links.front().position.x, ballState.position.y - bodyState.links.front().position.y));
            // (Plano v5: o atraso do pedido mais antigo e a folga pe-bola.)
            // (Os pedidos de mais de 1,2 s ja venceram: sem eles o atraso
            // pareava o toque com um clique perdido.)
            while (!metrics.pendingRequests.empty() && t - metrics.pendingRequests.front() > 1.2f) {
                metrics.pendingRequests.erase(metrics.pendingRequests.begin());
                ++metrics.lostRequests;
            }
            if (!metrics.pendingRequests.empty()) {
                metrics.latencies.push_back(t - metrics.pendingRequests.front());
                metrics.pendingRequests.erase(metrics.pendingRequests.begin());
            }
            {
                const AthleteLegs3D legs = resolveAthleteLegs3D(profile);
                const float radius = scene->inflatedBallProfile(ballHandle).radiusMeters;
                float gap = 10.0f;
                for (std::size_t side = 0; side < 2 && legs.valid; ++side) {
                    const std::size_t link = legs.foot[side];
                    if (link >= bodyState.links.size()) continue;
                    const RagdollCapsuleDefinition3D& collider = profile.links[link].collider;
                    if (!ragdollColliderHasBox3D(collider)) continue;
                    const Quaternion q = (bodyState.links[link].orientation * collider.localOrientation).normalized();
                    const Vec3 center = bodyState.links[link].position
                        + bodyState.links[link].orientation.rotate(collider.localPosition);
                    const Vec3 local = q.conjugate().rotate(ballState.position - center);
                    // (A distancia com sinal ao pe de verdade: a casca da chuteira.)
                    const float signedGap = ragdollColliderClosestPoint3D(collider, local).distance - radius;
                    gap = std::min(gap, signedGap);
                    if (std::getenv("XV5") && settings.dribbleV5)
                        std::printf("  t %.3f pe %c folga com sinal %.3f m\n", t, side == 0 ? 'E' : 'D', signedGap);
                }
                (settings.dribbleV5 && athlete.dribbleOutput.touchGuaranteed ? metrics.guaranteedGaps : metrics.contactGaps)
                    .push_back(std::max(0.0f, gap));
                if (!athlete.dribbleOutput.touchGuaranteed) metrics.contactInside = std::max(metrics.contactInside, -gap);
                if (std::getenv("XV5") && settings.dribbleV5)
                    std::printf("  t %.3f folga do toque %.3f m | bola contra a prevista %.3f m\n", t, std::max(0.0f, gap),
                        athlete.dribbleOutput.touchMiss);
                if (std::getenv("XV5") && settings.dribbleV5 && legs.valid && athlete.dribbleOutput.touchSide >= 0) {
                    const std::size_t side = static_cast<std::size_t>(athlete.dribbleOutput.touchSide);
                    const PhysicsBodyState3D& foot = bodyState.links[legs.foot[side]];
                    const Vec3 ankle = foot.position + foot.orientation.rotate(legs.ankleInFoot[side]);
                    const Vec3 forward = (foot.orientation * footGeometry.flat[side].conjugate()).rotate({ 1.0f, 0.0f, 0.0f });
                    std::printf("  t %.3f pe real tornozelo %.3f %.3f %.3f | rumo %.1f graus inclinacao %.1f\n", t, ankle.x, ankle.y,
                        ankle.z, athleteFootYaw3D(footGeometry, static_cast<int>(side), foot.orientation) * 57.2958f,
                        std::asin(std::clamp(forward.z, -1.0f, 1.0f)) * 57.2958f);
                }
            }
            if (metrics.forceTime < 0.0f && settings.newBallHandling) {
                metrics.forceTime = t;
                metrics.moveSide = athlete.ballPlay.move().side();
                metrics.moveTouch = static_cast<int>(athlete.ballPlay.move().choice().touch);
                metrics.moveSector = athlete.ballPlay.move().choice().sector;
            }
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
        if (std::getenv("XV2") && settings.newBallHandling && (athlete.ballOutput.body.active || athlete.ballPlay.move().active()))
            std::printf("  t %.3f v3 movimento %d fase %d t %.3f pouso em %.3f volta %.2f perna %.2f peso %.2f lado %d agindo %d solta %d %.2f passada %.2f interrompe %d pedido %.2f %.2f\n",
                t, athlete.ballPlay.move().active() ? 1 : 0, static_cast<int>(athlete.ballPlay.move().phase()),
                athlete.ballPlay.move().time(), athlete.ballPlay.move().timeToLanding(),
                athlete.ballPlay.move().rejoinProgress(), athlete.ballOutput.body.legWeight,
                athlete.ballOutput.body.weight, athlete.ballOutput.body.side, athlete.ballOutput.actingSide,
                athlete.ballOutput.body.releaseSide, athlete.ballOutput.body.releaseWeight, athlete.ballOutput.gaitRateScale,
                athlete.ballPlay.interrupting() ? 1 : 0, control.intent.requestedVelocityWorld.x,
                control.intent.requestedVelocityWorld.y);
        // (A perna sai do gesto depois da mudanca de movimento - contada da
        // mudanca ou da forca, o que vier depois.)
        if (scenario.changeAt >= 0.0f && metrics.legReturn < 0.0f && metrics.forceTime >= 0.0f) {
            const float from = std::max(scenario.changeAt, metrics.forceTime);
            const BallBodyTargets3D& body = athlete.ballOutput.body;
            if (t >= from && (!body.active || body.legWeight <= 0.01f)) metrics.legReturn = t - from;
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
        if (trace != nullptr && step >= 0) trace->push_back(athleteTickDigest3D(athlete, bodyState));
        if (step >= 0 && context.noCapsule) contractAfter(hooks, athlete, bodyState, slotCommand);
        if (scenario.ball && step >= 0) ballState = scene->inflatedBallState(ballHandle);
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
            const BallBodyTargets3D& bt = athlete.ballOutput.body;
            std::printf(" w %.3f desl %.3f %.3f queda %.3f giro %.3f perna %.2f lado %d olha %.2f %.2f %.2f (%.2f) bracos %.2f",
                bt.weight, bt.pelvisShift.x, bt.pelvisShift.y, bt.pelvisDrop, bt.pelvisYaw, bt.legWeight, bt.side,
                bt.look.x, bt.look.y, bt.look.z, bt.lookWeight, bt.armWeight[0]);
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
            std::printf("  t %.3f marcha %d mov %.2f | E %c sai %.3f pousa %.3f balanco %.3f | D %c sai %.3f pousa %.3f balanco %.3f | pedido %d\n",
                t, o.cycling ? 1 : 0, o.movingShare, o.cyclePlanted[0] ? 'A' : 'B', o.secondsToLiftoff[0],
                o.cycleSecondsToTouchdown[0], o.swingSeconds[0], o.cyclePlanted[1] ? 'A' : 'B', o.secondsToLiftoff[1],
                o.cycleSecondsToTouchdown[1], o.swingSeconds[1], athlete.ballPlay.queue().active() != nullptr ? 1 : 0);
            std::printf("      estilo D %.3f %.3f %.3f | alvo D %.3f %.3f %.3f\n", o.styleFootPosition[1].x,
                o.styleFootPosition[1].y, o.styleFootPosition[1].z, athlete.footTargets[1].position.x,
                athlete.footTargets[1].position.y, athlete.footTargets[1].position.z);
        }
        if (std::getenv("XCLIPE") && athlete.ballPlay.move().active()) {
            const BallMove3D& g = athlete.ballPlay.move();
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            const std::size_t side = static_cast<std::size_t>(g.side());
            const Vec3 clip = o.locomotionFootPosition[side];
            const Vec3 v = scene->characterState().velocity;
            const Vec3 dir = std::hypot(v.x, v.y) > 0.1f ? Vec3 { v.x, v.y, 0.0f }.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
            const Vec3 apart = clip - g.foot();
            std::printf("  t %.3f fase %d | pe do gesto %.3f %.3f %.3f | pe do clipe %.3f %.3f %.3f %s | clipe a frente %+.3f\n", t,
                static_cast<int>(g.phase()), g.foot().x, g.foot().y, g.foot().z, clip.x, clip.y, clip.z,
                o.cyclePlanted[side] ? "apoiado" : "no ar", apart.x * dir.x + apart.y * dir.y);
        }
        if (std::getenv("XGESTO") && athlete.ballPlay.move().active()) {
            const BallMove3D& g = athlete.ballPlay.move();
            const Vec3 p = g.ankle();
            const BallBodyTargets3D& bt = athlete.ballOutput.body;
            const Vec3 toe = g.orientation().rotate({ 1.0f, 0.0f, 0.0f });
            std::printf("  t %.3f movimento fase %d t %.3f tornozelo %.3f %.3f %.3f v %.2f | joelho %.2f %.2f %.2f ponta %.2f %.2f %.2f\n",
                t, static_cast<int>(g.phase()), g.time(), p.x, p.y, p.z, g.velocity().length(), bt.knee.x, bt.knee.y,
                bt.knee.z, toe.x, toe.y, toe.z);
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
        // (O movimento anterior voltando na outra perna.)
        if (std::getenv("XPREV") && settings.newBallHandling && thighs[0] < bodyState.links.size()
            && (athlete.ballPlay.previous().active() || athlete.ballOutput.body.releaseSide >= 0)) {
            const AthleteLegs3D legs = resolveAthleteLegs3D(profile);
            const int r = athlete.ballOutput.body.releaseSide;
            if (r >= 0) {
                const std::size_t side = static_cast<std::size_t>(r);
                const PhysicsBodyState3D& pelvisLink = bodyState.links.front();
                const PhysicsBodyState3D& footLink = bodyState.links[legs.foot[side]];
                const Vec3 hip = pelvisLink.position + pelvisLink.orientation.rotate(legs.hipInPelvis[side]);
                const Vec3 ankle = footLink.position + footLink.orientation.rotate(legs.ankleInFoot[side]);
                const Vec3 target = athlete.ballOutput.body.releaseAnkle;
                const float length = legs.thighLength[side] + legs.shinLength[side];
                const DefaultLocomotionOutput3D& styleNow = legacy.defaultLocomotion.output();
                const Vec3 clipNow = styleNow.locomotionFootPosition[side]
                    + styleNow.locomotionFootOrientation[side].rotate(legs.ankleInFoot[side]);
                std::printf("  t %.3f anterior %d fase %d peso %.2f | alvo %.3f %.3f %.3f | corpo %.3f %.3f %.3f alcance %.2f | clipe %.3f %.3f %.3f | joelho %.2f %.2f %.2f\n",
                    t, athlete.ballPlay.previous().active() ? 1 : 0, static_cast<int>(athlete.ballPlay.previous().phase()),
                    athlete.ballOutput.body.releaseWeight, target.x, target.y, target.z, ankle.x, ankle.y, ankle.z,
                    (ankle - hip).length() / length, clipNow.x, clipNow.y, clipNow.z, athlete.ballOutput.body.releaseKnee.x,
                    athlete.ballOutput.body.releaseKnee.y, athlete.ballOutput.body.releaseKnee.z);
            }
        }
        // O movimento com a bola: a perna esticada e o pe arrastando.
        if (settings.newBallHandling && athlete.ballPlay.move().active() && thighs[0] < bodyState.links.size()
            && leftFoot < bodyState.links.size() && footGeometry.valid) {
            const int kick = athlete.ballPlay.move().side();
            const std::size_t side = static_cast<std::size_t>(kick);
            const AthleteLegs3D legs = resolveAthleteLegs3D(profile);
            const PhysicsBodyState3D& pelvisLink = bodyState.links.front();
            const PhysicsBodyState3D& footLink = bodyState.links[kick == 0 ? leftFoot : rightFoot];
            const Vec3 hip = pelvisLink.position + pelvisLink.orientation.rotate(legs.hipInPelvis[side]);
            const Vec3 ankle = footLink.position + footLink.orientation.rotate(legs.ankleInFoot[side]);
            const float length = std::max(0.1f, legs.thighLength[side] + legs.shinLength[side]);
            // (So no gesto: pousar com a perna quase estendida e o normal, e na
            // volta a perna anda com a do clipe; apoiado esperando, a perna
            // nunca alem de estendida.)
            const float reachNow = (ankle - hip).length() / length;
            // (Depois do comeco: a perna do clipe ja vinha esticada atras.)
            if (athlete.ballPlay.move().gesturing() && athlete.ballPlay.move().time() > 0.06f)
                metrics.moveReach = std::max(metrics.moveReach, reachNow);
            {
                // (Por fase: o alcance maximo e os ticks com a perna quase
                // reta - a "perna esticada" do usuario, 11/10 -, e com ela mais
                // reta que a do clipe no mesmo quadril.)
                const std::size_t phase = static_cast<std::size_t>(athlete.ballPlay.move().phase());
                const DefaultLocomotionOutput3D& style = legacy.defaultLocomotion.output();
                const Vec3 clipAnkle = style.locomotionFootPosition[side]
                    + style.locomotionFootOrientation[side].rotate(legs.ankleInFoot[side]);
                const float clipReach = (clipAnkle - hip).length() / length;
                if (phase < metrics.phaseReach.size()) {
                    metrics.phaseReach[phase] = std::max(metrics.phaseReach[phase], reachNow);
                    if (reachNow > 0.95f) ++metrics.phaseStraightTicks[phase];
                    if (reachNow > 0.95f && reachNow > clipReach + 0.02f) ++metrics.phaseBeyondClipTicks[phase];
                    const Vec3 travel { pelvisLink.linearVelocity.x, pelvisLink.linearVelocity.y, 0.0f };
                    if (travel.length() > 0.8f) {
                        const float back = -dot(Vec3 { footLink.linearVelocity.x, footLink.linearVelocity.y, 0.0f },
                            travel * (1.0f / travel.length()));
                        metrics.phaseBackSpeed[phase] = std::max(metrics.phaseBackSpeed[phase], back);
                    }
                }
            }
            if (std::getenv("XMOV")) {
                static bool lengthShown = false;
                if (!lengthShown) {
                    lengthShown = true;
                    std::printf("  perna: coxa %.3f canela %.3f (soma %.3f), quadril no chao %.3f\n", legs.thighLength[side],
                        legs.shinLength[side], length, hip.z);
                }
                const Vec3 target = athlete.ballPlay.move().ankle();
                const Vec3 targetHip = legacy.animationPose.linkPositions.empty() ? hip
                    : legacy.animationPose.linkPositions.front()
                        + legacy.animationPose.linkOrientations.front().rotate(legs.hipInPelvis[side]);
                const Vec3 heelNow = footLink.position + footLink.orientation.rotate(footGeometry.heel[side]);
                const Vec3 toeNow = footLink.position + footLink.orientation.rotate(footGeometry.toeBase[side]);
                const float yaw = legacy.defaultLocomotion.output().facingYawRadians;
                const Vec3 rel = target - pelvisLink.position;
                const DefaultLocomotionOutput3D& styleNow = legacy.defaultLocomotion.output();
                const Vec3 clipNow = styleNow.locomotionFootPosition[side]
                    + styleNow.locomotionFootOrientation[side].rotate(legs.ankleInFoot[side]);
                std::printf("  t %.3f fase %d | local frente %+.3f lado %+.3f | alvo %.3f %.3f %.3f alcance %.2f | corpo %.3f %.3f %.3f alcance %.2f | sola %.3f v %.2f | clipe %.3f %.3f %.3f alcance %.2f %s\n",
                    t, static_cast<int>(athlete.ballPlay.move().phase()), rel.x * std::cos(yaw) + rel.y * std::sin(yaw),
                    -rel.x * std::sin(yaw) + rel.y * std::cos(yaw), target.x, target.y, target.z,
                    (target - targetHip).length() / length, ankle.x, ankle.y, ankle.z, (ankle - hip).length() / length,
                    std::min(heelNow.z, toeNow.z), std::hypot(footLink.linearVelocity.x, footLink.linearVelocity.y),
                    clipNow.x, clipNow.y, clipNow.z, (clipNow - targetHip).length() / length,
                    styleNow.cyclePlanted[side] ? "apoiado" : "balanco");
            }
            const Vec3 heel = footLink.position + footLink.orientation.rotate(footGeometry.heel[side]);
            const Vec3 toe = footLink.position + footLink.orientation.rotate(footGeometry.toeBase[side]);
            const float sole = std::min(heel.z, toe.z) - athleteGroundHeight3D(*scene, profile, footLink.position);
            // (No gesto; na volta o pe anda com o do clipe, que sai e pousa
            // rente ao chao.)
            if (athlete.ballPlay.move().gesturing() && sole < 0.02f
                && std::hypot(footLink.linearVelocity.x, footLink.linearVelocity.y) > 1.0f)
                ++metrics.moveDragTicks;
        }
        // Manejo novo: as pernas nunca se cruzam no gesto (o pe que joga do
        // outro lado do de apoio, com os dois lado a lado).
        // (So na parte desenhada do gesto: na espera e na volta o pe anda com
        // o do clipe - o que o clipe cruza e dele.)
        const bool designed = athlete.ballPlay.move().gesturing()
            || athlete.ballPlay.move().phase() == BallMovePhase3D::Land;
        if (settings.newBallHandling && designed && leftFoot < bodyState.links.size()
            && rightFoot < bodyState.links.size()) {
            const int kick = athlete.ballPlay.move().side();
            const float yaw = legacy.defaultLocomotion.output().facingYawRadians;
            const Vec3 forward { std::cos(yaw), std::sin(yaw), 0.0f };
            const Vec3 leftward { -forward.y, forward.x, 0.0f };
            const Vec3 kickFoot = bodyState.links[kick == 0 ? leftFoot : rightFoot].position;
            const Vec3 supportFoot = bodyState.links[kick == 0 ? rightFoot : leftFoot].position;
            const Vec3 apart = kickFoot - supportFoot;
            const float lateral = (apart.x * leftward.x + apart.y * leftward.y) * (kick == 0 ? 1.0f : -1.0f);
            const float along = apart.x * forward.x + apart.y * forward.y;
            if (lateral < -0.02f && std::abs(along) < 0.20f) ++metrics.crossings;
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
                    // (No gesto desenhado: a perna dele e a de apoio - na volta e
                    // fora dele, o clipe.)
                    if (athlete.ballPlay.move().gesturing() || athlete.ballPlay.move().phase() == BallMovePhase3D::Land)
                        metrics.gestureOverlap = std::max(metrics.gestureOverlap, overlap);
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
                if (t > 0.35f && center.z < collider.boxHalfExtents.z + 0.03f)
                    metrics.footSlide.push_back(std::hypot(link.linearVelocity.x, link.linearVelocity.y));
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
                    const float end = style.swingWindows[side][0].end;
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
                            if (settings.dribbleV5 && athlete.dribbleOutput.legs[side].weight > 0.0f) ++metrics.swingFootOnBallTicks;
                            if (std::getenv("XPEBOLA"))
                                std::printf("  t %.3f PE NA BOLA %c entra %.3f m | balanco %.2f | pe z %.3f\n", t, side == 0 ? 'E' : 'D',
                                    inside, athlete.dribbleOutput.legs[side].weight, foot.position.z);
                        }
                    }
                const float left = bodyState.links[legs.foot[0]].position.z, right = bodyState.links[legs.foot[1]].position.z;
                metrics.flightRun = left > 0.10f && right > 0.10f ? metrics.flightRun + Dt : 0.0f;
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
    for (const BallActionRecord3D& record : athlete.ballPlay.queue().history()) {
        if (metrics.acceptTime < 0.0f && record.latency >= 0.0f && metrics.requestTime >= 0.0f)
            metrics.acceptTime = metrics.requestTime + record.latency;
        metrics.outcomes += std::string(metrics.outcomes.empty() ? "" : ", ") + ballActionOutcomeName3D(record.outcome);
        if (record.outcome != BallActionOutcome3D::Touched) metrics.outcomes += std::string(" (") + record.reason + ")";
        if (record.latency >= 0.0f) metrics.worstLatency = std::max(metrics.worstLatency, record.latency);
    }
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
    std::printf("%-18s   andou %.2f m, recaidas %d, %s, fim %s (%s)\n", "", metrics.travel, metrics.relapses,
        metrics.fell ? "CAIU" : "de pe", bodyModeName3D(metrics.finalMode), metrics.trail.c_str());
    std::printf("%-18s   joelho esticando max %.0f graus/s (alvo %.0f), acel max %.0f graus/s2\n", "",
        metrics.peakKneeExtension, metrics.peakKneeExtensionTarget, metrics.peakKneeAcceleration);
    std::printf("%-18s   apoio %5.2f / %5.2f / %5.2f m/s%s", "", percentile(metrics.plantedSlide, 0.5f),
        percentile(metrics.plantedSlide, 0.95f), percentile(metrics.plantedSlide, 1.0f), footsteps ? "" : "\n");
    if (!metrics.outcomes.empty())
        std::printf("%-18s   manejo novo: %s | atraso ate o gesto max %.3f s\n", "", metrics.outcomes.c_str(),
            metrics.worstLatency);
    if (metrics.ballDistanceCount > 0)
        std::printf("%-18s   conduzir: bola longe (>2,5 m) %.2f s, distancia media %.2f m\n", "", metrics.ballFarSeconds,
            metrics.ballDistanceSum / static_cast<float>(metrics.ballDistanceCount));
    if (metrics.requests > 0) {
        // (Plano v5: a folga pe-bola na forca, o atraso do clique a forca, a
        // perna louca e o voo.)
        float latencySum = 0.0f, latencyMax = 0.0f;
        for (const float value : metrics.latencies) {
            latencySum += value;
            latencyMax = std::max(latencyMax, value);
        }
        std::printf("%-18s   v5: pedidos %d toques %d | folga pe-bola na forca med %.3f max %.3f m | atraso med %.3f max %.3f s | perna louca %d joelho virando %d giro max %.1f rad/s | voo max %.3f s | garantia %zu folga med %.3f | perdidos %d | desistiu %d errou %d juntados %d | pe na bola %d (balanco %d) | dentro na forca %.3f\n",
            "", metrics.requests, metrics.touches, percentile(metrics.contactGaps, 0.5f), percentile(metrics.contactGaps, 1.0f),
            metrics.latencies.empty() ? 0.0f : latencySum / static_cast<float>(metrics.latencies.size()), latencyMax,
            metrics.crazyLegTicks, metrics.kneeFlipTicks, metrics.legSpinMax, metrics.longestFlight,
            metrics.guaranteedGaps.size(), percentile(metrics.guaranteedGaps, 0.5f),
            metrics.lostRequests + static_cast<int>(metrics.pendingRequests.size()), metrics.abortedTouches,
            metrics.missedTouches, metrics.mergedRequests, metrics.footOnBallTicks, metrics.swingFootOnBallTicks,
            metrics.contactInside);
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
        std::printf("%-18s   manejo: pedido %.3f aceite %.3f forca %.3f | movimento %d setor %d perna %d | perna volta %.3f s, cruzou %d\n",
            "", metrics.requestTime, metrics.acceptTime, metrics.forceTime, metrics.moveTouch, metrics.moveSector,
            metrics.moveSide, metrics.legReturn, metrics.crossings);
    if (footsteps)
        std::printf(" | saida do apoio max %.3f m | alcance max %.3f m | arranca %d, acomoda %d, antes %d, no ar %d\n",
            metrics.worstLiftCorrection, metrics.worstReach, metrics.startSteps, metrics.settleSteps, metrics.earlyLifts,
            metrics.catchSteps);
    if (metrics.phaseReach[1] > 0.0f)
        std::printf("%-18s   perna no movimento por fase (alcance / ticks reta / alem do clipe): passagem %.2f/%d/%d acompanhamento %.2f/%d/%d pouso %.2f/%d/%d apoiado %.2f/%d/%d volta %.2f/%d/%d\n",
            "", metrics.phaseReach[1], metrics.phaseStraightTicks[1], metrics.phaseBeyondClipTicks[1],
            metrics.phaseReach[2], metrics.phaseStraightTicks[2], metrics.phaseBeyondClipTicks[2],
            metrics.phaseReach[3], metrics.phaseStraightTicks[3], metrics.phaseBeyondClipTicks[3],
            metrics.phaseReach[4], metrics.phaseStraightTicks[4], metrics.phaseBeyondClipTicks[4],
            metrics.phaseReach[5], metrics.phaseStraightTicks[5], metrics.phaseBeyondClipTicks[5]);
    if (metrics.phaseReach[1] > 0.0f)
        std::printf("%-18s   pe do gesto para tras no mundo max (m/s): passagem %.2f acompanhamento %.2f pouso %.2f apoiado %.2f volta %.2f\n",
            "", metrics.phaseBackSpeed[1], metrics.phaseBackSpeed[2], metrics.phaseBackSpeed[3], metrics.phaseBackSpeed[4],
            metrics.phaseBackSpeed[5]);
    if (metrics.transitionPeak > 0.0f)
        std::printf("%-18s   troca fisico>padrao: pelve max %.2f m/s, pelve x capsula max %.2f m/s, pico %.0f m/s2, peito %.3f em 0,1 s\n",
            "", metrics.transitionSpeedMax, metrics.transitionSpeedGap, metrics.transitionPeak, metrics.transitionTrunkRate);
    std::printf("%-18s   pernas: cruzadas %d ticks, sobreposicao %.3f m (no gesto %.3f)", "", metrics.legCrossTicks,
        metrics.legOverlap, metrics.gestureOverlap);
    if (metrics.moveSide >= 0)
        std::printf(" | movimento: perna esticada %.2f, pe arrastando %d", metrics.moveReach, metrics.moveDragTicks);
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
            expect(!has(m, "fisico") && m.pelvisOverCapsule < 0.15f && m.travel >= travel && !m.fell, what, m);
        }
        for (const char* name : { "degrau_35_anda", "degrau_35_corre" }) {
            const Metrics m = run(name, *config, false);
            char what[160];
            std::snprintf(what, sizeof(what), "terreno%s, %s: sem lancamento (pelve %.2f m sobre a capsula)",
                steps ? " (pegadas)" : "", name, m.pelvisOverCapsule);
            expect(m.pelvisOverCapsule < 0.3f, what, m);
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
    // O manejo (plano v3): com a bola na area, toda acao sai, na hora; o
    // movimento padronizado nunca cruza nem atravessa as pernas, nao estica a
    // perna no ar, nao arrasta o pe, sem salto de pose e sem tranco; mudando o
    // movimento, a perna sai dele logo. Os casos do plano v2 e as combinacoes
    // (parado e as 8 direcoes andando e correndo, vezes os 8 toques), no jogo
    // de hoje (sem as pegadas, fora desde 11/10).
    AthleteRuntimeSettings3D handling;
    handling.newBallHandling = true;
    {
        int total = 0, failed = 0;
        float worstLegs = 0.0f, worstSlide = 0.0f, worstReach = 0.0f;
        for (const Scenario& scenario : scenarios()) {
            const std::string& name = scenario.name;
            const bool old = name == "toque_parado" || name == "toque_lado" || name == "chute_parado" || name == "chute_lado"
                || name == "bola_vindo" || name == "bola_alta" || name == "bola_quadril" || name == "atras_fora"
                || name == "atras_tras" || name == "toque_corre" || name == "chute_corre" || name == "chute_corre_cedo"
                || name == "corte_corre" || name.rfind("bola_", 0) == 0;
            const bool grid = name.rfind("mov_", 0) == 0;
            if (name.rfind("v2_", 0) != 0 && !old && !grid) continue;
            for (const AthleteRuntimeSettings3D* config : { &handling }) {
                const Metrics m = runScenario(context, scenario, *config, false, nullptr, 1);
                ++total;
                const bool steps = config->footstepLocomotion;
                std::vector<std::string> problems;
                if (m.requests != 1 || m.touches != 1 || m.outcomes != "tocou")
                    problems.push_back("acao (" + m.outcomes + ")");
                if (m.forceTime >= 0.0f && m.acceptTime >= 0.0f && std::abs(m.forceTime - m.acceptTime) > 1.5f * Dt)
                    problems.push_back("forca fora do aceite");
                // (Sem as pegadas, os clipes cruzam sozinhos - o strafe, o giro -:
                // ali so o gesto conta.)
                const int crossed = steps ? std::max(m.crossings, m.legCrossTicks) : m.crossings;
                if (crossed != 0) problems.push_back("cruzou " + std::to_string(crossed));
                if (m.gestureOverlap >= 0.02f) problems.push_back("atravessou no gesto " + std::to_string(m.gestureOverlap));
                if (m.moveReach > 0.95f) problems.push_back("perna esticada " + std::to_string(m.moveReach));
                // (O chute sai do chao: ate dois ticks com a ponta rente.)
                const bool shot = name.find("chute") != std::string::npos;
                if (m.moveDragTicks > (shot ? 2 : 1)) problems.push_back("pe arrastando " + std::to_string(m.moveDragTicks));
                // (O primeiro tick do movimento num pe que ja balancava: o joelho
                // do IK difere um pouco do do clipe - ate 3,5 cm.)
                if (m.worstPoseJump >= 0.035f) problems.push_back("salto " + std::to_string(m.worstPoseJump));
                if (m.peakUpperAcceleration >= 550.0f) problems.push_back("tronco " + std::to_string(m.peakUpperAcceleration));
                if (m.peakLegAcceleration >= 2200.0f) problems.push_back("pernas " + std::to_string(m.peakLegAcceleration));
                if (steps && percentile(m.plantedSlide, 1.0f) >= 2.5f)
                    problems.push_back("apoio escorrega " + std::to_string(percentile(m.plantedSlide, 1.0f)));
                if (scenario.changeAt >= 0.0f && (m.legReturn < 0.0f || m.legReturn > 0.22f + 2.0f * Dt))
                    problems.push_back("perna volta " + std::to_string(m.legReturn));
                worstLegs = std::max(worstLegs, m.peakLegAcceleration);
                worstReach = std::max(worstReach, m.moveReach);
                if (steps) worstSlide = std::max(worstSlide, percentile(m.plantedSlide, 1.0f));
                if (!problems.empty()) {
                    ++failed;
                    std::string text;
                    for (const std::string& problem : problems) text += (text.empty() ? "" : ", ") + problem;
                    std::printf("  FALHOU manejo%s, %s: %s\n", steps ? " (pegadas)" : "", name.c_str(), text.c_str());
                }
            }
        }
        char what[240];
        std::snprintf(what, sizeof(what),
            "manejo: %d de %d cenarios com acao na hora, sem cruzar nem atravessar, sem esticar nem arrastar, sem tranco (pernas max %.0f m/s2, perna no ar max %.2f, apoio max %.2f m/s)",
            total - failed, total, worstLegs, worstReach, worstSlide);
        Metrics none;
        none.trail = "manejo";
        expect(failed == 0, what, none);
    }
    // A forca sai no aceite (plano v3: sem a regua do tempo ate o contato).
    for (const char* name : { "v2_parado_35_000", "v2_parado_35_270", "v2_anda_35_045", "v2_corre_35_000", "v2_chute_parado_000" }) {
        const Metrics m = run(name, handling, false);
        char what[160];
        std::snprintf(what, sizeof(what), "manejo, %s: a forca no aceite (%.3f, aceite %.3f)", name, m.forceTime, m.acceptTime);
        expect(m.forceTime >= 0.0f && std::abs(m.forceTime - m.acceptTime) <= 1.5f * Dt, what, m);
    }
    // Conduzindo (toques seguidos, a perna do toque anterior ainda voltando
    // enquanto a outra toca): sem salto de pose nem tranco nas pernas. (Ate
    // 11/10 a volta da perna anterior corria atras do proprio pe - o "pe do
    // clipe" dela vinha da pose com o gesto: 36 cm de salto andando.)
    for (const Scenario& scenario : scenarios()) {
        if (scenario.name.rfind("conduz_", 0) != 0) continue;
        const Metrics m = runScenario(context, scenario, handling, false, nullptr, 1);
        char what[192];
        std::snprintf(what, sizeof(what), "conduzir, %s: %d toques, pernas max %.0f m/s2, salto %.3f m",
            scenario.name.c_str(), m.touches, m.peakLegAcceleration, m.worstPoseJump);
        expect(m.touches >= 3 && m.peakLegAcceleration < 2200.0f && m.worstPoseJump < 0.035f, what, m);
    }
    // A capsula com o pedido faz o mesmo caminho que sem ele - o gesto nao
    // mexe no caminho nem atrasa o corpo (o corpo nao desacelera mais no
    // gesto: usuario, 11/10).
    for (const char* name : { "v2_anda_35_045", "v2_anda_65_270", "v2_corre_35_000", "v2_corre_35_270", "v2_corre_90_180",
             "v2_fase_3", "v2_chute_corre_000", "v2_chute_corre_090" }) {
        for (const Scenario& scenario : scenarios()) {
            if (scenario.name != name) continue;
            const Metrics with = runScenario(context, scenario, handling, false, nullptr, 1);
            const Metrics without = runScenario(context, scenario, handling, false, nullptr, 1, true);
            const std::vector<Vec3>& path = without.capsulePath;
            float side = 0.0f, behind = 0.0f;
            for (std::size_t i = 0; i < with.capsulePath.size(); ++i) {
                const Vec3 point { with.capsulePath[i].x, with.capsulePath[i].y, 0.0f };
                float nearest = 1e9f;
                for (std::size_t j = 0; j + 1 < path.size(); ++j) {
                    const Vec3 a { path[j].x, path[j].y, 0.0f }, b { path[j + 1].x, path[j + 1].y, 0.0f };
                    const Vec3 ab = b - a;
                    const float u = ab.lengthSquared() > 1e-12f ? std::clamp(dot(point - a, ab) / ab.lengthSquared(), 0.0f, 1.0f) : 0.0f;
                    nearest = std::min(nearest, (point - (a + ab * u)).length());
                }
                side = std::max(side, nearest);
                if (i < path.size())
                    behind = std::max(behind, std::hypot(with.capsulePath[i].x - path[i].x, with.capsulePath[i].y - path[i].y));
            }
            char what[192];
            std::snprintf(what, sizeof(what),
                "manejo v2, %s: a capsula faz o mesmo caminho (de lado %.4f m, atraso max %.3f m)", name, side, behind);
            expect(side < 0.01f && behind < 0.05f, what, with);
        }
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

// Plano v5 (E3): as poses de contato paradas - o corpo de pe (a pose de
// ligacao, a pelve 5 cm mais baixa), uma perna com a parte certa do pe
// encostada na bola, para os 8 rumos do toque e as duas pernas. Um quadro por
// pose (0,1 s cada), para a folha de quadros.
int writeContactPoses(const RagdollProfile3D& profile, const std::string& path) {
    const ContactFeet3D feet = resolveContactFeet3D(profile);
    if (!feet.valid) {
        std::fprintf(stderr, "perfil sem a geometria dos pes\n");
        return 1;
    }
    FILE* file = std::fopen(path.c_str(), "w");
    if (file == nullptr) return 1;
    std::fprintf(file, "S poses_contato\n");
    const Quaternion rootOrientation = profile.links.front().modelOrientation;
    // (A pelve do modelo fica na origem: de pe, na altura de pe do perfil.)
    const Vec3 standingRoot { 0.0f, 0.0f, profile.standingRootHeightMeters };
    const Vec3 rootPosition = standingRoot - Vec3 { 0.0f, 0.0f, 0.05f };
    std::vector<Vec3> bind(profile.links.size(), Vec3 {});
    RagdollAnimationPose3D standing;
    athleteWorldPose3D(profile, bind, standingRoot, rootOrientation, standing);
    const float radius = 0.11f;
    const Vec3 forward { 1.0f, 0.0f, 0.0f }, left { 0.0f, 1.0f, 0.0f };
    int frame = 0;
    for (int side = 0; side < 2; ++side) {
        const float out = side == 0 ? 1.0f : -1.0f;
        for (int k = 0; k < 8; ++k) {
            const float angle = static_cast<float>(k) * 0.785398f;
            const Vec3 push { std::cos(angle), std::sin(angle), 0.0f };
            // (A bola onde esta perna a toca bem nesse rumo.)
            const Vec3 ball = Vec3 { 0.0f, 0.0f, radius } + forward * (0.26f * std::cos(angle) + 0.06f)
                + left * (0.13f * out + 0.10f * std::sin(angle));
            ContactStyle3D style = ContactStyle3D::Front;
            BallTouchSurface3D surface = BallTouchSurface3D::Inside;
            if (!contactStyle3D(side, ball, rootPosition, forward, push, false, BallActionKind3D::Dribble, 0.0f, style,
                    surface))
                continue;
            ContactRequest3D request;
            request.side = side;
            request.surface = surface;
            request.style = style;
            request.ball = ball;
            request.radius = radius;
            request.push = push;
            request.bodyForward = forward;
            request.power = 0.4f;
            const ContactPose3D pose = contactPose3D(feet, request);
            std::vector<Vec3> coordinates = bind;
            // O apoio plano no lugar dele (a pelve desceu) e a perna do toque.
            const std::size_t support = static_cast<std::size_t>(1 - side);
            const std::size_t supportFoot = feet.legs.foot[support];
            solveAthleteLeg3D(profile, feet.legs, 1 - side, coordinates, rootPosition, rootOrientation,
                standing.linkPositions[supportFoot] + standing.linkOrientations[supportFoot].rotate(feet.legs.ankleInFoot[support]),
                forward, standing.linkOrientations[supportFoot]);
            const Vec3 knee = (forward + left * (out * (surface == BallTouchSurface3D::Outside ? -0.25f : 0.25f))).normalized();
            solveAthleteLeg3D(profile, feet.legs, side, coordinates, rootPosition, rootOrientation, pose.ankle, knee,
                pose.orientation);
            RagdollAnimationPose3D world;
            athleteWorldPose3D(profile, coordinates, rootPosition, rootOrientation, world);
            RagdollState3D state;
            state.links.resize(profile.links.size());
            for (std::size_t i = 0; i < profile.links.size(); ++i) {
                state.links[i].position = world.linkPositions[i];
                state.links[i].orientation = world.linkOrientations[i];
            }
            std::fprintf(file, "T\n");
            dumpFrame(file, profile, state, 0.1f * static_cast<float>(frame), 0);
            std::fprintf(file, "K %.4f %.4f %.4f 0 0 0\n", ball.x, ball.y, ball.z);
            const float gap = contactFootGap3D(feet, side, world.linkPositions[feet.legs.foot[static_cast<std::size_t>(side)]],
                world.linkOrientations[feet.legs.foot[static_cast<std::size_t>(side)]], ball, radius);
            const Vec3 hip = world.linkPositions.front() + world.linkOrientations.front().rotate(feet.legs.hipInPelvis[static_cast<std::size_t>(side)]);
            if (std::getenv("XPOSES"))
                std::printf("  bola %.3f %.3f %.3f | pe %.3f %.3f %.3f | tornozelo %.3f %.3f %.3f | ponto %.3f %.3f %.3f | quadril %.3f %.3f %.3f | pe no corpo %.3f %.3f %.3f | raiz %.3f %.3f %.3f\n",
                    ball.x, ball.y, ball.z, pose.foot.x, pose.foot.y, pose.foot.z, pose.ankle.x, pose.ankle.y, pose.ankle.z,
                    pose.surfacePoint.x, pose.surfacePoint.y, pose.surfacePoint.z, hip.x, hip.y, hip.z,
                    world.linkPositions[feet.legs.foot[static_cast<std::size_t>(side)]].x,
                    world.linkPositions[feet.legs.foot[static_cast<std::size_t>(side)]].y,
                    world.linkPositions[feet.legs.foot[static_cast<std::size_t>(side)]].z, rootPosition.x, rootPosition.y, rootPosition.z);
            std::printf("quadro %2d perna %c rumo %3d superficie %d estilo %d | folga %.3f m | alcance %.2f\n", frame,
                side == 0 ? 'E' : 'D', k * 45, static_cast<int>(surface), static_cast<int>(style), gap,
                (pose.ankle - hip).length() / feet.legLength[static_cast<std::size_t>(side)]);
            ++frame;
        }
    }
    std::fclose(file);
    return 0;
}

int main(int argc, char** argv) {
    std::string only, dumpPath, contactPoses;
    bool dribbleV5 = false;
    int every = 3;
    bool startPhysical = false;
    bool activeBody = AthleteRuntimeSettings3D {}.activeBody;
    bool footsteps = false;
    bool balanceSteps = AthleteRuntimeSettings3D {}.activeStanceSettings.balanceSteps;
    bool checks = false;
    int parity = -1;
    int baseline = -1;
    float response = AthleteRuntimeSettings3D {}.capsuleResponsePercent;
    bool newBall = false;
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
        else if (arg == "--manejo") newBall = std::atoi(need()) != 0;
        else if (arg == "--sem-pedido") noRequests = true;
        else if (arg == "--ponte") bridge = std::atoi(need()) != 0;
        else if (arg == "--teste") checks = true;
        else if (arg == "--paridade") parity = i + 1 < argc && argv[i + 1][0] != '-' ? std::atoi(argv[++i]) : 0;
        else if (arg == "--linha-de-base") baseline = i + 1 < argc && argv[i + 1][0] != '-' ? std::atoi(argv[++i]) : 0;
        else if (arg == "--poses-contato") contactPoses = need();
        else if (arg == "--v5") dribbleV5 = std::atoi(need()) != 0;
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
        else if (arg == "--lista") {
            for (const Scenario& scenario : scenarios()) std::printf("%s\n", scenario.name.c_str());
            return 0;
        } else {
            std::fprintf(stderr, "argumento desconhecido: %s\n", arg.c_str());
            return 2;
        }
    }
    const std::string assets = MATTERENGINE_ASSETS_DIR;
    const RagdollCharacter3D character = loadRagdollCharacter3D(assets + "/characters/football_player/character.json");
    const RagdollProfile3D& profile = character.profile;
    std::vector<std::string> clipErrors;
    const std::vector<AnimationClip3D> clipLibrary =
        loadAthleteClipLibrary3D(assets + "/animations/clips", profile, &clipErrors);
    const CharacterLocomotionAnimations3D clips = athleteLocomotionClips3D(character, clipLibrary);
    const CharacterLocomotionAnimations3D defaultClips =
        defaultLocomotionAnimations3D(clips, character.defaultLocomotionClipIds, clipLibrary);
    const CharacterGaitSpeeds3D gaitSpeeds = characterGaitSpeeds3D(clips);
    CharacterMotorSettings3D avatar = athleteAvatarMotorSettings3D();
    if (gaitSpeeds.walk > 0.0f) {
        avatar.walkSpeed = gaitSpeeds.walk;
        avatar.sprintSpeed = gaitSpeeds.sprint;
    }
    AthleteRuntimeSettings3D settings;
    settings.activeBody = activeBody;
    settings.footstepLocomotion = footsteps;
    settings.activeStanceSettings.balanceSteps = balanceSteps;
    settings.capsuleResponsePercent = response;
    settings.newBallHandling = newBall;
    settings.dribbleV5 = dribbleV5;
    settings.physicalBridge = bridge;
    settings.physicalFeetAssistPercent = feetAssist;
    settings.physicalBoneGuidePercent = boneGuide;
    settings.legClearance.enabled = legClearance;
    settings.returnSmoothing = returnSmoothing;
    settings.modeTable = modeTable;
    settings.physicalUsesDefaultSpeeds = defaultSpeeds;
    settings.e5StandingAssistPercent = standingAssist;
    settings.e5WalkingAssistPercent = walkingAssist;
    if (totalPhysical) {
        settings.defaultLocomotion = false;
        startPhysical = true;
    }
    ProbeContext context;
    context.profile = &profile;
    context.gaitSpeeds = gaitSpeeds;
    context.noCapsule = noCapsule;
    context.avatar = avatar;
    context.runtimeAssets.profile = &profile;
    context.runtimeAssets.clips = &clips;
    context.runtimeAssets.defaultClips = &defaultClips;
    context.runtimeAssets.avatarSettings = avatar;
    if (checks) return runChecks(context);
    if (parity >= 0) return runParity(context, parity, 0);
    if (baseline >= 0) return runBaseline(context, baseline);
    if (!contactPoses.empty()) return writeContactPoses(profile, contactPoses);

    FILE* dump = dumpPath.empty() ? nullptr : std::fopen(dumpPath.c_str(), "w");
    std::printf("cenario            | deslize pe no chao med/p95/max m/s | pico acel tronco+bracos / pernas | salto pose | modos (trocas; animado/ativo/fisico/volta s; ajuda N.s) | bola\n");
    for (const Scenario& scenario : scenarios()) {
        if (!only.empty() && scenario.name != only) continue;
        printMetrics(scenario, runScenario(context, scenario, settings, startPhysical, dump, every, noRequests),
            settings.footstepLocomotion);
    }
    if (dump != nullptr) std::fclose(dump);
    return 0;
}
