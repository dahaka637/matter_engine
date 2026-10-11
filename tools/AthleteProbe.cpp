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
// Sem --cenario roda todos.
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"
#include "Engine/Control/AdaptivePhysicalCharacter3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

constexpr float Pi = 3.14159265358979323846f;
constexpr float Dt = 1.0f / 120.0f;

// O que o "jogador" faz num instante: teclas no referencial da camera (x
// frente, y esquerda), o olhar e os pedidos.
struct Keys {
    float forward = 0.0f;
    float left = 0.0f;
    bool sprint = false;
    float cameraYaw = 0.0f;
    bool jump = false;
    bool crouch = false;
    BallActionKind3D ball = BallActionKind3D::None;
    float power = 0.5f;
};

// Empurrao: forca (N, no mundo) por um tempo, repartida em pelve, peito e
// alto do peito como o teste de impacto do laboratorio.
struct Push {
    float at = 0.0f;
    float seconds = 0.1f;
    Vec3 force;
};

// Uma caixa parada no mundo (parede, degrau, rampa): centro, meia-dimensao e
// orientacao.
struct StaticBox {
    Vec3 center;
    Vec3 half;
    Quaternion orientation;
};

// Um objeto arremessado no atleta (a pancada de verdade: contato, nao
// forca): caixa de massa e meia-dimensao dadas, saindo de `start` (em
// relacao aos pes) com `velocity` no instante `at`.
struct Throw {
    float at = 0.0f;
    float massKg = 20.0f;
    Vec3 halfExtents { 0.2f, 0.2f, 0.2f };
    Vec3 start;
    Vec3 velocity;
};

struct Scenario {
    std::string name;
    float seconds;
    std::function<Keys(float)> keys;
    bool ball = false;
    Vec3 ballStart;     // em relacao aos pes, no referencial inicial
    Vec3 ballVelocity;
    std::vector<Push> pushes;
    std::vector<Throw> throws;
    // Pedido automatico com a bola a menos disto da pelve (no plano).
    float autoDistance = 0.0f;
    BallActionKind3D autoKind = BallActionKind3D::Dribble;
    // O rumo pedido vira isto (rad, + esquerda) a partir do pedido automatico.
    float autoTurn = 0.0f;
    // Manejo v2: a bola posta em relacao a pelve no instante placeAt (x a
    // frente, y a esquerda no rumo do corpo; z a altura do centro), andando
    // junto com o corpo; o pedido sai na mesma hora (pelas teclas). E o
    // instante em que o movimento pedido muda depois dele (-1 nenhum).
    float placeAt = -1.0f;
    Vec3 placeOffset;
    float changeAt = -1.0f;
    // Comeca no fisico (o empurrao so age no corpo fisico).
    bool startPhysical = false;
    // Uma parede parada (centro e meia-dimensao; sem parede, meia-dimensao 0)
    // e o terreno parado (degraus, escada, rampa).
    Vec3 wallCenter;
    Vec3 wallHalf;
    std::vector<StaticBox> statics;
};

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

struct Metrics {
    std::vector<float> footSlide;
    float peakUpperAcceleration = 0.0f;
    float peakLegAcceleration = 0.0f;
    float worstPoseJump = 0.0f;
    int modeSwitches = 0;
    float animatedSeconds = 0.0f;
    float physicalSeconds = 0.0f;
    float returningSeconds = 0.0f;
    float activeSeconds = 0.0f;
    float assistImpulse = 0.0f;
    int requests = 0;
    int touches = 0;
    bool fell = false;
    float lowestPelvis = 10.0f;
    float ballSpeedAfter = 0.0f;
    // O pe apoiado (o estilo diz apoio, no modo padrao): a velocidade dele
    // no plano (m/s) - zero e o certo. Pegadas: a maior correcao na saida do
    // apoio (m), o maior erro de alcance da perna (m) e os passos de
    // acomodacao.
    std::vector<float> plantedSlide;
    float worstLiftCorrection = 0.0f;
    float worstReach = 0.0f;
    int settleSteps = 0;
    int earlyLifts = 0;
    int catchSteps = 0;
    int startSteps = 0;
    // Manejo novo: os desfechos dos pedidos, o maior atraso ate o gesto (s) e
    // os ticks com as pernas cruzadas no gesto.
    std::string outcomes;
    float worstLatency = 0.0f;
    int crossings = 0;
    // Os modos por onde passou ("animado>ativo>volta>animado") e o do fim.
    std::string trail;
    // O joelho: a maior velocidade de esticar (graus/s) no modo padrao, no
    // corpo e no alvo (a pose), e a maior aceleracao angular dele (graus/s2)
    // no corpo - o estalo da perna que estica de uma vez.
    float peakKneeExtension = 0.0f;
    float peakKneeExtensionTarget = 0.0f;
    float peakKneeAcceleration = 0.0f;
    // Manejo v2: o pedido, o aceite e a forca (s; -1 nao houve), a forca
    // planejada pela regua (do aceite) e o contato estimado, o gesto na bola
    // imaginada, o tempo ate a perna sair do gesto depois de mudar o
    // movimento (s; -1 nao medido) e o caminho da capsula.
    float requestTime = -1.0f;
    float acceptTime = -1.0f;
    float forceTime = -1.0f;
    float forceAt = -1.0f;
    float estimatedContact = -1.0f;
    bool imagined = false;
    float legReturn = -1.0f;
    std::vector<Vec3> capsulePath;
    // O levantar: no comeco (o primeiro Rising) a frente do peito e a da
    // pelve (z: + de costas, - de brucos) e o lado escolhido; quando acabou
    // de pe (s; -1 nao levantou).
    // O terreno: a maior velocidade para cima da capsula e da pelve (m/s) - o
    // lancamento - e a altura maxima da pelve sobre a capsula (m).
    float capsuleUp = 0.0f;
    float pelvisUp = 0.0f;
    float pelvisOverCapsule = 0.0f;
    float riseAt = -1.0f;
    float riseChest = 0.0f;
    float risePelvis = 0.0f;
    int riseSide = 0;
    float risenAt = -1.0f;
    // O quanto a pelve andou no plano (m), e as idas ao fisico depois de ja
    // ter voltado uma vez ao padrao (a recaida).
    float travel = 0.0f;
    int relapses = 0;
    LegacyBodyMode3D finalMode = LegacyBodyMode3D::Animated;
};

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
        if (collider.shape == RagdollColliderShape3D::Box) {
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

const char* modeName(LegacyBodyMode3D mode) {
    switch (mode) {
    case LegacyBodyMode3D::Animated: return "animado";
    case LegacyBodyMode3D::Physical: return "fisico";
    case LegacyBodyMode3D::Returning: return "volta";
    case LegacyBodyMode3D::Active: return "ativo";
    }
    return "?";
}

} // namespace

namespace {

struct ProbeContext {
    const RagdollProfile3D* profile = nullptr;
    CharacterGaitSpeeds3D gaitSpeeds;
    CharacterMotorSettings3D avatar;
    AthleteRuntimeAssets3D runtimeAssets;
};

// Um cenario do comeco ao fim (a cena nova, o boneco, as teclas, as medidas).
Metrics runScenario(const ProbeContext& context, const Scenario& scenario, const AthleteRuntimeSettings3D& settings,
    bool startPhysicalArgument, FILE* dump, int every, bool suppressRequests = false) {
    const bool startPhysical = startPhysicalArgument || scenario.startPhysical;
    const RagdollProfile3D& profile = *context.profile;
    const AthleteFootGeometry3D footGeometry = resolveAthleteFootGeometry3D(profile, resolveAthleteLegs3D(profile), 0.08f);
    const CharacterGaitSpeeds3D& gaitSpeeds = context.gaitSpeeds;
    const CharacterMotorSettings3D& avatar = context.avatar;
    const AthleteRuntimeAssets3D& runtimeAssets = context.runtimeAssets;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    auto scene = engine.createScene({}, materials);
    {
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = { 60.0f, 60.0f, 0.1f };
        shape.materialId = "concrete";
        PhysicsBodyDefinition3D ground;
        ground.motionType = PhysicsMotionType3D::Static;
        ground.position = { 0.0f, 0.0f, -0.1f };
        ground.materialId = "concrete";
        (void)scene->createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
    }
    {
        std::vector<StaticBox> statics = scenario.statics;
        if (scenario.wallHalf.lengthSquared() > 0.0f) statics.push_back({ scenario.wallCenter, scenario.wallHalf, {} });
        for (const StaticBox& box : statics) {
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = box.half;
            shape.materialId = "concrete";
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            body.position = box.center;
            body.orientation = box.orientation;
            body.materialId = "concrete";
            (void)scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
        }
    }
    scene->createCharacter({ 0.0f, 0.0f, 0.0f }, avatar);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 1;
    spawn.active = true;
    spawn.pelvisPosition = { 0.0f, 0.0f, profile.standingRootHeightMeters };
    const RagdollHandle3D handle = scene->createRagdoll(profile, spawn);
    RagdollState3D bodyState = scene->ragdollState(handle);
    std::size_t leftFoot = profile.links.size(), rightFoot = profile.links.size();
    std::array<std::size_t, 2> thighs { profile.links.size(), profile.links.size() };
    std::array<std::size_t, 2> shins { profile.links.size(), profile.links.size() };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") leftFoot = i;
        if (profile.links[i].id == "RightFoot") rightFoot = i;
        if (profile.links[i].id == "LeftThigh") thighs[0] = i;
        if (profile.links[i].id == "RightThigh") thighs[1] = i;
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
    AthleteState3D athlete;
    LegacyAthleteState3D& legacy = athlete.legacy;
    legacy.guideFacingYawRadians = 0.0f;
    legacy.locomotion.reset(profile, bodyState);
    legacy.adaptivePhysics.reset(profile);
    // Como "Restaurar pose" do laboratorio: ja no modo padrao. (--fisico:
    // como ao assumir o boneco, no fisico ate ele assentar.)
    if (!startPhysical) {
        legacy.defaultLocomotion.reset(0.0f);
        legacy.guided = true;
        legacy.bodyMode = LegacyBodyMode3D::Animated;
        scene->setRagdollGuided(handle, true);
    }
    InflatedBallHandle3D ballHandle;
    InflatedBallState3D ballState;
    BallOwnership3D ownership;
    if (dump != nullptr) std::fprintf(dump, "S %s\n", scenario.name.c_str());

    AthleteRuntime3D runtime;
    Vec3 followVelocity {};
    bool controlledAnimated = !startPhysical;
    float capsuleSpeedScale = 1.0f;
    Vec3 capsuleMoved {};
    float capsuleRose = 0.0f;
    float wallImpact = 0.0f;
    Metrics metrics;
    // (O modo Ativo por dentro da ponte fisico -> padrao: "ponte".)
    const auto modeOf = [&]() {
        return legacy.bridge && legacy.bodyMode == LegacyBodyMode3D::Active ? "ponte" : modeName(legacy.bodyMode);
    };
    metrics.trail = modeOf();
    LegacyBodyMode3D previousMode = legacy.bodyMode;
    std::vector<PhysicsBodyState3D> previousLinks = bodyState.links;
    std::vector<Vec3> previousVelocities(bodyState.links.size());
    bool previousJump = false;
    float jumpHeld = 0.0f;
    float autoCooldown = 0.0f;
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
            const float wanted = controlledAnimated
                ? athleteDefaultTravelSpeed3D(settings, travelRelative, keys.sprint)
                : characterGaitSpeed3D(gaitSpeeds, travelRelative, keys.sprint);
            const float nominal = keys.sprint ? avatar.sprintSpeed : avatar.walkSpeed;
            if (wanted > 0.0f && nominal > 0.0f) command.speedScale = wanted / nominal;
            if (controlledAnimated) command.speedScale *= capsuleSpeedScale;
            if (!controlledAnimated && wanted > 0.0f)
                command.speedScale *= std::clamp(legacy.locomotion.telemetry().terrainSpeedLimit / wanted, 0.2f, 1.0f);
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
        if (step >= 0 && scenario.autoDistance > 0.0f && scenario.ball && autoCooldown <= 0.0f && !bodyState.links.empty()) {
            const Vec3 pelvis = bodyState.links.front().position;
            const Vec3 offset = ballState.position - pelvis;
            if (std::hypot(offset.x, offset.y) < scenario.autoDistance) {
                // (Sem pedido: so a virada do rumo, para comparar.)
                if (!suppressRequests) control.ballRequest = scenario.autoKind;
                control.ballRequestPower = scenario.autoKind == BallActionKind3D::Shot ? 0.8f : 0.45f;
                autoCooldown = 10.0f;
                if (scenario.autoTurn != 0.0f) {
                    headingTurn = scenario.autoTurn;
                    control.intent.requestedDirectionWorld = turned(control.intent.requestedDirectionWorld, headingTurn);
                    control.intent.requestedVelocityWorld = turned(control.intent.requestedVelocityWorld, headingTurn);
                }
            }
        }
        if (control.ballRequest != BallActionKind3D::None) {
            ++metrics.requests;
            if (metrics.requestTime < 0.0f) metrics.requestTime = t;
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
        AthleteSlot3D slot { 1, handle, &bodyState, true, false, &athlete };
        std::vector<AthleteBall3D> balls;
        if (scenario.ball && step >= 0) balls.push_back({ ballHandle, &ballState, &ownership });
        AthleteRuntimeOutput3D output;
        runtime.update(*scene, runtimeAssets, std::span<AthleteSlot3D>(&slot, 1), balls, step < 0 ? warmSettings : settings,
            control, Dt, output);
        if (output.followVelocitySet) followVelocity = output.followVelocityWorld;
        if (output.controlledAnimatedSet) controlledAnimated = output.controlledAnimated;
        capsuleSpeedScale = output.controlledSpeedScale;
        if (legacy.ballLastOutput.touched) {
            ++metrics.touches;
            if (dump != nullptr) std::fprintf(dump, "T\n");
            if (metrics.forceTime < 0.0f && settings.newBallHandling) {
                metrics.forceTime = t;
                metrics.forceAt = athlete.ballPlay.forceAt();
                metrics.estimatedContact = athlete.ballPlay.estimatedContact();
                metrics.imagined = athlete.ballPlay.imagined();
            }
        }
        if (step >= 0) metrics.capsulePath.push_back(scene->characterState().position);
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
        if (legacy.bodyMode == LegacyBodyMode3D::Physical && !bodyState.links.empty()) {
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
            && (legacy.bodyMode != LegacyBodyMode3D::Physical
                || legacy.locomotion.telemetry().getUpPhase == CharacterGetUpPhase3D::None))
            metrics.risenAt = t;
        if (std::getenv("XQUEDA") && step % 15 == 0 && legacy.bodyMode == LegacyBodyMode3D::Physical && !bodyState.links.empty()) {
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
        if (std::getenv("XV2") && settings.newBallHandling && (athlete.ballOutput.body.active || athlete.ballPlay.acting()))
            std::printf("  t %.3f v2 acao %d gesto %d fase %d t %.3f contato %.3f perna %.2f peso %.2f lado %d agindo %d interrompe %d pedido %.2f %.2f\n",
                t, athlete.ballPlay.acting() ? 1 : 0, athlete.ballPlay.gesture().active() ? 1 : 0,
                static_cast<int>(athlete.ballPlay.gesture().phase()), athlete.ballPlay.gesture().time(),
                athlete.ballPlay.gesture().contactTime(), athlete.ballOutput.body.legWeight, athlete.ballOutput.body.weight,
                athlete.ballOutput.body.side, athlete.ballOutput.actingSide, athlete.ballPlay.interrupting() ? 1 : 0,
                control.intent.requestedVelocityWorld.x, control.intent.requestedVelocityWorld.y);
        // (A perna sai do gesto depois da mudanca de movimento - contada da
        // mudanca ou da forca, o que vier depois.)
        if (scenario.changeAt >= 0.0f && metrics.legReturn < 0.0f && metrics.forceTime >= 0.0f) {
            const float from = std::max(scenario.changeAt, metrics.forceTime);
            const BallBodyTargets3D& body = athlete.ballOutput.body;
            if (t >= from && (!body.active || body.legWeight <= 0.01f)) metrics.legReturn = t - from;
        }
        for (const Throw& item : scenario.throws)
            if (t >= item.at && t < item.at + Dt) {
                PhysicsShape3D shape;
                shape.type = PhysicsShapeType3D::Box;
                shape.halfExtents = item.halfExtents;
                shape.materialId = "wood";
                PhysicsBodyDefinition3D box;
                box.position = item.start;
                box.linearVelocity = item.velocity;
                box.massKg = item.massKg;
                box.materialId = "wood";
                box.characteristicSizeMeters = 2.0f * std::max({ item.halfExtents.x, item.halfExtents.y, item.halfExtents.z });
                (void)scene->createBody(box, std::span<const PhysicsShape3D>(&shape, 1));
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
        if (scenario.ball && step >= 0) ballState = scene->inflatedBallState(ballHandle);
        if (step < 0) {
            previousLinks = bodyState.links;
            for (std::size_t i = 0; i < bodyState.links.size() && i < previousVelocities.size(); ++i)
                previousVelocities[i] = bodyState.links[i].linearVelocity;
            previousMode = legacy.bodyMode;
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

        if (std::getenv("XATIVO") && legacy.bodyMode == LegacyBodyMode3D::Active && step % (std::getenv("XATIVO")[0] == '2' ? 1 : 6) == 0) {
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
        if (std::getenv("XGESTO") && athlete.ballPlay.gesture().active()) {
            const BallGesture3D& g = athlete.ballPlay.gesture();
            const Vec3 p = g.position();
            {
                const BallBodyTargets3D& bt = athlete.ballOutput.body;
                const Vec3 toe = g.orientation().rotate({ 1.0f, 0.0f, 0.0f });
                std::printf("  t %.3f joelho %.2f %.2f %.2f ponta %.2f %.2f %.2f\n", t, bt.knee.x, bt.knee.y, bt.knee.z,
                    toe.x, toe.y, toe.z);
            }
            std::printf("  t %.3f gesto fase %d t %.3f contato %.3f pe %.3f %.3f %.3f v %.2f | chao %.3f\n", t,
                static_cast<int>(g.phase()), g.time(), g.contactTime(), p.x, p.y, p.z, g.velocity().length(),
                athleteGroundHeight3D(*scene, profile, p));
        }
        if (std::getenv("XPEG") && settings.footstepLocomotion
            && (legacy.bodyMode == LegacyBodyMode3D::Animated || legacy.bodyMode == LegacyBodyMode3D::Returning)) {
            const DefaultLocomotionOutput3D& o = legacy.defaultLocomotion.output();
            const AthleteLegs3D& legs = athlete.footsteps.legs();
            std::printf("  t %.3f mov %.2f v %.2f |", t, o.movingShare, std::hypot(scene->characterState().velocity.x,
                scene->characterState().velocity.y));
            for (std::size_t side = 0; side < 2; ++side) {
                const char* mode = "LSWP" + static_cast<int>(athlete.footTargets[side].mode);
                const Vec3 stylePos = o.pose.linkPositions[legs.foot[side]];
                const Vec3 target = athlete.footTargets[side].position;
                std::printf(" %c %c%s d %.3f z %.3f alc %.3f lo %.2f td %.2f |", side == 0 ? 'E' : 'D',
                    "FLSP"[static_cast<int>(athlete.footTargets[side].mode)], o.footPlanted[side] ? "*" : " ",
                    std::hypot(target.x - stylePos.x, target.y - stylePos.y), target.z - stylePos.z,
                    athlete.legSynthesis.reachError[side], o.secondsToLiftoff[side], o.secondsToTouchdown[side]);
                (void)mode;
            }
            std::printf("\n");
        }
        // Medidas.
        // O joelho no modo padrao (no corpo e no alvo).
        if (thighs[0] < bodyState.links.size() && thighs[1] < bodyState.links.size()) {
            const bool animatedMode = legacy.bodyMode == LegacyBodyMode3D::Animated;
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
                            ? "FLSP"[static_cast<int>(athlete.footTargets[side].mode)] : (o.footPlanted[side] ? 'A' : 'B'),
                        o.footPlanted[side] ? "*" : " ", flexion, target, rate, targetRate, o.secondsToLiftoff[side],
                        o.secondsToTouchdown[side], reachNow, reachTarget, reachStyle, behind);
                }
                kneeRateBefore[side] = kneeBefore[side] >= 0.0f ? rate : 0.0f;
                kneeBefore[side] = flexion;
                kneeTargetBefore[side] = target;
            }
        }
        // Manejo novo: as pernas nunca se cruzam no gesto (o pe que joga do
        // outro lado do de apoio, com os dois lado a lado).
        if (settings.newBallHandling && athlete.ballPlay.gesture().active() && leftFoot < bodyState.links.size()
            && rightFoot < bodyState.links.size()) {
            const int kick = athlete.ballPlay.gesture().side();
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
        if (legacy.bodyMode != previousMode) {
            if (legacy.bodyMode == LegacyBodyMode3D::Physical && metrics.trail.find("volta>animado") != std::string::npos)
                ++metrics.relapses;
            ++metrics.modeSwitches;
            metrics.trail += std::string(">") + modeOf();
            if (dump != nullptr) std::fprintf(dump, "E %s\n", modeOf());
        }
        previousMode = legacy.bodyMode;
        (legacy.bodyMode == LegacyBodyMode3D::Animated ? metrics.animatedSeconds
            : legacy.bodyMode == LegacyBodyMode3D::Physical ? metrics.physicalSeconds
            : legacy.bodyMode == LegacyBodyMode3D::Active ? metrics.activeSeconds : metrics.returningSeconds) += Dt;
        metrics.assistImpulse = athlete.activeOutput.assistImpulseTotal;
        for (std::size_t i = 0; i < bodyState.links.size() && i < profile.links.size(); ++i) {
            const PhysicsBodyState3D& link = bodyState.links[i];
            const std::string& id = profile.links[i].id;
            const Vec3 acceleration = (link.linearVelocity - previousVelocities[i]) * (1.0f / Dt);
            if (t > 0.35f) {
                const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                    || id.find("Foot") != std::string::npos;
                if (std::getenv("XPICOTODOS") && acceleration.length() > std::atof(std::getenv("XPICOTODOS")))
                    std::printf("  t %.3f %s acel %.0f salto %.3f\n", t, id.c_str(), acceleration.length(),
                        (link.position - (previousLinks[i].position + link.linearVelocity * Dt)).length());
                if (std::getenv("XPICO") && leg && acceleration.length() > std::atof(std::getenv("XPICO"))) {
                    const auto mode = [&](std::size_t side) { return "FLSP"[static_cast<int>(athlete.footTargets[side].mode)]; };
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
                    && (legacy.bodyMode == LegacyBodyMode3D::Animated || legacy.bodyMode == LegacyBodyMode3D::Returning)) {
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
        previousLinks = bodyState.links;
        metrics.lowestPelvis = std::min(metrics.lowestPelvis, bodyState.links.front().position.z);
        if (bodyState.links.front().position.z < 0.5f) metrics.fell = true;
        if (scenario.ball) metrics.ballSpeedAfter = std::max(metrics.ballSpeedAfter, ballState.linearVelocity.length());
        if (dump != nullptr && step % every == 0) {
            dumpFrame(dump, profile, bodyState, t, static_cast<int>(legacy.bodyMode));
            if (scenario.ball)
                std::fprintf(dump, "K %.4f %.4f %.4f %.3f %.3f %.3f\n", ballState.position.x, ballState.position.y,
                    ballState.position.z, ballState.linearVelocity.x, ballState.linearVelocity.y,
                    ballState.linearVelocity.z);
        }
    }
    metrics.finalMode = legacy.bodyMode;
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
    return metrics;
}

void printMetrics(const Scenario& scenario, const Metrics& metrics, bool footsteps) {
    std::printf("%-18s | %5.2f / %5.2f / %5.2f | %7.1f / %7.1f m/s2 | %5.3f m | %d; %.1f/%.1f/%.1f/%.1f; %.0f | %s%d pedidos, %d toques, bola max %.1f m/s\n",
        scenario.name.c_str(), percentile(metrics.footSlide, 0.5f), percentile(metrics.footSlide, 0.95f),
        percentile(metrics.footSlide, 1.0f), metrics.peakUpperAcceleration, metrics.peakLegAcceleration,
        metrics.worstPoseJump, metrics.modeSwitches, metrics.animatedSeconds, metrics.activeSeconds,
        metrics.physicalSeconds, metrics.returningSeconds, metrics.assistImpulse, metrics.fell ? "CAIU, " : "",
        metrics.requests, metrics.touches, metrics.ballSpeedAfter);
    std::printf("%-18s   andou %.2f m, recaidas %d, fim %s (%s)\n", "", metrics.travel, metrics.relapses,
        modeName(metrics.finalMode), metrics.trail.c_str());
    std::printf("%-18s   joelho esticando max %.0f graus/s (alvo %.0f), acel max %.0f graus/s2\n", "",
        metrics.peakKneeExtension, metrics.peakKneeExtensionTarget, metrics.peakKneeAcceleration);
    std::printf("%-18s   apoio %5.2f / %5.2f / %5.2f m/s%s", "", percentile(metrics.plantedSlide, 0.5f),
        percentile(metrics.plantedSlide, 0.95f), percentile(metrics.plantedSlide, 1.0f), footsteps ? "" : "\n");
    if (!metrics.outcomes.empty())
        std::printf("%-18s   manejo novo: %s | atraso ate o gesto max %.3f s\n", "", metrics.outcomes.c_str(),
            metrics.worstLatency);
    if (metrics.capsuleUp > 0.0f || metrics.pelvisUp > 0.0f)
        std::printf("%-18s   subida max: capsula %.2f m/s, pelve %.2f m/s, pelve sobre a capsula %.2f m\n", "",
            metrics.capsuleUp, metrics.pelvisUp, metrics.pelvisOverCapsule);
    if (metrics.riseAt >= 0.0f)
        std::printf("%-18s   levantar: comeca %.2f s (peito %+.2f pelve %+.2f, %s), de pe em %.2f s\n", "", metrics.riseAt,
            metrics.riseChest, metrics.risePelvis, metrics.riseSide == 1 ? "de costas" : metrics.riseSide == 2 ? "de brucos" : "?",
            metrics.risenAt >= 0.0f ? metrics.risenAt - metrics.riseAt : -1.0f);
    if (metrics.forceTime >= 0.0f)
        std::printf("%-18s   v2: pedido %.3f aceite %.3f forca %.3f (planejada %.3f, contato estimado %.3f)%s, perna volta %.3f s, cruzou %d\n",
            "", metrics.requestTime, metrics.acceptTime, metrics.forceTime, metrics.acceptTime + metrics.forceAt,
            metrics.estimatedContact, metrics.imagined ? " imaginada" : "", metrics.legReturn, metrics.crossings);
    if (footsteps)
        std::printf(" | saida do apoio max %.3f m | alcance max %.3f m | arranca %d, acomoda %d, antes %d, no ar %d\n",
            metrics.worstLiftCorrection, metrics.worstReach, metrics.startSteps, metrics.settleSteps, metrics.earlyLifts,
            metrics.catchSteps);
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
                && (m.finalMode == LegacyBodyMode3D::Animated || m.finalMode == LegacyBodyMode3D::Returning),
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
        expect(m.finalMode == LegacyBodyMode3D::Animated && m.relapses == 0 && m.travel >= travel, what, m);
    }
    for (const char* name : { "encostao_andando", "encostao_correndo", "caixa_media", "caixa_correndo" }) {
        const Metrics m = run(name, settings, false);
        char what[192];
        std::snprintf(what, sizeof(what), "%s: nao derruba, volta ao padrao (pelve min %.2f m)", name, m.lowestPelvis);
        expect(!m.fell && m.relapses == 0
                && (m.finalMode == LegacyBodyMode3D::Animated || m.finalMode == LegacyBodyMode3D::Returning),
            what, m);
    }
    {
        const Metrics m = run("caixa_leve", activeOn, false);
        expect(has(m, "ativo") && !has(m, "fisico"), "Ativo ligado, caixa leve: segura sem fisico pleno", m);
        expect(m.finalMode == LegacyBodyMode3D::Animated, "Ativo ligado, caixa leve: volta ao padrao", m);
    }
    // O terreno (plano v2, fase E): degrau baixo, escada e rampa sem ir ao
    // fisico e sem o lancamento (a pelve longe acima da capsula), andando ate o
    // fim; o degrau alto (35 cm, parede para a capsula) pode ir ao fisico, sem
    // lancamento; encostado na parede volta ao padrao. Sem e com as pegadas.
    AthleteRuntimeSettings3D terrainSteps;
    terrainSteps.footstepLocomotion = true;
    const AthleteRuntimeSettings3D& terrainFlat = settings;
    for (const AthleteRuntimeSettings3D* config : { &terrainFlat, static_cast<const AthleteRuntimeSettings3D*>(&terrainSteps) }) {
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
        expect(m.finalMode != LegacyBodyMode3D::Physical, "terreno, parede_corre: encostado na parede volta ao padrao", m);
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
        expect(has(m, "ponte") && (m.finalMode == LegacyBodyMode3D::Animated || m.finalMode == LegacyBodyMode3D::Returning)
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
        expect((rose || (!m.fell && m.riseAt < 0.0f)) && has(m, "ponte") && m.finalMode != LegacyBodyMode3D::Physical,
            what, m);
    }
    // Locomocao por pegadas (Etapa 3): o pe apoiado nao escorrega, sem
    // estalo, e as acoes com bola continuam saindo.
    AthleteRuntimeSettings3D footsteps;
    footsteps.footstepLocomotion = true;
    for (const char* name : { "arranca", "sprint", "inverte", "lados", "corte", "para_sprint", "diagonal", "costas" }) {
        const Metrics m = run(name, footsteps, false);
        char what[128];
        std::snprintf(what, sizeof(what), "pegadas, %s: o pe apoiado nao escorrega (p95 %.2f m/s)", name,
            percentile(m.plantedSlide, 0.95f));
        expect(percentile(m.plantedSlide, 0.95f) < 0.4f && percentile(m.plantedSlide, 1.0f) < 1.0f, what, m);
        std::snprintf(what, sizeof(what), "pegadas, %s: sem salto de pose (%.3f m)", name, m.worstPoseJump);
        expect(m.worstPoseJump < 0.03f, what, m);
    }
    // O joelho no fim da pisada (plano v2, fase B): com as pegadas, a perna
    // que sai do chao estica no ritmo do clipe - nada de travar reta num tick
    // (antes: 2788 graus/s correndo contra 927 do clipe).
    for (const char* name : { "anda", "trota", "arranca", "sprint" }) {
        const Metrics clip = run(name, settings, false);
        const Metrics m = run(name, footsteps, false);
        char what[192];
        std::snprintf(what, sizeof(what), "pegadas, %s: o joelho estica como no clipe (%.0f contra %.0f graus/s)", name,
            m.peakKneeExtension, clip.peakKneeExtension);
        expect(m.peakKneeExtension < 1.4f * clip.peakKneeExtension + 50.0f, what, m);
    }
    {
        const Metrics m = run("zigue", footsteps, false);
        expect(percentile(m.plantedSlide, 0.95f) < 1.5f && m.worstPoseJump < 0.03f,
            "pegadas, zigue-zague: pouco deslize, sem salto de pose", m);
    }
    {
        const Metrics m = run("parado", footsteps, false);
        expect(m.settleSteps == 0 && percentile(m.plantedSlide, 1.0f) < 0.01f, "pegadas, parado: quieto", m);
    }
    for (const char* name : { "toque_parado", "chute_parado", "toque_corre" }) {
        const Metrics m = run(name, footsteps, false);
        char what[96];
        std::snprintf(what, sizeof(what), "pegadas, %s: a bola sai", name);
        expect(m.touches == 1 && m.ballSpeedAfter > 1.0f, what, m);
    }
    // Manejo v2 (plano v2, fase C): com a bola na area, toda acao sai; as
    // pernas nunca se cruzam; sem tranco; mudando o movimento, a perna sai do
    // gesto em ate ~0,15 s; a forca na fracao da regua; e a capsula faz o
    // mesmo caminho que sem o pedido (o manejo nao mexe no movimento). Sem e
    // com as pegadas.
    AthleteRuntimeSettings3D handling;
    handling.newBallHandling = true;
    AthleteRuntimeSettings3D handlingSteps = handling;
    handlingSteps.footstepLocomotion = true;
    {
        int total = 0, failed = 0;
        for (const Scenario& scenario : scenarios()) {
            const std::string& name = scenario.name;
            const bool old = name == "toque_parado" || name == "toque_lado" || name == "chute_parado" || name == "chute_lado"
                || name == "bola_vindo" || name == "bola_alta" || name == "bola_quadril" || name == "atras_fora"
                || name == "atras_tras" || name == "toque_corre" || name == "chute_corre" || name == "chute_corre_cedo"
                || name == "corte_corre" || name.rfind("bola_", 0) == 0;
            if (name.rfind("v2_", 0) != 0 && !old) continue;
            for (const AthleteRuntimeSettings3D* config : { &handling, &handlingSteps }) {
                const Metrics m = runScenario(context, scenario, *config, false, nullptr, 1);
                ++total;
                const bool steps = config->footstepLocomotion;
                const bool shot = name.find("chute") != std::string::npos;
                std::vector<std::string> problems;
                if (m.requests != 1 || m.touches != 1 || m.outcomes != "tocou")
                    problems.push_back("acao (" + m.outcomes + ")");
                if (m.crossings != 0) problems.push_back("cruzou " + std::to_string(m.crossings));
                if (m.worstPoseJump >= 0.03f) problems.push_back("salto " + std::to_string(m.worstPoseJump));
                if (m.peakUpperAcceleration >= 320.0f) problems.push_back("tronco " + std::to_string(m.peakUpperAcceleration));
                if (m.peakLegAcceleration >= (shot ? 950.0f : 1050.0f)) problems.push_back("pernas " + std::to_string(m.peakLegAcceleration));
                if (steps && percentile(m.plantedSlide, 1.0f) >= 0.6f)
                    problems.push_back("apoio escorrega " + std::to_string(percentile(m.plantedSlide, 1.0f)));
                if (scenario.changeAt >= 0.0f && (m.legReturn < 0.0f || m.legReturn > 0.15f + 2.0f * Dt))
                    problems.push_back("perna volta " + std::to_string(m.legReturn));
                if (!problems.empty()) {
                    ++failed;
                    std::string text;
                    for (const std::string& problem : problems) text += (text.empty() ? "" : ", ") + problem;
                    std::printf("  FALHOU manejo v2%s, %s: %s\n", steps ? " (pegadas)" : "", name.c_str(), text.c_str());
                }
            }
        }
        char what[160];
        std::snprintf(what, sizeof(what), "manejo v2: %d de %d cenarios (sem e com pegadas) com acao, sem cruzar, sem tranco",
            total - failed, total);
        Metrics none;
        none.trail = "manejo v2";
        expect(failed == 0, what, none);
    }
    // A forca na fracao da regua do tempo ate o contato estimado (a bola indo
    // sair da area pode adiantar).
    for (const float timing : { 0.0f, 0.5f, 1.0f }) {
        AthleteRuntimeSettings3D timed = handling;
        timed.ballContactTiming = timing;
        for (const char* name : { "v2_parado_35_000", "v2_parado_35_270", "v2_anda_35_045", "v2_corre_35_000", "v2_chute_parado_000" }) {
            const Metrics m = run(name, timed, false);
            const float planned = m.acceptTime + timing * m.estimatedContact;
            char what[192];
            std::snprintf(what, sizeof(what), "manejo v2, regua %.1f, %s: a forca em %.3f (planejada %.3f)", timing, name,
                m.forceTime, planned);
            expect(m.forceTime >= 0.0f && m.forceTime <= planned + 1.5f * Dt
                    && (m.forceTime >= planned - 1.5f * Dt || m.forceAt < timing * m.estimatedContact - 1e-3f),
                what, m);
        }
    }
    // A capsula com o pedido faz o mesmo caminho que sem ele.
    for (const char* name : { "v2_anda_35_045", "v2_anda_65_270", "v2_corre_35_000", "v2_corre_35_270", "v2_corre_90_180",
             "v2_fase_3", "v2_chute_corre_000", "v2_chute_corre_090" }) {
        for (const Scenario& scenario : scenarios()) {
            if (scenario.name != name) continue;
            const Metrics with = runScenario(context, scenario, handling, false, nullptr, 1);
            const Metrics without = runScenario(context, scenario, handling, false, nullptr, 1, true);
            float worst = 0.0f;
            for (std::size_t i = 0; i < with.capsulePath.size() && i < without.capsulePath.size(); ++i)
                worst = std::max(worst, std::hypot(with.capsulePath[i].x - without.capsulePath[i].x,
                                            with.capsulePath[i].y - without.capsulePath[i].y));
            char what[160];
            std::snprintf(what, sizeof(what), "manejo v2, %s: a capsula faz o mesmo caminho (desvio max %.4f m)", name, worst);
            expect(worst < 0.01f, what, with);
        }
    }
    std::printf(failures == 0 ? "atleta: tudo certo\n" : "atleta: %d falha(s)\n", failures);
    return failures == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    std::string only, dumpPath;
    int every = 3;
    bool startPhysical = false;
    bool activeBody = AthleteRuntimeSettings3D {}.activeBody;
    bool footsteps = false;
    bool balanceSteps = AthleteRuntimeSettings3D {}.activeStanceSettings.balanceSteps;
    bool checks = false;
    float response = AthleteRuntimeSettings3D {}.capsuleResponsePercent;
    bool newBall = false;
    bool noRequests = false;
    bool bridge = AthleteRuntimeSettings3D {}.physicalBridge;
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
    settings.physicalBridge = bridge;
    ProbeContext context;
    context.profile = &profile;
    context.gaitSpeeds = gaitSpeeds;
    context.avatar = avatar;
    context.runtimeAssets.profile = &profile;
    context.runtimeAssets.clips = &clips;
    context.runtimeAssets.defaultClips = &defaultClips;
    context.runtimeAssets.avatarSettings = avatar;
    if (checks) return runChecks(context);

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
