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
// Sem --cenario roda todos. O dump e o formato do MatterBallPreview: os
// desenhos saem de tools/analysis/ball_touch_preview.py --ler arquivo.
#include "Engine/Animation/RagdollVisualPose3D.hpp"
#include "Engine/Character/BallTouchPreview3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Character/RagdollPoseKinematics3D.hpp"

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
    float cameraYaw = 0.0f;
    bool jump = false;
    // A velocidade do pulo (m/s; a do laboratorio e 4,6, carregado 5,3).
    float jumpSpeed = 4.6f;
    // Manejo de bola: o pedido (borda), a forca e a direcao das teclas.
    BallActionKind3D request = BallActionKind3D::None;
    float power = 0.5f;
    // A direcao do pedido em relacao a camera (rad), sem andar (o teste do
    // gesto parado nas oito direcoes); NAN usa as teclas/o olhar.
    float touchYaw = NAN;
    // Chute: a mira em relacao a camera (rad), a altura e o efeito (-1..1).
    float aimYaw = 0.0f;
    float lift = 0.0f;
    float curve = 0.0f;
    // Carrinho (a borda do agachar correndo).
    bool tackle = false;
    // Carga do pulo parado (0..1): o agachar de preparo.
    float jumpCrouch = 0.0f;
};

struct Scenario {
    const char* name;
    float seconds;
    std::function<Keys(float)> keys;
    // A bola (no inicio, em relacao a capsula; sem bola: nenhuma).
    bool ball = false;
    Vec3 ballStart;
    // Velocidade inicial da bola e se ela fica parada no ar ate o toque (o
    // teste da bola alta).
    Vec3 ballVelocity {};
    bool ballFloat = false;
    // Pedido automatico (o jogador aperta quando a bola esta perto): a menos
    // disto (m, da pelve no plano), no maximo um a cada autoInterval s; a
    // direcao do toque em relacao a camera (NAN: a das teclas).
    float autoDistance = 0.0f;
    BallActionKind3D autoKind = BallActionKind3D::Dribble;
    float autoPower = 0.35f;
    float autoInterval = 0.45f;
    float autoYaw = NAN;
    float autoFrom = 0.0f;
    int autoCount = 1000;
    // (A garantia, 06/10 noite: so com a bola a pelo menos isto - a borda da
    // area - e, com autoBehind, atras do corpo andando.)
    float autoMinimum = 0.0f;
    bool autoBehind = false;
    // (A forca, 06/10 noite: a bola levada junto, nos pes - a esta distancia
    // da capsula - ate o primeiro pedido; e a medida depois do toque.)
    bool ballCarried = false;
    Vec3 carriedOffset { 0.45f, -0.10f, 0.0f };
};

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
            k.jumpSpeed = 2.5f;
            return k; } },
        { "pulo alto", 3.5f, [](float t) {
            Keys k;
            if (t > 0.3f && t < 3.0f) k.forward = 1.0f;
            k.jump = t > 1.2f && t < 1.22f;
            k.jumpSpeed = 5.3f;
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
            k.jumpSpeed = 5.3f;
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
        { "toqueparado", 2.5f, [](float t) {
            Keys k;
            if (pulse(t, 0.6f)) { k.request = BallActionKind3D::Dribble; k.power = 0.3f; }
            return k; }, true, { 0.55f, 0.10f, 0.11f } },
        { "toquecorre", 4.0f, [](float t) {
            Keys k;
            if (t > 0.3f) k.forward = 1.0f;
            for (float at : { 0.9f, 1.6f, 2.3f, 3.0f })
                if (pulse(t, at)) { k.request = BallActionKind3D::Dribble; k.power = 0.3f; }
            return k; }, true, { 1.6f, 0.05f, 0.11f } },
        { "chuteparado", 2.5f, [](float t) {
            Keys k;
            if (pulse(t, 0.6f)) { k.request = BallActionKind3D::Shot; k.power = 0.8f; }
            return k; }, true, { 0.55f, 0.10f, 0.11f } },
        { "chutecorre", 3.0f, [](float t) {
            Keys k;
            if (t > 0.3f) k.forward = 1.0f;
            k.sprint = t > 0.3f;
            if (pulse(t, 0.6f)) { k.request = BallActionKind3D::Shot; k.power = 1.0f; }
            return k; }, true, { 3.5f, -0.10f, 0.11f } },
    };
}

// Os gestos com a bola para a revisao visual (rework de 06/10): parado, nas
// oito direcoes, passe, chutes pela forca, sola, bola alta, bola longe e
// andando. O pedido em 0,6 s.
std::vector<Scenario> ballScenarios() {
    std::vector<Scenario> list;
    static const char* names[8] = { "frente", "frente_esq", "esq", "tras_esq", "tras", "tras_dir", "dir", "frente_dir" };
    static std::vector<std::string> keep;
    keep.reserve(512);
    const auto add = [&](std::string name, float seconds, std::function<Keys(float)> keys, Vec3 ball,
        Vec3 velocity = {}, bool floating = false) {
        keep.push_back(name);
        Scenario sc { keep.back().c_str(), seconds, keys, true, ball };
        sc.ballVelocity = velocity;
        sc.ballFloat = floating;
        list.push_back(sc);
    };
    for (int k = 0; k < 8; ++k) {
        const float yaw = k * Pi * 0.25f;
        add(std::string("toque_") + names[k], 1.8f, [yaw](float t) {
            Keys keys;
            if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = yaw; }
            return keys; }, { 0.40f, -0.12f, 0.11f });
    }
    for (int k = 0; k < 8; ++k) {
        const float yaw = k * Pi * 0.25f;
        add(std::string("toqueesq_") + names[k], 1.8f, [yaw](float t) {
            Keys keys;
            if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = yaw; }
            return keys; }, { 0.40f, 0.12f, 0.11f });
    }
    add("passe", 2.0f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.85f; keys.touchYaw = 0.0f; }
        return keys; }, { 0.38f, -0.14f, 0.11f });
    add("passe_esq", 2.0f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.85f; keys.touchYaw = 0.8f; }
        return keys; }, { 0.38f, -0.14f, 0.11f });
    for (float power : { 0.3f, 0.6f, 1.0f }) {
        add("chute_" + std::to_string(static_cast<int>(power * 100.0f)), 2.2f, [power](float t) {
            Keys keys;
            if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = power; }
            return keys; }, { 0.42f, -0.16f, 0.11f });
    }
    add("chute_diag", 2.2f, [](float t) {
        Keys keys;
        keys.aimYaw = 0.5f;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        return keys; }, { 0.42f, -0.16f, 0.11f });
    add("chute_esq", 2.2f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        return keys; }, { 0.42f, 0.16f, 0.11f });
    // Chutes com efeito (o lado de dentro curvando para a esquerda com o
    // direito, a trivela para a direita) e alto/rasteiro.
    for (const auto& [name, lift, curve] : std::vector<std::tuple<const char*, float, float>> {
            { "chute_curva", 0.0f, -0.7f }, { "chute_trivela", 0.0f, 0.7f }, { "chute_alto", 0.8f, 0.0f },
            { "chute_rasteiro", -0.8f, 0.0f } }) {
        add(name, 2.2f, [lift, curve](float t) {
            Keys keys;
            keys.lift = lift;
            keys.curve = curve;
            if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.85f; }
            return keys; }, { 0.42f, -0.16f, 0.11f });
    }
    add("bola_longe", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = 0.0f; }
        return keys; }, { 0.78f, -0.25f, 0.11f });
    add("alta_canela", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = 0.0f; }
        return keys; }, { 0.40f, -0.14f, 0.30f }, {}, true);
    add("alta_joelho", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = 0.0f; }
        return keys; }, { 0.42f, -0.14f, 0.50f }, {}, true);
    add("alta_cintura", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = 0.0f; }
        return keys; }, { 0.45f, -0.16f, 0.85f }, {}, true);
    add("voleio", 2.2f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        return keys; }, { 0.45f, -0.16f, 0.55f }, {}, true);
    const auto run = [&](std::string name, float seconds, bool sprint, float yaw, BallActionKind3D kind, float power,
        float distance, float interval, Vec3 ball, int count = 1000, float from = 0.3f) {
        add(name, seconds, [sprint](float t) {
            Keys keys;
            if (t > 0.3f) { keys.forward = 1.0f; keys.sprint = sprint; }
            return keys; }, ball);
        Scenario& sc = list.back();
        sc.autoDistance = distance;
        sc.autoKind = kind;
        sc.autoPower = power;
        sc.autoInterval = interval;
        sc.autoYaw = yaw;
        sc.autoCount = count;
        sc.autoFrom = from;
    };
    run("corre_toques", 5.0f, false, 0.0f, BallActionKind3D::Dribble, 0.2f, 0.85f, 0.5f, { 1.2f, -0.10f, 0.11f });
    run("sprint_toques", 5.0f, true, 0.0f, BallActionKind3D::Dribble, 0.2f, 0.9f, 0.45f, { 1.5f, -0.10f, 0.11f });
    run("corre_chute", 3.0f, false, NAN, BallActionKind3D::Shot, 0.9f, 0.9f, 1.0f, { 2.2f, -0.10f, 0.11f }, 1);
    run("sprint_chute", 3.0f, true, NAN, BallActionKind3D::Shot, 0.9f, 0.9f, 1.0f, { 3.0f, -0.10f, 0.11f }, 1);
    run("corre_esq", 3.0f, false, Pi * 0.5f, BallActionKind3D::Dribble, 0.35f, 0.85f, 1.0f, { 2.0f, -0.10f, 0.11f }, 1);
    run("corre_dir", 3.0f, false, -Pi * 0.5f, BallActionKind3D::Dribble, 0.35f, 0.85f, 1.0f, { 2.0f, -0.10f, 0.11f }, 1);
    run("corre_diag", 3.0f, false, Pi * 0.25f, BallActionKind3D::Dribble, 0.35f, 0.85f, 1.0f, { 2.0f, -0.10f, 0.11f }, 1);
    run("corre_puxa", 3.0f, false, Pi, BallActionKind3D::Dribble, 0.35f, 0.85f, 1.0f, { 2.0f, -0.10f, 0.11f }, 1);
    run("corre_passe", 3.0f, false, 0.0f, BallActionKind3D::Dribble, 0.85f, 0.85f, 1.0f, { 2.0f, -0.10f, 0.11f }, 1);
    // De lado (strafe para a esquerda) e de costas, tocando no rumo do
    // movimento.
    add("lado_toques", 4.0f, [](float t) {
        Keys keys;
        if (t > 0.3f) keys.left = 1.0f;
        return keys; }, { 0.05f, 1.0f, 0.11f });
    {
        Scenario& sc = list.back();
        sc.autoDistance = 0.85f; sc.autoKind = BallActionKind3D::Dribble; sc.autoPower = 0.2f;
        sc.autoInterval = 0.5f; sc.autoYaw = Pi * 0.5f; sc.autoFrom = 0.3f;
    }
    add("costas_toques", 4.0f, [](float t) {
        Keys keys;
        if (t > 0.3f) keys.forward = -1.0f;
        return keys; }, { -1.0f, -0.05f, 0.11f });
    {
        Scenario& sc = list.back();
        sc.autoDistance = 0.85f; sc.autoKind = BallActionKind3D::Dribble; sc.autoPower = 0.2f;
        sc.autoInterval = 0.5f; sc.autoYaw = Pi; sc.autoFrom = 0.3f;
    }
    // Arrancando com a bola (o caso do laboratorio): parado, ele aperta a
    // direcao e o toque juntos - o corpo comeca a andar e o toque sai.
    for (int k = 0; k < 4; ++k) {
        static const char* names4[4] = { "arranca_frente", "arranca_esq", "arranca_dir", "arranca_tras" };
        const float fx = k == 0 ? 1.0f : k == 3 ? -1.0f : 0.0f;
        const float fy = k == 1 ? 1.0f : k == 2 ? -1.0f : 0.0f;
        add(names4[k], 2.0f, [fx, fy](float t) {
            Keys keys;
            if (t >= 0.6f) { keys.forward = fx; keys.left = fy; }
            if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.4f; }
            return keys; }, { 0.40f, -0.12f, 0.11f });
    }
    // Recepcao: parado, a bola vindo de frente a 4 e a 8 m/s; o toque sai
    // quando ela entra no alcance (para o lado e para a frente).
    for (float speed : { 4.0f, 8.0f }) {
        for (int k = 0; k < 2; ++k) {
            const float yaw = k == 0 ? Pi * 0.5f : 0.0f;
            add(std::string("recebe_") + std::to_string(static_cast<int>(speed)) + (k == 0 ? "_lado" : "_frente"), 2.0f,
                [](float) { return Keys {}; }, { 3.0f, -0.12f, 0.11f }, { -speed, 0.0f, 0.0f });
            Scenario& sc = list.back();
            sc.autoDistance = 0.9f; sc.autoKind = BallActionKind3D::Dribble; sc.autoPower = 0.3f;
            sc.autoInterval = 1.0f; sc.autoYaw = yaw; sc.autoCount = 1; sc.autoFrom = 0.0f;
        }
    }
    // A bola atras do corpo: calcanhar (para tras) e a sola trazendo-a (para
    // a frente e para o lado).
    add("calcanhar", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = Pi; }
        return keys; }, { -0.30f, -0.12f, 0.11f });
    add("sola_frente", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = 0.0f; }
        return keys; }, { -0.25f, -0.15f, 0.11f });
    add("sola_lado", 1.8f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.35f; keys.touchYaw = Pi * 0.5f; }
        return keys; }, { -0.25f, -0.15f, 0.11f });
    // V3 (06/10 noite): a bola atras tocada para o lado (a perna para tras e
    // para fora, Pro Soccer Online; e para dentro, a sola curta), um pouco
    // atras (o gesto de frente com ela imaginada a frente), a antecipacao (o
    // pedido antes da bola chegar: guardado e sincronizado) e de lado com a
    // bola do lado contrario (as pernas nao cruzam).
    for (const auto& [name, ball, yaw] : std::vector<std::tuple<const char*, Vec3, float>> {
            { "tras_fora_dir", { -0.26f, -0.30f, 0.11f }, -Pi * 0.5f },
            { "tras_fora_longe", { -0.42f, -0.40f, 0.11f }, -Pi * 0.5f },
            { "tras_fora_longe_esq", { -0.42f, 0.40f, 0.11f }, Pi * 0.5f },
            { "tras_fora_esq", { -0.26f, 0.30f, 0.11f }, Pi * 0.5f },
            { "tras_fora_diag", { -0.30f, -0.26f, 0.11f }, -Pi * 0.75f },
            { "tras_dentro", { -0.26f, -0.22f, 0.11f }, Pi * 0.5f },
            { "atras_leve", { -0.14f, -0.14f, 0.11f }, 0.0f },
            { "atras_leve_esq", { -0.18f, 0.14f, 0.11f }, Pi * 0.25f } }) {
        const float touchYaw = yaw;
        add(name, 1.8f, [touchYaw](float t) {
            Keys keys;
            if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.4f; keys.touchYaw = touchYaw; }
            return keys; }, ball);
    }
    add("atras_chute", 2.2f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        return keys; }, { -0.12f, -0.14f, 0.11f });
    // Antecipacao: parado, a bola vindo de longe; o pedido sai cedo (bola a
    // 2,2 m) e fica guardado ate o gesto poder encontra-la.
    add("antecipa_frente", 2.0f, [](float t) {
        Keys keys;
        if (pulse(t, 0.3f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.3f; keys.touchYaw = Pi * 0.5f; }
        return keys; }, { 3.4f, -0.12f, 0.11f }, { -5.0f, 0.0f, 0.0f });
    add("antecipa_lado", 2.0f, [](float t) {
        Keys keys;
        if (pulse(t, 0.3f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.3f; keys.touchYaw = 0.0f; }
        return keys; }, { 0.35f, -2.6f, 0.11f }, { 0.0f, 4.5f, 0.0f });
    add("antecipa_chute", 2.2f, [](float t) {
        Keys keys;
        if (pulse(t, 0.3f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.9f; }
        return keys; }, { 2.6f, -0.16f, 0.11f }, { -3.5f, 0.0f, 0.0f });
    // Correndo ao encontro da bola parada: o pedido sai com ela a 1,4 m (fora
    // da area) - o toque sai no encontro.
    add("corre_encontro", 2.4f, [](float t) {
        Keys keys;
        if (t > 0.2f) keys.forward = 1.0f;
        if (pulse(t, 0.75f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.25f; keys.touchYaw = 0.0f; }
        return keys; }, { 3.0f, -0.10f, 0.11f });
    add("corre_encontro_chute", 2.4f, [](float t) {
        Keys keys;
        if (t > 0.2f) keys.forward = 1.0f;
        if (pulse(t, 0.75f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.9f; }
        return keys; }, { 3.0f, -0.10f, 0.11f });
    // De lado para a direita com a bola do lado esquerdo a frente (o pe
    // direito cruzaria): toques no rumo do movimento.
    add("lado_cruzado", 3.0f, [](float t) {
        Keys keys;
        if (t > 0.3f) keys.left = -1.0f;
        return keys; }, { 0.30f, -0.9f, 0.11f });
    {
        Scenario& sc = list.back();
        sc.autoDistance = 0.85f; sc.autoKind = BallActionKind3D::Dribble; sc.autoPower = 0.2f;
        sc.autoInterval = 0.5f; sc.autoYaw = -Pi * 0.5f; sc.autoFrom = 0.3f;
    }
    add("lado_dir_toques", 3.0f, [](float t) {
        Keys keys;
        if (t > 0.3f) keys.left = -1.0f;
        return keys; }, { 0.05f, -1.0f, 0.11f });
    {
        Scenario& sc = list.back();
        sc.autoDistance = 0.85f; sc.autoKind = BallActionKind3D::Dribble; sc.autoPower = 0.2f;
        sc.autoInterval = 0.5f; sc.autoYaw = -Pi * 0.5f; sc.autoFrom = 0.3f;
    }
    // Diagonal para a frente e a direita com a bola a frente e a esquerda (a
    // segunda imagem do usuario).
    add("diag_toques", 3.0f, [](float t) {
        Keys keys;
        if (t > 0.3f) { keys.forward = 0.7071f; keys.left = -0.7071f; }
        return keys; }, { 0.9f, -0.5f, 0.11f });
    {
        Scenario& sc = list.back();
        sc.autoDistance = 0.85f; sc.autoKind = BallActionKind3D::Dribble; sc.autoPower = 0.2f;
        sc.autoInterval = 0.5f; sc.autoYaw = -Pi * 0.25f; sc.autoFrom = 0.3f;
    }
    add("fora_alcance", 1.5f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        return keys; }, { 1.4f, 0.0f, 0.11f });
    // A garantia (06/10 noite, pedido do usuario: "estou correndo sprint e a
    // bola esta dentro da area mas na beira, atras, e eu tento manejar, ele
    // nao deixa"): o pedido com a bola na borda da area, atras do corpo
    // correndo (ele passou por ela) - o toque sai sempre (o gesto na bola
    // imaginada a frente).
    const auto edge = [&](std::string name, bool sprint, float side, BallActionKind3D kind, float power, float yaw,
        float leftKey = 0.0f) {
        add(name, 3.2f, [sprint, leftKey](float t) {
            Keys keys;
            if (t > 0.3f) { keys.forward = leftKey == 0.0f ? 1.0f : 0.0f; keys.left = leftKey; keys.sprint = sprint; }
            return keys; }, leftKey == 0.0f ? Vec3 { 5.0f, side, 0.11f } : Vec3 { side, 3.0f * leftKey, 0.11f });
        Scenario& sc = list.back();
        sc.autoDistance = 0.95f; sc.autoMinimum = 0.80f; sc.autoBehind = true; sc.autoKind = kind;
        sc.autoPower = power; sc.autoYaw = yaw; sc.autoCount = 1; sc.autoFrom = 0.3f;
    };
    edge("borda_sprint", true, -0.20f, BallActionKind3D::Dribble, 0.3f, 0.0f);
    edge("borda_sprint_esq", true, 0.25f, BallActionKind3D::Dribble, 0.3f, 0.0f);
    edge("borda_sprint_lado", true, -0.20f, BallActionKind3D::Dribble, 0.4f, -Pi * 0.5f);
    edge("borda_sprint_puxa", true, -0.20f, BallActionKind3D::Dribble, 0.4f, Pi);
    edge("borda_sprint_chute", true, -0.20f, BallActionKind3D::Shot, 0.9f, NAN);
    edge("borda_corre", false, -0.20f, BallActionKind3D::Dribble, 0.3f, 0.0f);
    edge("borda_corre_chute", false, 0.20f, BallActionKind3D::Shot, 0.8f, NAN);
    // De lado (para a esquerda) com a bola ficando para tras do movimento.
    edge("borda_lado", false, 0.25f, BallActionKind3D::Dribble, 0.3f, Pi * 0.5f, 1.0f);
    // Clique duplo parado (o segundo durante o primeiro gesto, para outro
    // lado) e pedidos seguidos correndo rapido (no intervalo entre pedidos):
    // cada um tem o seu toque.
    add("duplo_clique", 2.2f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.3f; keys.touchYaw = 0.0f; }
        if (pulse(t, 0.7f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.3f; keys.touchYaw = Pi * 0.5f; }
        return keys; }, { 0.40f, -0.12f, 0.11f });
    add("duplo_chute", 2.4f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.2f; keys.touchYaw = 0.0f; }
        if (pulse(t, 0.68f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        return keys; }, { 0.40f, -0.12f, 0.11f });
    // Chute e logo depois um clique de toque (no armado): o chute sai e a bola
    // vai embora por ele - o toque nao a pega na saida.
    add("chute_e_toque", 2.4f, [](float t) {
        Keys keys;
        if (pulse(t, 0.6f)) { keys.request = BallActionKind3D::Shot; keys.power = 0.8f; }
        if (pulse(t, 0.7f)) { keys.request = BallActionKind3D::Dribble; keys.power = 0.3f; keys.touchYaw = Pi * 0.5f; }
        return keys; }, { 0.42f, -0.16f, 0.11f });
    // A forca no toque (06/10 noite, pedido do usuario: "falta de forca
    // quando tento fazer a bola ir em direcao contraria quando em movimento" e
    // "levar em consideracao a velocidade do meu boneco"): correndo com a
    // bola nos pes, em 1,6 s ele aperta a direcao do toque (W, S, D) junto
    // com o clique - como no laboratorio, as teclas dao a direcao e a corrida.
    const auto forceTest = [&](std::string name, int speed, float forwardKey, float leftKey, float power) {
        add(name, 4.2f, [speed, forwardKey, leftKey, power](float t) {
            Keys keys;
            if (t >= 1.6f) {
                keys.forward = forwardKey;
                keys.left = leftKey;
                keys.sprint = speed == 2;
            } else if (t > 0.3f && speed > 0) {
                keys.forward = 1.0f;
                keys.sprint = speed == 2;
            }
            if (pulse(t, 1.6f)) { keys.request = BallActionKind3D::Dribble; keys.power = power; }
            return keys; }, { 0.45f, -0.10f, 0.11f });
        list.back().ballCarried = true;
    };
    for (const auto& [speed, label] : std::vector<std::pair<int, const char*>> { { 0, "parado" }, { 1, "trote" }, { 2, "sprint" } }) {
        const std::string tag = label;
        forceTest("forca_frente_" + tag, speed, 1.0f, 0.0f, 0.15f);
        forceTest("forca_tras_" + tag, speed, -1.0f, 0.0f, 0.15f);
        forceTest("forca_tras_meia_" + tag, speed, -1.0f, 0.0f, 0.5f);
        forceTest("forca_lado_" + tag, speed, 0.0f, -1.0f, 0.15f);
        forceTest("forca_diag_tras_" + tag, speed, -0.7071f, -0.7071f, 0.15f);
    }
    run("sprint_rapidos", 4.0f, true, 0.0f, BallActionKind3D::Dribble, 0.15f, 0.95f, 0.12f, { 1.5f, -0.10f, 0.11f });
    run("corre_rapidos", 4.0f, false, 0.0f, BallActionKind3D::Dribble, 0.15f, 0.95f, 0.12f, { 1.2f, -0.10f, 0.11f });
    return list;
}

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
    float captureFrom = 1.5f;
    float acceleration = 16.0f, deceleration = 22.0f, turnRate = 10.0f, brake = 30.0f;
    int every = 4;
    std::vector<std::pair<std::string, float>> tweaks;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto need = [&]() -> const char* { return i + 1 < argc ? argv[++i] : ""; };
        if (arg == "--cenario") only = need();
        else if (arg == "--acel") acceleration = std::strtof(need(), nullptr);
        else if (arg == "--desacel") deceleration = std::strtof(need(), nullptr);
        else if (arg == "--giro") turnRate = std::strtof(need(), nullptr);
        else if (arg == "--dump") dumpPath = need();
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
    const BallPreviewAssets3D assets = loadBallPreviewAssets3D(MATTERENGINE_ASSETS_DIR);
    const RagdollProfile3D& profile = assets.character.profile;
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
        const auto& rotations = check.update(assets.character, input);
        float worst = 0.0f;
        for (std::size_t k = 0; k < rotations.size(); ++k) {
            const Quaternion a = rotations[k], b = assets.character.visualBones[k].restRotation;
            const float d = std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
            worst = std::max(worst, 2.0f * std::acos(std::min(1.0f, d)) * 180.0f / Pi);
        }
        std::printf("ossos visuais: %zu (mao relaxada difere da exportada %.4f graus; dedos do pe na ligacao %.2f graus)\n",
            rotations.size(), worst, std::max(check.toeBend(0), check.toeBend(1)) * 180.0f / Pi);
    }
    // Os clipes do modo padrao (os do manifesto por cima dos do fisico), como
    // no laboratorio.
    const CharacterLocomotionAnimations3D animations = defaultLocomotionAnimations3D(
        mapCharacterAnimations3D(assets.character, assets.clips), assets.character.defaultLocomotionClipIds,
        assets.clips);
    std::size_t feet[2] = { 0, 0 };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
    }
    std::printf("%-10s | %-17s | %-23s | %-27s | %-14s | %s\n", "cenario", "sola min/apoio max", "deslize apoio med/p95/max",
        "pico acel tronco/bracos (t)", "pico pernas", "giro/tick max | pelve-capsula max");
    std::vector<Scenario> all = scenarios();
    for (const Scenario& extra : ballScenarios()) all.push_back(extra);
    for (const Scenario& scenario : all) {
        if (!only.empty() && only != scenario.name
            && !(only.back() == '*' && std::string(scenario.name).rfind(only.substr(0, only.size() - 1), 0) == 0))
            continue;
        DefaultLocomotion3D locomotion;
        locomotion.reset(0.0f);
        BallControl3D ballControl;
        BallControlOutput3D ballOutput;
        Vec3 ballPosition = scenario.ballStart, ballVelocity = scenario.ballVelocity;
        bool ballHeld = scenario.ballFloat;
        float autoLast = -10.0f;
        int autoDone = 0;
        int ballPhase = 0, ballTouchedFlag = 0;
        RagdollState3D bodyState;
        int touches = 0;
        float firstTouch = -1.0f;
        // A garantia: cada pedido (o instante e se a bola estava na area) ate
        // o toque dele, o descarte ou ser substituido por um novo guardado.
        struct Asked { float at; bool inArea; };
        std::deque<Asked> asked;
        int askedInArea = 0, servedInArea = 0, replacedInArea = 0, failedInArea = 0;
        // A forca: a bola levada ate o pedido; depois do toque, onde ela esta
        // em relacao a capsula (+0,5, +1, +1,5 s) e quando ele a alcanca.
        bool carryReleased = false;
        float forceTouchAt = -1.0f, forceReach = -1.0f, forceBodySpeed = 0.0f;
        Vec3 forceTouchVelocity, forceBodyVelocity;
        std::array<Vec3, 3> forceRel {};
        float worstWait = 0.0f;
        bool wasPending = false;
        std::string failures;
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
            dump = std::fopen(dumpPath.c_str(), "w");
            std::fprintf(dump, "S padrao %s\n", scenario.name);
            std::fprintf(dump, "I pedido 0 toque -1 pe -1 superficie 0 falha 0 rumo 0 direcao 1 0 pelve 0 0 1 bola 0 0 0\n");
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
        const int steps = static_cast<int>(scenario.seconds / Dt);
        for (int step = 0; step < steps; ++step) {
            const float t = step * Dt;
            Keys keys = scenario.keys(t);
            if (scenario.autoDistance > 0.0f && !bodyState.links.empty() && autoDone < scenario.autoCount
                && t >= scenario.autoFrom && t - autoLast >= scenario.autoInterval) {
                const Vec3 pelvis = bodyState.links.front().position;
                const float away = std::hypot(ballPosition.x - pelvis.x, ballPosition.y - pelvis.y);
                const bool behind = (ballPosition.x - pelvis.x) * velocity.x + (ballPosition.y - pelvis.y) * velocity.y < 0.0f
                    && std::hypot(velocity.x, velocity.y) > 1.0f;
                if (away < scenario.autoDistance && away >= scenario.autoMinimum && (!scenario.autoBehind || behind)) {
                    keys.request = scenario.autoKind;
                    keys.power = scenario.autoPower;
                    keys.touchYaw = scenario.autoYaw;
                    autoLast = t;
                    ++autoDone;
                    std::printf("           pedido automatico em %.2f s (bola a %.2f m)\n", t,
                        std::hypot(ballPosition.x - pelvis.x, ballPosition.y - pelvis.y));
                }
            }
            // A capsula: velocidade definida, aceleracao alta (Unreal).
            Vec3 move { keys.forward * std::cos(keys.cameraYaw) - keys.left * std::sin(keys.cameraYaw),
                keys.forward * std::sin(keys.cameraYaw) + keys.left * std::cos(keys.cameraYaw), 0.0f };
            Vec3 wanted;
            if (move.lengthSquared() > 1e-4f) {
                move = move.normalized();
                const float relative = std::atan2(move.y, move.x) - keys.cameraYaw;
                wanted = move * defaultLocomotionSpeed3D(settings, relative, keys.sprint);
                // (O chute correndo freia um pouco - a escala do manejo.)
                wanted = wanted * ballOutput.speedScale;
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
            if (scenario.ballCarried && !carryReleased) {
                ballPosition = { position.x + scenario.carriedOffset.x, position.y + scenario.carriedOffset.y, 0.11f };
                ballVelocity = { velocity.x, velocity.y, 0.0f };
                if (keys.request != BallActionKind3D::None) carryReleased = true;
            }
            if (forceTouchAt >= 0.0f) {
                const Vec3 rel { ballPosition.x - position.x, ballPosition.y - position.y, 0.0f };
                for (int k = 0; k < 3; ++k)
                    if (std::abs(t - forceTouchAt - 0.5f * static_cast<float>(k + 1)) < Dt * 0.5f) forceRel[static_cast<std::size_t>(k)] = rel;
                if (forceReach < 0.0f && t - forceTouchAt > 0.3f && rel.length() < 0.5f) forceReach = t - forceTouchAt;
            }
            // O manejo de bola sobre o corpo do tick anterior (como no
            // laboratorio).
            if (scenario.ball && !bodyState.links.empty()) {
                BallControlInput3D ballInput;
                ballInput.request = keys.request;
                ballInput.power = keys.power;
                ballInput.direction = !std::isnan(keys.touchYaw)
                    ? Vec3 { std::cos(keys.cameraYaw + keys.touchYaw), std::sin(keys.cameraYaw + keys.touchYaw), 0.0f }
                    : wanted.lengthSquared() > 1e-4f ? wanted.normalized()
                    : Vec3 { std::cos(keys.cameraYaw), std::sin(keys.cameraYaw), 0.0f };
                ballInput.lookYawRadians = keys.cameraYaw + keys.aimYaw;
                ballInput.shotLift = keys.lift;
                ballInput.shotCurve = keys.curve;
                ballInput.ballValid = true;
                ballInput.ballPosition = ballPosition;
                ballInput.ballVelocity = ballVelocity;
                ballInput.profile = &profile;
                ballInput.state = &bodyState;
                ballInput.facingYawRadians = keys.cameraYaw;
                ballInput.bodyYawRadians = locomotion.output().facingYawRadians;
                ballInput.bodyYawValid = true;
                ballInput.requestedVelocity = wanted;
                ballInput.bodyVelocity = velocity;
                ballInput.bodyVelocityValid = true;
                if (const char* factor = std::getenv("XFATOR")) ballInput.contactTimingFactor = std::strtof(factor, nullptr);
                ballInput.forceOnly = std::getenv("XSOFORCA") != nullptr;
                // (XSEMHITBOX: a forca pelo tempo da V3, sem a hitbox do pe.)
                ballInput.hitboxContact = std::getenv("XSEMHITBOX") == nullptr;
                ballInput.standing = std::hypot(velocity.x, velocity.y) < 0.3f && wanted.lengthSquared() < 0.01f;
                ballInput.animatedBody = true;
                ballInput.deltaTime = Dt;
                for (std::size_t side = 0; side < 2; ++side) {
                    ballInput.swingProgress[side] = locomotion.output().swingProgress[side];
                    ballInput.footPlanted[side] = locomotion.output().footPlanted[side];
                    ballInput.secondsToTouchdown[side] = locomotion.output().secondsToTouchdown[side];
                }
                bool requestInArea = false;
                if (keys.request != BallActionKind3D::None) {
                    const Vec3 pelvis = bodyState.links.front().position;
                    const BallReachZone3D& zone = ballControl.reachZone();
                    requestInArea = ballPosition.z - 0.11f <= pelvis.z + zone.waistAbovePelvis
                        && std::hypot(ballPosition.x - pelvis.x, ballPosition.y - pelvis.y) <= zone.radius;
                    // (O guardado de antes foi substituido por este.)
                    if (wasPending && !asked.empty()) {
                        if (asked.back().inArea) ++replacedInArea;
                        asked.pop_back();
                    }
                    asked.push_back({ t, requestInArea });
                    if (requestInArea) ++askedInArea;
                    std::printf("           pedido em %.2f s (bola %s da area, a %.2f m)\n", t,
                        requestInArea ? "dentro" : "fora", std::hypot(ballPosition.x - pelvis.x, ballPosition.y - pelvis.y));
                }
                ballControl.update(ballInput, ballOutput);
                wasPending = ballOutput.pending;
                ballPhase = static_cast<int>(ballOutput.phase);
                if (ballOutput.missReason != 0 && !asked.empty()) {
                    if (asked.front().inArea) {
                        ++failedInArea;
                        char text[96];
                        std::snprintf(text, sizeof text, " %.2f s (motivo %d)", asked.front().at, ballOutput.missReason);
                        failures += text;
                    }
                    asked.pop_front();
                }
                if (ballOutput.touched && !asked.empty()) {
                    if (asked.front().inArea) {
                        ++servedInArea;
                        worstWait = std::max(worstWait, t - asked.front().at);
                    }
                    asked.pop_front();
                }
                if (ballOutput.missReason != 0 && keys.request != BallActionKind3D::None)
                    std::printf("           pedido em %.2f s descartado (motivo %d)\n", t, ballOutput.missReason);
                else if (ballOutput.missReason != 0)
                    std::printf("           gesto sem toque em %.2f s (motivo %d)\n", t, ballOutput.missReason);
                if (ballOutput.setMotion) {
                    ballVelocity = ballOutput.ballVelocity;
                    ballHeld = false;
                    if (ballOutput.touched && scenario.ballCarried && forceTouchAt < 0.0f) {
                        forceTouchAt = t;
                        forceTouchVelocity = ballOutput.ballVelocity;
                        forceBodyVelocity = { velocity.x, velocity.y, 0.0f };
                        forceBodySpeed = std::hypot(velocity.x, velocity.y);
                    }
                    if (ballOutput.touched) {
                        ++touches;
                        ballTouchedFlag = 1;
                        if (firstTouch < 0.0f) firstTouch = t;
                        std::printf("           toque em %.2f s: pe %d superficie %d, bola %.2f m/s (%.2f %.2f %.2f) pe-bola %.3f m\n", t,
                            ballOutput.side, static_cast<int>(ballOutput.surface), ballVelocity.length(),
                            ballVelocity.x, ballVelocity.y, ballVelocity.z, ballOutput.footGap);
                    }
                }
            }
            // A bola: rola (freio de 0,6 m/s2), voa e quica.
            if (scenario.ball && ballHeld) {
                ballVelocity = {};
            } else if (scenario.ball) {
                if (ballPosition.z > 0.111f || ballVelocity.z > 0.0f) {
                    ballVelocity.z -= 9.81f * Dt;
                } else {
                    ballPosition.z = 0.11f;
                    if (ballVelocity.z < -1.0f) ballVelocity.z *= -0.6f; else ballVelocity.z = 0.0f;
                    const float planarSpeed = std::hypot(ballVelocity.x, ballVelocity.y);
                    if (planarSpeed > 1e-4f) {
                        const float slower = std::max(0.0f, planarSpeed - 0.6f * Dt) / planarSpeed;
                        ballVelocity.x *= slower; ballVelocity.y *= slower;
                    }
                }
                ballPosition += ballVelocity * Dt;
                if (ballPosition.z < 0.11f) ballPosition.z = 0.11f;
            }
            DefaultLocomotionInput3D input;
            input.ballGesture = ballOutput.gesture;
            input.requestedVelocityWorld = wanted;
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
            {
                // O corpo deste tick (para o manejo do proximo).
                const bool first = bodyState.links.empty();
                bodyState.links.resize(pose.linkPositions.size());
                for (std::size_t i = 0; i < pose.linkPositions.size(); ++i) {
                    PhysicsBodyState3D& link = bodyState.links[i];
                    link.linearVelocity = first ? Vec3 {} : (pose.linkPositions[i] - link.position) * (1.0f / Dt);
                    link.position = pose.linkPositions[i];
                    link.orientation = pose.linkOrientations[i];
                }
            }
            history.push_back(pose.linkPositions);
            rotations.push_back(pose.linkOrientations);
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
                            std::printf("           GIRO %.1f graus em %.3f s no %s (fase da bola %d)\n", turn * 180.0f / Pi, t,
                                profile.links[i].id.c_str(), ballPhase);
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
                visualPose.update(assets.character, visualInput);
            }
            if (const char* path = std::getenv("XPROBEPOSES")) {
                // Poses dos links (origem, orientacao xyzw) de XPROBEDE a
                // XPROBEATE, uma por linha (previa renderizada).
                const float from = std::getenv("XPROBEDE") ? std::strtof(std::getenv("XPROBEDE"), nullptr) : 0.0f;
                const float to = std::getenv("XPROBEATE") ? std::strtof(std::getenv("XPROBEATE"), nullptr) : 1e9f;
                if (t >= from && t <= to) {
                    if (FILE* file = std::fopen(path, "a")) {
                        std::fprintf(file, "{\"t\": %.4f, \"z\": %.4f, \"b\": [%.4f, %.4f, %.4f], \"ph\": %d, \"tc\": %d, \"p\": [",
                            t, position.z, ballPosition.x, ballPosition.y, ballPosition.z, ballPhase, ballTouchedFlag);
                        ballTouchedFlag = 0;
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
                if (scenario.ball)
                    std::fprintf(dump, "K %.4f %.4f %.4f %.3f %.3f %.3f\n", ballPosition.x, ballPosition.y,
                        ballPosition.z, ballVelocity.x, ballVelocity.y, ballVelocity.z);
                if (ballOutput.foot.active)
                    std::fprintf(dump, "Q %d %.4f %.4f %.4f %.3f\n", ballOutput.foot.side, ballOutput.foot.position.x,
                        ballOutput.foot.position.y, ballOutput.foot.position.z, ballOutput.foot.weight);
                if (ballOutput.touched) std::fprintf(dump, "T\n");
            }
        }
        if (dump != nullptr) std::fclose(dump);
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
        if (scenario.ball) {
            std::printf("           bola: %d toques, primeiro em %.2f s, bola agora a %.2f m/s, %.2f m da capsula\n", touches,
                firstTouch, ballVelocity.length(), std::hypot(ballPosition.x - position.x, ballPosition.y - position.y));
            int open = 0;
            for (const Asked& a : asked) if (a.inArea) ++open;
            if (scenario.ballCarried) {
                const Vec3 rel = forceTouchVelocity - forceBodyVelocity;
                std::printf("           FORCA %-24s corpo %.1f m/s | bola (%+.2f %+.2f) %.2f m/s, rel corpo %.2f |"
                    " +0,5 s (%+.2f %+.2f) +1 s (%+.2f %+.2f) +1,5 s (%+.2f %+.2f) | alcanca %.2f s\n", scenario.name,
                    forceBodySpeed, forceTouchVelocity.x, forceTouchVelocity.y, std::hypot(forceTouchVelocity.x, forceTouchVelocity.y),
                    std::hypot(rel.x, rel.y), forceRel[0].x, forceRel[0].y, forceRel[1].x, forceRel[1].y, forceRel[2].x,
                    forceRel[2].y, forceReach);
            }
            std::printf("           GARANTIA %s: %d pedidos com a bola na area -> %d tocados (espera max %.2f s), %d substituidos,"
                " %d falhas%s, %d sem resposta no fim\n", scenario.name, askedInArea, servedInArea, worstWait, replacedInArea,
                failedInArea, failures.c_str(), open);
        }
    }
    return 0;
}
