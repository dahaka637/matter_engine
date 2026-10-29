#pragma once

#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Gesture/GestureChoice3D.hpp"

#include <cstdint>

namespace MatterEngine {

// A IA de captura (29/10, o teste dele dos contatos; nao e a IA do jogo): um
// jogador de teclado e mouse. Usa so o que o jogador usa - as oito teclas em
// relacao ao olhar, o Shift, o Ctrl (o carrinho), o olhar girando como um
// mouse e o botao do drible com a barra (a forca e o tempo segurado) - e o
// comando sai pelo mesmo mapeamento do analogico (athleteStickCommand3D).
// Busca a bola solta, tenta roubar de quem a tem e, com ela, fica entre
// nearMeters e farMeters do jogador. Humanizada (pedido dele: "para nao
// parecer um missil teleguiado"): decide a cada tempo de reacao, o olhar
// gira com o limite de uma mao, as teclas trocam com folga, os toques tem a
// forca da barra com erro, ela espreita antes de dar o bote e respira depois
// de perder a bola; cada boneco tem o seu jeito (pela semente).

// O que o piloto ve neste tick (de quem chama).
struct CapturePilotView3D {
    std::uint64_t self = 0;
    // O corpo: a pelve no chao (z e o chao), a velocidade no plano e o rumo;
    // no padrao (a tabela de velocidades dele) e sem teclas (caido,
    // levantando, no carrinho: como o laboratorio faz com o jogador).
    Vec3 pelvisGround;
    Vec3 velocity;
    float bodyYaw = 0.0f;
    bool animated = true;
    bool down = false;
    float terrainSpeedLimit = 1.0e9f;
    // A bola (sem ela, ball falso) e o dono (0: ninguem), com a pelve e a
    // velocidade dele quando e outro atleta da cena.
    bool ball = false;
    Vec3 ballPosition;
    Vec3 ballVelocity;
    std::uint64_t ballOwner = 0;
    bool ownerSeen = false;
    Vec3 ownerPelvis;
    Vec3 ownerVelocity;
    // O jogador (o controlado; a faixa de distancia e dele).
    bool player = false;
    Vec3 playerPelvis;
    Vec3 playerVelocity;
};

struct CapturePilotSettings3D {
    float nearMeters = 2.0f;
    float farMeters = 5.0f;
    bool tackles = true;
    // A barra do drible: o tempo para encher (a do laboratorio).
    float touchChargeSeconds = 0.6f;
    // A semente do jeito dele (0: a do atleta).
    std::uint64_t seed = 0;
};

// O corpo do atleta na visao: a pelve no chao, o rumo, o modo e se esta no chao.
void capturePilotSelf3D(CapturePilotView3D& view, const RagdollProfile3D& profile, const AthleteState3D& athlete,
    const RagdollState3D& body);

class CapturePilot3D {
public:
    [[nodiscard]] AthleteCommand3D update(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot,
        const AthleteRuntimeSettings3D& settings, const CharacterGaitSpeeds3D& gaitSpeeds, const GestureReach3D& reach,
        float deltaTime);
    void reset() { *this = CapturePilot3D {}; }

private:
    enum class Plan : std::uint8_t { Chase, Press, Keep, Recover };
    enum class Look : std::uint8_t { Target, Ball, Heading };

    float random();
    void decide(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot);
    bool bandHeading(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot);
    void press(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot);
    int keysFor(Vec3 desired) const;

    bool m_started = false;
    std::uint32_t m_rng = 1;
    // O jeito dele: o tempo de reacao, a mao no mouse (rad/s), a vontade de
    // dar o bote, o ritmo dos toques e o gosto pelo Shift.
    float m_reaction = 0.25f;
    float m_mouseRate = 6.0f;
    float m_aggression = 0.6f;
    float m_rhythm = 0.6f;
    float m_sprintLove = 0.6f;
    // O olhar (a camera dele) e o giro.
    float m_look = 0.0f;
    float m_lookRate = 0.0f;
    // A decisao: o plano, o ponto (ou o rumo), o raio de chegada, o Shift, o
    // olhar e o tempo ate a proxima.
    Plan m_plan = Plan::Chase;
    Look m_lookAt = Look::Target;
    bool m_toPoint = true;
    Vec3 m_target;
    Vec3 m_heading;
    float m_stopRadius = 0.0f;
    bool m_arrived = false;
    bool m_sprint = false;
    bool m_catching = false;
    float m_decisionLeft = 0.0f;
    // As teclas: a combinacao de agora (0..7 a partir de W no sentido
    // anti-horario; -1 nenhuma) e ha quanto tempo.
    int m_keys = -1;
    float m_keysAge = 0.0f;
    // O botao do drible: segurando, ha quanto tempo, a forca querida; o
    // proximo toque conduzindo.
    bool m_holding = false;
    float m_held = 0.0f;
    float m_holdPower = 0.0f;
    float m_sinceTouch = 10.0f;
    float m_nextTouch = 0.6f;
    // O carrinho (o Ctrl segurado e a espera), o lado de rodear e a finta, o
    // bote e o respiro.
    float m_ctrlLeft = 0.0f;
    float m_sinceTackle = 10.0f;
    bool m_tackleIntent = false;
    float m_side = 1.0f;
    float m_feintLeft = 2.0f;
    float m_lungeLeft = 0.0f;
    float m_recoverLeft = 0.0f;
    std::uint64_t m_lastOwner = 0;
    bool m_wasDown = false;
};

} // namespace MatterEngine
