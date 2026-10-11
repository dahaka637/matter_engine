#pragma once

#include "Engine/Athlete/Dribble/BodyForecast3D.hpp"
#include "Engine/Athlete/Dribble/ContactPose3D.hpp"
#include "Engine/Athlete/Dribble/DribbleBall3D.hpp"
#include "Engine/Athlete/Dribble/DribbleTypes3D.hpp"

#include <array>

namespace MatterEngine {

// O plano de um toque (plano v5): a perna, o balanco (do pe sair do chao ao
// pouso), o instante do contato, a parte do pe e onde ele encosta na bola.
// Os tempos sao do relogio do manejo (s).
struct TouchPlan3D {
    bool valid = false;
    int side = 0;
    // Parado: o passo procedural (andando, o balanco do clipe).
    bool step = false;
    float start = 0.0f;
    float contact = 0.0f;
    float end = 0.0f;
    // A volta do pe para o do clipe acaba aqui (relogio): no maximo
    // followMaximum depois do contato, nunca depois do pouso (end).
    float followEnd = 0.0f;
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    ContactStyle3D style = ContactStyle3D::Front;
    Vec3 push { 1.0f, 0.0f, 0.0f };
    // A bola prevista no contato e o pe nele.
    Vec3 ball;
    ContactPose3D pose;
    // O quanto a pelve precisa descer para o pe alcancar (m).
    float drop = 0.0f;
    // A bola dentro da area no contato.
    bool inArea = false;
    // A forca do pedido (0..1) e se e o chute.
    float power = 0.5f;
    bool shot = false;
    float cost = 1e9f;
    // Diagnostico: os candidatos avaliados e os recusados por motivo (sem
    // estilo, alcance, descida, aproximacao, ritmo, passo, cruzando, fora da
    // area, giro da perna).
    int candidates = 0;
    std::array<int, 9> rejected {};
};

struct TouchPlanInput3D {
    const ContactFeet3D* feet = nullptr;
    const BodyForecast3D* body = nullptr;
    const DribbleBallPath3D* ball = nullptr;
    const DribbleGaitForecast3D* gait = nullptr;
    const DribbleSettings3D* settings = nullptr;
    BallOutcomeRequest3D intent;
    float ballRadius = 0.11f;
    // O relogio agora e o primeiro contato permitido (relogio). Os contatos
    // caem na grade dos ticks (tick s): entre dois ticks, a pose do contato
    // saia no tick seguinte, com a bola ja 3 cm adiante.
    float now = 0.0f;
    float earliest = 0.0f;
    float tick = 1.0f / 120.0f;
    // Andando (no ciclo da passada) e a velocidade do corpo agora.
    bool cycling = false;
    Vec3 bodyVelocity;
    float pelvisHeight = 0.9f;
    // Cada perna ocupada ate este instante (relogio; um toque em andamento
    // nela) e a janela que nao pode usar (o toque anterior a usou: o fim
    // dela, relogio; -1 nenhuma).
    std::array<float, 2> busyUntil {};
    std::array<float, 2> usedWindowEnd { -1.0f, -1.0f };
    // Parado: os pes agora (a origem do link, mundo) e o rumo deles (rad).
    std::array<Vec3, 2> footNow {};
    std::array<float, 2> footYawNow {};
    // So esta perna e so a janela que pousa em onlyWindowEnd (relogio): o
    // plano refeito de um toque ja comprometido (-1: qualquer).
    int onlySide = -1;
    float onlyWindowEnd = -1.0f;
    // O plano refeito de um toque ja comprometido: o contato que ele tinha
    // (relogio; -1: um plano novo), o comeco do balanco dele (relogio) e o pe
    // dele agora (a origem do link, mundo). O contato so anda retargetBand s
    // para cada lado, nunca a menos de lockLead de agora, e o pe sai de onde
    // esta (sem isso a aproximacao minima contava de agora e o contato fugia
    // um tick a cada tick).
    float committedContact = -1.0f;
    float committedStart = 0.0f;
    Vec3 swingFoot;
    float swingFootYaw = 0.0f;
    // (Refeito, a parte do pe e o estilo ficam os comprometidos: trocando a
    // cada tick, o pe e o joelho viravam de um lado para o outro.)
    ContactStyle3D committedStyle = ContactStyle3D::Front;
    BallTouchSurface3D committedSurface = BallTouchSurface3D::Inside;
};

[[nodiscard]] TouchPlan3D planTouch3D(const TouchPlanInput3D& input);

// O toque ja comprometido ainda cabe com a previsao de agora: o pe (de onde
// esta, in.swingFoot) chega no contato e volta dele ao pouso no maximo a
// paceShare vezes o limite de velocidade do pe.
[[nodiscard]] bool touchPlanFeasible3D(const TouchPlanInput3D& input, const TouchPlan3D& plan, float paceShare);

// A bola dentro da area em volta da pelve (andando a velocity, adiantada).
[[nodiscard]] bool dribbleInArea3D(Vec3 ball, float radius, Vec3 pelvis, Vec3 velocity,
    const DribbleSettings3D& settings);

} // namespace MatterEngine
