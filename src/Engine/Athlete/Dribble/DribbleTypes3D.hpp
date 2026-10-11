#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Athlete/Dribble/ContactPose3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// Os tipos do manejo refeito (plano v5, 11/10 noite). A ideia central: o
// toque mora dentro de um balanco - andando, o balanco do proprio clipe
// daquela perna (do pe sair do chao ao pouso), entortado para o pe passar
// pela bola no instante do contato; parado, um passo procedural com a mesma
// estrutura. A forca sai no contato, com o pe de fato na bola.

// Uma janela de balanco de um pe (s a partir de agora: o pe sai do chao em
// start e pousa em end; start negativo: o balanco atual).
struct DribbleWindow3D {
    float start = -1.0f;
    float end = -1.0f;
};

// A passada como o manejo a ve (do tick anterior da locomocao): as janelas
// de cada pe, o ciclo e o pe do clipe no futuro (a origem do link e a
// orientacao) no referencial do corpo - a pelve no chao, o rumo dela (x
// frente, y esquerda) - a cada forecastStep s a partir de agora.
struct DribbleGaitForecast3D {
    static constexpr int WindowCount = 3;
    static constexpr int ForecastCount = 16;
    bool valid = false;
    float cycleSeconds = -1.0f;
    std::array<std::array<DribbleWindow3D, WindowCount>, 2> windows {};
    float forecastStep = 0.04f;
    std::array<std::array<Vec3, ForecastCount>, 2> foot {};
    std::array<std::array<Quaternion, ForecastCount>, 2> footTurn {};
};

// A resposta da capsula do modo padrao (a previsao do corpo).
struct DribbleCapsule3D {
    float acceleration = 16.0f;
    float deceleration = 22.0f;
    float brake = 30.0f;
    float turnRate = 10.0f;
};

struct DribbleSettings3D {
    // A area (a do usuario): a base da bola ate waistAbovePelvis acima da
    // pelve e, no plano, ate areaRadius dela - andando, adiantada no rumo do
    // movimento ate areaLead (cheia a areaLeadSpeed).
    float areaRadius = 0.95f;
    float waistAbovePelvis = 0.05f;
    float areaLead = 0.45f;
    float areaLeadSpeed = 4.5f;
    // Os pedidos: ate queueSize pendentes; sem a bola na area, cada um espera
    // ate requestLife (s); com ela (a garantia) nunca vence: espera o pe
    // alcancar, a forca saindo antes da bola sair da area ou, com a bola longe
    // do pe sem sair, em guaranteeLife (s). Entre dois contatos, pelo menos
    // touchGap (s).
    // (2: o proximo toque e o seguinte, para o encadeamento; com 3, cliques
    // mais rapidos que os pes viravam toques de quase 2 s atras.)
    int queueSize = 2;
    float requestLife = 0.6f;
    float guaranteeLife = 1.5f;
    float touchGap = 0.12f;
    // A velocidade "a frente" do toque (a do manejo V3, onde a forca foi
    // calibrada): a do corpo indo a pedida nesta aceleracao (m/s2), por
    // carryAhead (da aba "Forca na bola").
    float carryAcceleration = 16.0f;
    // O planejamento: ate horizon (s) a frente, em passos de candidateStep;
    // andando, o contato de swingEarliest do balanco ate followMinimum (s)
    // antes do pouso.
    float horizon = 0.8f;
    float candidateStep = 1.0f / 60.0f;
    float swingEarliest = 0.30f;
    float followMinimum = 0.08f;
    // Depois do contato o pe volta para o do clipe em no maximo followMaximum
    // (s) - no trote lento a janela dura 0,9 s e o pe ficava 0,65 s no ar
    // depois do toque -, passando da bola no maximo followThrough (m).
    float followMaximum = 0.25f;
    float followThrough = 0.10f;
    // O custo: a espera (por s), a bola a idealAhead (m) a frente do quadril
    // no rumo do movimento (a sola puxando, idealAheadSole), de lado alem de
    // lateralFree (m), o desvio do pe do clipe (por m) e o pe que precisaria
    // andar mais que footSpeed + footSpeedPerBody x a velocidade do corpo.
    float waitWeight = 1.5f;
    // O pe precisa de pelo menos minimumApproach (s) do comeco do balanco (ou
    // de agora, no balanco ja em curso) ate o contato, e nunca anda mais que
    // paceLimitShare vezes o limite (sem isso o toque comprometia e acontecia
    // no mesmo tick: o pe saltava ate a bola).
    float minimumApproach = 0.08f;
    float paceLimitShare = 1.3f;
    float idealAhead = 0.30f;
    float idealAheadSole = 0.05f;
    float lateralFree = 0.45f;
    float lateralWeight = 0.6f;
    float deviationWeight = 1.2f;
    // E a velocidade a mais (m/s) que esse desvio pede, na ida e na volta:
    // ate deviationRateFree de graca, deviationRateWeight por m/s acima.
    float deviationRateFree = 1.5f;
    float deviationRateWeight = 0.15f;
    float footSpeed = 4.0f;
    float footSpeedPerBody = 0.8f;
    // As pernas nunca se cruzam: no contato, o tornozelo pelo menos footGap
    // (m, de lado no rumo do corpo) do lado dele contra o pe de apoio (o pe do
    // clipe da outra perna nesse instante) e no maximo midlineSlack (m) alem
    // da linha do meio do corpo.
    float footGap = 0.12f;
    float midlineSlack = 0.04f;
    // O giro da perna (o rumo do pe, do pe do clipe no comeco do balanco ate o
    // contato e dele ate o pouso), em media: ate legTurnFree (rad/s) de graca,
    // custando legTurnWeight por rad/s acima, nunca mais que legTurnMaximum
    // (girando rapido demais, o eixo da perna girava 50-70 rad/s na volta).
    float legTurnFree = 5.0f;
    float legTurnWeight = 0.08f;
    float legTurnMaximum = 9.0f;
    // O alcance: o tornozelo no contato a no maximo reach da perna do
    // quadril, a pelve podendo descer ate dropMoving (andando) ou
    // dropStanding (parado) - o corpo do gesto faz isso. (E onde a trava do
    // balanco comeca a ceder, swingReach - reachBand: com o planejador
    // aceitando mais que ela, o pe parava 5 cm antes da bola.)
    float reach = 0.93f;
    float dropMoving = 0.05f;
    float dropStanding = 0.10f;
    // O compromisso: andando, a janela comecando (ou a atual, com o contato a
    // menos de commitLead s); o ponto e o instante do contato ainda se
    // ajustam ate lockLead s antes dele.
    float commitLead = 0.15f;
    float lockLead = 0.06f;
    // Refeito, o contato anda no maximo retargetBand (s) do comprometido e
    // cada s de diferenca custa retargetWeight.
    float retargetBand = 0.06f;
    float retargetEarlier = 0.02f;
    float retargetWeight = 3.0f;
    // No contato, a bola a mais de missDistance (m) da prevista: errou - sem
    // forca, o pedido volta para a fila.
    float missDistance = 0.07f;
    // Comprometido, o toque desiste se a previsao de agora pedir do pe mais
    // que abortPaceShare vezes o limite de velocidade dele (o corpo nao foi
    // para onde se previa: a camera girou, ele parou): o pe volta para o
    // clipe em pelo menos abortFollow s e o pedido volta para a fila.
    float abortPaceShare = 1.6f;
    float abortFollow = 0.12f;
    // Parado, o passo: o pe anda ate o contato a stepSpeed (m/s) - nunca em
    // menos de stepMinimum (s) - e pousa stepFollow (s; o chute, shotFollow)
    // depois do contato.
    float stepSpeed = 2.5f;
    float stepMinimum = 0.10f;
    float stepFollow = 0.14f;
    float shotFollow = 0.20f;
    // A batida: a velocidade do pe no contato, em relacao ao corpo, no rumo
    // do empurrao (m/s; mais a parte pela forca).
    float strikeSpeed = 1.0f;
    float strikeSpeedPower = 2.0f;
    float shotStrikeSpeed = 5.0f;
    float shotStrikeSpeedPower = 6.0f;
    // O pe no ar: a sola a pelo menos clearance (m) do chao antes do contato
    // e o arco do caminho ate ele (m, no meio do caminho, por m andado).
    // (A mesma folga da pose do contato: com mais, o pe subia 1-2 cm no
    // contato.)
    float clearance = 0.008f;
    float liftPerMeter = 0.12f;
    float liftMaximum = 0.07f;
    // A perna no ar nunca mais esticada que a do clipe ou que swingReach
    // (o que for maior), cedendo na faixa reachBand (fracoes da perna).
    float swingReach = 0.96f;
    float reachBand = 0.03f;
    // A passada de conducao: com pedido ou toque em andamento e ate
    // dribbleLinger (s) depois do ultimo toque.
    float dribbleLinger = 0.8f;
    // O corpo do gesto (TouchBody3D, as chaves do V3): correndo pesa
    // bodyRunningShare (a corrida manda; inteiro ate 1 m/s, essa parte a
    // 4 m/s) e some bodyFade (s) depois da volta do pe; o olhar na bola.
    float bodyRunningShare = 0.15f;
    float bodyFade = 0.15f;
    // A mola de cada canal do corpo (s, criticamente amortecida).
    float bodySpringSeconds = 0.05f;
    float lookWeight = 0.8f;
    float bodySeconds = 0.10f;
    // (O olhar entra e sai mais devagar: de uma vez, a cabeca dava 780 m/s2.)
    float lookSeconds = 0.22f;
};

// O alvo de uma perna para a locomocao (mundo), com peso.
struct DribbleLeg3D {
    Vec3 ankle;
    Vec3 knee { 1.0f, 0.0f, 0.0f };
    Quaternion foot;
    float weight = 0.0f;
};

// O corpo do gesto por cima da locomocao (os campos da camada de pose).
struct DribbleBody3D {
    float weight = 0.0f;
    Vec3 pelvisShift;
    float pelvisDrop = 0.0f;
    float pelvisYaw = 0.0f;
    float pelvisPitch = 0.0f;
    float pelvisRoll = 0.0f;
    float trunkTwist = 0.0f;
    float trunkBend = 0.0f;
    float trunkSide = 0.0f;
    std::array<Vec3, 2> wrist {};
    std::array<float, 2> armWeight {};
    Vec3 look;
    float lookWeight = 0.0f;
};

struct DribbleOutput3D {
    std::array<DribbleLeg3D, 2> legs {};
    DribbleBody3D body;
    // A forca na bola neste tick (impulsos linear e angular) e o resultado.
    bool impulse = false;
    std::uint64_t impulseBallKey = 0;
    Vec3 linearImpulse;
    Vec3 angularImpulse;
    Vec3 outcomeVelocity;
    bool touched = false;
    // Interagindo (pedido ou toque em andamento): o dono da bola.
    bool interacting = false;
    std::uint64_t interactingBallKey = 0;
    // A passada: o ritmo (so acelera) e a de conducao (0..1).
    float gaitRateScale = 1.0f;
    float dribbleStyle = 0.0f;
    // Telemetria do toque deste tick: a perna, a superficie, se foi a
    // garantia (a forca sem o pe chegar) e a folga prevista pe-bola (m).
    int touchSide = -1;
    BallTouchSurface3D touchSurface = BallTouchSurface3D::Inside;
    bool touchGuaranteed = false;
    // (A bola na forca contra a prevista no plano, m; toques que desistiram
    // ou erraram neste tick.)
    float touchMiss = 0.0f;
    int aborted = 0;
    int missed = 0;
    // Cliques juntados ao pendente mais novo (a fila cheia) neste tick.
    int merged = 0;
    int pending = 0;
};

} // namespace MatterEngine
