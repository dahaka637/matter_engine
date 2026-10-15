#pragma once

#include "Engine/Math/Vec3.hpp"
#include "Engine/RL/Commands/ControlAdapter3D.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace MatterEngine {

// O piloto (plano_claude_rl.md secao 9.1): o "jogador de mentira" que segura
// o direcional, gira a camera e aperta os botoes nos exercicios. Ele conhece o
// roteiro inteiro (os pontos, quando soltar), mas o Corpo so recebe o comando
// de agora, pelo mesmo adaptador do controle do jogo. Dados puros, sem fisica:
// a IA do jogo reaproveita o "ir ate um ponto". Quem decide quando frear e o
// piloto; o Corpo decide como.

struct PilotRange3D {
    float minimum = 0.0f;
    float maximum = 0.0f;
};

// Uma faixa de velocidade (andar, trote, corrida, sprint) e a distancia das
// pontas da estrela nela.
struct PilotBand3D {
    std::string name;
    PilotRange3D speed;
    PilotRange3D distance;
    float weight = 1.0f;
};

// A camera do piloto num episodio: parada, acompanhando o movimento (o antigo
// "peito livre") ou girando em trechos.
enum class PilotCameraMode3D : std::uint8_t { Fixed = 0, Follow, Turns };

struct PilotCamera3D {
    // As fatias dos episodios em cada modo.
    float fixedWeight = 0.4f;
    float followWeight = 0.2f;
    float turnsWeight = 0.4f;
    // Acompanhando: o giro maximo atras do movimento (rad/s).
    float followRate = 4.0f;
    // Em trechos: quanto cada um gira (rad, sentido sorteado), a velocidade
    // (rad/s) e a pausa entre eles (s).
    PilotRange3D turnRadians { 0.8f, 3.2f };
    PilotRange3D turnRate { 0.4f, 1.2f };
    PilotRange3D pauseSeconds { 1.0f, 3.0f };
};

// O roteiro de um exercicio.
struct PilotScript3D {
    // "hold": o direcional solto (parado). "stick": segura uma direcao em
    // relacao a camera e uma faixa por um tempo e troca (outra direcao, outra
    // faixa, ou solta). "star": a estrela (o centro e as pontas, indo e
    // voltando ou de ponta a ponta, parando ou passando direto).
    std::string kind = "hold";
    std::vector<PilotBand3D> bands;
    // A janela de velocidade de agora (m/s; o currículo da aquisição): as
    // velocidades pedidas saem da faixa cortada por ela. Desligada se o maximo
    // for 0. O pedido herdado de um quadro de clipe também é cortado nela.
    PilotRange3D speedWindow { 0.0f, 0.0f };
    // So para a frente da camera (o segurar nao sorteia outra direcao): a
    // aquisicao da passada, antes das 8 direcoes.
    bool forwardOnly = false;
    PilotCamera3D camera;
    // O segurar: quanto dura cada pedido, a chance da troca soltar (parar), a
    // fatia das teclas (as 8 direcoes; o resto e o analogico em qualquer
    // angulo), a chance da troca vir junto com um giro da camera (as duas
    // juntas) e a distancia da origem a partir da qual a direcao nova volta.
    PilotRange3D stickSeconds { 0.5f, 4.0f };
    float releaseFraction = 0.2f;
    float keysFraction = 0.7f;
    float syncFraction = 0.3f;
    float leashMeters = 40.0f;
    // A estrela.
    std::uint32_t starDirections = 8;
    float starJitterDegrees = 8.0f;
    // Depois de uma ponta, a chance de ir direto a outra.
    float continuousFraction = 0.5f;
    // A chance de soltar para parar num ponto (senao passa direto).
    float stopFraction = 0.5f;
    // Parado no ponto antes de seguir (s).
    PilotRange3D holdSeconds { 0.8f, 1.5f };
    // A desaceleracao com que o piloto vai soltando ao chegar num ponto de
    // parar (m/s2): o "quando soltar" de uma pessoa ou da IA.
    PilotRange3D brake { 3.0f, 6.0f };
    float arriveRadius = 0.6f;
};

// O que o piloto ve do corpo: onde esta e para onde vai.
struct PilotView3D {
    Vec3 position;
    Vec3 velocity;
};

// O estado do piloto num mundo.
struct PilotState3D {
    Vec3 origin;
    float startHeading = 0.0f;
    // A camera.
    PilotCameraMode3D cameraMode = PilotCameraMode3D::Fixed;
    float camera = 0.0f;
    float cameraTurnLeft = 0.0f;
    float cameraTurnRate = 0.0f;
    float cameraPause = 0.0f;
    // O direcional de agora (no referencial da camera), a direcao dele no
    // mundo quando foi escolhido, a velocidade pedida e o tempo que falta.
    float stickForward = 0.0f;
    float stickLeft = 0.0f;
    float stickWorld = 0.0f;
    float speed = 0.0f;
    float stickTime = 0.0f;
    // A estrela: o ponto de agora e se e para parar nele; a velocidade da ida;
    // a ultima foi uma ponta (e qual); parado no ponto, quanto falta.
    Vec3 target;
    bool hasTarget = false;
    bool stopAtTarget = false;
    float cruise = 0.0f;
    float brake = 4.0f;
    bool atTip = false;
    std::uint32_t lastSlot = 0;
    float holdLeft = 0.0f;
    // O relogio, a ultima troca do pedido (s) e as contagens.
    float clock = 0.0f;
    float lastChange = 0.0f;
    bool changed = false;
    std::uint32_t changes = 0;
    std::uint32_t reached = 0;
    // A espera nos pontos de parar: a soma da distancia ate o ponto a cada
    // decisao parado nele, e quantas decisoes (a deriva media da espera).
    float holdDistanceSum = 0.0f;
    std::uint32_t holdDecisions = 0;
};

class CommandPilot3D {
public:
    // O comeco de um episodio: o corpo em origin, olhando para heading (a camera).
    static void start(const PilotScript3D& script, PilotState3D& state, Vec3 origin, float heading,
        std::mt19937_64& rng);
    // O comeco pelo acervo: o primeiro pedido segue o movimento de agora (a
    // velocidade no mundo): o segurar aponta para la nessa velocidade e a
    // estrela pega a ponta dessa direcao; parado (ou no "hold"), nada muda.
    static void follow(const PilotScript3D& script, PilotState3D& state, Vec3 velocity, std::mt19937_64& rng);
    // O que ele aperta na proxima decisao (dt: a duracao dela).
    [[nodiscard]] static ControlInput3D update(const PilotScript3D& script, PilotState3D& state, const PilotView3D& view,
        float dt, std::mt19937_64& rng);
};

} // namespace MatterEngine
