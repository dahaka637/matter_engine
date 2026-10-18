#pragma once

#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// A virada brusca em movimento (experimento do laboratorio, 17/10; vault
// 04_locomocao/virada_brusca.md): quando o pedido muda de direcao em
// velocidade, o corpo faz uma manobra curta - antecipa (inclina para dentro
// da curva), prepara, planta o pe de fora, corta e sai - que contrapoe o
// embalo e o deixa pronto para o rumo novo.
//
// E um gerador de movimento procedural (nao toca clipe): do estado do corpo e
// do pedido calcula, por fase, as MESMAS correcoes que a rede do RL tem
// (PhysicalCorrections3D: inclinacao, centro de massa, altura da pelve,
// ritmo e comprimento da passada, onde o pe de fora pousa) e uma escala da
// velocidade pedida (a freada do plantio). Nenhuma forca ou torque de fora no
// corpo: tudo passa pelos motores das juntas e pelo plano dos pes (a ajuda
// auxiliar fica em zero). Desligado (nao chamado), o atleta e identico.
enum class TurnMoveState3D : std::uint8_t { Idle = 0, Anticipate, Prepare, Plant, Cut, Exit };

struct TurnMoveSettings3D {
    // O disparo: pedido e corpo mais rapidos que isto (m/s) e o angulo entre
    // eles (graus) a partir de triggerDegrees; sai da manobra abaixo de
    // releaseDegrees. persistTicks: quantos ticks seguidos antes de comecar.
    float minRequestedSpeed = 1.2f;
    float minBodySpeed = 1.0f;
    float triggerDegrees = 45.0f;
    float releaseDegrees = 25.0f;
    int persistTicks = 3;
    // Abaixo deste pedido (m/s) a manobra aborta (parou de querer andar).
    float abortRequestedSpeed = 0.6f;
    // A entrada: o tempo (s) que os pesos levam a subir depois do disparo (o
    // pedido muda de uma vez, nao ha como antecipar) e o teto da manobra (s).
    float onsetSeconds = 0.05f;
    float maximumSeconds = 1.6f;
    // O angulo (graus) que falta virar: abaixo do primeiro o peso e zero, acima
    // do segundo e inteiro (a manobra acompanha o corpo se alinhando; em corrida
    // ele se realinha em ~0,5 s e um tempo fixo chegava tarde).
    float fullDegrees = 55.0f;
    float emptyDegrees = 12.0f;

    // A intensidade geral (0 desliga; 1 = os valores abaixo).
    float amount = 1.0f;
    // Inclinacao para dentro da curva: base + por m/s (rad), teto (rad).
    float lateralLeanBase = 0.05f;
    float lateralLeanPerSpeed = 0.02f;
    float lateralLeanMaximum = 0.16f;
    // Inclinacao para tras na freada e para a frente na saida (rad).
    float brakeLean = 0.09f;
    float exitLean = 0.06f;
    // O centro de massa para dentro (m): base + por m/s, teto. (Zero por padrao:
    // com a manobra no tempo do corpo, o deslocamento do COM, a passada mais
    // curta e o pe de fora aberto derrubavam mais do que ajudavam na sonda;
    // ficam como ajustes para o laboratorio.)
    float comInsideBase = 0.0f;
    float comInsidePerSpeed = 0.0f;
    float comInsideMaximum = 0.20f;
    // A pelve mais baixa (m, positivo: abaixa): joelhos e quadris dobrados.
    float crouchBase = 0.04f;
    float crouchPerSpeed = 0.012f;
    float crouchMaximum = 0.10f;
    // O ritmo (oitavas, mais rapido) e a passada (oitavas, mais curta).
    float cadenceOctaves = 0.0f;
    float strideOctaves = 0.0f;
    // O pe de fora pousa mais aberto e mais curto (m).
    float outsideSideBase = 0.0f;
    float outsideSidePerSin = 0.0f;
    float outsideShort = 0.0f;
    // A velocidade no corte: a fracao da de entrada, com o piso (m/s).
    float cutSpeedFraction = 0.45f;
    float cutSpeedFloor = 1.5f;
};

struct TurnMoveInput3D {
    // O pedido e o corpo, no plano do mundo (m/s).
    Vec3 requestedVelocity;
    Vec3 bodyVelocity;
    float deltaTime = 1.0f / 120.0f;
};

struct TurnMoveOutput3D {
    TurnMoveState3D state = TurnMoveState3D::Idle;
    // O peso da manobra (0..1) e o andamento (0..1: o quanto do angulo da
    // virada ja foi feito pelo corpo).
    float weight = 0.0f;
    float progress = 0.0f;
    // O angulo (graus) e o lado (+1 esquerda, -1 direita) da virada.
    float turnDegrees = 0.0f;
    float side = 0.0f;
    float entrySpeed = 0.0f;
    // A escala da velocidade pedida (1 fora da freada) e o tempo desde o
    // disparo (s).
    float speedScale = 1.0f;
    float elapsedSeconds = 0.0f;
};

class DynamicTurnMove3D {
public:
    void reset();
    TurnMoveSettings3D& settings() { return m_settings; }
    [[nodiscard]] const TurnMoveSettings3D& settings() const { return m_settings; }

    // Um tick. Idle devolve peso zero; as correcoes so valem com weight > 0.
    void update(const TurnMoveInput3D& input);
    [[nodiscard]] const TurnMoveOutput3D& output() const { return m_output; }
    // As correcoes da manobra de agora (no referencial do rumo; o passo do
    // atleta poe o frame). active() falso fora da manobra.
    [[nodiscard]] const PhysicalCorrections3D& corrections() const { return m_corrections; }

private:
    void start(const TurnMoveInput3D& input, float degrees, float side);
    void synthesize(float degreesNow);

    TurnMoveSettings3D m_settings;
    TurnMoveOutput3D m_output;
    PhysicalCorrections3D m_corrections;
    float m_clock = 0.0f;
    int m_persist = 0;
    // Os pesos suavizados (inclinacao, freada, pe de fora, saida) e o quanto
    // eles andam em direcao ao alvo neste tick.
    float m_smoothLean = 0.0f;
    float m_smoothSide = 0.0f;
    float m_smoothBrake = 0.0f;
    float m_smoothFoot = 0.0f;
    float m_smoothExit = 0.0f;
    float m_followBlend = 1.0f;
};

// Soma as correcoes da manobra as do RL (ou so as dela, sem as outras).
void addTurnMoveCorrections3D(PhysicalCorrections3D& target, const PhysicalCorrections3D& move);

} // namespace MatterEngine
