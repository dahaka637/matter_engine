#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>

namespace MatterEngine {

// Os gestos de bola como animacoes procedurais (24/10/2026, pedido do usuario:
// o pe nao vai mais ate a bola; a forca sai no comando e o gesto e so
// animacao). A posicao da bola so escolhe o gesto, no comando - a perna, o
// alcance lateral e a posicao frente-tras -; depois e ignorada.
//  - Manejo: 36 gestos = 2 pernas x 3 alcances laterais x 3 posicoes
//    frente-tras x 2 movimentos (puxar para dentro, jogar para fora).
//  - Chute: 2 pernas x 3 alcances laterais; a barra (continua) define o
//    tamanho da trajetoria (o acompanhamento mais longo e mais alto).
// O modulo e so matematica (nada de fisica, atleta ou personagem): a
// locomocao padrao toca o gesto nas juntas dela (GesturePlayer3D) e o atleta
// escolhe o gesto (chooseGesture3D) e aplica a forca.
// (Raise: levantar a bola - a ponta do pe por baixo dela e para cima; 24/10.)
enum class GestureKind3D : std::uint8_t { Touch, Shot, Raise };
// Puxar para dentro (a bola vem para a linha do corpo) ou jogar para fora.
enum class GestureMove3D : std::uint8_t { In, Out };

// A parte do corpo (24/10, pedido do usuario): o pe, o peito (o dominio) e a
// cabeca (o cabeceio). No peito e na cabeca o gesto e so do corpo: as pernas
// seguem a locomocao.
enum class GesturePart3D : std::uint8_t { Foot, Chest, Head };

struct GestureSpec3D {
    GestureKind3D kind = GestureKind3D::Touch;
    // A perna: 0 esquerda, 1 direita.
    int side = 0;
    // O alcance lateral: 0 curto (perto), 1 medio, 2 longe (distante).
    int reach = 0;
    // Frente-tras (so o manejo): 0 atras, 1 no meio, 2 a frente.
    int depth = 1;
    GestureMove3D move = GestureMove3D::Out;
    // A barra (0..1), a altura do chute (-1 rasteiro .. 1 alto, o gesto do
    // mouse) e o tempo do gesto (s).
    float power = 0.5f;
    float lift = 0.0f;
    float seconds = 0.35f;
    // O chute: o giro da cintura ate o rumo em que a bola vai (rad, + para a
    // esquerda, no referencial do corpo; ate 70 graus).
    float turn = 0.0f;
    // A bola alta (o centro de pouco abaixo do joelho para cima): o gesto com o
    // pe na altura do joelho. E a recepcao (a bola vindo para ele): o gesto de
    // parada, amortecendo. Gestos prontos: a bola so escolhe (o pe nunca vai
    // ate ela; 24/10, pedido do usuario).
    bool high = false;
    bool receive = false;
    // Na recepcao, o rumo em que ela vem contra o corpo (referencial do corpo,
    // no plano): escolhe a superficie do pe.
    Vec3 incoming;
    GesturePart3D part = GesturePart3D::Foot;
    // O jeito de cada gesto (o corpo procedural sorteia por ela).
    std::uint32_t seed = 0;
};

// Uma perna nas coordenadas de junta do esqueleto (os vetores de rotacao de
// cada junta, como os clipes): a coxa (3), o joelho (1) e o tornozelo (2; o
// terceiro eixo nao existe e fica 0).
struct GestureLegPose3D {
    std::array<float, 7> value {};
    [[nodiscard]] Vec3 thigh() const { return { value[0], value[1], value[2] }; }
    [[nodiscard]] float knee() const { return value[3]; }
    [[nodiscard]] Vec3 foot() const { return { value[4], value[5], value[6] }; }
    void set(Vec3 thigh, float knee, Vec3 foot) {
        value = { thigh.x, thigh.y, thigh.z, knee, foot.x, foot.y, foot.z };
    }
};

// O que o gesto pede ao resto do corpo neste tick, somado a pose da
// locomocao: a pelve (descida, deslocamento no plano no rumo do corpo - x
// frente, y esquerda - e giros em volta da vertical e do rumo, + esquerda), a
// coluna (cada vertebra: giro, flexao para a frente, inclinacao para a
// esquerda; rad, como as juntas), o pescoco e a cabeca, e os bracos (o ombro
// para a frente, para fora e de lado - em volta da vertical, + a mao para
// dentro -, rad, e o cotovelo dobrando).
struct GestureBody3D {
    float pelvisDrop = 0.0f;
    float pelvisForward = 0.0f;
    float pelvisLeft = 0.0f;
    float pelvisYaw = 0.0f;
    float pelvisRoll = 0.0f;
    std::array<Vec3, 3> spine {};
    Vec3 neck;
    Vec3 head;
    std::array<float, 2> armForward {};
    std::array<float, 2> armOut {};
    std::array<float, 2> armAcross {};
    std::array<float, 2> elbow {};
};

// O gesto neste tick: ativo, a perna dele, a pose dela (inteira, ja com a
// volta a locomocao no fim) e o corpo.
struct GestureFrame3D {
    bool active = false;
    int side = 0;
    GestureLegPose3D leg;
    GestureBody3D body;
    // Tempo desde o comeco e o total (s).
    float time = 0.0f;
    float duration = 0.0f;
};

} // namespace MatterEngine
