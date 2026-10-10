#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace MatterEngine {

// O gesto do corpo inteiro com a bola (rework de 06/10, pedido do usuario:
// "o manuseio e o chute estao terrivelmente feios" - a perna esticava reta ate
// a bola). O manejo (BallControl3D) decide o gesto e as duas locomocoes o
// aplicam por cima da pose delas:
//  - a perna que joga por IK analitico (coxa e canela pelo tornozelo, o
//    joelho para o lado pedido, o pe na orientacao da superficie): armada com
//    o joelho dobrado, desdobrando ate a bola, acompanhando depois;
//  - a perna de apoio fica onde a locomocao a pos (o corpo gira e desce em
//    volta dela);
//  - pelve (desce, vai para o apoio, gira), tronco (giro, flexao, inclinacao
//    para o lado contrario da perna), bracos (equilibrio) e cabeca (olha a
//    bola).
// Tudo no mundo (pontos e direcoes) ou em valores relativos a pose da
// locomocao, com pesos: quem aplica so precisa da propria pose.
struct BallGesturePose3D {
    bool active = false;
    // A perna que joga (0 esquerda, 1 direita).
    int side = 0;
    // O gesto inteiro (0..1): pelve, tronco, bracos e cabeca.
    float weight = 0.0f;
    // A perna que joga: o centro da junta do tornozelo (mundo), a direcao
    // para onde o joelho aponta (mundo) e a orientacao do link do pe (mundo),
    // com peso sobre a perna da locomocao.
    Vec3 ankle;
    Vec3 knee { 1.0f, 0.0f, 0.0f };
    Quaternion foot;
    float legWeight = 0.0f;
    // A perna de apoio presa onde a locomocao a pos (o corpo gira em volta).
    bool holdSupport = true;
    // Pelve, por cima da da locomocao: desce (m), vai para o lado (m, mundo,
    // no plano) e gira (rad, em volta da propria pelve: rumo + para a
    // esquerda; frente + inclinando para a frente; lado + inclinando para a
    // esquerda).
    float pelvisDrop = 0.0f;
    Vec3 pelvisShift;
    float pelvisYaw = 0.0f;
    float pelvisPitch = 0.0f;
    float pelvisRoll = 0.0f;
    // Tronco (rad, repartido na coluna): giro + para a esquerda, flexao + para
    // a frente, inclinacao + para a esquerda.
    float trunkTwist = 0.0f;
    float trunkBend = 0.0f;
    float trunkSide = 0.0f;
    // Bracos: o punho em relacao ao ombro (eixos do mundo, m), com peso cada.
    std::array<Vec3, 2> wrist {};
    std::array<float, 2> armWeight {};
    // Cabeca: olha para este ponto (mundo), com peso.
    Vec3 look;
    float lookWeight = 0.0f;
    // As pernas nunca se cruzam (V3): a de apoio fica do lado dela da que
    // joga - lado a lado, pelo menos separateGap (m) de lado no rumo do corpo
    // (separateForward), com peso (no ar: plantada, quem cede e a que joga).
    float separateGap = 0.0f;
    Vec3 separateForward { 1.0f, 0.0f, 0.0f };
    float separateWeight = 0.0f;
    // Um gesto cortado por este com a outra perna ainda no ar (V3): ela sai
    // do alvo dele (tornozelo, joelho e pe no mundo) para a pose da
    // locomocao pelo peso.
    int releaseSide = -1;
    Vec3 releaseAnkle;
    Vec3 releaseKnee { 1.0f, 0.0f, 0.0f };
    Quaternion releaseFoot;
    float releaseWeight = 0.0f;
};

// As pernas do perfil para o IK analitico (coxa, canela, pe; juntas e
// comprimentos). Valido se o perfil tem as duas.
struct BallGestureLegs3D {
    bool valid = false;
    std::array<std::size_t, 2> thigh {};
    std::array<std::size_t, 2> shin {};
    std::array<std::size_t, 2> foot {};
    // Juntas no referencial do link: quadril na pelve, tornozelo no pe.
    std::array<Vec3, 2> hipInPelvis {};
    std::array<Vec3, 2> ankleInFoot {};
    std::array<Vec3, 2> ankleInShin {};
    std::array<float, 2> thighLength {};
    std::array<float, 2> shinLength {};
    std::array<std::size_t, 2> upperArm {};
    std::array<std::size_t, 2> forearm {};
    std::array<std::size_t, 2> hand {};
    std::size_t neck = 0;
    std::size_t head = 0;
};

[[nodiscard]] BallGestureLegs3D resolveBallGestureLegs3D(const RagdollProfile3D& profile);

// A perna pelo tornozelo: coxa, canela e pe (as coordenadas de junta delas)
// para o centro da junta do tornozelo ir a ankleWorld, o joelho apontar para
// kneeWorld (direcao) e o pe ficar o mais perto possivel de footWorld (o
// tornozelo so tem dois eixos). A raiz no mundo (a pelve). Devolve onde o
// tornozelo ficou.
Vec3 solveBallGestureLeg3D(const RagdollProfile3D& profile, const BallGestureLegs3D& legs, int side,
    std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3 ankleWorld,
    Vec3 kneeWorld, Quaternion footWorld);

// Onde a perna esta (mundo): tornozelo, joelho apontando e o pe.
void ballGestureLegState3D(const RagdollProfile3D& profile, const BallGestureLegs3D& legs, int side,
    const std::vector<Vec3>& coordinates, Vec3 rootPosition, Quaternion rootOrientation, Vec3& ankle,
    Vec3& knee, Quaternion& foot);

// O gesto por cima da pose da locomocao (coordenadas de junta e a raiz no
// mundo). movePelvis: a pelve gira e desce (o modo padrao; no fisico a raiz e
// do corpo). legs: a perna que joga e a de apoio presa (o padrao; no fisico os
// pes vao pelo planejador).
void applyBallGesturePose3D(const RagdollProfile3D& profile, const BallGestureLegs3D& legs,
    const BallGesturePose3D& gesture, std::vector<Vec3>& coordinates, Vec3& rootPosition,
    Quaternion& rootOrientation, bool movePelvis, bool solveLegs);

} // namespace MatterEngine
