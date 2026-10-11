#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"
#include "Engine/Athlete/Core/FootGeometry3D.hpp"
#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>

namespace MatterEngine {

// O pe na bola (plano v5, o manejo refeito): a pose do pe (a origem do link e
// a orientacao) com a parte certa dele encostada na bola, empurrando no rumo
// pedido. A geometria e a do manejo V3 (aprovada), calculada pela caixa do pe
// do perfil no referencial do pe plano (a orientacao de ligacao do perfil nao
// e a identidade).

// O jeito do toque pela bola em relacao ao corpo (V3).
enum class ContactStyle3D : std::uint8_t {
    // A bola a frente ou do lado.
    Front,
    // Atras, tocada para fora: a perna para tras e para fora, o pe de ponta
    // para baixo, o lado de fora dele na bola (o toque do Pro Soccer Online).
    BehindOut,
    // Atras, tocada para dentro: a sola puxa curta.
    BehindIn
};

// A geometria dos pes para o contato (do perfil).
struct ContactFeet3D {
    bool valid = false;
    AthleteLegs3D legs;
    AthleteFootGeometry3D geometry;
    // A caixa do pe no link (centro, orientacao, meia-dimensao).
    std::array<Vec3, 2> boxCenter {};
    std::array<Quaternion, 2> boxOrientation {};
    std::array<Vec3, 2> boxHalf {};
    // Cada parte do pe (indice: BallTouchSurface3D) - o ponto e a normal para
    // fora, no link.
    std::array<std::array<Vec3, 5>, 2> surfacePoint {};
    std::array<std::array<Vec3, 5>, 2> surfaceNormal {};
    // Coxa + canela e a altura do tornozelo com o pe plano no chao (m).
    std::array<float, 2> legLength {};
    std::array<float, 2> ankleHeight {};
};

[[nodiscard]] ContactFeet3D resolveContactFeet3D(const RagdollProfile3D& profile);

struct ContactRequest3D {
    int side = 0;
    BallTouchSurface3D surface = BallTouchSurface3D::Inside;
    ContactStyle3D style = ContactStyle3D::Front;
    // A bola (centro) e o raio; o rumo do empurrao e a frente do corpo (no
    // plano); a forca (0..1), o gesto do mouse no chute (altura, -1..1) e o
    // chao.
    Vec3 ball;
    float radius = 0.11f;
    Vec3 push { 1.0f, 0.0f, 0.0f };
    Vec3 bodyForward { 1.0f, 0.0f, 0.0f };
    float power = 0.5f;
    float lift = 0.0f;
    float groundHeight = 0.0f;
};

struct ContactPose3D {
    // O link do pe (origem e orientacao), o tornozelo, o ponto que encosta e
    // a normal dele (mundo).
    Vec3 foot;
    Quaternion orientation;
    Vec3 ankle;
    Vec3 surfacePoint;
    Vec3 surfaceNormal;
};

[[nodiscard]] ContactPose3D contactPose3D(const ContactFeet3D& feet, const ContactRequest3D& request);

// A orientacao do link do pe olhando para `toe` (no plano), com a ponta
// `pitchDown` rad para baixo.
[[nodiscard]] Quaternion contactFootOrientation3D(const ContactFeet3D& feet, int side, Vec3 toe, float pitchDown);

// A parte do pe pelo rumo do empurrao em relacao ao corpo e a esta perna (V3):
// para tras a sola (o calcanhar com a bola atras); andando, para a frente o
// peito; para fora o lado de fora; o resto, o de dentro. O chute: o peito, ou
// com efeito o lado (curve: -1..1).
[[nodiscard]] BallTouchSurface3D contactSurface3D(int side, Vec3 push, Vec3 bodyForward, BallActionKind3D kind,
    bool moving, bool ballBehind, float curve);
// O jeito e a parte pela bola em relacao a pelve (V3); falso: nao da (o
// chute com a bola atras).
bool contactStyle3D(int side, Vec3 ball, Vec3 pelvis, Vec3 forward, Vec3 push, bool moving, BallActionKind3D kind,
    float curve, ContactStyle3D& style, BallTouchSurface3D& surface);

// A distancia da caixa do pe (na pose dada) a superficie da bola (m; 0
// encostando ou dentro).
[[nodiscard]] float contactFootGap3D(const ContactFeet3D& feet, int side, Vec3 footPosition,
    Quaternion footOrientation, Vec3 ball, float radius);
// O quanto subir o pe para a caixa ficar `clearance` acima do chao (m).
[[nodiscard]] float contactGroundDeficit3D(const ContactFeet3D& feet, int side, Vec3 footPosition,
    Quaternion footOrientation, float groundHeight, float clearance);

} // namespace MatterEngine
