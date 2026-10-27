#pragma once

#include "Engine/Gesture/GestureRig3D.hpp"
#include "Engine/Gesture/GestureTypes3D.hpp"

namespace MatterEngine {

// A perna de dois ossos resolvida uma vez (nunca por quadro): o tornozelo
// onde se pede (no referencial da pelve), o joelho apontando para a frente do
// corpo girado de twist (rad; + a ponta do pe para fora) e para cima quando a
// perna vai a frente, e o tornozelo dobrado (plantar: + a ponta para baixo;
// inversion: + a sola para dentro). Longe demais, a perna fica quase reta
// (99,5%) apontando para la. bodyToPelvis: o giro do corpo para a pelve (a
// frente e o cima do joelho sao os do corpo).
[[nodiscard]] GestureLegPose3D gestureLegPose3D(const GestureLegRig3D& leg, int side, Vec3 ankle, float twist,
    float plantar, float inversion, Quaternion bodyToPelvis = {});

// O corpo de um gesto antes do sorteio: o pico de cada canal (rad, m) - a
// pelve, a coluna (giro, flexao para a frente, inclinacao para a esquerda), a
// cabeca (flexao) e os bracos (o da perna do gesto e o outro).
struct GestureBodyPeaks3D {
    float drop = 0.0f, forward = 0.0f, left = 0.0f, yaw = 0.0f, roll = 0.0f;
    // O giro da cintura ate o rumo da bola no chute (rad): cheio no contato,
    // seguro no acompanhamento.
    float turn = 0.0f;
    float spineTwist = 0.0f, spinePitch = 0.0f, spineRoll = 0.0f, spineBack = 0.0f;
    float headPitch = 0.0f;
    // O cabeceio: o tronco e a cabeca armados para tras antes da batida (rad,
    // no pico da armada; somem no contato) e a cabeca indo ao rumo da bola
    // (rad, + para a esquerda; cheia no contato).
    float windupBack = 0.0f, windupHead = 0.0f, headYaw = 0.0f;
    // [0] o braco do lado da perna do gesto, [1] o outro.
    std::array<float, 2> armForward {}, armOut {}, elbow {};
};

// As chaves de um gesto: a pose de contato (a perna na bola de um balde) e a
// do fim do acompanhamento, os tempos (s: o contato, o fim do acompanhamento e
// o fim do gesto, ja de volta a locomocao) e o corpo. A perna passa pela bola
// andando: contactTangent e a velocidade das juntas por metro do tornozelo no
// rumo do toque (rad/m; a coxa e o joelho), e a velocidade do pe ali e
// contactSpeed x followDistance (o caminho do tornozelo ate o fim do
// acompanhamento, m) / o tempo do acompanhamento.
struct GestureKeys3D {
    // A pegada de impulso do chute (a perna armada para tras, pela barra) e o
    // tempo dela (s; 0 no manejo, que vai direto a bola).
    GestureLegPose3D windup;
    float windupTime = 0.0f;
    // A perna subindo antes de ir a bola (a recepcao e a bola alta): a pose
    // por onde ela passa no meio da batida, sem parar.
    GestureLegPose3D lift;
    bool liftValid = false;
    GestureLegPose3D contact;
    GestureLegPose3D follow;
    GestureLegPose3D contactTangent;
    float followDistance = 0.0f;
    float contactSpeed = 1.5f;
    // O quanto a perna e do gesto (0..1; o resto e a da locomocao): conduzindo
    // - o toque a frente e o chute fracos -, o toque fica dentro do passo.
    float legWeight = 1.0f;
    float contactTime = 0.05f;
    float followTime = 0.2f;
    float duration = 0.35f;
    GestureBodyPeaks3D body;
};

// O catalogo: as chaves de um gesto pelo balde, pela forca e pelo tempo.
[[nodiscard]] GestureKeys3D gestureKeys3D(const GestureSpec3D& spec, const GestureRig3D& rig);

// A bola de referencia de um balde (o centro dela no referencial do corpo:
// a pelve no chao, x frente, y esquerda, z altura; jogador de 1,80 m) - onde
// o pe da pose de contato passa. So para conferir (as folhas de quadros).
[[nodiscard]] Vec3 gestureNominalBall3D(const GestureSpec3D& spec);

} // namespace MatterEngine
