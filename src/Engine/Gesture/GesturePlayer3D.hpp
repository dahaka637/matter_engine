#pragma once

#include "Engine/Gesture/GestureCatalog3D.hpp"
#include "Engine/Gesture/GestureCurves3D.hpp"

#include <array>

namespace MatterEngine {

// O tocador de um gesto numa perna. A perna sai de onde esta (a pose e a
// velocidade de agora) - no chute, arma para tras primeiro -, passa pela pose
// de contato no mais rapido do gesto e vai ao fim do acompanhamento em trechos
// quinticos (sem quina: a velocidade e a aceleracao continuas); na volta, a
// pose do gesto, parando, cede para a da locomocao numa mistura suave. O corpo (pelve, coluna, cabeca e bracos) sao pulsos suaves com o pico,
// o instante e a largura sorteados pela semente: cada gesto um pouco diferente,
// sem ruido por quadro.
class GesturePlayer3D {
public:
    // Comeca o gesto (no lugar de um em andamento na mesma perna): current e
    // currentVelocity, a perna como esta agora (coordenadas e por segundo).
    void start(const GestureSpec3D& spec, const GestureRig3D& rig, const GestureLegPose3D& current,
        const GestureLegPose3D& currentVelocity);
    // Avanca dt e da a pose deste tick; live, a perna como a locomocao a poria
    // agora (a volta vai para ela); moving, o quanto o corpo anda (0 parado, 1
    // andando: o corpo do gesto entra por cima do da passada). Acabou:
    // inativo.
    [[nodiscard]] GestureFrame3D update(float dt, const GestureLegPose3D& live, float moving = 0.0f);
    [[nodiscard]] bool active() const { return m_active; }
    // O gesto mexe na perna (o do pe; o do peito e o da cabeca sao so do corpo).
    [[nodiscard]] bool movesLeg() const { return m_active && m_spec.part == GesturePart3D::Foot; }
    [[nodiscard]] int side() const { return m_spec.side; }
    [[nodiscard]] const GestureKeys3D& keys() const { return m_keys; }
    void stop() { m_active = false; }
    // Termina em seconds: a perna volta de onde esta para a locomocao e o
    // corpo some junto (outro gesto comecou na outra perna: as duas no ar,
    // nenhum pe de apoio).
    void finishSoon(float seconds);

private:
    // Um canal do corpo: o pico e os instantes do pulso (s; hold > top: o pico
    // fica ate hold).
    struct Channel {
        float peak = 0.0f, start = 0.0f, top = 0.0f, end = 0.0f, hold = -1.0f;
        [[nodiscard]] float at(float t) const {
            if (hold <= top || t <= top) return peak * gestureBump(t, start, top, end);
            if (t <= hold) return peak;
            return peak * (1.0f - gestureSmoothStep5((t - hold) / std::max(1e-4f, end - hold)));
        }
    };
    bool m_active = false;
    bool m_returning = false;
    // A perna no ultimo tick e a velocidade dela (de onde finishSoon volta) e
    // o sumico do corpo (s; < 0 sem).
    GestureLegPose3D m_lastLeg, m_lastLegVelocity;
    bool m_lastValid = false;
    // De onde a perna saiu e a diferenca dela para a da locomocao no primeiro
    // tick (conduzindo, a mistura parte dali: os pes da locomocao somam o
    // empurrao de nao cruzar e a acomodacao, que a pose crua nao tem).
    GestureLegPose3D m_startLeg, m_startOffset;
    bool m_startOffsetValid = false;
    float m_fadeStart = -1.0f, m_fadeEnd = -1.0f;
    float m_time = 0.0f;
    GestureSpec3D m_spec;
    GestureKeys3D m_keys;
    std::array<GestureQuintic, 7> m_windup {};
    std::array<GestureQuintic, 7> m_strike {};
    // O quanto a batida passa da curva no meio dela (a subida; zero sem).
    std::array<float, 7> m_lift {};
    std::array<GestureQuintic, 7> m_follow {};
    std::array<GestureQuintic, 7> m_return {};
    // Pelve (descida, frente, esquerda, giro, inclinacao), coluna (giro,
    // flexao, para tras, inclinacao), cabeca, bracos (frente, fora, cotovelo
    // e de lado, de cada lado), o giro da cintura ate o rumo da bola e a
    // equilibrada de cada braco (fora, frente).
    std::array<Channel, 26> m_channels {};
};

} // namespace MatterEngine
