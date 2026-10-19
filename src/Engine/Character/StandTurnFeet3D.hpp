#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>

namespace MatterEngine {

// O giro parado do modo padrao pelos passos (o usuario, 19/10: adaptar o
// planejamento de passos do fisico, "um movimento bonito com a rapidez
// necessaria, sem travar a movimentacao, pois e apenas gesto"). E a logica de
// giro do ContactFootwork3D sem a fisica: os pes ficam onde pousaram enquanto
// a pelve (a mola do padrao, intocada) gira; passando de 12 graus de torcao,
// sai o pe mais atrasado no giro e logo o outro, cada um pousando onde a pose
// parada o quer no rumo final (ate 72 graus a frente por passo), mais rapido
// quanto mais rapido o giro. So pose: nada aqui muda o rumo, a velocidade ou
// a capsula.
struct StandTurnFoot3D {
    Vec3 position;
    Quaternion orientation;
};

struct StandTurnFeetInput3D {
    // Parado no chao, sem gesto de bola: senao os pes voltam a locomocao na
    // hora (em 0,08 s).
    bool standing = false;
    float facingYawRadians = 0.0f;
    // Para onde a pelve vai e quanto ela gira agora (rad/s).
    float targetYawRadians = 0.0f;
    float turnRateRadiansPerSecond = 0.0f;
    // A raiz da pose (em volta dela a pose parada gira) e o chao.
    Vec3 rootWorld;
    float groundHeight = 0.0f;
    // Onde a pose parada poe cada pe agora (mundo; 0 = esquerdo).
    std::array<StandTurnFoot3D, 2> reference {};
    float deltaTime = 0.0f;
};

class StandTurnFeet3D final {
public:
    void reset();
    // feet: os links dos pes (esquerdo, direito).
    void update(const RagdollProfile3D& profile, std::array<std::size_t, 2> feet, const StandTurnFeetInput3D& input);
    // O peso dos pes daqui sobre os da locomocao (0..1) e o pe de cada lado.
    [[nodiscard]] float weight() const { return m_weight; }
    [[nodiscard]] const StandTurnFoot3D& foot(std::size_t side) const { return m_pose[side]; }

private:
    enum class Path { Pivot, Around };
    struct Foot {
        StandTurnFoot3D anchor;
        bool swinging = false;
        Path path = Path::Pivot;
        float seconds = 0.0f;
        float duration = 0.2f;
        StandTurnFoot3D from;
        // O rumo da pelve no comeco do passo e o sentido do giro.
        float fromFacing = 0.0f;
        float sign = 1.0f;
    };
    [[nodiscard]] StandTurnFoot3D landing(const RagdollProfile3D& profile, std::size_t link,
        const StandTurnFeetInput3D& input, std::size_t side, const Foot& foot) const;
    void start(const RagdollProfile3D& profile, std::size_t link, const StandTurnFeetInput3D& input,
        std::size_t side, Path path);

    std::array<Foot, 2> m_feet {};
    std::array<StandTurnFoot3D, 2> m_pose {};
    int m_pending = -1;
    bool m_engaged = false;
    float m_weight = 0.0f;
};

} // namespace MatterEngine
