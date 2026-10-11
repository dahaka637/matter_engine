#pragma once

#include "Engine/Athlete/Core/FootGeometry3D.hpp"
#include "Engine/Athlete/Core/Kinematics3D.hpp"

#include <array>
#include <vector>

namespace MatterEngine {

// As pernas nunca se atravessam na pose do padrao (plano v4: sem as pegadas,
// os clipes cruzavam uma perna por dentro da outra nas trocas de rumo). So a
// parte "nunca atravessar", sem trava nenhuma (o que deixava as pegadas
// roboticas): o pe no balanco (mesmo rente ao chao), chegando perto da outra
// perna, tira a canela e o pe dele das capsulas dela (as do perfil) sempre
// para fora, o lado dele - escolhendo o caminho mais curto a cada tick, ele
// trocava de lado no meio da passagem e o pe saltava; e as pernas tambem nao
// cruzam por dentro uma da outra (o usuario: "cruzando as pernas,
// atravessando elas"). A perna empurrada e refeita pelo IK; o empurrao cresce
// numa mola rapida e sai numa mais lenta (apoiado, bem devagar).
struct LegClearanceSettings3D {
    // (Desligado: ainda troca o atravessamento raro do clipe por trancos de
    // ate ~1700 m/s2 em algumas trocas de rumo - ver Execucao_Claude, plano
    // v4. Na sonda: --espaco 1.)
    bool enabled = false;
    // A folga entre as capsulas (m); na canela que anda, so o toque da metade
    // de baixo (a partir desta fracao do joelho ao tornozelo).
    float margin = 0.01f;
    float shinLeverMinimum = 0.5f;
    // Os dois pes no balanco (o voo da corrida), contorna o mais alto.
    // O empurrao: no maximo isto (m); crescendo, nesta mola (s), deixando no
    // maximo touchTolerance (m) de toque, mas nunca mais depressa que
    // maximumPushSpeed (m/s; de uma vez, o pe dava o tranco); soltando, nesta
    // mola (s) e no maximo nesta velocidade (m/s).
    float maximumPush = 0.25f;
    float pushSeconds = 0.06f;
    float touchTolerance = 0.02f;
    float maximumPushSpeed = 1.5f;
    float releaseSeconds = 0.06f;
    float maximumReleaseSpeed = 1.5f;
    // Com o pe apoiado, o empurrao sai no maximo nesta velocidade (m/s).
    float plantedReleaseSpeed = 0.25f;
};

class LegClearance3D {
public:
    void reset(const RagdollProfile3D& profile);
    // Na pose (as coordenadas, a raiz e a pose no mundo, refeitas no lugar).
    // swinging: o pe esta no balanco (o que contorna; apoiado, ele nao mexe).
    void apply(const RagdollProfile3D& profile, std::vector<Vec3>& coordinates, Vec3 rootPosition,
        Quaternion rootOrientation, RagdollAnimationPose3D& pose, std::array<bool, 2> swinging, float deltaTime,
        const LegClearanceSettings3D& settings);
    void clear() { m_applied = {}; }
    [[nodiscard]] bool ready() const { return m_ready; }
    // O empurrao aplicado em cada pe (m, no plano).
    [[nodiscard]] Vec3 applied(int side) const { return m_applied[static_cast<std::size_t>(side & 1)]; }

private:
    struct Piece {
        Vec3 a, b;
        float radius = 0.05f;
    };
    [[nodiscard]] Piece piece(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose, std::size_t link) const;
    // O quanto o pe `moving` precisa ir para fora (no plano) para a perna dele
    // sair da outra (o corpo olhando para forward).
    [[nodiscard]] Vec3 needed(const RagdollProfile3D& profile, const RagdollAnimationPose3D& pose, int moving,
        Vec3 forward, const LegClearanceSettings3D& settings) const;

    bool m_ready = false;
    AthleteLegs3D m_legs;
    AthleteFootGeometry3D m_geometry;
    std::array<Vec3, 2> m_applied {};
    // O pe que contornou no tick anterior (os dois no balanco, ele segue).
    int m_lastMoving = -1;
};

} // namespace MatterEngine
