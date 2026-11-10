#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace MatterEngine {

// Motion Matching (07/11, ele: "trabalhar bem nesse Motion Matching pois eu irei usar ele futuramente para a locomocao
// default"): a cada instante, o quadro do banco de clipes cujo vetor de caracteristicas mais se parece com o do personagem -
// a trajetoria (passado e futuro previsto) e a pose. A referencia e o PoseSearch da UE 5.8 e a configuracao do Game Animation
// Sample (vault 04_locomocao; build-profile/tmp/gasp_export/mm_research). Unidades do motor: metros, segundos, Z para cima,
// X a frente; a "raiz" e o ponto no chao sob o corpo com o rumo dele (yaw).
//
// O esquema diz o que entra no vetor, folha por folha, na ordem:
// - as amostras da trajetoria, cada uma num instante relativo (s; negativo = passado), na ordem velocidade, direcao da
//   velocidade, posicao, rumo (todas XY, no referencial da raiz de agora): a velocidade da raiz entao (a do mundo girada para
//   o referencial de agora), a direcao dela (unitaria; abaixo de 1 cm/s, proporcional), a posicao da raiz entao e o rumo
//   entao (cos, sen do yaw relativo);
// - os canais de ossos (pontos e eixos da pose): a posicao XYZ de um ponto no referencial da raiz (ou em relacao a outro
//   ponto, girada pela raiz), a velocidade XYZ de um ponto no referencial que se move com a raiz (a derivada da posicao
//   local) e a direcao XY de um eixo no referencial da raiz (o "heading" com Z tirado).
// Cada folha tem um peso; as folhas de mesmo rotulo (ou do mesmo grupo de normalizacao) dividem o desvio da normalizacao.
struct MotionMatchingTrajectorySample3D {
    float offsetSeconds = 0.0f;
    bool position = false;
    bool velocity = false;
    bool velocityDirection = false;
    bool facing = false;
    float weight = 1.0f;
};

enum class MotionMatchingBoneFeature3D { Position, Velocity, Heading };

struct MotionMatchingBoneChannel3D {
    static constexpr std::size_t Root = std::numeric_limits<std::size_t>::max();
    MotionMatchingBoneFeature3D feature = MotionMatchingBoneFeature3D::Position;
    // Position e Velocity: o ponto (MotionMatchingPose3D::points); Heading: o eixo (MotionMatchingPose3D::axes).
    std::size_t index = 0;
    // Position: a origem (outro ponto; Root = a raiz).
    std::size_t origin = Root;
    float weight = 1.0f;
    // O grupo de normalizacao (vazio: o rotulo da propria folha).
    std::string normalizationGroup;
    std::string name;
};

struct MotionMatchingSchema3D {
    // Os quadros do banco por segundo.
    float sampleRate = 30.0f;
    // O passo da diferenca finita das velocidades do banco (s; a do PoseSearch e 1/60).
    float finiteDeltaSeconds = 1.0f / 60.0f;
    // O fim de cada clipe que nao repete fora do banco (s; ExcludeFromDatabase (0, -0,3) do PoseSearch).
    float excludeEndSeconds = 0.3f;
    // O peso do canal da trajetoria (multiplica o de cada amostra).
    float trajectoryWeight = 1.0f;
    std::vector<MotionMatchingTrajectorySample3D> trajectory;
    std::vector<MotionMatchingBoneChannel3D> bones;

    // Os pontos e eixos que os presets usam (a ordem de MotionMatchingPose3D): o pe esquerdo, o direito, a pelve; o eixo
    // lateral da pelve (o Y do pelvis do manequim) e o da frente dela.
    static constexpr std::size_t LeftFoot = 0, RightFoot = 1, Pelvis = 2;
    static constexpr std::size_t PelvisLateral = 0, PelvisForward = 1;
    // O padrao do PoseSearch (um esquema novo): trajetoria -0,4 / 0 / 0,35 / 0,7 s, canal 7; os pes, posicao e velocidade.
    [[nodiscard]] static MotionMatchingSchema3D unrealDefault();
    // Os do Game Animation Sample (PSS_Default, PSS_Stop, PSS_Idle - mm_research/schemas.json): a trajetoria -0,05 / 0 /
    // 0,35 / 0,7 / 1,0 s; o pe esquerdo em relacao ao direito, as velocidades dos pes (0,3, o grupo FeetVelZ) e o eixo
    // lateral da pelve (0,1); o da parada com a trajetoria 5 e a direcao da velocidade em 0; o do parado sem o 1,0 s e com
    // a pose da pelve.
    [[nodiscard]] static MotionMatchingSchema3D gaspDefault();
    [[nodiscard]] static MotionMatchingSchema3D gaspStop();
    [[nodiscard]] static MotionMatchingSchema3D gaspIdle();

    struct Leaf {
        std::size_t begin = 0;
        std::size_t size = 0;
        float weight = 0.0f;
        // O rotulo da normalizacao: folhas de rotulo igual (em qualquer esquema do mesmo banco) dividem o desvio.
        std::string label;
    };
    [[nodiscard]] std::vector<Leaf> leaves() const;
    [[nodiscard]] std::size_t featureSize() const;
    [[nodiscard]] float pastHorizonSeconds() const;
    [[nodiscard]] float futureHorizonSeconds() const;
};

} // namespace MatterEngine
