#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace MatterEngine {

// Levantar pelo corpo: sem forca externa nenhuma (nem na pelve, nem no peito,
// nem pares de endireitar), so os motores das juntas dentro da forca nominal
// do perfil.
//
// Uma sequencia de poses-chave com estados intermediarios estaveis (como o
// levantar de robos humanoides): cada pose e descrita no referencial do
// proprio corpo (braco perpendicular ao tronco, coxa a 90 graus...) e
// resolvida pela cinematica inversa com as medidas do boneco; os alvos das
// juntas vao suavemente de uma pose a outra, e a etapa so avanca quando o
// corpo medido chegou (contatos, alturas, orientacao). E o empurrao dos
// membros apoiados no chao que move o corpo.
//
// Caminho: de costas ou de lado ele rola ate ficar de bruços (dobra a perna de
// fora, planta o pe e empurra, com o braco cruzando o peito); apoia as maos;
// faz a flexao; sobe para quatro apoios; dobra os dedos dos pes e ergue o
// quadril (urso: maos e pontas dos pes); anda com os pes em direcao as maos
// (passos curtos, o peso no triangulo das maos e do pe que fica); balanca para
// tras ate o agachamento
// (o quadril desce sobre os calcanhares, os calcanhares baixam, as maos
// soltam); e fica de pe com o centro de massa sobre os pes.
//
// Das ultimas etapas em diante a pelve segue uma trajetoria absoluta no mundo
// (do urso ao agachamento, dele a de pe): os membros apoiados sao resolvidos
// juntos contra ela, entao as cadeias fechadas (maos e pes no chao) ficam
// coerentes em vez de travar; o plano so anda enquanto o corpo acompanha.
enum class BodilyGetUpStage3D : std::uint8_t {
    None,
    Roll,
    PlaceHands,
    PushUp,
    Quadruped,
    TuckToes,
    Bear,
    Inchworm,
    RockBack,
    Stand,
    Done
};

struct BodilyGetUpOutput3D {
    // Alvos das juntas (mesma convencao dos motores) e multiplicadores dos
    // ganhos por link.
    std::vector<Vec3> coordinates;
    std::vector<float> stiffness;
    std::vector<float> damping;
    std::vector<float> torque;
};

struct BodilyGetUpTelemetry3D {
    BodilyGetUpStage3D stage = BodilyGetUpStage3D::None;
    float stageSeconds = 0.0f;
    float stageProgress = 0.0f;
    float pelvisHeight = 0.0f;
    float rootError = 0.0f;
    float comOutside = 0.0f;
    int rollSide = 0;
    std::uint32_t attempts = 0;
};

class BodilyGetUp3D {
public:
    void begin(const RagdollProfile3D& profile, const RagdollState3D& state,
        float groundHeight);
    // Avanca um tick. Devolve true quando terminou em pe (equilibrado).
    bool update(const RagdollProfile3D& profile, const RagdollState3D& state,
        float groundHeight, float deltaTime, BodilyGetUpOutput3D& output);
    // Desistiu desta tentativa (emperrou): volta a deitar e tenta de novo.
    [[nodiscard]] bool failed() const { return m_failed; }
    [[nodiscard]] const BodilyGetUpTelemetry3D& telemetry() const { return m_telemetry; }
    [[nodiscard]] BodilyGetUpStage3D stage() const { return m_stage; }
    // Diagnostico: a pose pedida, presa a pelve medida.
    [[nodiscard]] Vec3 desiredRootPosition() const { return m_desiredRoot; }
    [[nodiscard]] Quaternion desiredRootOrientation() const { return m_desiredOrientation; }
    [[nodiscard]] const std::vector<Vec3>& targetCoordinates() const { return m_previousCoordinates; }

private:
    struct Links {
        std::size_t pelvis = 0;
        std::size_t abdomen = 0;
        std::size_t chest = 0;
        std::size_t upperChest = 0;
        std::size_t neck = 0;
        std::size_t head = 0;
        std::array<std::size_t, 2> upperArm {};
        std::array<std::size_t, 2> forearm {};
        std::array<std::size_t, 2> hand {};
        std::array<std::size_t, 2> thigh {};
        std::array<std::size_t, 2> shin {};
        std::array<std::size_t, 2> foot {};
        // Pontos de cada membro no referencial do link (punho no antebraco,
        // joelho na coxa, tornozelo na canela) e o ombro no peito alto.
        std::array<Vec3, 2> wristInForearm {};
        std::array<Vec3, 2> kneeInThigh {};
        std::array<Vec3, 2> ankleInShin {};
        std::array<Vec3, 2> shoulderInChest {};
        std::array<Vec3, 2> hipInPelvis {};
        // Ponto da sola sob a bola do pe (no referencial do pe).
        std::array<Vec3, 2> ballInFoot {};
        float armLength = 0.57f;
        float thighLength = 0.42f;
        float shinLength = 0.40f;
        bool valid = false;
    };
    struct Body {
        Vec3 pelvis;
        Quaternion pelvisOrientation;
        Vec3 chest;
        Vec3 along;     // pelve -> peito
        Vec3 anterior;  // frente do peito
        Vec3 left;      // esquerda do peito
        Vec3 pelvisAnterior;
        Vec3 pelvisLeft;
        Vec3 centerOfMass;
        Vec3 centerOfMassVelocity;
        std::array<bool, 2> handContact {};
        std::array<bool, 2> kneeContact {};
        std::array<bool, 2> footContact {};
        std::array<Vec3, 2> shoulder {};
        std::array<Vec3, 2> wrist {};
        std::array<Vec3, 2> knee {};
        std::array<Vec3, 2> ankle {};
        std::array<Vec3, 2> ball {};
        Vec3 heading { 1.0f, 0.0f, 0.0f };
        // Elevacao do tronco (pelve -> peito) sobre o horizonte, rad.
        float trunkPitch = 0.0f;
        float coreSpeed = 0.0f;
        float chestHeight = 0.0f;
        float pelvisHeight = 0.0f;
    };

    void resolveLinks(const RagdollProfile3D& profile);
    void measure(const RagdollProfile3D& profile, const RagdollState3D& state,
        float groundHeight);
    void enter(BodilyGetUpStage3D stage);

    // Poses no referencial da pelve (x frente, y esquerda, z para a cabeca).
    void neutral(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const;
    [[nodiscard]] Vec3 shoulderInRoot(const RagdollProfile3D& profile,
        const std::vector<Vec3>& pose, std::size_t side) const;
    void aimWrist(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        std::size_t side, Vec3 offsetFromShoulder) const;
    void aimKnee(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        std::size_t side, Vec3 offsetFromHip) const;
    void legsStraight(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const;
    void armsBeside(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        float push) const;
    void legsKneeling(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        float ankle) const;
    void kneelingLegs(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        float upright, float shift) const;
    void strutArm(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        std::size_t side, Vec3 targetInRoot) const;
    void freeArms(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const;
    void planRockBack(const RagdollProfile3D& profile);
    void planBearRoot(const RagdollProfile3D& profile);
    void planStand(const RagdollProfile3D& profile);
    void rollPose(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        float chestProne, float pelvisProne) const;
    // Leva o tornozelo a um ponto do mundo (pela pelve medida) e deixa a sola
    // paralela ao chao.
    void placeAnkle(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        std::size_t side, Vec3 targetWorld) const;
    void flattenFoot(const RagdollProfile3D& profile, std::vector<Vec3>& pose,
        std::size_t side) const;
    [[nodiscard]] Vec3 toRoot(Vec3 world) const;
    [[nodiscard]] Vec3 toDesired(Vec3 world) const;
    // Servo da pelve: a pelve desejada e a medida um pouco deslocada para a
    // altura, a inclinacao do tronco e o centro de massa pedidos. Os membros
    // apoiados resolvidos a partir dela empurram o corpo para la.
    // O giro do tronco e em torno de `pivot` (o apoio que carrega o peso:
    // ombros sobre as maos, quadril sobre os pes). Altura negativa: sem alvo.
    void servoRoot(float pelvisHeight, float trunkPitch, Vec3 centerOfMassTarget, Vec3 pivot);
    // Membros apoiados presos aos pontos guardados do chao.
    void holdPlants(const RagdollProfile3D& profile, std::vector<Vec3>& pose) const;
    void plantFromBody(bool hands, bool feet);

    // Alvos indo da pose do comeco da etapa a esta (smoothstep na duracao).
    void blendTo(BodilyGetUpOutput3D& output, const std::vector<Vec3>& pose,
        float duration);
    void followTo(BodilyGetUpOutput3D& output, const std::vector<Vec3>& pose,
        float timeConstant, float deltaTime) const;
    void setGains(const RagdollProfile3D& profile, BodilyGetUpOutput3D& output,
        float stiffness, float damping, float torque) const;

    Links m_links;
    Body m_body;
    BodilyGetUpStage3D m_stage = BodilyGetUpStage3D::None;
    float m_stageSeconds = 0.0f;
    float m_stageProgress = 0.0f;
    float m_stalledSeconds = 0.0f;
    float m_bestProgress = -2.0f;
    bool m_failed = false;
    // Lado do rolar (+1: a esquerda desce).
    int m_rollSide = 0;
    int m_rollFlips = 0;
    // Perna que vai a frente (0 esquerda, 1 direita) e o caminho do pe.
    std::size_t m_stepSide = 1;
    Vec3 m_stepFrom;
    Vec3 m_stepTo;
    float m_stepLiftAt = 1e9f;
    // Deslocamento lateral do corpo sobre o tripe (m, + para o lado da perna
    // que fica), realimentado pelo centro de massa.
    float m_lateralLean = 0.0f;
    float m_lateralIntegral = 0.0f;
    float m_groundHeight = 0.0f;
    // Apoios guardados (mundo): punhos e bolas dos pes; o pe da frente
    // apoiado pela sola inteira (tornozelo + sola paralela ao chao).
    std::array<Vec3, 2> m_handPlant {};
    std::array<Vec3, 2> m_footPlant {};
    std::array<bool, 2> m_handPlanted {};
    std::array<bool, 2> m_footPlanted {};
    std::array<bool, 2> m_footFlat {};
    // Comeco da etapa (para interpolar os alvos do servo).
    float m_fromHeight = 0.0f;
    float m_fromPitch = 0.0f;
    Vec3 m_fromCenterOfMass;
    // Trajetoria da pelve (etapas planejadas): de onde parte, para onde vai,
    // o quanto ja andou e a correcao do centro de massa.
    Vec3 m_planFrom;
    Quaternion m_planFromOrientation;
    Vec3 m_planTo;
    Quaternion m_planToOrientation;
    float m_planProgress = 0.0f;
    Vec3 m_balanceShift;
    std::array<Vec3, 2> m_ankleFrom {};
    std::array<Vec3, 2> m_ankleTo {};
    // Passos dos pes no urso: pelve no comeco do passo e a correcao do centro
    // de massa (integral, absoluta), fase e quantos ja foram.
    Vec3 m_inchRoot;
    Quaternion m_inchOrientation;
    float m_inchHeight = 0.75f;
    Vec3 m_inchFromRoot;
    Quaternion m_inchFromOrientation;
    float m_inchSettle = 0.0f;
    Vec3 m_inchOffset;
    int m_inchPhase = 0;
    int m_inchSteps = 0;
    std::vector<Vec3> m_stageFrom;
    std::vector<Vec3> m_previousCoordinates;
    std::vector<Vec3> m_measuredJoints;
    // Pesos por eixo para o IK das pernas: torcao e abertura do quadril quase
    // travadas (sem elas o IK abria as pernas em "sapo" a cada passo).
    std::vector<Vec3> m_legWeights;
    // Os mesmos com o tornozelo travado: o ponto da bola do pe tem uma
    // solucao so (sem isso o IK chegava nele dobrando joelho e tornozelo e a
    // canela deitava).
    std::vector<Vec3> m_legWeightsAnkleLocked;
    Vec3 m_desiredRoot;
    Quaternion m_desiredOrientation;
    BodilyGetUpTelemetry3D m_telemetry;
};

} // namespace MatterEngine
