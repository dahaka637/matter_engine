#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace MatterEngine {

// Pose de um link no mundo. Deliberadamente menor que PhysicsBodyState3D: este
// modulo e matematica sobre a arvore do perfil e nao deve arrastar handles,
// eventos de contato nem a interface de cena para dentro dele.
struct ArticulationLinkPose3D {
    // Origem do link (nao o centro de massa).
    Vec3 position;
    Quaternion orientation;
};

// Feedforward de compensacao de gravidade para uma articulacao flutuante:
// o torque que cada eixo de junta precisa aplicar para sustentar o peso de
// tudo o que pende dele, na configuracao atual.
//
// Substitui `PxArticulationReducedCoordinate::computeGravityCompensation`, que
// nao tem equivalente no Jolt. Medicao anterior do projeto ja mostrou que
// desligar essa compensacao PIORA o rastreio de pose, portanto ela nao e
// opcional - e a diferenca entre motores que sustentam o corpo e motores que
// lutam contra o proprio peso dele.
//
// O que ele e, com honestidade:
//
// - **Exato** para a carga estatica de gravidade: o torque no ancoradouro de
//   cada junta e o momento do peso de toda a sua subarvore, computado por uma
//   passada de tras para frente (O(n), forma da passada reversa do RNEA com
//   velocidade e aceleracao nulas).
// - **Aproximado** na escolha do eixo: o torque e projetado nos tres eixos do
//   frame articular do link FILHO, que e o frame em que os motores por eixo
//   agem. Numa junta fora do repouso, os frames de pai e filho diferem pela
//   propria rotacao da junta, e a projecao ignora esse acoplamento. Isso e
//   feedforward, nao dinamica inversa fechada.
// - **Sem Coriolis e sem inercia.** A versao do PhysX tambem oferecia os dois
//   separadamente, e o projeto nunca usou nenhum dos dois no caminho de drive.
//
// Nao aplica limite de torque: quem consome decide, junto com o limite do eixo
// no perfil, o que cabe. O wrench dos seis DOFs da raiz nunca e produzido -
// aplicar essa parcela seria a forca "magica" na pelve que a arquitetura
// proibe, e o caminho PhysX tambem a descartava explicitamente.
//
// Criterio de validacao: pendulo de geometria conhecida tem resposta analitica
// (ver os testes), e no corpo inteiro a metrica e o erro RMS de rastreio de
// pose contra a linha de base do PhysX.
class ArticulationGravityCompensator3D final {
public:
    // Cacheia o que depende somente do perfil: massas, centros de massa,
    // ancoradouros e frames articulares no espaco local de cada link. Chamar
    // uma vez por ragdoll, nao por passo.
    void prepare(const RagdollProfile3D& profile);

    // Avalia na configuracao atual. `linkPoses` usa o mesmo indice do perfil.
    // Nao aloca depois de prepare().
    void compute(std::span<const ArticulationLinkPose3D> linkPoses,
        Vec3 gravity);

    // Mesmo indice dos links do perfil; eixo desabilitado e a propria raiz
    // ficam em zero. Valido ate a proxima chamada de compute()/prepare().
    [[nodiscard]] std::span<const std::array<float, 3>>
        jointTorques() const {
        return m_jointTorques;
    }

    [[nodiscard]] bool ready() const { return !m_links.empty(); }

private:
    struct LinkConstants {
        int parentIndex = -1;
        float massKg = 0.0f;
        Vec3 centerOfMassLocal;
        // Ancoradouro da junta de entrada, no espaco local deste link.
        Vec3 anchorLocal;
        // Frame articular da junta de entrada, no espaco local deste link.
        // X = Twist, Y = Swing1, Z = Swing2.
        Quaternion jointFrameLocal;
        std::array<bool, 3> axisEnabled {};
        bool hasInboundJoint = false;
    };

    std::vector<LinkConstants> m_links;
    std::vector<std::array<float, 3>> m_jointTorques;
    // Totais da subarvore, reusados entre passos para nao alocar no loop de
    // 120 Hz.
    std::vector<Vec3> m_subtreeForce;
    std::vector<Vec3> m_subtreeMoment;
};

} // namespace MatterEngine
