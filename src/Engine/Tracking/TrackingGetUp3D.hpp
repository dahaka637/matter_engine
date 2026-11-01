#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingClipReference3D.hpp"

#include <array>
#include <iterator>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// O levantar sem ajuda pelo DeepMimic (vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secao 8):
//
// A acao: o alvo absoluto de cada um dos 39 eixos (o DeepMimic 5.1: a orientacao
// alvo do PD em cada junta), velocidade alvo zero, o ganho do perfil.
//
// A observacao (DeepMimic 5.1): no referencial do corpo (a raiz na origem, o
// rumo horizontal da raiz, a gravidade), de cada link a posicao em relacao a
// raiz, a rotacao (quaternion), a velocidade linear e a angular; mais a altura
// da raiz (o codigo do DeepMimic a inclui; o texto nao diz) e a fase t/T.
// O rumo da raiz deitada: a soma das projecoes no chao da frente e do alto da
// pelve (so a frente degenera de brucos ou de costas; declarado).
//
// A nota (DeepMimic 5.3): r = 0,65 r_p + 0,1 r_v + 0,15 r_e + 0,1 r_c, com
//   r_p = exp(-2 sum_j |q^_j (-) q_j|^2)    (a raiz no mundo e as juntas no pai)
//   r_v = exp(-0,1 sum_j |w^_j - w_j|^2)    (a velocidade angular de cada junta)
//   r_e = exp(-40 sum_e |p^_e - p_e|^2)     (maos e pes, no mundo)
//   r_c = exp(-10 |p^_c - p_c|^2)           (o centro de massa, no mundo)
class TrackingGetUpContract3D final {
public:
    explicit TrackingGetUpContract3D(const RagdollProfile3D& profile);

    [[nodiscard]] std::size_t observationSize() const { return 1 + 13 * m_profile->links.size() + 1; }
    [[nodiscard]] std::size_t actionSize() const { return m_axes.size(); }
    [[nodiscard]] const std::vector<std::string>& observationNames() const { return m_observationNames; }
    [[nodiscard]] const std::vector<std::string>& actionNames() const { return m_actionNames; }

    // Os alvos absolutos dos 39 eixos (velocidade alvo zero).
    void driveTargets(std::span<const float> action, std::vector<RagdollDriveTarget3D>& out) const;
    void observe(std::span<const PhysicsBodyState3D> links, float phase, std::span<float> out) const;
    // So o corpo (sem a fase): a altura da raiz e os 13 numeros de cada link (o levantar livre, 31/10).
    [[nodiscard]] std::size_t bodyObservationSize() const { return 1 + 13 * m_profile->links.size(); }
    void observeBody(std::span<const PhysicsBodyState3D> links, std::span<float> out) const;
    // O corpo contra o chao (01/11, o contrato v3 - ele: "precisamos trazer novas informacoes a ele... a velocidade e
    // direcao, e relacao com o solo e direcao do solo"; o rumo de observeBody vira 180 graus com a pelve alem de 45 graus
    // para tras, no meio da queda de costas, e estas nao viram): o baixo (a gravidade) no quadro da pelve e no do peito
    // alto, a velocidade do centro de massa no do peito alto, a altura dele e a das GroundLinks ate o chao, o tempo ate o
    // chao da cabeca e do peito alto, os contatos das GroundLinks e se algo que nao o chao encosta (parede, outro corpo).
    static constexpr const char* GroundLinks[11] { "Head", "UpperChest", "Pelvis", "LeftHand", "RightHand",
        "LeftForearm", "RightForearm", "LeftShin", "RightShin", "LeftFoot", "RightFoot" };
    static constexpr std::size_t GroundObservationSize = 3 + 3 + 3 + 1 + 11 + 2 + 11 + 1;
    void observeGround(std::span<const PhysicsBodyState3D> links, std::span<const RagdollInteraction3D> contacts,
        float groundHeight, std::span<float> out) const;
    [[nodiscard]] static std::vector<std::string> groundObservationNames();
    // O minimo e o maximo (rad) de cada eixo da acao, os da junta no perfil (o levantar livre normaliza a acao neles).
    [[nodiscard]] std::vector<float> actionRanges() const;
    // As coordenadas de junta de uma pose na ordem da acao (o alvo que a reproduz).
    void actionOf(std::span<const Vec3> coordinates, std::span<float> out) const;

    // A nota do DeepMimic contra a referencia, e as parcelas.
    struct Terms {
        float pose = 0.0f, velocity = 0.0f, endEffector = 0.0f, centerOfMass = 0.0f, total = 0.0f;
    };
    [[nodiscard]] Terms reward(std::span<const PhysicsBodyState3D> body, const TrackingClipFrame3D& reference) const;

    [[nodiscard]] std::uint32_t headLink() const { return m_head; }
    [[nodiscard]] std::uint32_t rootLink() const { return m_root; }

private:
    struct Axis {
        std::uint32_t link = 0;
        std::uint8_t axis = 0;
    };
    const RagdollProfile3D* m_profile = nullptr;
    std::vector<Axis> m_axes;
    std::vector<std::uint32_t> m_endEffectors;
    std::uint32_t m_root = 0;
    std::uint32_t m_head = 0;
    std::uint32_t m_chest = 0;
    std::array<std::uint32_t, std::size(GroundLinks)> m_groundLinks {};
    std::vector<std::string> m_observationNames;
    std::vector<std::string> m_actionNames;
};

// O rumo horizontal do corpo pela pelve (a soma das projecoes da frente e do alto).
[[nodiscard]] float trackingBodyYaw3D(const RagdollLinkDefinition3D& pelvis, const PhysicsBodyState3D& state);

} // namespace MatterEngine
