#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingLeader3D.hpp"

#include <array>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// O contrato do rastreador (DReCon, vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md, secoes 3 e 4):
//
// A acao: 21 correcoes dos alvos PD, nas juntas das pernas (coxa 3, canela 1,
// pe 2, dos dois lados), do abdomen (3, o Spine do DReCon) e dos bracos (3 de
// cada lado). Nas juntas de 3 eixos a correcao e um eixo-angulo no espaco de
// restricao da junta, composto pela esquerda: R(alvo) = R(corr) * R(lider); na
// canela e no pe, somada por eixo. As outras juntas seguem o lider. Todas as 39
// recebem alvo a cada passo (no modo ativo, eixo sem alvo amolece).
//
// A observacao (106): v_kin, v_sim, v_sim - v_kin (3 cada), v_des, v_diff (2
// cada), s_sim e s_sim - s_kin (36 cada: posicao e velocidade do centro de
// massa de 6 corpos - os pes, o abdomen, a cabeca e os antebracos) e a acao
// filtrada anterior (21). Tudo no rumo do lider (F_kin; F_sim com a mesma
// orientacao, na origem do corpo).
class TrackingContract3D final {
public:
    static constexpr std::size_t ActionSize = 21;
    static constexpr std::size_t KeyBodyCount = 6;
    static constexpr std::size_t ObservationSize = 3 + 3 + 3 + 2 + 2 + 2 * KeyBodyCount * 6 + ActionSize;

    explicit TrackingContract3D(const RagdollProfile3D& profile);

    [[nodiscard]] const std::vector<std::string>& actionNames() const { return m_actionNames; }
    [[nodiscard]] const std::vector<std::string>& observationNames() const { return m_observationNames; }

    // Os alvos dos motores: o lider mais a correcao filtrada (radianos), a
    // velocidade alvo pela variacao das coordenadas do lider (medida a partir
    // das juntas do corpo), nas 39 juntas, com as escalas do perfil.
    void driveTargets(const TrackingLeaderState3D& leader, const RagdollState3D& body,
        std::span<const float> filteredAction, std::vector<RagdollDriveTarget3D>& out) const;

    // A observacao do DReCon (secao 6.3) para o corpo e o lider de agora.
    void observe(const TrackingLeaderState3D& leader, std::span<const PhysicsBodyState3D> bodyLinks,
        Vec3 bodyCenterOfMass, Vec3 bodyCenterOfMassVelocity, std::span<const float> previousAction,
        std::span<float> out) const;

    [[nodiscard]] const std::array<std::uint32_t, KeyBodyCount>& keyBodies() const { return m_keyBodies; }

private:
    struct ActionLink {
        std::uint32_t link = 0;
        std::uint32_t first = 0;
        // Os eixos habilitados da junta (0 twist, 1 swing1, 2 swing2), na ordem.
        std::vector<std::uint8_t> axes;
        bool composed = false;
    };
    const RagdollProfile3D* m_profile = nullptr;
    std::vector<ActionLink> m_actionLinks;
    // Por link: o indice do ActionLink (-1: segue o lider).
    std::vector<int> m_actionOfLink;
    std::array<std::uint32_t, KeyBodyCount> m_keyBodies {};
    std::vector<std::string> m_actionNames;
    std::vector<std::string> m_observationNames;
};

} // namespace MatterEngine
