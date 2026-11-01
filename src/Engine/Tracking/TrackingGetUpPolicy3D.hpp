#pragma once

#include "Engine/Athlete/Body/GetUpPolicy3D.hpp"

#include <filesystem>
#include <memory>

namespace MatterEngine {

class PolicyRuntime3D;
class TrackingGetUpContract3D;

// A rede do levantar do treino levantar_v2 (tools/rl/levantar_amp.py, layout "levantar-v2") para o jogo (01/11): a
// observacao do corpo do contrato do levantar mais as 6 entradas do comando (sem pedido: {1, 0, 0, 0, 1, 0}) e os alvos
// absolutos dos 39 eixos com a rigidez e o amortecimento do treino (velocidade alvo zero).
class TrackingGetUpPolicy3D final : public GetUpPolicy3D {
public:
    explicit TrackingGetUpPolicy3D(const std::filesystem::path& file);
    ~TrackingGetUpPolicy3D() override;
    TrackingGetUpPolicy3D(const TrackingGetUpPolicy3D&) = delete;
    TrackingGetUpPolicy3D& operator=(const TrackingGetUpPolicy3D&) = delete;

    [[nodiscard]] std::uint64_t iteration() const override;
    // (O contrato e refeito quando o perfil muda: os controles de corpo do laboratorio refazem o perfil.)
    void decide(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> links,
        std::vector<RagdollDriveTarget3D>& drives) const override;

private:
    std::unique_ptr<PolicyRuntime3D> m_policy;
    mutable const RagdollProfile3D* m_profile = nullptr;
    mutable std::unique_ptr<TrackingGetUpContract3D> m_contract;
    mutable std::vector<float> m_observation;
    mutable std::vector<float> m_action;
};

} // namespace MatterEngine
