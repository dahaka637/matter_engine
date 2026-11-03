#pragma once

#include "Engine/Athlete/Body/GetUpPolicy3D.hpp"

#include <filesystem>
#include <memory>
#include <mutex>
#include <utility>

namespace MatterEngine {

class PolicyRuntime3D;
class TrackingGetUpContract3D;

// A rede do levantar do treino levantar_v2 (tools/rl/levantar_amp.py, layout "levantar-v2") fora do treino dela (01/11,
// a interligacao das redes nas quedas do treino da locomocao): a
// observacao do corpo do contrato do levantar mais as 6 entradas do comando (sem pedido: {1, 0, 0, 0, 1, 0}; com o pedido
// de deslocamento, as do levantar em movimento do treino), no layout "levantar-v3" as do corpo contra o chao
// (TrackingGetUpContract3D::observeGround), e os alvos absolutos dos 39 eixos com a rigidez e o amortecimento do treino
// (velocidade alvo zero).
class TrackingGetUpPolicy3D final : public GetUpPolicy3D {
public:
    explicit TrackingGetUpPolicy3D(const std::filesystem::path& file);
    ~TrackingGetUpPolicy3D() override;
    TrackingGetUpPolicy3D(const TrackingGetUpPolicy3D&) = delete;
    TrackingGetUpPolicy3D& operator=(const TrackingGetUpPolicy3D&) = delete;

    [[nodiscard]] std::uint64_t iteration() const override;
    // (Um contrato por perfil, feito na primeira vez: os corpos do treino da locomocao tem perfis proprios. Varias
    // threads podem decidir ao mesmo tempo, cada mundo o seu corpo.)
    void decide(const RagdollProfile3D& profile, const RagdollState3D& body, std::vector<RagdollDriveTarget3D>& drives,
        Vec3 velocity, float groundHeight, const PhysicsScene3D* scene, RagdollHandle3D self) const override;

private:
    [[nodiscard]] const TrackingGetUpContract3D& contractOf(const RagdollProfile3D& profile) const;

    std::unique_ptr<PolicyRuntime3D> m_policy;
    bool m_ground = false;
    std::size_t m_phase = 0;
    std::size_t m_environment = 0;
    mutable std::mutex m_mutex;
    mutable std::vector<std::pair<const RagdollProfile3D*, std::unique_ptr<TrackingGetUpContract3D>>> m_contracts;
};

} // namespace MatterEngine
