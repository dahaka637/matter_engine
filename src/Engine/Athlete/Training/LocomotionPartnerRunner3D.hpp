#pragma once

#include "Engine/Athlete/Body/LocomotionPartner3D.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace MatterEngine {

// A rede da locomocao exportada por um treino (o .mrlpolicy da LocomotionPolicy3D, como no jogo) com as configuracoes
// do atleta da tarefa dele, como a parceira da rede do levantar no treino dela (01/11, a interligacao das redes). O
// corpo da cena tem de ser o do pacote padrao dos assets (o mesmo do treino do levantar).
class LocomotionPartnerRunner3D final : public LocomotionPartner3D {
public:
    LocomotionPartnerRunner3D(const std::string& assetsDirectory, const std::filesystem::path& taskFile,
        const std::filesystem::path& policyFile);
    ~LocomotionPartnerRunner3D() override;
    LocomotionPartnerRunner3D(const LocomotionPartnerRunner3D&) = delete;
    LocomotionPartnerRunner3D& operator=(const LocomotionPartnerRunner3D&) = delete;

    [[nodiscard]] std::uint64_t iteration() const;
    Result drive(PhysicsScene3D& scene, RagdollHandle3D handle, std::span<const RagdollDriveTarget3D> handoff,
        float maximumSeconds, const Push* push = nullptr, Vec3 velocity = {}) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
