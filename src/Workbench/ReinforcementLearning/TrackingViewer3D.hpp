#pragma once

#include "Workbench/ReinforcementLearning/ViewerWorld3D.hpp"

#include <filesystem>
#include <memory>

namespace MatterEngine {

// O visualizador de um treino do rastreador fisico (DReCon; vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md) na tela de
// Aprendizado: o ragdoll do jogo a 0% de ajuda seguindo o lider (o modo padrao
// do jogo), de frente na velocidade da tela, com a rede exportada pelo treino
// (a acao media, como no jogo).
class TrackingViewer3D final : public ViewerWorld3D {
public:
    TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed);
    ~TrackingViewer3D() override;
    TrackingViewer3D(const TrackingViewer3D&) = delete;
    TrackingViewer3D& operator=(const TrackingViewer3D&) = delete;

    // Se o tarefa.json e de um treino do rastreador (esquema matter-tracking-1).
    [[nodiscard]] static bool isTrackingTask(const std::filesystem::path& taskPath);

    void setPolicy(const std::filesystem::path& file, std::string& label) override;
    void clearPolicy() override;
    [[nodiscard]] bool hasPolicy() const override;
    void step() override;
    [[nodiscard]] float policyStepSeconds() const override { return 1.0f / 30.0f; }
    void reset(std::uint64_t seed) override;
    void setManualSpeed(std::optional<float> metersPerSecond) override;
    void push(Vec3 force, float seconds) override;
    [[nodiscard]] BodyWorldView3D view() const override;
    [[nodiscard]] const RagdollProfile3D& profile() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
