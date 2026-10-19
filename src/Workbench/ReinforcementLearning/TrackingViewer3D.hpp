#pragma once

#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"
#include "Workbench/ReinforcementLearning/ViewerWorld3D.hpp"

#include <array>
#include <filesystem>
#include <memory>

namespace MatterEngine {

// O visualizador de um treino do rastreador fisico (DReCon; vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md) na tela de
// Aprendizado: o ragdoll do jogo a 0% de ajuda seguindo o lider (o modo padrao
// do jogo), num dos 8 rumos na velocidade da tela, com a rede exportada pelo treino
// (a acao media, como no jogo).
class TrackingViewer3D final : public ViewerWorld3D {
public:
    // Os rumos de 45 em 45 graus a partir da frente, para a esquerda (como no portao).
    static constexpr std::array<const char*, 8> HeadingNames { "Frente", "Diagonal frente-esquerda", "Esquerda",
        "Diagonal tras-esquerda", "Tras", "Diagonal tras-direita", "Direita", "Diagonal frente-direita" };
    // O historico dos graficos de um treino do rastreador (ele nao grava no rl.db):
    // as rodadas do log.jsonl e, por rumo ("rumoN"), os episodios que nao chegaram ao fim nos
    // portoes de rumos na velocidade mais testada. Com sinceSeconds > 0, so as
    // rodadas desse tanto antes da ultima (sem tanto treino, desde o comeco).
    [[nodiscard]] static TrainingHistory3D readHistory(const std::filesystem::path& runDirectory,
        std::size_t maximumPoints, double sinceSeconds = 0.0);
    // Os exercicios parados (ids e nomes, na ordem da lista, depois dos rumos).
    static constexpr std::array<std::array<const char*, 2>, 3> StillDrills { { { "parado", "Parado" },
        { "giro", "Parado com giro suave" }, { "giro_rapido", "Giro rápido parado" } } };
    // O rumo (rad) do exercicio "rumoN" (outro: a frente).
    [[nodiscard]] static float headingOf(const std::string& drill);

    // drill: "rumoN" (andar no rumo N) ou um dos parados.
    TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed, const std::string& drill = {});
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
