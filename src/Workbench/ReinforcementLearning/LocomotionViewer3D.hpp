#pragma once

#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteState3D.hpp"
#include "Engine/Athlete/Policy/LocomotionPolicy3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Commands/CommandPilot3D.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Workbench/ReinforcementLearning/ViewerWorld3D.hpp"

#include <cstdint>
#include <span>
#include <filesystem>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace MatterEngine {

// O visualizador de um treino da locomocao (o RL dentro dos modulos, esquema
// matter-rl-locomocao-1) na tela de Aprendizado. O treino em si (Athlete/
// Training) nao entra no laboratorio: aqui o atleta roda pelo runtime do jogo
// (o "Fisico com RL" do laboratorio), num mundo com o chao, comandado pelo
// piloto do exercicio escolhido (o mesmo CommandPilot3D, lido do tarefa.json)
// pela mesma conta do analogico do jogo, com as ajudas da tarefa, e a rede
// exportada pelo treino. Entrega a mesma vista (BodyWorldView3D) que o
// visualizador do Corpo, para a tela desenhar igual.
class LocomotionViewer3D final : public ViewerWorld3D {
public:
    struct Drill {
        std::string id;
        std::string name;
        // A fase do plano a que o exercicio pertence (1, 2, 3...; a tela agrupa por cor).
        int phase = 1;
        // A categoria (basico, compostos...; vazia nas tarefas antigas) e o peso
        // no treino (0: pronto, mas ainda fora do treino).
        std::string category;
        float weight = 1.0f;
        // A velocidade-meta (m/s): a maior das faixas do piloto; 0 nos que nao andam.
        float goalSpeed = 0.0f;
        // O levantar do chao (conta o "levantou-se").
        bool getUp = false;
    };

    // Lanca std::runtime_error se a tarefa nao for da locomocao.
    LocomotionViewer3D(const std::filesystem::path& taskPath, const std::filesystem::path& assetsDirectory,
        const std::string& drill, std::uint64_t seed);
    ~LocomotionViewer3D() override;
    LocomotionViewer3D(const LocomotionViewer3D&) = delete;
    LocomotionViewer3D& operator=(const LocomotionViewer3D&) = delete;

    // Os exercicios de um tarefa.json da locomocao (vazio se nao for uma).
    [[nodiscard]] static std::vector<Drill> drillsOf(const std::filesystem::path& taskPath);
    // Se o tarefa.json e de um treino da locomocao.
    [[nodiscard]] static bool isLocomotionTask(const std::filesystem::path& taskPath);

    // A rede (nula: o classico). O texto diz qual e, ou o erro.
    void setPolicy(const std::filesystem::path& file, std::string& label) override;
    void clearPolicy() override;
    [[nodiscard]] bool hasPolicy() const override;
    // A interligacao das redes nas quedas (01/11, o "getUpHandoff" da tarefa): se agora quem dirige e a rede do levantar,
    // quantas vezes ela assumiu e se, na ultima, ela evitou a queda (nada do tronco, do pescoco ou da cabeca no chao).
    [[nodiscard]] bool gettingUp() const;
    [[nodiscard]] std::uint32_t recoveries() const;
    [[nodiscard]] bool lastRecoveryAvoided() const;
    // A troca vinda da rede do levantar de outro mundo (o visualizador do levantar): o corpo dado nesta cena, o atleta do
    // zero nele, os alvos do classico saindo dos dela e o exercicio recomecando dali.
    void resumeFrom(std::span<const PhysicsBodyState3D> links, std::span<const RagdollDriveTarget3D> drives);

    // Uma decisao (30 Hz: 4 ticks de fisica).
    void step() override;
    [[nodiscard]] float policyStepSeconds() const override { return 1.0f / 30.0f; }
    void reset(std::uint64_t seed) override;
    // Velocidade manual para a frente (no lugar do piloto); nulo devolve o piloto.
    void setManualSpeed(std::optional<float> metersPerSecond) override;
    // Um empurrao no peito (N), por alguns decimos de segundo.
    void push(Vec3 force, float seconds) override;
    // A ajuda auxiliar do mundo (0..100%, as quatro como a regua do laboratorio).
    void setAssist(float percent);
    // A de andar agora (ao abrir, a do treino: a tarefa).
    [[nodiscard]] float assist() const;

    [[nodiscard]] BodyWorldView3D view() const override;
    [[nodiscard]] const RagdollProfile3D& profile() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
