#pragma once

#include "Engine/Modeling/Raster/Camera.hpp"
#include "Engine/Modeling/Raster/Image.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"
#include "Engine/Modeling/Rig/RagdollSkeletonView.hpp"
#include "Engine/RL/Environment/TrainingBatch3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Supervisor/TrainerLink3D.hpp"
#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"
#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"

#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>

namespace MatterEngine::Workbench {

// A tela "Aprendizado por Reforco": comanda o treinador (um processo proprio,
// que segue com a tela fechada) e mostra o treino. A lateral tem o estado, os
// controles (iniciar, pausar, retomar, parar, salvar, avaliar), o modo de
// desempenho, as habilidades e a ultima avaliacao; o visualizador roda a
// politica mais recente num mundo proprio, em tempo real (inferencia em C++,
// a fisica do jogo), com o jogador de verdade, empurroes para testar e os
// graficos do historico embaixo. Nada daqui muda a simulacao do treino.
class ReinforcementLearningScreen {
public:
    struct Options {
        // A raiz do repositorio (o treinador e as tarefas ficam nela).
        std::filesystem::path sourceRoot;
        // Onde "Salvar imagem" grava.
        std::filesystem::path capturesDirectory;
        std::string run = "principal";
        std::string task = "h1_ficar_em_pe";
    };

    // O que o mundo mostra: o jogador inteiro, so o esqueleto ou so os colisores.
    enum class Display { Full, Skeleton, Colliders };

    ReinforcementLearningScreen(Options options, std::function<void()> onBack);
    ~ReinforcementLearningScreen();

    void update(float deltaSeconds);
    void draw(CanvasTarget& canvas, float uiScale);

    // Automacao (testes, capturas).
    [[nodiscard]] TrainerStatus3D status() const;
    [[nodiscard]] bool viewerReady() const { return m_viewer != nullptr; }
    [[nodiscard]] bool modelLoaded() const { return m_model.has_value(); }
    void waitUntilLoaded();
    void advanceViewer(int decisions);
    [[nodiscard]] const Modeling::Image& image() const { return m_image; }
    [[nodiscard]] bool renderViewport(int width, int height) { return render(width, height); }
    [[nodiscard]] const TrainingHistory3D& history() const { return m_history; }
    void push(int direction, float newtons);
    void setDisplay(Display display) {
        m_display = display;
        m_dirty = true;
    }
    // As habilidades (uma por tarefa em assets/rl/tasks) e a escolhida: o que o
    // mundo mostra (a postura dela) e o que "Iniciar" treina.
    [[nodiscard]] std::vector<std::string> skillTasks() const;
    void selectSkill(std::size_t index);
    [[nodiscard]] std::uint32_t viewerPosture() const;

private:
    struct ModelOutcome {
        std::optional<Modeling::LoadedPlayerModel> loaded;
        std::string error;
    };

    void drawSidebar(float height);
    void drawTrainer();
    void drawControls(const TrainerStatus3D& status);
    void drawModes(const TrainerStatus3D& status);
    void drawSkills(const TrainerStatus3D& status);
    void drawEvaluation(const TrainerStatus3D& status);
    void drawToolbar();
    void drawViewport(CanvasTarget& canvas, float bottomHeight);
    void drawPushes();
    void drawCharts(float height);
    void drawFooter();
    void notify(std::string text);
    [[nodiscard]] float ui(float value) const { return value * m_uiScale; }

    void loadSkills();
    [[nodiscard]] std::string skillName(const std::string& battery) const;
    void ensureViewer();
    void reloadPolicy(bool force);
    void stepViewer(float deltaSeconds);
    void stepOnce();
    [[nodiscard]] bool render(int width, int height);
    void pollModel();
    void refreshHistory();
    void frame(float yawDegrees, float pitchDegrees);
    void saveCapture();
    // O treino mostrado: o do treinador aberto no PC (pode ter sido aberto pelo
    // terminal com outro nome) ou, sem treinador, o desta tela.
    [[nodiscard]] std::string activeRun() const;
    [[nodiscard]] std::filesystem::path taskPath() const;
    [[nodiscard]] std::filesystem::path policyPath() const;

    Options m_options;
    std::function<void()> m_onBack;
    std::unique_ptr<TrainerLink3D> m_link;
    std::string m_mode = "equilibrado";
    float m_uiScale = 1.0f;

    // O mundo do visualizador e a politica dele.
    std::unique_ptr<TrainingBatch3D> m_viewer;
    std::unique_ptr<PolicyRuntime3D> m_policy;
    std::filesystem::file_time_type m_policyWrite {};
    std::filesystem::path m_policyPath;
    std::string m_policyText;
    bool m_useBest = false;
    float m_policyWatch = 0.0f;
    float m_accumulator = 0.0f;
    bool m_paused = false;
    std::vector<float> m_action;
    std::string m_viewerError;

    // O jogador de verdade (carregado numa thread propria).
    std::optional<std::future<ModelOutcome>> m_pending;
    std::optional<Modeling::LoadedPlayerModel> m_model;
    std::unique_ptr<Modeling::RagdollSkeletonView> m_skeleton;
    std::optional<RagdollColliderWireframe3D> m_colliders;
    std::string m_modelError;

    // O desenho.
    Modeling::OrbitCamera m_camera;
    Modeling::Image m_image;
    float m_aspect = 0.0f;
    bool m_dirty = true;
    Display m_display = Display::Full;
    bool m_showCenterOfMass = true;
    bool m_showContacts = true;
    // 100 N por 0,2 s no peito: o que a H1 (sem treino de empurroes) aguenta.
    float m_pushNewtons = 100.0f;

    struct Skill {
        std::string task;
        std::string name;
        // A postura principal (a de maior fatia na tarefa) e a bateria dela.
        std::string posture;
        std::string battery;
    };
    std::vector<Skill> m_skills;
    std::size_t m_skill = 0;
    bool m_skillChosen = false;
    // Quantos canais da observacao a politica le (uma politica de antes do
    // comando da postura le o comeco, que e igual).
    std::size_t m_policyInputs = 0;

    // O historico (graficos) e os avisos.
    TrainingHistory3D m_history;
    // A leitura do banco roda fora da interface (semanas de treino sao muitas
    // linhas).
    std::optional<std::future<TrainingHistory3D>> m_historyPending;
    float m_historyWatch = 0.0f;
    std::string m_notice;
    float m_noticeSeconds = 0.0f;
};

} // namespace MatterEngine::Workbench
