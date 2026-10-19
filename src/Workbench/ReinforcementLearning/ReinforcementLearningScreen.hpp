#pragma once

#include "Engine/Modeling/Raster/Camera.hpp"
#include "Engine/Modeling/Raster/Image.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"
#include "Engine/Modeling/Rig/RagdollSkeletonView.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Supervisor/TrainerLink3D.hpp"
#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"
#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"
#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include <array>
#include <deque>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {

// A tela "Aprendizado por Reforco" do Corpo: comanda o treinador (um processo
// proprio, que segue com a tela fechada) e mostra o treino. A lateral tem o
// estado, o treino escolhido, os controles (continuar, pausar, retomar, parar,
// salvar, avaliar), o modo de desempenho, os exercicios da tarefa e a ultima
// avaliacao; o visualizador roda a politica exportada do treino num mundo
// proprio, em tempo real (inferencia em C++, a fisica do jogo), com o jogador
// de verdade, o comando do piloto ou na mao, empurroes para testar e os
// graficos do historico embaixo. Nada daqui muda a simulacao do treino.
class ReinforcementLearningScreen {
public:
    struct Options {
        // A raiz do repositorio (o treinador e as tarefas ficam nela).
        std::filesystem::path sourceRoot;
        // Onde "Imagem" grava.
        std::filesystem::path capturesDirectory;
        // O treino mostrado ao abrir (vazio = o mais recente).
        std::string run;
    };

    // O que o mundo mostra: o jogador inteiro, so o esqueleto ou so os colisores.
    enum class Display { Full, Skeleton, Colliders };

    ReinforcementLearningScreen(Options options, std::function<void()> onBack);
    ~ReinforcementLearningScreen();

    void update(float deltaSeconds);
    void draw(CanvasTarget& canvas, float uiScale);

    // Automacao (testes, capturas).
    [[nodiscard]] TrainerStatus3D status() const;
    [[nodiscard]] bool viewerReady() const { return hasViewer(); }
    [[nodiscard]] bool policyLoaded() const { return m_policy != nullptr || (m_locomotion && m_locomotion->hasPolicy()); }
    [[nodiscard]] const std::string& viewerError() const { return m_viewerError; }
    [[nodiscard]] bool modelLoaded() const { return m_model.has_value(); }
    void waitUntilLoaded();
    void advanceViewer(int decisions);
    [[nodiscard]] const Modeling::Image& image() const { return m_image; }
    [[nodiscard]] bool renderViewport(int width, int height) { return render(width, height); }
    [[nodiscard]] const TrainingHistory3D& history() const { return m_history; }
    // Os graficos expandidos e a janela de tempo (0..3: 60 s, 60 min, 5 h, 24 h).
    void setChartsExpanded(bool expanded, int window) {
        m_chartsExpanded = expanded;
        m_chartWindow = window;
        m_historyWatch = 0.0f;
    }
    void push(int direction, float newtons);
    // O comando na mao (m/s para a frente da camera do piloto) ou, com
    // std::nullopt, o piloto da tarefa.
    void setManualSpeed(std::optional<float> metersPerSecond);
    void setDisplay(Display display) {
        m_display = display;
        m_dirty = true;
    }
    void frameView(float yawDegrees, float pitchDegrees, float distance = 4.4f) {
        frame(yawDegrees, pitchDegrees, distance);
    }
    // Os treinos do Corpo na pasta de dados (o mais recente primeiro) e o mostrado.
    [[nodiscard]] const std::vector<std::string>& runs() const { return m_runs; }
    [[nodiscard]] std::string shownRun() const { return activeRun(); }
    void selectRun(const std::string& run);
    // Os exercicios da tarefa do treino e o do visualizador.
    [[nodiscard]] std::vector<std::string> drills() const;
    void showDrill(const std::string& drill);
    [[nodiscard]] float viewerRequestedSpeed() const;
    [[nodiscard]] float viewerSpeed() const;
    // Pausa e mostra o quadro de tantas decisoes atras na gravacao (0: o ultimo).
    void scrubViewer(float decisionsBehind) {
        if (m_replay.empty()) return;
        m_paused = true;
        m_playhead = std::max(0.0f, static_cast<float>(m_replay.size() - 1) - decisionsBehind);
        updateShown();
    }
    [[nodiscard]] float viewerChestHeading() const { return hasViewer() ? viewerView().chestHeading : 0.0f; }
    [[nodiscard]] Vec3 viewerPelvis() const {
        const BodyWorldView3D view = hasViewer() ? viewerView() : BodyWorldView3D {};
        return view.state && !view.state->links.empty() ? view.state->links[0].position : Vec3 {};
    }
    [[nodiscard]] std::uint32_t viewerSteps() const { return hasViewer() ? viewerView().steps : 0u; }
    [[nodiscard]] std::array<float, 2> viewerPelvisOnScreen() const { return m_pelvisOnScreen; }

private:
    struct ModelOutcome {
        std::optional<Modeling::LoadedPlayerModel> loaded;
        std::string error;
    };

    // O visualizador de agora (o do Corpo ou o da locomocao).
    [[nodiscard]] bool hasViewer() const { return m_viewer != nullptr || m_locomotion != nullptr; }
    [[nodiscard]] BodyWorldView3D liveView() const { return m_locomotion ? m_locomotion->view() : m_viewer->view(0); }
    // O quadro mostrado: o da gravacao (pausa, camera lenta, voltar) ou o de agora.
    [[nodiscard]] BodyWorldView3D viewerView() const {
        if (!m_shownValid) return liveView();
        BodyWorldView3D view = m_shownView;
        view.state = &m_shownState;
        return view;
    }
    // A gravacao dos ultimos quadros (uma decisao cada) e o quadro mostrado.
    void recordFrame();
    void clearReplay();
    void updateShown();
    [[nodiscard]] float liveHead() const;
    [[nodiscard]] const RagdollProfile3D& viewerProfile() const {
        return m_locomotion ? m_locomotion->profile() : m_viewer->body().profile;
    }
    [[nodiscard]] float viewerStepSeconds() const {
        return m_locomotion ? m_locomotion->policyStepSeconds() : m_viewer->policyStepSeconds();
    }

    void drawSidebar(float height);
    void drawTrainer();
    void drawControls(const TrainerStatus3D& status);
    void drawModes(const TrainerStatus3D& status);
    void drawDrills();
    void drawEvaluation(const TrainerStatus3D& status);
    void drawToolbar();
    void drawViewport(CanvasTarget& canvas, float bottomHeight);
    void drawCommands();
    void drawCharts(float height);
    void drawFooter();
    void notify(std::string text);
    [[nodiscard]] float ui(float value) const { return value * m_uiScale; }

    void scanRuns();
    void loadDrills();
    void ensureViewer();
    void resetViewer();
    void reloadPolicy(bool force);
    void applyManual();
    void stepViewer(float deltaSeconds);
    void stepOnce();
    [[nodiscard]] bool render(int width, int height);
    void pollModel();
    void refreshHistory();
    void frame(float yawDegrees, float pitchDegrees, float distance = 4.4f);
    void saveCapture();
    // O treino mostrado: o do treinador aberto no PC (so ha um; pode ter sido
    // aberto pelo terminal) ou, sem treinador, o escolhido nesta tela.
    [[nodiscard]] std::string activeRun() const;
    [[nodiscard]] std::filesystem::path taskPath() const;
    [[nodiscard]] std::filesystem::path policyPath() const;

    Options m_options;
    std::function<void()> m_onBack;
    TrainerPaths3D m_paths;
    std::unique_ptr<TrainerLink3D> m_link;
    std::vector<std::string> m_runs;
    std::string m_run;
    std::string m_mode = "equilibrado";
    float m_uiScale = 1.0f;

    // Os exercicios da tarefa do treino mostrado (ids e nomes) e o do visualizador.
    std::vector<std::string> m_drillIds;
    std::vector<std::string> m_drillNames;
    // A fase de cada exercicio (0: sem fase, no estilo de sempre).
    std::vector<int> m_drillPhases;
    // Os exercicios que a escada ainda vai ativar (nomes e fases): aparecem apagados e sem clique.
    std::vector<std::string> m_futureNames;
    std::vector<int> m_futurePhases;
    std::string m_viewerDrill;
    // O treino e a tarefa de que o mundo do visualizador foi feito.
    std::string m_viewerRun;
    std::filesystem::path m_viewerTask;

    // O mundo do visualizador e a politica dele.
    std::unique_ptr<BodyTrainingBatch3D> m_viewer;
    // Um treino da locomocao (o RL dentro dos modulos) no lugar do Corpo.
    // O mundo mostrado quando o treino e da locomocao ou do rastreador.
    std::unique_ptr<ViewerWorld3D> m_locomotion;
    // A escala de desenho do visualizador (1 = os pixels da tela; menos = o quadro
    // e desenhado menor e esticado): baixa sozinha quando o quadro passa de ~28 ms
    // (a tela grande ou de alta densidade, o treino disputando os nucleos) e sobe
    // quando sobra folga, em degraus e devagar (sem piscar).
    float m_renderScale = 1.0f;
    float m_renderMilliseconds = 0.0f;
    int m_renderScaleHold = 0;
    std::unique_ptr<PolicyRuntime3D> m_policy;
    std::filesystem::file_time_type m_policyWrite {};
    std::filesystem::path m_policyPath;
    std::string m_policyText;
    float m_policyWatch = 0.0f;
    float m_accumulator = 0.0f;
    // Onde a pelve aparece no ultimo quadro (0 a 1 na imagem; -1 fora dela).
    std::array<float, 2> m_pelvisOnScreen { -1.0f, -1.0f };
    // Onde o olho (o ponto para onde o comando manda olhar) aparece (0 a 1; -1 fora).
    std::array<float, 2> m_lookOnScreen { -1.0f, -1.0f };
    bool m_paused = false;
    struct ReplayFrame {
        RagdollState3D state;
        BodyWorldView3D view;
    };
    std::deque<ReplayFrame> m_replay;
    // O quadro (fracionario) mostrado da gravacao; < 0: o de agora.
    float m_playhead = -1.0f;
    float m_shownHead = -1.0f;
    float m_playbackSpeed = 1.0f;
    bool m_shownValid = false;
    RagdollState3D m_shownState;
    BodyWorldView3D m_shownView;
    std::vector<float> m_action;
    std::string m_viewerError;
    // O comando na mao (no lugar do piloto) e a velocidade dele.
    bool m_manual = false;
    float m_manualSpeed = 1.0f;

    // O jogador de verdade (carregado numa thread propria).
    std::optional<std::future<ModelOutcome>> m_pending;
    std::optional<Modeling::LoadedPlayerModel> m_model;
    std::unique_ptr<Modeling::RagdollSkeletonView> m_skeleton;
    std::optional<RagdollColliderWireframe3D> m_colliders;
    std::string m_modelError;

    // O desenho.
    Modeling::OrbitCamera m_camera;
    Modeling::Image m_image;
    bool m_dirty = true;
    Display m_display = Display::Full;
    bool m_showCenterOfMass = true;
    bool m_showContacts = true;

    // O historico (graficos) e os avisos.
    TrainingHistory3D m_history;
    // A leitura do banco roda fora da interface (semanas de treino sao muitas
    // linhas).
    std::optional<std::future<TrainingHistory3D>> m_historyPending;
    std::string m_historyRun;
    float m_historyWatch = 0.0f;
    // Os graficos no lugar do visualizador, um embaixo do outro, e a janela de tempo deles.
    bool m_chartsExpanded = false;
    int m_chartWindow = 1;
    std::string m_notice;
    float m_noticeSeconds = 0.0f;
};

} // namespace MatterEngine::Workbench
