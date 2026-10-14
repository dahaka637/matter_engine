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

#include <array>
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
        std::string task = "corpo";
        // Lista tambem as tarefas com "menu": false (as antigas): so os testes.
        bool showUnlistedSkills = false;
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
    // Fixa a velocidade do jogador do mundo (m/s para frente) no lugar do ciclo
    // de demonstracao (automacao).
    void setWalkSpeed(float metersPerSecond);
    void setDisplay(Display display) {
        m_display = display;
        m_dirty = true;
    }
    // As habilidades (uma por tarefa em assets/rl/tasks) e a escolhida: o que o
    // mundo mostra (a postura dela) e o que "Iniciar" treina.
    [[nodiscard]] std::vector<std::string> skillTasks() const;
    void selectSkill(std::size_t index);
    // "Treinar esta habilidade": o treino aberto salva, fecha e reabre na escolhida.
    void trainSelectedSkill();
    [[nodiscard]] bool switchingSkill() const { return !m_switchTask.empty(); }
    [[nodiscard]] std::uint32_t viewerPosture() const;
    [[nodiscard]] float viewerWalkCommand() const { return m_viewer ? m_viewer->view(0).command.x : 0.0f; }
    [[nodiscard]] bool viewerHasTarget() const { return m_viewer && m_viewer->view(0).hasTarget; }
    [[nodiscard]] std::uint32_t viewerTargetsReached() const { return m_viewer ? m_viewer->view(0).targetsReached : 0u; }
    [[nodiscard]] std::uint32_t viewerSteps() const { return m_viewer ? m_viewer->view(0).steps : 0u; }
    [[nodiscard]] std::uint32_t viewerDrill() const { return m_viewer ? m_viewer->view(0).drill : 0u; }
    [[nodiscard]] std::array<float, 2> viewerPelvisOnScreen() const { return m_pelvisOnScreen; }
    [[nodiscard]] std::vector<std::string> skillDrills() const {
        return m_skill < m_skills.size() ? m_skills[m_skill].drills : std::vector<std::string> {};
    }
    void showDrill(const std::string& drill, int chest) { setViewerDrill(drill, chest); }
    // O visualizador com o professor (refaz o mundo dele) ou com a politica.
    void showTeacher(bool show);
    [[nodiscard]] bool showingTeacher() const { return m_showTeacher; }

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
    void updateRaceCamera(float seconds, bool snap);
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
    // A tarefa do mundo do visualizador: a da habilidade escolhida (a arena e o
    // termino dela: andando, o chao e grande).
    std::filesystem::path m_viewerTask;
    std::unique_ptr<PolicyRuntime3D> m_policy;
    // A entrada da politica montada pelos nomes (o canal do visualizador de
    // cada entrada dela, -1 = falta, entra com zero); vazio = pelo comeco.
    std::vector<int> m_policyMap;
    std::vector<float> m_policyInput;
    std::filesystem::file_time_type m_policyWrite {};
    std::filesystem::path m_policyPath;
    std::string m_policyText;
    bool m_useBest = false;
    // O visualizador mostra o professor (a locomocao fisica do jogo conduzindo
    // o corpo do RL) no lugar da politica.
    bool m_showTeacher = false;
    float m_policyWatch = 0.0f;
    float m_accumulator = 0.0f;
    // A camera da corrida (no mundo), em 3a pessoa presa ao mapa: o rumo ate
    // o alvo anda atras do pedido, sem pular quando um alvo troca pelo
    // seguinte (o pulo parecia o episodio recomecando).
    struct RaceCamera {
        bool valid = false;
        float yaw = 0.0f;
        std::uint32_t steps = 0;
    } m_raceCamera;
    // Onde a pelve aparece no ultimo quadro (0 a 1 na imagem; -1 fora dela).
    std::array<float, 2> m_pelvisOnScreen { -1.0f, -1.0f };
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
    // Andar: o jogador do mundo demonstra sozinho, num ciclo de velocidades
    // (parado, metade e o limite que o treino ja pede, parado). A automacao
    // pode fixar uma velocidade no lugar do ciclo.
    float m_walkSpeed = 0.0f;
    float m_demoSeconds = 0.0f;
    float m_demoLimit = 1.0f;
    bool m_walkOverride = false;

    struct Skill {
        std::string task;
        std::string name;
        // A postura principal (a de maior fatia na tarefa) e a bateria dela.
        std::string posture;
        std::string battery;
        // A velocidade maxima para frente (m/s; 0 = so parado) e a bateria do andar.
        float maxForward = 0.0f;
        std::string walkBattery;
        // A tarefa tem professor (o aprendizado guiado).
        bool teacher = false;
        // A tarefa corre ate um alvo no mundo (o marcador no chao).
        bool target = false;
        // Os exercicios da tarefa (os ids) e se ela tem o comando de olhar.
        std::vector<std::string> drills;
        std::vector<std::string> drillNames;
        // Os modos do peito de cada exercicio (bit 0 livre, 1 fixo, 2 girando):
        // um so, sem ramos; mais de um, um ramo por modo.
        std::vector<unsigned> drillChests;
        // O peito girando do exercicio vem com as forcas (o "girando + forca").
        std::vector<bool> drillForce;
        // O grupo de cada exercicio (vazio: sem grupo).
        std::vector<std::string> drillGroups;
        bool facing = false;
    };
    std::vector<Skill> m_skills;
    std::size_t m_skill = 0;
    bool m_skillChosen = false;
    // O que o visualizador mostra de uma tarefa com exercicios: qual deles e o
    // peito (livre ou fixo para a frente).
    std::string m_viewerDrill;
    // O peito no visualizador: 0 livre, 1 fixo, 2 girando.
    int m_viewerChest = 0;
    void setViewerDrill(const std::string& drill, int chest);
    void drawDrillTree(std::size_t index);
    // A troca de habilidade em andamento: fecha o treino e reabre nesta tarefa.
    std::string m_switchTask;
    std::string m_switchMode;
    std::string m_switchRun;
    float m_switchSeconds = 0.0f;
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
