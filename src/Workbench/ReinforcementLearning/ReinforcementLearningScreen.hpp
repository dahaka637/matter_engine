#pragma once

#include "Engine/Modeling/Raster/Camera.hpp"
#include "Engine/Modeling/Raster/Image.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"
#include "Engine/Modeling/Rig/RagdollSkeletonView.hpp"
#include "Engine/RL/Environment/BodyTrainingBatch3D.hpp"
#include "Engine/RL/Policy/PolicyRuntime3D.hpp"
#include "Engine/RL/Supervisor/TrainerLink3D.hpp"
#include "Engine/RL/Supervisor/TrainingEvolution3D.hpp"
#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"
#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"
#include "Workbench/ReinforcementLearning/LocomotionViewer3D.hpp"
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <filesystem>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <set>
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
    // O resumo do bloco do levantar no painel TREINO (o teste com os treinos de verdade imprime).
    [[nodiscard]] std::string getUpLiveText() const;
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
    // Os graficos expandidos e o periodo deles (o mesmo do recolhido; 0..5: 1, 3, 6,
    // 12 e 24 horas e desde o inicio).
    void setChartsExpanded(bool expanded, int window) {
        m_chartsExpanded = expanded;
        m_chartWindow = std::clamp(window, 0, 5);
        m_historyWatch = 0.0f;
    }
    void setShowRequirements(bool show) { m_showRequirements = show; }
    // O que os graficos desenham agora (a evolucao e as series do exercicio mostrado).
    [[nodiscard]] const TrainingCharts3D& chartData() { return charts(m_chartsExpanded ? 240 : 160); }
    void push(int direction, float newtons);
    // O comando na mao (m/s para a frente da camera do piloto) ou, com
    // std::nullopt, o piloto da tarefa.
    void setManualSpeed(std::optional<float> metersPerSecond);
    // (12/11 05h30) A caixa "Empurroes" do visualizador (os testes: o fogo do levantar desenhado).
    void setViewerPushes(bool on);
    // A caixa no visualizador aberto agora (false sem visualizador do levantar v2).
    [[nodiscard]] bool viewerPushes() const;
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
    // A locomocao: recolhido, a evolucao; expandido, um grafico para cada coisa.
    void drawLocomotionCharts(float height);
    // Os treinos antigos (o Corpo, o motor, o rastreador): os graficos de antes.
    void drawLegacyCharts(float height);
    // O levantar do chao (o treino a parte, 31/10): as avaliacoes dele nas 64 quedas fixas, relidas quando o log muda.
    void drawGetUpCharts(float height);
    void refreshGetUpCurve();
    // O treino do levantar ao vivo no painel TREINO, junto do treino do canal (02/11, ele: "quero que no laboratorio
    // mostre corretamente os treinos do B21 e do levantar do chao"): o status.json e o ultimo teste do log dele.
    void refreshGetUpLive();
    void drawGetUpLive();
    // O seletor do periodo dos graficos (o mesmo no recolhido e no expandido).
    void drawChartWindow(float width);
    [[nodiscard]] const TrainingCharts3D& charts(std::size_t slices);
    // A evolucao real: a avaliacao da rede (um ponto por avaliacao, a linhagem inteira).
    [[nodiscard]] const TrainingEvaluationCharts3D& evaluationCharts();
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
    [[nodiscard]] std::filesystem::path getUpRunDirectory() const;
    [[nodiscard]] std::filesystem::path policyPath() const;

    Options m_options;
    std::function<void()> m_onBack;
    TrainerPaths3D m_paths;
    std::unique_ptr<TrainerLink3D> m_link;
    std::vector<std::string> m_runs;
    std::string m_run;
    std::string m_mode = "maximo";
    float m_uiScale = 1.0f;

    // Os exercicios da tarefa do treino mostrado (ids e nomes) e o do visualizador.
    std::vector<std::string> m_drillIds;
    std::vector<std::string> m_drillNames;
    // A fase de cada exercicio (0: sem fase, no estilo de sempre).
    std::vector<int> m_drillPhases;
    // A categoria de cada exercicio (vazia: a lista sem categorias) e o peso dele.
    std::vector<std::string> m_drillCategories;
    std::vector<float> m_drillWeights;
    // Da locomocao: o peso, a velocidade-meta e o levantar de cada exercicio (a evolucao).
    std::vector<TrainingDrillInfo3D> m_drillInfo;
    bool m_locomotionTask = false;
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
    // Se o treino a parte do levantar do chao esta rodando (o exercicio dele entre os ativos).
    bool m_getUpTraining = false;
    // As avaliacoes do levantar (de pe no fim, em %: no total e por lado; se levantou alguma vez; o tempo ate ficar
    // de pe, s) e as amostras de treino de cada uma.
    // Cada linha do log (as do treino, a cada ~70 s, e as "aval"), em ordem: o treino com ruido (de pe no fim das
    // quedas, por lado, e o de pe dos que comecam de pe) e o teste sem ruido (repetido ate o seguinte).
    struct GetUpCurve {
        std::string drill;
        // O levantar em movimento (chegou, de pe, velocidade rumo ao alvo) e o evitar a queda (evitou, o tempo para se
        // recompor, de pe no fim e a estabilidade), do treino.
        std::vector<float> reached, reachedTest, movingStood, towardSpeed, towardSpeedTest;
        std::vector<float> avoided, recomposeSeconds, avoidStanding, avoidStill;
        // (06/11, o modulo unico) As cinco medidas do exercicio mostrado (os rumos, as manobras, o levantar e o equilibrio):
        // ate o fim, o segundo grafico (a velocidade como % do pedido; no levantar o tempo, s; no equilibrio os passos), a
        // semelhanca, a estabilidade e a forca (no equilibrio o nivel dos empurroes), com o exigido de cada uma (o
        // "exigencias" do log); nos cenarios, levantou/evitou.
        std::vector<float> sceneryStood, scenerySeconds, sceneryAvoided;
        std::vector<float> sceneryImitation, sceneryStability, sceneryAssist;
        std::vector<float> sceneryRatio, requiredStood, requiredRatio, requiredImitation, requiredStability, requiredSeconds;
        // (06/11 07h) O teste fixo do exercicio, sem ruido e sem forca ("capacidade" a 0% das avaliacoes, repetida ate a
        // seguinte): % de pe e % da velocidade pedida.
        std::vector<float> capacityZero, capacitySpeed;
        // (06/11) O levantar pela avaliacao fixa.
        std::vector<float> evalStood, evalRose, evalSeconds, evalPose;
        // (07/11, ele: "fica medindo o quanto ele esta se aproximando do juiz" e "os levantares nao mostram os dados completos")
        // A nota do juiz no exercicio (% - (D + 1) / 2 nas avaliacoes) e o treino dos levantares (de pe, tempo, levantou).
        std::vector<float> judge, trainStood, trainSeconds, trainRose;
        // (11/11 15h, ele: "os graficos nao aparecem nos exercicios de levantares correndo") O levantar pelo clipe: a semelhanca com
        // o clipe no treino, o tempo para levantar (s; 12/11 03h20, ele: "um novo dado e grafico de velocidade para levantar, dizendo
        // quantos segundos demora... pode tirar o grafico voo de pe") e a velocidade do centro de massa na troca, no treino e no
        // teste; e se o treino tem o levantar pelo clipe (v2.levantar_unreal), lido junto com o log.
        std::vector<float> trainPose, trainRiseSeconds, evalRiseSeconds, trainHandSpeed, evalHandSpeed;
        // (12/11 04h40, a fase de fogo do levantar - ele: "no nosso grafico de % que conseguiu se levantar, ele levaria em conta isso, a
        // media de todos os cenarios") Com o fogo (o log traz <prefixo>sob_fogo): o levantou sob fogo no lugar do chegou ao fim, o estilo
        // do juiz no lugar da semelhanca e o recuperou depois de ir ao chao.
        std::vector<float> trainRecovered, evalRecovered;
        // (12/11 16h50, o fogo da locomocao) No grupo do exercicio mostrado: sob fogo nao caiu e o firme (%, treino) e o nivel L (0-1).
        std::vector<float> locoFireStood, locoFireFirm, locoFireLevel;
        // (12/11 18h55, ele: o grafico do sucesso "ter 2 linhas, a do limpo e a do fogo... para TODOS os exercicios, ate os de levantar") O
        // sucesso sob fogo do exercicio mostrado (o limpo fica em sceneryStood; no levantar, o firme limpo e o firme sob fogo).
        std::vector<float> sceneryStoodFire, trainFirmClean, trainFirmFire;
        // (12/11 19h) O teste sem ruido do levantar nos limpos (o firme dos limpos da avaliacao).
        std::vector<float> evalFirmClean;
        bool getUpFire = false;
        bool clipGetUp = false;
        // A locomocao na rede do levantar (na ordem de TrackingViewer3D::LocomotionDrills): o primeiro grafico e o sem
        // cair, no treino e no teste. (12: os rumos e, desde 01/11, as faixas da rl_final.)
        std::array<std::vector<float>, 12> locoMain, locoMainTest, locoUp, locoUpTest;
        // A hora de cada ponto (s, unix) e a janela de tempo com que foi lido (ChartWindows).
        std::vector<double> times;
        int window = -1;
        // A coluna como a do parado em alerta, de pe no fim (o treino de pe e o teste do parado).
        std::vector<float> spine, spineTest;
        std::filesystem::file_time_type write {};
    } m_getUpCurve;
    struct GetUpLive {
        bool known = false, alive = false;
        std::string run, state;
        std::uint64_t iteration = 0;
        double sps = 0.0;
        // O ultimo teste (sem ruido): levantou, tempo, em movimento e evitou (< 0: sem o exercicio); a hora dele.
        double stood = -1.0, seconds = -1.0, moving = -1.0, avoided = -1.0;
        std::string testTime;
        double assist = 0.0;
        // A mao de verdade no ultimo trecho do treino (N): a sustentacao media e o p90 (o segurar no tropeco); < 0 sem dado.
        double handMean = -1.0, handPeak = -1.0;
        // A forca auxiliar da corrida (< 0: sem a corrida).
        double runAssist = -1.0;
        double maneuverAssist = -1.0;
        // A forca dinamica por exercicio (v2.ajuda_exercicios) e a fatia da locomocao no treino.
        std::map<std::string, double> drillAssist;
        double locomotionShare = 0.0;
        // (11/11) O que o treino ao vivo treina: os exercicios com episodios na ultima linha dele ("cenarios") e as cabecas
        // congeladas (run.json v2) - o botao de quem nao treina fica cinza.
        std::set<std::string> drillsRun;
        bool frozenRecovery = false, frozenLocomotion = false;
    } m_getUpLive;
    [[nodiscard]] static std::string assistText(const GetUpLive& g);
    [[nodiscard]] std::string drillAssist(const std::string& id) const;
    float m_accumulator = 0.0f;
    // Onde a pelve aparece no ultimo quadro (0 a 1 na imagem; -1 fora dela).
    std::array<float, 2> m_pelvisOnScreen { -1.0f, -1.0f };
    // Onde o olho (o ponto para onde o comando manda olhar) aparece (0 a 1; -1 fora).
    std::array<float, 2> m_lookOnScreen { -1.0f, -1.0f };
    // O rotulo do empurrao na tela (fracao da imagem; newtons 0 sem empurrao).
    struct PushLabel {
        float x = 0.0f, y = 0.0f, newtons = 0.0f, fade = 0.0f;
    } m_pushLabel;
    // (12/11 05h30, ele: o fogo do levantar com "um visual muito mais bonito (seja criativo) que mostra de onde vem a pancada/empurrao, a
    // forca, direcao") Os rotulos do fogo na tela (fracao da imagem; a cor, a opacidade e se e grande) e o fogo do mundo mostrado (o
    // cartao no canto).
    struct FireLabel {
        float x = 0.0f, y = 0.0f;
        std::string text;
        std::array<int, 3> color { 255, 255, 255 };
        float alpha = 1.0f;
        bool big = false;
    };
    std::vector<FireLabel> m_fireLabels;
    GetUpFireView3D m_fireCard;
    // (08/11) O rastro da pelve na estrela (o ponto no chao e a velocidade, m/s).
    std::vector<std::pair<Vec3, float>> m_starTrail;
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
    float m_motorLook = 0.0f;
    float m_locomotionAssist = 0.0f;
    // A regua so manda depois que ele mexe nela (antes, a do treino).
    bool m_assistChosen = false;
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
    // O rumo pedido da ultima imagem (o termo da camera), o passo e o exercicio: os pulos dele a camera compensa.
    float m_headingTerm = 0.0f;
    bool m_headingValid = false;
    std::uint32_t m_headingSteps = 0;
    std::string m_headingDrill;
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
    // Os historicos dos ramos anteriores da linhagem (nao mudam: lidos uma vez; so a tarefa da leitura mexe nele).
    std::shared_ptr<std::map<std::string, TrainingHistory3D>> m_ancestorHistories
        = std::make_shared<std::map<std::string, TrainingHistory3D>>();
    // Cada leitura nova do banco (os graficos refazem as series so quando muda).
    std::uint64_t m_historyVersion = 0;
    // As series dos graficos e o que as fez (a leitura, o exercicio, as fatias).
    TrainingCharts3D m_chartData;
    std::uint64_t m_chartDataVersion = ~std::uint64_t { 0 };
    std::string m_chartDataDrill;
    std::size_t m_chartDataSlices = 0;
    TrainingEvaluationCharts3D m_evaluationData;
    std::uint64_t m_evaluationDataVersion = ~std::uint64_t { 0 };
    // Os graficos no lugar do visualizador, um embaixo do outro, e o periodo deles
    // (o mesmo recolhido e expandido; comeca em "desde o inicio").
    bool m_chartsExpanded = false;
    // (04/11 12h40, ele: no expandido, "mostrar exigencias" - o minimo para descer a forca auxiliar em cada grafico.)
    bool m_showRequirements = false;
    // (12/11 18h45) A caixa "Empurroes" marcada vale para todo exercicio que abrir (cada visualizador novo nasce com ela).
    bool m_viewerPushes = false;
    int m_chartWindow = 5;
    std::string m_notice;
    float m_noticeSeconds = 0.0f;
};

} // namespace MatterEngine::Workbench
