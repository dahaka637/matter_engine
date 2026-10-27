#pragma once

#include "Engine/Modeling/Inspection/InspectionSession.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"
#include "Workbench/PlayerInspector/PlayerAnimations.hpp"

#include <array>
#include <filesystem>
#include <functional>
#include <future>
#include <optional>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {

// A tela "Inspecao de jogador": duas abas na lateral - as partes do corpo (a
// visao geral e cada parte em close-up, com as dobras de teste) e as
// animacoes do jogo (os clipes do personagem, com o tocador embaixo do
// viewport) -, as vistas e os modos de desenho, e o viewport com a imagem da
// sessao de inspecao - o mesmo renderizador das ferramentas de linha de
// comando (MatterModelInspector). So desenha e repassa o mouse: o estado fica
// na InspectionSession. O modelo e montado numa thread propria (a interface
// nao trava): enquanto isso a tela mostra "carregando", ou o modelo de antes
// numa recarga.
class PlayerInspectorScreen {
public:
    struct Options {
        std::filesystem::path manifest;
        // Onde "Salvar imagem" grava.
        std::filesystem::path capturesDirectory;
    };

    PlayerInspectorScreen(Options options, std::function<void()> onBack);

    // Transicoes da camera e recarga quando os arquivos do modelo mudam.
    void update(float deltaSeconds);
    // A tela inteira, dentro do quadro do ImGui.
    void draw(CanvasTarget& canvas, float uiScale);

    [[nodiscard]] bool loaded() const { return m_loaded.has_value(); }
    [[nodiscard]] bool loading() const { return m_pending.has_value(); }
    // Automacao (testes, capturas): espera a carga em andamento e aplica.
    void waitUntilLoaded();
    [[nodiscard]] const std::string& error() const { return m_error; }
    [[nodiscard]] const Modeling::InspectionSession& session() const { return m_session; }
    // Automacao (testes, capturas): o mesmo que os controles da tela.
    void selectRegion(int region) { m_session.selectRegion(region); }
    void setReference(int index);
    void setMeshSource(const std::string& source);
    // O grupo de dobras (0 pernas, 1 tronco, 2 maos) e a mao de partida
    // (0 reta, 1 solta - a de repouso do jogo -, 2 aberta, 3 punho).
    void setBendGroup(int group) { m_bendGroup = group; }
    void setHandPose(int hand);
    // Uma dobra (o indice de m_bends, em graus), como o controle da tela.
    void setBend(std::size_t index, float degrees);
    [[nodiscard]] Modeling::InspectionSession& editableSession() { return m_session; }
    // A aba da lateral: 0 as partes do corpo, 1 as animacoes (o tocador
    // aparece embaixo do viewport).
    void setTab(int tab);
    // O corpo (cm, kg), como a secao "CORPO": o modelo remontado nele.
    void setBody(float heightCm, float massKg);
    [[nodiscard]] int tab() const { return m_tab; }
    [[nodiscard]] PlayerAnimations& animations() { return m_animations; }

private:
    struct LoadOutcome {
        std::optional<Modeling::LoadedPlayerModel> loaded;
        std::string error;
    };
    struct PendingLoad {
        std::future<LoadOutcome> result;
        bool keepView = false;
    };

    void ensureLoaded();
    // Comeca a montar em segundo plano (com uma em andamento, uma nova fica
    // para quando ela terminar).
    void load(bool keepView);
    // A carga terminada, se houver, entra na tela.
    void pollLoad();
    void apply(LoadOutcome outcome, bool keepView);
    // Os arquivos de que o modelo carregado depende (os do manifesto e os que
    // a montagem leu: o desenho da pintura, as fontes, os SVG, as imagens).
    [[nodiscard]] std::vector<std::filesystem::path> watchedFiles() const;
    void drawSidebar(float height);
    void drawRegionList(float height);
    // A altura e o peso do jogador (o corpo variavel): o modelo remontado no
    // corpo ao soltar o controle.
    void drawBodyPanel();
    // As dobras de teste (joelho, quadril, tornozelo): a malha deformada.
    void drawBendPanel();
    void applyBends();
    void drawAnimationList();
    // O tocador da animacao (embaixo do viewport, na aba das animacoes).
    void drawPlayerBar();
    // A pose da animacao no instante vai para a sessao.
    void applyAnimationPose();
    void drawToolbar();
    void drawViewport(CanvasTarget& canvas, float footerHeight);
    void drawFooter();
    void saveCapture();
    void notify(std::string text);
    [[nodiscard]] float ui(float value) const { return value * m_uiScale; }

    Options m_options;
    std::function<void()> m_onBack;
    Modeling::InspectionSession m_session;
    std::optional<Modeling::LoadedPlayerModel> m_loaded;
    std::optional<PendingLoad> m_pending;
    bool m_reloadQueued = false;
    std::string m_meshSource;
    bool m_loadAttempted = false;
    std::string m_error;
    float m_uiScale = 1.0f;
    // Recarga: a escrita mais nova dos arquivos do modelo.
    std::filesystem::file_time_type m_lastWrite {};
    float m_watchSeconds = 0.0f;
    // Referencia sobreposta (indice em m_referenceViews, -1 = nenhuma).
    std::vector<Modeling::ReferenceImage> m_referenceViews;
    int m_reference = -1;
    Modeling::Image m_referenceOverlay;
    bool m_referenceDirty = true;
    std::string m_referenceText;
    // O que o canvas tem agora (0 nada, 1 a sessao, 2 a referencia).
    int m_canvasContent = 0;
    // Arrasto em andamento (um clique sem arrasto escolhe a parte).
    bool m_dragged = false;
    std::string m_notice;
    float m_noticeSeconds = 0.0f;
    // Graus. Pernas: joelho, quadril para a frente, quadril abrindo,
    // tornozelo; tronco e bracos: baixar o braco, braco a frente, cotovelo
    // (direita e esquerda de cada), coluna e pescoco; as maos (pulso, de lado,
    // dedos, polegar); a cabeca (o pescoco de lado e girando, a cabeca para a
    // frente, de lado e girando). O grupo na tela (as maos sao o 3).
    std::array<float, 29> m_bends {};
    float m_bodyHeightCm = BaseBodyHeightMeters * 100.0f;
    float m_bodyMassKg = BaseBodyMassKg;
    int m_bendGroup = 0;
    // A mao de partida das dobras: reta (a ligacao), solta (a de repouso),
    // aberta ou punho (as do jogo).
    int m_handPose = 0;
    int m_tab = 0;
    PlayerAnimations m_animations;
    // A pose da animacao que a sessao tem (a revisao dela).
    std::uint64_t m_animationRevision = 0;
    // Arrastando a linha do tempo: o tocador nao avanca.
    bool m_scrubbing = false;
};

} // namespace MatterEngine::Workbench
