#include "Workbench/PlayerInspector/PlayerInspectorScreen.hpp"

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Inspection/ReferenceComparison.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "Workbench/UiKit.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <optional>

namespace MatterEngine::Workbench {

using namespace Modeling;

namespace {

struct ToolbarView {
    std::string_view id;
    const char* label;
};

constexpr std::array<ToolbarView, 7> ToolbarViews { { { "frente", "Frente" }, { "costas", "Costas" },
    { "direita", "Direita" }, { "esquerda", "Esquerda" }, { "34", "3/4 dir." }, { "34e", "3/4 esq." },
    { "cima", "Cima" } } };

constexpr ImVec4 MutedText { 0.47f, 0.54f, 0.60f, 1.0f };
constexpr ImVec4 AccentText { 0.38f, 0.69f, 1.0f, 1.0f };
constexpr ImVec4 WarningText { 1.0f, 0.62f, 0.42f, 1.0f };

// Data e hora locais para o nome das capturas.
std::string timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm local {};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local);
    return buffer;
}

// O instante do tocador: minutos, segundos e centesimos.
std::string clockText(float seconds) {
    const int hundredths = static_cast<int>(std::lround(std::max(0.0f, seconds) * 100.0f));
    char buffer[24];
    std::snprintf(buffer, sizeof buffer, "%02d:%02d.%02d", hundredths / 6000, (hundredths / 100) % 60, hundredths % 100);
    return buffer;
}

} // namespace

PlayerInspectorScreen::PlayerInspectorScreen(Options options, std::function<void()> onBack)
    : m_options(std::move(options)), m_onBack(std::move(onBack)) {
    // Sem titulo, notas nem legenda na imagem: a tela ja diz o que e.
    InspectionStyle& style = m_session.editStyle();
    style.hud = false;
    style.gizmoLegend = false;
}

void PlayerInspectorScreen::ensureLoaded() {
    if (m_loadAttempted) return;
    m_loadAttempted = true;
    load(false);
}

void PlayerInspectorScreen::load(bool keepView) {
    if (m_pending) {
        m_reloadQueued = true;
        return;
    }
    PlayerModelOptions options;
    options.meshSource = m_meshSource;
    const std::filesystem::path manifest = m_options.manifest;
    m_pending = PendingLoad { std::async(std::launch::async,
                                  [manifest, options] {
                                      LoadOutcome outcome;
                                      outcome.loaded = loadPlayerModel(manifest, options, &outcome.error);
                                      return outcome;
                                  }),
        keepView };
}

void PlayerInspectorScreen::pollLoad() {
    if (!m_pending || m_pending->result.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    LoadOutcome outcome = m_pending->result.get();
    const bool keepView = m_pending->keepView;
    m_pending.reset();
    apply(std::move(outcome), keepView);
    if (m_reloadQueued) {
        m_reloadQueued = false;
        load(true);
    }
}

void PlayerInspectorScreen::waitUntilLoaded() {
    ensureLoaded();
    while (m_pending) {
        m_pending->result.wait();
        pollLoad();
    }
}

std::vector<std::filesystem::path> PlayerInspectorScreen::watchedFiles() const {
    if (!m_loaded) return {};
    std::vector<std::filesystem::path> files = playerModelFiles(m_loaded->manifest);
    files.insert(files.end(), m_loaded->sourceFiles.begin(), m_loaded->sourceFiles.end());
    return files;
}

void PlayerInspectorScreen::apply(LoadOutcome outcome, bool keepView) {
    if (!outcome.loaded) {
        m_error = outcome.error;
        return;
    }
    m_error.clear();
    m_loaded = std::move(outcome.loaded);
    if (m_meshSource.empty()) m_meshSource = m_loaded->manifest.meshSource;
    m_lastWrite = newestWrite(watchedFiles());
    m_referenceViews.clear();
    for (const ReferenceImage& reference : m_loaded->manifest.references)
        if (findViewPreset(reference.view) != nullptr) m_referenceViews.push_back(reference);
    if (m_reference >= static_cast<int>(m_referenceViews.size())) m_reference = -1;
    m_referenceDirty = true;
    // Recarregando, a sessao mantem a parte e a camera.
    m_session.setModel(m_loaded->model);
    if (!keepView) m_session.selectRegion(0, false);
    m_animations.setCharacter(m_loaded->character, m_loaded->rig);
    double milliseconds = 0.0;
    for (const auto& [stage, ms] : m_loaded->timings) milliseconds += ms;
    if (milliseconds > 0.0) {
        char text[64];
        std::snprintf(text, sizeof text, "Modelo montado em %.1f s", milliseconds / 1000.0);
        notify(text);
    }
}

void PlayerInspectorScreen::update(float deltaSeconds) {
    pollLoad();
    m_animations.poll();
    if (m_tab == 1) {
        if (!m_scrubbing) m_animations.update(deltaSeconds);
        if (m_animations.revision() != m_animationRevision) applyAnimationPose();
        // Tocando, a imagem rapida (a completa quando para).
        if (m_animations.playing() && m_animations.clip() != nullptr) m_session.keepFast();
    }
    m_session.update(deltaSeconds);
    if (m_noticeSeconds > 0.0f) m_noticeSeconds -= deltaSeconds;
    if (!m_loaded || m_pending) return;
    m_watchSeconds += deltaSeconds;
    if (m_watchSeconds < 1.0f) return;
    m_watchSeconds = 0.0f;
    if (const auto newest = newestWrite(watchedFiles()); newest > m_lastWrite) {
        m_lastWrite = newest;
        load(true);
        notify("Os arquivos mudaram: remontando o modelo");
    }
}

void PlayerInspectorScreen::setReference(int index) {
    m_reference = index >= 0 && index < static_cast<int>(m_referenceViews.size()) ? index : -1;
    m_referenceDirty = true;
}

void PlayerInspectorScreen::setMeshSource(const std::string& source) {
    m_meshSource = source;
    if (m_loadAttempted) load(true);
}

void PlayerInspectorScreen::notify(std::string text) {
    m_notice = std::move(text);
    m_noticeSeconds = 6.0f;
}

void PlayerInspectorScreen::setTab(int tab) {
    tab = std::clamp(tab, 0, 1);
    if (tab == m_tab) return;
    m_tab = tab;
    m_reference = -1;
    if (m_tab == 1) {
        m_animations.load();
        // O corpo inteiro: a animacao mexe tudo.
        if (m_session.selectedRegion() != 0) m_session.selectRegion(0);
        applyAnimationPose();
    } else {
        applyBends();
    }
}

void PlayerInspectorScreen::applyAnimationPose() {
    m_animationRevision = m_animations.revision();
    const InspectionModel* model = m_session.model();
    if (model == nullptr) return;
    std::optional<SkeletonPose> pose = m_animations.pose();
    if (pose && pose->joints.size() != model->skeleton.joints.size()) pose.reset();
    m_session.setPose(std::move(pose));
}

void PlayerInspectorScreen::draw(CanvasTarget& canvas, float uiScale) {
    m_uiScale = uiScale;
    ensureLoaded();
    pollLoad();

    beginFullscreenPanel("MatterEnginePlayerInspector");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, uiScale);
    const char* status = m_session.model() != nullptr ? nullptr : m_error.empty() ? "CARREGANDO" : "ERRO AO CARREGAR";
    drawWorkbenchHeader(drawList, io.DisplaySize, uiScale, "INSPEÇÃO DE JOGADOR", status);

    const float margin = ui(12.0f);
    const float top = headerHeight(uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float sidebar = std::max(sidebarWidth(uiScale), ui(300.0f));
    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("InspectorParts", { sidebar, height }, true, ImGuiWindowFlags_NoScrollbar);
    drawSidebar(height);
    ImGui::EndChild();

    const float left = margin * 2.0f + sidebar;
    ImGui::SetCursorPos({ left, top });
    ImGui::BeginChild("InspectorWorkspace", { io.DisplaySize.x - left - margin, height }, true,
        ImGuiWindowFlags_NoScrollbar);
    panelHeader("VISUALIZADOR");
    drawToolbar();
    drawViewport(canvas, m_tab == 1 ? ui(76.0f) : ui(30.0f));
    if (m_tab == 1) drawPlayerBar();
    drawFooter();
    ImGui::EndChild();
    ImGui::End();
}

void PlayerInspectorScreen::drawSidebar(float height) {
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (tabButton("Partes do corpo", m_tab == 0, { half, ui(32.0f) })) setTab(0);
    ImGui::SameLine();
    if (tabButton("Animações", m_tab == 1, { half, ui(32.0f) })) setTab(1);
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    if (m_tab == 0) drawRegionList(height);
    else drawAnimationList();
    const std::string back = UI::FontAwesome::label(UI::FontAwesome::ArrowLeft, "Menu principal");
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), height - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) }) && m_onBack) m_onBack();
}

void PlayerInspectorScreen::drawRegionList(float height) {
    const InspectionModel* model = m_session.model();
    const float listHeight = std::max(ui(120.0f), height - ui(552.0f));
    ImGui::BeginChild("InspectorRegionList", { -1.0f, listHeight }, false);
    if (model == nullptr) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(m_error.empty() ? MutedText : ImVec4 { 1.0f, 0.45f, 0.45f, 1.0f }, "%s",
            m_error.empty() ? "Carregando o modelo..." : m_error.c_str());
        ImGui::PopTextWrapPos();
    } else {
        for (std::size_t r = 0; r < model->regions.regions.size(); ++r) {
            ImGui::PushID(static_cast<int>(r));
            if (navigationButton(model->regions.regions[r].label.c_str(), static_cast<int>(r) == m_session.selectedRegion(),
                    { -1.0f, ui(34.0f) })) {
                m_session.selectRegion(static_cast<int>(r));
                m_reference = -1;
            }
            ImGui::PopID();
        }
    }
    ImGui::EndChild();
    drawBendPanel();
}

void PlayerInspectorScreen::drawAnimationList() {
    const float listHeight = std::max(ui(120.0f), ImGui::GetContentRegionAvail().y - ui(62.0f));
    ImGui::BeginChild("InspectorAnimationList", { -1.0f, listHeight }, false);
    if (!m_animations.loaded()) {
        ImGui::TextColored(MutedText, "%s", m_session.model() == nullptr ? "Carregando o modelo..." : "Carregando as animações...");
    } else {
        const std::vector<PlayerAnimations::Entry>& entries = m_animations.entries();
        std::string group;
        for (std::size_t i = 0; i < entries.size(); ++i) {
            if (entries[i].group != group) {
                group = entries[i].group;
                ImGui::Dummy({ 1.0f, ui(i == 0 ? 2.0f : 6.0f) });
                sectionLabel(group.c_str());
            }
            ImGui::PushID(static_cast<int>(i));
            ImGui::BeginDisabled(!m_animations.available(i));
            if (navigationButton(entries[i].label.c_str(), i == m_animations.selected(), { -1.0f, ui(30.0f) })) m_animations.select(i);
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        if (!m_animations.error().empty()) {
            ImGui::Dummy({ 1.0f, ui(6.0f) });
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(WarningText, "%s", m_animations.error().c_str());
            ImGui::PopTextWrapPos();
        }
    }
    ImGui::EndChild();
}

void PlayerInspectorScreen::drawPlayerBar() {
    const AnimationClip3D* clip = m_animations.clip();
    ImGui::BeginDisabled(clip == nullptr);
    const bool playing = m_animations.playing();
    const std::string play = playing ? UI::FontAwesome::label(UI::FontAwesome::Pause, "Pausar")
                                     : UI::FontAwesome::label(UI::FontAwesome::Play, "Tocar");
    if (ImGui::Button(play.c_str(), { ui(108.0f), ui(32.0f) })) m_animations.setPlaying(!playing);
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::RotateLeft, "Do início").c_str(), { ui(118.0f), ui(32.0f) })) {
        m_animations.seek(0.0f);
        m_animations.setPlaying(true);
    }
    ImGui::SameLine(0.0f, ui(14.0f));
    bool loop = m_animations.loop();
    ImGui::AlignTextToFramePadding();
    if (ImGui::Checkbox("Repetir", &loop)) m_animations.setLoop(loop);
    ImGui::SameLine(0.0f, ui(14.0f));
    static constexpr std::array<std::pair<const char*, float>, 4> Speeds { { { "¼×", 0.25f }, { "½×", 0.5f }, { "1×", 1.0f },
        { "2×", 2.0f } } };
    for (const auto& [label, speed] : Speeds) {
        if (tabButton(label, std::abs(m_animations.speed() - speed) < 1e-3f, { ui(44.0f), ui(32.0f) })) m_animations.setSpeed(speed);
        ImGui::SameLine();
    }
    ImGui::SameLine(0.0f, ui(14.0f));
    const float duration = m_animations.duration();
    const std::string clock = clockText(m_animations.time()) + " / " + clockText(duration);
    const float clockWidth = ImGui::CalcTextSize(clock.c_str()).x + ui(12.0f);
    float time = m_animations.time();
    ImGui::SetNextItemWidth(std::max(ui(120.0f), ImGui::GetContentRegionAvail().x - clockWidth));
    if (ImGui::SliderFloat("##InspectorTimeline", &time, 0.0f, std::max(duration, 1e-3f), "")) m_animations.seek(time);
    m_scrubbing = ImGui::IsItemActive();
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(MutedText, "%s", clock.c_str());
    ImGui::EndDisabled();
}

void PlayerInspectorScreen::drawBendPanel() {
    ImGui::Dummy({ 1.0f, ui(6.0f) });
    panelHeader("DOBRAR (TESTE)");
    ImGui::BeginDisabled(m_session.model() == nullptr);
    struct Control {
        const char* label;
        float minimum;
        float maximum;
    };
    static constexpr std::array<Control, 29> Controls { {
        { "Joelho D", 0.0f, 150.0f }, { "Joelho E", 0.0f, 150.0f }, { "Quadril D", -30.0f, 130.0f },
        { "Quadril E", -30.0f, 130.0f }, { "Abrir D", 0.0f, 60.0f }, { "Abrir E", 0.0f, 60.0f },
        { "Tornozelo D", -45.0f, 45.0f }, { "Tornozelo E", -45.0f, 45.0f }, { "Baixar braço D", -60.0f, 90.0f },
        { "Baixar braço E", -60.0f, 90.0f }, { "Braço à frente D", -45.0f, 90.0f }, { "Braço à frente E", -45.0f, 90.0f },
        { "Cotovelo D", 0.0f, 140.0f }, { "Cotovelo E", 0.0f, 140.0f }, { "Coluna", -30.0f, 45.0f },
        { "Pescoço", -30.0f, 40.0f }, { "Pulso D", -80.0f, 80.0f }, { "Pulso E", -80.0f, 80.0f },
        { "Pulso de lado D", -30.0f, 30.0f }, { "Pulso de lado E", -30.0f, 30.0f }, { "Dedos D", 0.0f, 100.0f },
        { "Dedos E", 0.0f, 100.0f }, { "Polegar D", -20.0f, 80.0f }, { "Polegar E", -20.0f, 80.0f },
        { "Pescoço de lado", -30.0f, 30.0f }, { "Pescoço girar", -50.0f, 50.0f }, { "Cabeça", -35.0f, 35.0f },
        { "Cabeça de lado", -25.0f, 25.0f }, { "Cabeça girar", -45.0f, 45.0f } } };
    // Quatro grupos (a lateral nao cabe todos): as pernas, o tronco e os
    // bracos, a cabeca com o pescoco e as maos (-1 = vazio).
    static constexpr std::array<const char*, 4> Groups { "Pernas", "Tronco", "Cabeça", "Mãos" };
    static constexpr std::array<std::array<int, 8>, 4> Members { { { 0, 1, 2, 3, 4, 5, 6, 7 },
        { 8, 9, 10, 11, 12, 13, 14, -1 }, { 15, 24, 25, 26, 27, 28, -1, -1 }, { 16, 17, 18, 19, 20, 21, 22, 23 } } };
    const float quarterGroup = (ImGui::GetContentRegionAvail().x - 3.0f * ImGui::GetStyle().ItemSpacing.x) / 4.0f;
    for (int group = 0; group < 4; ++group) {
        if (group > 0) ImGui::SameLine();
        if (navigationButton(Groups[static_cast<std::size_t>(group)], m_bendGroup == group, { quarterGroup, ui(26.0f) }))
            m_bendGroup = group;
    }
    bool changed = false;
    if (m_bendGroup == 3) {
        // A mao de partida: as do jogo (as dobras somam por cima).
        static constexpr std::array<const char*, 4> Hands { "Reta", "Solta", "Aberta", "Punho" };
        const float quarter = (ImGui::GetContentRegionAvail().x - 3.0f * ImGui::GetStyle().ItemSpacing.x) / 4.0f;
        for (int hand = 0; hand < 4; ++hand) {
            if (hand > 0) ImGui::SameLine();
            if (navigationButton(Hands[static_cast<std::size_t>(hand)], m_handPose == hand, { quarter, ui(24.0f) })) {
                m_handPose = hand;
                changed = true;
            }
        }
    }
    // Baixar o braco mostra o angulo do braco abaixo da horizontal (0 = a
    // pose T, -90 = para cima): a ligacao pode ser a pose A (o braco ja
    // baixado), e somar por cima dela passava o braco por dentro do tronco.
    const auto armDrop = [&](bool right) {
        const InspectionModel* model = m_session.model();
        if (model == nullptr) return 0.0f;
        const int upper = model->skeleton.find(right ? "RightUpperArm" : "LeftUpperArm");
        const int lower = model->skeleton.find(right ? "RightForearm" : "LeftForearm");
        if (upper < 0 || lower < 0) return 0.0f;
        const Vec3 along = model->skeleton.joints[static_cast<std::size_t>(lower)].bind.position
            - model->skeleton.joints[static_cast<std::size_t>(upper)].bind.position;
        return std::atan2(-along.z, std::abs(along.y)) / DegreesToRadians;
    };
    for (const int member : Members[static_cast<std::size_t>(std::clamp(m_bendGroup, 0, 3))]) {
        if (member < 0) continue;
        const std::size_t index = static_cast<std::size_t>(member);
        ImGui::PushID(static_cast<int>(index));
        ImGui::SetNextItemWidth(-ui(108.0f));
        if (index == 8 || index == 9) {
            const float drop = armDrop(index == 8);
            float shown = drop + m_bends[index];
            if (ImGui::SliderFloat(Controls[index].label, &shown, -90.0f, 85.0f, "%.0f°")) {
                m_bends[index] = shown - drop;
                changed = true;
            }
        } else {
            // Os dedos: quanto a mao fechou (ate o punho).
            const bool percent = index == 20 || index == 21;
            changed |= ImGui::SliderFloat(Controls[index].label, &m_bends[index], Controls[index].minimum,
                Controls[index].maximum, percent ? "%.0f%%" : "%.0f°");
        }
        ImGui::PopID();
    }
    if (ImGui::Button("Zerar dobras", { -1.0f, ui(28.0f) })) {
        m_bends.fill(0.0f);
        m_handPose = 0;
        changed = true;
    }
    ImGui::EndDisabled();
    if (changed) applyBends();
}

void PlayerInspectorScreen::setHandPose(int hand) {
    m_handPose = std::clamp(hand, 0, 3);
    applyBends();
}

void PlayerInspectorScreen::setBend(std::size_t index, float degrees) {
    if (index >= m_bends.size()) return;
    m_bends[index] = degrees;
    applyBends();
}

void PlayerInspectorScreen::applyBends() {
    const InspectionModel* model = m_session.model();
    if (model == nullptr) return;
    const Skeleton& skeleton = model->skeleton;
    // A mao de partida: a ligacao com os dedos da mao do jogo (reta = os da
    // ligacao); o corpo fica na ligacao.
    static constexpr std::array<RigHandPose, 4> Hands { RigHandPose::Bind, RigHandPose::Relaxed, RigHandPose::Open,
        RigHandPose::Fist };
    const SkeletonPose bind = bindPose(skeleton);
    const SkeletonPose start = m_handPose > 0 && m_loaded
        ? withGameHands(*m_loaded->character, *m_loaded->rig, bind, Hands[static_cast<std::size_t>(m_handPose)])
        : bind;
    // Cada eixo e o da partida e gira junto com o pai: o cotovelo dobra no
    // plano do braco mesmo com o braco levantado, a cabeca gira em volta do
    // pescoco inclinado, os dedos fecham para a palma com o pulso dobrado.
    std::vector<JointBend> bends;
    const auto add = [&](const std::string& joint, Vec3 axis, float degrees) {
        const int index = skeleton.find(joint);
        if (index >= 0 && degrees != 0.0f) bends.push_back({ index, axis, degrees * DegreesToRadians, true });
    };
    // A dobradica da junta (jointHingeAxis): + fecha.
    const auto hinge = [&](const std::string& joint, float degrees) {
        const int index = skeleton.find(joint);
        if (index >= 0) add(joint, jointHingeAxis(skeleton, start, index), degrees);
    };
    const auto at = [&](const std::string& joint) {
        const int index = skeleton.find(joint);
        return index >= 0 ? start.joints[static_cast<std::size_t>(index)] : Transform {};
    };
    // Joelho: + leva a canela para tras; quadril: a coxa para a frente
    // (-y) e para fora (x); tornozelo: + leva a ponta do pe para cima.
    hinge("RightShin", m_bends[0]);
    hinge("LeftShin", m_bends[1]);
    add("RightThigh", { 0.0f, 1.0f, 0.0f }, -m_bends[2]);
    add("LeftThigh", { 0.0f, 1.0f, 0.0f }, -m_bends[3]);
    add("RightThigh", { 1.0f, 0.0f, 0.0f }, -m_bends[4]);
    add("LeftThigh", { 1.0f, 0.0f, 0.0f }, m_bends[5]);
    add("RightFoot", { 0.0f, 1.0f, 0.0f }, -m_bends[6]);
    add("LeftFoot", { 0.0f, 1.0f, 0.0f }, -m_bends[7]);
    // Os bracos (o direito para -y, o esquerdo para +y; na ligacao do modelo
    // montado, ja 56 graus abaixo da horizontal): baixar gira em volta do x
    // do peito, a frente em volta do z; o cotovelo fecha na dobradica dele (o
    // plano do braco); a coluna (no abdomen) e o pescoco + para a frente.
    add("RightUpperArm", { 1.0f, 0.0f, 0.0f }, m_bends[8]);
    add("LeftUpperArm", { 1.0f, 0.0f, 0.0f }, -m_bends[9]);
    add("RightUpperArm", { 0.0f, 0.0f, 1.0f }, m_bends[10]);
    add("LeftUpperArm", { 0.0f, 0.0f, 1.0f }, -m_bends[11]);
    hinge("RightForearm", m_bends[12]);
    hinge("LeftForearm", m_bends[13]);
    add("Abdomen", { 0.0f, 1.0f, 0.0f }, m_bends[14]);
    add("Neck", { 0.0f, 1.0f, 0.0f }, m_bends[15]);
    // A cabeca: o pescoco de lado (em volta do x) e girando (em volta do z),
    // a cabeca para a frente, de lado e girando (em volta do pescoco).
    add("Neck", { 1.0f, 0.0f, 0.0f }, m_bends[24]);
    add("Neck", { 0.0f, 0.0f, 1.0f }, m_bends[25]);
    add("Head", { 0.0f, 1.0f, 0.0f }, m_bends[26]);
    add("Head", { 1.0f, 0.0f, 0.0f }, m_bends[27]);
    add("Head", { 0.0f, 0.0f, 1.0f }, m_bends[28]);
    // As maos: o pulso + dobra para a palma (o mesmo eixo dos dedos) e de
    // lado + para o polegar (em volta da normal da palma: o comprimento da
    // mao vai para o lado do indicador). Os dedos fecham da mao de partida
    // ate o punho do jogo (a fracao do controle, em cada falange o giro no
    // pai): de qualquer partida a mao termina fechada, sem passar da palma
    // (sem o jogo, dobram em volta do eixo de flexao de cada osso, a base 85%,
    // o meio 100% e a ponta 70% do valor). O polegar, girado em relacao aos
    // dedos, fecha atravessando a palma dos dois lados (a base 40%, o meio 80%
    // e a ponta 100%). Os eixos sao os da mao de partida.
    for (const char* side : { "Right", "Left" }) {
        const bool right = std::string_view(side) == "Right";
        const std::string hand = std::string(side) + "Hand";
        const Vec3 along = at(std::string(side) + "Middle1").position - at(hand).position;
        const Vec3 lateral = at(std::string(side) + "Index1").position - at(std::string(side) + "Pinky1").position;
        add(hand, fingerFlexionAxis(at(hand).orientation, false), m_bends[right ? 16 : 17]);
        add(hand, safeNormalized(cross(along, lateral), { 0.0f, 0.0f, 1.0f }), m_bends[right ? 18 : 19]);
    }
    const std::optional<SkeletonPose> fist = m_loaded
        ? std::optional<SkeletonPose>(withGameHands(*m_loaded->character, *m_loaded->rig, bind, RigHandPose::Fist))
        : std::nullopt;
    for (const char* side : { "Right", "Left" }) {
        const bool right = std::string_view(side) == "Right";
        const float fingers = m_bends[right ? 20 : 21], thumb = m_bends[right ? 22 : 23];
        if (fist) {
            const std::vector<JointBend> closing
                = blendJointBends(skeleton, start, *fist, fingerJoints(skeleton, right ? Side::Right : Side::Left, false), fingers / 100.0f);
            bends.insert(bends.end(), closing.begin(), closing.end());
        } else {
            for (const char* finger : { "Index", "Middle", "Ring", "Pinky" }) {
                hinge(std::string(side) + finger + "1", 0.85f * fingers);
                hinge(std::string(side) + finger + "2", fingers);
                hinge(std::string(side) + finger + "3", 0.7f * fingers);
            }
        }
        hinge(std::string(side) + "Thumb1", 0.4f * thumb);
        hinge(std::string(side) + "Thumb2", 0.8f * thumb);
        hinge(std::string(side) + "Thumb3", thumb);
    }
    if (bends.empty() && m_handPose == 0) {
        m_session.setPose(std::nullopt);
        return;
    }
    m_session.setPose(bendPose(skeleton, start, bends));
}

void PlayerInspectorScreen::drawToolbar() {
    const InspectionModel* model = m_session.model();
    ImGui::BeginDisabled(model == nullptr);
    for (const ToolbarView& view : ToolbarViews) {
        if (ImGui::Button(view.label, { ui(82.0f), ui(30.0f) })) {
            if (const ViewPreset* preset = findViewPreset(view.id)) m_session.applyView(*preset);
            m_reference = -1;
        }
        ImGui::SameLine();
    }
    const std::string recenter = UI::FontAwesome::label(UI::FontAwesome::Crosshairs, "Recentrar");
    if (ImGui::Button(recenter.c_str(), { ui(116.0f), ui(30.0f) })) {
        m_session.resetView();
        m_reference = -1;
    }

    // Modo de desenho, malha, referencia e captura.
    const SurfaceMode current = m_session.style().surface;
    ImGui::SetNextItemWidth(ui(170.0f));
    if (ImGui::BeginCombo("##InspectorMode", std::string(surfaceModeLabel(current)).c_str())) {
        for (const SurfaceMode mode : allSurfaceModes()) {
            if (ImGui::Selectable(std::string(surfaceModeLabel(mode)).c_str(), mode == current)) {
                InspectionStyle& style = m_session.editStyle();
                style.surface = mode;
                if (mode == SurfaceMode::Weights && style.weightJoint < 0 && model != nullptr)
                    for (std::size_t j = 0; j < model->jointRegion.size(); ++j)
                        if (model->jointRegion[j] == m_session.selectedRegion()) {
                            style.weightJoint = static_cast<int>(j);
                            break;
                        }
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    const char* meshLabels[] = { "Modelo montado", "Só os ossos" };
    const char* meshSources[] = { "montagem", "nenhuma" };
    int meshIndex = m_meshSource == "nenhuma" ? 1 : 0;
    ImGui::SetNextItemWidth(ui(250.0f));
    if (ImGui::Combo("##InspectorMesh", &meshIndex, meshLabels, 2)) {
        m_meshSource = meshSources[meshIndex];
        load(true);
    }
    ImGui::SameLine();
    const std::string referenceLabel = m_reference >= 0
        ? "Referência: " + m_referenceViews[static_cast<std::size_t>(m_reference)].label
        : std::string("Sem referência");
    ImGui::SetNextItemWidth(ui(280.0f));
    if (ImGui::BeginCombo("##InspectorReference", referenceLabel.c_str())) {
        if (ImGui::Selectable("Sem referência", m_reference < 0)) m_reference = -1;
        for (std::size_t i = 0; i < m_referenceViews.size(); ++i) {
            const std::string label = m_referenceViews[i].label + " (" + m_referenceViews[i].view + ")";
            if (ImGui::Selectable(label.c_str(), m_reference == static_cast<int>(i))) {
                m_reference = static_cast<int>(i);
                m_referenceDirty = true;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    const std::string save = UI::FontAwesome::label(UI::FontAwesome::Camera, "Salvar imagem");
    if (ImGui::Button(save.c_str(), { ui(150.0f), ui(30.0f) })) saveCapture();

    // O que aparece por cima.
    InspectionStyle style = m_session.style();
    bool changed = false;
    changed |= ImGui::Checkbox("Ossos", &style.skeleton);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Juntas", &style.joints);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Nomes", &style.jointLabels);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Grade", &style.grid);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Só a parte", &style.isolate);
    ImGui::SameLine();
    changed |= ImGui::Checkbox("Avesso em magenta", &style.backfaces);
    ImGui::SameLine();
    bool orthographic = m_session.orthographic();
    if (ImGui::Checkbox("Ortográfica", &orthographic)) m_session.setOrthographic(orthographic);
    if (style.surface == SurfaceMode::Weights && model != nullptr) {
        ImGui::SameLine();
        const std::string currentJoint = style.weightJoint >= 0
            ? model->skeleton.joints[static_cast<std::size_t>(style.weightJoint)].name : std::string("junta...");
        ImGui::SetNextItemWidth(ui(190.0f));
        if (ImGui::BeginCombo("##InspectorWeightJoint", currentJoint.c_str())) {
            for (std::size_t j = 0; j < model->skeleton.joints.size(); ++j) {
                const Joint& joint = model->skeleton.joints[j];
                if (joint.kind == JointKind::Tip) continue;
                if (ImGui::Selectable(joint.name.c_str(), style.weightJoint == static_cast<int>(j))) {
                    style.weightJoint = static_cast<int>(j);
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
    }
    // O sombreamento do modo solido: liso com quinas, ou de volta as faces.
    if (style.surface == SurfaceMode::Solid || style.surface == SurfaceMode::SolidWire) {
        ImGui::TextColored(MutedText, "Sombreamento");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ui(170.0f));
        changed |= ImGui::SliderFloat("Facetado##InspectorFacet", &style.facet, 0.0f, 1.0f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ui(170.0f));
        changed |= ImGui::SliderFloat("Quinas##InspectorCrease", &style.creaseAngle, 0.0f, 90.0f, "%.0f°");
        ImGui::SameLine();
        if (ImGui::SmallButton("Padrão##InspectorShading")) {
            const InspectionStyle defaults;
            style.facet = defaults.facet;
            style.creaseAngle = defaults.creaseAngle;
            changed = true;
        }
    }
    if (changed) m_session.editStyle() = style;
    ImGui::EndDisabled();
    ImGui::Dummy({ 1.0f, ui(4.0f) });
}

void PlayerInspectorScreen::drawViewport(CanvasTarget& canvas, float footerHeight) {
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 size { std::max(ui(160.0f), available.x), std::max(ui(160.0f), available.y - footerHeight) };
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##InspectorViewport", size,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const bool activated = ImGui::IsItemActivated();
    const bool deactivated = ImGui::IsItemDeactivated();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 end { origin.x + size.x, origin.y + size.y };
    drawList->AddRectFilled(origin, end, IM_COL32(6, 8, 11, 255), ui(4.0f));
    const float scaleX = io.DisplayFramebufferScale.x > 0.0f ? io.DisplayFramebufferScale.x : 1.0f;
    const float scaleY = io.DisplayFramebufferScale.y > 0.0f ? io.DisplayFramebufferScale.y : 1.0f;
    const int width = std::max(16, static_cast<int>(std::lround(size.x * scaleX)));
    const int height = std::max(16, static_cast<int>(std::lround(size.y * scaleY)));

    if (m_session.model() == nullptr) {
        const char* text = m_error.empty() ? "CARREGANDO O MODELO..." : "NÃO FOI POSSÍVEL CARREGAR O MODELO";
        const ImVec2 textSize = ImGui::CalcTextSize(text);
        drawList->AddText({ origin.x + (size.x - textSize.x) * 0.5f, origin.y + (size.y - textSize.y) * 0.5f }, UiMuted,
            text);
        return;
    }

    // Mouse: arrastar gira (esquerdo) ou move (direito/meio), a roda
    // aproxima, um clique sem arrasto escolhe a parte da junta mais perto e
    // o duplo clique recentraliza.
    if (activated) m_dragged = false;
    if (active) {
        const ImVec2 delta = io.MouseDelta;
        const bool moved = delta.x != 0.0f || delta.y != 0.0f;
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f)) {
            m_dragged = true;
            if (moved) {
                m_session.orbit(delta.x, delta.y);
                m_reference = -1;
            }
        }
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 3.0f) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 3.0f)) {
            m_dragged = true;
            if (moved) {
                m_session.pan(delta.x, delta.y, static_cast<int>(size.y));
                m_reference = -1;
            }
        }
    }
    if (hovered && std::abs(io.MouseWheel) > 0.0f) {
        m_session.zoom(io.MouseWheel);
        m_reference = -1;
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        m_session.resetView();
        m_reference = -1;
    } else if (deactivated && !m_dragged && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        m_session.click((io.MousePos.x - origin.x) * scaleX, (io.MousePos.y - origin.y) * scaleY, width, height);
    }

    if (m_reference >= 0 && m_reference < static_cast<int>(m_referenceViews.size())) {
        // A referencia com o modelo por cima, na escala dela (imagem fixa).
        bool changed = m_canvasContent != 2;
        if (m_referenceDirty) {
            const ReferenceImage& reference = m_referenceViews[static_cast<std::size_t>(m_reference)];
            std::string error;
            const std::optional<Image> image = readImage(reference.path.string(), &error);
            if (image) {
                const ReferenceComparison comparison = compareWithReference(m_session.scene(), *image,
                    *findViewPreset(reference.view), "Referência · " + reference.label);
                m_referenceOverlay = comparison.overlay;
                m_referenceText = comparison.hasMesh
                    ? "silhueta em comum com a referência: " + formatNumber(comparison.intersectionOverUnion * 100.0f, 1) + "%"
                    : "só os ossos sobre a referência (proporções)";
            } else {
                m_referenceOverlay = {};
                m_referenceText = error;
            }
            m_referenceDirty = false;
            changed = true;
        }
        if (!m_referenceOverlay.empty()) {
            const std::uint64_t id = canvas.present(m_referenceOverlay, changed);
            m_canvasContent = 2;
            const float fit = std::min(size.x / static_cast<float>(m_referenceOverlay.width()),
                size.y / static_cast<float>(m_referenceOverlay.height()));
            const ImVec2 imageSize { m_referenceOverlay.width() * fit, m_referenceOverlay.height() * fit };
            const ImVec2 imageOrigin { origin.x + (size.x - imageSize.x) * 0.5f, origin.y + (size.y - imageSize.y) * 0.5f };
            drawList->AddImage(static_cast<ImTextureID>(id), imageOrigin,
                { imageOrigin.x + imageSize.x, imageOrigin.y + imageSize.y });
        }
    } else {
        const bool rendered = m_session.render(width, height);
        const std::uint64_t id = canvas.present(m_session.image(), rendered || m_canvasContent != 1);
        m_canvasContent = 1;
        drawList->AddImage(static_cast<ImTextureID>(id), origin, end);
    }

}

void PlayerInspectorScreen::drawFooter() {
    if (m_noticeSeconds > 0.0f && !m_notice.empty()) ImGui::TextColored(AccentText, "%s", m_notice.c_str());
    else if (m_reference >= 0 && !m_referenceText.empty()) ImGui::TextColored(MutedText, "%s", m_referenceText.c_str());
}

void PlayerInspectorScreen::saveCapture() {
    const bool reference = m_reference >= 0 && !m_referenceOverlay.empty();
    const Image& image = reference ? m_referenceOverlay : m_session.image();
    if (image.empty() || m_session.model() == nullptr) return;
    const InspectionModel& model = *m_session.model();
    const int region = std::clamp(m_session.selectedRegion(), 0, static_cast<int>(model.regions.regions.size()) - 1);
    const std::filesystem::path path = m_options.capturesDirectory
        / ("captura_" + timestamp() + "_" + model.regions.regions[static_cast<std::size_t>(region)].id + ".png");
    std::string error;
    if (writePng(image, path.string(), &error)) notify("Imagem salva: " + path.string());
    else notify("Falha ao salvar: " + error);
}

} // namespace MatterEngine::Workbench
