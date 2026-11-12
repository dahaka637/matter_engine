#include "Workbench/PlayerInspector/PlayerAnimations.hpp"

#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"

#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace MatterEngine::Workbench {

namespace {

// O nome do clipe sem o comentario do fim ("Carrinho sentado (mais rapido)"
// vira "Carrinho sentado").
std::string plainName(const std::string& name) {
    const std::size_t open = name.rfind(" (");
    return open != std::string::npos && !name.empty() && name.back() == ')' ? name.substr(0, open) : name;
}

} // namespace

std::vector<PlayerAnimations::Entry> playerAnimationEntries(const RagdollCharacter3D& character) {
    std::vector<PlayerAnimations::Entry> entries;
    const auto add = [&](const std::string& clipId, const std::string& label, const char* group, std::uint32_t color = 0) {
        if (clipId.empty()) return;
        for (const PlayerAnimations::Entry& entry : entries)
            if (entry.clipId == clipId) return;
        entries.push_back({ clipId, label, group, color });
    };
    // O modo padrao troca alguns papeis (o sprint de costas): o dele primeiro.
    const auto defaultClip = [&](const char* role) {
        for (const auto& [name, clipId] : character.defaultLocomotionClipIds)
            if (name == role) return clipId;
        return std::string();
    };
    add(character.idleClipId, "Parado em alerta", "Parado");
    add(character.turnLeftClipId, "Giro à esquerda", "Parado");
    add(character.turnRightClipId, "Giro à direita", "Parado");
    add(character.walkClipId, "Corrida leve", "Corrida");
    add(character.walkBackwardClipId, "Corrida de costas", "Corrida");
    add(character.strafeLeftClipId, "Lateral à esquerda", "Corrida");
    add(character.strafeRightClipId, "Lateral à direita", "Corrida");
    add(character.runForwardArcLeftClipId, "Curva à esquerda", "Corrida");
    add(character.runForwardArcRightClipId, "Curva à direita", "Corrida");
    add(character.runBackwardArcRightClipId, "Curva de costas", "Corrida");
    add(character.sprintClipId, "Sprint", "Sprint");
    const std::string sprintBackward = defaultClip("sprintBackward");
    add(sprintBackward, "Sprint de costas", "Sprint");
    add(character.sprintBackwardClipId, sprintBackward.empty() ? "Sprint de costas" : "Sprint de costas (físico)", "Sprint");
    add(character.sprintStrafeLeftClipId, "Sprint lateral à esquerda", "Sprint");
    add(character.sprintStrafeRightClipId, "Sprint lateral à direita", "Sprint");
    add(character.idleToSprintClipId, "Arrancada", "Sprint");
    add(character.crouchToSprintClipId, "Arrancada agachada", "Sprint");
    add(character.jumpPrepClipId, "Preparo do pulo", "Pulo");
    add(character.jumpStandingClipId, "Pulo parado", "Pulo");
    add(character.jumpForwardClipId, "Pulo para a frente", "Pulo");
    add(character.jumpBackwardClipId, "Pulo para trás", "Pulo");
    add(character.jumpLeftClipId, "Pulo à esquerda", "Pulo");
    add(character.jumpRightClipId, "Pulo à direita", "Pulo");
    // Os carrinhos e os outros papeis do modo padrao: o nome sai do clipe
    // (depois da carga).
    for (const std::string& clipId : character.tackleClipIds) add(clipId, "", "No chão");
    add(character.standUpFrontClipId, "Levantar de bruços", "No chão");
    add(character.standUpBackClipId, "Levantar de costas", "No chão");
    for (const auto& [role, clipId] : character.defaultLocomotionClipIds) add(clipId, "", "Corrida");
    // (06/11, ele: "crie uma nova categoria e coloque essas animacoes boas que tu encontrou... para eu visualizar e entender
    // como sao os movimentos"; depois "adicione... todas as outras principais de locomocao e curvas... e deixa de cor diversa
    // no menu a de cada tipo de locomocao": as do Game Animation Sample da Epic, convertidas para o nosso esqueleto, so para
    // ver - fora da biblioteca do jogo, na subpasta delas - num grupo e numa cor por tipo.)
    struct UnrealKind { const char* prefix; const char* group; std::uint32_t color; };
    static constexpr UnrealKind unrealKinds[] = {
        { "stand_", "Unreal · Parado", IM_COL32(170, 195, 225, 255) },
        { "idle_", "Unreal · Parado", IM_COL32(170, 195, 225, 255) },
        { "walk_", "Unreal · Caminhada", IM_COL32(130, 215, 145, 255) },
        { "trote_", "Unreal · Trote (a corrida deles a 4,5 m/s)", IM_COL32(250, 225, 120, 255) },
        { "run_", "Unreal · Corrida (original)", IM_COL32(245, 195, 95, 255) },
        { "sprint_", "Unreal · Sprint (a nossa corrida)", IM_COL32(245, 125, 105, 255) },
        { "transition_", "Unreal · Transições", IM_COL32(195, 155, 245, 255) },
        { "ragdoll_flail_", "Unreal · Queda protegida", IM_COL32(95, 210, 215, 255) },
        { "ragdoll_reach_", "Unreal · Queda protegida", IM_COL32(95, 210, 215, 255) },
        { "ragdoll_getup_", "Unreal · Levantar", IM_COL32(240, 145, 205, 255) },
        { "ragdoll_poses_", "Unreal · No chão", IM_COL32(215, 215, 120, 255) },
    };
    const std::vector<std::string> unrealClips = Modeling::availableClipIds("referencias_unreal");
    for (const UnrealKind& kind : unrealKinds)
        for (const std::string& clipId : unrealClips)
            if (clipId.starts_with(std::string("referencias_unreal/") + kind.prefix))
                add(clipId, "", kind.group, kind.color);
    return entries;
}

PlayerAnimations::PlayerAnimations() = default;

PlayerAnimations::~PlayerAnimations() {
    if (m_pending) m_pending->wait();
}

void PlayerAnimations::setCharacter(std::shared_ptr<const RagdollCharacter3D> character, std::shared_ptr<const Modeling::GameRig> rig) {
    const bool same = m_character && character && m_character->id == character->id;
    m_character = std::move(character);
    m_rig = std::move(rig);
    if (same) {
        ++m_revision;
        return;
    }
    if (m_pending) m_pending->wait();
    m_pending.reset();
    m_entries = m_character ? playerAnimationEntries(*m_character) : std::vector<Entry> {};
    m_clips.clear();
    m_loaded = false;
    m_error.clear();
    m_selected = 0;
    m_time = 0.0f;
    ++m_revision;
}

void PlayerAnimations::load() {
    if (m_loaded || m_pending || m_entries.empty()) return;
    std::vector<std::string> ids;
    for (const Entry& entry : m_entries) ids.push_back(entry.clipId);
    m_pending = std::async(std::launch::async, [ids] {
        Loaded result;
        for (const std::string& id : ids) {
            std::string error;
            result.clips.push_back(Modeling::loadCharacterClip(id, &error));
            if (!result.clips.back() && result.error.empty()) result.error = id + ": " + error;
        }
        return result;
    });
}

void PlayerAnimations::poll() {
    if (!m_pending || m_pending->wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    Loaded result = m_pending->get();
    m_pending.reset();
    m_clips = std::move(result.clips);
    m_error = std::move(result.error);
    m_loaded = true;
    for (std::size_t i = 0; i < m_entries.size() && i < m_clips.size(); ++i)
        if (m_entries[i].label.empty()) m_entries[i].label = m_clips[i] ? plainName(m_clips[i]->displayName) : m_entries[i].clipId;
    select(std::min(m_selected, m_entries.empty() ? std::size_t { 0 } : m_entries.size() - 1));
}

void PlayerAnimations::waitUntilLoaded() {
    load();
    if (m_pending) m_pending->wait();
    poll();
}

void PlayerAnimations::select(std::size_t index) {
    if (index >= m_entries.size()) return;
    m_selected = index;
    m_time = 0.0f;
    m_playing = true;
    if (const AnimationClip3D* current = clip()) m_loop = current->loops;
    ++m_revision;
}

const AnimationClip3D* PlayerAnimations::clip() const {
    return m_selected < m_clips.size() && m_clips[m_selected] ? &*m_clips[m_selected] : nullptr;
}

float PlayerAnimations::duration() const {
    const AnimationClip3D* current = clip();
    return current != nullptr ? std::max(0.0f, current->durationSeconds) : 0.0f;
}

void PlayerAnimations::update(float deltaSeconds) {
    const float length = duration();
    if (!m_playing || length <= 0.0f) return;
    m_time += std::max(0.0f, deltaSeconds) * m_speed;
    if (m_time >= length) {
        if (m_loop) {
            m_time = std::fmod(m_time, length);
        } else {
            m_time = length;
            m_playing = false;
        }
    }
    ++m_revision;
}

void PlayerAnimations::setPlaying(bool playing) {
    // Tocar de novo um clipe que parou no fim recomeca.
    if (playing && !m_playing && duration() > 0.0f && m_time >= duration()) m_time = 0.0f;
    m_playing = playing;
    ++m_revision;
}

void PlayerAnimations::seek(float seconds) {
    m_time = std::clamp(seconds, 0.0f, duration());
    ++m_revision;
}

std::optional<Modeling::SkeletonPose> PlayerAnimations::pose() const {
    const AnimationClip3D* current = clip();
    if (current == nullptr || !m_character || !m_rig) return std::nullopt;
    // O fim de um clipe que nao repete e o ultimo quadro (a amostra do jogo
    // repete: no fim exato voltaria ao primeiro).
    const float length = duration();
    const float at = length > 0.0f ? std::min(m_time, length - 1e-4f) : 0.0f;
    return Modeling::sampleGameRigPose(*m_character, *m_rig, current, std::max(0.0f, at), Modeling::RigHandPose::Relaxed, true);
}

} // namespace MatterEngine::Workbench
