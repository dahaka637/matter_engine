#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"

#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {

// A aba "Animacoes" da inspecao de jogador, sem interface: os clipes que o
// personagem usa no jogo (os papeis do manifesto, na ordem do jogo, com o
// nome da tela e o grupo), carregados numa thread propria, e a reproducao -
// tocar, pausar, repetir, a velocidade e o instante - com a pose do esqueleto
// da modelagem nele (a mesma do jogo: sampleGameRigPose, a mao de repouso,
// sem andar).
class PlayerAnimations {
public:
    struct Entry {
        std::string clipId;
        std::string label;
        std::string group;
    };

    PlayerAnimations();
    ~PlayerAnimations();

    // O personagem do modelo carregado (a lista sai dele). O mesmo personagem
    // de novo (uma recarga) mantem os clipes, o escolhido e o instante.
    void setCharacter(std::shared_ptr<const RagdollCharacter3D> character, std::shared_ptr<const Modeling::GameRig> rig);
    // Comeca a carregar os clipes (uma vez por personagem).
    void load();
    // A carga terminada, se houver, entra; waitUntilLoaded espera por ela.
    void poll();
    void waitUntilLoaded();
    [[nodiscard]] bool loading() const { return m_pending.has_value(); }
    [[nodiscard]] bool loaded() const { return m_loaded; }
    [[nodiscard]] const std::string& error() const { return m_error; }

    [[nodiscard]] const std::vector<Entry>& entries() const { return m_entries; }
    [[nodiscard]] std::size_t selected() const { return m_selected; }
    // Escolhe o clipe: do comeco, tocando, repetindo se ele repete.
    void select(std::size_t index);
    // O escolhido (nulo antes da carga ou se ele faltou).
    [[nodiscard]] const AnimationClip3D* clip() const;
    // O clipe da entrada carregou.
    [[nodiscard]] bool available(std::size_t index) const { return index < m_clips.size() && m_clips[index].has_value(); }

    // Tocando, avanca o instante (sem repetir, para no fim).
    void update(float deltaSeconds);
    void setPlaying(bool playing);
    [[nodiscard]] bool playing() const { return m_playing; }
    void setLoop(bool loop) { m_loop = loop; }
    [[nodiscard]] bool loop() const { return m_loop; }
    void setSpeed(float speed) { m_speed = speed; }
    [[nodiscard]] float speed() const { return m_speed; }
    void seek(float seconds);
    [[nodiscard]] float time() const { return m_time; }
    [[nodiscard]] float duration() const;

    // A pose no instante (nada sem clipe). A revisao muda quando ela muda.
    [[nodiscard]] std::optional<Modeling::SkeletonPose> pose() const;
    [[nodiscard]] std::uint64_t revision() const { return m_revision; }

private:
    struct Loaded {
        std::vector<std::optional<AnimationClip3D>> clips;
        std::string error;
    };

    std::shared_ptr<const RagdollCharacter3D> m_character;
    std::shared_ptr<const Modeling::GameRig> m_rig;
    std::vector<Entry> m_entries;
    std::vector<std::optional<AnimationClip3D>> m_clips;
    std::optional<std::future<Loaded>> m_pending;
    bool m_loaded = false;
    std::string m_error;
    std::size_t m_selected = 0;
    float m_time = 0.0f;
    float m_speed = 1.0f;
    bool m_playing = true;
    bool m_loop = true;
    std::uint64_t m_revision = 1;
};

// Os clipes do personagem na ordem e com os nomes da tela.
[[nodiscard]] std::vector<PlayerAnimations::Entry> playerAnimationEntries(const RagdollCharacter3D& character);

} // namespace MatterEngine::Workbench
