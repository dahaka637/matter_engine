#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Character/BallControl3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// Visualizador de toque na bola (04/10): a simulacao do jogo num cenario
// isolado. Desde 06/10 (pedido do usuario: ver o movimento limpo) na
// locomocao padrao - a capsula, o corpo pela animacao (DefaultLocomotion3D)
// com o gesto do manejo por cima e a bola fisica na cena; o mesmo laco da
// sonda MatterLocomotionProbe. O jogador vem numa marcha, a bola aparece num ponto em
// relacao ao corpo e sai o pedido (drible ou chute, numa direcao) num
// instante escolhido da passada. Os quadros gravados mostram o movimento
// inteiro: no Workbench (menu inicial, "Toque na bola") e na ferramenta de
// linha de comando MatterBallPreview (desenhos para analise).

enum class BallPreviewGait3D : std::uint8_t {
    Idle,
    Trot,
    Sprint,
    StrafeLeft,
    StrafeRight,
    Backpedal,
    SprintStrafeLeft,
    SprintStrafeRight,
    Count
};

[[nodiscard]] const char* ballPreviewGaitName(BallPreviewGait3D gait);
// Identificador curto (linha de comando): parado, trote, sprint, lado-esq,
// lado-dir, recuo, sprint-esq, sprint-dir.
[[nodiscard]] const char* ballPreviewGaitKey(BallPreviewGait3D gait);
[[nodiscard]] bool parseBallPreviewGait(const std::string& key, BallPreviewGait3D& gait);

struct BallTouchPreviewSettings3D {
    BallPreviewGait3D gait = BallPreviewGait3D::Idle;
    // Fase do ciclo da marcha (0..1, a do clipe) no instante do pedido.
    float requestPhase = 0.0f;
    // A bola no pedido, em relacao ao corpo: x para a frente e y para a
    // esquerda a partir da pelve (m, no plano); z a altura do centro sobre o
    // chao.
    Vec3 ballOffset { 0.55f, 0.12f, 0.1098f };
    // A bola chega rolando junto com o jogador (a velocidade pedida da
    // marcha) ou parada.
    bool ballRolling = true;
    // A bola com esta velocidade no pedido (m/s, no referencial do corpo: x
    // para a frente, y para a esquerda) - a bola solta, cruzando, vindo.
    bool ballVelocitySet = false;
    Vec3 ballVelocity;
    BallActionKind3D kind = BallActionKind3D::Dribble;
    // Direcao do toque em relacao ao rumo do corpo (graus, + para a
    // esquerda; 0 para a frente).
    float directionDegrees = 0.0f;
    float power = 0.3f;
    float shotLift = 0.0f;
    float shotCurve = 0.0f;
    // (Antigo: o toque pela area em volta do jogador; ignorado.)
    bool areaTouch = false;
    // Quando a forca sai (V3): a fracao do tempo estimado ate o pe chegar na
    // bola (0 na hora, 1 no contato visual), como no laboratorio.
    float contactTimingFactor = 1.0f;
    // Variacao de movimento (vida) como no laboratorio; desligada o
    // movimento e o canonico.
    bool motionVariety = false;
    // Gravado antes e depois do pedido (s).
    float recordBeforeSeconds = 0.5f;
    float recordAfterSeconds = 1.6f;
};

struct BallTouchPreviewFrame3D {
    // Relativo ao pedido (s).
    float timeSeconds = 0.0f;
    std::vector<Vec3> linkPositions;
    std::vector<Quaternion> linkOrientations;
    // A pose alvo da locomocao (o que os motores perseguem; analise).
    std::vector<Vec3> targetLinkPositions;
    std::vector<Quaternion> targetLinkOrientations;
    bool ballPresent = false;
    Vec3 ballPosition;
    Quaternion ballOrientation;
    Vec3 ballVelocity;
    // Os pes (origem do link) e se estao plantados.
    std::array<Vec3, 2> feet {};
    std::array<bool, 2> planted {};
    // Fase de cada pe no planejador (FootPhase3D) e se o pe fisico toca o
    // chao / apoia.
    std::array<int, 2> footPhase {};
    std::array<bool, 2> touching {};
    std::array<bool, 2> supporting {};
    // O que o manejo pede neste tick.
    BallFootGoal3D footGoal;
    BallPlant3D plant;
    BallKickPose3D kick;
    BallControlPhase3D phase = BallControlPhase3D::Idle;
    int side = -1;
    float footGap = 0.0f;
    float gaitRate = 1.0f;
    bool touched = false;
    float cyclePhase = 0.0f;
    int locomotionState = 0;
};

struct BallTouchPreviewResult3D {
    bool valid = false;
    std::string error;
    std::vector<BallTouchPreviewFrame3D> frames;
    // Indices em frames: o pedido e o toque (-1 sem toque).
    std::size_t requestFrame = 0;
    int touchFrame = -1;
    // O toque: o pe (0 esquerdo, 1 direito), a superficie, a bola depois.
    int side = -1;
    BallTouchSurface3D surface = BallTouchSurface3D::Laces;
    Vec3 ballVelocityAfter;
    Vec3 ballSpinAfter;
    float touchDelaySeconds = -1.0f;
    // Por que nao tocou (0 tocou ou nada pedido, 1 bola longe, 2 o pe nao
    // chegou a tempo, 3 o pedido acabou sem toque).
    int missReason = 0;
    // O corpo no pedido: rumo, pelve, a direcao pedida (mundo) e a fase.
    float facingYawRadians = 0.0f;
    Vec3 pelvisAtRequest;
    Vec3 directionWorld { 1.0f, 0.0f, 0.0f };
    Vec3 ballAtRequest;
    float cyclePhaseAtRequest = 0.0f;
    // O clipe da marcha (para mostrar o quadro): id, quadros por segundo e
    // duracao.
    std::string gaitClipId;
    float gaitClipFramesPerSecond = 30.0f;
    float gaitClipSeconds = 0.0f;
    bool fell = false;
    float simulatedSeconds = 0.0f;
    float wallSeconds = 0.0f;
};

class BallTouchPreview3D {
public:
    // Copia o perfil e os clipes que a locomocao usa: a simulacao pode rodar
    // em outra thread.
    BallTouchPreview3D(const RagdollCharacter3D& character,
        std::span<const AnimationClip3D> clips);
    ~BallTouchPreview3D();
    BallTouchPreview3D(const BallTouchPreview3D&) = delete;
    BallTouchPreview3D& operator=(const BallTouchPreview3D&) = delete;

    // Uma simulacao inteira (alguns segundos de jogo). Nao reentrante: uma
    // por vez. cancel interrompe; progress vai de 0 a 1.
    [[nodiscard]] BallTouchPreviewResult3D run(const BallTouchPreviewSettings3D& settings,
        const std::atomic<bool>* cancel = nullptr,
        std::atomic<float>* progress = nullptr);

    [[nodiscard]] const RagdollProfile3D& profile() const { return m_profile; }
    [[nodiscard]] const CharacterLocomotionAnimations3D& animations() const { return m_animations; }
    // O clipe que a marcha toca (nullptr no parado sem clipe).
    [[nodiscard]] const AnimationClip3D* gaitClip(BallPreviewGait3D gait) const;

private:
    RagdollProfile3D m_profile;
    std::vector<AnimationClip3D> m_clips;
    CharacterLocomotionAnimations3D m_animations;
    struct Physics;
    std::unique_ptr<Physics> m_physics;
};

// Personagem e clipes do disco (a ferramenta de linha de comando).
struct BallPreviewAssets3D {
    RagdollCharacter3D character;
    std::vector<AnimationClip3D> clips;
};
[[nodiscard]] BallPreviewAssets3D loadBallPreviewAssets3D(const std::string& assetsDirectory);
// Os clipes do personagem nos papeis da locomocao (o mesmo mapeamento do
// laboratorio). Os ponteiros apontam para dentro de `clips`.
[[nodiscard]] CharacterLocomotionAnimations3D mapCharacterAnimations3D(const RagdollCharacter3D& character,
    const std::vector<AnimationClip3D>& clips);

} // namespace MatterEngine
