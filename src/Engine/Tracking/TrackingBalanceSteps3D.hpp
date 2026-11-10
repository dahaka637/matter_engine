#pragma once

#include "Engine/Athlete/Planning/FootstepPlanner3D.hpp"
#include "Engine/Tracking/TrackingBalance3D.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace MatterEngine {

// Os canais da rede para o passo (depois dos eixos da acao, crus em -1..1; zero = a captura pura decide).
struct TrackingStepCommand3D {
    // Antecipar (+) ou segurar (-) o passo de recuperacao: desloca o limiar da margem.
    float go = 0.0f;
    // O pouso a frente (+) e a esquerda (+) do sugerido pela captura, no rumo do corpo.
    float landForward = 0.0f;
    float landLeft = 0.0f;
    // O passo mais demorado (+) ou mais rapido (-).
    float time = 0.0f;
};

struct TrackingBalanceStepsInput3D {
    TrackingBalance3D balance;
    // Os pes de agora (a origem do link e a orientacao, no mundo) e onde o idle os quer agora (o quadro do idle posto na
    // pelve do corpo, no rumo dele).
    std::array<Vec3, 2> foot {};
    std::array<Quaternion, 2> footOrientation {};
    std::array<Vec3, 2> idleFoot {};
    std::array<Quaternion, 2> idleOrientation {};
    Vec3 pelvis;
    float facingYaw = 0.0f;
    float groundHeight = 0.0f;
    TrackingStepCommand3D command;
    float deltaTime = 1.0f / 30.0f;
};

// O gerador dos passos de recuperacao e de estabilizacao do idle (04/11 15h, vault
// 04_locomocao/plano_passos_de_recuperacao_04_11.md): um passo humano pronto que a rede comanda. O pe apoiado fica onde
// esta; quando o ponto de captura passa da borda da base (o limiar deslocado pela rede), o pe que melhor captura faz o
// arco ate o pouso que segura o ponto de captura previsto no fim do passo (mais a folga e o deslocamento da rede); quieto
// com a base diferente da do idle, um passo calmo devolve o pe a ela (o de acomodacao do giro do modo padrao). As pernas
// vao aos pes pelo IK de quem usa (synthesizeAthleteLegs3D); a pelve da referencia vai para cima da base (rootShift).
// So pose: nada aqui aplica forca.
class TrackingBalanceSteps3D final {
public:
    // O estado para a observacao: o pe no ar (esquerdo, direito), o progresso do passo, o pouso planejado contra o pe no
    // ar (frente, esquerda), o pouso sugerido pela captura contra o centro de massa, o tempo desde o ultimo pouso (ate 1 s)
    // e a base contra a do idle (o pe direito contra onde o idle o quer em relacao ao esquerdo: frente, esquerda).
    static constexpr std::size_t FeatureCount = 10;
    // Os limiares e medidas (m, s).
    static constexpr float TriggerMargin = -0.01f, TriggerShift = 0.05f, LandShift = 0.15f, CaptureOffset = 0.05f;
    static constexpr float StepSeconds = 0.30f, StepSecondsShift = 0.07f, SideGap = 0.12f, MaximumReach = 0.60f,
                           MaximumStep = 0.75f, MinimumDoubleSupport = 0.08f;
    // (15h50, a sonda do parado: com 6 cm e 2,5 cm ele pisoteava - 2,6 passos de estabilizacao em 6 s -, o pouso do PD erra 4-5
    // cm e a base media no rumo da pelve, que balanca no idle, "girava" sozinha.)
    // (05/11 02h10, testado e NAO adotado: margem > 0, < 0,25 m/s por 0,25 s e a volta em 1 s - depois de levantar os passos
    // calmos 1,6 -> 4,6 por episodio, quieto 35 -> 27%, os de recuperacao iguais (~7); no parado pisoteava, 1,3 -> 3,0.)
    static constexpr float SettleMargin = 0.03f, SettleSpeed = 0.15f, ReleaseSeconds = 0.3f;
    static constexpr float SettleDrift = 0.08f, SettleQuietSeconds = 0.5f, SettleSeconds = 0.36f, SettleLift = 0.05f,
                           ReleaseDrift = 0.05f, RootPull = 0.05f;
    // O pe com mais que isto da carga nao sai (o outro passa) e nao ha passo com o corpo subindo mais rapido que isto (m/s).
    static constexpr float MaximumRiseSpeed = 0.4f;
    // (Ajustaveis para as sondas: a carga maxima do pe que sai - 1 desliga a trava - e o peso da carga na escolha.)
    float maximumSwingLoad = 0.8f, loadWeight = 0.4f;

    void reset();
    // Uma vez por decisao, antes dos alvos dela.
    void update(const RagdollProfile3D& profile, std::array<std::uint32_t, 2> footLinks, const TrackingBalanceStepsInput3D& input);
    // O alvo de cada pe daqui a `ahead` s (o apoiado onde esta, plano no chao; o do passo no arco).
    void feet(float ahead, std::array<FootTarget3D, 2>& out) const;
    // Quanto a pelve da referencia vai para cima da base (no plano, ja limitado e pesado).
    [[nodiscard]] Vec3 rootShift() const { return m_rootShift; }
    // O peso do IK sobre as pernas do clipe (0: o clipe puro).
    [[nodiscard]] float weight() const { return m_weight; }
    // O pe no ar (-1 nenhum).
    [[nodiscard]] int swinging() const { return m_swing; }
    [[nodiscard]] bool recoverySwing() const { return m_swing >= 0 && m_recovery; }
    // Nesta decisao comecou um passo de recuperacao (o custo do passo) / de estabilizacao.
    [[nodiscard]] bool recoveryStarted() const { return m_recoveryStarted; }
    [[nodiscard]] bool settleStarted() const { return m_settleStarted; }
    // A distancia da base de agora a do idle (m).
    [[nodiscard]] float stanceError() const { return m_stanceError; }
    void features(std::span<float> out) const;
    std::uint32_t recoverySteps = 0, settleSteps = 0;

private:
    struct Plan {
        bool valid = false;
        int side = -1;
        Vec3 to;
        float score = -1.0e9f;
    };
    [[nodiscard]] Plan plan(const RagdollProfile3D& profile, std::array<std::uint32_t, 2> footLinks,
        const TrackingBalanceStepsInput3D& input, int only) const;
    void begin(const RagdollProfile3D& profile, std::uint32_t link, const TrackingBalanceStepsInput3D& input, int side, Vec3 to,
        float seconds, float lift, bool recovery);

    std::array<Vec3, 2> m_anchor {};
    std::array<Quaternion, 2> m_anchorOrientation {};
    int m_swing = -1;
    bool m_recovery = false;
    Vec3 m_from, m_to;
    Quaternion m_fromOrientation, m_toOrientation;
    float m_elapsed = 0.0f, m_duration = StepSeconds, m_lift = 0.07f;
    float m_sinceLanding = 1.0f, m_quietSeconds = 0.0f;
    float m_weight = 0.0f, m_stanceError = 0.0f;
    Vec3 m_rootShift;
    Vec3 m_suggested;
    Vec3 m_center;
    float m_yaw = 0.0f;
    Vec3 m_stanceOffset;
    std::array<Vec3, 2> m_foot {};
    bool m_recoveryStarted = false, m_settleStarted = false, m_started = false;
    int m_touches = 0;
};

} // namespace MatterEngine
