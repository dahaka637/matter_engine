#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"
#include "Engine/Tracking/TrackingBasePosture3D.hpp"

#include "Engine/Character/ProtectiveFall3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/MotionMatching/MotionMatchingDatabase3D.hpp"
#include "Engine/Animation/MotionMatching/MotionMatchingTrajectory3D.hpp"
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/Athlete/Body/GetUpPolicy3D.hpp"
#include "Engine/Athlete/Core/Kinematics3D.hpp"
#include "Engine/Athlete/Core/FootGeometry3D.hpp"
#include "Engine/Athlete/Synthesis/LegSynthesis3D.hpp"
#include "Engine/Physics/Articulation/ArticulationGravity3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingBalanceSteps3D.hpp"
#include "Engine/Tracking/TrackingClipGetUp3D.hpp"
#include "Engine/Tracking/TrackingClipReference3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmp3D.hpp"
#include "Engine/Tracking/TrackingScene3D.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979f;
// A queda: os musculos a esta fracao da rigidez do rastreador, segurando a pose sorteada com este ruido (rad), de
// 0,9 a 1,5 s; a pose comeca com o ponto mais baixo de 0,35 a 0,85 m do chao.
constexpr float FallStiffness = 0.15f;
constexpr float FallPoseNoise = 0.35f;
// Caido (31/10, o evitar a queda; ele: "nao deixar o peito ou costas ou cabeca tocar o chao... que ele use os joelhos,
// bracos enfim para evitar a queda e se recobrar"): o chao tocado pelo tronco (pelve, abdomen, peito), pelo pescoco ou
// pela cabeca; os pes, as pernas e os bracos podem apoiar. Cair depois de ja ter ficado de pe custa isto, uma vez por
// queda (alem do potencial que desce).
constexpr float FallPenalty = 10.0f;
// (13/11, settings.fallToCommand) Depois da queda que virou comando, o episodio dura pelo menos isto (s): levantar (~1-1,5 s) e seguir.
constexpr float FallResumeSeconds = 4.0f;
// O levantar em movimento: a troca de pe por este tanto (s), mesmo andando, e a parceira correndo ate o alvo ate esta
// velocidade (m/s; a do treino dela).
constexpr float MovingHandoffSeconds = 0.2f;
constexpr float PartnerRunSpeed = 3.5f;
// A flexao na queda vale ate este tanto depois do primeiro toque do tronco no chao (s): o chegar ao chao.
constexpr float FlexAfterTouchSeconds = 0.75f;
// Os pares da arrancada agachada pesam isto no discriminador (a referencia do levantar em movimento).
constexpr int SprintStartPairWeight = 6;
// O levantar em movimento: chegou ao alvo neste raio (m), e dai o pedido vira ficar parado.
constexpr float ArriveRadius = 1.0f;
// Recomposto depois do empurrao do evitar: de pe, parecido com o parado e quase parado. (0,5 na pose de 21h25, o largo
// mais o estreito: a soma dos angulos^2 ~0,55; era 0,6 so no largo, a soma ~1,0.)
constexpr float RecomposedPose = 0.5f;
constexpr float RecomposedSpeed = 0.3f;
// A entrega do corpo de pe a cabeca da locomocao (o idle): quieto ha tanto tempo (s).
constexpr float IdleHandoffSeconds = 0.3f;
// O parado obrigado a pose do idle (04/11 07h40, ele: "vai afastando as pernas com o tempo e cai para o lado... a semelhanca
// com o clipe obrigatoriamente muito mais alta"): um pe a mais de IdleFeetLimit do lugar dele no idle ou a pelve
// IdlePelvisDrop abaixo, por IdleLostSeconds, encerra com a multa da queda. Medido: firme, os pes a 2-6 cm e a pelve -0,3
// a -1,2 cm; desmontando (17% de forca), 12 cm em 2 s e 28 cm em 4 s, a pelve -6 cm.
// (07h55: os primeiros 2 s ficam de fora e o limite foi a 12 cm por 0,5 s - com 10 cm por 0,3 s desde o comeco, ~30% dos
// episodios morriam em 1,0-1,6 s no balanco da largada, que se acerta sozinho, e o parado piorava; a abertura lenta vai a 28 cm.)
constexpr float IdleFeetLimit = 0.12f, IdlePelvisDrop = 0.05f, IdleLostSeconds = 0.5f, IdleLockAfterSeconds = 2.0f;
// (07/11) A folga da quietude do parado por elo, rad/s: o respirar e o micro-equilibrio do idle nao custam (ver idleQuiet).
constexpr float IdleQuietSlack = 0.15f;
// (07/11) O topo do levantar: o centro de massa acima disto (m; de pe ~0,96) e a subida que ali nao custa (m/s) - ver riseOvershoot.
constexpr float RiseTopHeight = 0.80f, RiseTopSpeed = 0.35f;
// A rl_final: os passos de cada faixa (0 andar, 1 trote, 2 corrida, 3 sprint) - o clipe em ciclo e o rumo do
// deslocamento dele em relacao ao corpo (0 a frente, de 45 em 45 graus para a esquerda). O primeiro de cada faixa e o da
// avaliacao. Na troca do pedido, parar tem esta chance.
struct GaitEntry {
    const char* clip;
    std::uint32_t band;
    std::uint32_t heading;
};
// (03/11: so os clipes do modo padrao do jogo - character.json -, os cardinais; as diagonais, gaitFor.)
constexpr GaitEntry GaitTable[] {
    { "jog", 1, 0 }, { "jog_strafe_left", 1, 2 }, { "jog_backward", 1, 4 }, { "jog_strafe_right", 1, 6 },
    { "run_forward_medium", 2, 0 }, { "sprint_strafe_left", 2, 2 }, { "run_backward_trote", 2, 4 }, { "sprint_strafe_right", 2, 6 },
    { "sprint_maximo", 3, 0 }, { "sprint", 3, 0 }, { "sprint_strafe_left", 3, 2 }, { "sprint_backward_long", 3, 4 },
    { "sprint_strafe_right", 3, 6 },
};

// Uma rotacao uniforme (Shoemake, "Uniform random rotations", 1992).
Quaternion uniformRotation(std::mt19937& rng) {
    std::uniform_real_distribution<float> u(0.0f, 1.0f);
    const float u1 = u(rng), u2 = 2.0f * Pi * u(rng), u3 = 2.0f * Pi * u(rng);
    const float a = std::sqrt(1.0f - u1), b = std::sqrt(u1);
    return Quaternion { a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3) }.normalized();
}

float planarAngle(Vec3 v) { return std::atan2(v.y, v.x); }

// O vetor de rotacao como no backend (o log do quaternio com w >= 0) e a volta.
Quaternion rotationExp(Vec3 v) {
    const float angle = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return angle > 1.0e-7f ? Quaternion::fromAxisAngle({ v.x / angle, v.y / angle, v.z / angle }, angle) : Quaternion {};
}
Vec3 rotationLog(Quaternion q) {
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    const float norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 1.0e-7f) return {};
    const float scale = 2.0f * std::atan2(norm, q.w) / norm;
    return { q.x * scale, q.y * scale, q.z * scale };
}
float axisValue(Vec3 v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }
float& axisRef(Vec3& v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

// (05/11 21h05: com o laco "while (a > Pi) a -= 2 Pi" um angulo infinito ou enorme - um mundo que explode com o ruido -
// nunca terminava: a captura da troca forte travava a 375% de CPU. O resto exato, e 0 no que nao e finito.)
float wrapAngle(float a) {
    if (!std::isfinite(a)) return 0.0f;
    if (a >= -Pi && a <= Pi) return a;
    return std::remainder(a, 2.0f * Pi);
}

} // namespace

// (07/11) Um clipe de referencia como fonte do Motion Matching: a raiz e a pelve no chao com o rumo do corpo
// (trackingBodyYaw3D), os pontos sao os pes e a pelve (o centro de massa dos links), os eixos sao o lateral e o da frente da
// pelve - a mesma definicao da consulta do corpo simulado.
struct TrackingMotionPoseReader {
    const RagdollProfile3D* profile = nullptr;
    std::uint32_t pelvis = 0, chest = 0, feet[2] { 0, 0 };
    // (O rumo da raiz: a media dos rumos da pelve e do peito - na corrida eles giram em sentidos opostos a cada passada e a
    // pelve sozinha fazia a raiz balancar; a busca pulava entre os ciclos de frente e os de 20 graus, ~10 trocas por segundo.)
    void read(std::span<const PhysicsBodyState3D> links, MotionMatchingPose3D& out) const {
        const PhysicsBodyState3D& p = links[pelvis];
        out.rootPosition = { p.position.x, p.position.y, 0.0f };
        const float hips = trackingBodyYaw3D(profile->links[pelvis], p);
        const float shoulders = trackingBodyYaw3D(profile->links[chest], links[chest]);
        out.rootYaw = std::atan2(std::sin(hips) + std::sin(shoulders), std::cos(hips) + std::cos(shoulders));
        out.points = { rlLinkCenterOfMass3D(profile->links[feet[0]], links[feet[0]]),
            rlLinkCenterOfMass3D(profile->links[feet[1]], links[feet[1]]), rlLinkCenterOfMass3D(profile->links[pelvis], p) };
        const Quaternion r = (p.orientation * profile->links[pelvis].modelOrientation.conjugate()).normalized();
        out.axes = { r.rotate({ 0.0f, 1.0f, 0.0f }), r.rotate({ 1.0f, 0.0f, 0.0f }) };
    }
};

class TrackingMotionSource final : public MotionMatchingSource3D {
public:
    TrackingMotionSource(const TrackingClipReference3D& reference, bool loops, const TrackingMotionPoseReader& reader)
        : m_reference(&reference), m_loops(loops), m_reader(reader) {}
    float durationSeconds() const override { return m_reference->durationSeconds(); }
    bool loops() const override { return m_loops; }
    void sample(float seconds, MotionMatchingPose3D& out) const override {
        TrackingClipFrame3D frame;
        m_reference->frame(seconds, {}, 0.0f, frame);
        m_reader.read(frame.links, out);
    }

private:
    const TrackingClipReference3D* m_reference;
    bool m_loops;
    TrackingMotionPoseReader m_reader;
};

struct TrackingGetUpAmpTraining3D::Impl {
    TrackingGetUpAmpSettings3D settings;
    std::shared_ptr<const AthleteBundle3D> bundle;
    const RagdollProfile3D* profile = nullptr;
    // Os clipes do estilo (os de levantar primeiro, o parado, depois os da arrancada e da corrida) e as referencias.
    std::vector<std::unique_ptr<AnimationClip3D>> clips;
    std::vector<std::unique_ptr<TrackingClipReference3D>> references;
    std::size_t standingIndex = 0;
    std::size_t standingMirrorIndex = 0;
    // (08/11 16h20, setStopClips) As paradas da Unreal carregadas: o indice da referencia, o rumo do deslocamento no comeco
    // (rad, no quadro do clipe), a velocidade de entrada (m/s, os primeiros 0,2 s), quando a raiz assenta (s, < 0,2 m/s) e a
    // velocidade da raiz suavizada (0,23 s) a cada StopSample s, no quadro do clipe. stopSlots[faixa 0 trote, 1 corrida][rumo]
    // [pe 0 esquerdo, 1 direito]: a posicao em stopClipInfo (SIZE_MAX sem).
    struct StopClip {
        std::size_t clip = 0;
        float direction = 0.0f, entrySpeed = 0.0f, settle = 0.0f;
        std::vector<Vec3> velocity;
    };
    std::vector<StopClip> stopClipInfo;
    std::array<std::array<std::array<std::size_t, 2>, 8>, 2> stopSlots {};
    static constexpr float StopCadenceFloor = 0.5f, StopIdleSpeed = 0.5f;
    static constexpr bool StopIdleMirror = false;
    static constexpr float StopBodySpeed = 1.5f, StopBodyTimeout = 2.5f;
    static constexpr float StopSample = 1.0f / 60.0f, StopEntryWindow = 0.3f, StopClipBlend = 0.2f, StopSettleExtra = 0.3f;
    // (10/11 04h, settings.unrealStarts) As arrancadas da corrida da Unreal (corrida_start_<rumo>_<pe>, 8 rumos x 2 pes, no fim da
    // lista): a velocidade da raiz suavizada a cada StopSample s, o rumo do deslocamento e a saida - o instante em que ela chega a
    // StartExitFraction da velocidade de cruzeiro (a media do ultimo meio segundo), e a velocidade ali.
    struct UnrealStart {
        std::size_t clip = 0;
        float direction = 0.0f, exit = 0.0f, exitSpeed = 0.0f;
        std::vector<Vec3> velocity;
    };
    std::vector<UnrealStart> startInfo;
    // (10/11 08h20, settings.launchCalmArms) Os alvos dos motores do idle em alerta (a pose dos bracos calma da largada).
    std::vector<float> idleArmTargets;
    // (10/11, settings.spineVelocityFeedforward) Os indices da acao dos eixos acoplados do perfil (jointCouplings).
    std::vector<std::size_t> coupledAxes;
    // (10/11, settings.spineTone) Os links da coluna (Abdomen, Chest, UpperChest).
    std::vector<bool> spineLinks;
    // (17/11, settings.armTone) Os links dos bracos (UpperArm, Forearm, Hand).
    std::vector<bool> armLinks;
    std::array<std::array<std::array<std::size_t, 2>, 8>, 2> startSlots {};
    static constexpr float StartExitFraction = 0.92f, StartExitMinimum = 0.5f;
    // A arrancada agachada (crouched_to_sprint), quando carregada: o peso maior no estilo. E os clipes da arrancada e
    // da corrida, os comecos do tipo 5 (o RSI do DeepMimic/AMP: ele vive os estados de corrida e aprende a seguir).
    std::size_t sprintStartIndex = std::numeric_limits<std::size_t>::max();
    // O rolar pelo clipe: o levantar de costas e o espelho dele, e o eixo da pelve que aponta para cima no comeco dele.
    std::vector<std::size_t> rollIndices;
    Vec3 rollBellyAxis { 1.0f, 0.0f, 0.0f };
    std::vector<std::size_t> runStartIndices;
    // A rl_final: os passos carregados de cada faixa (o indice do clipe e o rumo do deslocamento, GaitTable).
    struct Gait {
        std::size_t clip = 0;
        std::uint32_t heading = 0;
    };
    std::array<std::vector<Gait>, 4> gaits;
    // (08/11) Os passos de antes e os dos ciclos na meta (setGaitCycles), e os indices do trote e da corrida de costas de cada.
    std::array<std::vector<Gait>, 4> baseGaits, cycleGaits;
    std::size_t baseRunBack = std::numeric_limits<std::size_t>::max(), baseJogBack = std::numeric_limits<std::size_t>::max();
    std::size_t cycleRunBack = std::numeric_limits<std::size_t>::max(), cycleJogBack = std::numeric_limits<std::size_t>::max();
    std::vector<std::string> clipNames;
    // (07/11) Os clipes carregados pela tarefa (os comecos, o rastreio); os de depois (addStyleClips) sao so do juiz.
    std::size_t taskClipCount = 0;
    // (07/11, o juiz com o Motion Matching) Os bancos do Game Animation Sample (o conjunto denso), os pares do juiz de cada
    // quadro, a categoria de cada grupo (a escolha dos bancos por mundo, o chooser deles) e a previsao da trajetoria.
    std::unique_ptr<MotionMatchingDatabase3D> motionDb;
    std::vector<std::unique_ptr<TrackingMotionSource>> motionSources;
    std::vector<std::vector<float>> motionPairs;
    std::vector<int> motionCategory;
    std::vector<std::size_t> motionQueryOffset;
    std::size_t motionQueryWidth = 0, motionMainGroup = 0;
    MotionMatchingMovementSettings3D motionMovement;
    TrackingMotionPoseReader motionReader;
    // (07/11, ele: "temporariamente, coloca no laboratorio os movimentos dos exercicios de forma cinematica, so para eu ver o
    // Motion Matching operando") O corpo posto no quadro que o Motion Matching escolhe a cada decisao, sem fisica; o indice
    // da referencia de cada fonte de cada grupo.
    bool kinematicMotion = false;
    std::vector<std::vector<std::size_t>> motionRefs;
    // (07/11, ele: "vamos fazer os clipes tipo nos faziamos antes, sem Motion Matching... ligar as trocas adequadamente" - so
    // para ver no laboratorio) A sequencia de clipes da Unreal de cada exercicio, por regra: a partida da direcao, o ciclo, a
    // virada do angulo e do lado na troca, a curva pelo raio, a parada, o parado, o giro. Os clipes pelo nome; das viradas, o
    // instante em que metade do giro acontece e o fim dele; das curvas, o raio.
    bool clipSequence = false;
    // (07/11, ele: "uma seccao de exercicios compostos... os da estrela de 8 pontas, peito livre e peito fixo, trote e corrida...
    // so para observar") A estrela na sequencia de clipes: 0 desligada, 1 peito fixo, 2 peito livre.
    int starDrill = 0;
    struct SequenceTurn {
        std::size_t clip;
        float degrees;
        bool left;
        float half, end;
        bool fast = false;
    };
    std::vector<SequenceTurn> sequenceTurns;
    std::vector<bool> sequenceUpper;
    std::size_t sequenceAbdomen = 0;
    std::vector<std::pair<std::size_t, float>> sequenceArcs[2];
    // (07/11, ele: "vamos treinar o levantar de costas e de brucos e... o de levantar correndo") Os comecos deitados: o quadro 0
    // das levantadas da Unreal - os do clipe (1) das que acabam em pe [de costas, de brucos]; os do levantar em movimento (3)
    // das que acabam correndo. Vazios: os comecos de antes.
    std::vector<std::size_t> lyingStand, lyingRun;
    // (11/11 14h, settings.clipGetUpFraction) Os quatro clipes do levantar pelo clipe (de costas, de brucos, correndo de costas,
    // correndo de brucos) e, de cada um, o rumo da raiz e a velocidade do centro de massa nos ultimos 0,2 s (no quadro do clipe): o
    // pedido do levantar correndo.
    std::vector<std::size_t> clipGetUpClips;
    std::vector<float> clipGetUpDirection, clipGetUpSpeed, clipGetUpVariants, clipGetUpStartBias;
    // (13/11 11h30, settings.getUpFormWeight) A forma da cabeca e do peito do clipe de cada exercicio a cada 1/30 s: a altura da cabeca,
    // a cabeca contra o peito (para a frente +, de lado) e a torcao do peito contra a pelve, em graus - setGetUpForm.
    // (12h25, a imagem do laboratorio: saindo do chao ele arqueia o peito para tras e olha o ceu - a cabeca contra o peito nao via isso)
    // Mais o peito contra a pelve (+ para a frente, - arqueado para tras) e o olhar (a frente da cabeca contra o horizonte, + para cima).
    // (12h40, ele: o de brucos "fica travado assim que levanta meio arqueado ate se posturar") Mais o peito contra a vertical (+ a frente,
    // no rumo da pelve): de pe curvado, com o peito igual ao do clipe contra a pelve, so isto ve.
    struct GetUpFormSample { float head = 0.0f, pitch = 0.0f, roll = 0.0f, twist = 0.0f, lean = 0.0f, gaze = 0.0f, tilt = 0.0f; };
    std::vector<std::vector<GetUpFormSample>> clipGetUpForm;
    // (14/11 20h10, settings.getUpLegWeight) De cada exercicio a cada 1/30 s do clipe: a altura da cabeca e o joelho (a canela) e o pe
    // esquerdos contra a pelve, nos eixos dela (m) - setGetUpLegForm.
    std::vector<std::vector<std::array<float, 8>>> clipGetUpLeg;
    std::array<std::size_t, 2> leftLegLinks { std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max() };
    // (12h50, medido: na janela de firmeza os de pe ficavam de pe agachados e curvados - a cabeca a 1,43-1,54 m e o peito a +30-38 graus
    // por 1,2 s, e so se endireitavam quando o idle assumia; o clipe acaba com a cabeca a 1,59-1,65 m e o peito a +17-26) De cada
    // exercicio, o fim do clipe (a media dos ultimos 0,3 s): a altura da cabeca e o peito contra a vertical.
    std::vector<std::array<float, 2>> clipGetUpStand;
    std::array<std::size_t, 2> handLinksForm { std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max() };
    // (12/11 00h35) O ponto de entrega de cada exercicio (s do clipe; <= 0: no fim dele) - setClipGetUpDeliver.
    std::vector<float> clipGetUpDeliver;
    // (12/11 02h50) O reforco da pose de cada exercicio (TrackingClipGetUp3D::Widths::emphasis) - setClipGetUpEmphasis.
    std::vector<float> clipGetUpEmphasis;
    // (12/11 03h20) O peso da agilidade de cada exercicio (TrackingClipGetUp3D::AgilityMaxRate; 0: o relogio do clipe so anda com o
    // tempo) - setClipGetUpAgility.
    std::vector<float> clipGetUpAgility;
    // (12/11 04h40) O rumo da frente do corpo no fim de cada clipe (rad, com o clipe no rumo 0) - os 8 rumos do fogo saem dele.
    std::vector<float> clipGetUpFacing;
    // (12/11 04h40, a fase de fogo do levantar - setGetUpFire) As fatias, os niveis (4 por exercicio) e a prova.
    std::array<float, 4> getUpFireShares { 1.0f, 0.0f, 0.0f, 0.0f };
    std::vector<float> getUpFireLevels;
    bool getUpFireExam = false;
    // (12/11 05h15, ele: "nos empurroes/pancadas... algumas variantes que vem de CIMA para baixo, mesmo que poucas") A fracao dos
    // cenarios com fogo em que a forca vem de cima: inclinada 45-80 graus para baixo (o rumo + 8 na coluna do rumo); a pancada no alto
    // do tronco (o peito alto ou um ombro). A quinta das fatias de setGetUpFire.
    float getUpFireAbove = 0.0f;
    static constexpr std::array<std::uint32_t, 3> FireHoldLinks { 0u, 2u, 3u };
    static constexpr std::array<float, 3> FireHoldShares { 0.20f, 0.32f, 0.48f };
    // O orcamento do levantar adaptativo: o ponto de entrega (ou o fim do clipe) mais isto; sem ficar firme, acaba (nao levantou).
    static constexpr float AdaptiveSpareSeconds = 3.5f;
    // (20h20, o rolamento livre) A hora em que cada clipe fica de brucos (freeRollUntil; nan = ainda nao medida).
    std::vector<float> freeRollUntilCache;
    // (20h40) Se o clipe comeca de costas (clipStartsSupine; -1 = ainda nao medido).
    std::vector<int> supineStartCache;
    // (15/11 13h30, a revisao do crash: os dois caches eram enchidos sem trava pelas threads dos mundos - o resize concorrente da o
    // memset em NULL e o double free dos segfaults; como o clipRolls, com trava.)
    std::mutex clipCacheMutex;
    // (12/11 01h05) O correndo voltou ao treino de antes das 20h31 - o clipe longo, a troca no fim dele e a locomocao (congelada) por
    // clipGetUpAfterSeconds: era essa continuacao que obrigava a chegar num estado de corrida de verdade (sem ela, de 20h31 a 01h00, a
    // pose do fim se perdeu: de 1,4 a 1,75 s os erros de 9-16 cm foram a 44-89 cm). O pedido dele - a estatistica sem as quedas falsas -
    // fica pela entrega no corte (setClipGetUpDeliver): ali conta a troca, e nada depois conta como queda.
    // (20h20, ele: "Os de levantar correndo bem no finalzinho ele baixa a cabeca e cai, apos ja estar de pe, vamos apenas encurtar o
    // treino para a parte que ele recem fica de pe posturado em forma de corrida ja impulsionado, assim nao polui a estatistica de
    // queda falsa") No levantar correndo o episodio acaba na troca (sem os clipGetUpAfterSeconds da locomocao congelada depois).
    // (18h, o quinto - de costas rolando: correndo e o exercicio cujo clipe termina acima de ClipRunEndSpeed, nao um indice fixo.)
    bool clipRunExercise(std::uint32_t e) const { return e < clipGetUpSpeed.size() && clipGetUpSpeed[e] > ClipRunEndSpeed; }
    static constexpr float ClipRunEndSpeed = 1.5f;
    // Os giros parados do modo padrao (turn_left, turn_right), quando carregados.
    std::array<std::size_t, 2> turnIndices { std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max() };
    // (04/11 23h55, ele: "a troca de direcao... observe o Running Turn 180 - precisamos adaptar ele, pois e movimento de 180, e
    // precisamos de um que sirva para todos os graus com um encaixe perfeito") A meia-volta correndo e o espelho dela (0: a
    // que gira para a direita, giro negativo; 1: a da esquerda); o giro da pelve dela desde o comeco, em RunTurnSamples + 1
    // instantes (rad, desenrolado), e o rumo do comeco (calculados no construtor, antes das threads - 05/11 21h50).
    std::array<std::size_t, 2> runTurnIndices { std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max() };
    std::array<std::vector<float>, 2> runTurnYaws;
    std::array<float, 2> runTurnHeading {};
    // O rastreio: a parte de cada link (a pelve, a perna esquerda, a direita, o tronco, os bracos).
    enum TrackingPart : std::uint8_t { PartRoot, PartLeftLeg, PartRightLeg, PartTrunk, PartArms, PartHead };
    std::vector<std::uint8_t> trackingParts;
    // Quantos links em cada parte (o erro medio por junta, em graus).
    std::array<float, 6> trackingPartCount {};
    // O clipe da arrancada de pe (idle_to_sprint), quando carregado.
    std::size_t startClipIndex = std::numeric_limits<std::size_t>::max();
    // (05/11 23h20) A arrancada agachada da saida da meia-volta (turnLaunch): so rastreada, fora do juiz da recuperacao.
    std::size_t launchExitIndex = std::numeric_limits<std::size_t>::max();
    // (So para o rastreio - sem o levantar em movimento, fora do juiz da recuperacao.)
    bool startClipTrackedOnly = false;
    // A corrida de costas e o trote de costas (04/11 05h15: a corrida de costas aceita os dois jeitos).
    std::size_t runBackIndex = std::numeric_limits<std::size_t>::max(), jogBackIndex = std::numeric_limits<std::size_t>::max();
    // As quedas com protecao carregadas (os indices).
    std::vector<std::size_t> protectionIndices;
    // O rumo medio da pelve de cada clipe em relacao a frente dele (01/11 22h30: no parado em alerta, nos rumos do trote
    // e nas corridas de lado a pelve fica ~22 graus virada; nos clipes de frente e de costas do Mixamo, ~0).
    std::vector<float> yawOffsets;
    // O agachado de cada clipe de levantar (s): o instante mais parecido com o comeco da arrancada agachada (ele, 23h35:
    // "o crouched_to_sprint... e justamente o intermedio do ato de levantar-se com ir para ficar em pe").
    std::vector<float> crouchTimes;
    // A velocidade de deslocamento que falta aos clipes no lugar (os pares do discriminador): constante (corrida) ou
    // em rampa desde zero (a arrancada, com o deslocamento da fonte), para a frente do clipe.
    struct Travel {
        float speed = 0.0f;
        float rampDisplacement = 0.0f;
        float direction = 0.0f;
        // O pouso do pe esquerdo (fracao do ciclo): a troca de clipe numa manobra alinha os ciclos por ele, como o modo
        // padrao.
        float leftLand = 0.0f;
        // (04/11 08h50: e o do direito - a arrancada acaba com o direito apoiado e o esquerdo no ar; o ciclo entra na fase
        // dele.)
        float rightLand = 0.0f;
    };
    std::vector<Travel> travel;
    std::unique_ptr<TrackingGetUpContract3D> contract;
    std::unique_ptr<TrackingAmpFeatures3D> features;
    std::unique_ptr<TrackingGetUpPostureProbe3D> posture;
    std::vector<std::string> observationNames;
    std::vector<std::string> actionNames;
    std::uint32_t rootLink = 0;
    // (08/11) Os ombros (os bracos de cima) para o empurrao num ombro.
    std::uint32_t leftShoulderLink = std::numeric_limits<std::uint32_t>::max(), rightShoulderLink = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t leftThighLink = std::numeric_limits<std::uint32_t>::max(), rightThighLink = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t footLinks[2] { 0, 0 };
    // A mao do tronco de cima (04/11 05h45): o peito alto, a cabeca e os bracos, com o ganho, o amortecimento e o teto do
    // torque (N.m a 100%) de cada um; o link fora do perfil fica sem.
    // (04/11 06h00, ele: "garantir que a forca auxiliar atue em todos os ossos do corpo": a lista do corpo todo - a coluna,
    // o pescoco, a cabeca, os bracos, os antebracos, as maos, as coxas, as canelas e os pes; o tronco de cima e o comeco
    // dela (os 4 primeiros: peito alto, cabeca e os bracos).)
    std::vector<std::uint32_t> assistLinks;
    // Os eixos da acao que sao das pernas (coxa, canela, pe e dedos): a corrida de lado mais livre.
    std::vector<bool> legAxes;
    std::vector<bool> armAxes;
    // (08/11) Os eixos do pescoco e da cabeca (headClipReach).
    std::vector<bool> headAxes;
    // (17/11, settings.clipVelocityRotation) As juntas de mais de um eixo na acao: os indices do twist, do swing1 e do swing2 (npos fora
    // da acao) e se sao do braco ou da cabeca.
    struct ActionJoint {
        std::array<std::size_t, 3> index {};
        bool arm = false, head = false;
    };
    std::vector<ActionJoint> actionJoints;
    // (17/11, settings.clipVelocityRotation - ver o .hpp) A velocidade-alvo das juntas de varios eixos do braco (arms) e da cabeca (heads)
    // entre os alvos de agora e os da proxima decisao, como o backend a quer; armScale: o calmo da largada nos bracos.
    void jointVelocities(const std::vector<float>& now, const std::vector<float>& next, float dt, bool arms, bool heads, float armScale,
        std::vector<float>& velocity, bool all = false) const {
        constexpr std::size_t None = std::numeric_limits<std::size_t>::max();
        for (const ActionJoint& j : actionJoints) {
            if (!all && !((arms && j.arm) || (heads && j.head))) continue;
            Vec3 a0, a1;
            bool wrap = false, valid = true;
            for (int a = 0; a < 3; ++a) {
                const std::size_t k = j.index[static_cast<std::size_t>(a)];
                if (k == None) continue;
                if (k >= now.size() || k >= next.size() || k >= velocity.size()) {
                    valid = false;
                    break;
                }
                axisRef(a0, a) = now[k];
                axisRef(a1, a) = next[k];
                // (A volta do ciclo: o salto do fim ao comeco do clipe fica sem velocidade, como no calculo por eixo.)
                if (std::abs(next[k] - now[k]) >= 1.0f) wrap = true;
            }
            if (!valid || wrap) continue;
            const Vec3 omega = rotationLog((rotationExp(a0).conjugate() * rotationExp(a1)).normalized()) * (1.0f / dt);
            const float scale = j.arm ? armScale : 1.0f;
            for (int a = 0; a < 3; ++a)
                if (j.index[static_cast<std::size_t>(a)] != None)
                    velocity[j.index[static_cast<std::size_t>(a)]] = scale * axisValue(omega, a);
        }
    }
    // O meio e a metade da faixa de cada eixo da acao (a naturalidade da recuperacao).
    std::vector<float> actionMid, actionHalf;
    static constexpr float RecoveryRoughScale = 0.10f;
    std::vector<float> assistGain, assistDamping, assistCap;
    std::size_t upperHandCount = 0;
    // O pe de equilibrio: as pernas e a sola para o IK.
    AthleteLegs3D legs;
    AthleteFootGeometry3D footGeometry;
    // A flexao na queda: o peito alto, as maos e as canelas (a origem delas e o joelho).
    std::uint32_t upperChestLink = 0;
    std::uint32_t handLinks[2] { 0, 0 };
    std::uint32_t shinLinks[2] { 0, 0 };
    // As partes que podem tocar o chao sem contar como caido: as pernas e os bracos (nao o tronco, o pescoco e a cabeca).
    std::vector<bool> allowedContact;
    // O apoio de protecao: as maos, os bracos e as pernas (os joelhos), sem os pes.
    std::vector<bool> supportContact;
    std::uint32_t workers = 1;
    std::shared_ptr<TaskScheduler> scheduler;
    std::unique_ptr<PhysicsEngine3D> engine;
    MaterialLibrary materials;

    struct World {
        std::unique_ptr<PhysicsScene3D> scene;
        // (17/11, setGrassGroundFraction) O chao do mundo e o material dele.
        PhysicsBodyHandle3D ground;
        bool grassGround = false;
        RagdollHandle3D handle;
        std::mt19937 rng;
        float time = 0.0f;
        TrackingClipFrame3D frame;
        std::vector<float> pose;
        std::vector<RagdollDriveTarget3D> drives;
        const RagdollState3D* body = nullptr;
        std::vector<float> observation;
        std::vector<float> finalObservation;
        std::vector<float> ampPair;
        // (07/11) O Motion Matching: o passado da raiz, a consulta de agora (todos os grupos, ponderada), os bancos que a escolha
        // permite e se ja vale (0,05 s de passado), e o pedido anterior (o giro da direcao e do rumo pedidos).
        MotionMatchingTrajectoryHistory3D motionHistory { 1.0f };
        std::vector<float> motionQuery;
        std::uint64_t motionGroups = 0;
        bool motionValid = false;
        float motionPreviousMove = 0.0f, motionPreviousFace = 0.0f;
        bool motionPreviousValid = false;
        std::vector<std::vector<float>> motionRaw;
        // (O cinematico: o clipe tocando - grupo, fonte, instante -, a raiz no mundo, os quadros escolhidos ha pouco (nao
        // voltam por 0,3 s), a pose do clipe sem a inercia (a consulta le dela, como o PoseSearch le a do continuar) e a
        // inercia de cada link (a diferenca para a pose de antes do salto, no referencial da raiz, sumindo em 0,3 s).)
        bool mmPlaying = false;
        std::size_t mmGroup = 0, mmSource = 0;
        float mmTime = 0.0f, mmInertiaElapsed = 1.0e9f;
        Vec3 mmRootPosition;
        float mmRootYaw = 0.0f;
        std::vector<std::pair<std::size_t, float>> mmRecent;
        std::vector<PhysicsBodyState3D> mmPose;
        // (A inercia nas coordenadas das juntas e na pelve - misturar duas poses dentro dos limites fica dentro deles;
        // 07/11, ele: "o joelho indo alem do limite da junta" com a inercia por link no mundo.)
        std::vector<Vec3> mmOffsetCoordinates;
        Vec3 mmOffsetRootPosition;
        Quaternion mmOffsetRootRotation;
        std::size_t mmJumps = 0;
        // (07/11, ele: "as transicoes estao horriveis") A sequencia: a diferenca na troca com a velocidade dela (juntas, pelve
        // no mundo, giro da pelve como vetor de rotacao), sumindo numa mola criticamente amortecida - sem salto de
        // velocidade; o ultimo quadro mostrado (a velocidade do de agora); o giro do tronco de cima para a vista e o peso
        // dele; a correcao do rumo do deslocamento (o setor contra o pedido, devagar).
        std::vector<Vec3> seqOffsetCoordinateVelocity;
        Vec3 seqOffsetRootVelocity, seqOffsetSpin, seqOffsetSpinVelocity;
        float seqWarpWeight = 0.0f, seqWarpAngle = 0.0f, seqTravelFix = 0.0f, seqTurnRate = 0.0f;
        // (07/11, ele: "nas curvas ele nao esta fazendo as inclinacoes... so faz viradas bruscas"; "o giro parado nao usa as
        // movimentacoes dos pes... ha para cada tipo de grau"; "falta a visao cinematica para os exercicios de levantar") A
        // escala do giro da virada (o angulo pedido sobre o do clipe), a inclinacao para dentro da curva (rad), o giro e a
        // velocidade do corpo suavizados; o clipe de levantar do comeco deitado.
        float seqTurnScale = 1.0f, seqLean = 0.0f, seqYawRate = 0.0f, seqSpeed = 0.0f, seqLift = 0.0f;
        std::size_t lyingClip = std::numeric_limits<std::size_t>::max();
        // (A capsula, como no Game Animation Sample: em movimento quem anda e ela, pelo modelo de movimento a partir do pedido,
        // e a pose vai sobre ela; a consulta le a trajetoria dela.)
        // (A sequencia: o tipo do clipe tocando - 0 parado, 1 partida, 2 ciclo, 3 virada, 4 parada, 5 curva, 6 giro -, ate
        // quando a partida/virada/parada manda, e o rumo pedido do corpo da decisao anterior - a virada da troca.)
        int seqKind = -1, seqSector = -1;
        float seqUntil = -1.0f, seqTurnFrom = -1.0f;
        // (A estrela: o centro, o rumo do comeco - as pontas de 45 em 45 graus a partir dele -, o ponto de agora, se para nele,
        // se ele e o centro, a ponta de agora e ate quando fica parado.)
        bool starValid = false, starStop = false, starAtCenter = true;
        Vec3 starCenter, starTarget;
        float starHeading = 0.0f, starHoldUntil = -1.0f;
        int starTip = -1;
        bool mmCapsule = false;
        Vec3 mmCapsuleVelocity;
        float mmCapsuleYawRate = 0.0f;
        std::uint32_t startKind = 0;
        std::uint32_t startSide = 0;
        float startHead = 0.0f;
        float stoodAt = -1.0f;
        double rewardSum = 0.0;
        std::uint32_t decisions = 0;
        // O empurrao (o de pe do jogo de pes ou o forte do evitar): a forca, os passos de fisica que faltam, o proximo e
        // quando o ultimo acaba (s).
        Vec3 pushForce;
        int pushTicks = 0;
        // (17/11, settings.projectileRate) O projetil de agora: a forca, os ticks que faltam e o elo.
        Vec3 projectileForce;
        int projectileTicks = 0;
        std::uint32_t projectileLink = 0;
        // (08/11, o fogo versatil) Onde o empurrao de agora entra: os elos e a parte da forca de cada um (o de sempre: pelve 20%,
        // peito 32%, peito alto 48%).
        std::array<std::uint32_t, 3> pushLinks { 0u, 2u, 3u };
        std::array<float, 3> pushShares { 0.20f, 0.32f, 0.48f };
        float nextPushAt = -1.0f;
        float lastPushEnd = -1.0e9f;
        // O comando (o levantar em movimento): o alvo no chao e a velocidade pedida (0: ficar onde esta).
        Vec3 target;
        float commandSpeed = 0.0f;
        float startCommandSpeed = 0.0f;
        // (07/11) O sorteio da faixa da velocidade deste episodio (ver speedRangeLow): o pedido fixo vezes ele.
        float speedScale = 1.0f;
        float reachedAt = -1.0f;
        double towardSum = 0.0;
        std::uint32_t towardSamples = 0;
        // (04/11 07h45) A velocidade que a referencia pede no rumo, nos mesmos instantes (a arrancada, as frenagens e as viradas).
        double wantedSum = 0.0;
        // (04/11) As partes da nota do rastreio no episodio: a semelhanca com o clipe e a estabilidade (somas e quantas).
        double imitationSum = 0.0;
        // (05/11 23h) Os apoios na corrida para a frente e nas diagonais da frente, e quantos com o pe apoiado mais de
        // CrossedInward para dentro que o mesmo pe do clipe (o pe cruzado; ver footCross do episodio).
        std::uint32_t crossStance = 0, crossBad = 0;
        std::uint32_t trackSamples = 0;
        // (05/11: a estabilidade a parte - no levantar pelo clipe so depois dele, no idle; deitado e subindo nao ha base.)
        std::uint32_t stabilitySamples = 0;
        // (05/11 04h55) A folga entre as pernas desta decisao (m; < 0 encostando) - legGap.
        float legGap = 1.0f;
        // O evitar a queda: quando vem o empurrao forte, quando ele acaba e quando se recompos.
        float avoidAt = -1.0f;
        float avoidEnd = -1.0f;
        float recomposedAt = -1.0f;
        // O ponto do idle: onde o corpo estava quando o idle passou a ser seguido (o parado nao sai do lugar).
        Vec3 idleAnchor {};
        // (15/11, settings.idleClipAnchor) Na troca do levantar de pe para o parado, onde a pelve do clipe do levantar estava (o clipe
        // preso no mundo): o parado nasce ali, sem o clipe pular; e quando (s; < 0: nenhuma).
        Vec3 handoffAnchor {};
        // (15/11, settings.idleHoldWeight) O ponto do parado para a nota do lugar e se ja foi posto.
        Vec3 holdAnchor {};
        bool holdAnchored = false;
        // (15/11 15h20, ele: "ele ainda se move um pouco apos levantar... ao inves de ficar parado onde levantou") Na troca do levantar
        // de pe para o parado o lugar e posto na hora - antes so com o corpo abaixo de 0,1 m/s, e a caminhada logo depois da troca saia
        // de graca.
        bool holdAtHandoff = false;
        // (15/11) A ancora do fantasma desenhado (referenceLines; a mesma do referencePose do laboratorio).
        Vec3 viewAnchor {}, viewClipPelvis {};
        bool viewAnchored = false;
        float viewClipAt = -1.0f, viewTime = 0.0f;
        float handoffAnchorAt = -1.0f;
        bool idleAnchored = false;
        // A queda programada: a dela e o comeco ja caindo que ela assume.
        std::unique_ptr<ProtectiveFall3D> fall;
        bool fallPending = false;
        bool fallActive = false;
        bool fallFromStart = false;
        // O levantar guiado: seguindo um clipe de levantar (sem ciclo e acelerado: o tempo dele e trackStart + (time -
        // trackFrom) * trackScale) e, de pe no fim dele, o parado em alerta.
        bool getUpGuided = false;
        bool idleAfterGetUp = false;
        // (11/11 14h, settings.clipGetUpFraction) O levantar pelo clipe (TrackingClipGetUp3D): o exercicio (0 de costas, 1 de
        // brucos, 2 correndo de costas, 3 correndo de brucos), o clipe e o instante do comeco, o clipe ancorado no mundo (o ponto
        // do chao e o rumo), o tempo longe dele, a troca (o instante; < 0 antes dela), a velocidade do centro de massa na troca, o
        // voo de pe (os dois pes no ar com a cabeca acima de 1,2 m, com a recuperacao) e a nota do passo.
        bool clipGetUp = false;
        std::uint32_t clipExercise = 0;
        // (14h30) O exercicio do clipe adotado pela pose (o rumo e a velocidade da corrida); o do comeco ate a pose mandar.
        std::uint32_t clipPoseExercise = 0;
        std::size_t clipReference = 0;
        Vec3 clipGround {};
        float clipStart = 0.0f, clipYaw = 0.0f, clipFarSeconds = 0.0f, clipFacingSeconds = 0.0f, clipHandedAt = -1.0f, clipHandSpeed = 0.0f,
              clipFlightSeconds = 0.0f;
        // (20h20, o rolamento livre - TrackingClipGetUp3D::FreeRollProne): o relogio do clipe e clipStart + time - clipHeld (o tempo que
        // ele esperou ou adiantou); clipFree enquanto o corpo rola pelo caminho dele, ate o clipe estar de brucos em clipFreeUntil.
        float clipHeld = 0.0f, clipFreeSeconds = 0.0f, clipFreeUntil = -1.0f;
        bool clipFree = false;
        // (12/11 00h35, a entrega no ponto do corte - setClipGetUpDeliver): quando foi entregue (s; -1 sem), com o clipe seguindo depois.
        float clipDeliveredAt = -1.0f;
        // (12/11 03h20, a agilidade) O ritmo do relogio do clipe (1 o tempo; suavizado), o premio da decisao e, na entrega, os segundos
        // do clipe percorridos desde o comeco e o ponto dela (o tempo para levantar = ponto x tempo real / percorridos; -1 sem entrega).
        float clipRate = 1.0f, clipAgilityReward = 0.0f, getUpCovered = -1.0f, getUpTarget = -1.0f;
        // (12/11 04h40, a fase de fogo do levantar - TrackingGetUpAmpEpisode3D::fireKind) O cenario, o momento, o rumo, a fracao e o
        // tamanho; a forca continua (mundo, N) e a escala dela agora (a rampa); a hora da pancada e se ja veio; quando foi ao chao depois
        // dela; e o firme do levantar adaptativo (o tempo firme seguido, de quando, e quando levantou).
        int fireKind = 0, fireTiming = 0, fireDirection = 0;
        float fireFraction = 0.0f, fireSize = 0.0f, fireHoldScale = 0.0f, fireShoveAt = -1.0f, fireDownAt = -1.0f;
        float fireReleaseFrom = -1.0f;
        float riseFirmSeconds = 0.0f, riseFirmFrom = -1.0f, upAt = -1.0f, fireShoveSize = 0.0f, fireShoveHead = 0.0f, fireAbovePitch = 0.0f;
        float handoffCalmSeconds = 0.0f;
        // (12/11 08h20, settings.adaptiveRetry) A tentativa de agora: desde quando, quantas e ha quanto tempo ele esta para tras do clipe.
        float attemptFrom = 0.0f, retryBehindSeconds = 0.0f, attemptPhase = 0.0f, attemptPeakHead = 0.0f;
        // (12/11 12h30, fase 2) As pancadas que faltam, o rumo da pancada de agora, a parede pendente (rumo, distancia; < 0 sem), a proxima
        // troca do vento e os bits das adversidades do episodio.
        int fireShovesLeft = 0, fireShoveDirection = 0, fireExtras = 0;
        bool fireAfterStand = false, fireStandHandled = false, fireSlip = false;
        // (12h50) O agarrao: o elo (a mao), a forca (N) e de quando ate quando.
        int fireGrabLink = -1;
        Vec3 fireGrabForce {};
        float fireGrabFrom = -1.0f, fireGrabUntil = -1.0f;
        float fireWallAngle = 0.0f, fireWallDistance = -1.0f, fireNextTurnAt = -1.0f;
        int attempts = 1;
        // (12/11 05h30, o desenho no laboratorio) A pancada que veio: a forca (N, mundo), o elo e quanto durou.
        Vec3 fireShoveForce {};
        int fireShoveLink = -1;
        float fireShoveSeconds = 0.0f;
        // (06h20) A pancada do meio e a do quase fim pela subida: a altura da cabeca que a dispara (m; < 0: pelo tempo, a do comeco).
        float fireShoveHeight = -1.0f;
        // (06h40, settings.fireObservation) A aceleracao do centro de massa suavizada (m/s^2) e a velocidade dele na decisao anterior.
        Vec3 comAcceleration {}, comVelocityBefore {};
        bool comVelocityKnown = false;
        Vec3 fireHold {};
        bool fireShoveDone = false;
        // (12/11 16h30, o fogo da locomocao - settings.locoFireShare) Se o episodio tem fogo, o nivel L, a proxima adversidade (s; < 0
        // nenhuma), o fim do continuo de agora (s), a perturbacao p (0-1, ver locoFireRelief), quantas adversidades vieram, o centro e a
        // meia largura da grama molhada (m), a prova (o tipo do mundo, -1 fora dela) e o tempo seguindo a tarefa sob fogo (amostras).
        bool locoFire = false;
        float locoFireLevel = 0.0f, locoFireNextAt = -1.0f, locoFireHoldUntil = -1.0f, firePerturb = 0.0f;
        int locoFireEvents = 0, locoFireExamKind = -1;
        Vec3 fireSlipCenter {};
        float fireSlipHalf = 0.0f;
        int locoKeep = 0, locoKeepSamples = 0;
        // (12/11 17h, settings.crookedStartShare) O parado que nasceu torto: quando reajustou (s; -1 ainda nao) e ha quanto tempo esta perto
        // da pose e quieto (s).
        bool crooked = false;
        float crookedFixedAt = -1.0f, crookedFixSeconds = 0.0f;
        // (12/11 19h10, settings.getUpGoShare) O levantar e ir: o ponto, a distancia dele no comeco e quando chegou (s; -1 nao).
        bool getUpGo = false;
        Vec3 goTarget;
        float goDistance = 0.0f, goArrivedAt = -1.0f;
        TrackingClipGetUp3D::Terms clipTerms {};
        // No levantar em movimento guiado, a arrancada agachada no lugar do fim do clipe de levantar.
        bool sprintGuided = false;
        float trackFrom = 0.0f;
        float trackScale = 1.0f;
        // Caido agora, e se ja caiu depois de ter ficado de pe.
        bool down = false;
        bool fellAfterStanding = false;
        // O rumo da arrancada agachada (rad).
        float sprintYaw = 0.0f;
        // A locomocao (os rumos e o giro): o rumo pedido (0 a 7), o do deslocamento e o do corpo (rad), quando o pedido
        // para (s; < 0: nao para), no giro a velocidade (rad/s, com o sinal), quanto falta do giro (rad) e o fim da pausa
        // (s; < 0: girando), e o erro do rumo do corpo de pe (a soma, rad).
        bool locomotion = false;
        // O rastreio: a ultima nota e os erros (TrackingFeedbackSize), o alvo do clipe da decisao
        // (o teste de mergulho), a media movel da nota (o fim por desvio) e a velocidade do centro de massa suavizada
        // na passada.
        std::array<float, TrackingGetUpAmpTraining3D::TrackingFeedbackSize> locomotionFeedback {};
        std::vector<float> clipAction;
        // (17/11, settings.imitationVelocityTargets) Os alvos do clipe de agora (o comeco da decisao) e a velocidade-alvo dele por eixo.
        std::vector<float> clipActionNow, clipActionVelocity;
        TrackingClipFrame3D imitationNowFrame;
        float trackingAverage = 1.0f;
        Vec3 smoothVelocity;
        // (18h45) A velocidade do rumo da nota: suavizada em courseSmoothing s - um passo inteiro.
        Vec3 courseVelocity;
        // (20h05) Ha quanto tempo o deslocamento esta fora do rumo pedido alem do courseAngleLimit (s).
        float straySeconds = 0.0f;
        std::vector<float> jointErrors;
        std::vector<float> clipVelocity;
        // De pe para o passo: a velocidade pedida em rampa desde este instante (s; < 0: sem rampa, o RSI).
        float rampFrom = -1.0f;
        // O erro do rumo da pelve contra a do clipe na ultima nota (rad) e ha quanto tempo ele passa do teto (s).
        float headingError = 0.0f;
        float turnedSeconds = 0.0f;
        // (17/11, settings.headingBiasWeight) A media movel (~0,6 s) do erro do rumo da pelve nas retas com pedido (rad).
        float headingBias = 0.0f;
        // (17/11, settings.lateralBiasWeight) As medias moveis (~0,6 s) do lado da cabeca, do peito alto e do meio dos pes contra o clipe (m).
        Vec3 lateralBias;
        // (13/11 v1, settings.turnedGrace) O rumo da pelve da referencia na decisao anterior e ate quando ela girava rapido.
        float lastReferenceHeading = 0.0f, referenceTurningUntil = -1.0f;
        // (14/11 v1, settings.getUpImitationMask) O clipe e o instante do comeco do levantar (as tentativas novas trocam a referencia).
        std::size_t imitationReference = std::numeric_limits<std::size_t>::max();
        float imitationStart = 0.0f, imitationLowSeconds = 0.0f;
        // (15/11, settings.getUpImitationAnchor) O lugar do clipe no ritmo preso no mundo (o rumo e o chao) e se ja foi preso.
        float imitationYaw = 0.0f;
        Vec3 imitationGround {};
        bool imitationAnchored = false;
        bool hasReferenceHeading = false;
        float stableSeconds = 0.0f;
        // O parado com a pose perdida (os pes fora do idle ou a pelve afundada) ha tanto tempo (s).
        float idleLostSeconds = 0.0f;
        // (09/11 19h50, settings.poseBoost) Ha quanto tempo o parado esta dobrado (o peito alto alem de IdleBentLimit do idle; s).
        float bentSeconds = 0.0f;
        // (04/11 09h00) A entrega ao idle depois de levantar: quando foi e os alvos da recuperacao naquela hora - o idle entra
        // a partir deles em IdleBlendSeconds; e os alvos da ultima decisao da recuperacao.
        float idleBlendFrom = -1.0e9f;
        // (04/11 11h25) A arrancada de pe: ate quando (s; < 0 sem arrancada - o comeco no clipe) e a naturalidade da recuperacao
        // nesta decisao (a aspereza dos alvos e as juntas no limite).
        float launchEnd = -1.0f;
        // (04/11 12h35) Com o pedido fixo: a duracao da rampa da largada (s) e, depois da arrancada da corrida, de quando e de
        // que velocidade o pedido sobe ate o fixo (s, m/s; accelFrom < 0: sem).
        float rampSeconds = 0.8f;
        float accelFrom = -1.0f, accelSpeed = 0.0f;
        // (06/11) A largada progressiva da corrida: a aceleracao do pedido depois do impulso (0: StartAcceleration) e a subida
        // da cadencia do ciclo (de quando, de que escala a que escala, no clipe easeClip; easeLast, a escala posta - outra
        // mudanca dela, de uma manobra, cancela a subida).
        float accelRate = 0.0f, easeFrom = -1.0f, easeScale = 1.0f, easeTarget = 1.0f, easeLast = 1.0f;
        std::size_t easeClip = 0;
        float recoveryRough = 0.0f, recoveryLimits = 0.0f;
        // A linha da altura da pelve contra a do clipe (a media movel; o pulo e o que sobe acima dela).
        float riseMean = 0.0f;
        bool riseKnown = false;
        // (04/11 16h30, a firmeza) A margem do ultimo apoio (no voo vale ela), o tropeco (1 no instante, esvai) e as decisoes
        // de cada pe no ar; a soma do quadrado da falta de estabilidade (a do episodio na v13).
        float lastMargin = 0.0f, stumble = 0.0f;
        // (21h35) Desde quando esta de pe com a recuperacao esperando o corpo acalmar (-1: nao esta).
        float calmFrom = -1.0f;
        std::array<int, 2> firmAir {};
        double stabilityDeficitSum = 0.0;
        std::vector<float> idleBlendPose, lastRecoveryAction;
        // (04/11 15h, os passos) O gerador de passos do idle (ligado so no idle da locomocao), o equilibrio medido no comeco da
        // decisao, o quadro do idle e a pose da referencia com as pernas do passo (a da nota), as coordenadas do clipe antes do
        // IK; o equilibrio parado do episodio, os empurroes, o pior ponto de captura fora da base e quando quietou de novo.
        TrackingBalanceSteps3D steps;
        bool stepsOn = false;
        TrackingBalance3D balance;
        TrackingClipFrame3D stepFrame;
        RagdollAnimationPose3D stepPose;
        bool stepPoseValid = false;
        float stepPoseWeight = 0.0f;
        std::vector<Vec3> clipCoordinates;
        bool balanceDrill = false;
        std::uint32_t pushes = 0, recoveryStepCount = 0, settleStepCount = 0;
        float worstCaptureOut = 0.0f;
        float quietAgainAt = -1.0f;
        // A chegada do levantar: a qualidade dela na entrega (< 0 sem entrega) e o premio a somar nesta decisao.
        float arrivalQuality = -1.0f;
        float arrivalReward = 0.0f;
        // A ameaca do ponto de captura no comeco da decisao (o alcance sob ameaca do idle).
        float threat = 0.0f;
        bool threatKnown = false;
        bool locomotionControl = false;
        std::uint32_t heading = 0;
        float moveYaw = 0.0f;
        float faceYaw = 0.0f;
        float turnRate = 0.0f;
        float turnLeft = 0.0f;
        float turnPauseUntil = -1.0f;
        double faceErrorSum = 0.0;
        std::uint32_t faceErrorSamples = 0;
        // A rl_final: a faixa do passo de agora e a do comeco, e a proxima troca do pedido (s; < 0: sem troca).
        std::uint32_t band = 1;
        std::uint32_t startBand = 1;
        // O comeco num instante de uma queda com protecao (a protecao vale, como no evitar a queda).
        bool protectiveStart = false;
        // A arrancada (idle_to_sprint) antes do ciclo da corrida ou do sprint saindo de pe: o ciclo que vem depois (max:
        // nenhum).
        std::size_t pendingClip = std::numeric_limits<std::size_t>::max();
        // O ultimo comeco de pe no parado em alerta (o clipe, o instante e o rumo; max: outro).
        std::size_t lastClip = std::numeric_limits<std::size_t>::max();
        float lastClipTime = 0.0f;
        float lastClipYaw = 0.0f;
        // O alvo misturado com o do clipe (clipBlend).
        std::vector<float> blendAction;
        // (06/11, freeVelocityFeedforward) O alvo da cabeca da locomocao na decisao anterior (vazio: sem).
        std::vector<float> freeTarget;
        std::vector<float> armClipNow, armClipNext;
        // (17/11, settings.clipVelocityRotation) Os alvos misturados do calmo da largada, para a velocidade por junta.
        std::vector<float> rotationNow, rotationNext;
        RagdollAnimationPose3D balancePose;
        // O rolar pelo clipe: o clipe (max: sem), o tempo dele e quando comecou; e quando pode comecar outro.
        std::size_t rollClip = std::numeric_limits<std::size_t>::max();
        float rollStart = 0.0f;
        float rollFrom = 0.0f;
        float rollAgain = 0.0f;
        // A imitacao: o clipe do passo pedido (max: sem), o tempo dele no comeco do episodio e o rumo.
        std::size_t trackClip = std::numeric_limits<std::size_t>::max();
        // O giro da coluna no clipe do rumo (as diagonais: a pelve no rumo, o peito na vista) e o clipe a que ele vale.
        float trackTwist = 0.0f;
        // A mao da forca auxiliar na ultima decisao (as medias dos ticks): a sustentacao (N, + para cima), o empurrao no
        // plano (N) e o torque na pelve (N.m), ja escalados pela forca.
        float handUp = 0.0f, handSide = 0.0f, handTurn = 0.0f;
        // A faixa da corrida neste episodio de rumo (a forca auxiliar dela).
        bool runBand = false;
        // A manobra (0 nenhuma; os codigos do evaluationManeuver) e o estado dela: a proxima troca (s); a transicao do quadro
        // - o clipe de antes (inicio, escala, rumo, giro da cintura) e de quando, quanto dura -; a velocidade de referencia,
        // que freia e reacelera ate ManeuverAcceleration; nas curvas e na troca livre o giro do rumo (rad/s), quanto falta
        // (rad), o lado e a reta depois (s); na troca livre, o rumo novo.
        int maneuver = 0;
        // (13/11, settings.fallToCommand) Caiu num exercicio de locomocao e virou comando: a manobra de antes (os registros do episodio).
        bool fallConverted = false;
        int originalManeuver = 0;
        std::uint32_t originalKind = 0;
        float convertedAt = -1.0f;
        float nextManeuverAt = -1.0f;
        std::size_t blendClip = std::numeric_limits<std::size_t>::max();
        float blendStart = 0.0f, blendScale = 1.0f, blendYaw = 0.0f, blendTwist = 0.0f, blendFrom = -1.0f, blendSeconds = 0.0f;
        // (15/11, settings.reviewFixes) A inclinacao do tronco e das pernas da mistura, de quando ela comecou.
        float blendPitch = 0.0f, blendLegPitch = 0.0f;
        bool refActive = false;
        Vec3 refVelocity;
        float arcRate = 0.0f, arcLeft = 0.0f, straightUntil = -1.0f, viewTarget = 0.0f;
        // A janela de improviso da troca (v6): ate quando a semelhanca e afrouxada (s; < 0 nenhuma).
        float improviseUntil = -1.0f;
        // (04/11 23h40) A aceleracao da velocidade pedida nas manobras (m/s^2, no plano; media de AccelLeanSmoothing).
        Vec3 refAccel;
        // (23h55) A meia-volta da troca livre: o lado (-1: nenhuma), quando comecou, a escala do giro (o angulo pedido sobre o
        // do clipe) e a vista do comeco.
        int runTurn = -1;
        float runTurnFrom = 0.0f, runTurnScale = 1.0f, runTurnStartYaw = 0.0f;
        // (09/11 04h40) A escala do tempo desta meia-volta (settings.runTurnTimeScale nas saidas de costas e de lado; 1 de frente).
        float runTurnTime = 1.0f;
        // (05/11 21h) A troca correndo pela freada (runTurnBrake): o giro esperando a freada (rad), quando ele entra e quando a
        // corrida volta depois da meia-volta (s; < 0 nenhum).
        float brakeTurn = 0.0f, brakeTurnAt = -1.0f, runAgainAt = -1.0f;
        // (05/11 21h40, runTurnAlign) O giro esperando ja foi alinhado ao pe (entra na hora marcada).
        bool turnAligned = false;
        // (05/11 21h10) A ancora da estabilidade medida (idleReanchor): recomeca depois de cada empurrao.
        Vec3 measureAnchor;
        bool measureAnchored = false;
        int arcSign = 1;
        // (04/11 12h25, ele: "as marcacoes nas curvas seria melhor se os pontos fossem fixos no mundo - da a sensacao que estao
        // soltos, e somem antes da hora") A curva sorteada inteira no comeco do episodio: os arcos (rad, a velocidade
        // angular com o sinal) e as retas depois de cada um (s); e o percurso no mundo, um ponto a cada 1 m desde o comeco, e
        // o quanto do percurso a referencia ja andou (m).
        struct CourseArc {
            float radians = 0.0f, rate = 0.0f, straightAfter = 0.0f;
        };
        std::vector<CourseArc> coursePlan;
        std::size_t courseNext = 0;
        std::vector<Vec3> course;
        float courseWalked = 0.0f;
        // (04/11 14h) O ponto do percurso mais perto do corpo (o indice, so avanca), a distancia do corpo ao percurso (m) e
        // por quanto tempo ele esta fora (s).
        std::size_t courseAt = 0;
        float courseError = 0.0f, offCourseSeconds = 0.0f;
        // (08/11) Por quanto tempo ele anda abaixo do piso de velocidade (slowLimit; s).
        float slowSeconds = 0.0f;
        // (14h40) O progresso continuo do corpo ao longo do percurso (em pontos: -1 o comeco, k o ponto k).
        float courseProgress = -1.0f;
        // O comeco do percurso (a pelve e o rumo do deslocamento no comeco do episodio).
        Vec3 courseOrigin;
        float courseHeading = 0.0f;
        // (08/11, ele: "6 exercicios novos, da estrela de oito pontas... garanta bem a boa estruturacao das fases de
        // aprendizado") A estrela (manobras 8 peito fixo, 9 livre, 10 girando): o centro e o rumo da ponta 0, o raio (m), de onde
        // a perna sai e o ponto dela (-1 o centro), o seguinte (sorteado antes: a freada sabe o angulo da troca), o angulo da
        // proxima troca (rad), se ja freou, a troca de rumo esperando o clipe deixar, ate quando a saida do percurso nao conta
        // (s; a passagem pela ponta), o giro da vista na perna (rad/s), as pernas feitas e a fase.
        struct Star {
            Vec3 center, from;
            float base = 0.0f, radius = 6.0f, nextTurn = 0.0f, graceUntil = -1.0f, spin = 0.0f, arrive = 0.0f;
            int at = -1, goal = -1, next = -1, legs = 0, phase = 1, pendingHeading = -1;
            bool braked = false;
            // (12h05) A saida devagar no rumo novo: ate quando (s; < 0 nenhuma).
            float fullAt = -1.0f;
            // (12/11 17h20, settings.starCurveShare) A flecha da perna de agora e da seguinte (sobre a corda, com o sinal: + a esquerda; 0
            // reta), as curvas feitas e a distancia ao caminho somada (settings.starPathWeight).
            float bend = 0.0f, nextBend = 0.0f;
            int curves = 0, pathSamples = 0;
            double pathErrorSum = 0.0;
        } star;
        // (08/11, a fase de fogo) Se este episodio do treino tem empurroes (pushEpisodeShare).
        bool fireEpisode = true;
        // (08/11 15h20) A parada: quando o pedido some no fim do reto (s; < 0 nenhuma), se esta parando (o pedido zerado na
        // observacao; o comando tambem), a fase (0 nada, 1 freando, 2 parado) e o ponto de parada de referencia (a freada perfeita
        // integrada desde o comeco dela), o erro dele no fim (m; < 0 sem parada).
        float finalStopAt = -1.0f;
        bool stopping = false;
        int stopPhase = 0;
        Vec3 stopRef;
        float stopError = -1.0f;
        // (16h20) A parada pelo clipe: a posicao em stopClipInfo (-1 sem), quando entrou, o instante de entrada no clipe e a escala
        // da velocidade (a de agora sobre a de entrada do clipe).
        int stopClip = -1;
        float stopFrom = 0.0f, stopEntry = 0.0f, stopScale = 1.0f;
        // (10/11 04h) A arrancada da Unreal: a posicao em startInfo (-1 sem) e o rumo, a cintura e o clipe da cintura do ciclo que
        // vem depois (a arrancada toca no rumo do proprio deslocamento, sem a cintura das diagonais - o clipe ja olha para a vista).
        int unrealStart = -1;
        // (10/11 07h05, settings.launchStepReward) Os pes na largada: o tempo no ar de cada um e se estava no chao.
        float launchFootAir[2] { 0.0f, 0.0f };
        // (10/11 07h10, settings.launchClipShare) Os alvos do clipe na largada (agora e daqui a uma decisao).
        std::vector<float> launchClipNow, launchClipNext;
        // (15/11, settings.launchImitationReward) A ultima nota da semelhanca com o clipe da largada (-1: fora da largada).
        float launchImitation = -1.0f;
        // (15/11, settings.handBehindWeight) Ha quanto tempo cada mao esta atras das costas (s).
        float handBehindSeconds[2] { 0.0f, 0.0f };
        bool launchFootDown[2] { true, true };
        // (10/11 08h20, settings.launchInPlacePenalty) Onde cada pe saiu do chao na largada.
        Vec3 launchFootLift[2] {};
        float startCycleYaw = 0.0f, startCycleTwist = 0.0f;
        std::size_t startCycleTwistClip = std::numeric_limits<std::size_t>::max();
        // (09/11 03h50) Quando a parada comecou (s; settings.stopWaitBody).
        float stopBeganAt = 0.0f;
        // (09/11 10h05) Quando o idle chegou de uma parada (a final ou a do comando; s) - o assentar do idlePoseGate.
        float idleSince = -10.0f;
        // (10/11 01h50: o rumo do peito quando o parado livre comeca fora da locomocao - o alvo do rosto dele.)
        bool idleFaceSet = false;
        // (18h25) Na freada da referencia: a velocidade e a cadencia do clipe no comeco dela.
        float stopFromSpeed = 0.0f, stopFromScale = 1.0f;
        // (09/11 14h, settings.freeStop) A parada livre deste mundo (ate a proxima tecla do comando ou o fim do episodio) e a
        // velocidade do corpo na ordem (m/s; a regua do progresso).
        bool freeStop = false;
        float freeFromSpeed = 1.0f;
        // (09/11 15h, settings.freeTroca) A troca livre deste mundo: ate quando (s), o erro de velocidade no comeco (m/s; a regua do
        // progresso) e a vista para onde gira (rad; a meia-volta pela vista).
        bool freeTroca = false;
        float freeTrocaUntil = -1.0f, freeTrocaError = 1.0f, freeTrocaView = 0.0f;
        // (08/11 14h40) O comando (manobra 12): a tecla (0-7 em relacao a vista; -1 nenhuma), a fase, o estado (0 andando, 1
        // freando, 2 parado, 3 girando no lugar), os eventos do jogador, as paradas e as arrancadas feitas; o shift; a vista que o
        // mouse pede (rad), quando vem a proxima tecla e o proximo gesto do mouse, a panoramica (rad/s, ate quando) e o giro no
        // lugar (rad/s).
        struct Command {
            int key = -1, phase = 1, state = 0, events = 0, stops = 0, starts = 0;
            // (12/11 18h, settings.commandGetUp) As quedas, os levantares de volta, de quando e quanto tempo somado (s).
            int falls = 0, rises = 0, runRises = 0;
            float riseFrom = -1.0f, riseSum = 0.0f;
            bool run = false;
            float target = 0.0f, nextKeyAt = 0.0f, nextMouseAt = 0.0f, panRate = 0.0f, panUntil = -1.0f, turnRate = 0.0f;
        } command;
        TrackingClipFrame3D blendFrame;
        // O quadro do idle no rumo do corpo (a imitacao do parado).
        TrackingClipFrame3D idleFrame;
        // (10/11, settings.basePosture) O idle alerta no rumo da vista, a distancia dele na decisao anterior (-1: nenhuma) e os pes.
        TrackingClipFrame3D baseFrame;
        std::vector<RagdollDriveTarget3D> tonedDrives;
        float basePrevDistance = -1.0f;
        bool baseFootDown[2] { true, true };
        // O outro jeito aceito na semelhanca (a corrida de costas: o trote de costas, ou o contrario) e o quadro dele.
        std::size_t altClip = std::numeric_limits<std::size_t>::max();
        TrackingClipFrame3D altFrame;
        std::vector<float> altErrors;
        // A mao do corpo: o alvo e o giro do clipe de cada osso nesta decisao.
        std::vector<Quaternion> handTarget;
        std::vector<Vec3> handSpin;
        std::size_t twistClip = std::numeric_limits<std::size_t>::max();
        float trackStart = 0.0f;
        float trackYaw = 0.0f;
        // A troca de rede ao ficar de pe: ha quanto tempo ele esta de pe e firme (s) e quando trocou (s; < 0: nao).
        float firmSeconds = 0.0f;
        float handoffAt = -1.0f;
        // A parceira depois da troca (quanto dirigiu, s; < 0: sem) e se o tronco ja tocou o chao.
        float partnerDrove = -1.0f;
        bool partnerFell = false;
        bool touchedDown = false;
        // A protecao na queda: se esta protegendo-se agora e se ja se apoiou.
        bool protecting = false;
        bool supported = false;
        // A flexao na queda: ate quando vale depois do toque (s; < 0: sem toque ainda), a soma e as amostras.
        float flexUntil = -1.0f;
        double flexSum = 0.0;
        std::uint32_t flexSamples = 0;
        // O cenario sujo do episodio: a parede e o peso (desfeitos no comeco seguinte).
        std::vector<PhysicsBodyHandle3D> obstacles;
        bool obstructed = false;
        // Os corpos soltos do episodio (embaixo e em cima) e a rampa: a normal dela e a altura dela no meio (m).
        std::vector<RagdollHandle3D> extras;
        // As caixas do cenario, para as folhas (o corpo, e a caixa nele).
        struct SceneryBox {
            PhysicsBodyHandle3D body;
            Vec3 local;
            Quaternion orientation;
            Vec3 half;
            bool arena = false, slip = false;
        };
        std::vector<SceneryBox> boxes;
        bool sloped = false;
        Vec3 slopeNormal { 0.0f, 0.0f, 1.0f };
        float slopeHeight = 0.0f;
        // Quanto a queda sobe a mais por causa dos corpos embaixo (m).
        float underLift = 0.0f;
        // O ambiente garantido do episodio (so com evaluationScenery; -1 sem): 0 a parede, 1 o corredor, 2 os pilares,
        // 3 o peso, 4 o corpo em cima, 5 a rampa, 6 o acidentado, 7 os corpos embaixo.
        int forcedScenery = -1;
        int controlledPreset = -1;
        // O premio do passo: o tempo no ar de cada pe (s) e se ele estava no chao.
        float footAir[2] { 0.0f, 0.0f };
        bool footDown[2] { true, true };
        std::uint32_t index = 0;
        // A professora (setLocomotionTeacher): a sessao do episodio (nula: sem ela), se ela dirige o episodio todo, ate
        // quando ela retomou (s), a media do erro do peito (rad) e a inclinacao do tronco da ultima decisao (graus; < 0:
        // nenhuma); o rotulo da ultima decisao (fora do comeco: o recomeco zera o mundo) e as contas do episodio; a
        // gravidade da aluna para o torque dela em sombra; a soma da velocidade no chao (o desvio do rumo).
        std::unique_ptr<LocomotionPartner3D::Session> teacherSession;
        // O episodio ensinado (os rumos, o giro e o parado com a professora ou taughtHeadings): da cabeca da locomocao.
        bool taught = false;
        bool teacherEpisode = false;
        float teacherUntil = -1.0f;
        float chestError = 0.0f;
        float lastTilt = -1.0f;
        std::vector<RagdollDriveTarget3D> teacherDrives;
        std::vector<float> label;
        std::vector<float> labelRaw;
        bool labelValid = false;
        bool labelDrove = false;
        bool labelTakeover = false;
        std::uint32_t teacherDecisions = 0;
        std::uint32_t takeovers = 0;
        std::uint32_t labeled = 0;
        float teacherAssist = 0.0f;
        ArticulationGravityCompensator3D gravity;
        std::vector<ArticulationLinkPose3D> gravityPoses;
        Vec3 travelSum;
        // O musculo da aluna nesta decisao: a compensacao de gravidade (o atuador da locomocao, setStudentActuator).
        bool driveGravity = false;
        // As escalas dos drives da professora por eixo da acao (a rigidez, o amortecimento, o teto, a gravidade e o
        // |antecipacao|, N.m) somadas tick a tick, e quantos ticks (teacherDriveStats).
        std::vector<double> teacherStats;
        std::uint64_t teacherTicks = 0;
        // (Rascunhos do rotulo por junta.)
        std::vector<Vec3> linkError;
        std::vector<Vec3> linkTarget;
        std::vector<float> axisError;
        std::vector<float> axisTarget;
    };
    std::vector<World> worlds;
    // As quedas da locomocao (setFallStarts) e a rede da locomocao parceira (setLocomotionPartner).
    std::vector<std::vector<PhysicsBodyState3D>> fallStarts;
    std::shared_ptr<const LocomotionPartner3D> partner;
    std::shared_ptr<const LocomotionPartner3D> teacher;
    // A aluna-oraculo (o portao da exatidao, antes de treinar): 0 a rede, 1 o rotulo do tick 0 segurado, 2 a cada tick.
    int teacherOracle = 0;
    // O musculo da cabeca da locomocao (setStudentActuator; vazio: o do rastreador, 0,2/0,5 sem gravidade): as escalas por
    // eixo da acao e a compensacao de gravidade.
    std::vector<float> studentStiffness;
    std::vector<float> studentDamping;
    std::vector<float> studentMaximumTorque;
    bool studentGravity = false;
    void applyStudentActuator(World& w) const {
        w.driveGravity = false;
        if (studentStiffness.size() != w.drives.size() || !w.locomotionControl) return;
        for (std::size_t k = 0; k < w.drives.size(); ++k) {
            w.drives[k].stiffnessScale = studentStiffness[k];
            w.drives[k].dampingScale = studentDamping[k];
            w.drives[k].maximumTorqueScale = studentMaximumTorque[k];
        }
        w.driveGravity = studentGravity;
    }
    std::vector<TrackingGetUpAmpEpisode3D> episodes;
    std::mutex episodeMutex;

    // O rumo puro da pelve no plano (a frente dela).
    float pelvisHeading(const PhysicsBodyState3D& pelvis) const {
        const Vec3 f = (pelvis.orientation * profile->links[rootLink].modelOrientation.conjugate()).normalized()
                           .rotate({ 1.0f, 0.0f, 0.0f });
        return std::atan2(f.y, f.x);
    }
    // A forca auxiliar da locomocao (settings.locomotionAssist): so com a cabeca da locomocao seguindo um clipe de pe.
    // As constantes por kg (altura, velocidade) e em N.m (a pelve): a 1, a pelve afunda ~5 cm com as pernas soltas.
    static constexpr float AssistHeightGain = 200.0f, AssistHeightDamping = 28.0f, AssistVelocityGain = 4.0f,
                           AssistTurnGain = 1500.0f, AssistTurnDamping = 80.0f, AssistTurnDampingLow = 300.0f,
                           AssistMaximumForce = 2000.0f,
                           AssistMaximumTorque = 1500.0f;
    // Os alvos de motor do clipe seguido daqui a `ahead` s (out, na acao). Com o pe de equilibrio, o pe do clipe no ar
    // (mais de 2 cm acima do outro) pousa deslocado pelo ponto de captura: (v - pedida) / omega mais o centro de massa
    // do corpo contra o do clipe, no plano, ate 0,3 m.
    // (04/11 15h20, a sonda do pos-levantar: a entrega na primeira decisao de pe pegava o corpo ainda subindo a ~1,5 m/s - os
    // dois pes saiam do chao por ~0,35 s, a cabeca a 1,76 m - e ele aterrissava num pe so, desequilibrado.) Com o gerador, o
    // levantar entrega quando chegou: um pe no chao e o centro de massa sem subir nem descer rapido.
    bool arrived(const World& w) const {
        if (!settings.balanceSteps) return true;
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        bool foot = false;
        for (const RagdollInteraction3D& c : w.body->interactions)
            foot = foot || ((c.linkIndex == footLinks[0] || c.linkIndex == footLinks[1]) && c.normalWorld.z > 0.5f);
        // (15h25: exigir |v.z| < 0,35 m/s deixava a recuperacao de pe sem entregar - ela desaba de pe desde que entrega na hora,
        // 09h00 -; so subindo (a decolagem do fim do levantar) e no ar nao entrega; descendo no toque, sim.)
        return foot && v.z < ArrivedRiseSpeed;
    }
    static constexpr float ArrivedRiseSpeed = 0.3f, RiseHeadHeight = 1.0f, CalmMargin = 0.05f, CalmSpeed = 0.4f;
    // (05/11 02h15) A entrega na subida (riseHandoffHead): ja subindo, os dois pes no chao, o corpo alto e o tronco em pe.
    static constexpr float RiseHandoffPelvis = 0.55f, RiseHandoffUpright = 0.6f;
    bool risingIntoIdle(const World& w, const TrackingGetUpPosture3D& now) const {
        if (settings.riseHandoffHead <= 0.0f || !w.idleAfterGetUp || w.locomotionControl || settings.idleBlendSeconds <= 0.0f
            || now.headHeight < settings.riseHandoffHead || now.pelvisHeight < RiseHandoffPelvis || now.upright < RiseHandoffUpright)
            return false;
        bool left = false, right = false;
        for (const RagdollInteraction3D& c : w.body->interactions) {
            left = left || (c.linkIndex == footLinks[0] && c.normalWorld.z > 0.5f);
            right = right || (c.linkIndex == footLinks[1] && c.normalWorld.z > 0.5f);
        }
        return left && right;
    }
    // (21h35: a entrega calma - ver calmHandoffSeconds.)
    bool calm(World& w) const {
        if (settings.calmHandoffSeconds <= 0.0f) return true;
        if (w.calmFrom < 0.0f) w.calmFrom = w.time;
        if (w.time - w.calmFrom >= settings.calmHandoffSeconds) return true;
        const TrackingBalance3D b = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions,
            { footLinks[0], footLinks[1] }, floorUnder(w));
        return b.margin > -CalmMargin && std::hypot(b.velocity.x, b.velocity.y) < CalmSpeed;
    }
    // (04/11 15h, os passos) O gerador de passos so no idle da cabeca da locomocao.
    bool stepping(const World& w) const {
        return settings.balanceSteps && w.stepsOn && legs.valid && footGeometry.valid && (idleHeld(w) || parkedIdle(w));
    }
    // (10/11 11h50, settings.basePosture; ele: "da a sensacao de isso ser um outro modo que tu configurou... o que e incorreto") O
    // parado depois de uma parada na locomocao e o mesmo parado do exercicio de parado: o idle alerta seguido, com o gerador de passos
    // (o passo sugerido e a base na observacao, as pernas do passo na nota) - nao um modo a parte.
    bool parkedIdle(const World& w) const {
        return (settings.basePosture > 0.0f || settings.stopToIdle || settings.idleAfterStop) && w.locomotion && w.locomotionControl && w.stopping
            && w.stopPhase == 2 && idleTracked(w);
    }
    // O quadro do clipe seguido (sem por no mundo) posto na pelve do corpo: a raiz no plano da pelve (mais `shift`), na altura
    // do clipe sobre o chao, girada para o rumo da pelve do corpo.
    std::pair<Vec3, Quaternion> placedOnBody(World& w, float ahead, Vec3 shift) {
        referenceFrame(w, ahead, false, w.stepFrame);
        const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
        // (11/11, E5-15: no parado depois do parkStop, o idle no rumo do alvo (o do rastreio), nao no da pelve de agora - medido na
        // E5-12: com o idle girando junto com o corpo, cada passo do gerador reajustava a base na pelve atual e nunca o rumo, e a
        // pelve derivava 15 -> 50 graus passo a passo enquanto o peito torcia de volta para a vista, a coluna de lado.)
        const float angle = parkedIdle(w) ? trackSample(w, ahead).second
                                          : wrapAngle(pelvisHeading(pelvis) - pelvisHeading(w.stepFrame.links[rootLink]));
        const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, angle);
        const Vec3 root { pelvis.position.x + shift.x, pelvis.position.y + shift.y, floorUnder(w) + w.stepFrame.rootPosition.z };
        return { root, (turn * w.stepFrame.rootOrientation).normalized() };
    }
    void updateSteps(World& w, std::span<const float> channels) {
        if (!(settings.balanceSteps && !settings.freeIdle && legs.valid && footGeometry.valid && (idleHeld(w) || parkedIdle(w)))) {
            if (w.stepsOn) w.steps.reset();
            w.stepsOn = false;
            return;
        }
        if (!w.stepsOn) {
            w.steps.reset();
            w.stepsOn = true;
        }
        w.steps.maximumSwingLoad = settings.stepLoadLimit;
        w.steps.loadWeight = settings.stepLoadWeight;
        const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
        const float ground = floorUnder(w);
        TrackingBalanceStepsInput3D in;
        in.balance = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions, { footLinks[0], footLinks[1] }, ground);
        in.pelvis = pelvis.position;
        in.groundHeight = ground;
        const auto [root, orientation] = placedOnBody(w, 0.0f, {});
        // (E5-15: parado depois do parkStop, o rumo do gerador e o da pelve do idle posto no alvo.)
        in.facingYaw = parkedIdle(w) ? wrapAngle(trackSample(w, 0.0f).second + pelvisHeading(w.stepFrame.links[rootLink]))
                                     : pelvisHeading(pelvis);
        athleteWorldPose3D(*profile, w.stepFrame.coordinates, root, orientation, w.stepPose);
        for (std::size_t s = 0; s < 2; ++s) {
            in.foot[s] = w.body->links[footLinks[s]].position;
            in.footOrientation[s] = w.body->links[footLinks[s]].orientation;
            in.idleFoot[s] = w.stepPose.linkPositions[footLinks[s]];
            in.idleOrientation[s] = w.stepPose.linkOrientations[footLinks[s]];
        }
        const auto channel = [&](std::size_t k) { return k < channels.size() && std::isfinite(channels[k]) ? channels[k] : 0.0f; };
        in.command = { channel(0), channel(1), channel(2), channel(3) };
        in.deltaTime = TicksPerDecision * TickSeconds;
        const bool swinging = w.steps.swinging() >= 0;
        w.steps.update(*profile, { footLinks[0], footLinks[1] }, in);
        if (w.steps.recoveryStarted()) ++w.recoveryStepCount;
        if (w.steps.settleStarted()) ++w.settleStepCount;
        // (O ponto do idle acompanha o passo: re-ancorado em cada pouso.)
        // (15/11, settings.idleClipAnchor: so o passo de recuperacao - empurrao recente ou fogo - re-ancora; um passo sem motivo depois
        // do levantar re-ancorava o parado onde o pe pousava e a rede andava passo a passo sem pagar nada - medido 23 e 15 cm.)
        if (swinging && w.steps.swinging() < 0
            && (!settings.idleClipAnchor || w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f || w.fireKind != 0 || w.locoFire))
            w.idleAnchored = false;
        w.balance = in.balance;
    }
    // Os alvos do idle com as pernas do passo: o IK leva os pes aos do gerador a partir da raiz do clipe posta na pelve (mais o
    // quanto o gerador leva a pelve para cima da base), e as pernas vao do clipe ao IK no peso do gerador.
    void stepTargets(World& w, float ahead, std::vector<float>& out, bool keepPose) {
        const auto [root, orientation] = placedOnBody(w, ahead, w.steps.rootShift());
        w.clipCoordinates = w.stepFrame.coordinates;
        athleteWorldPose3D(*profile, w.stepFrame.coordinates, root, orientation, w.balancePose);
        std::array<FootTarget3D, 2> feet {};
        w.steps.feet(ahead, feet);
        // (Os pes apoiados a partir da pelve que o idle quer - em pe, por cima da base: a perna de apoio leva o corpo para la -;
        // o pe no ar a partir da pelve de agora, na altura e na inclinacao reais - 15h20, a sonda: com o corpo afundando, o alvo
        // dele partindo da altura do clipe ficava dentro do chao. O alvo igual ao pe da pose o IK pula.)
        const int swing = w.steps.swinging();
        std::array<FootTarget3D, 2> planted = feet;
        if (swing >= 0 && settings.stepSwingFromBody) {
            const std::size_t s = static_cast<std::size_t>(swing);
            planted[s] = { FootMode3D::Free, w.balancePose.linkPositions[legs.foot[s]], w.balancePose.linkOrientations[legs.foot[s]] };
        }
        synthesizeAthleteLegs3D(*profile, legs, footGeometry, planted, root, orientation, w.stepFrame.coordinates, w.balancePose);
        if (swing >= 0 && settings.stepSwingFromBody) {
            const std::size_t s = static_cast<std::size_t>(swing), o = 1 - s;
            const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
            athleteWorldPose3D(*profile, w.stepFrame.coordinates, pelvis.position, pelvis.orientation, w.balancePose);
            std::array<FootTarget3D, 2> air {};
            air[o] = { FootMode3D::Free, w.balancePose.linkPositions[legs.foot[o]], w.balancePose.linkOrientations[legs.foot[o]] };
            air[s] = feet[s];
            synthesizeAthleteLegs3D(*profile, legs, footGeometry, air, pelvis.position, pelvis.orientation, w.stepFrame.coordinates,
                w.balancePose);
        }
        const float k = w.steps.weight();
        for (int side = 0; side < 2; ++side)
            for (std::size_t link : { legs.thigh[static_cast<std::size_t>(side)], legs.shin[static_cast<std::size_t>(side)],
                     legs.foot[static_cast<std::size_t>(side)] })
                if (link < w.stepFrame.coordinates.size())
                    w.stepFrame.coordinates[link] = w.clipCoordinates[link] + (w.stepFrame.coordinates[link] - w.clipCoordinates[link]) * k;
        contract->actionOf(w.stepFrame.coordinates, out);
        if (keepPose) {
            // (A pose da nota a partir da pelve de agora: onde os motores poem os pes com estas pernas.)
            const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
            athleteWorldPose3D(*profile, w.stepFrame.coordinates, pelvis.position, pelvis.orientation, w.stepPose);
            w.stepPoseValid = true;
            w.stepPoseWeight = k;
        }
    }
    void clipTargets(World& w, float ahead, std::vector<float>& out) {
        out.assign(contract->actionSize(), 0.0f);
        if (stepping(w) && w.steps.weight() > 0.0f) {
            stepTargets(w, ahead, out, ahead > 0.0f);
            return;
        }
        if (!settings.balanceFeet || !legs.valid || !footGeometry.valid) {
            referenceFrame(w, ahead, false, w.frame);
            contract->actionOf(w.frame.coordinates, out);
            return;
        }
        referenceFrame(w, ahead, true, w.frame);
        const Vec3 com = rlBodyCenterOfMass3D(*profile, w.body->links);
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        const float speed = starting(w) ? std::min(w.commandSpeed, startSpeed(w, ahead)) : commandNow(w, w.time + ahead);
        const float omega = std::sqrt(9.81f / std::max(0.5f, com.z - floorUnder(w)));
        // (So a velocidade: o centro de massa do clipe posto pela pelve difere do corpo pela pose, um desvio fixo.)
        Vec3 shift { (v.x - std::cos(w.moveYaw) * speed) / omega, (v.y - std::sin(w.moveYaw) * speed) / omega, 0.0f };
        const float size = shift.length();
        if (size > 0.3f) shift = shift * (0.3f / size);
        athleteWorldPose3D(*profile, w.frame.coordinates, w.frame.rootPosition, w.frame.rootOrientation, w.balancePose);
        std::array<FootTarget3D, 2> feet {};
        bool any = false;
        for (int side = 0; side < 2; ++side) {
            const std::size_t foot = legs.foot[static_cast<std::size_t>(side)], other = legs.foot[static_cast<std::size_t>(1 - side)];
            feet[side].position = w.balancePose.linkPositions[foot];
            feet[side].orientation = w.balancePose.linkOrientations[foot];
            if (w.balancePose.linkPositions[foot].z > w.balancePose.linkPositions[other].z + 0.02f) {
                feet[side].mode = FootMode3D::Swing;
                feet[side].position = feet[side].position + shift;
                any = true;
            }
        }
        if (any)
            synthesizeAthleteLegs3D(*profile, legs, footGeometry, feet, w.frame.rootPosition, w.frame.rootOrientation,
                w.frame.coordinates, w.balancePose);
        contract->actionOf(w.frame.coordinates, out);
    }
    float twist(const World& w) const {
        return w.trackClip == w.twistClip || (starting(w) && w.pendingClip == w.twistClip) ? w.trackTwist : 0.0f;
    }
    // (04/11 12h20, ele: "no trote de costas e nas diagonais costas, no inicio ele tenta trotar para tras com a coluna muito
    // inclinada para frente e fica patinando ate pegar no tranco - o correto seria jogar a coluna para tras, ou ate mesmo em
    // todo o movimento a coluna estar para tras e nao para frente, para ajudar na inercia": nos rumos de costas (3, 4 e 5,
    // trote e corrida) a coluna da referencia vai BackLean para tras - os clipes de costas tem o tronco quase reto, 1-2 graus
    // para a frente.)
    static constexpr float BackLean = -0.14f;
    // O lado da inclinacao da coluna: -1 nos rumos de costas (3-5), +1 no trote de frente e nas diagonais da frente (0, 1, 7);
    // so sem manobra, com pedido e fora da arrancada da corrida.
    // (04/11 16h, ele: "no trote curva ele ainda leva a coluna para tras, apesar de estar correto no trote frente": as curvas
    // 5-7 seguem o clipe da frente no rumo 0, entao a coluna da frente vale nelas tambem.)
    int leanSide(const World& w) const {
        // (04/11 23h40: e nas trocas 1-4 tambem - o rumo delas vale como o reto.)
        if (!(settings.trackingReward >= 10 && w.locomotion && w.commandSpeed > 0.0f && !starting(w)))
            return 0;
        if (w.heading >= 3 && w.heading <= 5) return -1;
        return !w.runBand && (w.heading == 0 || w.heading == 1 || w.heading == 7) ? 1 : 0;
    }
    // (Na largada de pe com a inclinacao propria dela - a coluna cobrada inteira ali.)
    bool startLeaning(const World& w) const {
        const int side = leanSide(w);
        return launchWeight(w) > 0.0f
            && (side < 0 ? settings.backStartLean < BackLean : side > 0 && settings.frontStartLean > settings.frontLean);
    }
    // (04/11 23h50, ele: "nas trocas nao ha movimento de pegar no tranco... nem de freada e inversao") Nas trocas 1-4 a
    // coluna da referencia (nos motores) soma a inclinacao da aceleracao da velocidade pedida no rumo do peito, atan(a / g)
    // ate accelLean: freando ou arrancando para tras, para tras; arrancando para a frente, para a frente - qualquer angulo.
    // (Medido 23h45: inclinar o quadro inteiro, pernas junto, piorava a firmeza das trocas 0,60 -> 0,55; a coluna e o
    // jeito da largada, que ele aprovou.)
    float accelPitch(const World& w) const {
        if (settings.accelLean <= 0.0f || !w.locomotion || ((w.maneuver < 1 || w.maneuver > 4) && !isStar(w) && !isCommand(w))) return 0.0f;
        const float along = w.refAccel.x * std::cos(w.faceYaw) + w.refAccel.y * std::sin(w.faceYaw);
        return std::clamp(std::atan(along / 9.81f), -settings.accelLean, settings.accelLean);
    }
    float pitch(const World& w) const {
        if (w.stopClip >= 0 || unrealStarting(w)) return 0.0f;
        return leanPitch(w) + accelPitch(w) + runPitch(w);
    }
    // (05/11 00h40, ele: "no corrida... o corpo todo mais inclinado para frente, nao a coluna apenas") A corrida de frente,
    // as diagonais da frente e as curvas da corrida (rumo 0): o dorso reto runLean para a frente, e as pernas acompanham
    // runLeanLegs dele (legPitch) - os pes mais para tras da pelve, o corpo todo inclinado. Fora da meia-volta (ela tem a
    // inclinacao dela) e da largada.
    float runPitch(const World& w) const {
        if (!w.runBand || w.runTurn >= 0) return 0.0f;
        if (!(settings.trackingReward >= 10 && w.locomotion && w.commandSpeed > 0.0f && !starting(w)))
            return 0.0f;
        if (w.band == 3) return w.heading == 0 ? settings.sprintLean : 0.0f;
        if (w.heading == 0 || w.heading == 1 || w.heading == 7) return settings.runLean;
        // (05/11 04h10: e para tras na corrida de costas, runBackLean.)
        return backRun(w) ? -settings.runBackLean : 0.0f;
    }
    static bool backRun(const World& w) { return w.band != 3 && w.heading >= 3 && w.heading <= 5; }
    float legPitch(const World& w) const {
        if (w.stopClip >= 0 || unrealStarting(w)) return 0.0f;
        return runPitch(w) * (w.band == 3 ? settings.sprintLeanLegs : backRun(w) ? settings.runBackLeanLegs : settings.runLeanLegs);
    }
    float leanPitch(const World& w) const {
        const int side = leanSide(w);
        // (04/11 14h15, ele: "no trote de frente e diagonais frente, uma inclinada na coluna para frente para pegar tranco
        // quando comeca, e no trote nao exigir a coluna perfeitamente reta - uma leve inclinada para frente, hoje ela vai
        // muito para tras": medido, a 19% de forca o tronco 4-13 graus PARA TRAS na largada e 9 no trote. A coluna a
        // frontLean e, na largada de pe, a frontStartLean voltando a ela com a janela da arrancada.)
        if (side > 0) return settings.frontLean + std::max(0.0f, settings.frontStartLean - settings.frontLean) * launchWeight(w);
        if (side == 0) return 0.0f;
        // (04/11 13h40, ele: "no comeco ele tenta arrancar com a coluna para frente ao inves de para tras": medido, sem a mao
        // o clipe de costas tomba para a frente - o passo para tras deixa o centro de massa na frente dos pes - e a rede
        // herdou, 9-16 graus para a frente e parada. Na largada de pe, a coluna vai a backStartLean e volta a BackLean com a
        // janela da arrancada.)
        return settings.backStartLean < BackLean ? BackLean + (settings.backStartLean - BackLean) * launchWeight(w) : BackLean;
    }
    // A largada pela arrancada (idle_to_sprint antes do ciclo): a velocidade dela no instante tocado, na escala - sobe de 0
    // ao fim do clipe como na fonte (aceleracao constante).
    static constexpr float StartClipScaleMinimum = 0.55f, StartClipScaleMaximum = 1.25f, StartBlendSeconds = 0.3f;
    bool unrealStarting(const World& w) const {
        return w.unrealStart >= 0 && w.pendingClip < references.size() && w.trackClip < references.size();
    }
    bool starting(const World& w) const {
        return w.pendingClip < references.size() && w.trackClip < references.size()
            && (w.trackClip == startClipIndex || w.trackClip == launchExitIndex || w.unrealStart >= 0);
    }
    float startSpeed(const World& w, float ahead) const {
        if (unrealStarting(w)) {
            // (A arrancada da Unreal: a velocidade da raiz dela no instante tocado, na escala.)
            const UnrealStart& c = startInfo[static_cast<std::size_t>(w.unrealStart)];
            const float t = std::max(0.0f, w.trackStart + (w.time + ahead) * w.trackScale);
            const Vec3 v = c.velocity[std::min(c.velocity.size() - 1, static_cast<std::size_t>(t / StopSample))];
            return w.trackScale * std::hypot(v.x, v.y);
        }
        const std::size_t clip = w.trackClip == launchExitIndex ? launchExitIndex : startClipIndex;
        const float d = std::max(1.0e-3f, references[clip]->durationSeconds());
        const float u = std::clamp(w.trackStart + (w.time + ahead) * w.trackScale, 0.0f, d);
        return w.trackScale * 2.0f * travel[clip].rampDisplacement * u / (d * d);
    }
    // O quadro de referencia do clipe seguido daqui a `ahead` s: posto sob a pelve do corpo no rumo do quadro (placed) ou
    // so as coordenadas (o alvo dos motores). Numa troca de clipe da manobra, o de antes se mistura no novo pela transicao
    // (smoothstep), e nas curvas o giro do rumo entra na velocidade angular do quadro.
    // (15/11, settings.clipUnderPelvis) O quadro posto com o chao em (x, y): a pelve do clipe vai para (x, y), nao a origem dele.
    void underPelvis(TrackingClipFrame3D& f, float x, float y) const {
        if (settings.clipUnderPelvis) pelvisTo(f, x, y);
    }
    // A pelve do quadro em (x, y), o quadro inteiro junto.
    void pelvisTo(TrackingClipFrame3D& f, float x, float y) const {
        if (rootLink >= f.links.size()) return;
        const float dx = x - f.links[rootLink].position.x, dy = y - f.links[rootLink].position.y;
        for (PhysicsBodyState3D& link : f.links) {
            link.position.x += dx;
            link.position.y += dy;
        }
        f.rootPosition.x += dx;
        f.rootPosition.y += dy;
        f.centerOfMass.x += dx;
        f.centerOfMass.y += dy;
    }
    void referenceFrame(World& w, float ahead, bool placed, TrackingClipFrame3D& out) const {
        const auto [t, yaw] = trackSample(w, ahead);
        const Vec3 root = w.body->links[rootLink].position;
        const Vec3 ground = placed ? Vec3 { root.x, root.y, floorUnder(w) } : Vec3 {};
        references[w.trackClip]->frame(t, ground, placed ? yaw : 0.0f, out, twist(w), pitch(w), legPitch(w));
        const float now = w.time + ahead;
        if (w.blendClip < references.size() && now < w.blendFrom + w.blendSeconds) {
            const TrackingClipReference3D& old = *references[w.blendClip];
            const float d = std::max(1.0e-3f, old.durationSeconds());
            float u = w.blendStart + now * w.blendScale;
            float blendYawNow = w.blendYaw;
            if (settings.reviewFixes) {
                // (settings.reviewFixes) O ciclo enrola com o rumo dos ciclos (o giro); o clipe sem ciclo fica no fim.
                if (old.clip().loops) {
                    const float cycles = std::floor(u / d);
                    u -= cycles * d;
                    blendYawNow += cycles * old.clip().turnCycleRadians;
                } else {
                    u = std::clamp(u, 0.0f, d);
                }
                old.frame(u, ground, placed ? blendYawNow : 0.0f, w.blendFrame, w.blendTwist, w.blendPitch, w.blendLegPitch);
            } else {
                u -= std::floor(u / d) * d;
                old.frame(u, ground, placed ? w.blendYaw : 0.0f, w.blendFrame, w.blendTwist);
            }
            const float x = std::clamp((now - w.blendFrom) / std::max(1.0e-3f, w.blendSeconds), 0.0f, 1.0f);
            const float k = x * x * (3.0f - 2.0f * x);
            const auto mix = [k](Quaternion a, Quaternion b) {
                if (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w < 0.0f) b = { -b.x, -b.y, -b.z, -b.w };
                return Quaternion { a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k, a.z + (b.z - a.z) * k, a.w + (b.w - a.w) * k }
                    .normalized();
            };
            for (std::size_t i = 0; i < out.coordinates.size() && i < w.blendFrame.coordinates.size(); ++i)
                out.coordinates[i] = w.blendFrame.coordinates[i] + (out.coordinates[i] - w.blendFrame.coordinates[i]) * k;
            out.rootPosition = w.blendFrame.rootPosition + (out.rootPosition - w.blendFrame.rootPosition) * k;
            out.rootOrientation = mix(w.blendFrame.rootOrientation, out.rootOrientation);
            for (std::size_t i = 0; i < out.links.size() && i < w.blendFrame.links.size(); ++i) {
                PhysicsBodyState3D& b = out.links[i];
                const PhysicsBodyState3D& a = w.blendFrame.links[i];
                b.position = a.position + (b.position - a.position) * k;
                b.orientation = mix(a.orientation, b.orientation);
                b.linearVelocity = a.linearVelocity + (b.linearVelocity - a.linearVelocity) * k;
                b.angularVelocity = a.angularVelocity + (b.angularVelocity - a.angularVelocity) * k;
            }
            out.centerOfMass = w.blendFrame.centerOfMass + (out.centerOfMass - w.blendFrame.centerOfMass) * k;
        }
        if (placed) underPelvis(out, root.x, root.y);
        if (placed && w.maneuver >= 4 && w.arcRate != 0.0f)
            for (PhysicsBodyState3D& link : out.links) link.angularVelocity.z += w.arcRate;
        // (04/11 14h05, ele: "adaptar as curvas para ele fazer com a inclinacao corporal correta - dinamica, conforme o
        // fechamento ou abertura da curva; hoje ele inclina o corpo para o lado oposto ao fechamento": nas curvas o quadro
        // inteiro gira em volta do ponto do chao sob a pelve, para dentro, atan(v x giro / g) - a inclinacao que equilibra
        // a curva, ate CurveLeanMaximum. Rigido: as juntas - os motores - nao mudam; mudam a pelve e o corpo que a mao segura
        // e a inclinacao cobrada.)
        if (placed && settings.curveLean && isCurve(w) && w.arcRate != 0.0f) {
            const float speed = w.refActive ? w.refVelocity.length() : w.commandSpeed;
            const float bank = std::clamp(std::atan(speed * w.arcRate / 9.81f), -CurveLeanMaximum, CurveLeanMaximum);
            const Quaternion roll = Quaternion::fromAxisAngle({ std::cos(w.moveYaw), std::sin(w.moveYaw), 0.0f }, -bank);
            const auto turn = [&](Vec3 p) { return ground + roll.rotate(p - ground); };
            out.rootPosition = turn(out.rootPosition);
            out.rootOrientation = (roll * out.rootOrientation).normalized();
            out.centerOfMass = turn(out.centerOfMass);
            for (PhysicsBodyState3D& link : out.links) {
                link.position = turn(link.position);
                link.orientation = (roll * link.orientation).normalized();
                link.linearVelocity = roll.rotate(link.linearVelocity);
                link.angularVelocity = roll.rotate(link.angularVelocity);
            }
        }
    }
    static constexpr float CurveLeanMaximum = 0.45f;
    // A velocidade pedida agora: na manobra, a de referencia (freia e reacelera); senao o pedido na rampa da largada.
    Vec3 wantedVelocity(const World& w, float ahead) const {
        if (starting(w)) {
            const float speed = std::min(w.commandSpeed, startSpeed(w, ahead));
            return { std::cos(w.moveYaw) * speed, std::sin(w.moveYaw) * speed, 0.0f };
        }
        if (w.refActive) return w.refVelocity;
        const float speed = commandNow(w, w.time + ahead);
        return { std::cos(w.moveYaw) * speed, std::sin(w.moveYaw) * speed, 0.0f };
    }
    // O exercicio da forca auxiliar por exercicio (TrackingGetUpAmpSettings3D::drillAssist).
    std::size_t drillOf(const World& w) const {
        if (guidedRise(w)) return TrackingGetUpAmpSettings3D::GetUpDrill;
        if (idleHeld(w)) return 24;
        if (w.startKind == 7) return 23;
        // (04/11 11h50: as curvas na corrida com a forca delas, 25-27.)
        // (08/11: a estrela, 30-35.)
        if (isStar(w)) return TrackingGetUpAmpSettings3D::StarDrill + (w.runBand ? 3u : 0u) + static_cast<std::size_t>(w.maneuver - 8);
        if (isCommand(w)) return TrackingGetUpAmpSettings3D::CommandDrill;
        if (w.maneuver >= 5 && w.runBand) return 20 + static_cast<std::size_t>(std::min(w.maneuver, 7));
        if (w.maneuver > 0) return 15 + static_cast<std::size_t>(std::min(w.maneuver, 7));
        if (w.band == 3) return TrackingGetUpAmpSettings3D::SprintDrill;
        return (w.runBand ? 8 : 0) + (w.heading % 8u);
    }
    float baseAssistLevel(const World& w) const {
        if (const float drill = settings.drillAssist[drillOf(w)]; drill >= 0.0f) return std::clamp(drill, 0.0f, 1.0f);
        if (w.maneuver > 0 && settings.maneuverAssist >= 0.0f) return std::clamp(settings.maneuverAssist, 0.0f, 1.0f);
        return std::clamp(w.runBand && settings.runAssist >= 0.0f ? settings.runAssist : settings.locomotionAssist, 0.0f, 1.0f);
    }
    // (10/11 07h15, settings.launchHand; ele aprovou "mao so na largada": copiar so os angulos do clipe fazia o corpo pular no lugar
    // - o impulso vem do corpo inteiro tombar para a frente, que motor nenhum controla. Na janela da largada, a mao na pelve no nivel
    // launchHand x a janela; o treino o desce a 0 junto com a parcela do clipe. Fora da largada, a de sempre.)
    float assistLevel(const World& w) const {
        const float base = baseAssistLevel(w);
        return settings.launchHand > 0.0f && w.locomotion && starting(w)
            ? std::max(base, std::clamp(settings.launchHand, 0.0f, 1.0f) * launchWeight(w)) : base;
    }
    // A entrega ao idle depois de levantar (idleBlendSeconds): o peso do idle no instante t (smoothstep de 0 a 1) - dos alvos
    // dos motores e da mao -, e o alcance da correcao da rede no idle, que vai de idleBlendReach ao do idle junto.
    bool idleBlending(const World& w) const {
        return settings.idleBlendSeconds > 0.0f && idleHeld(w) && w.time < w.idleBlendFrom + settings.idleBlendSeconds;
    }
    float idleBlendWeight(const World& w, float t) const {
        if (settings.idleBlendSeconds <= 0.0f) return 1.0f;
        const float x = std::clamp((t - w.idleBlendFrom) / settings.idleBlendSeconds, 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }
    float idleReach(const World& w) const {
        float base = settings.idleResidualScale;
        if (settings.idleThreatReach > base && w.threatKnown) {
            // (04/11 15h45: a autoridade sob ameaca - ver idleThreatReach.)
            const float x = std::clamp((w.threat - 0.4f) / 0.6f, 0.0f, 1.0f);
            base += (settings.idleThreatReach - base) * x * x * (3.0f - 2.0f * x);
        }
        if (!idleBlending(w) || settings.idleBlendReach < 0.0f) return base;
        const float k = idleBlendWeight(w, w.time);
        return settings.idleBlendReach + (base - settings.idleBlendReach) * k;
    }
    // A mao linear: o braco de alavanca do torque de tombar (m).
    static constexpr float LinearHandLever = 1.0f;
    // (05/11 04h40) O levantar pelo clipe agora (getUpTracking).
    bool guidedRise(const World& w) const { return settings.getUpTracking && w.getUpGuided; }
    // (12h) A corrida de lado (2 e 6), sem manobra.
    bool lateralRun(const World& w) const {
        return w.locomotion && w.runBand && w.maneuver == 0 && (w.heading == 2 || w.heading == 6);
    }
    // O idle na cabeca da locomocao (idleOnLocomotion): de pe, seguindo o parado em alerta.
    bool idleHeld(const World& w) const {
        return settings.idleOnLocomotion && settings.separateSkills && w.locomotionControl && !w.locomotion && idleTracked(w);
    }
    // O alvo e a velocidade-alvo dos motores da cabeca da locomocao sobre o clipe (o ramo locomotionResidual de decide), com
    // a correcao `action` em radianos. (06/11: tambem o rotulo da professora da locomocao livre, no mesmo instante.)
    void composeResidual(World& w, std::span<const float> action, std::vector<float>& target, std::vector<float>& velocity) {
        // (A velocidade do clipe na decisao como velocidade-alvo dos motores: a diferenca do alvo do fim desta decisao
        // para o do comeco dela, no tempo do clipe tocado.)
        const float dt = TicksPerDecision * TickSeconds;
        clipTargets(w, 0.0f, velocity);
        clipTargets(w, dt, target);
        // (04/11 09h00, ele: "parece haver 2 fases entre ficar de pe e ficar na pose idle alerta... o objetivo deveria ser
        // levantar-se para ficar de pe no idle alerta diretamente": a entrega e logo ao ficar de pe, e os alvos vao dos da
        // recuperacao aos do idle em IdleBlendSeconds (smoothstep) - antes ela esperava o corpo quieto e o idle entrava de
        // uma vez.)
        const bool blending = idleBlending(w) && w.idleBlendPose.size() == action.size();
        // (04/11 15h20: as pernas do passo nao entram na mistura da entrega - ela diluia o passo logo depois de levantar.)
        const bool stepLegs = stepping(w) && w.steps.weight() > 0.0f;
        if (blending) {
            const float k0 = idleBlendWeight(w, w.time), k1 = idleBlendWeight(w, w.time + dt);
            for (std::size_t k = 0; k < action.size(); ++k) {
                if (stepLegs && k < legAxes.size() && legAxes[k]) continue;
                velocity[k] = w.idleBlendPose[k] + (velocity[k] - w.idleBlendPose[k]) * k0;
                target[k] = w.idleBlendPose[k] + (target[k] - w.idleBlendPose[k]) * k1;
            }
        }
        for (std::size_t k = 0; k < action.size(); ++k) {
            float d = target[k] - velocity[k];
            // (Na volta do ciclo o angulo nao salta - so a passagem do fim ao comeco do clipe; ali, sem velocidade.)
            velocity[k] = std::abs(d) < 1.0f ? d / dt : 0.0f;
            // (04/11 05h25: no idle da cabeca da locomocao o alcance da correcao e menor - ela trazia as correcoes da corrida
            // e tirava o corpo da pose: com o clipe do idle puro a semelhanca e 0,82, com a correcao inteira 0,25.)
            const float legScale = k < legAxes.size() && legAxes[k] && lateralRun(w) ? settings.lateralRunLegReach : 1.0f;
            if (std::isfinite(action[k])) target[k] += action[k] * (idleHeld(w) ? idleReach(w) : legScale);
        }
    }
    bool assisted(const World& w) const {
        // (12h15, ele: "garanta que os exercicios do levantar e evitar queda nao tem ajuda de forca auxiliar em nenhuma etapa -
        // suspeito que esta tendo depois que ele fica de pe": tinha - o idle depois de levantar ou de evitar recebia a forca do
        // parado. Agora a mao no idle e so no exercicio do parado.)
        if (guidedRise(w)) return assistLevel(w) > 0.0f && !w.fallActive;
        if (idleHeld(w)) return w.startKind == 2 && !w.balanceDrill && assistLevel(w) > 0.0f && !w.fallActive;
        return assistLevel(w) > 0.0f && w.locomotion && locomotionCommanded(w) && clipTracked(w)
            && (!settings.separateSkills || w.locomotionControl) && !w.fallActive && !w.getUpGuided;
    }
    void settle(World& w, int ticks) {
        // (17/11, settings.projectileRate - ver o .hpp) O projetil: a chance por decisao, na locomocao de pe do treino.
        if (settings.projectileRate > 0.0f && !settings.evaluation && w.locomotion && !w.down && w.projectileTicks == 0
            && !w.body->links.empty()) {
            std::uniform_real_distribution<float> u(0.0f, 1.0f);
            if (u(w.rng) < settings.projectileRate * TicksPerDecision * TickSeconds) {
                const float angle = 2.0f * Pi * u(w.rng);
                const float up = 0.3f * (2.0f * u(w.rng) - 1.0f);
                const float seconds = std::max(TickSeconds, settings.projectileSeconds);
                const float impulse = settings.projectileImpulse[0] + (settings.projectileImpulse[1] - settings.projectileImpulse[0]) * u(w.rng);
                const Vec3 direction = Vec3 { std::cos(angle), std::sin(angle), up }.normalized();
                w.projectileForce = direction * (impulse / seconds);
                w.projectileTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
                w.projectileLink = std::min(static_cast<std::uint32_t>(w.body->links.size() - 1),
                    static_cast<std::uint32_t>(u(w.rng) * static_cast<float>(w.body->links.size())));
            }
        }
        const bool assist = assisted(w);
        w.handUp = w.handSide = w.handTurn = 0.0f;
        float assistHeight = 0.0f;
        Quaternion assistOrientation;
        // (04/11 09h55: o amortecimento do giro da pelve contra o giro do clipe - contra zero, a 100% a mao freava o giro parado
        // e o erro do giro da pelve era 14 (rad/s)^2, contra 0,3 a 30%.)
        Vec3 assistSpin;
        const std::size_t handCount = settings.bodyHand ? assistLinks.size() : settings.upperHand ? upperHandCount : 0;
        w.handTarget.resize(assistLinks.size());
        w.handSpin.resize(assistLinks.size());
        if (assist) {
            // (O quadro do meio da decisao, sob a pelve do corpo: a altura e a orientacao da pelve do clipe.)
            referenceFrame(w, 0.5f * TicksPerDecision * TickSeconds, true, w.frame);
            assistHeight = w.frame.links[rootLink].position.z;
            assistOrientation = w.frame.links[rootLink].orientation;
            const float rate = w.trackScale * startRamp(w, w.time);
            assistSpin = w.frame.links[rootLink].angularVelocity * rate;
            for (std::size_t u = 0; u < handCount; ++u)
                if (assistLinks[u] < w.frame.links.size()) {
                    w.handTarget[u] = w.frame.links[assistLinks[u]].orientation;
                    w.handSpin[u] = w.frame.links[assistLinks[u]].angularVelocity * rate;
                }
        }
        // (10/11, settings.motorTone: o tonus do corpo - uma copia dos alvos com a rigidez e o amortecimento escalados, sem acumular.)
        const std::vector<RagdollDriveTarget3D>* drives = &w.drives;
        if (settings.motorTone != 1.0f || settings.spineTone != 1.0f || settings.armTone != 1.0f) {
            w.tonedDrives = w.drives;
            for (RagdollDriveTarget3D& d : w.tonedDrives) {
                // (settings.spineTone: a coluna com o tonus dela por cima do do corpo; settings.armTone, os bracos.)
                const bool spine = d.linkIndex < spineLinks.size() && spineLinks[d.linkIndex];
                const bool arm = d.linkIndex < armLinks.size() && armLinks[d.linkIndex];
                const float tone = settings.motorTone * (spine ? settings.spineTone : 1.0f) * (arm ? settings.armTone : 1.0f);
                d.stiffnessScale *= tone;
                d.dampingScale *= tone;
            }
            drives = &w.tonedDrives;
        }
        // (10/11, settings.motorGravity: a compensacao da gravidade nos motores da cabeca da locomocao.)
        const bool gravity = w.driveGravity || (settings.motorGravity && w.locomotionControl);
        for (int k = 0; k < ticks; ++k) {
            w.scene->setRagdollActiveDriveTargets(w.handle, *drives, gravity);
            if (assist) {
                const RagdollState3D& s = *w.scene->ragdollStateView(w.handle);
                const PhysicsBodyState3D& pelvis = s.links[rootLink];
                const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, s.links);
                const Vec3 wanted = wantedVelocity(w, static_cast<float>(k) * TickSeconds);
                const float m = profile->totalMassKg, a = assistLevel(w) * (idleBlending(w) ? idleBlendWeight(w, w.time) : 1.0f);
                Vec3 force { m * AssistVelocityGain * (wanted.x - v.x), m * AssistVelocityGain * (wanted.y - v.y),
                    m * (AssistHeightGain * (assistHeight - pelvis.position.z) - AssistHeightDamping * pelvis.linearVelocity.z) };
                // (04/11 11h05, ele: "varios movimentos desandaram e estao caindo mais": o amortecimento 80 por igual tirou a ajuda
                // da forca baixa - a 20% amortecia 16, antes 60. O aplicado e min(300 x forca, 80): o de antes ate ~27%, e o
                // estavel acima (300 cheio vibrava).)
                const float turnDamping = a > 1.0e-3f ? std::min(AssistTurnDampingLow, AssistTurnDamping / a) : AssistTurnDampingLow;
                Vec3 torque = rotationLog((assistOrientation * pelvis.orientation.conjugate()).normalized()) * AssistTurnGain
                    - (pelvis.angularVelocity - assistSpin) * turnDamping;
                const float f = force.length(), q = torque.length();
                if (settings.linearHand) {
                    // (04/11 04h35, ele: "a forca auxiliar linear de verdade, onde 0% e zero de verdade": a 100% a mao
                    // sustenta no maximo o peso do corpo, empurra no maximo metade dele e segura a pelve no maximo com o
                    // torque de tombar o corpo (peso x 1 m); a x%, x% de cada limite. Antes, 45% seguravam 82% do peso no
                    // tropeco - o vetor era cortado em 2000 N e 1500 N.m antes da escala.)
                    const float weight = m * 9.81f;
                    force.z = std::clamp(force.z, -weight, weight);
                    // (04/11 07h00: nas costas a mao nao puxa - levado a 4,4 m/s mais rapido que o passo, o pe ficava para
                    // tras arrastando e o erro das pernas subia com a forca (2,05 a 100%, 0,61 a 0%). Ela segura a altura e
                    // o equilibrio; o avanco e das pernas.)
                    const bool backward = !w.getUpGuided && w.maneuver == 0
                        && ((w.heading >= 3 && w.heading <= 5)
                            || (!settings.lateralHandPush && !w.runBand && (w.heading == 2 || w.heading == 6)));
                    const float side = std::hypot(force.x, force.y), sideLimit = backward ? 0.0f : 0.5f * weight;
                    if (side > sideLimit) {
                        force.x *= sideLimit / side;
                        force.y *= sideLimit / side;
                    }
                    if (q > weight * LinearHandLever) torque = torque * (weight * LinearHandLever / q);
                } else {
                    if (f > AssistMaximumForce) force = force * (AssistMaximumForce / f);
                    if (q > AssistMaximumTorque) torque = torque * (AssistMaximumTorque / q);
                }
                w.scene->applyRagdollControlRootForce(w.handle, force * a, torque * a);
                w.teacherAssist = std::max(w.teacherAssist, a * force.length());
                w.handUp += a * force.z / static_cast<float>(ticks);
                w.handSide += a * std::hypot(force.x, force.y) / static_cast<float>(ticks);
                w.handTurn += a * torque.length() / static_cast<float>(ticks);
                if (handCount > 0) {
                    // (04/11 05h45, ele: "no trote e no geral, dificuldade para manter a coluna e a cabeca mais estaveis...
                    // de um jeito de dar uma ajuda para ele melhorar e nao parecer tao mole": a mesma mao, no peito alto, na
                    // cabeca e nos bracos - o torque para a orientacao do clipe, amortecendo o balanco que o clipe nao tem;
                    // linear (a x%, x% do teto) e com a mesma forca do exercicio, que desce pelo dominio.)
                    for (std::size_t u = 0; u < handCount; ++u) {
                        if (assistLinks[u] >= s.links.size()) continue;
                        const PhysicsBodyState3D& link = s.links[assistLinks[u]];
                        Vec3 twist = rotationLog((w.handTarget[u] * link.orientation.conjugate()).normalized()) * assistGain[u]
                            - (link.angularVelocity - w.handSpin[u]) * assistDamping[u];
                        const float size = twist.length();
                        if (size > assistCap[u]) twist = twist * (assistCap[u] / size);
                        w.scene->applyRagdollControlLinkForce(w.handle, assistLinks[u], {}, twist * a);
                    }
                }
            }
            if (w.pushTicks > 0) {
                // (Repartido como o do B19: pelve, peito e peito alto.)
                for (std::size_t k = 0; k < w.pushLinks.size(); ++k)
                    if (w.pushShares[k] > 0.0f && w.pushLinks[k] < w.body->links.size())
                        w.scene->applyRagdollLinkForce(w.handle, w.pushLinks[k], w.pushForce * w.pushShares[k], {});
                --w.pushTicks;
            }
            // (17/11, settings.projectileRate) O projetil no elo dele.
            if (w.projectileTicks > 0) {
                if (w.projectileLink < w.body->links.size())
                    w.scene->applyRagdollLinkForce(w.handle, w.projectileLink, w.projectileForce, {});
                --w.projectileTicks;
            }
            if (w.fireHoldScale > 0.0f) {
                // (12/11 04h40, a fase de fogo do levantar: a forca continua, repartida no tronco como o empurrao.)
                const Vec3 hold = fireHoldNow(w);
                for (std::size_t k = 0; k < FireHoldLinks.size(); ++k)
                    if (FireHoldLinks[k] < w.body->links.size())
                        w.scene->applyRagdollLinkForce(w.handle, FireHoldLinks[k], hold * FireHoldShares[k], {});
            }
            if (fireGrabActive(w))
                w.scene->applyRagdollLinkForce(w.handle, static_cast<std::uint32_t>(w.fireGrabLink), w.fireGrabForce, {});
            w.scene->simulate(TickSeconds);
        }
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // Uma decisao com a professora (03/11 18h27): em cada tick ela le o corpo e encena os alvos dela. Com ela no volante
    // a fisica anda com eles (o movimento EXATO dela); senao eles sao sobrescritos pelos da aluna (w.drives) e ela fica em
    // sombra. O rotulo junta, tick a tick, o alvo que no motor da aluna (rigidez e amortecimento do perfil, sem gravidade,
    // alvo segurado, sem velocidade-alvo) daria o torque dela no eixo: no volante, o torque medido do passo (o motor
    // implicito mais a antecipacao e a gravidade) contra o estado do fim dele; em sombra, o torque que o drive dela daria
    // no estado de agora, pela mola explicita. Tambem o alvo dela cru. As medias dos ticks validos.
    void settleTaught(World& w, bool drives) {
        const std::size_t n = w.drives.size();
        w.label.assign(n, 0.0f);
        w.labelRaw.assign(n, 0.0f);
        int valid = 0;
        // (Com a gravidade no musculo da aluna, o torque dela ja vem do backend: o rotulo tira essa parte.)
        const auto computeGravity = [&]() {
            w.gravityPoses.resize(w.body->links.size());
            for (std::size_t i = 0; i < w.body->links.size(); ++i)
                w.gravityPoses[i] = { w.body->links[i].position, w.body->links[i].orientation };
            w.gravity.compute(w.gravityPoses, { 0.0f, 0.0f, -9.81f });
        };
        const auto limitOf = [&](std::size_t k) -> const RagdollAxisDefinition3D& {
            return profile->links[w.drives[k].linkIndex].inboundJoint.axes[static_cast<int>(w.drives[k].axis)];
        };
        // O erro que o motor da aluna precisa ver no eixo para dar o torque (sem a gravidade dela, que o backend soma).
        const auto studentError = [&](std::size_t k, float qd, float torque) {
            const RagdollDriveTarget3D& d = w.drives[k];
            const int a = static_cast<int>(d.axis);
            const RagdollAxisDefinition3D& limit = limitOf(k);
            const float kp = limit.stiffness * std::clamp(d.stiffnessScale, 0.0f, 2.0f) * 5.0f;
            const float kd = limit.damping * std::clamp(d.dampingScale, 0.0f, 2.5f) * 2.0f;
            if (w.driveGravity)
                torque -= w.gravity.jointTorques()[d.linkIndex][a] * std::clamp(d.gravityCompensationScale, 0.0f, 1.0f);
            return kp > 1.0e-6f ? (torque + kd * qd) / kp : 0.0f;
        };
        // (A revisao de 03/11: o motor do Jolt mede o erro compondo quaternios - diff = q^-1 * alvo, no quadro do filho -,
        // nao somando os vetores: no ombro, a ~1,3 rad, a soma errava ~0,1 rad. Por junta: alvo = log(exp(q) * exp(e)).)
        std::vector<Vec3>& linkError = w.linkError;
        const auto compose = [&](std::span<const float> error, std::span<float> out) {
            linkError.assign(w.body->links.size(), Vec3 {});
            for (std::size_t k = 0; k < n; ++k) axisRef(linkError[w.drives[k].linkIndex], static_cast<int>(w.drives[k].axis)) = error[k];
            for (std::size_t k = 0; k < n; ++k) {
                const std::uint32_t link = w.drives[k].linkIndex;
                const auto& q = w.body->joints[link].positionRadians;
                const Vec3 target = rotationLog((rotationExp({ q[0], q[1], q[2] }) * rotationExp(linkError[link])).normalized());
                const RagdollAxisDefinition3D& limit = limitOf(k);
                out[k] = std::clamp(axisValue(target, static_cast<int>(w.drives[k].axis)), limit.minimumRadians,
                    limit.maximumRadians);
            }
        };
        std::vector<float>& error = w.axisError;
        std::vector<float>& theta = w.axisTarget;
        error.assign(n, 0.0f);
        theta.assign(n, 0.0f);
        if (w.teacherStats.size() != 5 * n) w.teacherStats.assign(5 * n, 0.0);
        const auto teacherTarget = [&](const RagdollDriveTarget3D& d) -> const RagdollDriveTarget3D* {
            for (const RagdollDriveTarget3D& t : w.teacherDrives)
                if (t.linkIndex == d.linkIndex && t.axis == d.axis) return &t;
            return nullptr;
        };
        for (int tick = 0; tick < TicksPerDecision; ++tick) {
            LocomotionPartner3D::Tick result;
            if (w.teacherSession) result = teacher->tick(*w.teacherSession, *w.scene, w.handle);
            w.teacherAssist = std::max(w.teacherAssist, result.assistNewtons);
            bool gravity = false;
            if (result.valid) w.scene->stagedRagdollDriveTargets(w.handle, w.teacherDrives, gravity);
            else w.teacherSession.reset();
            const bool driving = drives && result.valid;
            if (result.valid && stillClipTaught(w)) {
                // (A professora hibrida do parado: o IDLE ALERTA deste tick - parado no alvo -, nas pernas com as
                // antecipacoes de contato dela, o equilibrio dela. No volante, assim encenado.)
                references[w.trackClip]->frame(trackTime(w, static_cast<float>(tick + 1) * TickSeconds), {}, 0.0f, w.frame);
                w.clipAction.assign(n, 0.0f);
                contract->actionOf(w.frame.coordinates, w.clipAction);
                for (std::size_t k = 0; k < n; ++k) {
                    const bool upper = upperBody(w.drives[k].linkIndex);
                    if (!upper && settings.teacherStillClip < 2) continue;
                    for (RagdollDriveTarget3D& t : w.teacherDrives)
                        if (t.linkIndex == w.drives[k].linkIndex && t.axis == w.drives[k].axis) {
                            t.positionRadians = w.clipAction[k];
                            t.velocityRadiansPerSecond = 0.0f;
                            if (upper) t.feedforwardTorqueNewtonMeters = 0.0f;
                        }
                }
                if (driving) w.scene->setRagdollActiveDriveTargets(w.handle, w.teacherDrives, gravity);
            }
            if (result.valid) {
                ++w.teacherTicks;
                for (std::size_t k = 0; k < n; ++k)
                    if (const RagdollDriveTarget3D* t = teacherTarget(w.drives[k])) {
                        w.teacherStats[5 * k] += t->stiffnessScale;
                        w.teacherStats[5 * k + 1] += t->dampingScale;
                        w.teacherStats[5 * k + 2] += t->maximumTorqueScale;
                        w.teacherStats[5 * k + 3] += gravity ? t->gravityCompensationScale : 0.0f;
                        w.teacherStats[5 * k + 4] += std::abs(t->feedforwardTorqueNewtonMeters);
                    }
            }
            if (result.valid && !driving) {
                // (Em sombra: o torque do drive dela no estado de agora - o erro dela composto por junta, a mola explicita,
                // a antecipacao e a gravidade no teto do motor, como o backend.)
                if (gravity || w.driveGravity) computeGravity();
                linkError.assign(w.body->links.size(), Vec3 {});
                std::vector<Vec3>& teacherPose = w.linkTarget;
                teacherPose.assign(w.body->links.size(), Vec3 {});
                for (std::size_t i = 0; i < w.body->links.size(); ++i) {
                    const auto& q = w.body->joints[i].positionRadians;
                    teacherPose[i] = { q[0], q[1], q[2] };
                }
                for (std::size_t k = 0; k < n; ++k)
                    if (const RagdollDriveTarget3D* t = teacherTarget(w.drives[k])) {
                        const RagdollAxisDefinition3D& limit = limitOf(k);
                        axisRef(teacherPose[w.drives[k].linkIndex], static_cast<int>(w.drives[k].axis))
                            = std::clamp(t->positionRadians, limit.minimumRadians, limit.maximumRadians);
                    }
                for (std::size_t i = 0; i < w.body->links.size(); ++i) {
                    const auto& q = w.body->joints[i].positionRadians;
                    linkError[i] = rotationLog((rotationExp({ q[0], q[1], q[2] }).conjugate() * rotationExp(teacherPose[i])).normalized());
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const RagdollDriveTarget3D& d = w.drives[k];
                    const int a = static_cast<int>(d.axis);
                    const float qd = w.body->joints[d.linkIndex].velocityRadiansPerSecond[a];
                    const RagdollDriveTarget3D* t = teacherTarget(d);
                    float torque = 0.0f;
                    if (t != nullptr) {
                        const RagdollAxisDefinition3D& limit = limitOf(k);
                        const float kp = limit.stiffness * std::clamp(t->stiffnessScale, 0.0f, 2.0f) * 5.0f;
                        const float kd = limit.damping * std::clamp(t->dampingScale, 0.0f, 2.5f) * 2.0f;
                        const float maximum = limit.maximumTorque * std::clamp(t->maximumTorqueScale, 0.0f, 3.0f);
                        const float g = gravity ? w.gravity.jointTorques()[d.linkIndex][a]
                                * std::clamp(t->gravityCompensationScale, 0.0f, 1.0f) : 0.0f;
                        const float ff = std::clamp(t->feedforwardTorqueNewtonMeters + g, -maximum, maximum);
                        const float motor = std::clamp(kp * axisValue(linkError[d.linkIndex], a)
                                + kd * (t->velocityRadiansPerSecond - qd), -maximum - ff, maximum - ff);
                        torque = motor + ff;
                    }
                    // (Os oraculos 3 e 4: o musculo da professora deste tick - as escalas dela - nos motores da aluna; o
                    // quanto o ganho variavel dela pesa.)
                    if (t != nullptr && (teacherOracle == 3 || (teacherOracle == 4 && tick == 0))) {
                        w.drives[k].stiffnessScale = t->stiffnessScale;
                        w.drives[k].dampingScale = t->dampingScale;
                        w.drives[k].maximumTorqueScale = t->maximumTorqueScale;
                        w.drives[k].gravityCompensationScale = t->gravityCompensationScale;
                    }
                    error[k] = studentError(k, qd, torque);
                }
                compose(error, theta);
                for (std::size_t k = 0; k < n; ++k) {
                    w.label[k] += theta[k];
                    // (A aluna-oraculo: o rotulo nos motores dela - no tick 0 e segurado, como a rede a 30 Hz, ou a cada
                    // tick. A melhor aluna possivel: se ela nao anda como a professora, nenhuma rede treinada anda.)
                    if (teacherOracle == 2 || teacherOracle == 3 || ((teacherOracle == 1 || teacherOracle == 4) && tick == 0))
                        w.drives[k].positionRadians = theta[k];
                }
            }
            if (!driving) w.scene->setRagdollActiveDriveTargets(w.handle, w.drives, w.driveGravity);
            if (result.valid)
                for (std::size_t k = 0; k < n; ++k) {
                    const RagdollDriveTarget3D* t = teacherTarget(w.drives[k]);
                    w.labelRaw[k] += t != nullptr ? t->positionRadians : 0.0f;
                }
            if (w.pushTicks > 0) {
                for (std::size_t k = 0; k < w.pushLinks.size(); ++k)
                    if (w.pushShares[k] > 0.0f && w.pushLinks[k] < w.body->links.size())
                        w.scene->applyRagdollLinkForce(w.handle, w.pushLinks[k], w.pushForce * w.pushShares[k], {});
                --w.pushTicks;
            }
            // (17/11, settings.projectileRate) O projetil no elo dele.
            if (w.projectileTicks > 0) {
                if (w.projectileLink < w.body->links.size())
                    w.scene->applyRagdollLinkForce(w.handle, w.projectileLink, w.projectileForce, {});
                --w.projectileTicks;
            }
            if (w.fireHoldScale > 0.0f) {
                // (12/11 04h40, a fase de fogo do levantar: a forca continua, repartida no tronco como o empurrao.)
                const Vec3 hold = fireHoldNow(w);
                for (std::size_t k = 0; k < FireHoldLinks.size(); ++k)
                    if (FireHoldLinks[k] < w.body->links.size())
                        w.scene->applyRagdollLinkForce(w.handle, FireHoldLinks[k], hold * FireHoldShares[k], {});
            }
            if (fireGrabActive(w))
                w.scene->applyRagdollLinkForce(w.handle, static_cast<std::uint32_t>(w.fireGrabLink), w.fireGrabForce, {});
            w.scene->simulate(TickSeconds);
            w.body = w.scene->ragdollStateView(w.handle);
            if (driving && w.driveGravity) computeGravity();
            if (driving) {
                // (No volante: o torque que os motores dela deram neste passo, contra o estado do fim dele - o motor
                // implicito.)
                for (std::size_t k = 0; k < n; ++k) {
                    const int a = static_cast<int>(w.drives[k].axis);
                    const RagdollJointState3D& joint = w.body->joints[w.drives[k].linkIndex];
                    error[k] = studentError(k, joint.velocityRadiansPerSecond[a],
                        joint.motorTorqueNewtonMeters[a] + joint.feedforwardTorqueNewtonMeters[a]);
                }
                compose(error, theta);
                for (std::size_t k = 0; k < n; ++k) w.label[k] += theta[k];
            }
            if (result.valid) ++valid;
        }
        w.labelValid = valid == TicksPerDecision;
        w.labelDrove = drives && valid > 0;
        if (valid > 0)
            for (std::size_t k = 0; k < n; ++k) {
                w.label[k] /= static_cast<float>(valid);
                w.labelRaw[k] /= static_cast<float>(valid);
            }
        if (w.labelDrove) ++w.teacherDecisions;
        if (w.labelValid) ++w.labeled;
    }

    // A queda: uma pose de um clipe (qualquer um do estilo) num instante sorteado, o corpo inteiro girado num sentido
    // sorteado em torno do centro de massa, erguido, com giro e deriva sorteados, e solto com os musculos fracos
    // segurando a pose com ruido ate assentar. Se nao caiu (a cabeca acima de 1 m), outra.
    void beginFall(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        std::normal_distribution<float> gauss(0.0f, 1.0f);
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto& reference = *references[std::uniform_int_distribution<std::size_t>(0, taskClipCount - 1)(w.rng)];
            reference.frame(u(w.rng) * reference.durationSeconds(), {}, 0.0f, w.frame);
            const Quaternion turn = uniformRotation(w.rng);
            const Vec3 center = w.frame.centerOfMass;
            float lowest = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < w.frame.links.size(); ++i) {
                PhysicsBodyState3D& s = w.frame.links[i];
                s.position = center + turn.rotate(s.position - center);
                s.orientation = (turn * s.orientation).normalized();
                const Vec3 com = rlLinkCenterOfMass3D(profile->links[i], s);
                lowest = std::min(lowest, com.z - terrainHeight(w, com));
            }
            const Vec3 lifted = center + Vec3 { 0.0f, 0.0f, 0.35f + 0.5f * u(w.rng) - lowest + w.underLift };
            const Vec3 spin = Vec3 { gauss(w.rng), gauss(w.rng), gauss(w.rng) } * 1.5f;
            // (Com corpos embaixo, a deriva menor: ele cai em cima deles - 02/11 23h15.)
            const float driftScale = w.extras.empty() ? 1.0f : 0.3f;
            const Vec3 drift = Vec3 { 2.0f * u(w.rng) - 1.0f, 2.0f * u(w.rng) - 1.0f, 0.0f } * driftScale;
            for (PhysicsBodyState3D& s : w.frame.links) {
                s.position = s.position + (lifted - center);
                s.linearVelocity = drift + cross(spin, s.position - lifted);
                s.angularVelocity = spin;
            }
            w.scene->resetRagdollState(w.handle, w.frame.links);
            contract->actionOf(w.frame.coordinates, w.pose);
            for (float& value : w.pose) value += FallPoseNoise * (2.0f * u(w.rng) - 1.0f);
            contract->driveTargets(w.pose, w.drives);
            for (RagdollDriveTarget3D& drive : w.drives) drive.stiffnessScale *= FallStiffness;
            if (settings.protectiveFall) {
                // (A queda programada: no laboratorio ela cai na tela - a primeira decisao a comeca -; no treino o corpo
                // assenta por ela aqui, e o levantar parte das poses de quem caiu de verdade, como no jogo.)
                w.body = w.scene->ragdollStateView(w.handle);
                if (settings.protectiveFallStepwise) {
                    w.fallPending = true;
                    return;
                }
                w.fall->begin(static_cast<std::uint32_t>(w.rng()));
                for (int k = 0; k < static_cast<int>(4.0f / TickSeconds) && !w.fall->settled(); ++k) {
                    w.fall->step(*w.body, 0.0f, TickSeconds, w.drives);
                    settle(w, 1);
                }
            } else {
                settle(w, static_cast<int>((0.9f + 0.6f * u(w.rng)) / TickSeconds));
            }
            if (posture->measure(w.body->links, floorUnder(w)).headHeight < 1.0f) return;
        }
    }

    // O evitar a queda com a parceira (01/11, ele: "o evitar queda precisa saber operar em conjunto com a outra rede nos
    // momentos de queda iminente"): de pe, ela leva o empurrao forte (o do evitar, de 0,3 a 1,0 s depois do comeco) e a
    // rede do levantar assume na queda iminente dela, com o resto do empurrao (pushLeft, s; a forca em pushForce). Se ela
    // aguenta sozinha ate 1,5 s depois do empurrao, outro (ate 3 vezes); sem a queda iminente, false.
    bool beginPartnerAvoid(World& w, float& pushLeft, Vec3& pushForce) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (attempt > 0) beginFromClip(w, true);
            LocomotionPartner3D::Push push;
            push.startSeconds = 0.3f + 0.7f * u(w.rng);
            const float angle = 2.0f * Pi * u(w.rng);
            float newtons = 0.0f;
            if (u(w.rng) < settings.avoidHoldFraction) {
                newtons = settings.avoidHoldMinimumNewtons
                    + (settings.avoidHoldMaximumNewtons - settings.avoidHoldMinimumNewtons) * u(w.rng);
                push.seconds = settings.avoidHoldMinimumSeconds
                    + (settings.avoidHoldMaximumSeconds - settings.avoidHoldMinimumSeconds) * u(w.rng);
            } else {
                newtons = settings.avoidMinimumNewtons + (settings.avoidMaximumNewtons - settings.avoidMinimumNewtons) * u(w.rng);
                push.seconds = 0.1f + 0.1f * u(w.rng);
            }
            push.force = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
            const LocomotionPartner3D::Result result =
                partner->drive(*w.scene, w.handle, {}, push.startSeconds + push.seconds + 1.5f, &push);
            w.body = w.scene->ragdollStateView(w.handle);
            if (result.fell) {
                pushLeft = result.pushLeft;
                pushForce = push.force;
                return true;
            }
        }
        return false;
    }

    // A queda da locomocao (01/11, a interligacao das redes): o corpo no instante em que a rede da locomocao caiu, com as
    // velocidades, trazido para a origem e, no treino, girado num rumo sorteado (na avaliacao, a do mundo, sem giro).
    void beginLocomotionFall(World& w) {
        const std::size_t pick = settings.evaluation
            ? w.index % fallStarts.size()
            : std::uniform_int_distribution<std::size_t>(0, fallStarts.size() - 1)(w.rng);
        const std::vector<PhysicsBodyState3D>& source = fallStarts[pick];
        const Vec3 root { source[rootLink].position.x, source[rootLink].position.y, 0.0f };
        const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
            settings.evaluation ? 0.0f : std::uniform_real_distribution<float>(-Pi, Pi)(w.rng));
        w.frame.links = source;
        for (PhysicsBodyState3D& s : w.frame.links) {
            s.position = turn.rotate(s.position - root);
            s.orientation = (turn * s.orientation).normalized();
            s.linearVelocity = turn.rotate(s.linearVelocity);
            s.angularVelocity = turn.rotate(s.angularVelocity);
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // A arrancada e a corrida (31/10 18h25: so a arrancada agachada, curta, nao bastou - em 300 atualizacoes ele saia do
    // clipe e freava ate parar): um dos clipes da arrancada e da corrida, num instante sorteado (as arrancadas na
    // primeira metade), num rumo sorteado (o alvo vem perto dele), com a velocidade do clipe (a dos no lugar somada).
    void beginSprintStart(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const std::size_t index = runStartIndices[std::uniform_int_distribution<std::size_t>(0, runStartIndices.size() - 1)(w.rng)];
        const auto& reference = *references[index];
        const Travel& run = travel[index];
        const bool transition = !clips[index]->loops;
        w.sprintYaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        const float t = u(w.rng) * (transition ? 0.5f : 1.0f) * reference.durationSeconds();
        reference.frame(t, {}, w.sprintYaw, w.frame);
        float speed = run.speed;
        if (run.rampDisplacement > 0.0f && reference.durationSeconds() > 0.0f)
            speed = 2.0f * run.rampDisplacement * t / (reference.durationSeconds() * reference.durationSeconds());
        if (speed > 0.0f) {
            const float heading = w.sprintYaw + run.direction;
            const Vec3 v { std::cos(heading) * speed, std::sin(heading) * speed, 0.0f };
            for (PhysicsBodyState3D& s : w.frame.links) s.linearVelocity = s.linearVelocity + v;
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // Um instante sorteado de um clipe de levantar (o RSI do DeepMimic) ou, de pe, do parado, num rumo sorteado. (01/11,
    // a rl_final: com as quedas com protecao carregadas, metade dos comecos sao delas, nos primeiros 70% - a queda.)
    // Deitado no chao: o quadro 0 de um clipe de levantar (pick: qual do pool; -1 sorteado), num rumo sorteado, parado, com
    // a parte mais baixa no minimo a LyingClearance do chao.
    void beginLying(World& w, const std::vector<std::size_t>& pool, int pick) {
        constexpr float LyingClearance = 0.04f;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const std::size_t index = pick >= 0 && static_cast<std::size_t>(pick) < pool.size()
            ? pool[static_cast<std::size_t>(pick)] : pool[std::uniform_int_distribution<std::size_t>(0, pool.size() - 1)(w.rng)];
        const float yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        w.protectiveStart = false;
        w.lastClip = std::numeric_limits<std::size_t>::max();
        w.lastClipTime = 0.0f;
        w.lastClipYaw = yaw;
        w.lyingClip = index;
        references[index]->frame(0.0f, {}, yaw, w.frame);
        float lowest = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < w.frame.links.size(); ++i) {
            const Vec3 com = rlLinkCenterOfMass3D(profile->links[i], w.frame.links[i]);
            lowest = std::min(lowest, com.z - terrainHeight(w, com));
        }
        const float lift = std::max(0.0f, LyingClearance - lowest);
        for (PhysicsBodyState3D& s : w.frame.links) {
            s.position.z += lift;
            s.linearVelocity = {};
            s.angularVelocity = {};
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // (11/11 14h, settings.clipGetUpFraction) O comeco do levantar pelo clipe: um dos quatro (na avaliacao, o 11 a 14), no rumo
    // sorteado; em clipGetUpMidStartFraction dos episodios num instante sorteado do clipe, com a pose e as velocidades dele (o RSI do
    // DeepMimic: todas as fases aprendem juntas), senao deitado no quadro 0 com a coluna e as pernas variadas ate
    // clipGetUpStartNoise rad. O clipe seguido no tempo dele dali, ancorado no mundo.
    static constexpr float ClipRunTargetMeters = 60.0f, ClipHandoffHead = 1.2f;
    void beginClipGetUp(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const std::uint32_t count = static_cast<std::uint32_t>(clipGetUpClips.size());
        // (12/11 19h10) O levantar e ir: no treino na fatia; na avaliacao com a fatia cheia, todos, com o exercicio sorteado.
        const bool goEval = settings.evaluation && settings.getUpGoShare >= 1.0f;
        w.getUpGo = settings.getUpGoShare > 0.0f && (settings.evaluation ? goEval : u(w.rng) < settings.getUpGoShare);
        const std::uint32_t exercise = goEval ? std::uniform_int_distribution<std::uint32_t>(0u, count - 1u)(w.rng)
            : settings.evaluation
            ? std::min<std::uint32_t>(count - 1u, settings.evaluationKind >= 11u ? settings.evaluationKind - 11u : 0u)
            : settings.clipGetUpWeights.size() == count
            ? static_cast<std::uint32_t>(std::discrete_distribution<int>(settings.clipGetUpWeights.begin(),
                  settings.clipGetUpWeights.end())(w.rng))
            : std::uniform_int_distribution<std::uint32_t>(0u, count - 1u)(w.rng);
        const std::size_t clip = clipGetUpClips[exercise];
        const TrackingClipReference3D& reference = *references[clip];
        const bool mid = !settings.evaluation && !w.getUpGo && u(w.rng) < settings.clipGetUpMidStartFraction;
        // (18h22: o vies dos comecos no meio por exercicio - setClipGetUpStartBias: o expoente da fracao sorteada; 2 no rolando, metade
        // dos comecos no meio antes de ~0,8 s, onde ele rola.)
        const float bias = exercise < clipGetUpStartBias.size() ? std::max(0.1f, clipGetUpStartBias[exercise]) : 1.0f;
        const float t0 = mid ? std::pow(u(w.rng), bias) * std::max(0.0f, reference.durationSeconds() - 0.3f) : 0.0f;
        const float yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        const float spread = mid || settings.evaluation ? 0.0f : settings.clipGetUpStartNoise;
        const auto noise = [&] { return spread * (2.0f * u(w.rng) - 1.0f); };
        w.protectiveStart = false;
        w.lastClip = std::numeric_limits<std::size_t>::max();
        w.lastClipTime = 0.0f;
        w.lastClipYaw = yaw;
        w.clipGetUp = true;
        w.clipExercise = exercise;
        w.clipPoseExercise = exercise;
        w.clipReference = clip;
        w.clipStart = t0;
        w.imitationReference = clip;
        w.imitationStart = t0;
        w.imitationLowSeconds = 0.0f;
        w.imitationAnchored = false;
        w.clipYaw = yaw;
        w.clipGround = { 0.0f, 0.0f, terrainHeight(w, {}) };
        // (20h20) O rolando comeca livre ate o clipe estar de brucos (TrackingClipGetUp3D::FreeRollProne).
        if (exercise < clipGetUpVariants.size() && clipGetUpVariants[exercise] > 0.5f) {
            w.clipFreeUntil = freeRollUntil(clip);
            w.clipFree = w.clipFreeUntil > 0.0f && t0 < w.clipFreeUntil;
        }
        const float twist = noise(), pitch = noise(), legs = noise();
        reference.frame(t0, w.clipGround, yaw, w.frame, twist, pitch, legs);
        // (14h30, settings.getUpPoseVariety) A pose girada em torno do comprido do corpo (pelve -> cabeca no chao), de 60 a 180 graus.
        const bool varied = !mid && !settings.evaluation && settings.getUpPoseVariety > 0.0f && u(w.rng) < settings.getUpPoseVariety;
        if (varied) {
            const Vec3 r0 = w.frame.links[rootLink].position;
            Vec3 axis = w.frame.links[contract->headLink()].position - r0;
            axis.z = 0.0f;
            axis = axis.length() > 1.0e-3f ? axis.normalized() : Vec3 { 1.0f, 0.0f, 0.0f };
            const float angle = (u(w.rng) < 0.5f ? -1.0f : 1.0f) * (60.0f + 120.0f * u(w.rng)) * Pi / 180.0f;
            const Quaternion q = Quaternion::fromAxisAngle(axis, angle);
            for (PhysicsBodyState3D& s : w.frame.links) {
                s.position = r0 + q.rotate(s.position - r0);
                s.orientation = (q * s.orientation).normalized();
                s.linearVelocity = {};
                s.angularVelocity = {};
            }
        }
        // (Os clipes corrigidos ja tem o apoio no chao: so nada abaixo dele - o centro de massa de cada parte a ClipStartClearance.)
        constexpr float ClipStartClearance = 0.03f;
        float lowest = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < w.frame.links.size(); ++i) {
            const Vec3 com = rlLinkCenterOfMass3D(profile->links[i], w.frame.links[i]);
            lowest = std::min(lowest, com.z - terrainHeight(w, com));
        }
        const float lift = std::max(0.0f, ClipStartClearance - lowest);
        for (PhysicsBodyState3D& s : w.frame.links) {
            s.position.z += lift;
            if (!mid) {
                s.linearVelocity = {};
                s.angularVelocity = {};
            }
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
        if (varied) {
            adoptClipForPose(w);
            w.attemptPhase = clipTime(w);
            w.fireExtras |= 128;
            // (15/11 13h30, a revisao) A imitacao no ritmo segue o clipe e a fase adotados (antes: o clipe do sorteio no quadro 0,
            // com o rumo do adotado - errado em dezenas de graus).
            w.imitationReference = w.clipReference;
            w.imitationStart = clipTime(w);
            w.imitationAnchored = false;
            w.imitationLowSeconds = 0.0f;
        }
        w.trackClip = clip;
        w.trackStart = t0;
        w.trackFrom = 0.0f;
        w.trackScale = 1.0f;
        w.trackYaw = yaw;
        w.rampFrom = -1.0f;
        if (w.getUpGo) {
            // (O ponto: para onde o clipe termina virado - de pe, +-60 graus - ou correndo - +-20 -, a getUpGoMin-Max m.)
            const std::size_t e = w.clipExercise;
            const bool run = clipRunExercise(w.clipExercise);
            const float base = w.clipYaw + (run ? (e < clipGetUpDirection.size() ? clipGetUpDirection[e] : 0.0f)
                                                : (e < clipGetUpFacing.size() ? clipGetUpFacing[e] : 0.0f));
            // (15/11, settings.getUpViewReward) De pe, o ponto em qualquer rumo e a vista nele desde o comeco (a camera no objetivo).
            const bool view = settings.getUpViewReward > 0.0f && !run;
            const float angle = base + (2.0f * u(w.rng) - 1.0f) * (run ? GoRunSpread : view ? Pi : GoStandSpread);
            w.goDistance = settings.getUpGoMin + (std::max(settings.getUpGoMin, settings.getUpGoMax) - settings.getUpGoMin) * u(w.rng);
            const Vec3 root = w.body->links[rootLink].position;
            w.goTarget = { root.x + std::cos(angle) * w.goDistance, root.y + std::sin(angle) * w.goDistance, 0.0f };
            if (view) w.faceYaw = wrapAngle(angle);
        }
        beginGetUpFire(w);
    }
    static constexpr float GoStandSpread = Pi / 3.0f, GoRunSpread = Pi / 9.0f, GoArriveMeters = 1.0f, GoTurnRate = 1.5f;
    // (12/11 19h10) O prazo do levantar e ir depois da troca (s): 3 s mais o percurso a 2,5 m/s.
    float goBudget(const World& w) const { return 3.0f + w.goDistance / 2.5f; }
    // (12/11 19h10, o levantar e ir) Depois da troca, a cada decisao: chegou (a GoArriveMeters do ponto) ou o rumo e a vista virando para
    // o ponto a GoTurnRate rad/s; o alvo longe na frente, como na troca do correndo.
    void steerGo(World& w) {
        if (!w.getUpGo || !w.clipGetUp || w.clipHandedAt < 0.0f || w.goArrivedAt >= 0.0f || w.body->links.empty()) return;
        const Vec3 root = w.body->links[rootLink].position;
        const float dx = w.goTarget.x - root.x, dy = w.goTarget.y - root.y;
        if (std::hypot(dx, dy) <= GoArriveMeters) {
            w.goArrivedAt = w.time;
            return;
        }
        const float dt = TicksPerDecision * TickSeconds;
        const float step = std::clamp(wrapAngle(std::atan2(dy, dx) - w.faceYaw), -GoTurnRate * dt, GoTurnRate * dt);
        w.faceYaw = wrapAngle(w.faceYaw + step);
        w.moveYaw = w.faceYaw;
        w.trackYaw = wrapAngle(w.faceYaw - w.trackTwist);
        w.target = { root.x + std::cos(w.faceYaw) * ClipRunTargetMeters, root.y + std::sin(w.faceYaw) * ClipRunTargetMeters, 0.0f };
    }

    // (12/11 04h40, ele: a fase de fogo do levantar - vault 04_locomocao/plano_fogo_do_levantar_12_11.md: "uma forca constante empurrando
    // ele, 8 direcoes, apenas uma por mundo, variando... do inicio ao fim" e "no comeco, meio ou quase fim do ato de se levantar... um
    // empurrao seco bem forte para faze-lo cair") O cenario do episodio: pelas fatias (limpo, pancada, continuo, os dois) ou, na prova,
    // pelo mundo (de 16: 4 limpos, 6 continuos - frente, tras, esquerda, direita e duas diagonais -, 6 pancadas - comeco, meio e quase
    // fim, de frente e de tras), no topo do nivel. O rumo e um dos 8 a partir da frente do corpo no fim do clipe; o tamanho, a fracao
    // sorteada (0,4 a 1) do nivel do exercicio: o continuo em fracao do peso, a pancada em m/s ao centro de massa em 0,1-0,2 s; a
    // pancada no instante do clipe do momento sorteado (do tempo dele, a 1x), ou logo, se o episodio comecou depois.
    void beginGetUpFire(World& w) {
        // (12/11 19h10) O levantar e ir comeca limpo (a troca primeiro).
        if (w.getUpGo) return;
        const std::size_t e = w.clipExercise;
        if (getUpFireLevels.size() < 4 * (e + 1)) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        int kind = 0, timing = 0, direction = 0;
        float fraction = 1.0f;
        if (getUpFireExam) {
            static constexpr int ExamDirections[6] { 0, 4, 2, 6, 1, 5 };
            const std::size_t k = w.index % 16u;
            // (05h40, ele: o continuo "em TODO o exercicio... do comeco ao fim": as pancadas da prova vem com ele.)
            if (k >= 4u && k < 10u) kind = 2, direction = ExamDirections[k - 4u];
            else if (k >= 10u) kind = 3, timing = 1 + static_cast<int>((k - 10u) / 2u), direction = (k % 2u) == 0u ? 0 : 4;
        } else {
            float draw = u(w.rng) * (getUpFireShares[0] + getUpFireShares[1] + getUpFireShares[2] + getUpFireShares[3]);
            for (int c = 0; c < 4; ++c) {
                if (draw < getUpFireShares[static_cast<std::size_t>(c)]) { kind = c; break; }
                draw -= getUpFireShares[static_cast<std::size_t>(c)];
                kind = 3;
            }
            direction = std::uniform_int_distribution<int>(0, 7)(w.rng);
            timing = std::uniform_int_distribution<int>(1, 3)(w.rng);
            // (13/11, settings.fireEarlyShare) A maioria cedo, pelo tempo; o resto pela altura, meio a meio.
            if (settings.fireEarlyShare > 0.0f) timing = u(w.rng) < settings.fireEarlyShare ? 1 : u(w.rng) < 0.5f ? 2 : 3;
            // (06h20, ele: "as forcas e desafios precisam ser mais fortes, para tentar IMPEDIR ele de chegar a ficar de pe") A fracao do
            // nivel de 0,8 a 1 (era de 0,4 a 1).
            fraction = 0.8f + 0.2f * u(w.rng);
        }
        w.fireKind = kind;
        w.fireDirection = direction;
        w.fireFraction = kind == 0 ? 0.0f : fraction;
        if (kind != 0 && !getUpFireExam && getUpFireAbove > 0.0f && u(w.rng) < getUpFireAbove) {
            w.fireDirection = direction + 8;
            w.fireAbovePitch = (45.0f + 35.0f * u(w.rng)) * Pi / 180.0f;
        }
        const float* level = &getUpFireLevels[4 * e];
        const float front = w.clipYaw + (e < clipGetUpFacing.size() ? clipGetUpFacing[e] : 0.0f);
        if (kind == 2 || kind == 3) {
            const float angle = front + static_cast<float>(direction) * Pi / 4.0f;
            const float newtons = std::max(0.0f, level[0]) * fraction * profile->totalMassKg * 9.81f;
            const float level2 = std::cos(w.fireAbovePitch), down = std::sin(w.fireAbovePitch);
            w.fireHold = { std::cos(angle) * newtons * level2, std::sin(angle) * newtons * level2, -newtons * down };
            w.fireSize = std::max(0.0f, level[0]) * fraction;
        }
        if (kind == 1 || kind == 3) {
            w.fireTiming = timing;
            const float deliverAt = e < clipGetUpDeliver.size() && clipGetUpDeliver[e] > 0.0f ? clipGetUpDeliver[e]
                : references[w.clipReference]->durationSeconds();
            // (06h20, ele: "a partir do momento em que ele ficou de pe posturado, ele se levantou com sucesso... ha algumas equivocacoes
            // enquanto isso": a pancada do quase fim, pelo tempo do clipe, chegava com ele ja de pe.) O momento pela subida do corpo: o
            // comeco ainda no chao (0,15-0,6 s depois do comeco); o meio quando a cabeca passa de 0,7-1,0 m; o quase fim quando passa de
            // 1,1-1,3 m (correndo, 1,0-1,2 m) - sempre antes do de pe. (deliverAt fica para a referencia.)
            (void)deliverAt;
            const float draw = getUpFireExam ? 0.5f : u(w.rng);
            const bool run = clipRunExercise(static_cast<std::uint32_t>(e));
            if (timing == 1)
                w.fireShoveAt = settings.fireEarlyShare > 0.0f && !getUpFireExam ? 0.05f + (settings.fireEarlyWindow - 0.05f) * draw
                                                                                 : 0.15f + 0.45f * draw;
            else {
                w.fireShoveAt = -1.0f;
                w.fireShoveHeight = timing == 2 ? 0.7f + 0.3f * draw : (run ? 1.0f : 1.1f) + 0.2f * draw;
            }
            const float shove = std::max(0.0f, level[static_cast<std::size_t>(timing)]) * fraction;
            if (kind == 1) w.fireSize = shove;
            w.fireShoveSize = shove;
        }
        w.fireShoveDirection = w.fireDirection;
        // (12/11 12h30, fase 2 - settings.fireShoveSeries/fireWallShare/fireTurnShare) As adversidades novas, fora da prova.
        if (getUpFireExam) return;
        if ((kind == 1 || kind == 3) && settings.fireShoveSeries > 1) {
            w.fireShovesLeft = std::uniform_int_distribution<int>(0, settings.fireShoveSeries - 1)(w.rng);
            if (w.fireShovesLeft > 0) w.fireExtras |= 1;
        }
        if ((kind == 2 || kind == 3) && settings.fireWallShare > 0.0f && u(w.rng) < settings.fireWallShare) {
            const float angle = std::atan2(w.fireHold.y, w.fireHold.x);
            bool blocks = false;
            if (clipRunExercise(static_cast<std::uint32_t>(e)) && !w.body->links.empty()) {
                const Vec3 root = w.body->links[rootLink].position;
                const float rx = w.target.x - root.x, ry = w.target.y - root.y;
                blocks = rx * std::cos(angle) + ry * std::sin(angle) > 0.0f;
            }
            if (!blocks && std::hypot(w.fireHold.x, w.fireHold.y) > 1.0f) {
                w.fireWallAngle = angle;
                w.fireWallDistance = settings.fireWallMin + (std::max(settings.fireWallMin, settings.fireWallMax) - settings.fireWallMin) * u(w.rng);
                w.fireExtras |= 4;
            }
        }
        if ((kind == 1 || kind == 3) && settings.fireAfterStandShare > 0.0f && u(w.rng) < settings.fireAfterStandShare) w.fireAfterStand = true;
        if (kind != 0 && settings.fireSlipShare > 0.0f && u(w.rng) < settings.fireSlipShare) {
            w.fireSlip = true;
            w.fireExtras |= 32;
        }
        if (kind != 0 && settings.fireGrabShare > 0.0f && u(w.rng) < settings.fireGrabShare) {
            const int side = u(w.rng) < 0.5f ? 0 : 1;
            const float angle = 2.0f * Pi * u(w.rng);
            const float newtons = settings.fireGrabFraction * (0.8f + 0.2f * u(w.rng)) * profile->totalMassKg * 9.81f;
            w.fireGrabLink = static_cast<int>(handLinks[side]);
            w.fireGrabForce = Vec3 { std::cos(angle), std::sin(angle), 0.25f }.normalized() * newtons;
            w.fireGrabFrom = 0.2f + 1.3f * u(w.rng);
            w.fireGrabUntil = w.fireGrabFrom + 0.5f + 1.0f * u(w.rng);
            w.fireExtras |= 64;
        }
        if ((kind == 2 || kind == 3) && settings.fireTurnShare > 0.0f && u(w.rng) < settings.fireTurnShare) {
            w.fireNextTurnAt = settings.fireTurnSeconds * (0.7f + 0.6f * u(w.rng));
            w.fireExtras |= 8;
        }
    }

    // (12/11 04h40) A cada decisao, no levantar com fogo: a rampa da forca continua (entra em 0,3 s; depois da entrega sai em 0,3 s), a
    // pancada na hora dela (pelos pontos do fogo versatil, no rumo a partir da frente do corpo de agora) e o "foi ao chao" depois dela.
    // (12/11 08h45, settings.fireRunHeadwindCap) A forca continua do fogo de agora (com a rampa); nos correndo, a parte contra a corrida
    // limitada.
    // (12h50, settings.fireGrabShare) O agarrao puxando agora.
    bool fireGrabActive(const World& w) const {
        return w.fireGrabLink >= 0 && static_cast<std::size_t>(w.fireGrabLink) < w.body->links.size() && w.time >= w.fireGrabFrom
            && w.time < w.fireGrabUntil && w.clipDeliveredAt < 0.0f;
    }
    Vec3 fireHoldNow(const World& w) const {
        Vec3 f = w.fireHold * w.fireHoldScale;
        // (12/11 16h30, o fogo da locomocao) Andando, a parte contra o deslocamento pedido limitada do mesmo jeito.
        if (settings.fireRunHeadwindCap > 0.0f && w.locoFire && w.commandSpeed > 0.0f) {
            const float ux = std::cos(w.moveYaw), uy = std::sin(w.moveYaw);
            const float along = f.x * ux + f.y * uy;
            const float cap = settings.fireRunHeadwindCap * profile->totalMassKg * 9.81f * w.fireHoldScale;
            if (along < -cap) {
                f.x += (-cap - along) * ux;
                f.y += (-cap - along) * uy;
            }
            return f;
        }
        if (settings.fireRunHeadwindCap > 0.0f && w.clipGetUp && clipRunExercise(w.clipExercise) && !w.body->links.empty()) {
            const Vec3 root = w.body->links[rootLink].position;
            float ux = w.target.x - root.x, uy = w.target.y - root.y;
            const float n = std::hypot(ux, uy);
            if (n > 1.0e-3f) {
                ux /= n;
                uy /= n;
                const float along = f.x * ux + f.y * uy;
                const float cap = settings.fireRunHeadwindCap * profile->totalMassKg * 9.81f * w.fireHoldScale;
                if (along < -cap) {
                    f.x += (-cap - along) * ux;
                    f.y += (-cap - along) * uy;
                }
            }
        }
        return f;
    }
    void maybeGetUpFire(World& w, const TrackingGetUpPosture3D& now) {
        if (!w.clipGetUp || w.fireKind == 0) return;
        if (w.fireKind == 2 || w.fireKind == 3) {
            // (13/11 12h55, settings.fireReleaseAtFirm) O soltar desde o primeiro firme de pe, nao desde a entrega.
            const float from = settings.fireReleaseAtFirm && w.fireReleaseFrom >= 0.0f ? w.fireReleaseFrom : w.clipDeliveredAt;
            w.fireHoldScale = from >= 0.0f ? std::max(0.0f, 1.0f - (w.time - from) / settings.fireReleaseSeconds) : std::min(1.0f, w.time / 0.3f);
        }
        // (06h20) Depois de levantar (a entrega), nada mais: a pancada que nao veio fica sem vir. (12h45, fase 2: uma vez, na entrega; com
        // settings.fireAfterStandShare, a pancada de depois de ficar de pe, na janela de firmeza.)
        if (w.clipDeliveredAt >= 0.0f && !w.fireStandHandled) {
            w.fireStandHandled = true;
            if (!w.fireShoveDone && (w.fireKind == 1 || w.fireKind == 3)) w.fireShoveDone = true, w.fireShoveAt = -1.0f;
            w.fireShovesLeft = 0;
            w.fireNextTurnAt = -1.0f;
            if (w.fireGrabUntil > w.time) w.fireGrabUntil = w.time;
            if (w.fireAfterStand && w.clipHandedAt < 0.0f && w.pushTicks == 0) {
                std::uniform_real_distribution<float> u(0.0f, 1.0f);
                w.fireShoveDone = false;
                w.fireShoveHeight = 0.0f;
                w.fireShoveAt = w.time + 0.1f + 0.3f * u(w.rng);
                w.fireShoveDirection = std::uniform_int_distribution<int>(0, 7)(w.rng);
                w.fireExtras |= 16;
            }
        }
        // (12/11 12h30, fase 2 - settings.fireTurnShare) O vento que muda: o rumo do continuo gira +-45-135 graus.
        if (w.fireNextTurnAt >= 0.0f && w.time >= w.fireNextTurnAt) {
            std::uniform_real_distribution<float> u(0.0f, 1.0f);
            const float turn = (u(w.rng) < 0.5f ? -1.0f : 1.0f) * static_cast<float>(1 + std::uniform_int_distribution<int>(0, 2)(w.rng)) * Pi / 4.0f;
            const float c = std::cos(turn), s = std::sin(turn);
            w.fireHold = { c * w.fireHold.x - s * w.fireHold.y, s * w.fireHold.x + c * w.fireHold.y, w.fireHold.z };
            w.fireNextTurnAt = w.time + settings.fireTurnSeconds * (0.7f + 0.6f * u(w.rng));
        }
        // (12/11 12h30, fase 2 - settings.fireShoveSeries) A pancada seguinte da serie, 0,4-1,2 s depois da anterior, num rumo novo.
        if (w.fireShovesLeft > 0 && w.fireShoveDone && w.fireShoveAt >= 0.0f && w.pushTicks == 0 && w.clipDeliveredAt < 0.0f
            && w.time >= w.fireShoveAt + w.fireShoveSeconds) {
            std::uniform_real_distribution<float> u(0.0f, 1.0f);
            --w.fireShovesLeft;
            w.fireShoveDone = false;
            w.fireShoveHeight = 0.0f;
            w.fireShoveAt = w.time + settings.fireSeriesGapMin + (settings.fireSeriesGapMax - settings.fireSeriesGapMin) * u(w.rng);
            w.fireShoveDirection = std::uniform_int_distribution<int>(0, 7)(w.rng) + (w.fireDirection >= 8 ? 8 : 0);
        }
        const bool shoveNow = w.fireShoveHeight > 0.0f ? now.headHeight >= w.fireShoveHeight : w.fireShoveAt >= 0.0f && w.time >= w.fireShoveAt;
        // (13/11 13h15, settings.fireReleaseAtFirm; o laboratorio: uma pancada da serie comecou entre o firme e a entrega e acertou o idle)
        // Firme de pe, nenhuma pancada nova.
        const bool released = settings.fireReleaseAtFirm && w.fireReleaseFrom >= 0.0f;
        if ((w.fireKind == 1 || w.fireKind == 3) && !w.fireShoveDone && shoveNow && w.pushTicks == 0 && !released) {
            std::uniform_real_distribution<float> u(0.0f, 1.0f);
            const float angle = trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink])
                + static_cast<float>(w.fireShoveDirection % 8) * Pi / 4.0f;
            const float seconds = getUpFireExam ? 0.15f
                : settings.fireShoveSecondsMin + (settings.fireShoveSecondsMax - settings.fireShoveSecondsMin) * u(w.rng);
            float newtons = profile->totalMassKg * w.fireShoveSize / seconds;
            // (13h40, settings.fireShoveNewtonsMax) A forca sorteada em N; o tamanho (m/s) segue para o registro e o desenho.
            if (!getUpFireExam && settings.fireShoveNewtonsMax > 0.0f) {
                newtons = settings.fireShoveNewtonsMin + (settings.fireShoveNewtonsMax - settings.fireShoveNewtonsMin) * u(w.rng);
                w.fireShoveSize = newtons * seconds / std::max(1.0f, profile->totalMassKg);
            }
            const float flat = std::cos(w.fireAbovePitch), down = std::sin(w.fireAbovePitch);
            w.pushForce = { std::cos(angle) * newtons * flat, std::sin(angle) * newtons * flat, -newtons * down };
            pickPushPoint(w);
            // (12/11 12h30, fase 2 - settings.fireLegShare) A rasteira: a pancada numa coxa, canela ou pe (mais na canela e no pe), de lado.
            const bool leg = settings.fireLegShare > 0.0f && !getUpFireExam && u(w.rng) < settings.fireLegShare;
            if (leg) {
                const float draw = u(w.rng);
                const int side = u(w.rng) < 0.5f ? 0 : 1;
                w.pushShares = { 1.0f, 0.0f, 0.0f };
                w.pushLinks[0] = draw < 0.2f ? (side == 0 ? leftThighLink : rightThighLink) : draw < 0.6f ? shinLinks[side] : footLinks[side];
                if (w.pushLinks[0] >= profile->links.size()) w.pushLinks[0] = shinLinks[side];
                w.pushForce.z = 0.0f;
                w.fireExtras |= 2;
            } else if (w.fireShoveDirection >= 8) {
                // (De cima: no alto do tronco - o peito alto ou um ombro.)
                const float draw = u(w.rng);
                w.pushShares = { 1.0f, 0.0f, 0.0f };
                w.pushLinks[0] = draw < 0.5f ? 3u : draw < 0.75f ? leftShoulderLink : rightShoulderLink;
                if (w.pushLinks[0] >= profile->links.size()) w.pushLinks[0] = 3u;
            }
            w.pushTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
            w.lastPushEnd = w.time + seconds;
            ++w.pushes;
            w.fireShoveDone = true;
            w.fireShoveAt = w.time;
            w.fireShoveHead = now.headHeight;
            w.fireShoveForce = w.pushForce;
            w.fireShoveSeconds = seconds;
            w.fireShoveLink = static_cast<int>(w.pushLinks[0]);
            for (std::size_t k = 1; k < w.pushLinks.size(); ++k)
                if (w.pushShares[k] > w.pushShares[static_cast<std::size_t>(0)] && w.pushShares[k] >= 0.48f) w.fireShoveLink = static_cast<int>(w.pushLinks[k]);
        }
        // (Foi ao chao: a cabeca 0,35 m abaixo de onde estava na pancada e abaixo de 0,6 m - a pancada que pega o corpo ainda deitado
        // nao conta.)
        if (w.fireShoveDone && w.fireShoveAt >= 0.0f && w.fireDownAt < 0.0f && w.time > w.fireShoveAt && w.fireShoveHead >= 0.6f
            && now.headHeight < std::min(0.6f, w.fireShoveHead - 0.35f))
            w.fireDownAt = w.time;
    }

    // (12/11 16h30, o fogo da locomocao v1 - ver TrackingGetUpAmpSettings3D::locoFireShare.) Os extremos do nivel L: o continuo (do
    // peso), a pancada seca de L = 0 (N; a de L = 1 e a do levantar, fireShoveNewtons*), o agarrao de L = 0 (do peso); a forca que e
    // p = 1 (do peso) e em quanto tempo p cai a zero (s).
    static constexpr float LocoFireHold0 = 0.05f, LocoFireHold1 = 0.5f, LocoFireShoveMin0 = 80.0f, LocoFireShoveMax0 = 180.0f,
                           LocoFireGrab0 = 0.08f, LocoFirePerturbForce = 0.2f, LocoFirePerturbFade = 1.5f;
    // A duracao do episodio (s): o comando, a estrela, a locomocao ou a de sempre.
    float episodeLimit(const World& w) const {
        // (12/11 19h10: o levantar e ir vira locomocao na troca e fica na duracao do levantar.)
        if (w.getUpGo) return settings.episodeSeconds;
        if (w.fallConverted) {
            // (13/11, settings.fallToCommand) A duracao do exercicio de antes, com pelo menos FallResumeSeconds depois da queda - para
            // levantar e seguir.
            const bool star = w.originalManeuver >= 8 && w.originalManeuver <= 10;
            const float before = star && settings.starSeconds > 0.0f ? settings.starSeconds
                : settings.locomotionSeconds > 0.0f ? settings.locomotionSeconds : settings.episodeSeconds;
            return std::max(before, w.convertedAt + FallResumeSeconds);
        }
        return isCommand(w) && settings.commandSeconds > 0.0f ? settings.commandSeconds
            : isStar(w) && settings.starSeconds > 0.0f ? settings.starSeconds
            : w.locomotion && settings.locomotionSeconds > 0.0f ? settings.locomotionSeconds
            : (!settings.evaluation && (w.controlledPreset == 3 || w.controlledPreset == 4) && !w.locomotion
                  ? std::max(12.0f, settings.episodeSeconds) : settings.episodeSeconds);
    }
    // A liberdade sob o fogo agora (0-1): a perturbacao p vezes locoFireRelief.
    float fireFreedom(const World& w) const {
        return settings.locoFireRelief > 0.0f && w.locoFire ? std::clamp(settings.locoFireRelief * w.firePerturb, 0.0f, 1.0f) : 0.0f;
    }
    // A inclinacao que a forca continua de agora pede (rad): a da vertical efetiva (a gravidade com a forca horizontal por massa).
    float fireLean(const World& w) const {
        if (!w.locoFire || settings.locoFireRelief <= 0.0f || w.body->links.empty()) return 0.0f;
        const Vec3 f = fireHoldNow(w) + (fireGrabActive(w) ? w.fireGrabForce : Vec3 {});
        return std::atan(std::hypot(f.x, f.y) / std::max(1.0f, profile->totalMassKg * 9.81f));
    }
    bool locoFireEligible(const World& w) const {
        // (12/11 18h, ele: "agora pode tambem aplicar a fase de fogo no treino de comando" - o comando entra.)
        return !w.clipGetUp && (w.startKind == 2 || w.startKind == 6 || w.startKind == 7) && !w.body->links.empty();
    }
    float locoShoveNewtons(const World& w, float draw) const {
        const float lo1 = settings.fireShoveNewtonsMax > 0.0f ? settings.fireShoveNewtonsMin : 350.0f;
        const float hi1 = settings.fireShoveNewtonsMax > 0.0f ? settings.fireShoveNewtonsMax : 800.0f;
        const float L = w.locoFireLevel;
        const float lo = LocoFireShoveMin0 + (lo1 - LocoFireShoveMin0) * L, hi = LocoFireShoveMax0 + (hi1 - LocoFireShoveMax0) * L;
        return lo + (hi - lo) * draw;
    }
    // No comeco do episodio (antes do cenario): se tem fogo, o nivel e o que e do episodio inteiro - a grama molhada e o prensado.
    void beginLocomotionFire(World& w) {
        if (!locoFireEligible(w)) return;
        if (settings.crookedNoFire && w.crooked && !(settings.evaluation && settings.locoFireExam)) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const bool exam = settings.evaluation && settings.locoFireExam;
        if (settings.evaluation ? !(exam || (settings.fireShown && settings.locoFireShare > 0.0f))
                                : (settings.locoFireShare <= 0.0f || u(w.rng) >= settings.locoFireShare))
            return;
        w.locoFire = true;
        w.locoFireLevel = exam ? settings.locoFireExamLevel : std::clamp(settings.locoFireLevels[fireGroup(w)], 0.0f, 1.0f);
        w.fireShoveDone = true;
        if (exam) {
            // (O tipo pelo mundo de 8 em 8 - nos rumos o rumo e o mundo % 8, entao cada tipo passa pelos 8 rumos -; o lado pelo mundo % 4.)
            w.locoFireExamKind = static_cast<int>((w.index / 8u) % 4u);
            w.locoFireNextAt = 2.0f;
            if (w.locoFireExamKind == 3) w.fireSlip = true, w.fireExtras |= 32;
            return;
        }
        const bool still = w.startKind == 2 || w.startKind == 7;
        w.locoFireNextAt = still ? 0.5f + u(w.rng) : 1.0f + 1.5f * u(w.rng);
        // (12/11 18h, o parado torto: com o fogo em 90% o primeiro empurrao chegava no meio do reajuste - 0,5-1,5 s, ele leva 3-4 s - e o
        // reajustou caiu de 33% para 5%. Nascido torto, o fogo comeca so depois de 3-4 s.)
        if (w.crooked) w.locoFireNextAt = std::max(w.locoFireNextAt, 3.0f + u(w.rng));
        if (settings.fireSlipShare > 0.0f && u(w.rng) < settings.fireSlipShare * (0.4f + 0.6f * w.locoFireLevel))
            w.fireSlip = true, w.fireExtras |= 32;
        // (O prensado: so de pe no lugar - parado e giro -, a parede do lado do primeiro continuo, que entao vem primeiro.)
        if (still && settings.fireWallShare > 0.0f && u(w.rng) < settings.fireWallShare) {
            const float yaw = trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink]);
            w.fireWallAngle = yaw + static_cast<float>(std::uniform_int_distribution<int>(0, 7)(w.rng)) * Pi / 4.0f;
            // (16h50, a sonda do prensado: a parede a menos de ~1 m derrubava o parado em 0,5 s sem tocar nele - a pelve de 0,92 a
            // 0,19 m; a 2,5 m nada - a locomocao nunca viu parede na observacao do ambiente. A distancia minima desce com L: de
            // fireWallMax no L 0 a fireWallMin no L 1.)
            const float far = std::max(settings.fireWallMin, settings.fireWallMax);
            const float near = far - (far - settings.fireWallMin) * w.locoFireLevel;
            w.fireWallDistance = near + (far - near) * u(w.rng);
            w.fireExtras |= 4;
        }
    }
    // Uma pancada seca da locomocao: no rumo fireShoveDirection a partir da frente do corpo, pelos pontos do fogo versatil ou, na
    // rasteira, numa perna; com os campos do desenho do laboratorio.
    void locoFireShove(World& w, const TrackingGetUpPosture3D& now, float newtons, bool legAllowed) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const bool exam = settings.evaluation && settings.locoFireExam;
        const float angle = trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink])
            + static_cast<float>(w.fireShoveDirection % 8) * Pi / 4.0f;
        const float seconds = exam ? 0.3f
            : settings.fireShoveSecondsMin + (std::max(settings.fireShoveSecondsMin, settings.fireShoveSecondsMax) - settings.fireShoveSecondsMin) * u(w.rng);
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        pickPushPoint(w);
        if (legAllowed && !exam && settings.fireLegShare > 0.0f && u(w.rng) < settings.fireLegShare) {
            const float draw = u(w.rng);
            const int side = u(w.rng) < 0.5f ? 0 : 1;
            w.pushShares = { 1.0f, 0.0f, 0.0f };
            w.pushLinks[0] = draw < 0.2f ? (side == 0 ? leftThighLink : rightThighLink) : draw < 0.6f ? shinLinks[side] : footLinks[side];
            if (w.pushLinks[0] >= profile->links.size()) w.pushLinks[0] = shinLinks[side];
            w.fireExtras |= 2;
        }
        w.pushTicks = std::max(1, static_cast<int>(std::lround(std::max(TickSeconds, seconds) / TickSeconds)));
        w.lastPushEnd = std::max(w.lastPushEnd, w.time + seconds);
        ++w.pushes;
        w.fireShoveDone = true;
        w.fireShoveAt = w.time;
        w.fireShoveHead = now.headHeight;
        w.fireShoveForce = w.pushForce;
        w.fireShoveSeconds = seconds;
        w.fireShoveSize = newtons * seconds / std::max(1.0f, profile->totalMassKg);
        w.fireShoveLink = static_cast<int>(w.pushLinks[0]);
        for (std::size_t k = 1; k < w.pushLinks.size(); ++k)
            if (w.pushShares[k] > w.pushShares[0] && w.pushShares[k] >= 0.48f) w.fireShoveLink = static_cast<int>(w.pushLinks[k]);
        w.fireKind |= 1;
    }
    // A cada decisao, com o fogo da locomocao: o continuo (a rampa de 0,3 s, o vento que vira), a serie de pancadas, a proxima
    // adversidade quando a anterior acabou, a perturbacao p (e as janelas do empurrao abertas enquanto houver forca) e o seguir a tarefa.
    void maybeLocomotionFire(World& w, const TrackingGetUpPosture3D& now) {
        if (!w.locoFire) return;
        const float dt = TicksPerDecision * TickSeconds;
        // (12/11 18h, settings.commandGetUp) Levantando no comando, o fogo pausa: o continuo sai na rampa, nada novo, e volta 1,5 s depois.
        if (w.clipGetUp && w.clipHandedAt < 0.0f) {
            w.fireHoldScale = std::max(0.0f, w.fireHoldScale - dt / 0.3f);
            if (w.locoFireNextAt >= 0.0f) w.locoFireNextAt = std::max(w.locoFireNextAt, w.time + 1.5f);
            w.firePerturb = std::max(0.0f, w.firePerturb - dt / LocoFirePerturbFade);
            return;
        }
        // (12/11 18h20) Nascido torto, o fogo espera o reajuste (+0,5 s), no maximo ate 6 s: com o piso de 3-4 s o reajuste (que levava
        // 4-5 s) ainda caiu de 33% para 14-20%.
        if (w.crooked && w.crookedFixedAt < 0.0f && w.time < 6.0f && w.locoFireNextAt >= 0.0f)
            w.locoFireNextAt = std::max(w.locoFireNextAt, w.time + 0.5f);
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float L = w.locoFireLevel;
        const float weight = profile->totalMassKg * 9.81f;
        const bool exam = settings.evaluation && settings.locoFireExam;
        const bool still = w.startKind == 2 || w.startKind == 7;
        static constexpr int ExamDirections[4] { 0, 4, 2, 6 };
        const int examDirection = ExamDirections[w.index % 4u];
        const bool holding = w.locoFireHoldUntil > w.time;
        w.fireHoldScale = holding ? std::min(1.0f, w.fireHoldScale + dt / 0.3f) : std::max(0.0f, w.fireHoldScale - dt / 0.3f);
        if (holding && w.fireNextTurnAt >= 0.0f && w.time >= w.fireNextTurnAt) {
            const float turn = (u(w.rng) < 0.5f ? -1.0f : 1.0f) * static_cast<float>(1 + std::uniform_int_distribution<int>(0, 2)(w.rng)) * Pi / 4.0f;
            const float c = std::cos(turn), sn = std::sin(turn);
            w.fireHold = { c * w.fireHold.x - sn * w.fireHold.y, sn * w.fireHold.x + c * w.fireHold.y, w.fireHold.z };
            w.fireNextTurnAt = w.time + settings.fireTurnSeconds * (0.7f + 0.6f * u(w.rng));
        }
        if (w.fireShovesLeft > 0 && w.fireShoveDone && w.pushTicks == 0 && w.time >= w.fireShoveAt + w.fireShoveSeconds) {
            --w.fireShovesLeft;
            w.fireShoveDone = false;
            w.fireShoveAt = w.time + (exam ? 0.5f : settings.fireSeriesGapMin + (settings.fireSeriesGapMax - settings.fireSeriesGapMin) * u(w.rng));
            if (!exam) w.fireShoveDirection = std::uniform_int_distribution<int>(0, 7)(w.rng);
        }
        if (!w.fireShoveDone && w.fireShoveAt >= 0.0f && w.time >= w.fireShoveAt && w.pushTicks == 0)
            locoFireShove(w, now, locoShoveNewtons(w, exam ? 0.5f : u(w.rng)), !exam);
        const bool busy = holding || w.fireShovesLeft > 0 || !w.fireShoveDone || w.pushTicks > 0 || w.fireGrabUntil > w.time;
        // (12/11 20h05, a revisao com o Codex: o fim do continuo nao conta a serie de pancadas de dentro dele, que pode passar dele - o
        // intervalo saia do fim estimado e o evento seguinte vinha logo depois da ultima pancada. O minimo do intervalo conta do fim real.)
        if (busy && w.locoFireNextAt >= 0.0f && !exam)
            w.locoFireNextAt = std::max(w.locoFireNextAt, w.time + settings.locoFireGapScale * (still ? 1.0f : 1.5f));
        if (w.locoFireNextAt >= 0.0f && w.time >= w.locoFireNextAt && !busy) {
            // (0 o continuo, 1 a serie de pancadas, 2 o agarrao.)
            int kind = 1;
            if (exam) kind = w.locoFireExamKind == 3 ? 1 : w.locoFireExamKind;
            else if (w.fireWallDistance >= 0.0f && w.locoFireEvents == 0) kind = 0;
            else {
                const float d = u(w.rng);
                kind = d < 0.4f ? 0 : d < 0.8f || settings.fireGrabShare <= 0.0f ? 1 : 2;
            }
            float end = w.time;
            const float bodyYaw = trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink]);
            const int most = 1 + static_cast<int>(std::lround(static_cast<float>(std::max(1, settings.fireShoveSeries) - 1) * L));
            if (kind == 0) {
                const int direction = exam ? examDirection : std::uniform_int_distribution<int>(0, 7)(w.rng);
                const float angle = w.fireWallDistance >= 0.0f && w.locoFireEvents == 0 ? w.fireWallAngle
                    : bodyYaw + static_cast<float>(direction) * Pi / 4.0f;
                const float size = (LocoFireHold0 + (LocoFireHold1 - LocoFireHold0) * L) * (exam ? 1.0f : 0.8f + 0.2f * u(w.rng));
                w.fireHold = { std::cos(angle) * size * weight, std::sin(angle) * size * weight, 0.0f };
                w.fireSize = size;
                w.fireDirection = direction;
                const float seconds = exam ? 4.0f : 3.0f + 3.0f * u(w.rng);
                w.locoFireHoldUntil = w.time + seconds;
                end = w.locoFireHoldUntil;
                w.fireKind |= 2;
                w.fireNextTurnAt = !exam && settings.fireTurnShare > 0.0f && u(w.rng) < settings.fireTurnShare
                    ? w.time + settings.fireTurnSeconds * (0.7f + 0.6f * u(w.rng)) : -1.0f;
                if (w.fireNextTurnAt >= 0.0f) w.fireExtras |= 8;
                // (Metade dos continuos com uma serie de pancadas dentro - o "continuo + pancada" do levantar.)
                if (!exam && u(w.rng) < 0.5f) {
                    w.fireShovesLeft = std::uniform_int_distribution<int>(0, most - 1)(w.rng);
                    if (w.fireShovesLeft > 0) w.fireExtras |= 1;
                    w.fireShoveDone = false;
                    w.fireShoveAt = w.time + 0.5f + std::max(0.0f, seconds - 1.5f) * u(w.rng);
                    w.fireShoveDirection = std::uniform_int_distribution<int>(0, 7)(w.rng);
                }
            } else if (kind == 1) {
                w.fireShovesLeft = exam ? (w.locoFireExamKind == 3 ? 0 : 2) : std::uniform_int_distribution<int>(0, most - 1)(w.rng);
                if (w.fireShovesLeft > 0) w.fireExtras |= 1;
                w.fireShoveDone = false;
                w.fireShoveAt = w.time;
                w.fireShoveDirection = exam ? examDirection : std::uniform_int_distribution<int>(0, 7)(w.rng);
                end = w.time + static_cast<float>(w.fireShovesLeft + 1) * (settings.fireShoveSecondsMax + settings.fireSeriesGapMax);
            } else {
                const int side = u(w.rng) < 0.5f ? 0 : 1;
                const float angle = exam ? bodyYaw + static_cast<float>(examDirection) * Pi / 4.0f : 2.0f * Pi * u(w.rng);
                const float fraction = LocoFireGrab0 + (std::max(LocoFireGrab0, settings.fireGrabFraction) - LocoFireGrab0) * L;
                w.fireGrabLink = static_cast<int>(handLinks[side]);
                w.fireGrabForce = Vec3 { std::cos(angle), std::sin(angle), 0.25f }.normalized()
                    * (fraction * (exam ? 1.0f : 0.8f + 0.2f * u(w.rng)) * weight);
                w.fireGrabFrom = w.time;
                w.fireGrabUntil = w.time + (exam ? 1.0f : 0.5f + u(w.rng));
                end = w.fireGrabUntil;
                w.fireExtras |= 64;
            }
            ++w.locoFireEvents;
            w.locoFireNextAt = exam ? -1.0f : end + settings.locoFireGapScale * (still ? 1.0f + 1.5f * u(w.rng) : 1.5f + 2.0f * u(w.rng));
        }
        const Vec3 f = fireHoldNow(w) + (w.pushTicks > 0 ? w.pushForce : Vec3 {}) + (fireGrabActive(w) ? w.fireGrabForce : Vec3 {});
        const float pf = std::min(1.0f, f.length() / std::max(1.0f, LocoFirePerturbForce * weight));
        w.firePerturb = std::max(pf, w.firePerturb - dt / LocoFirePerturbFade);
        if (w.fireSlip && w.fireSlipHalf > 0.0f) {
            const Vec3 root = w.body->links[rootLink].position;
            if (std::abs(root.x - w.fireSlipCenter.x) < w.fireSlipHalf && std::abs(root.y - w.fireSlipCenter.y) < w.fireSlipHalf)
                w.firePerturb = std::max(w.firePerturb, 0.5f);
        }
        if (pf > 0.05f) w.lastPushEnd = std::max(w.lastPushEnd, w.time);
        if (w.fireDownAt < 0.0f && now.headHeight < 0.6f) w.fireDownAt = w.time;
    }
    // (12/11 16h45) Seguindo a tarefa: com fogo, desde a primeira adversidade; sem, desde 1,5 s (a mesma regua no limpo do mesmo grupo -
    // a escada compara o fogo com ele: nos laterais e nas manobras a regua absoluta reprovava ate o limpo).
    void trackLocomotionKeep(World& w, const TrackingGetUpPosture3D& now) {
        if ((settings.locoFireShare <= 0.0f && !settings.locoFireExam) || !locoFireEligible(w)) return;
        if (w.locoFire ? w.locoFireEvents == 0 : w.time < 1.5f) return;
        {
            bool keep = false;
            if (w.commandSpeed > 0.0f) {
                const float along = w.smoothVelocity.x * std::cos(w.moveYaw) + w.smoothVelocity.y * std::sin(w.moveYaw);
                const float speed = std::hypot(w.smoothVelocity.x, w.smoothVelocity.y);
                const float course = speed > 0.3f ? std::abs(wrapAngle(std::atan2(w.smoothVelocity.y, w.smoothVelocity.x) - w.moveYaw)) : Pi;
                keep = along >= 0.6f * w.commandSpeed && course < 30.0f * Pi / 180.0f;
            } else {
                keep = now.standing && std::hypot(w.smoothVelocity.x, w.smoothVelocity.y) < 0.5f;
            }
            w.locoKeep += keep ? 1 : 0;
            ++w.locoKeepSamples;
        }
    }

    void beginFromClip(World& w, bool standing) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const bool protective = !standing && !protectionIndices.empty() && u(w.rng) < 0.5f;
        const auto& reference = standing ? *references[u(w.rng) < 0.5f ? standingIndex : standingMirrorIndex]
            : protective
            ? *references[protectionIndices[std::uniform_int_distribution<std::size_t>(0, protectionIndices.size() - 1)(w.rng)]]
            : *references[std::uniform_int_distribution<std::size_t>(0, std::size(GetUpClips) - 1)(w.rng)];
        const float last = std::max(0.0f, reference.durationSeconds() * (protective ? 0.7f : 1.0f) - 2.0f / 30.0f);
        w.protectiveStart = protective;
        const float t = u(w.rng) * last, yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        w.lastClip = static_cast<std::size_t>(&reference == references[standingIndex].get() ? standingIndex
            : &reference == references[standingMirrorIndex].get() ? standingMirrorIndex : std::numeric_limits<std::size_t>::max());
        w.lastClipTime = t;
        w.lastClipYaw = yaw;
        reference.frame(t, {}, yaw, w.frame);
        // (Na rampa do evitar ambientado: o corpo sobe ate o chao embaixo das partes de baixo dele.)
        float lift = 0.0f;
        for (const PhysicsBodyState3D& s : w.frame.links)
            if (s.position.z < 0.3f) lift = std::max(lift, terrainHeight(w, s.position));
        for (PhysicsBodyState3D& s : w.frame.links) s.position.z += lift;
        // (12/11 17h, settings.crookedStartShare) O parado que nasce torto.
        if (standing && w.startKind == 2 && settings.crookedStartShare > 0.0f && u(w.rng) < settings.crookedStartShare) crookPose(w);
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }
    // (12/11 17h, ele: "no treino do Parado... que ele nascesse em uma posicao torta propositalmente, e um pouco mais alto (para cair um
    // pouco), nascer com a coluna, bracos, pernas tortas... e fazer ele treinar, aprender justamente a se REAJUSTAR (o mais rapido que
    // ele conseguir) e ficar firme... vai solucionar ou pelo menos melhorar os problemas da coluna torta apos frear") A pose do idle com
    // cada parte girada ao acaso em torno da junta dela (o pai antes do filho, o subtronco inteiro junto): as vertebras ate 12 graus, o
    // pescoco e a cabeca 12, os bracos 35, os antebracos e as maos 20, as coxas 12, as canelas 10, os pes 8 (40-100% disso, eixo sorteado);
    // e o corpo 3-8 cm acima do chao.
    void crookPose(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        std::vector<PhysicsBodyState3D>& links = w.frame.links;
        const std::size_t n = std::min(links.size(), profile->links.size());
        const auto below = [&](std::size_t d, std::size_t c) {
            for (int k = static_cast<int>(d); k >= 0; k = profile->links[static_cast<std::size_t>(k)].parentIndex)
                if (static_cast<std::size_t>(k) == c) return true;
            return false;
        };
        for (std::size_t c = 0; c < n; ++c) {
            const RagdollLinkDefinition3D& def = profile->links[c];
            if (def.parentIndex < 0) continue;
            const std::string& id = def.id;
            const auto has = [&](const char* part) { return id.find(part) != std::string::npos; };
            const float most = has("Abdomen") || has("Chest") || has("Neck") || has("Head") ? 12.0f
                : has("UpperArm") ? 35.0f : has("Forearm") || has("Hand") ? 20.0f : has("Thigh") ? 12.0f : has("Shin") ? 10.0f
                : has("Foot") ? 8.0f : 0.0f;
            if (most <= 0.0f) continue;
            Vec3 axis { 2.0f * u(w.rng) - 1.0f, 2.0f * u(w.rng) - 1.0f, 2.0f * u(w.rng) - 1.0f };
            if (axis.length() < 1.0e-3f) continue;
            axis = axis.normalized();
            const float angle = most * (0.4f + 0.6f * u(w.rng)) * Pi / 180.0f;
            const Quaternion turn = Quaternion::fromAxisAngle(axis, angle);
            const PhysicsBodyState3D& s = links[c];
            const Vec3 anchor = s.position + s.orientation.rotate(def.modelOrientation.conjugate().rotate(def.inboundJoint.anchorModelPosition - def.modelPosition));
            for (std::size_t d = c; d < n; ++d) {
                if (!below(d, c)) continue;
                links[d].position = anchor + turn.rotate(links[d].position - anchor);
                links[d].orientation = (turn * links[d].orientation).normalized();
            }
        }
        const float lift = 0.03f + 0.05f * u(w.rng);
        for (PhysicsBodyState3D& s : links) s.position.z += lift;
        w.crooked = true;
    }

    // Um rumo. (03/11, o rastreio - vault 04_locomocao/plano_rastreio_locomocao_03_11.md: o passo da faixa sorteada, um
    // dos clipes cardinais do modo padrao, a velocidade pedida a do clipe +-15% e o clipe tocado nessa proporcao
    // (trackScale). O deslocamento pedido e 45 graus x rumo a partir da frente do comeco; o corpo de frente para a frente
    // do clipe (faceYaw = trackYaw = o deslocamento menos o rumo do clipe: nas diagonais, o clipe da frente ou de costas
    // com o corpo virado para o rumo, como o modo padrao). Em locomotionClipFraction do treino ja no clipe (o RSI:
    // instante e rumo sorteados, na velocidade pedida), senao de pe no parado em alerta com o clipe desde o comeco e a
    // velocidade pedida em rampa. Na avaliacao: a faixa (ou o trote), o rumo pedido, a velocidade do clipe, de pe.)
    // (Sem o rastreio, como antes: a parada no meio de 20% do treino ou a troca do pedido.)
    void beginHeading(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (teacher || settings.taughtHeadings) {
            beginTaughtHeading(w);
            return;
        }
        const Gait* gait = nullptr;
        if (settings.evaluation) {
            w.band = settings.evaluationBand >= 0 && settings.evaluationBand < 4 ? static_cast<std::uint32_t>(settings.evaluationBand) : 1u;
            w.heading = settings.heading >= 0 ? static_cast<std::uint32_t>(settings.heading) % 8u : w.index % 8u;
            // (05/11: o sprint so para a frente.)
            if (w.band == 3) w.heading = 0;
            gait = gaitFor(w.band, w.heading);
            if (gait == nullptr) {
                w.band = 1;
                gait = gaitFor(1, w.heading);
            }
        } else {
            gait = &pickGait(w);
            w.heading = gait->heading;
            // (04/11, ele: "nos temos mecanica das diagonais" - no rastreio, os 8 rumos por igual: a diagonal com o clipe
            // da frente ou de costas, a pelve no rumo e a coluna devolvendo o peito para a vista.)
            const std::uint32_t h = std::uniform_int_distribution<std::uint32_t>(0u, 7u)(w.rng);
            if (const Gait* g = gaitFor(w.band, h)) {
                gait = g;
                w.heading = h;
            }
            // (05/11: o sprint so para a frente, no sprint_maximo - o primeiro da faixa 3.)
            if (w.band == 3)
                if (const Gait* g = gaitFor(3u, 0u)) {
                    gait = g;
                    w.heading = 0;
                }
        }
        // (As manobras: na avaliacao a fixa - ou uma por mundo em ciclo -, no treino na fatia maneuverFraction. Os codigos
        // 1-3 seguem do rumo sorteado; a troca livre e as curvas comecam para a frente.)
        // (08/11, a estrela: 8-10 os tres peitos, 11 os tres em ciclo pelo mundo; no treino, a fatia starFraction - um
        // dos 6 pelos pesos, a faixa dele.)
        if (settings.evaluation)
            w.maneuver = settings.evaluationManeuver == CommandManeuver ? CommandManeuver
                : settings.evaluationManeuver >= 11 ? 8 + static_cast<int>(w.index % 3u)
                : settings.evaluationManeuver >= 8 ? settings.evaluationManeuver
                : settings.evaluationManeuver > 0 ? std::min(settings.evaluationManeuver, 7)
                : settings.evaluationManeuver == 0 ? 1 + static_cast<int>(w.index % 7u) : 0;
        else if (w.band < 3 && settings.commandFraction > 0.0f && u(w.rng) < settings.commandFraction) {
            w.maneuver = CommandManeuver;
            if (settings.commandPhase <= 1) w.band = 1u;
        } else if (w.band < 3 && settings.starFraction > 0.0f && u(w.rng) < settings.starFraction) {
            float total = 0.0f;
            for (const float v : settings.starWeights) total += std::max(0.0f, v);
            float draw = u(w.rng) * (total > 0.0f ? total : 1.0f);
            int pick = 5;
            for (int k = 0; k < 6; ++k)
                if ((draw -= std::max(0.0f, settings.starWeights[static_cast<std::size_t>(k)])) < 0.0f) {
                    pick = k;
                    break;
                }
            w.maneuver = 8 + pick % 3;
            w.band = pick < 3 ? 1u : 2u;
        } else if (w.band < 3 && u(w.rng) < settings.maneuverFraction) {
            // (06/11 22h10, ele: "intensificar a quantidade de treinos destinados aos mais deficientes": os pesos das 7
            // manobras pelo treino - maneuverWeights.)
            const auto& Weights = settings.maneuverWeights;
            float total = 0.0f;
            for (const float v : Weights) total += std::max(0.0f, v);
            float draw = u(w.rng) * (total > 0.0f ? total : 1.0f);
            w.maneuver = 7;
            for (int k = 0; k < 7; ++k)
                if ((draw -= std::max(0.0f, Weights[static_cast<std::size_t>(k)])) < 0.0f) {
                    w.maneuver = k + 1;
                    break;
                }
        }
        // (06/11 01h10, ele: "precisamos de uma solucao melhor para a troca de direcoes, ainda esta muito fragil... pense bem
        // o que fazer para ajudar ele a aprender melhor" - trocaPractice) Nas trocas de peito fixo e pela meia-volta (1-3),
        // o rumo de partida pelos pesos que o treino manda (metade por igual, metade pelas quedas recentes de cada rumo): as
        // variantes que caem treinam mais. Medido na r35080: das quedas da media no trote, 13 de 15 partiam do lateral
        // esquerdo; da media e da forte correndo, 9 de 13 e 7 de 11.
        if (!settings.evaluation && settings.trocaPractice && w.maneuver >= 1 && w.maneuver <= 3) {
            float total = 0.0f;
            for (float v : settings.trocaHeadingWeights) total += std::max(0.0f, v);
            if (total > 0.0f) {
                float draw = u(w.rng) * total;
                std::uint32_t h = 7u;
                for (std::uint32_t k = 0; k < 8u; ++k)
                    if ((draw -= std::max(0.0f, settings.trocaHeadingWeights[k])) < 0.0f) {
                        h = k;
                        break;
                    }
                if (const Gait* g = gaitFor(w.band, h)) {
                    gait = g;
                    w.heading = h;
                }
            }
        }
        if (isCommand(w)) {
            // (O comando: o rumo e a faixa sorteados sao a primeira tecla.)
            if (const Gait* g = gaitFor(w.band, w.heading)) gait = g;
        } else if (isStar(w)) {
            // (A estrela: a primeira ponta sorteada; o peito fixo e o girando saem no rumo dela, o livre de frente.)
            w.star = {};
            w.star.phase = std::clamp(settings.starPhases[(w.band >= 2 ? 3u : 0u) + static_cast<std::size_t>(w.maneuver - 8)], 1, 4);
            w.star.next = std::uniform_int_distribution<int>(0, 7)(w.rng);
            const std::uint32_t h = w.maneuver == 9 ? 0u : static_cast<std::uint32_t>(w.star.next);
            if (const Gait* g = gaitFor(w.band, h)) {
                gait = g;
                w.heading = h;
            }
        } else if (w.maneuver >= 4) {
            if (const Gait* g = gaitFor(w.band, 0u)) {
                gait = g;
                w.heading = 0;
            }
        }
        w.startBand = w.band;
        w.pendingClip = std::numeric_limits<std::size_t>::max();
        if (gait == nullptr) {
            beginFromClip(w, true);
            w.faceYaw = w.moveYaw = bodyYaw(w);
            return;
        }
        const float clipSpeed = std::max(0.1f, travel[gait->clip].speed);
        const float clipDirection = travel[gait->clip].direction;
        w.trackTwist = wrapAngle(clipDirection - 0.25f * Pi * static_cast<float>(w.heading));
        w.twistClip = gait->clip;
        w.runBand = w.band >= 2;
        // (04/11 12h35: com o pedido fixo do rumo, ele - na avaliacao e no treino, sem o sorteio de +-15%.)
        // (07/11, a receita nova: no treino o pedido fixo vezes o sorteio da faixa - ver speedRangeLow.)
        w.speedScale = settings.evaluation || settings.speedRangeLow >= 1.0f ? 1.0f
            : settings.speedRangeLow + (1.0f - settings.speedRangeLow) * u(w.rng);
        const float fixed = fixedSpeed(w.band, w.heading) * w.speedScale;
        w.commandSpeed = fixed > 0.0f ? fixed : clipSpeed * (settings.evaluation ? 1.0f : 0.85f + 0.3f * u(w.rng));
        // (04/11 05h15, ele: "na corrida de costas aceitar que ele faca o movimento do trote de costas tambem, podendo ser
        // ambos" - e 05h17 "o mesmo vale para as diagonais costas": na corrida com o clipe de costas, metade dos episodios
        // do treino com o trote de costas nos motores (tocado na velocidade da corrida), e a semelhanca vale a melhor das
        // duas - a do clipe nos motores ou a do outro.)
        std::size_t motor = gait->clip;
        float motorSpeed = clipSpeed;
        if (w.maneuver == 0 && gait->clip == runBackIndex && jogBackIndex < references.size()) {
            // (04/11 06h45: o trote nos motores saiu - tocado na velocidade da corrida era ~1,4x a cadencia dele, e a
            // semelhanca da corrida de costas caiu de 0,60 a 0,15 com o pe engasgando e arrastando. O sprint sempre nos
            // motores; a semelhanca segue aceitando a melhor das duas.)
            w.altClip = jogBackIndex;
        }
        if (!settings.evaluation && u(w.rng) < settings.locomotionClipFraction) {
            startInClip(w, motor);
        } else {
            beginFromClip(w, true);
            w.trackClip = motor;
            w.trackStart = 0.0f;
            w.trackYaw = w.lastClipYaw + 0.25f * Pi * static_cast<float>(w.heading) - clipDirection;
            w.trackScale = fixed > 0.0f ? 1.0f : w.commandSpeed / motorSpeed;
            w.rampFrom = 0.0f;
            // (06/11: os rumos de lado com a largada mais suave - ver lateralStartAcceleration.)
            const bool sideways = settings.lateralStartAcceleration > 0.0f && (w.heading % 8u == 2u || w.heading % 8u == 6u);
            const float startAcceleration = sideways ? settings.lateralStartAcceleration : StartAcceleration;
            w.rampSeconds = fixed > 0.0f ? std::max(StartRampSeconds, w.commandSpeed / startAcceleration) : StartRampSeconds;
            w.launchEnd = w.rampSeconds;
            // (01/11 20h: de pe direto no ciclo do sprint ele caia na largada; na corrida e no sprint para a frente, a
            // arrancada do clipe idle_to_sprint primeiro, e no fim dela o ciclo.)
            // (04/11 08h50: tambem o trote, e as diagonais da frente - o passo delas e o da frente com a cintura; a arrancada
            // tocada na escala que a faz acabar na velocidade pedida, e a velocidade pedida e a dela, de 0 - ver startSpeed.)
            // (12h15, ele: "tirar a necessidade de usar aquele movimento de tomada de impulso agressivo do idle para trote -
            // encaixa bem na corrida mas fica exagerado no trote; deixar ele mesmo adaptar um movimento mais suave": so a
            // corrida; o trote larga pela rampa, a velocidade subindo de 0, com a janela da arrancada.)
            if (w.band >= 2 && gait->heading == 0 && startClipIndex < references.size()
                && travel[startClipIndex].rampDisplacement > 0.0f && !(settings.unrealStarts && w.band == 2 && w.maneuver == 0)) {
                w.pendingClip = w.trackClip;
                w.trackClip = startClipIndex;
                const float end = 2.0f * travel[startClipIndex].rampDisplacement
                    / std::max(1.0e-3f, references[startClipIndex]->durationSeconds());
                w.trackScale = fixed > 0.0f ? (settings.runLaunchScale > 0.0f ? settings.runLaunchScale : 1.0f)
                    : std::clamp(w.commandSpeed / std::max(0.1f, end), StartClipScaleMinimum, StartClipScaleMaximum);
                w.rampFrom = -1.0f;
                // (Com o pedido fixo a arrancada na cadencia dela, e a janela vai ate o fim da subida depois dela.)
                const float rise = settings.runLaunchAcceleration > 0.0f ? settings.runLaunchAcceleration : StartAcceleration;
                w.launchEnd = references[startClipIndex]->durationSeconds() / w.trackScale
                    + (fixed > 0.0f ? std::max(0.0f, w.commandSpeed - w.trackScale * end) / rise : 0.0f);
            }
        }
        // (01/11 22h30: a frente do corpo e a frente do clipe, nao a pelve - nas corridas de lado a pelve fica ~22 graus
        // virada.)
        w.faceYaw = w.trackYaw + w.trackTwist;
        // (03/11 17h40, ele: o peito sempre para o ponto de vista e o rumo exato - "em treino de andar para direita, andar
        // na diagonal frente direita" e trapaca.)
        w.moveYaw = w.trackYaw + clipDirection;
        // (10/11 04h, settings.unrealStarts) De pe nos retos da corrida, a arrancada da Unreal do rumo antes do ciclo.
        w.unrealStart = -1;
        if (settings.unrealStarts && (w.band == 1 || w.band == 2) && w.maneuver == 0 && w.rampFrom >= 0.0f)
            enterUnrealStart(w);
        if (w.maneuver > 0) {
            w.refActive = true;
            const float start = w.rampFrom >= 0.0f || starting(w) ? 0.0f : w.commandSpeed;
            w.refVelocity = { std::cos(w.moveYaw) * start, std::sin(w.moveYaw) * start, 0.0f };
            w.nextManeuverAt = settings.evaluation ? 2.0f : 1.5f + 1.5f * u(w.rng);
            w.viewTarget = w.faceYaw;
            w.straightUntil = w.nextManeuverAt;
            w.arcLeft = 0.0f;
            planCourse(w);
            if (isStar(w)) startStar(w);
            if (isCommand(w)) startCommand(w);
        }
        // (08/11 15h20: a parada no fim dos retos - na fatia stopFraction do treino, no instante sorteado na janela; na avaliacao,
        // so com stopInEvaluation, no meio dela.)
        if (w.maneuver == 0 && w.band < 3
            && (settings.evaluation ? settings.stopInEvaluation : settings.stopFraction > 0.0f && u(w.rng) < settings.stopFraction))
            w.finalStopAt = settings.evaluation ? 0.5f * (settings.stopEarliest + settings.stopLatest)
                : settings.stopEarliest + (settings.stopLatest - settings.stopEarliest) * u(w.rng);
    }

    // O giro parado e o parado com a professora (03/11 20h30): parada, com a vista no rumo pedido do corpo (no giro, a que
    // gira - o pedido dela muda a cada decisao).
    void beginTaughtStill(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        w.taught = true;
        if (!teacher) return;
        w.teacherEpisode = settings.evaluation ? settings.teacherShare >= 0.5f : u(w.rng) < settings.teacherShare;
        w.teacherSession = teacher->begin(*w.scene, w.handle, {}, w.faceYaw);
        beginTeacherLead(w);
    }
    // (21h55, a ultima tentativa: nos episodios da aluna, a B21 dirige o comeco - de teacherLeadMinimum a ...Maximum s - e
    // entrega no embalo dela: a aluna pratica a partir dos estados reais do andar dela.)
    void beginTeacherLead(World& w) {
        if (settings.evaluation || w.teacherEpisode || settings.teacherLeadMaximum <= 0.0f) return;
        w.teacherUntil = settings.teacherLeadMinimum
            + (settings.teacherLeadMaximum - settings.teacherLeadMinimum) * std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng);
    }
    // O parado com a professora hibrida (teacherStillClip): o tronco, os bracos e a cabeca na pose do IDLE ALERTA.
    bool stillClipTaught(const World& w) const {
        return settings.teacherStillClip > 0 && w.taught && w.startKind == 2 && w.trackClip < references.size();
    }
    bool upperBody(std::uint32_t link) const {
        const std::string& id = profile->links[link].id;
        return id.find("Thigh") == std::string::npos && id.find("Shin") == std::string::npos
            && id.find("Foot") == std::string::npos && id.find("Toe") == std::string::npos;
    }

    // Um rumo com a professora (03/11 18h27; o revisor: sem o RSI - ela comeca do zero e nao sabe o pe de apoio -, sem o
    // desvio de +-1 rumo, sem paradas, trocas nem rampa): de pe no parado em alerta, a vista no rumo do corpo, o rumo
    // sorteado de 0 a 7 (na avaliacao, o dela), a aluna com o pedido do que a professora faz e ela com o da tabela.
    void beginTaughtHeading(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        w.band = 1;
        w.startBand = 1;
        w.heading = settings.evaluation ? (settings.heading >= 0 ? static_cast<std::uint32_t>(settings.heading) % 8u : w.index % 8u)
                                        : std::uniform_int_distribution<std::uint32_t>(0u, 7u)(w.rng);
        w.pendingClip = std::numeric_limits<std::size_t>::max();
        w.taught = true;
        w.commandSpeed = std::max(0.1f, settings.teacherStudentSpeeds[w.heading]);
        // (03/11 23h, ele: "o RL o mais puro possivel... exatamente igual fizemos para ele aprender a se levantar": como o
        // levantar comecava em instantes dos clipes, nos rumos cardinais uma fatia comeca num instante do clipe do trote
        // daquele rumo - a vista no rumo do clipe, a velocidade na pedida -; o resto, de pe.)
        const Gait* cardinal = w.heading % 2u == 0u ? gaitFor(1, w.heading) : nullptr;
        if (!settings.evaluation && cardinal != nullptr && u(w.rng) < settings.locomotionClipFraction) {
            const std::size_t clip = cardinal->clip;
            const Travel& run = travel[clip];
            const float yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
            const float scale = run.speed > 0.0f ? w.commandSpeed / run.speed : 1.0f;
            references[clip]->frame(u(w.rng) * references[clip]->durationSeconds(), {}, yaw, w.frame);
            const Vec3 v { std::cos(yaw + run.direction) * w.commandSpeed, std::sin(yaw + run.direction) * w.commandSpeed, 0.0f };
            for (PhysicsBodyState3D& state : w.frame.links) {
                state.linearVelocity = state.linearVelocity * scale + v;
                state.angularVelocity = state.angularVelocity * scale;
            }
            w.scene->resetRagdollState(w.handle, w.frame.links);
            w.body = w.scene->ragdollStateView(w.handle);
            w.lastClipYaw = yaw;
        } else {
            beginFromClip(w, true);
        }
        w.trackClip = std::numeric_limits<std::size_t>::max();
        w.rampFrom = -1.0f;
        w.faceYaw = w.lastClipYaw;
        w.moveYaw = w.faceYaw + 0.25f * Pi * static_cast<float>(w.heading);
        if (!teacher) return;
        const float asked = std::max(0.1f, settings.teacherSpeeds[w.heading]);
        w.teacherSession = teacher->begin(*w.scene, w.handle,
            { std::cos(w.moveYaw) * asked, std::sin(w.moveYaw) * asked, 0.0f }, w.faceYaw);
        w.teacherEpisode = settings.evaluation ? settings.teacherShare >= 0.5f : u(w.rng) < settings.teacherShare;
        beginTeacherLead(w);
    }

    // O passo de uma faixa num rumo (0 a 7): o clipe cardinal; nas diagonais, o da frente (1, 7) ou o de costas (3, 5).
    const Gait* gaitFor(std::uint32_t band, std::uint32_t heading) const {
        if (band >= gaits.size()) return nullptr;
        const std::uint32_t cardinal = heading % 2u == 0u ? heading % 8u : (heading == 3u || heading == 5u ? 4u : 0u);
        for (const Gait& g : gaits[band])
            if (g.heading == cardinal) return &g;
        return nullptr;
    }

    // Um passo pelas fatias das faixas (andar, corrida, sprint; o resto trota), uniforme entre os carregados da faixa.
    const Gait& pickGait(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (!settings.onlyClip.empty())
            for (std::uint32_t b = 0; b < 4; ++b)
                for (const Gait& g : gaits[b])
                    if (clipNames[g.clip] == settings.onlyClip) {
                        w.band = b;
                        return g;
                    }
        const float draw = u(w.rng);
        std::uint32_t band = 1;
        if (draw < settings.walkFraction) band = 0;
        else if (draw < settings.walkFraction + settings.runFraction) band = 2;
        else if (draw < settings.walkFraction + settings.runFraction + settings.sprintFraction) band = 3;
        if (gaits[band].empty()) band = 1;
        w.band = band;
        const std::vector<Gait>& list = gaits[band];
        return list[std::uniform_int_distribution<std::size_t>(0, list.size() - 1)(w.rng)];
    }

    // O corpo num instante sorteado de um clipe de locomocao, num rumo sorteado, na velocidade pedida (o RSI; o clipe
    // tocado na proporcao dela).
    void startInClip(World& w, std::size_t clip) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
        const float t = u(w.rng) * references[clip]->durationSeconds();
        const Travel& run = travel[clip];
        const float speed = w.commandSpeed > 0.0f ? w.commandSpeed : run.speed;
        w.trackClip = clip;
        w.trackStart = t;
        w.trackYaw = yaw;
        w.trackScale = fixedSpeed(w.band, w.heading) > 0.0f ? 1.0f : run.speed > 0.0f ? speed / run.speed : 1.0f;
        w.rampFrom = -1.0f;
        references[clip]->frame(t, {}, yaw, w.frame, twist(w), pitch(w), legPitch(w));
        const Vec3 v { std::cos(yaw + run.direction) * speed, std::sin(yaw + run.direction) * speed, 0.0f };
        for (PhysicsBodyState3D& s : w.frame.links) {
            s.linearVelocity = s.linearVelocity * w.trackScale + v;
            s.angularVelocity = s.angularVelocity * w.trackScale;
        }
        w.scene->resetRagdollState(w.handle, w.frame.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // A troca do pedido no meio de um rumo (a rl_final: as transicoes): CommandStopChance para e fica no rumo do corpo,
    // senao um passo novo pelas fatias; nos passos para a frente o rumo pedido do corpo vira para um lado sorteado (ate
    // 180 graus, virar correndo), nos outros fica (o deslocamento muda em relacao a ele).
    // A troca de peito fixo (os codigos 1-3): o rumo pula 45, 90 ou 135-180 graus para um lado sorteado, com a vista
    // parada; o clipe novo entra na fase do pe esquerdo do de antes e o quadro se mistura do velho para o novo enquanto a
    // velocidade de referencia freia e reacelera (ManeuverAcceleration; o tempo da mistura e o dela, 0,25 a 1 s).
    // A perda do equilibrio no rastreio: a cabeca abaixo disto (m; 04/11: 1,1 cortava a frenagem agachada da troca).
    // (04/11 03h20, ele: "da uma afrouxada nessa regra, as vezes ele precisa disso para aprender, como ja recompensamos
    // por estabilidade" - so perto do chao: a cabeca abaixo de 0,6 m ou a pelve abaixo de 0,35 m; o corpo virado acima de
    // 100 graus por 0,5 s.)
    // (v6) A janela de improviso: dura a frenagem (ou o giro do olhar na troca livre) e mais isto (s); sai em 0,3 s.
    static constexpr float ImproviseAfterSeconds = 0.5f, ImproviseFadeSeconds = 0.3f;
    static constexpr float LostHeadHeight = 0.6f, LostPelvisHeight = 0.35f, TurnedAwayRadians = 1.75f, TurnedAwaySeconds = 0.5f;
    static constexpr float TurnGraceRate = 1.2f; // rad/s (~70 graus/s) da referencia que conta como girando rapido
    static constexpr float ManeuverAcceleration = 6.0f, ManeuverLateral = 4.0f, FreeViewRate = 4.0f;
    // (06/11 01h10, trocaPractice) O intervalo ate a proxima troca no treino: 1,5-3 s, ou 1-2 s com a pratica (na avaliacao
    // 2 s, sem sortear - o sorteio a mais mudava a sequencia das trocas).
    float maneuverGap(const World&, float draw) const { return settings.trocaPractice ? 1.0f + draw : 1.5f + 1.5f * draw; }
    // A troca chega com a anterior ainda acontecendo (a freada, a espera do pe, a arrancada): com a pratica ela espera
    // TrocaWaitSeconds em vez de se perder.
    static constexpr float TrocaWaitSeconds = 0.25f;
    // (07/11 07h55, ele: "seus treinos de trocas de direcoes estao ERRADOS, por isso nao evolui... tu esta pedindo 2 ou mais
    // trocas de direcoes por episodio ao inves de apenas uma": depois de 1,5-3 s cada troca agendava a seguinte - 2 a 4 num
    // episodio de 8 s, a forte sem tempo de se recompor. Agora uma so: feita a troca, nenhuma outra; o resto do episodio e o
    // passo no rumo novo.)
    static constexpr float NoMoreManeuvers = 1.0e9f;
    bool trocaBusy(const World& w) const { return settings.trocaPractice && (w.brakeTurnAt >= 0.0f || starting(w)); }
    void changeHeadingFixed(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const int steps = w.maneuver == 1 ? 1 : w.maneuver == 2 ? 2 : (u(w.rng) < 0.5f ? 3 : 4);
        const int sign = u(w.rng) < 0.5f ? 1 : -1;
        // (09/11 15h, settings.freeTroca) Media e forte livres: de frente, a vista gira o angulo da troca (a meia-volta do jogador, o
        // rumo em relacao a vista igual); de costas e de lado com o peito fixo, o rumo em relacao a vista muda.
        if (settings.freeTroca && w.maneuver >= 2) {
            const bool turnView = !(settings.trocaFixedChestNonForward && w.heading >= 2u && w.heading <= 6u);
            const std::uint32_t next = turnView ? w.heading : static_cast<std::uint32_t>((static_cast<int>(w.heading) + sign * steps + 16) % 8);
            if (startFreeTroca(w, w.band, next, turnView ? static_cast<float>(sign * steps) * 0.25f * Pi : 0.0f)) {
                w.nextManeuverAt = NoMoreManeuvers;
                return;
            }
        }
        // (05/11 00h15, ele: "nao e so para meia volta... eu falei para ADAPTAR aquela [meia-volta] para TODAS as trocas de
        // direcao que nao sao suaves ou exigem freada": na media (90 graus) e na forte (135-180) o rumo novo vem pela virada -
        // freia, planta, gira o corpo no angulo da troca e arranca para a frente no rumo novo; a suave segue de peito fixo.)
        if (w.maneuver >= 2 && !(settings.trocaFixedChestNonForward && w.heading >= 2u && w.heading <= 6u)) {
            const float turn = static_cast<float>(sign * steps) * 0.25f * Pi;
            w.nextManeuverAt = w.time + (settings.evaluation ? 2.0f : maneuverGap(w, u(w.rng)));
            if (w.runTurn < 0 && w.brakeTurnAt < 0.0f && requestRunTurn(w, turn)) {
                w.runAgainAt = -1.0f;
                w.nextManeuverAt = NoMoreManeuvers;
                return;
            }
            if (w.brakeTurnAt >= 0.0f) return;
        }
        const std::uint32_t next = static_cast<std::uint32_t>((static_cast<int>(w.heading) + sign * steps + 16) % 8);
        w.nextManeuverAt = NoMoreManeuvers;
        switchGait(w, w.band, next);
    }
    // A troca do clipe do passo (o rumo e a faixa) com a mistura do de antes, no pouso do pe esquerdo dele, a velocidade de
    // referencia indo a nova ate ManeuverAcceleration (a troca de peito fixo; e, 05/11 21h, a freada e a volta a corrida da
    // troca correndo). Falso sem o clipe ou na arrancada.
    bool switchGait(World& w, std::uint32_t band, std::uint32_t next, float speed = -1.0f) {
        const Gait* g = gaitFor(band, next);
        if (g == nullptr || w.trackClip >= references.size() || starting(w)) return false;
        const float view = w.trackYaw + twist(w);
        const float unwrapped = w.trackStart + startRampTime(w, w.time) * w.trackScale;
        const float oldDuration = std::max(1.0e-3f, references[w.trackClip]->durationSeconds());
        w.blendClip = w.trackClip;
        w.blendScale = w.trackScale;
        w.blendStart = unwrapped - w.time * w.trackScale;
        w.blendYaw = w.trackYaw;
        w.blendTwist = twist(w);
        w.blendPitch = pitch(w);
        w.blendLegPitch = legPitch(w);
        float phase = unwrapped / oldDuration - travel[w.trackClip].leftLand + travel[g->clip].leftLand;
        phase -= std::floor(phase);
        // (05/11 21h55, runTurnAlign) Alinhar sempre pelo pe esquerdo nao e simetrico: na troca suave do trote, da diagonal
        // frente-esquerda ao lateral esquerdo 7 de 10 caiam, e no espelho (ao lateral direito) 0 de 14. Os dois alinhamentos -
        // pelo pouso do esquerdo e pelo do direito - e fica o quadro do clipe novo mais perto do de agora (como a saida da
        // meia-volta): o espelho de uma troca escolhe o espelho do alinhamento.
        if (settings.runTurnAlign) {
            float other = unwrapped / oldDuration - travel[w.trackClip].rightLand + travel[g->clip].rightLand;
            other -= std::floor(other);
            if (std::abs(other - phase) > 1.0e-3f) {
                const float newDuration = references[g->clip]->durationSeconds();
                TrackingClipFrame3D current, left, right;
                references[w.trackClip]->frame((unwrapped / oldDuration - std::floor(unwrapped / oldDuration)) * oldDuration, {}, 0.0f,
                    current);
                references[g->clip]->frame(phase * newDuration, {}, 0.0f, left);
                references[g->clip]->frame(other * newDuration, {}, 0.0f, right);
                float dl = 0.0f, dr = 0.0f;
                for (std::size_t i = 0; i < current.coordinates.size() && i < left.coordinates.size() && i < right.coordinates.size(); ++i) {
                    dl += (left.coordinates[i] - current.coordinates[i]).lengthSquared();
                    dr += (right.coordinates[i] - current.coordinates[i]).lengthSquared();
                }
                if (dr < dl) phase = other;
            }
        }
        const float factor = w.commandSpeed / std::max(0.1f, travel[w.trackClip].speed);
        const float clipSpeed = std::max(0.1f, travel[g->clip].speed), clipDirection = travel[g->clip].direction;
        const float fixed = fixedSpeed(band, next) * w.speedScale;
        w.band = band;
        w.runBand = band >= 2;
        // (speed >= 0: a velocidade dada, o clipe na cadencia dela - a freada da troca assentada.)
        w.commandSpeed = speed >= 0.0f ? speed : fixed > 0.0f ? fixed : clipSpeed * factor;
        w.trackClip = g->clip;
        w.trackScale = speed >= 0.0f ? speed / clipSpeed : fixed > 0.0f ? 1.0f : w.commandSpeed / clipSpeed;
        w.rampFrom = -1.0f;
        w.trackStart = phase * references[g->clip]->durationSeconds() - w.time * w.trackScale;
        w.heading = next;
        w.trackTwist = wrapAngle(clipDirection - 0.25f * Pi * static_cast<float>(next));
        w.twistClip = g->clip;
        w.trackYaw = view + 0.25f * Pi * static_cast<float>(next) - clipDirection;
        w.moveYaw = w.trackYaw + clipDirection;
        const Vec3 velocity { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
        w.blendFrom = w.time;
        w.blendSeconds = std::clamp((velocity - w.refVelocity).length() / settings.maneuverAcceleration, 0.25f, 1.0f);
        w.improviseUntil = w.time + w.blendSeconds + ImproviseAfterSeconds;
        return true;
    }
    // (05/11 21h, a troca correndo pela freada - runTurnBrake) Na corrida, antes da meia-volta, a freada ate o trote no mesmo
    // rumo (o clipe do trote pela troca de clipe, a velocidade de referencia a ManeuverAcceleration) e BrakeSettleSeconds; a
    // meia-volta entra dai (brakeTurnAt). Falso fora da corrida, ja devagar ou sem o trote.
    static constexpr float BrakeSettleSeconds = 0.15f, RunAgainSeconds = 0.6f;
    static constexpr float FootCrossMargin = 0.015f, CrossedInward = 0.03f;
    // (05/11 21h40, runTurnAlign) A meia-volta no pe certo: a da direita planta o pe esquerdo aos 0,17 s (a da esquerda o
    // direito); entrando no instante errado da passada o corpo nao tem o pe para plantar - medido nas trocas correndo, erro de
    // fase de 0,3-0,5 ciclo: 36% de quedas (84 em 235); ate 0,2 ciclo: 0 em 139. A espera (s) ate o pe do passo de agora que
    // o clipe planta primeiro pousar no instante do plantio do clipe; ate AlignLateCycles de ciclo atrasado entra na hora.
    static constexpr float AlignLateCycles = 0.1f;
    float turnEntryWait(const World& w, float turn) const {
        const std::size_t index = runTurnIndices[turn > 0.0f ? 1u : 0u];
        if (index >= references.size() || w.trackClip >= references.size() || travel[w.trackClip].speed <= 0.0f) return 0.0f;
        const Travel& clip = travel[index];
        const Travel& gait = travel[w.trackClip];
        const bool left = clip.leftLand <= clip.rightLand;
        const float plant = (left ? clip.leftLand : clip.rightLand) * references[index]->durationSeconds();
        const float duration = std::max(1.0e-3f, references[w.trackClip]->durationSeconds());
        const float rate = std::max(0.1f, w.trackScale * startRamp(w, w.time));
        const float landing = (left ? gait.leftLand : gait.rightLand) * duration;
        const float cycle = duration / rate;
        float wait = std::fmod(landing - trackTime(w, 0.0f) + 2.0f * duration, duration) / rate - plant;
        if (wait < -AlignLateCycles * cycle) wait += cycle;
        return std::max(0.0f, wait);
    }
    // O pedido da meia-volta: pela freada (runTurnBrake), pelo pe (runTurnAlign) - os dois deixam o giro esperando em
    // brakeTurnAt - ou na hora. Falso se nao houve giro (nem esperando).
    bool requestRunTurn(World& w, float turn) {
        if (brakeBeforeTurn(w, turn)) {
            w.turnAligned = false;
            return true;
        }
        if (settings.runTurnAlign && !w.turnAligned) {
            const float wait = turnEntryWait(w, turn);
            if (wait > 0.5f * TicksPerDecision * TickSeconds) {
                w.brakeTurn = turn;
                w.brakeTurnAt = w.time + wait;
                w.turnAligned = true;
                return true;
            }
        }
        w.turnAligned = false;
        return beginRunTurn(w, turn);
    }
    // (05/11 22h40, turnSettle) De costas ou de lado, em qualquer faixa: a meia-volta e o giro de quem vem para a frente, e o
    // corpo chegava nela com ~2 m/s para o lado errado. Medido (r31640, 64 mundos com ruido): com 31% de forca, todas as quedas
    // da media (16) e da forte (7) comecam de costas - a media de costas com 135 graus de troca do deslocamento cai 16 de 17,
    // de frente 0 de 143; com 15%, de costas 100%. Freia ate TurnSettleSpeed no proprio passo (a cadencia dele junto) antes.
    static constexpr float TurnSettleSpeed = 1.5f;
    bool brakeBeforeTurn(World& w, float turn) {
        if (starting(w) || w.trackClip >= references.size()) return false;
        // (22h55: so de costas - de lado, correndo, a freada lenta no passo lateral derrubava: 23 das 31 quedas da forte
        // correndo vinham dela, e as trocas de lado nao caiam na forca de agora.)
        if (settings.turnSettle && w.heading >= 3u && w.heading <= 5u) {
            const float from = w.commandSpeed, settle = settings.turnSettleBackSpeed > 0.0f ? settings.turnSettleBackSpeed : TurnSettleSpeed;
            if (from <= settle + 0.05f || !switchGait(w, 1u, w.heading, settle)) return false;
            w.brakeTurn = turn;
            w.brakeTurnAt = w.time + (from - settle) / settings.maneuverAcceleration + BrakeSettleSeconds;
            w.runAgainAt = -1.0f;
            return true;
        }
        if (settings.turnSettleLateralSpeed > 0.0f && (w.heading == 2u || w.heading == 6u)) {
            // (06/11, de lado: a freada ate turnSettleLateralSpeed, no trote e na corrida.)
            const float from = w.commandSpeed, to = settings.turnSettleLateralSpeed;
            if (from <= to + 0.05f || !switchGait(w, 1u, w.heading, to)) return false;
            w.brakeTurn = turn;
            w.brakeTurnAt = w.time + (from - to) / settings.maneuverAcceleration + BrakeSettleSeconds;
            w.runAgainAt = -1.0f;
            return true;
        }
        if (settings.turnSettleFrontSpeed > 0.0f && (w.heading == 0u || w.heading == 1u || w.heading == 7u)) {
            // (09/11 04h30, ele: "acho que e mais na troca de direcao forte" - medido: o trote forte cai 11 de 24, a corrida forte
            // 6 de 24; a meia-volta girava ~180 graus em 0,6 s com o corpo a 3,5-4 m/s e o pedido em 4 m/s girando junto - o corpo
            // seguia reto, 150-200 graus atras, e acabava "virado" ou caido. De frente, a mesma freada de quem vem de costas.)
            const float from = w.commandSpeed, to = settings.turnSettleFrontSpeed;
            if (from <= to + 0.05f || !switchGait(w, 1u, w.heading, to)) return false;
            w.brakeTurn = turn;
            w.brakeTurnAt = w.time + (from - to) / settings.maneuverAcceleration + BrakeSettleSeconds;
            w.runAgainAt = -1.0f;
            return true;
        }
        if (!settings.runTurnBrake || w.band != 2u) return false;
        const float trot = fixedSpeed(1u, w.heading) * w.speedScale, from = w.commandSpeed;
        if (trot <= 0.0f || from <= trot + 0.05f || !switchGait(w, 1u, w.heading)) return false;
        w.brakeTurn = turn;
        w.brakeTurnAt = w.time + (from - w.commandSpeed) / settings.maneuverAcceleration + BrakeSettleSeconds;
        w.runAgainAt = -1.0f;
        return true;
    }
    // A manobra a cada decisao: as trocas na hora delas; a troca livre gira a vista (e o clipe da frente com ela) ate o
    // rumo novo a FreeViewRate; as curvas giram o rumo a velocidade / raio pelo arco sorteado, depois uma reta (na em S, o
    // lado sempre troca). A velocidade de referencia vai ao pedido ate ManeuverAcceleration - na troca livre, no maximo a
    // que cabe em ManeuverLateral no giro da vista.
    void advanceManeuver(World& w) {
        if (w.maneuver == 0 || (!locomotionCommanded(w) && !isCommand(w))) return;
        // (12/11 18h, settings.commandGetUp) Levantando, o jogador espera.
        if (isCommand(w) && w.clipGetUp && w.clipHandedAt < 0.0f) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float dt = TicksPerDecision * TickSeconds;
        // (05/11 21h, runTurnBrake) Freado, a meia-volta; e, trotando depois dela, a volta a corrida no rumo de agora.
        if (w.brakeTurnAt >= 0.0f && w.time >= w.brakeTurnAt && w.runTurn < 0) {
            const float turn = w.brakeTurn;
            w.brakeTurn = 0.0f;
            w.brakeTurnAt = -1.0f;
            if (!requestRunTurn(w, turn)) w.runAgainAt = w.time + RunAgainSeconds;
        }
        if (w.runAgainAt >= 0.0f && w.time >= w.runAgainAt && w.runTurn < 0 && w.brakeTurnAt < 0.0f && !starting(w)) {
            w.runAgainAt = -1.0f;
            if (w.startBand == 2u && w.band != 2u) switchGait(w, 2u, w.heading);
        }
        if (w.freeTroca) {
            // (A troca livre: o comando segue com as teclas e o mouse - outra tecla recomeca a troca, soltar e a parada livre.)
            if (isCommand(w)) commandTick(w, dt);
            if (w.freeTroca) updateFreeTroca(w, dt);
            return;
        }
        Vec3 target { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
        if (w.runTurn >= 0) {
            // (A virada em andamento - de qualquer troca: o rumo do quadro e a vista seguem o giro do clipe na escala do angulo.)
            const float before = w.faceYaw;
            if (!followRunTurn(w)) endRunTurn(w);
            w.arcRate = wrapAngle(w.faceYaw - before) / dt;
            target = { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
        } else if (isCommand(w)) {
            target = commandTick(w, dt);
        } else if (isStar(w)) {
            target = starTick(w, dt);
        } else if (w.maneuver <= 3) {
            if (w.time >= w.nextManeuverAt && trocaBusy(w)) {
                w.nextManeuverAt = w.time + TrocaWaitSeconds;
            } else if (w.time >= w.nextManeuverAt) {
                changeHeadingFixed(w);
                target = { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
            }
        } else if (w.maneuver == 4) {
            bool turned = false;
            if (w.time >= w.nextManeuverAt && trocaBusy(w)) {
                w.nextManeuverAt = w.time + TrocaWaitSeconds;
            } else if (w.time >= w.nextManeuverAt) {
                const float turn = (20.0f + 160.0f * u(w.rng)) * Pi / 180.0f * (u(w.rng) < 0.5f ? 1.0f : -1.0f);
                w.nextManeuverAt = NoMoreManeuvers;
                turned = std::abs(turn) >= RunTurnMinimum && w.brakeTurnAt < 0.0f
                    && (settings.freeTroca ? startFreeTroca(w, w.band, w.heading, turn) : requestRunTurn(w, turn));
                if (turned) w.runAgainAt = -1.0f;
                if (!turned) {
                    w.viewTarget = wrapAngle(w.faceYaw + turn);
                    w.moveYaw = w.viewTarget;
                }
            }
            if (turned) {
                target = { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
            } else {
                const float step = std::clamp(wrapAngle(w.viewTarget - w.faceYaw), -FreeViewRate * dt, FreeViewRate * dt);
                w.faceYaw = wrapAngle(w.faceYaw + step);
                w.trackYaw = w.faceYaw;
                w.arcRate = step / dt;
                if (std::abs(step) > 1.0e-4f) w.improviseUntil = w.time + ImproviseAfterSeconds;
                const float limit = std::abs(w.arcRate) > 0.1f ? std::min(w.commandSpeed, ManeuverLateral / std::abs(w.arcRate)) : w.commandSpeed;
                target = { std::cos(w.faceYaw) * limit, std::sin(w.faceYaw) * limit, 0.0f };
            }
        } else if (settings.courseLimit > 0.0f && w.course.size() >= 2) {
            measureCourse(w);
            // (04/11 14h, ele: "nas curvas ele nao segue a trajetoria correta tao bem e esta sendo deixado passar - deveria
            // compreender como fora do rumo quando sai ou escapa": o rumo girava num relogio e o percurso supunha a velocidade
            // pedida desde o comeco - lento, ele fazia a curva mais fechada e ficava longe das marcas, sem custo. Agora o rumo
            // persegue o percurso: o ponto dele CourseLookSeconds de pedido a frente do mais perto do corpo.)
            const Vec3 root = w.body->links[rootLink].position;
            const float look = std::max(CourseLookMinimum, CourseLookSeconds * w.commandSpeed);
            // (O ponto da mira interpolado na linha - os pontos sao a cada 1 m.)
            const float at = std::min(w.courseProgress + look, static_cast<float>(w.course.size() - 1));
            const std::size_t k = static_cast<std::size_t>(std::max(0.0f, std::floor(at)));
            const Vec3 from = at < 0.0f ? w.courseOrigin : w.course[std::min(k, w.course.size() - 1)];
            const Vec3 to = w.course[at < 0.0f ? 0 : std::min(k + 1, w.course.size() - 1)];
            const float u = at < 0.0f ? at + 1.0f : at - static_cast<float>(k);
            const Vec3 aim = from + (to - from) * u;
            const float yaw = std::atan2(aim.y - root.y, aim.x - root.x);
            w.arcRate += (wrapAngle(yaw - w.moveYaw) / dt - w.arcRate) * 0.3f;
            w.moveYaw = yaw;
            w.faceYaw = w.moveYaw;
            w.trackYaw = w.moveYaw;
            target = { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
        } else {
            measureCourse(w);
            if (w.arcLeft > 0.0f) {
                const float step = std::min(w.arcLeft, std::abs(w.arcRate) * dt);
                w.moveYaw = wrapAngle(w.moveYaw + static_cast<float>(w.arcSign) * step);
                w.arcLeft -= step;
                if (w.arcLeft <= 0.0f) {
                    w.arcRate = 0.0f;
                    w.straightUntil = w.time + (w.courseNext > 0 && w.courseNext <= w.coursePlan.size()
                        ? w.coursePlan[w.courseNext - 1].straightAfter : 1.0f);
                }
            } else if (w.time >= w.straightUntil) {
                // (O proximo arco do plano sorteado no comeco - planCourse; acabou o plano, sorteia mais.)
                if (w.courseNext >= w.coursePlan.size()) extendCourse(w, 4);
                const World::CourseArc& next = w.coursePlan[w.courseNext++];
                w.arcSign = next.rate >= 0.0f ? 1 : -1;
                w.arcLeft = next.radians;
                w.arcRate = next.rate;
            }
            w.faceYaw = w.moveYaw;
            w.trackYaw = w.moveYaw;
            target = { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
        }
        const Vec3 delta = target - w.refVelocity;
        const float size = delta.length(), most = settings.maneuverAcceleration * dt;
        const Vec3 change = size > most ? delta * (most / size) : delta;
        w.refVelocity = w.refVelocity + change;
        w.refAccel = w.refAccel + (change * (1.0f / dt) - w.refAccel) * std::min(1.0f, dt / AccelLeanSmoothing);
    }
    static constexpr float AccelLeanSmoothing = 0.15f;
    // (23h55) A meia-volta: a partir deste angulo (rad) a troca livre usa o clipe dela; a mistura de entrada e de saida (s); os
    // instantes da curva do giro; as fases da corrida testadas para o encaixe na saida.
    // (05/11 00h10, ele: "o movimento de troca de direcao nao esta nada semelhante com os exemplos": a meia-volta so entrava de
    // 90 graus para cima e na janela de improviso - a semelhanca a 0,20 e com metade da exigencia, paga para NAO imitar o
    // clipe. Agora de 45 graus e cobrada inteira.)
    static constexpr float RunTurnMinimum = 0.785f, RunTurnBlendIn = 0.15f, RunTurnBlendOut = 0.2f;
    static constexpr int RunTurnSamples = 40, RunTurnPhases = 24;
    const std::vector<float>& runTurnYaw(int side) {
        std::vector<float>& yaw = runTurnYaws[static_cast<std::size_t>(side)];
        const std::size_t index = runTurnIndices[static_cast<std::size_t>(side)];
        if (!yaw.empty() || index >= references.size()) return yaw;
        const TrackingClipReference3D& reference = *references[index];
        TrackingClipFrame3D frame;
        float previous = 0.0f, sum = 0.0f;
        for (int k = 0; k <= RunTurnSamples; ++k) {
            reference.frame(reference.durationSeconds() * static_cast<float>(k) / RunTurnSamples, {}, 0.0f, frame);
            const float h = pelvisHeading(frame.links[rootLink]);
            if (k == 0) runTurnHeading[static_cast<std::size_t>(side)] = h;
            else sum += wrapAngle(h - previous);
            previous = h;
            yaw.push_back(sum);
        }
        return yaw;
    }
    float runTurnYawAt(int side, float t) {
        const std::vector<float>& yaw = runTurnYaw(side);
        const std::size_t index = runTurnIndices[static_cast<std::size_t>(side)];
        if (yaw.empty()) return 0.0f;
        const float x = std::clamp(t / std::max(1.0e-3f, references[index]->durationSeconds()), 0.0f, 1.0f) * RunTurnSamples;
        const std::size_t k = std::min(static_cast<std::size_t>(x), static_cast<std::size_t>(RunTurnSamples - 1));
        return yaw[k] + (yaw[k + 1] - yaw[k]) * (x - static_cast<float>(k));
    }
    // A troca livre grande: o clipe da meia-volta do lado do giro entra (do de antes, RunTurnBlendIn), no comeco dele, e o giro
    // dele e escalado para dar exatamente o angulo pedido (45 graus pedidos num clipe de 180 = um quarto do giro em cada
    // instante); a rede ve a vista girando junto.
    bool beginRunTurn(World& w, float turn) {
        const int side = turn > 0.0f ? 1 : 0;
        const std::size_t index = runTurnIndices[static_cast<std::size_t>(side)];
        if (index >= references.size() || w.trackClip >= references.size() || starting(w)) return false;
        const float total = runTurnYaw(side).empty() ? 0.0f : runTurnYaw(side).back();
        if (std::abs(total) < 0.5f || (total > 0.0f) != (turn > 0.0f)) return false;
        const float unwrapped = w.trackStart + startRampTime(w, w.time) * w.trackScale;
        w.blendClip = w.trackClip;
        w.blendScale = w.trackScale;
        w.blendStart = unwrapped - w.time * w.trackScale;
        w.blendYaw = w.trackYaw;
        w.blendTwist = twist(w);
        w.blendPitch = pitch(w);
        w.blendLegPitch = legPitch(w);
        w.blendFrom = w.time;
        w.blendSeconds = RunTurnBlendIn;
        w.runTurn = side;
        w.runTurnFrom = w.time;
        w.runTurnScale = std::abs(turn) / std::abs(total);
        w.runTurnStartYaw = w.faceYaw;
        w.trackClip = index;
        // (09/11 04h40, runTurnTimeScale: a meia-volta tocada mais devagar - o mesmo giro em mais tempo - so saindo de costas ou de
        // lado: medido, ali a troca forte falha 50-60% (de frente 8-12%); quase parado, o corpo gira ~27 graus enquanto a
        // referencia gira ~220 em 0,6 s. De frente, a curva correndo funciona no tempo do clipe.)
        w.runTurnTime = w.heading == 0u || w.heading == 1u || w.heading == 7u ? 1.0f : settings.runTurnTimeScale;
        w.trackScale = w.runTurnTime;
        w.trackStart = -w.time * w.runTurnTime;
        w.rampFrom = -1.0f;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackYaw = w.runTurnStartYaw - runTurnHeading[static_cast<std::size_t>(side)];
        w.improviseUntil = -1.0f;
        w.viewTarget = wrapAngle(w.faceYaw + turn);
        w.moveYaw = w.viewTarget;
        return true;
    }
    // Durante a meia-volta: o rumo do quadro tira o giro a mais (o clipe gira o dele; o quadro devolve (1 - escala) dele) e a vista
    // segue o giro escalado. Falso quando acabou o clipe.
    bool followRunTurn(World& w) {
        const std::size_t index = runTurnIndices[static_cast<std::size_t>(w.runTurn)];
        const float t = (w.time - w.runTurnFrom) * w.runTurnTime;
        if (t >= references[index]->durationSeconds() - 1.0f / 30.0f) return false;
        const float yaw = runTurnYawAt(w.runTurn, t);
        w.trackYaw = w.runTurnStartYaw - runTurnHeading[static_cast<std::size_t>(w.runTurn)] + (w.runTurnScale - 1.0f) * yaw;
        w.faceYaw = wrapAngle(w.runTurnStartYaw + w.runTurnScale * yaw);
        return true;
    }
    // A saida: a corrida para a frente no rumo novo, na fase que mais se parece com o fim da meia-volta (RunTurnPhases testadas),
    // o fim dela parado na mistura de saida.
    void endRunTurn(World& w) {
        const std::size_t index = runTurnIndices[static_cast<std::size_t>(w.runTurn)];
        const float duration = references[index]->durationSeconds();
        const Gait* g = gaitFor(w.band, 0u);
        w.runTurn = -1;
        w.faceYaw = w.viewTarget;
        w.heading = 0;
        // (05/11 21h, runTurnBrake: na troca correndo a meia-volta sai trotando - a faixa da freada - e a corrida volta
        // RunAgainSeconds depois.)
        if (settings.runTurnBrake && w.startBand == 2u && w.band != 2u)
            w.runAgainAt = w.time + RunAgainSeconds;
        w.trackTwist = 0.0f;
        if (const float fixed = fixedSpeed(w.band, 0u) * w.speedScale; fixed > 0.0f) w.commandSpeed = fixed;
        if (g == nullptr) return;
        TrackingClipFrame3D last, candidate;
        references[index]->frame(duration - 1.0f / 30.0f, {}, 0.0f, last);
        const float runDuration = std::max(1.0e-3f, references[g->clip]->durationSeconds());
        float best = 0.0f, bestDistance = std::numeric_limits<float>::max();
        for (int k = 0; k < RunTurnPhases; ++k) {
            const float phase = static_cast<float>(k) / RunTurnPhases;
            references[g->clip]->frame(phase * runDuration, {}, 0.0f, candidate);
            float d = 0.0f;
            for (std::size_t i = 0; i < last.coordinates.size() && i < candidate.coordinates.size(); ++i)
                d += (candidate.coordinates[i] - last.coordinates[i]).lengthSquared();
            if (d < bestDistance) {
                bestDistance = d;
                best = phase;
            }
        }
        w.blendClip = index;
        w.blendScale = 0.0f;
        w.blendStart = duration - 1.0f / 30.0f;
        w.blendYaw = w.trackYaw;
        w.blendTwist = 0.0f;
        w.blendPitch = w.blendLegPitch = 0.0f;
        w.blendFrom = w.time;
        w.blendSeconds = RunTurnBlendOut;
        const float clipSpeed = std::max(0.1f, travel[g->clip].speed);
        const float fixed = fixedSpeed(w.band, 0u);
        w.trackClip = g->clip;
        w.trackScale = fixed > 0.0f ? 1.0f : w.commandSpeed / clipSpeed;
        w.trackStart = best * runDuration - w.time * w.trackScale;
        // (05/11 23h20, ele: "na forte, apos a troca de direcao, ele tenta ja de cara engatar o movimento de locomocao, sem
        // arrancada gradual... adaptar o clipe [da arrancada agachada] (certa parte dele em diante) apos a troca, pois ele tem
        // boa base para arrancada em base baixa" - turnLaunch) Na media e na forte, a saida pela arrancada agachada: do
        // quadro dela mais perto do fim do giro (nos primeiros 60%), na escala que a faz acabar na velocidade do passo; no fim
        // dela o ciclo, pela emenda da largada.
        if (settings.turnLaunch && (w.maneuver == 2 || w.maneuver == 3 || w.maneuver == 9) && launchExitIndex < references.size()
            && travel[launchExitIndex].rampDisplacement > 0.0f) {
            const std::size_t launch = launchExitIndex;
            const float launchDuration = std::max(1.0e-3f, references[launch]->durationSeconds());
            float start = 0.0f, startDistance = std::numeric_limits<float>::max();
            for (int k = 0; k < RunTurnPhases; ++k) {
                const float t = 0.6f * launchDuration * static_cast<float>(k) / (RunTurnPhases - 1);
                references[launch]->frame(t, {}, 0.0f, candidate);
                float d = 0.0f;
                for (std::size_t i = 0; i < last.coordinates.size() && i < candidate.coordinates.size(); ++i)
                    d += (candidate.coordinates[i] - last.coordinates[i]).lengthSquared();
                if (d < startDistance) {
                    startDistance = d;
                    start = t;
                }
            }
            const float end = 2.0f * travel[launch].rampDisplacement / launchDuration;
            w.pendingClip = g->clip;
            w.trackClip = launch;
            w.trackScale = std::clamp(w.commandSpeed / std::max(0.1f, end), StartClipScaleMinimum, StartClipScaleMaximum);
            w.trackStart = start - w.time * w.trackScale;
            w.rampFrom = -1.0f;
        }
        // (05/11 22h55: a saida pela rampa da largada - ele: "a forte engata direto sem arrancada gradual" - foi medida em A/B
        // no mesmo checkpoint e saiu: nao mudou a media e a forte e piorou a livre com 15% de forca, 6,2% -> 20,1% de quedas.)
        w.trackYaw = w.faceYaw - travel[w.trackClip].direction;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.improviseUntil = -1.0f;
    }

    // (08/11 14h40, ele: "comece a ja fazer os preparativos para que essa rede consiga interpretar e responder bem aos comandos -
    // olhar para a direcao apontada (o mover do mouse), as direcoes com WASD (as diagonais e etc.), corrida (shift), as paradas...
    // de nada serve uma rede que nao atende os objetivos finais a qual ela se destina"; 14h45: "nao quero controlar no laboratorio,
    // so que ela esteja preparada e adaptada para isso, para ser mais facil depois implementar como a nova locomocao fisica por RL
    // in-game") O comando (manobra 12): um jogador emulado - as teclas (os 8 rumos em relacao a vista, ou nenhuma), o shift (a
    // corrida) e o mouse (a vista: parada, panoramica, flick) - e o corpo atende como no jogo: o rumo novo e a faixa nova pela troca
    // de rumo (switchGait); a vista girando leva o quadro e o rumo juntos (o WASD e relativo a camera); soltar freia ate parar no
    // idle em alerta; apertar arranca do idle pela rampa da largada; parado, a vista longe do peito gira o corpo no lugar pelo clipe
    // do giro. As fases (commandPhase, o treino sobe): 1 so trote, trocas ate 90 graus, mouse ate 0,8 rad/s, um evento a cada
    // 1,5-3 s; 2 + o shift e trocas de 135 graus, mouse 1,5 rad/s; 3 + 180 graus e flicks, a vista a 3 rad/s, 0,8-2,5 s; 4 tudo,
    // flicks a 4 rad/s, 0,5-2 s.
    static constexpr int CommandManeuver = 12;
    static constexpr float CommandStopSpeed = 1.2f, CommandIdleBlend = 0.35f, CommandStartBlend = 0.3f, CommandTurnStart = 0.45f,
                           CommandTurnDone = 0.1f;
    static bool isCommand(const World& w) { return w.maneuver == CommandManeuver; }
    void blendFromNow(World& w, float seconds) {
        const float unwrapped = w.trackStart + startRampTime(w, w.time) * w.trackScale;
        w.blendClip = w.trackClip;
        w.blendScale = w.trackScale;
        w.blendStart = unwrapped - w.time * w.trackScale;
        w.blendYaw = w.trackYaw;
        // (settings.reviewFixes) Saindo do levantar pelo clipe: a mistura parte do clipe do levantar na fase da nota, parada, no rumo dele.
        // (16h50, o portao do ciclo 1: no levantar correndo a mistura parada freava as pernas na saida correndo - correndo de costas 11,9 ->
        // 22,5 cm do clipe -; so no levantar de pe.)
        if (settings.reviewFixes && w.trackClip == w.clipReference && w.clipReference < references.size() && !clipRunExercise(w.clipExercise)
            && std::find(clipGetUpClips.begin(), clipGetUpClips.end(), w.clipReference) != clipGetUpClips.end()) {
            w.blendScale = 0.0f;
            w.blendStart = std::clamp(clipTime(w), 0.0f, references[w.clipReference]->durationSeconds());
            w.blendYaw = w.clipYaw;
        }
        w.blendTwist = twist(w);
        w.blendPitch = pitch(w);
        w.blendLegPitch = legPitch(w);
        w.blendFrom = w.time;
        w.blendSeconds = seconds;
    }
    void startCommand(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        World::Command& c = w.command;
        c = {};
        c.phase = std::clamp(settings.commandPhase, 1, 4);
        c.key = static_cast<int>(w.heading % 8u);
        c.run = w.band >= 2;
        c.target = w.faceYaw;
        c.nextKeyAt = 1.5f + u(w.rng);
        c.nextMouseAt = 0.5f + u(w.rng);
    }
    float commandGap(const World& w, float draw) const {
        static constexpr float Low[4] { 1.5f, 1.2f, 0.8f, 0.5f }, High[4] { 3.0f, 2.8f, 2.5f, 2.0f };
        const std::size_t p = static_cast<std::size_t>(std::clamp(w.command.phase, 1, 4) - 1);
        return Low[p] + (High[p] - Low[p]) * draw;
    }
    // As teclas do jogador emulado: parado, aperta um rumo (a frente mais vezes); andando, solta (18%), troca o shift (18%, da
    // fase 2) ou troca o rumo - ate 90 graus na fase 1, 135 na 2, 180 da 3.
    void commandKeys(World& w) {
        World::Command& c = w.command;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (w.time < c.nextKeyAt) return;
        c.nextKeyAt = w.time + commandGap(w, u(w.rng));
        ++c.events;
        if (c.key < 0) {
            static constexpr float Weights[8] { 3.0f, 1.5f, 1.2f, 0.8f, 1.2f, 0.8f, 1.2f, 1.5f };
            float draw = u(w.rng) * 11.2f;
            int key = 7;
            for (int k = 0; k < 8; ++k)
                if ((draw -= Weights[k]) < 0.0f) {
                    key = k;
                    break;
                }
            c.key = key;
            if (c.phase >= 2) c.run = u(w.rng) < 0.4f;
            return;
        }
        const float r = u(w.rng);
        if (r < 0.18f) {
            c.key = -1;
            return;
        }
        if (c.phase >= 2 && r < 0.36f) {
            c.run = !c.run;
            return;
        }
        const int most = c.phase <= 1 ? 2 : c.phase == 2 ? 3 : 4;
        const int delta = 1 + std::min(most - 1, static_cast<int>(u(w.rng) * static_cast<float>(most)));
        c.key = (c.key + (u(w.rng) < 0.5f ? delta : -delta) + 8) % 8;
    }
    // O mouse do jogador emulado: a cada 0,8-2,5 s, parado (50%), panoramica (35%, ate PanMost da fase, por 0,5-2 s) ou, da fase
    // 3, um flick (15%, 45-120 graus de uma vez).
    void commandMouse(World& w) {
        World::Command& c = w.command;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float dt = TicksPerDecision * TickSeconds;
        static constexpr float PanMost[4] { 0.8f, 1.5f, 2.0f, 2.5f };
        const std::size_t p = static_cast<std::size_t>(std::clamp(c.phase, 1, 4) - 1);
        if (w.time >= c.nextMouseAt) {
            c.nextMouseAt = w.time + 0.8f + 1.7f * u(w.rng);
            const float r = u(w.rng);
            const float sign = u(w.rng) < 0.5f ? 1.0f : -1.0f;
            c.panRate = 0.0f;
            c.panUntil = -1.0f;
            if (r < 0.35f) {
                c.panRate = PanMost[p] * (0.25f + 0.75f * u(w.rng)) * sign;
                c.panUntil = w.time + 0.5f + 1.5f * u(w.rng);
            } else if (r < 0.5f && c.phase >= 3) {
                c.target = wrapAngle(c.target + (0.8f + 1.3f * u(w.rng)) * sign);
            }
        }
        if (c.panUntil >= w.time) c.target = wrapAngle(c.target + c.panRate * dt);
    }
    // Arrancar do idle no rumo e na faixa: o passo da faixa no rumo, a fase do ciclo do comeco, a rampa da largada, a mistura do
    // quadro de agora.
    bool commandStart(World& w, std::uint32_t band, int key) {
        const Gait* g = gaitFor(band, static_cast<std::uint32_t>(key));
        if (g == nullptr) return false;
        blendFromNow(w, CommandStartBlend);
        const float clipDirection = travel[g->clip].direction;
        w.band = band;
        w.runBand = band >= 2;
        w.heading = static_cast<std::uint32_t>(key);
        w.trackTwist = wrapAngle(clipDirection - 0.25f * Pi * static_cast<float>(key));
        w.twistClip = g->clip;
        w.trackClip = g->clip;
        w.trackScale = 1.0f;
        const float fixed = fixedSpeed(band, w.heading) * w.speedScale;
        w.commandSpeed = fixed > 0.0f ? fixed : std::max(0.5f, travel[g->clip].speed);
        w.rampFrom = w.time;
        w.rampSeconds = std::max(StartRampSeconds, w.commandSpeed / StartAcceleration);
        w.launchEnd = w.time + w.rampSeconds;
        w.trackStart = -w.time;
        w.trackYaw = wrapAngle(w.faceYaw + 0.25f * Pi * static_cast<float>(key) - clipDirection);
        w.moveYaw = wrapAngle(w.trackYaw + clipDirection);
        w.improviseUntil = w.time + CommandStartBlend + ImproviseAfterSeconds;
        return true;
    }
    // Parar: o idle em alerta no rumo da vista, misturado a partir do passo.
    // (10/11, settings.basePosture) O parado de uma parada dos retos: o idle alerta no rumo da vista, como o do comando.
    void parkStop(World& w) {
        w.freeStop = false;
        blendFromNow(w, CommandIdleBlend);
        // (O idle alerta mais perto da base de agora, o normal ou o espelho, os dois de frente para a vista: o menor numero de passos.)
        std::size_t idle = standingIndex;
        if (standingMirrorIndex < references.size() && standingMirrorIndex != standingIndex) {
            const auto near = [&](std::size_t clip) {
                const Vec3 root = w.body->links[rootLink].position;
                references[clip]->frame(0.0f, { root.x, root.y, floorUnder(w) }, w.faceYaw, w.baseFrame);
                return measureTrackingBasePosture3D(*profile, w.body->links, w.baseFrame.links, rootLink, upperChestLink,
                    { footLinks[0], footLinks[1] }).distance();
            };
            if (near(standingMirrorIndex) < near(standingIndex)) idle = standingMirrorIndex;
        }
        w.trackClip = idle;
        w.trackScale = 1.0f;
        w.trackStart = -w.time;
        w.rampFrom = -1.0f;
        w.trackYaw = w.faceYaw;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackTwist = 0.0f;
        w.commandSpeed = 0.0f;
        w.improviseUntil = w.time + CommandIdleBlend + ImproviseAfterSeconds;
        w.stopPhase = 2;
        w.idleSince = w.time;
    }
    void commandIdle(World& w) {
        blendFromNow(w, CommandIdleBlend);
        w.trackClip = standingIndex;
        w.trackScale = 1.0f;
        w.trackStart = -w.time;
        w.rampFrom = -1.0f;
        w.trackYaw = w.faceYaw;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackTwist = 0.0f;
        w.commandSpeed = 0.0f;
        w.improviseUntil = w.time + CommandIdleBlend + ImproviseAfterSeconds;
        w.command.state = 2;
        ++w.command.stops;
        w.stopPhase = 2;
        w.idleSince = w.time;
    }
    // (08/11 15h20, ele: "os nossos exercicios nao treinam PARAR, o frear... em todos os exercicios de locomocao basica
    // direcional, ao final da rodada a PARADA... recompensado pela precisao da parada, responsividade, algo mais imediato, mas
    // mantendo bom equilibrio... a rodada nao acaba com ele correndo para o infinito: a arrancada, o movimento, a parada") A
    // parada: o pedido some (zerado na observacao); a referencia freia a stopDeceleration (no comando, a da manobra) a partir da
    // velocidade de agora e, abaixo de CommandStopSpeed, passa ao idle em alerta; o ponto de parada de referencia e a freada
    // perfeita integrada desde a pelve no comeco dela - a nota vale 1 - StopPrecisionWeight + StopPrecisionWeight exp(-(d/
    // StopPrecisionWidth)^2) dali ate o fim.
    static constexpr float StopPrecisionWeight = 0.4f, StopPrecisionWidth = 0.5f;
    void beginStop(World& w) {
        if (!w.refActive) {
            w.refVelocity = wantedVelocity(w, 0.0f);
            w.refAccel = {};
            w.refActive = true;
        }
        w.stopping = true;
        w.stopPhase = 1;
        w.stopBeganAt = w.time;
        w.stopFromSpeed = std::hypot(w.refVelocity.x, w.refVelocity.y);
        w.stopFromScale = w.trackScale;
        const Vec3 pelvis = w.body->links[rootLink].position;
        w.stopRef = { pelvis.x, pelvis.y, 0.0f };
        w.improviseUntil = w.time + ImproviseAfterSeconds;
    }
    // A parada do fim do reto a cada decisao: comeca na hora, freia a referencia (nos retos; o comando freia pela manobra), passa
    // ao idle e integra o ponto de referencia.
    // (08/11 16h20, setStopClips) As paradas da Unreal: 8 rumos x 2 pes, no trote e na corrida, no fim da lista (nenhum indice
    // muda). A velocidade da raiz de cada uma, suavizada, a cada StopSample s; a de entrada, o rumo dela e quando assenta.
    // (10/11 04h55) Uma parada (a de nome `name`, no fim da lista): a velocidade da raiz suavizada, no lugar, a entrada e quando
    // assenta; devolve a posicao em stopClipInfo.
    std::size_t loadStopClip(const std::string& name) {
        clips.push_back(std::make_unique<AnimationClip3D>(
            loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json")));
        references.push_back(std::make_unique<TrackingClipReference3D>(*profile, *clips.back()));
        clipNames.push_back(name);
        travel.emplace_back();
        StopClip c;
        c.clip = references.size() - 1;
        const float duration = references.back()->durationSeconds();
        std::vector<Vec3> root;
        TrackingClipFrame3D frame;
        for (std::size_t k = 0; static_cast<float>(k) * StopSample <= duration; ++k) {
            references.back()->frame(static_cast<float>(k) * StopSample, {}, 0.0f, frame);
            root.push_back(frame.rootPosition);
        }
        const std::size_t n = root.size(), half = 7;
        if (n < 3) throw std::runtime_error("rastreador: parada curta demais: " + name);
        c.velocity.resize(n);
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t a = k > half ? k - half : 0, b = std::min(n - 1, k + half);
            const Vec3 d = root[b] - root[a];
            const float span = static_cast<float>(b - a) * StopSample;
            c.velocity[k] = { d.x / span, d.y / span, 0.0f };
        }
        // (16h40: no lugar, como os ciclos - a raiz da Unreal anda 2,5-4,5 m e a referencia ia junto, longe do corpo: a
        // folha mostrou o azul 1-2 m atras e o corpo caindo em 0,6 s. O deslocamento fica so na velocidade de referencia.)
        for (AnimationTrack3D& track : clips.back()->tracks)
            if (track.space == AnimationTrackSpace3D::Root)
                for (AnimationKeyframe3D& key : track.keyframes) key.translationOffsetMeters.x = key.translationOffsetMeters.y = 0.0f;
        const std::size_t e = std::clamp<std::size_t>(static_cast<std::size_t>(0.2f / StopSample), 1, n - 1);
        const Vec3 d0 = root[e] - root[0];
        c.entrySpeed = std::hypot(d0.x, d0.y) / (static_cast<float>(e) * StopSample);
        c.direction = std::atan2(d0.y, d0.x);
        c.settle = duration;
        for (std::size_t k = e; k < n; ++k)
            if (std::hypot(c.velocity[k].x, c.velocity[k].y) < 0.2f) {
                c.settle = static_cast<float>(k) * StopSample;
                break;
            }
        stopClipInfo.push_back(std::move(c));
        return stopClipInfo.size() - 1;
    }
    void loadStopClips() {
        static constexpr const char* Codes[8] { "f", "fl", "ll", "bl", "b", "br", "rr", "fr" };
        for (auto& band : stopSlots)
            for (auto& heading : band) heading.fill(std::numeric_limits<std::size_t>::max());
        for (std::size_t band = 0; band < 2; ++band)
            for (std::size_t h = 0; h < 8; ++h)
                for (std::size_t foot = 0; foot < 2; ++foot) {
                    const std::string name = std::string("referencias_unreal/") + (band ? "corrida_stop_" : "trote_run_stop_")
                        + Codes[h] + (foot ? "_rfoot" : "_lfoot");
                    stopSlots[band][h][foot] = loadStopClip(name);
                }
    }
    // (10/11 04h55, settings.stopClipsMixamo; ele: "reduzir a velocidade e inclinar o corpo todo para tras", com a "Standing Torch
    // Run Forward Stop" da Mixamo sem a mao erguida) A parada de frente, no trote e na corrida, pela tocha e o espelho dela.
    // (Ele, 05h: "ve qual dos tres fica melhor") As tres paradas de frente lado a lado: 0 a da Unreal, 1 a tocha, 2 a "Running To
    // Turn" sem o giro (cada uma com o espelho, o outro pe); no treino sorteada a cada parada, na medida a de stopFrontVariant.
    std::array<std::array<std::array<std::size_t, 2>, 4>, 2> frontStopVariants {};
    bool mixamoStopsLoaded = false;
    void loadMixamoStops() {
        if (mixamoStopsLoaded) return;
        if (stopClipInfo.empty()) loadStopClips();
        const std::size_t ta = loadStopClip("referencias_mixamo/parada_tocha_frente");
        const std::size_t tb = loadStopClip("referencias_mixamo/parada_tocha_frente_espelho");
        const std::size_t va = loadStopClip("referencias_mixamo/parada_virada_frente");
        const std::size_t vb = loadStopClip("referencias_mixamo/parada_virada_frente_espelho");
        // (05h10, ele: "Running To Stop... MUITO bom tambem, so ignorar os bracos (o rifle)": os bracos da tocha e sem o giro do fim.)
        const std::size_t ra = loadStopClip("referencias_mixamo/parada_rifle_frente");
        const std::size_t rb = loadStopClip("referencias_mixamo/parada_rifle_frente_espelho");
        for (std::size_t band = 0; band < 2; ++band) {
            frontStopVariants[band][0] = stopSlots[band][0];
            frontStopVariants[band][1] = { ta, tb };
            frontStopVariants[band][2] = { va, vb };
            frontStopVariants[band][3] = { ra, rb };
        }
        mixamoStopsLoaded = true;
    }
    // A entrada na parada: a do rumo e da faixa de agora, o pe e o instante (nos primeiros StopEntryWindow s dela) com as pernas
    // mais perto das do passo de agora; o quadro posto no rumo do deslocamento; a velocidade dela escalada a de agora. Falso
    // sem parada carregada para o caso (fica a freada da referencia).
    bool enterStopClip(World& w) {
        if (!settings.stopClips || stopClipInfo.empty() || w.trackClip >= references.size() || w.band < 1 || starting(w))
            return false;
        const std::size_t band = w.band >= 2 ? 1 : 0, h = w.heading % 8u;
        // (17h25: no trote de lado, nao - a parada lateral da Unreal sai de 3,9 m/s com uma passada larga e o lateral do
        // trote anda a 1,1-1,8 m/s: a sonda (r60400) caiu em 12 de 12 a esquerda e 10 de 12 a direita, contra 1 de 12 com a
        // freada antiga; e ele ficou devagar em todo lateral para nao cair nela - 70 -> 51% da meta em 1 h.)
        // (17h55: e na corrida so para a frente - a sonda r60800 com os clipes: costas 11, 11 e 8 quedas em 12 (a freada antiga
        // 2, 3 e 4), a lateral direita 6 e a diagonal frente-direita 7 (a esquerda delas 0); simetrico: o par inteiro fica na
        // freada antiga. O teste fixo da parada na corrida caiu de 96 a 54% de pe em 30 min.)
        static constexpr bool Use[2][8] { { true, true, false, true, true, true, false, true },
                                          { true, false, false, false, false, false, false, false } };
        if (!Use[band][h]) return false;
        // (09/11 08h55, settings.stopClipsFront: so de frente e nas diagonais da frente do trote (e a corrida de frente) - a freada
        // pela referencia que desacelera a passada de corrida nunca mostra como frear; de frente os clipes ensinaram em 08/11
        // (trote 46 -> 67%, corrida 58 -> 83% de pe em ~350 atualizacoes), de costas derrubaram.)
        if (settings.stopClipsFront && !(h == 0u || h == 1u || h == 7u)) return false;
        TrackingClipFrame3D current, candidate;
        references[w.trackClip]->frame(trackSample(w, 0.0f).first, {}, 0.0f, current);
        std::size_t pick = std::numeric_limits<std::size_t>::max();
        float entry = 0.0f, best = std::numeric_limits<float>::max();
        std::array<std::size_t, 2> slots = stopSlots[band][h];
        if (h == 0u && settings.stopFrontVariants && mixamoStopsLoaded) {
            std::uniform_real_distribution<float> draw(0.0f, 1.0f);
            // (O sorteio entre as variantes da mascara; a medida fixa uma.)
            int v = std::min(settings.stopFrontVariant, 3);
            if (v < 0) {
                int active[4], count = 0;
                for (int k = 0; k < 4; ++k)
                    if (settings.stopFrontVariantMask & (1 << k)) active[count++] = k;
                v = count > 0 ? active[std::min(count - 1, static_cast<int>(static_cast<float>(count) * draw(w.rng)))] : 0;
            }
            slots = frontStopVariants[band][static_cast<std::size_t>(v)];
        }
        for (std::size_t foot = 0; foot < 2; ++foot) {
            const std::size_t slot = slots[foot];
            if (slot >= stopClipInfo.size()) continue;
            const TrackingClipReference3D& r = *references[stopClipInfo[slot].clip];
            for (int k = 0; static_cast<float>(k) / 30.0f <= settings.stopEntryWindow; ++k) {
                r.frame(static_cast<float>(k) / 30.0f, {}, 0.0f, candidate);
                float d = 0.0f;
                for (std::size_t i = 0; i < current.coordinates.size() && i < candidate.coordinates.size() && i < trackingParts.size(); ++i)
                    if (trackingParts[i] == PartLeftLeg || trackingParts[i] == PartRightLeg)
                        d += (candidate.coordinates[i] - current.coordinates[i]).lengthSquared();
                if (d < best) {
                    best = d;
                    pick = slot;
                    entry = static_cast<float>(k) / 30.0f;
                }
            }
        }
        if (pick >= stopClipInfo.size()) return false;
        const StopClip& c = stopClipInfo[pick];
        const float speed = std::hypot(w.refVelocity.x, w.refVelocity.y);
        const float along = speed > 0.3f ? std::atan2(w.refVelocity.y, w.refVelocity.x) : w.moveYaw;
        // (10/11 05h25, settings.stopEntryBlend: a entrada gradual - a passada se mistura na parada por este tempo.)
        blendFromNow(w, settings.stopEntryBlend);
        w.stopClip = static_cast<int>(pick);
        w.stopFrom = w.time;
        w.stopEntry = entry;
        w.stopScale = std::clamp(speed / std::max(0.5f, c.entrySpeed), 0.3f, 1.3f);
        w.trackClip = c.clip;
        w.trackScale = 1.0f;
        w.trackStart = entry - w.time;
        w.rampFrom = -1.0f;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackTwist = 0.0f;
        w.trackYaw = along - c.direction;
        return true;
    }
    // (10/11 04h, settings.unrealStarts; ele: "a arrancada da corrida que e estranha, deveria ser a mesma arrancada igual do unreal
    // engine") As arrancadas da corrida da Unreal, carregadas uma vez, no lugar (a raiz so na velocidade de referencia, como as paradas).
    void loadUnrealStarts() {
        static constexpr const char* Codes[8] { "f", "fl", "ll", "bl", "b", "br", "rr", "fr" };
        for (auto& band : startSlots)
            for (auto& heading : band) heading.fill(std::numeric_limits<std::size_t>::max());
        // (07h45, ele: "claro que ha clipe de largada da lateral... voce que nao pesquisou": cada rumo pelo seu. O trote pelo
        // trote_run_start; a corrida pelo corrida_start (6 / 5,5 m/s) e, de lado (3,5 m/s pedidos), pelo run_start, mais lento.)
        for (std::size_t band = 0; band < 2; ++band)
        for (std::size_t h = 0; h < 8; ++h)
            for (std::size_t foot = 0; foot < 2; ++foot) {
                const char* family = band == 0 ? "trote_run_start_" : (h == 2 || h == 6) ? "run_start_" : "corrida_start_";
                const std::string name = std::string("referencias_unreal/") + family + Codes[h] + (foot ? "_rfoot" : "_lfoot");
                clips.push_back(std::make_unique<AnimationClip3D>(
                    loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json")));
                references.push_back(std::make_unique<TrackingClipReference3D>(*profile, *clips.back()));
                clipNames.push_back(name);
                travel.emplace_back();
                UnrealStart c;
                c.clip = references.size() - 1;
                const float duration = references.back()->durationSeconds();
                std::vector<Vec3> root;
                TrackingClipFrame3D frame;
                for (std::size_t k = 0; static_cast<float>(k) * StopSample <= duration; ++k) {
                    references.back()->frame(static_cast<float>(k) * StopSample, {}, 0.0f, frame);
                    root.push_back(frame.rootPosition);
                }
                const std::size_t n = root.size(), half = 7, tail = static_cast<std::size_t>(0.5f / StopSample);
                if (n < 2 * tail + static_cast<std::size_t>(StartExitMinimum / StopSample))
                    throw std::runtime_error("rastreador: arrancada curta demais: " + name);
                c.velocity.resize(n);
                for (std::size_t k = 0; k < n; ++k) {
                    const std::size_t a = k > half ? k - half : 0, b = std::min(n - 1, k + half);
                    const Vec3 d = root[b] - root[a];
                    const float span = static_cast<float>(b - a) * StopSample;
                    c.velocity[k] = { d.x / span, d.y / span, 0.0f };
                }
                for (AnimationTrack3D& track : clips.back()->tracks)
                    if (track.space == AnimationTrackSpace3D::Root)
                        for (AnimationKeyframe3D& key : track.keyframes) key.translationOffsetMeters.x = key.translationOffsetMeters.y = 0.0f;
                const Vec3 total = root.back() - root.front();
                c.direction = std::atan2(total.y, total.x);
                float cruise = 0.0f;
                for (std::size_t k = n - tail; k < n; ++k) cruise += std::hypot(c.velocity[k].x, c.velocity[k].y);
                cruise /= static_cast<float>(tail);
                c.exit = static_cast<float>(n - tail) * StopSample;
                for (std::size_t k = static_cast<std::size_t>(StartExitMinimum / StopSample); k < n - tail; ++k)
                    if (std::hypot(c.velocity[k].x, c.velocity[k].y) >= StartExitFraction * cruise) {
                        c.exit = static_cast<float>(k) * StopSample;
                        break;
                    }
                const Vec3 v = c.velocity[std::min(n - 1, static_cast<std::size_t>(c.exit / StopSample))];
                c.exitSpeed = std::hypot(v.x, v.y);
                startSlots[band][h][foot] = startInfo.size();
                startInfo.push_back(std::move(c));
            }
    }
    // A entrada na arrancada da Unreal (de pe, nos retos da corrida): a do rumo, o pe com as pernas do primeiro quadro mais perto das
    // do idle; tocada na escala que leva a velocidade de saida dela a pedida; o quadro no rumo do deslocamento, sem a cintura (o clipe
    // ja olha para a vista nas diagonais). O ciclo vem depois, pela emenda (pendingClip).
    bool enterUnrealStart(World& w) {
        if (!settings.unrealStarts || startInfo.empty() || w.trackClip >= references.size() || standingIndex >= references.size())
            return false;
        const std::size_t h = w.heading % 8u, band = w.band >= 2 ? 1 : 0;
        TrackingClipFrame3D idle, candidate;
        references[standingIndex]->frame(0.0f, {}, 0.0f, idle);
        std::size_t pick = std::numeric_limits<std::size_t>::max();
        float best = std::numeric_limits<float>::max();
        for (std::size_t foot = 0; foot < 2; ++foot) {
            const std::size_t slot = startSlots[band][h][foot];
            if (slot >= startInfo.size()) continue;
            references[startInfo[slot].clip]->frame(0.0f, {}, 0.0f, candidate);
            float d = 0.0f;
            for (std::size_t i = 0; i < idle.coordinates.size() && i < candidate.coordinates.size() && i < trackingParts.size(); ++i)
                if (trackingParts[i] == PartLeftLeg || trackingParts[i] == PartRightLeg)
                    d += (candidate.coordinates[i] - idle.coordinates[i]).lengthSquared();
            if (d < best) {
                best = d;
                pick = slot;
            }
        }
        if (pick >= startInfo.size()) return false;
        const UnrealStart& c = startInfo[pick];
        w.unrealStart = static_cast<int>(pick);
        w.startCycleYaw = w.trackYaw;
        w.startCycleTwist = w.trackTwist;
        w.startCycleTwistClip = w.twistClip;
        w.pendingClip = w.trackClip;
        w.trackClip = c.clip;
        // (10/11 06h05, settings.unrealStartScale > 0: a escala fixa - a 1,1x da velocidade pedida a arrancada ia de 0 a 6 m/s em ~1 s e
        // a rede balancava 51 graus; a 0,7, como a largada aprovada em 06/11, e dali a rampa de runLaunchAcceleration ate a pedida.)
        w.trackScale = settings.unrealStartScale > 0.0f ? settings.unrealStartScale
            : std::clamp(w.commandSpeed / std::max(0.5f, c.exitSpeed), StartClipScaleMinimum, StartClipScaleMaximum);
        w.trackStart = -w.time * w.trackScale;
        w.rampFrom = -1.0f;
        w.trackYaw = w.moveYaw - c.direction;
        w.trackTwist = 0.0f;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        const float rise = settings.runLaunchAcceleration > 0.0f ? settings.runLaunchAcceleration : StartAcceleration;
        w.launchEnd = c.exit / w.trackScale + std::max(0.0f, w.commandSpeed - w.trackScale * c.exitSpeed) / rise;
        return true;
    }
    // (09/11 15h, settings.freeTroca) A troca livre: o pedido novo na hora - a faixa, o rumo em relacao a vista e, na meia-volta, a
    // vista girando a FreeTrocaViewRate ate a nova -, a janela pelo tamanho da troca (0,8 s + o erro de velocidade a
    // FreeTrocaAccel, entre FreeTrocaMinimum e FreeTrocaMaximum); o clipe do rumo novo so para depois dela.
    static constexpr float FreeTrocaAccel = 4.0f, FreeTrocaMinimum = 1.2f, FreeTrocaMaximum = 3.5f, FreeTrocaViewRate = 4.0f;
    bool startFreeTroca(World& w, std::uint32_t band, std::uint32_t heading, float viewTurn) {
        const Gait* g = gaitFor(band, heading);
        if (g == nullptr) return false;
        w.freeStop = false;
        w.stopping = false;
        w.stopPhase = 0;
        w.stopClip = -1;
        w.brakeTurnAt = -1.0f;
        w.runAgainAt = -1.0f;
        w.pendingClip = std::numeric_limits<std::size_t>::max();
        w.band = band;
        w.runBand = band >= 2;
        w.heading = heading;
        const float clipSpeed = std::max(0.1f, travel[g->clip].speed), clipDirection = travel[g->clip].direction;
        const float fixed = fixedSpeed(band, heading) * w.speedScale;
        if (fixed > 0.0f) w.commandSpeed = fixed;
        w.trackClip = g->clip;
        w.trackScale = fixed > 0.0f ? 1.0f : w.commandSpeed / clipSpeed;
        w.rampFrom = -1.0f;
        w.accelFrom = -1.0f;
        w.trackStart = -w.time * w.trackScale;
        w.trackTwist = wrapAngle(clipDirection - 0.25f * Pi * static_cast<float>(heading));
        w.twistClip = g->clip;
        w.trackYaw = w.faceYaw + 0.25f * Pi * static_cast<float>(heading) - clipDirection;
        w.moveYaw = w.faceYaw + 0.25f * Pi * static_cast<float>(heading);
        w.freeTrocaView = wrapAngle(w.faceYaw + viewTurn);
        // (A troca livre de exercicio - a manobra 4 - gira a vista ate viewTarget depois; ja la, ela para.)
        w.viewTarget = w.freeTrocaView;
        const float move = w.freeTrocaView + 0.25f * Pi * static_cast<float>(heading);
        const float e0 = std::hypot(w.smoothVelocity.x - std::cos(move) * w.commandSpeed, w.smoothVelocity.y - std::sin(move) * w.commandSpeed);
        w.freeTrocaError = std::max(1.0f, e0);
        w.freeTrocaUntil = w.time + std::clamp(0.8f + e0 / FreeTrocaAccel, FreeTrocaMinimum, FreeTrocaMaximum);
        w.refActive = true;
        w.refVelocity = w.smoothVelocity;
        w.freeTroca = true;
        return true;
    }
    // A janela da troca livre a cada decisao: a vista indo a da meia-volta (fora do comando, que tem o mouse dele), a velocidade de
    // referencia com o corpo (o rastreio volta dela) e, no fim, o rastreio.
    void updateFreeTroca(World& w, float dt) {
        if (!isCommand(w)) {
            const float step = std::clamp(wrapAngle(w.freeTrocaView - w.faceYaw), -FreeTrocaViewRate * dt, FreeTrocaViewRate * dt);
            w.arcRate = 0.0f;
            if (std::abs(step) > 1.0e-5f) {
                w.faceYaw = wrapAngle(w.faceYaw + step);
                w.trackYaw = wrapAngle(w.trackYaw + step);
                w.moveYaw = wrapAngle(w.moveYaw + step);
                w.arcRate = step / dt;
            }
        }
        w.refVelocity = w.smoothVelocity;
        if (w.time >= w.freeTrocaUntil) resumeTracking(w);
    }
    // O fim da troca livre: o ciclo do rumo novo no quadro com as pernas mais parecidas com as do corpo agora (como a
    // sincronizacao da fase), a janela de improviso das trocas a partir dai.
    void resumeTracking(World& w) {
        w.freeTroca = false;
        if (w.trackClip >= references.size()) return;
        const TrackingClipReference3D& reference = *references[w.trackClip];
        const float duration = std::max(1.0e-3f, reference.durationSeconds());
        const Vec3 root = w.body->links[rootLink].position;
        float best = 1.0e30f;
        int bestFrame = 0;
        for (int k = 0; k < PhaseSyncFrames; ++k) {
            reference.frame(duration * static_cast<float>(k) / PhaseSyncFrames, { root.x, root.y, floorUnder(w) }, w.trackYaw, w.altFrame,
                twist(w), pitch(w), legPitch(w));
            w.altErrors.assign(w.jointErrors.size(), 0.0f);
            contract->jointErrors(w.body->links, w.altFrame, w.altErrors);
            float legs = 0.0f;
            for (std::size_t q = 0; q < trackingParts.size() && q < w.altErrors.size(); ++q)
                if (trackingParts[q] == PartLeftLeg || trackingParts[q] == PartRightLeg) legs += w.altErrors[q];
            if (legs < best) {
                best = legs;
                bestFrame = k;
            }
        }
        w.rampFrom = -1.0f;
        w.trackStart = duration * static_cast<float>(bestFrame) / PhaseSyncFrames - w.time * w.trackScale;
        w.improviseUntil = w.time + ImproviseAfterSeconds;
    }
    // (09/11 14h, settings.freeStop) A parada livre: o pedido some (zero na observacao, como no jogo) e a referencia nao freia - o
    // idle e so a pose de chegada, o termo leve da nota (freeStopScore). Nos retos ate o fim do episodio; no comando, ate a proxima
    // tecla (o giro no lugar segue o clipe dele).
    void startFreeStop(World& w) {
        w.freeStop = true;
        w.freeTroca = false;
        // (Sem a troca pendente da arrancada: ela devolveria o clipe da passada no fim do clipe de agora.)
        w.pendingClip = std::numeric_limits<std::size_t>::max();
        w.stopClip = -1;
        w.freeFromSpeed = std::max(1.0f, std::hypot(w.smoothVelocity.x, w.smoothVelocity.y));
        w.refVelocity = {};
        w.trackClip = standingIndex;
        w.trackScale = 1.0f;
        w.trackStart = -w.time;
        w.rampFrom = -1.0f;
        w.trackYaw = w.faceYaw;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackTwist = 0.0f;
    }
    bool freeStopNow(const World& w) const { return w.freeStop && w.stopping && !(isCommand(w) && w.command.state == 3); }
    void advanceStop(World& w) {
        const float dt = TicksPerDecision * TickSeconds;
        if (w.finalStopAt >= 0.0f && w.time >= w.finalStopAt && !w.stopping && w.maneuver == 0 && w.locomotion
            && w.commandSpeed > 0.0f && !starting(w)) {
            w.finalStopAt = -1.0f;
            beginStop(w);
            // (10/11 04h40, settings.stopClipsBase; ele: "ensinar a fazer pelos clipes, para ter base, e DEPOIS deixar liberdade"):
            // a parada da Unreal do rumo primeiro, onde ha uma boa; sem ela, a livre.
            // (11/11, settings.stopToIdle: direto ao idle alerta - ver o .hpp.)
            if (settings.stopToIdle) parkStop(w);
            else if (settings.stopClipsBase && enterStopClip(w)) {}
            else if (settings.freeStop) startFreeStop(w);
            else enterStopClip(w);
        }
        if (!w.stopping) return;
        // (A parada livre: nada a frear na referencia nem idle pelo relogio.)
        if (w.freeStop) {
            // (10/11, settings.basePosture: nos retos, o corpo freado - abaixo de FreeStopSettleSpeed - entra no parado, como no comando.)
            if ((settings.basePosture > 0.0f || settings.idleAfterStop) && !isCommand(w)
                && std::hypot(w.smoothVelocity.x, w.smoothVelocity.y) < FreeStopSettleSpeed)
                parkStop(w);
            return;
        }
        if (w.maneuver == 0) {
            bool settled = false;
            if (w.stopClip >= 0) {
                // (A velocidade de referencia e a da raiz da parada, no rumo do quadro, na escala da entrada; assentou quando a
                // raiz dela para - mais StopSettleExtra do assentar - ou no fim dela.)
                const StopClip& c = stopClipInfo[static_cast<std::size_t>(w.stopClip)];
                const float t = w.stopEntry + (w.time - w.stopFrom);
                const Vec3 v = c.velocity[std::min(c.velocity.size() - 1, static_cast<std::size_t>(std::max(0.0f, t) / StopSample))];
                const float cy = std::cos(w.trackYaw), sy = std::sin(w.trackYaw);
                w.refVelocity = Vec3 { (cy * v.x - sy * v.y) * w.stopScale, (sy * v.x + cy * v.y) * w.stopScale, 0.0f };
                settled = t >= c.settle + StopSettleExtra || t >= references[c.clip]->durationSeconds() - 0.25f;
            } else {
                const float speed = w.refVelocity.length(), most = settings.stopDeceleration * dt;
                w.refVelocity = speed > most ? w.refVelocity * ((speed - most) / speed) : Vec3 {};
                // (18h25, ele: "confira o porque o boneco esta caindo muito ao tentar frear, tipo trote esquerdo": no lateral do
                // trote, a ~2 m/s, a freada chegava a 1,2 m/s em 0,2 s e a referencia ia ao parado com o corpo ainda correndo de
                // lado - 10 e 12 quedas em 16, 0,8 s depois, tombando para o lado do deslocamento. Agora a passada continua com a
                // cadencia descendo junto com a velocidade - ate StopCadenceFloor -, e o parado so abaixo de StopIdleSpeed.)
                // (So nos laterais - de frente e de costas a rede ja para; la a mudanca derrubou a corrida de costas em 15 de 16.)
                // (09/11 05h, settings.stopGaitAll: a passada que desacelera em todos os rumos - a de 18h27 foi julgada numa
                // sonda da rede sem treino; a medida certa e um A/B treinado.)
                const bool lateral = w.heading == 2u || w.heading == 6u || settings.stopGaitAll;
                if (lateral && w.stopPhase == 1 && w.stopFromSpeed > 0.1f && w.trackClip < references.size() && w.trackClip != standingIndex) {
                    const float scale = w.stopFromScale * std::max(StopCadenceFloor, w.refVelocity.length() / w.stopFromSpeed);
                    const float clipTime = w.trackStart + startRampTime(w, w.time) * w.trackScale;
                    w.rampFrom = -1.0f;
                    w.trackScale = scale;
                    w.trackStart = clipTime - w.time * scale;
                }
                settled = w.refVelocity.length() <= (lateral ? StopIdleSpeed : CommandStopSpeed);
                // (09/11 03h50, ele: "na corrida ele cai muito ao frear ainda... na LOCOMOCAO nao podemos tolerar quedas": medido,
                // a corrida de frente caia 16 de 16 - a referencia ia ao idle 0,93 s depois da ordem com o corpo ainda a 4,0 m/s, e
                // a pose parada com esse embalo tombava 0,8 s depois. Com stopWaitBody o idle so vem com o corpo abaixo de
                // StopBodySpeed (ou StopBodyTimeout depois da ordem); ate la a velocidade pedida segue caindo a zero.)
                if (settings.stopWaitBody && settled && std::hypot(w.smoothVelocity.x, w.smoothVelocity.y) > StopBodySpeed
                    && w.time - w.stopBeganAt < StopBodyTimeout)
                    settled = false;
            }
            if (w.stopPhase == 1 && settled && settings.stopClipsBase && settings.freeStop) {
                // (A base: a parada da Unreal assentou - dali a livre, com a cabeca da locomocao ate ficar de pe e a postura cobrada.)
                blendFromNow(w, CommandIdleBlend);
                startFreeStop(w);
                return;
            }
            if (w.stopPhase == 1 && settled) {
                w.stopClip = -1;
                blendFromNow(w, CommandIdleBlend);
                // (08/11 21h45: o idle em alerta tem o apoio num pe - sempre o normal, a parada para um dos lados chegava no pe
                // errado; desde a troca para ele (18h27) o lateral direito caiu de 60 para ~35% da meta e o esquerdo nao. Agora o
                // normal ou o espelho, o mais perto da pose de agora, como o idle depois de levantar.)
                // (So nos laterais: de frente o espelho derrubou o trote em 7 de 16 depois da parada - a rede so conhecia o normal ali.)
                // (22h25: desligado - em 40 min o lateral esquerdo caiu de 64 para 47% da meta e o direito nao subiu; StopIdleMirror.)
                std::size_t idle = standingIndex;
                if (StopIdleMirror && (w.heading == 2u || w.heading == 6u) && standingMirrorIndex < references.size() && standingMirrorIndex != standingIndex) {
                    references[standingIndex]->frame(0.0f, {}, 0.0f, w.frame);
                    const float normal = poseDistance(*w.body, w.frame);
                    references[standingMirrorIndex]->frame(0.0f, {}, 0.0f, w.frame);
                    if (poseDistance(*w.body, w.frame) < normal) idle = standingMirrorIndex;
                }
                w.trackClip = idle;
                w.trackScale = 1.0f;
                w.trackStart = -w.time;
                w.rampFrom = -1.0f;
                w.trackYaw = idle < yawOffsets.size() && standingIndex < yawOffsets.size()
                    ? wrapAngle(w.faceYaw + yawOffsets[standingIndex] - yawOffsets[idle]) : w.faceYaw;
                w.twistClip = std::numeric_limits<std::size_t>::max();
                w.trackTwist = 0.0f;
                w.commandSpeed = 0.0f;
                w.improviseUntil = w.time + CommandIdleBlend + ImproviseAfterSeconds;
                w.stopPhase = 2;
                w.idleSince = w.time;
            }
        }
        w.stopRef = w.stopRef + Vec3 { w.refVelocity.x * dt, w.refVelocity.y * dt, 0.0f };
        const Vec3 pelvis = w.body->links[rootLink].position;
        w.stopError = std::hypot(pelvis.x - w.stopRef.x, pelvis.y - w.stopRef.y);
    }
    // Parado: a vista longe do peito (mais de CommandTurnStart) gira o corpo no lugar pelo clipe do giro, na velocidade dele,
    // ate a vista; de volta ao idle perto dela.
    void commandTurnInPlace(World& w, float dt) {
        World::Command& c = w.command;
        const float gap = wrapAngle(c.target - w.faceYaw);
        if (c.state == 2) {
            if (std::abs(gap) < CommandTurnStart) return;
            const std::size_t clip = turnIndices[gap > 0.0f ? 0 : 1];
            if (clip >= references.size()) return;
            blendFromNow(w, 0.25f);
            const float cycle = std::abs(references[clip]->clip().turnCycleRadians);
            c.turnRate = (gap > 0.0f ? 1.0f : -1.0f) * cycle / std::max(1.0e-3f, references[clip]->durationSeconds());
            w.trackClip = clip;
            w.trackScale = 1.0f;
            w.trackStart = -w.time;
            w.trackYaw = w.faceYaw;
            w.rampFrom = -1.0f;
            c.state = 3;
            return;
        }
        if (std::abs(gap) < CommandTurnDone || (gap > 0.0f) != (c.turnRate > 0.0f) || std::abs(c.turnRate) < 1.0e-3f) {
            blendFromNow(w, 0.3f);
            w.trackClip = standingIndex;
            w.trackScale = 1.0f;
            w.trackStart = -w.time;
            w.trackYaw = w.faceYaw;
            c.state = 2;
            return;
        }
        const float step = std::clamp(gap, -std::abs(c.turnRate) * dt, std::abs(c.turnRate) * dt);
        w.faceYaw = wrapAngle(w.faceYaw + step);
        w.arcRate = step / dt;
    }
    // Uma decisao do comando: o jogador emulado, a vista (andando, o quadro e o rumo giram juntos ate TurnMost da fase), as teclas
    // (arrancar, trocar o rumo ou a faixa, soltar), a freada ate o idle e o giro no lugar. Devolve a velocidade de referencia.
    Vec3 commandTick(World& w, float dt) {
        World::Command& c = w.command;
        commandKeys(w);
        commandMouse(w);
        static constexpr float TurnMost[4] { 0.8f, 1.5f, 3.0f, 4.0f };
        const float follow = TurnMost[static_cast<std::size_t>(std::clamp(c.phase, 1, 4) - 1)];
        w.arcRate = 0.0f;
        if (c.state <= 1) {
            const float step = std::clamp(wrapAngle(c.target - w.faceYaw), -follow * dt, follow * dt);
            if (std::abs(step) > 1.0e-5f) {
                w.faceYaw = wrapAngle(w.faceYaw + step);
                w.trackYaw = wrapAngle(w.trackYaw + step);
                w.blendYaw = wrapAngle(w.blendYaw + step);
                w.moveYaw = wrapAngle(w.moveYaw + step);
                w.arcRate = step / dt;
            }
        }
        const std::uint32_t band = c.run ? 2u : 1u;
        if (c.key < 0) {
            if (c.state == 0) {
                c.state = 1;
                beginStop(w);
                // (09/11 10h10, settings.stopClipsCommand: o soltar da tecla pelo clipe de parada, como a parada final - 20 das 28
                // falhas do comando vinham logo depois de parar.)
                if (settings.stopToIdle) commandIdle(w);
                else if (settings.freeStop) startFreeStop(w);
                else if (settings.stopClipsCommand) enterStopClip(w);
            }
        } else if (c.state >= 2) {
            if (commandStart(w, band, c.key)) {
                c.state = 0;
                ++c.starts;
                w.stopping = false;
                w.stopPhase = 0;
                w.freeStop = false;
            }
        } else if (c.state == 1 || static_cast<int>(w.heading % 8u) != c.key || w.band != band) {
            // (09/11 15h, settings.freeTroca: a tecla nova - ou a de novo freando, ou o shift no meio da troca livre - e a troca livre.)
            if (settings.freeTroca && (c.state == 1 || static_cast<int>(w.heading % 8u) != c.key || w.freeTroca)
                && startFreeTroca(w, band, static_cast<std::uint32_t>(c.key), 0.0f)) {
                c.state = 0;
            } else if (switchGait(w, band, static_cast<std::uint32_t>(c.key))) {
                c.state = 0;
                w.stopping = false;
                w.stopPhase = 0;
                w.stopClip = -1;
                w.freeStop = false;
            }
        }
        if (c.state == 1 && w.freeStop) {
            // (A parada livre: o parado - o giro no lugar e a tecla de novo - quando o corpo para de verdade; a nota segue a livre.)
            if (std::hypot(w.smoothVelocity.x, w.smoothVelocity.y) < FreeStopSettleSpeed) commandIdle(w);
            return {};
        }
        if (c.state == 1 && w.stopClip >= 0) {
            // (O clipe de parada: a velocidade de referencia e a da raiz dele, como na parada final; assentado, o idle.)
            const StopClip& sc = stopClipInfo[static_cast<std::size_t>(w.stopClip)];
            const float t = w.stopEntry + (w.time - w.stopFrom);
            const Vec3 v = sc.velocity[std::min(sc.velocity.size() - 1, static_cast<std::size_t>(std::max(0.0f, t) / StopSample))];
            const float cy = std::cos(w.trackYaw), sy = std::sin(w.trackYaw);
            w.refVelocity = Vec3 { (cy * v.x - sy * v.y) * w.stopScale, (sy * v.x + cy * v.y) * w.stopScale, 0.0f };
            if (t >= sc.settle + StopSettleExtra || t >= references[sc.clip]->durationSeconds() - 0.25f) {
                w.stopClip = -1;
                commandIdle(w);
                return {};
            }
            return w.refVelocity;
        }
        if (c.state == 1) {
            // (09/11 07h40, settings.stopGaitAll: no soltar da tecla, a mesma passada que desacelera da parada final - a cadencia
            // com a velocidade da referencia, o idle so abaixo de StopIdleSpeed. Medido: 20 das 28 falhas do comando caiam logo
            // depois de parar, o idle chegando com o corpo ainda correndo.)
            if (settings.stopGaitAll && w.stopFromSpeed > 0.1f && w.trackClip < references.size() && w.trackClip != standingIndex) {
                const float scale = w.stopFromScale * std::max(StopCadenceFloor, w.refVelocity.length() / w.stopFromSpeed);
                const float clipTime = w.trackStart + startRampTime(w, w.time) * w.trackScale;
                w.rampFrom = -1.0f;
                w.trackScale = scale;
                w.trackStart = clipTime - w.time * scale;
            }
            if (w.refVelocity.length() <= (settings.stopGaitAll ? StopIdleSpeed : CommandStopSpeed)) commandIdle(w);
            return {};
        }
        if (c.state >= 2) {
            commandTurnInPlace(w, dt);
            return {};
        }
        return { std::cos(w.moveYaw) * w.commandSpeed, std::sin(w.moveYaw) * w.commandSpeed, 0.0f };
    }

    // (08/11, ele: "6 exercicios novos, da estrela de oito pontas: trote peito livre, fixo e girando; corrida peito livre,
    // fixo e girando... garanta bem a boa estruturacao das fases de aprendizado") A estrela: o centro e 8 pontas no raio, nos
    // 8 rumos da vista do comeco (no peito livre, a primeira ponta a frente). Cada perna vai do ponto de agora ao seguinte - do
    // centro a uma ponta; da ponta de volta ao centro (metade) ou a ponta a 90 graus (a corda) -, e a chegada e uma troca de
    // rumo disparada pela geometria (as trocas de agora sao pelo relogio): no peito fixo e no girando, a troca de rumo com o
    // peito na vista (switchGait); no livre, a meia-volta (freia, planta o pe certo, gira) ou, ate 45 graus, a vista girando.
    // No girando a vista gira devagar durante a perna e o rumo em relacao ao peito muda de 45 em 45 graus (a tecla do jogo),
    // pelo ponto. Antes da ponta, a freada ate a velocidade de chegada da fase.
    // As fases (starPhases, o treino sobe cada exercicio): 1 chegar freando - ate StarSlowSpeed em todo ponto, no centro
    // trocas ate 90 graus, a vista girando ate 0,25 rad/s; 2 passar no centro - ate 45 graus direto, os outros freando, no
    // centro ate 135 graus, 0,4 rad/s; 3 frear so o necessario - a chegada pelo angulo da troca, todas as trocas, 0,55 rad/s,
    // 10 m na corrida; 4 a estrela de jogo - a freada minima, 0,7 rad/s.
    static constexpr float IdleRecoverSeconds = 2.0f;
    static constexpr float SlowAfterSeconds = 2.5f, SlowSeconds = 1.0f;
    static constexpr float StarArrival = 0.8f, StarSlowSpeed = 1.5f, StarGraceSeconds = 1.5f, StarKeySlack = 0.1f, StarArrivalSide = 1.5f;
    static constexpr float StarSteerGain = 0.15f, StarSteerMost = 0.17f, StarSnapMost = 0.35f;
    static bool isStar(const World& w) { return w.maneuver >= 8 && w.maneuver <= 10; }
    static bool isCurve(const World& w) { return w.maneuver >= 5 && w.maneuver <= 7; }
    Vec3 starPoint(const World& w, int k) const {
        if (k < 0) return w.star.center;
        const float a = w.star.base + 0.25f * Pi * static_cast<float>(k);
        return { w.star.center.x + std::cos(a) * w.star.radius, w.star.center.y + std::sin(a) * w.star.radius, 0.0f };
    }
    static float planarYaw(Vec3 from, Vec3 to) { return std::atan2(to.y - from.y, to.x - from.x); }
    // (12/11 17h20, settings.starCurveShare) A perna em arco: o circulo pela corda a->b com a flecha |bend| x corda para o lado do sinal
    // (+ a esquerda de a->b), sempre o arco menor (flecha < meia corda). sweep: o angulo varrido de a a b (+ anti-horario).
    struct StarArc { Vec3 center; float radius = 0.0f, start = 0.0f, sweep = 0.0f; bool valid = false; };
    static StarArc starArc(Vec3 a, Vec3 b, float bend) {
        StarArc arc;
        const float dx = b.x - a.x, dy = b.y - a.y, chord = std::hypot(dx, dy);
        if (std::abs(bend) < 1.0e-3f || chord < 0.5f) return arc;
        const float h = std::min(0.49f, std::abs(bend)) * chord, side = bend > 0.0f ? 1.0f : -1.0f;
        const float radius = (0.25f * chord * chord + h * h) / (2.0f * h);
        const Vec3 left { -dy / chord, dx / chord, 0.0f };
        arc.center = Vec3 { 0.5f * (a.x + b.x), 0.5f * (a.y + b.y), 0.0f } - left * (side * (radius - h));
        arc.radius = radius;
        arc.start = std::atan2(a.y - arc.center.y, a.x - arc.center.x);
        arc.sweep = wrapAngle(std::atan2(b.y - arc.center.y, b.x - arc.center.x) - arc.start);
        arc.valid = std::abs(arc.sweep) > 1.0e-3f;
        return arc;
    }
    static float arcTangent(const StarArc& arc, float angle) { return wrapAngle(angle + (arc.sweep > 0.0f ? 0.5f : -0.5f) * Pi); }
    // O rumo da perna na saida e na chegada (rad): a corda nas retas, as tangentes do arco nas curvas.
    static std::pair<float, float> starLegYaws(Vec3 a, Vec3 b, float bend) {
        const StarArc arc = starArc(a, b, bend);
        if (!arc.valid) return { planarYaw(a, b), planarYaw(a, b) };
        return { arcTangent(arc, arc.start), arcTangent(arc, arc.start + arc.sweep) };
    }
    // O corpo contra a perna de agora: o percorrido e o que falta (m, pelo caminho), o lado (+ a esquerda do caminho), a distancia ao
    // caminho e o rumo dele ali (rad).
    struct StarLeg { float along = 0.0f, remain = 0.0f, side = 0.0f, distance = 0.0f, tangent = 0.0f, length = 0.0f; };
    StarLeg starLeg(const World& w, Vec3 p) const {
        const World::Star& s = w.star;
        const Vec3 a = s.from, b = starPoint(w, s.goal);
        StarLeg out;
        const StarArc arc = starArc(a, b, s.bend);
        if (!arc.valid) {
            const float dx = b.x - a.x, dy = b.y - a.y, length = std::max(1.0e-3f, std::hypot(dx, dy));
            const float ux = dx / length, uy = dy / length, rx = p.x - a.x, ry = p.y - a.y;
            out.length = length;
            out.along = rx * ux + ry * uy;
            out.remain = length - out.along;
            out.side = ux * ry - uy * rx;
            out.distance = out.along < 0.0f ? std::hypot(rx, ry) : std::abs(out.side);
            out.tangent = std::atan2(uy, ux);
            return out;
        }
        const float span = std::abs(arc.sweep) * arc.radius;
        const float theta = std::atan2(p.y - arc.center.y, p.x - arc.center.x);
        const float r = std::hypot(p.x - arc.center.x, p.y - arc.center.y);
        const float t = wrapAngle(theta - arc.start) / arc.sweep;
        out.length = span;
        out.along = t * span;
        out.remain = span - out.along;
        out.side = arc.sweep > 0.0f ? arc.radius - r : r - arc.radius;
        if (t < 0.0f) {
            out.distance = std::hypot(p.x - a.x, p.y - a.y);
            out.tangent = arcTangent(arc, arc.start);
        } else if (t > 1.0f) {
            out.distance = std::hypot(p.x - b.x, p.y - b.y);
            out.tangent = arcTangent(arc, arc.start + arc.sweep);
        } else {
            out.distance = std::abs(r - arc.radius);
            out.tangent = arcTangent(arc, theta);
        }
        return out;
    }
    // O ponto do caminho a `at` m do comeco da perna (alem do fim, na reta da chegada).
    Vec3 starAim(const World& w, float at) const {
        const World::Star& s = w.star;
        const Vec3 a = s.from, b = starPoint(w, s.goal);
        const StarArc arc = starArc(a, b, s.bend);
        if (!arc.valid) {
            const float dx = b.x - a.x, dy = b.y - a.y, length = std::max(1.0e-3f, std::hypot(dx, dy));
            return { a.x + dx / length * at, a.y + dy / length * at, 0.0f };
        }
        const float span = std::abs(arc.sweep) * arc.radius;
        if (at > span) {
            const float yaw = arcTangent(arc, arc.start + arc.sweep);
            return { b.x + std::cos(yaw) * (at - span), b.y + std::sin(yaw) * (at - span), 0.0f };
        }
        const float angle = arc.start + arc.sweep * std::max(0.0f, at) / span;
        return { arc.center.x + std::cos(angle) * arc.radius, arc.center.y + std::sin(angle) * arc.radius, 0.0f };
    }
    // Os pontos de uma perna para o laboratorio (a reta: as 2 pontas; o arco: 24 trechos).
    static void starPath(Vec3 a, Vec3 b, float bend, std::vector<Vec3>& out) {
        out.clear();
        const StarArc arc = starArc(a, b, bend);
        if (!arc.valid) {
            out = { { a.x, a.y, 0.0f }, { b.x, b.y, 0.0f } };
            return;
        }
        for (int k = 0; k <= 24; ++k) {
            const float angle = arc.start + arc.sweep * static_cast<float>(k) / 24.0f;
            out.push_back({ arc.center.x + std::cos(angle) * arc.radius, arc.center.y + std::sin(angle) * arc.radius, 0.0f });
        }
    }
    // A flecha sorteada de uma perna (0 reta).
    float drawStarBend(World& w) const {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (settings.starCurveShare <= 0.0f || u(w.rng) >= settings.starCurveShare) return 0.0f;
        const float size = settings.starCurveMin + (std::max(settings.starCurveMin, settings.starCurveMax) - settings.starCurveMin) * u(w.rng);
        return u(w.rng) < 0.5f ? size : -size;
    }
    void startStar(World& w) {
        World::Star& s = w.star;
        const Vec3 root = w.body->links[rootLink].position;
        const int first = s.next;
        s.center = { root.x, root.y, 0.0f };
        s.from = s.center;
        s.base = w.maneuver == 9 ? wrapAngle(w.faceYaw - 0.25f * Pi * static_cast<float>(first)) : w.faceYaw;
        s.radius = !w.runBand ? 6.0f : s.phase <= 2 ? 8.0f : 10.0f;
        s.at = -1;
        s.goal = first;
        s.legs = 0;
        s.braked = false;
        s.pendingHeading = -1;
        s.graceUntil = -1.0f;
        s.bend = drawStarBend(w);
        s.curves = s.pathSamples = 0;
        s.pathErrorSum = 0.0;
        planStarNext(w);
        s.spin = starSpin(w);
    }
    // O ponto depois do de agora e o angulo da troca nele: do centro, as pontas ate o angulo da fase; da ponta, o centro (2 de
    // 4) ou a ponta a 90 graus de cada lado.
    void planStarNext(World& w) {
        World::Star& s = w.star;
        const float in = planarYaw(starPoint(w, s.at), starPoint(w, s.goal));
        int options[8];
        int count = 0;
        if (s.goal < 0) {
            const float widest = (s.phase <= 1 ? 0.5f : s.phase == 2 ? 0.75f : 1.0f) * Pi + 0.01f;
            for (int j = 0; j < 8; ++j)
                if (std::abs(wrapAngle(planarYaw(s.center, starPoint(w, j)) - in)) <= widest) options[count++] = j;
        } else {
            options[count++] = -1;
            options[count++] = -1;
            options[count++] = (s.goal + 2) % 8;
            options[count++] = (s.goal + 6) % 8;
        }
        if (count == 0) options[count++] = (s.at + 4 + 8) % 8;
        s.next = options[std::min(count - 1, static_cast<int>(std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng) * count))];
        // (12/11 17h20, settings.starCurveShare) A perna seguinte em curva ou reta; a troca entre a tangente da chegada e a da saida.
        s.nextBend = drawStarBend(w);
        s.nextTurn = wrapAngle(starLegYaws(starPoint(w, s.goal), starPoint(w, s.next), s.nextBend).first
                               - starLegYaws(starPoint(w, s.at), starPoint(w, s.goal), s.bend).second);
    }
    // O giro da vista na perna (o girando): um quarto das pernas sem, as outras de 40 a 100% do teto da fase, para um lado.
    float starSpin(World& w) const {
        if (w.maneuver != 10) return 0.0f;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        static constexpr float Most[4] { 0.25f, 0.4f, 0.55f, 0.7f };
        if (u(w.rng) < 0.25f) return 0.0f;
        const float rate = Most[std::clamp(w.star.phase, 1, 4) - 1] * (0.4f + 0.6f * u(w.rng));
        return u(w.rng) < 0.5f ? rate : -rate;
    }
    // A velocidade de chegada ao ponto (m/s) pela fase e pelo angulo da troca nele.
    // (08/11 12h05: em 2,5 h na fase 1 o fixo e o girando caiam 68-87% na primeira ponta - inverter 135-180 graus de 1,5 m/s
    // direto para a velocidade cheia do rumo novo. Na fase 1 a chegada a StarFirstSpeed e na 2 a StarSecondSpeed; e, nas
    // trocas de 90 graus ou mais nas fases 1-2, a saida devagar no rumo novo por StarStepOutSeconds - a troca assentada.)
    static constexpr float StarFirstSpeed = 0.8f, StarSecondSpeed = 1.2f, StarStepOutSeconds = 0.8f;
    float starArrival(const World& w, float speed) const {
        const World::Star& s = w.star;
        const float c = std::cos(s.nextTurn);
        switch (s.phase) {
        // (13h50: nunca abaixo de 60% do trote do rumo - abaixo disso o clipe do trote toca em camera lenta, 18% da cadencia a
        // 0,8 m/s, e a corrida da estrela caia na freada antes da primeira ponta.)
        case 1: return std::min(speed, std::max(StarFirstSpeed, 0.6f * fixedSpeed(1u, w.heading) * w.speedScale));
        case 2: return std::abs(s.nextTurn) <= 0.25f * Pi + 0.01f ? speed * (0.5f + 0.5f * c)
                : std::min(speed, std::max(StarSecondSpeed, 0.6f * fixedSpeed(1u, w.heading) * w.speedScale));
        case 3: return std::min(speed, std::max(StarSlowSpeed, speed * (0.5f + 0.5f * c)));
        default: return std::min(speed, std::max(1.2f, speed * (0.6f + 0.4f * c)));
        }
    }
    // Uma decisao da estrela: a distancia a linha da perna (courseError), o giro da vista (girando), a troca esperando o clipe,
    // a tecla (girando), a freada, a chegada e o rumo do pedido. Devolve a velocidade de referencia pedida.
    Vec3 starTick(World& w, float dt) {
        World::Star& s = w.star;
        const Vec3 root = w.body->links[rootLink].position;
        Vec3 goal = starPoint(w, s.goal);
        StarLeg g = starLeg(w, root);
        // (12/11 17h20, settings.starPathWeight: sem o encerramento por sair do caminho, a chegada pede estar perto - da ponta ou do caminho.)
        const bool arrived = g.remain <= StarArrival && (settings.starPathWeight <= 0.0f || std::abs(g.side) <= StarArrivalSide
            || std::hypot(root.x - goal.x, root.y - goal.y) <= StarArrivalSide);
        if (arrived) {
            // (A chegada: a proxima perna e a troca dela.)
            const float in = starLegYaws(s.from, goal, s.bend).second;
            s.legs += 1;
            if (s.bend != 0.0f) ++s.curves;
            s.at = s.goal;
            s.from = goal;
            s.goal = s.next;
            s.bend = s.nextBend;
            goal = starPoint(w, s.goal);
            const float out = starLegYaws(s.from, goal, s.bend).first;
            const float turn = wrapAngle(out - in);
            const bool wasBraked = s.braked;
            planStarNext(w);
            s.braked = false;
            s.graceUntil = w.time + StarGraceSeconds;
            s.spin = starSpin(w);
            w.runAgainAt = -1.0f;
            if (w.maneuver == 9) {
                const bool turned = std::abs(turn) >= RunTurnMinimum && w.brakeTurnAt < 0.0f && requestRunTurn(w, turn);
                if (!turned) {
                    w.viewTarget = out;
                    if (wasBraked) switchGait(w, w.startBand, 0u);
                }
            } else {
                const int h = (static_cast<int>(std::lround(wrapAngle(out - w.faceYaw) / (0.25f * Pi))) + 8) % 8;
                s.fullAt = -1.0f;
                if (s.phase <= 2 && std::abs(turn) >= 0.5f * Pi - 0.01f && wasBraked
                    && switchGait(w, 1u, static_cast<std::uint32_t>(h), std::max(0.5f, s.arrive))) {
                    s.pendingHeading = -1;
                    s.fullAt = w.time + StarStepOutSeconds;
                } else {
                    s.pendingHeading = switchGait(w, w.startBand, static_cast<std::uint32_t>(h)) ? -1 : h;
                }
            }
            g = starLeg(w, root);
        }
        const float along = g.along, remain = g.remain, side = g.side, length = g.length;
        const bool curved = s.bend != 0.0f;
        w.courseError = g.distance;
        s.pathErrorSum += g.distance;
        ++s.pathSamples;
        // (O girando: a vista gira na perna, andando - o quadro e a mistura juntos.)
        w.arcRate = 0.0f;
        if (w.maneuver == 10 && s.spin != 0.0f && w.refVelocity.length() > 0.5f) {
            const float turn = s.spin * dt;
            w.faceYaw = wrapAngle(w.faceYaw + turn);
            w.trackYaw = wrapAngle(w.trackYaw + turn);
            w.blendYaw = wrapAngle(w.blendYaw + turn);
            w.arcRate = s.spin;
        }
        // (A troca de rumo da chegada que o clipe nao deixou - a largada, outra troca: tenta a cada decisao.)
        if (s.pendingHeading >= 0 && switchGait(w, w.startBand, static_cast<std::uint32_t>(s.pendingHeading))) s.pendingHeading = -1;
        // (Acabou a saida devagar: a velocidade cheia da faixa no rumo de agora.)
        if (s.fullAt >= 0.0f && w.time >= s.fullAt && s.pendingHeading < 0 && switchGait(w, w.startBand, w.heading)) s.fullAt = -1.0f;
        // (O girando: a tecla - o rumo de 45 em 45 graus em relacao a vista - muda quando o ponto sai do setor dela.)
        if (w.maneuver == 10 && s.pendingHeading < 0 && remain > StarArrival + 0.5f) {
            // (Na curva, a tecla pela mira a frente no arco.)
            const float rel = wrapAngle(planarYaw(root, curved ? starAim(w, along + std::max(CourseLookMinimum, CourseLookSeconds * w.commandSpeed))
                                                               : goal) - w.faceYaw);
            if (std::abs(wrapAngle(rel - 0.25f * Pi * static_cast<float>(w.heading))) > 0.125f * Pi + StarKeySlack) {
                const int key = (static_cast<int>(std::lround(rel / (0.25f * Pi))) + 8) % 8;
                switchGait(w, w.band, static_cast<std::uint32_t>(key), s.braked || s.fullAt >= 0.0f ? w.commandSpeed : -1.0f);
            }
        }
        // (A freada para a chegada, a ManeuverAcceleration com folga, na mesma tecla; abaixo do trote, a faixa do trote.)
        if (!s.braked && w.brakeTurnAt < 0.0f && s.pendingHeading < 0 && s.fullAt < 0.0f) {
            const float v = w.commandSpeed, arrive = starArrival(w, v);
            const float need = (v * v - arrive * arrive) / (1.6f * settings.maneuverAcceleration) + 0.5f;
            if (arrive < v - 0.05f && remain <= need) {
                const std::uint32_t band = arrive <= fixedSpeed(1u, w.heading) * w.speedScale + 0.05f ? 1u : w.band;
                if (switchGait(w, band, w.heading, arrive)) {
                    s.braked = true;
                    s.arrive = arrive;
                    w.runAgainAt = -1.0f;
                }
            }
        }
        float speed = w.commandSpeed;
        if (w.maneuver == 9) {
            // (O livre: o rumo persegue a linha - a mira a frente na perna, como nas curvas; virada grande, pela vista girando.)
            if (w.brakeTurnAt < 0.0f) {
                const float look = std::max(CourseLookMinimum, CourseLookSeconds * w.commandSpeed);
                const float at = std::clamp(along + look, 0.0f, length + 1.0f);
                const Vec3 aim = starAim(w, at);
                const float yaw = planarYaw(root, aim);
                if (std::abs(wrapAngle(yaw - w.faceYaw)) > StarSnapMost) w.viewTarget = yaw;
                const float pending = wrapAngle(w.viewTarget - w.faceYaw);
                if (std::abs(pending) > StarSnapMost * 0.5f) {
                    const float step = std::clamp(pending, -FreeViewRate * dt, FreeViewRate * dt);
                    w.faceYaw = wrapAngle(w.faceYaw + step);
                    w.arcRate = step / dt;
                    w.improviseUntil = w.time + ImproviseAfterSeconds;
                    if (std::abs(w.arcRate) > 0.1f) speed = std::min(speed, ManeuverLateral / std::abs(w.arcRate));
                } else {
                    w.arcRate += (wrapAngle(yaw - w.moveYaw) / dt - w.arcRate) * 0.3f;
                    w.faceYaw = yaw;
                    w.viewTarget = yaw;
                }
                w.moveYaw = w.faceYaw;
                w.trackYaw = w.faceYaw;
            }
        } else {
            // (Fixo e girando: o rumo da tecla; no fixo, uma correcao pequena de volta a linha.)
            // (12/11 17h20: sem o encerramento, a correcao cresce longe do caminho; na curva, o fixo segue a tangente do arco e a tecla
            // muda de setor com ela.)
            const float most = StarSteerMost + (settings.starPathWeight > 0.0f ? 0.35f * std::clamp((std::abs(side) - 1.0f) / 2.0f, 0.0f, 1.0f) : 0.0f);
            const float steer = std::clamp(-StarSteerGain * side, -most, most);
            if (curved && w.maneuver == 8) {
                const float rel = wrapAngle(g.tangent - w.faceYaw);
                if (s.pendingHeading < 0 && s.fullAt < 0.0f && w.brakeTurnAt < 0.0f
                    && std::abs(wrapAngle(rel - 0.25f * Pi * static_cast<float>(w.heading))) > 0.125f * Pi + StarKeySlack) {
                    const int keyNow = (static_cast<int>(std::lround(rel / (0.25f * Pi))) + 8) % 8;
                    switchGait(w, w.band, static_cast<std::uint32_t>(keyNow), s.braked ? w.commandSpeed : -1.0f);
                }
                w.moveYaw = wrapAngle(g.tangent + steer);
            } else {
                const float key = wrapAngle(w.faceYaw + 0.25f * Pi * static_cast<float>(w.heading));
                w.moveYaw = w.maneuver == 8 ? wrapAngle(key + steer) : curved ? wrapAngle(g.tangent) : key;
            }
        }
        return { std::cos(w.moveYaw) * speed, std::sin(w.moveYaw) * speed, 0.0f };
    }

    // O corpo contra o percurso das curvas: o ponto mais perto dele (a busca so avanca, ate 10 m por decisao) e a distancia
    // ate a linha (os dois trechos em volta dele; antes do primeiro ponto, o trecho desde o comeco).
    static constexpr float CourseLookSeconds = 0.5f, CourseLookMinimum = 2.0f, OffCourseSeconds = 0.5f, StraySeconds = 1.0f;
    void measureCourse(World& w) {
        if (!isCurve(w) || w.course.size() < 2 || !w.body) return;
        const Vec3 root = w.body->links[rootLink].position;
        const auto planar = [&](Vec3 p) { return std::hypot(p.x - root.x, p.y - root.y); };
        std::size_t best = w.courseAt;
        float nearest = planar(w.course[best]);
        for (std::size_t k = w.courseAt + 1; k < w.course.size() && k <= w.courseAt + 10; ++k)
            if (const float d = planar(w.course[k]); d < nearest) {
                nearest = d;
                best = k;
            }
        w.courseAt = best;
        float t = 0.0f;
        const auto segment = [&](Vec3 a, Vec3 b) {
            const float dx = b.x - a.x, dy = b.y - a.y, length = dx * dx + dy * dy;
            t = length > 1.0e-6f ? std::clamp(((root.x - a.x) * dx + (root.y - a.y) * dy) / length, 0.0f, 1.0f) : 0.0f;
            return std::hypot(a.x + t * dx - root.x, a.y + t * dy - root.y);
        };
        float error = segment(best > 0 ? w.course[best - 1] : w.courseOrigin, w.course[best]);
        float progress = static_cast<float>(best) - 1.0f + t;
        if (best + 1 < w.course.size())
            if (const float after = segment(w.course[best], w.course[best + 1]); after < error) {
                error = after;
                progress = static_cast<float>(best) + t;
            }
        w.courseError = error;
        // (14h40, ele: "um tranco na camera a cada bolinha azul que ele passa": a mira pulava 1 m a cada ponto passado e o rumo
        // - e a camera, que gira com ele - dava um salto. O progresso continuo, que so avanca.)
        w.courseProgress = std::max(w.courseProgress, progress);
        if (w.courseAt + 40 > w.course.size() && w.coursePlan.size() < 64) extendCourse(w, 2);
    }
    // A curva sorteada no comeco (curvas 5-7): os arcos como antes - raio e graus por tipo, a troca de lado - e a reta depois
    // de cada um; o percurso no mundo segue a referencia (o rumo do deslocamento na velocidade pedida, decisao a decisao) a
    // partir da pelve no comeco.
    void extendCourse(World& w, int count) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        int sign = w.coursePlan.empty() ? w.arcSign : (w.coursePlan.back().rate >= 0.0f ? 1 : -1);
        for (int k = 0; k < count; ++k) {
            const float speed = std::max(0.5f, w.commandSpeed);
            const float low = w.maneuver == 5 ? 12.0f : w.maneuver == 6 ? 5.0f : 3.0f;
            const float high = w.maneuver == 5 ? 18.0f : w.maneuver == 6 ? 10.0f : 8.0f;
            const float minimum = speed * speed / ManeuverLateral;
            const float radius = std::max(minimum, low + (high - low) * u(w.rng));
            const float degrees = w.maneuver == 5 ? 45.0f + 75.0f * u(w.rng) : w.maneuver == 6 ? 60.0f + 90.0f * u(w.rng)
                                                                                               : 60.0f + 110.0f * u(w.rng);
            const float flip = w.maneuver == 5 ? 0.4f : w.maneuver == 6 ? 1.0f : 0.5f;
            if (u(w.rng) < flip) sign = -sign;
            World::CourseArc arc;
            arc.radians = degrees * Pi / 180.0f;
            arc.rate = static_cast<float>(sign) * speed / radius;
            arc.straightAfter = w.maneuver == 7 ? 0.5f + 0.5f * u(w.rng) : 1.0f + u(w.rng);
            w.coursePlan.push_back(arc);
        }
        traceCourse(w);
    }
    void planCourse(World& w) {
        w.coursePlan.clear();
        w.courseNext = 0;
        w.course.clear();
        w.courseWalked = 0.0f;
        w.courseAt = 0;
        w.courseError = w.offCourseSeconds = 0.0f;
        w.slowSeconds = 0.0f;
        w.courseProgress = -1.0f;
        if (!isCurve(w)) return;
        extendCourse(w, 6);
    }
    // O percurso no mundo do plano inteiro: da pelve no comeco, a reta ate o primeiro arco e cada arco e reta.
    void traceCourse(World& w) {
        w.course.clear();
        if (!w.body) return;
        const Vec3 root = w.body->links[rootLink].position;
        Vec3 at { root.x, root.y, 0.0f };
        float heading = w.moveYaw;
        // (Do instante de agora para tras nao importa: o percurso recomeca daqui com o que falta - a reta e o arco de agora e
        // os do plano.)
        if (w.time > 0.0f) {
            at = w.courseOrigin;
            heading = w.courseHeading;
        } else {
            w.courseOrigin = at;
            w.courseHeading = heading;
        }
        const float speed = std::max(0.5f, w.commandSpeed), dt = TicksPerDecision * TickSeconds;
        float walked = 0.0f, t = 0.0f;
        const auto run = [&](float seconds, float rate) {
            for (float s = 0.0f; s < seconds; s += dt) {
                heading += rate * dt;
                at = at + Vec3 { std::cos(heading) * speed * dt, std::sin(heading) * speed * dt, 0.0f };
                walked += speed * dt;
                t += dt;
                if (walked >= 1.0f) {
                    w.course.push_back(at);
                    walked -= 1.0f;
                }
            }
        };
        run(std::max(0.0f, w.nextManeuverAt), 0.0f);
        for (const World::CourseArc& arc : w.coursePlan) {
            run(arc.radians / std::max(1.0e-3f, std::abs(arc.rate)), arc.rate);
            run(arc.straightAfter, 0.0f);
            if (t > 30.0f) break;
        }
    }

    // O giro parado (o "giro" do B21: de pe, o rumo pedido do corpo gira e o deslocamento pedido e zero). Sem o rastreio,
    // como antes: 30-450 graus a 0,4-0,8 rad/s para um lado sorteado; na avaliacao, 0,6 rad/s o
    // episodio todo, metade dos mundos para cada lado. (03/11, ele: "tambem bote o treino de girar parado". Com o
    // rastreio: os clipes do modo padrao - turn_left e turn_right, 90 graus por ciclo de 1 s em dois passos - em ciclos
    // encadeados, cada um somando o giro do clipe ao rumo do quadro, na velocidade do clipe x 0,7-1,15; giros de 1 a 8
    // ciclos - a troca de lado com os dois pes no chao -, sem pausa; na avaliacao, na velocidade do clipe, o episodio todo.)
    void beginTurn(World& w) {
        beginFromClip(w, true);
        // (Ensinado: a vista do peito - o rumo do clipe parado -, nao a da pelve, ~22 graus virada no parado em alerta.)
        w.faceYaw = w.lastClipYaw;
        w.moveYaw = w.faceYaw;
        nextTurn(w);
    }
    void nextTurn(World& w, bool allowPause = true) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        // (Na avaliacao, metade para cada lado pelo indice e a semente: o laboratorio, um mundo, alterna pela semente.)
        bool left = settings.evaluation ? (w.index + static_cast<std::uint32_t>(settings.seed)) % 2u == 0u : u(w.rng) < 0.5f;
        if (settings.turnVariety) left = u(w.rng) < 0.5f;
        const std::size_t clip = turnIndices[left ? 0 : 1];
        if (settings.turnVariety && allowPause && w.time > 0.0f && standingIndex < references.size() && u(w.rng) < 0.3f) {
            // (settings.turnVariety: a pausa - o idle alerta no rumo de agora, 0,5-2 s; dai o proximo giro.)
            w.turnPauseUntil = w.time + 0.5f + 1.5f * u(w.rng);
            w.trackClip = standingIndex;
            w.trackScale = 1.0f;
            w.trackStart = -w.time;
            w.rampFrom = -1.0f;
            w.trackYaw = w.faceYaw;
            w.turnRate = 0.0f;
            w.turnLeft = 0.0f;
            return;
        }
        w.turnPauseUntil = -1.0f;
        if (clip < references.size()) {
            // (No comeco, a pelve do quadro na do corpo: o parado em alerta tem a pelve ~22 graus virada.)
            if (w.time <= 0.0f) {
                references[clip]->frame(0.0f, {}, 0.0f, w.frame);
                w.faceYaw = wrapAngle(bodyYaw(w) - trackingBodyYaw3D(profile->links[rootLink], w.frame.links[rootLink]));
                w.moveYaw = w.faceYaw;
            }
            const float cycle = std::abs(references[clip]->clip().turnCycleRadians);
            const float clipRate = cycle / std::max(1.0e-3f, references[clip]->durationSeconds());
            float scale = settings.evaluation ? 1.0f : 0.7f + 0.45f * u(w.rng);
            w.turnRate = left ? clipRate * scale : -clipRate * scale;
            w.turnLeft = settings.evaluation ? 1.0e9f
                                             : cycle * static_cast<float>(std::uniform_int_distribution<int>(1, 8)(w.rng));
            if (settings.turnVariety) {
                // (settings.turnVariety: o giro fixo de 60, 90, 180 ou 360 graus, na velocidade do clipe x 0,6-1,4.)
                static constexpr float Amounts[4] { 60.0f, 90.0f, 180.0f, 360.0f };
                scale = 0.6f + 0.8f * u(w.rng);
                w.turnRate = left ? clipRate * scale : -clipRate * scale;
                w.turnLeft = Amounts[std::uniform_int_distribution<int>(0, 3)(w.rng)] * Pi / 180.0f;
            }
            w.trackClip = clip;
            w.trackScale = scale;
            w.trackStart = -w.time * scale;
            w.trackYaw = w.faceYaw;
            // (11/11: o peito do clipe no alvo de agora - o quadro gira pelo desvio entre o peito dele e o alvo.)
            w.trackYaw = wrapAngle(w.trackYaw + wrapAngle(w.faceYaw - referenceChestYaw(w)));
            // (11h20, a folha r400: o corpo do giro ficava parado na base do idle, os pes plantados - nunca vivia um estado
            // girando. No comeco do treino, em locomotionClipFraction, ja dentro do clipe do giro - o RSI: instante e rumo
            // sorteados, as velocidades do clipe na velocidade pedida -, o rumo pedido no ponto do ciclo.)
            if (w.time <= 0.0f && !settings.evaluation && u(w.rng) < settings.locomotionClipFraction) {
                const float duration = std::max(1.0e-3f, references[clip]->durationSeconds());
                const float t = u(w.rng) * duration, yaw = (2.0f * u(w.rng) - 1.0f) * Pi;
                references[clip]->frame(t, {}, yaw, w.frame);
                for (PhysicsBodyState3D& s : w.frame.links) {
                    s.linearVelocity = s.linearVelocity * scale;
                    s.angularVelocity = s.angularVelocity * scale;
                }
                w.scene->resetRagdollState(w.handle, w.frame.links);
                w.body = w.scene->ragdollStateView(w.handle);
                const float done = cycle * t / duration;
                w.trackYaw = yaw;
                w.trackStart = t;
                w.faceYaw = w.moveYaw = wrapAngle(yaw + (left ? done : -done));
                w.turnLeft = std::max(cycle * 0.25f, w.turnLeft - done);
            }
            return;
        }
        const float rate = settings.evaluation ? 0.6f : 0.4f + 0.4f * u(w.rng);
        w.turnRate = left ? rate : -rate;
        w.turnLeft = settings.evaluation ? 1.0e9f : (30.0f + 420.0f * u(w.rng)) * Pi / 180.0f;
    }
    // O rumo pedido do giro, uma decisao adiante.
    // (11/11, ele: "corrija o giro parado... o clipe que ele precisa seguir aponta para uma direcao, mas o alvo esta em outra")
    // O rumo do peito do quadro de referencia posto, pela mesma conta do chestYaw do corpo.
    // (11h45: o tronco todo, nao o peito - o rumo do tronco do quadro seguido.)
    float referenceChestYaw(World& w) const {
        TrackingClipFrame3D frame;
        referenceFrame(w, 0.0f, true, frame);
        return frame.links.empty() ? w.faceYaw : trunkYawOf(frame.links);
    }
    void advanceTurn(World& w) {
        if (w.startKind != 7) return;
        // (settings.turnVariety: na pausa o rumo fica; acabada, o proximo giro - sem outra pausa logo em seguida.)
        if (w.turnPauseUntil >= 0.0f) {
            if (w.time < w.turnPauseUntil) return;
            nextTurn(w, false);
            return;
        }
        // (O giro com o clipe: o alvo e o peito do clipe a cada decisao - o clipe gira em passos, e o alvo que girava liso ficava
        // fora dele; agora os dois apontam sempre para o mesmo lado. O que falta girar desconta o quanto o peito girou.)
        if (w.trackClip < references.size() && (w.trackClip == turnIndices[0] || w.trackClip == turnIndices[1])) {
            const float chest = referenceChestYaw(w);
            w.turnLeft -= std::abs(wrapAngle(chest - w.faceYaw));
            w.faceYaw = w.moveYaw = chest;
            if (w.turnLeft > 0.0f) return;
            nextTurn(w);
            return;
        }
        const float step = std::min(std::abs(w.turnRate) * TicksPerDecision * TickSeconds, w.turnLeft);
        w.faceYaw = wrapAngle(w.faceYaw + std::copysign(step, w.turnRate));
        w.turnLeft -= step;
        if (w.turnLeft > 0.0f) return;
        nextTurn(w);
    }

    // A flexao do corpo agora (de -2 a 1): as maos a frente do peito e os joelhos a frente da pelve, cada um ate 0,25 m
    // (+1; atras, -1), na media, menos a coluna arqueada para tras (o peito inclinado para tras em relacao a pelve, ate
    // 30 graus: -1).
    float flexion(const RagdollState3D& body) const {
        const auto frame = [&](std::uint32_t i) {
            return (body.links[i].orientation * profile->links[i].modelOrientation.conjugate()).normalized();
        };
        const Quaternion pelvis = frame(rootLink), chest = frame(upperChestLink);
        const Vec3 pelvisForward = pelvis.rotate({ 1.0f, 0.0f, 0.0f }), chestForward = chest.rotate({ 1.0f, 0.0f, 0.0f });
        const auto ahead = [](float meters) { return std::clamp(meters / 0.25f, -1.0f, 1.0f); };
        float limbs = 0.0f;
        for (int side = 0; side < 2; ++side) {
            limbs += ahead(dot(body.links[handLinks[side]].position - body.links[upperChestLink].position, chestForward));
            limbs += ahead(dot(body.links[shinLinks[side]].position - body.links[rootLink].position, pelvisForward));
        }
        const float back = std::asin(std::clamp(-dot(chest.rotate({ 0.0f, 0.0f, 1.0f }), pelvisForward), -1.0f, 1.0f));
        return 0.25f * limbs - std::clamp(back / (30.0f * Pi / 180.0f), 0.0f, 1.0f);
    }

    // (01/11, o cenario sujo: so o que toca uma superficie de chao - o ombro na parede, ou o peso em cima, nao e caido.)
    bool fallen(const TrackingGetUpPosture3D& posture, const RagdollState3D& body) const {
        (void)posture;
        for (const RagdollContactPoint3D& contact : body.contacts)
            if (contact.linkIndex < allowedContact.size() && !allowedContact[contact.linkIndex] && contact.normal.z > 0.5f)
                return true;
        return false;
    }

    // A altura do chao (m) embaixo de p: a rampa do episodio (que acaba no chao plano, z = 0) ou o plano.
    float terrainHeight(const World& w, Vec3 p) const {
        if (!w.sloped) return 0.0f;
        const Vec3& n = w.slopeNormal;
        return std::max(0.0f, w.slopeHeight - (n.x * p.x + n.y * p.y) / n.z);
    }
    // (17/11, settings.contactForceObservation) A forca (N) dos contatos do ultimo tick que nao sao o apoio: tudo que se move (outro
    // corpo, a bola) e o estatico de normal de lado (a parede); o chao, a rampa e o degrau (estaticos de normal para cima, o mesmo corte
    // de 0,62 das interacoes) ficam fora. A mesma conta no jogo.
    static Vec3 nonSupportContactForce(std::span<const RagdollInteraction3D> interactions) {
        Vec3 sum;
        for (const RagdollInteraction3D& c : interactions)
            if (c.otherMotion != RagdollContactMotion3D::Static || c.normalWorld.z < 0.62f) sum += c.normalImpulseWorld;
        return sum * (1.0f / TickSeconds);
    }
    float floorUnder(const World& w) const {
        return w.body != nullptr && !w.body->links.empty() ? terrainHeight(w, w.body->links[rootLink].position) : 0.0f;
    }

    // O que o episodio anterior deixou (as paredes, os pesos, o chao e os corpos soltos), antes de qualquer queda.
    void clearScenery(World& w) {
        for (const PhysicsBodyHandle3D handle : w.obstacles)
            if (w.scene->contains(handle)) w.scene->destroyBody(handle);
        w.obstacles.clear();
        w.boxes.clear();
        for (const RagdollHandle3D handle : w.extras)
            if (w.scene->contains(handle)) w.scene->destroyRagdoll(handle);
        w.extras.clear();
        w.sloped = false;
        w.slopeNormal = { 0.0f, 0.0f, 1.0f };
        w.slopeHeight = 0.0f;
        w.underLift = 0.0f;
    }

    // Um corpo solto (passivo, meio mole) deitado num rumo sorteado, com a pelve em pelvis.
    bool controlledScenery() const {
        return settings.sceneryPreset >= 0 || (!settings.evaluation && (settings.obstacleFraction > 0.0f || settings.corridorFraction > 0.0f
            || settings.pillarFraction > 0.0f || settings.slopeFraction > 0.0f || settings.roughFraction > 0.0f
            || settings.bodiesUnderFraction > 0.0f || settings.bodyOnTopFraction > 0.0f));
    }

    void spawnLooseBody(World& w, Vec3 pelvis, float axisAngle = -1.0f) {
        auto fixedRng = w.rng;
        if (controlledScenery()) fixedRng.seed(7301u + 101u * static_cast<unsigned>((w.controlledPreset == 5 ? 1 : w.controlledPreset) + 1));
        auto& sceneryRng = controlledScenery() ? fixedRng : w.rng;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 2 + w.extras.size();
        spawn.active = false;
        spawn.rigidityPercent = 10.0f + 30.0f * u(sceneryRng);
        spawn.pelvisPosition = pelvis;
        const float axis = 2.0f * Pi * u(sceneryRng);
        spawn.orientation = Quaternion::fromAxisAngle({ std::cos(axis), std::sin(axis), 0.0f }, 0.5f * Pi * (0.85f + 0.3f * u(sceneryRng)));
        if (axisAngle >= 0.0f)
            spawn.orientation = Quaternion::fromAxisAngle({ std::cos(axisAngle), std::sin(axisAngle), 0.0f }, 0.5f * Pi);
        w.extras.push_back(w.scene->createRagdoll(*profile, spawn));
    }

    // O chao da queda (so no treino, antes dela): a rampa, os calombos e os corpos soltos embaixo.
    void placeTerrain(World& w) {
        auto fixedRng = w.rng;
        if (controlledScenery()) fixedRng.seed(7301u + 101u * static_cast<unsigned>((w.controlledPreset == 5 ? 1 : w.controlledPreset) + 1));
        auto& sceneryRng = controlledScenery() ? fixedRng : w.rng;
        if (settings.evaluation && !settings.evaluationScenery) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if ((!controlledScenery() && u(sceneryRng) < settings.slopeFraction) || w.forcedScenery == 5) {
            const float angle = 2.0f * Pi * u(sceneryRng);
            const float tilt = (controlledScenery() ? 22.0f : (6.0f + 12.0f * u(sceneryRng))) * Pi / 180.0f;
            const Quaternion turn = Quaternion::fromAxisAngle({ -std::sin(angle), std::cos(angle), 0.0f }, tilt);
            w.sloped = true;
            w.slopeNormal = turn.rotate({ 0.0f, 0.0f, 1.0f });
            w.slopeHeight = controlledScenery() ? 0.8f : 0.3f * u(sceneryRng);
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = { 8.0f, 8.0f, 1.0f };
            shape.materialId = "concrete";
            PhysicsBodyDefinition3D ramp;
            ramp.motionType = PhysicsMotionType3D::Static;
            ramp.position = Vec3 { 0.0f, 0.0f, w.slopeHeight } - w.slopeNormal * shape.halfExtents.z;
            ramp.orientation = turn;
            ramp.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(ramp, std::span<const PhysicsShape3D>(&shape, 1)));
            w.boxes.push_back({ w.obstacles.back(), {}, {}, shape.halfExtents });
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.roughFraction) || w.forcedScenery == 6) {
            std::vector<PhysicsShape3D> bumps(std::uniform_int_distribution<std::size_t>(10, 20)(sceneryRng));
            for (PhysicsShape3D& bump : bumps) {
                const float a = 2.0f * Pi * u(sceneryRng), r = 2.2f * std::sqrt(u(sceneryRng));
                bump.type = PhysicsShapeType3D::Box;
                bump.halfExtents = { 0.08f + 0.27f * u(sceneryRng), 0.08f + 0.27f * u(sceneryRng), 0.02f + 0.06f * u(sceneryRng) };
                bump.localPosition = { r * std::cos(a), r * std::sin(a), 0.0f };
                bump.localPosition.z = terrainHeight(w, bump.localPosition);
                bump.localOrientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 2.0f * Pi * u(sceneryRng))
                    * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, 0.3f * (u(sceneryRng) - 0.5f))).normalized();
                bump.materialId = "concrete";
            }
            PhysicsBodyDefinition3D ground;
            ground.motionType = PhysicsMotionType3D::Static;
            ground.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(ground, std::span<const PhysicsShape3D>(bumps)));
            for (const PhysicsShape3D& bump : bumps)
                w.boxes.push_back({ w.obstacles.back(), bump.localPosition, bump.localOrientation, bump.halfExtents });
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.bodiesUnderFraction) || w.forcedScenery == 7) {
            const int count = controlledScenery() ? (w.controlledPreset == 4 ? 2 : 1) : std::uniform_int_distribution<int>(1, 3)(sceneryRng);
            for (int k = 0; k < count; ++k) {
                const float a = 2.0f * Pi * u(sceneryRng), r = 0.5f * std::sqrt(u(sceneryRng));
                Vec3 pelvis { r * std::cos(a), r * std::sin(a), 0.0f };
                pelvis.z = terrainHeight(w, pelvis) + 0.25f + 0.35f * static_cast<float>(k);
                spawnLooseBody(w, pelvis);
            }
            w.underLift = 0.3f * static_cast<float>(count);
        }
    }

    // O chao do evitar ambientado (antes de ficar de pe, no meio): a rampa suave com ele em cima dela, os calombos fora
    // do lugar dos pes e os corpos caidos em volta (0,6-1,6 m), para tropecar ou pisar.
    void placeAvoidTerrain(World& w) {
        auto fixedRng = w.rng;
        if (controlledScenery()) fixedRng.seed(7301u + 101u * static_cast<unsigned>((w.controlledPreset == 5 ? 1 : w.controlledPreset) + 1));
        auto& sceneryRng = controlledScenery() ? fixedRng : w.rng;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if ((!controlledScenery() && u(sceneryRng) < settings.slopeFraction) || w.forcedScenery == 5) {
            const float angle = 2.0f * Pi * u(sceneryRng);
            const float tilt = (controlledScenery() ? 22.0f : (4.0f + 8.0f * u(sceneryRng))) * Pi / 180.0f;
            const Quaternion turn = Quaternion::fromAxisAngle({ -std::sin(angle), std::cos(angle), 0.0f }, tilt);
            w.sloped = true;
            w.slopeNormal = turn.rotate({ 0.0f, 0.0f, 1.0f });
            w.slopeHeight = controlledScenery() ? 0.8f : 0.2f + 0.3f * u(sceneryRng);
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = { 8.0f, 8.0f, 1.0f };
            shape.materialId = "concrete";
            PhysicsBodyDefinition3D ramp;
            ramp.motionType = PhysicsMotionType3D::Static;
            ramp.position = Vec3 { 0.0f, 0.0f, w.slopeHeight } - w.slopeNormal * shape.halfExtents.z;
            ramp.orientation = turn;
            ramp.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(ramp, std::span<const PhysicsShape3D>(&shape, 1)));
            w.boxes.push_back({ w.obstacles.back(), {}, {}, shape.halfExtents });
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.roughFraction) || w.forcedScenery == 6) {
            std::vector<PhysicsShape3D> bumps(std::uniform_int_distribution<std::size_t>(10, 20)(sceneryRng));
            for (PhysicsShape3D& bump : bumps) {
                const float a = 2.0f * Pi * u(sceneryRng), r = 0.45f + 1.75f * std::sqrt(u(sceneryRng));
                bump.type = PhysicsShapeType3D::Box;
                bump.halfExtents = { 0.08f + 0.2f * u(sceneryRng), 0.08f + 0.2f * u(sceneryRng), 0.02f + 0.06f * u(sceneryRng) };
                bump.localPosition = { r * std::cos(a), r * std::sin(a), 0.0f };
                bump.localPosition.z = terrainHeight(w, bump.localPosition);
                bump.localOrientation = (Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 2.0f * Pi * u(sceneryRng))
                    * Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, 0.3f * (u(sceneryRng) - 0.5f))).normalized();
                bump.materialId = "concrete";
            }
            PhysicsBodyDefinition3D ground;
            ground.motionType = PhysicsMotionType3D::Static;
            ground.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(ground, std::span<const PhysicsShape3D>(bumps)));
            for (const PhysicsShape3D& bump : bumps)
                w.boxes.push_back({ w.obstacles.back(), bump.localPosition, bump.localOrientation, bump.halfExtents });
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.bodiesUnderFraction) || w.forcedScenery == 7) {
            const int count = controlledScenery() ? 4 : std::uniform_int_distribution<int>(1, 3)(sceneryRng);
            for (int k = 0; k < count; ++k) {
                const float a = controlledScenery() ? 0.5f * Pi * static_cast<float>(k) : 2.0f * Pi * u(sceneryRng);
                const float r = controlledScenery() ? 0.72f : 0.6f + 1.0f * u(sceneryRng);
                Vec3 pelvis { r * std::cos(a), r * std::sin(a), 0.0f };
                if (controlledScenery()) pelvis += Vec3 { -0.35f * std::sin(a), 0.35f * std::cos(a), 0.0f };
                pelvis.z = terrainHeight(w, pelvis) + 0.25f;
                spawnLooseBody(w, pelvis, controlledScenery() ? a : -1.0f);
            }
        }
    }

    // O cenario sujo do comeco (so no treino, com o corpo ja assentado): a parede ou caixa fixa encostada nele, o
    // corredor, os pilares, o peso e o corpo solto caindo em cima.
    // (16/11, settings.locomotionSceneryFraction - ver o .hpp) A locomocao com coisas paradas em volta do caminho.
    void placeLocomotionScenery(World& w) {
        if (w.body == nullptr || w.body->links.empty()) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const Vec3 root = w.body->links[rootLink].position;
        const float dx = w.target.x - root.x, dy = w.target.y - root.y;
        const float heading = std::hypot(dx, dy) > 0.5f ? std::atan2(dy, dx) : characterYaw(w);
        const Vec3 forward { std::cos(heading), std::sin(heading), 0.0f }, left { -std::sin(heading), std::cos(heading), 0.0f };
        const int count = std::uniform_int_distribution<int>(2, 5)(w.rng);
        for (int k = 0; k < count; ++k) {
            const float along = -1.0f + 9.0f * u(w.rng);
            const bool inPath = along > 3.0f && u(w.rng) < 0.12f;
            const float side = inPath ? 0.3f * (u(w.rng) - 0.5f) : (u(w.rng) < 0.5f ? -1.0f : 1.0f) * (0.55f + 1.6f * u(w.rng));
            const int kind = std::uniform_int_distribution<int>(0, 2)(w.rng);
            Vec3 half;
            float yaw = 2.0f * Pi * u(w.rng);
            const char* material = "concrete";
            if (kind == 0) {  // a caixa
                half = { 0.2f + 0.3f * u(w.rng), 0.2f + 0.3f * u(w.rng), 0.15f + 0.45f * u(w.rng) };
                material = "wood";
            } else if (kind == 1) {  // a parede curta, mais ou menos ao longo do caminho
                half = { 0.4f + 0.8f * u(w.rng), 0.08f + 0.07f * u(w.rng), 0.5f + 0.6f * u(w.rng) };
                yaw = heading + 0.4f * (u(w.rng) - 0.5f);
            } else {  // um jogador parado: o pilar do tamanho de um corpo
                half = { 0.16f + 0.04f * u(w.rng), 0.12f + 0.04f * u(w.rng), 0.85f + 0.1f * u(w.rng) };
            }
            Vec3 position = root + forward * along + left * side;
            // (Nunca em cima do corpo: o meio da coisa a pelo menos a meia diagonal dela + 0,35 m de cada elo.)
            const float clearance = std::hypot(half.x, half.y) + 0.35f;
            bool clear = true;
            for (const PhysicsBodyState3D& link : w.body->links)
                clear = clear && std::hypot(link.position.x - position.x, link.position.y - position.y) >= clearance;
            if (!clear) continue;
            position.z = terrainHeight(w, position) + half.z;
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = half;
            shape.materialId = material;
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            body.position = position;
            body.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
            body.materialId = material;
            w.obstacles.push_back(w.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1)));
            w.boxes.push_back({ w.obstacles.back(), {}, {}, half });
        }
        w.obstructed = w.obstructed || !w.obstacles.empty();
    }
    void placeObstacles(World& w) {
        auto fixedRng = w.rng;
        if (controlledScenery()) fixedRng.seed(7301u + 101u * static_cast<unsigned>((w.controlledPreset == 5 ? 1 : w.controlledPreset) + 1));
        auto& sceneryRng = controlledScenery() ? fixedRng : w.rng;
        // (12/11 09h15, settings.getUpArenaMax - ver o .hpp) As paredes do levantar, antes de tudo (tambem nas avaliacoes e no laboratorio).
        if (w.clipGetUp && settings.getUpArenaMax > 0.0f && !w.body->links.empty()) {
            std::uniform_real_distribution<float> span(settings.getUpArenaMin, std::max(settings.getUpArenaMin, settings.getUpArenaMax));
            const float distance = span(w.rng);
            const Vec3 center = w.body->links[rootLink].position;
            constexpr float Height = 2.5f, Thickness = 0.3f;
            for (int side = 0; side < 4; ++side) {
                const float angle = 0.5f * Pi * static_cast<float>(side);
                const Vec3 direction { std::cos(angle), std::sin(angle), 0.0f };
                const Vec3 half { 0.5f * Thickness, distance + Thickness, 0.5f * (Height + 0.5f) };
                PhysicsShape3D shape;
                shape.type = PhysicsShapeType3D::Box;
                shape.halfExtents = half;
                shape.materialId = "concrete";
                PhysicsBodyDefinition3D body;
                body.motionType = PhysicsMotionType3D::Static;
                body.position = { center.x + direction.x * (distance + half.x), center.y + direction.y * (distance + half.x),
                    terrainHeight(w, {}) + Height - half.z };
                body.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, angle);
                body.materialId = "concrete";
                w.obstacles.push_back(w.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1)));
                w.boxes.push_back({ w.obstacles.back(), {}, {}, half, true });
            }
        }
        // (12/11 12h50, fase 2 - settings.fireSlipShare) O chao escorregadio: a placa de grama molhada, 5 mm acima do concreto.
        // (12/11 16h30: tambem no fogo da locomocao - andando, 30 x 30 m: ela sai dela correndo em 2-4 s, a grama seca de novo.)
        if ((w.clipGetUp || w.locoFire) && w.fireSlip && !w.body->links.empty()) {
            const Vec3 root = w.body->links[rootLink].position;
            const float side = w.locoFire && w.startKind == 6 ? 15.0f : 10.0f;
            const Vec3 half { side, side, 0.05f };
            w.fireSlipCenter = root;
            w.fireSlipHalf = side;
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = half;
            shape.materialId = "wet_grass";
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            body.position = { root.x, root.y, terrainHeight(w, {}) + 0.005f - half.z };
            body.materialId = "wet_grass";
            w.obstacles.push_back(w.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1)));
            Impl::World::SceneryBox box { w.obstacles.back(), {}, {}, half, false, true };
            w.boxes.push_back(box);
        }
        // (12/11 12h30, fase 2 - settings.fireWallShare) O prensado: a parede do lado para onde o continuo empurra, a fireWallDistance do
        // elo mais longe nesse lado; 6 m de comprimento; desenhada no laboratorio como as paredes do levantar.
        if ((w.clipGetUp || w.locoFire) && w.fireWallDistance >= 0.0f && !w.body->links.empty()) {
            const Vec3 root = w.body->links[rootLink].position;
            const Vec3 direction { std::cos(w.fireWallAngle), std::sin(w.fireWallAngle), 0.0f };
            float reach = 0.0f;
            for (const PhysicsBodyState3D& link : w.body->links) reach = std::max(reach, dot(link.position - root, direction));
            constexpr float Height = 2.5f, Thickness = 0.3f;
            const Vec3 half { 0.5f * Thickness, 3.0f, 0.5f * (Height + 0.5f) };
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = half;
            shape.materialId = "concrete";
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            const float at = reach + w.fireWallDistance + half.x;
            body.position = { root.x + direction.x * at, root.y + direction.y * at, terrainHeight(w, {}) + Height - half.z };
            body.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.fireWallAngle);
            body.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1)));
            w.boxes.push_back({ w.obstacles.back(), {}, {}, half, true });
        }
        if ((settings.evaluation && !settings.evaluationScenery) || w.body->links.empty()) {
            w.obstructed = !w.obstacles.empty() || !w.extras.empty();
            return;
        }
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const Vec3 root = w.body->links[rootLink].position;
        // (Do link mais longe nesse lado, para comecar encostada sem atravessar o corpo.)
        const auto reachToward = [&](Vec3 direction) {
            float reach = 0.0f;
            for (const PhysicsBodyState3D& link : w.body->links) reach = std::max(reach, dot(link.position - root, direction));
            return reach;
        };
        // (A base enterrada meio metro: na rampa a parede nao fica com vao embaixo.)
        const auto wall = [&](float angle, float gap, Vec3 half, float height) {
            const Vec3 direction { std::cos(angle), std::sin(angle), 0.0f };
            Vec3 position = root + direction * (reachToward(direction) + gap + half.x);
            const float top = terrainHeight(w, position) + height;
            half.z = 0.5f * (top + 0.5f);
            position.z = top - half.z;
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = half;
            shape.materialId = "concrete";
            PhysicsBodyDefinition3D body;
            body.motionType = PhysicsMotionType3D::Static;
            body.position = position;
            body.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, angle);
            body.materialId = "concrete";
            w.obstacles.push_back(w.scene->createBody(body, std::span<const PhysicsShape3D>(&shape, 1)));
            w.boxes.push_back({ w.obstacles.back(), {}, {}, half });
        };
        if ((!controlledScenery() && u(sceneryRng) < settings.obstacleFraction) || w.forcedScenery == 0) {
            if (controlledScenery()) {
                // Four short walls leave corner exits while blocking each cardinal direction.
                const float radius = std::max({ reachToward({ 1.0f, 0.0f, 0.0f }), reachToward({ -1.0f, 0.0f, 0.0f }),
                    reachToward({ 0.0f, 1.0f, 0.0f }), reachToward({ 0.0f, -1.0f, 0.0f }) }) + 1.20f;
                for (int side = 0; side < 4; ++side) {
                    const float angle = 0.5f * Pi * static_cast<float>(side);
                    wall(angle, radius - reachToward({ std::cos(angle), std::sin(angle), 0.0f }),
                        { 0.14f, 0.60f, 0.0f }, 1.8f);
                }
            } else {
                const Vec3 half { 0.15f + 0.25f * u(sceneryRng), 0.5f + 1.0f * u(sceneryRng), 0.0f };
                wall(2.0f * Pi * u(sceneryRng), 0.04f + 0.4f * u(sceneryRng), half, 0.4f + 1.1f * u(sceneryRng));
            }
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.corridorFraction) || w.forcedScenery == 1) {
            const bool fixedGetUp = controlledScenery() && w.startKind != 4;
            const Vec3 spine = w.body->links[contract->chestLink()].position - root;
            const float angle = fixedGetUp ? (std::hypot(spine.x, spine.y) > 0.2f ? std::atan2(spine.y, spine.x) : characterYaw(w)) + 0.5f * Pi
                : 2.0f * Pi * u(sceneryRng);
            for (const float side : { 0.0f, Pi }) {
                if (fixedGetUp) {
                    const float a = angle + side;
                    const float width = w.controlledPreset == 5 ? 1.36f : 1.30f;
                    // Absolute inside width: body pose must never resize the corridor.
                    wall(a, 0.5f * width - reachToward({ std::cos(a), std::sin(a), 0.0f }), { 0.20f, 2.0f, 0.0f }, 2.0f);
                } else {
                    const Vec3 half { 0.15f + 0.15f * u(sceneryRng), 1.0f + 1.0f * u(sceneryRng), 0.0f };
                    wall(angle + side, controlledScenery() ? 0.30f : 0.04f + 0.4f * u(sceneryRng), half, 1.0f + 1.5f * u(sceneryRng));
                }
            }
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.pillarFraction) || w.forcedScenery == 2) {
            if (controlledScenery()) {
                // Irregular fixed ring: angle, clearance, half-width, height. The get-up uses four pillars.
                constexpr float pillars[6][4] { { 0.12f, 0.85f, 0.14f, 1.8f }, { 1.15f, 1.20f, 0.19f, 2.2f },
                    { 3.15f, 1.00f, 0.16f, 1.6f }, { 4.65f, 1.40f, 0.12f, 2.0f },
                    { 2.15f, 0.95f, 0.18f, 2.4f }, { 5.65f, 1.15f, 0.15f, 1.9f } };
                const int count = 6;
                constexpr float getUpClearance[6] { 0.15f, 0.22f, 0.18f, 0.28f, 0.20f, 0.25f };
                for (int k = 0; k < count; ++k)
                    wall(pillars[k][0], w.startKind == 4 ? pillars[k][1] : getUpClearance[k],
                        { pillars[k][2], pillars[k][2], 0.0f }, pillars[k][3]);
            } else {
                const int count = std::uniform_int_distribution<int>(1, 4)(sceneryRng);
                for (int k = 0; k < count; ++k) {
                    const float side = 0.12f + 0.13f * u(sceneryRng);
                    wall(2.0f * Pi * u(sceneryRng), 0.1f + 0.6f * u(sceneryRng), { side, side, 0.0f }, 1.5f + 1.0f * u(sceneryRng));
                }
            }
        }
        if ((!controlledScenery() && u(sceneryRng) < settings.weightFraction) || w.forcedScenery == 3) {
            const std::size_t link = std::uniform_int_distribution<std::size_t>(0, w.body->links.size() - 1)(sceneryRng);
            PhysicsShape3D shape;
            shape.type = PhysicsShapeType3D::Box;
            shape.halfExtents = { 0.12f + 0.06f * u(sceneryRng), 0.4f + 0.3f * u(sceneryRng), 0.1f + 0.05f * u(sceneryRng) };
            shape.materialId = "wood";
            PhysicsBodyDefinition3D weight;
            weight.motionType = PhysicsMotionType3D::Dynamic;
            weight.position = w.body->links[link].position + Vec3 { 0.0f, 0.0f, 0.3f + 0.3f * u(sceneryRng) };
            weight.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 2.0f * Pi * u(sceneryRng));
            weight.massKg = 20.0f + 40.0f * u(sceneryRng);
            weight.materialId = "wood";
            w.obstacles.push_back(w.scene->createBody(weight, std::span<const PhysicsShape3D>(&shape, 1)));
            w.boxes.push_back({ w.obstacles.back(), {}, {}, shape.halfExtents });
        }
        // (So com ele no chao: de pe, um corpo caindo em cima seria outro exercicio. 02/11 23h15, ele: "jogasse eles em
        // cima direitinho de onde ele nasce para cairem nele logo antes dele comecar a levantar": um ou dois, centrados
        // no tronco (+-10 cm), 30-60 cm acima dele, caindo no comeco do episodio; no maximo 4 corpos soltos por cena.)
        if (w.down && ((!controlledScenery() && u(sceneryRng) < settings.bodyOnTopFraction) || w.forcedScenery == 4 || (controlledScenery() && w.controlledPreset == 4))) {
            float top = root.z;
            for (const PhysicsBodyState3D& link : w.body->links) top = std::max(top, link.position.z);
            const Vec3 chest = rlLinkCenterOfMass3D(profile->links[contract->chestLink()], w.body->links[contract->chestLink()]);
            const Vec3 torso = (root + chest) * 0.5f;
            const int count = std::min<int>(controlledScenery() ? (w.controlledPreset == 4 ? 2 : 1) : (u(sceneryRng) < 0.3f ? 2 : 1), 4 - static_cast<int>(w.extras.size()));
            for (int k = 0; k < count; ++k)
                spawnLooseBody(w, { torso.x + 0.1f * (2.0f * u(sceneryRng) - 1.0f), torso.y + 0.1f * (2.0f * u(sceneryRng) - 1.0f),
                                      top + 0.3f + 0.3f * u(sceneryRng) + 0.45f * static_cast<float>(k) });
        }
        w.obstructed = !w.obstacles.empty() || !w.extras.empty();
    }

    // (07/11, o cinematico) A raiz de um clipe num instante (no ciclo, uma volta a mais composta).
    std::pair<Vec3, float> clipRoot(std::size_t ref, float t) const {
        const float duration = references[ref]->durationSeconds();
        const bool loops = clips[ref]->loops;
        TrackingClipFrame3D frame;
        MotionMatchingPose3D pose;
        if (!loops || duration <= 0.0f || t <= duration) {
            references[ref]->frame(std::clamp(t, 0.0f, std::max(0.0f, duration)), {}, 0.0f, frame);
            motionReader.read(frame.links, pose);
            return { pose.rootPosition, pose.rootYaw };
        }
        const auto end = clipRoot(ref, duration), start = clipRoot(ref, 0.0f), inner = clipRoot(ref, t - duration);
        const float turn = end.second - start.second;
        const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, turn);
        return { end.first + q.rotate(inner.first - start.first), inner.second + turn };
    }
    // O quadro (grupo, fonte, instante) posto na raiz do mundo do cinematico.
    void placedFrame(const World& w, std::size_t g, std::size_t source, float t, TrackingClipFrame3D& out) const {
        const std::size_t ref = motionRefs[g][source];
        references[ref]->frame(t, {}, 0.0f, out);
        MotionMatchingPose3D pose;
        motionReader.read(out.links, pose);
        const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw - pose.rootYaw);
        for (PhysicsBodyState3D& s : out.links) {
            s.position = w.mmRootPosition + q.rotate(s.position - pose.rootPosition);
            s.orientation = (q * s.orientation).normalized();
            s.linearVelocity = q.rotate(s.linearVelocity);
            s.angularVelocity = q.rotate(s.angularVelocity);
        }
    }
    std::size_t clipIndex(const std::string& name) const {
        const auto it = std::find(clipNames.begin(), clipNames.end(), "referencias_unreal/" + name);
        return it == clipNames.end() ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>(it - clipNames.begin());
    }
    // O clipe (indice da referencia) posto no mundo na raiz da sequencia.
    // (07/11, ele: "no correr diagonal ele se move para a diagonal mas a pose dele e do para frente") O rumo do PERSONAGEM no
    // clipe (o osso root da Unreal, perdido na conversao - a raiz virou a pelve): nos ciclos, partidas e paradas ele olha
    // para a frente do clipe o tempo todo (a pelve cruza para o lado do movimento: na lateral a esquerda ela vira 34 graus
    // para a direita); na virada, o quanto o clipe ja girou desde o comeco; no parado, no giro e nas curvas, o da pelve e do
    // peito.
    // (Tipos: 0 parado, 1 partida, 2 ciclo, 3 virada correndo e partida virando, 4 parada, 5 curva, 7 virada parada, 8
    // levantar. O parado e o levantar tambem olham a frente do clipe; a curva gira por igual - o giro inteiro do clipe sobre a
    // duracao: o da pelve balanca 15 graus a cada passada e o corpo virava aos trancos.)
    float clipFacing(std::size_t ref, float t, int kind) {
        if (kind == 0 || kind == 1 || kind == 2 || kind == 4 || kind == 8) return 0.0f;
        if (kind == 3 || kind == 7) {
            // (Na virada o personagem comeca olhando a frente do clipe e acaba no rumo em que corre - na parada, no angulo da
            // virada. A pelve e o peito tem um desvio dele que muda: parado a pelve fica 15-20 graus torta, correndo nao; o
            // desvio vai de um ao outro com o giro. Pela pelve so, a saida para o ciclo girava esses 20 graus de uma vez.)
            const float y0 = clipRoot(ref, 0.0f).second, y = clipRoot(ref, t).second;
            const float yEnd = clipRoot(ref, references[ref]->durationSeconds()).second;
            const float endOffset = kind == 3 ? yEnd - clipTravelEnd(ref) : y0;
            const float s = std::abs(yEnd - y0) > 1.0e-3f ? std::clamp((y - y0) / (yEnd - y0), 0.0f, 1.0f) : 1.0f;
            return y - (y0 + (endOffset - y0) * s);
        }
        if (kind == 5) return clipTurnRate(ref) * t;
        return clipRoot(ref, t).second;
    }
    // O rumo em que o clipe corre no fim (o deslocamento dos ultimos 0,2 s), desenrolado perto do giro da pelve.
    float clipTravelEnd(std::size_t ref) {
        const float d = references[ref]->durationSeconds();
        const auto [a, ya] = clipRoot(ref, std::max(0.0f, d - 0.2f));
        const auto [b, yb] = clipRoot(ref, d);
        const float direction = std::atan2(b.y - a.y, b.x - a.x);
        return yb + std::remainder(direction - yb, 2.0f * Pi);
    }
    float clipTurnRate(std::size_t ref) {
        const float d = std::max(0.1f, references[ref]->durationSeconds());
        return (clipRoot(ref, d).second - clipRoot(ref, 0.0f).second) / d;
    }
    // A inclinacao lateral media de um clipe (pelve -> cabeca contra a vertical, no lado do rumo dele; + para a esquerda): a que
    // ele ja tem - medida, nao pela fisica (os arcos da Unreal quase nao inclinam: 8 graus onde a fisica pede 27).
    std::unordered_map<std::size_t, float> clipRolls;
    std::mutex clipRollMutex;
    std::size_t headLink = std::numeric_limits<std::size_t>::max();
    float clipRoll(std::size_t ref, int kind) {
        // (Os mundos andam em threads: a medida de cada clipe uma vez, guardada.)
        const std::lock_guard<std::mutex> lock(clipRollMutex);
        if (const auto it = clipRolls.find(ref); it != clipRolls.end()) return it->second;
        if (headLink >= profile->links.size())
            for (std::size_t i = 0; i < profile->links.size(); ++i)
                if (profile->links[i].id == "Head") headLink = i;
        const float length = references[ref]->durationSeconds();
        double sum = 0.0;
        int n = 0;
        TrackingClipFrame3D f;
        for (float t = 0.0f; t < length && headLink < profile->links.size(); t += 1.0f / 30.0f, ++n) {
            references[ref]->frame(t, {}, 0.0f, f);
            const Vec3 up = f.links[headLink].position - f.links[rootLink].position;
            const float h = clipFacing(ref, t, kind);
            sum += std::atan2(-std::sin(h) * up.x + std::cos(h) * up.y, up.z);
        }
        return clipRolls[ref] = n > 0 ? static_cast<float>(sum / n) : 0.0f;
    }
    // O instante em que o giro de uma virada chega a 90% do dela.
    float turnDone(std::size_t ref, int kind) {
        const float length = references[ref]->durationSeconds();
        const float total = clipFacing(ref, length, kind);
        for (float u = 0.0f; u < length; u += 1.0f / 30.0f)
            if (std::abs(clipFacing(ref, u, kind)) >= 0.9f * std::abs(total)) return u;
        return length;
    }
    void sequencePlaced(World& w, std::size_t ref, float t, TrackingClipFrame3D& out, int kind) {
        const float facing = clipFacing(ref, t, kind);
        references[ref]->frame(t, {}, 0.0f, out);
        MotionMatchingPose3D pose;
        motionReader.read(out.links, pose);
        const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw - facing);
        for (PhysicsBodyState3D& s : out.links) {
            s.position = w.mmRootPosition + q.rotate(s.position - pose.rootPosition);
            s.orientation = (q * s.orientation).normalized();
            s.linearVelocity = q.rotate(s.linearVelocity);
            s.angularVelocity = q.rotate(s.angularVelocity);
        }
    }
    // (A estrela de 8 pontas, so para ver) Do centro a uma ponta sorteada (6 m no trote, 10 m na corrida, de 45 em 45 graus a
    // partir do rumo do comeco); dali, metade das vezes de volta ao centro, metade direto a outra ponta; em metade dos pontos
    // para (freando, 0,8-1,5 s parado), nos outros passa direto. A velocidade e a dos clipes na direcao em relacao ao corpo; o
    // corpo: peito fixo, no rumo do comeco; peito livre, para onde vai.
    void starPilot(World& w) {
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const bool run = w.runBand || w.band >= 2;
        const float radius = run ? 10.0f : 6.0f;
        const Vec3 here = w.mmPlaying ? w.mmRootPosition : Vec3 { w.body->links[rootLink].position.x, w.body->links[rootLink].position.y, 0.0f };
        const auto tip = [&](int k) {
            const float a = w.starHeading + static_cast<float>(k) * 0.25f * Pi;
            return Vec3 { w.starCenter.x + std::cos(a) * radius, w.starCenter.y + std::sin(a) * radius, 0.0f };
        };
        if (!w.starValid) {
            w.starValid = true;
            w.starCenter = here;
            w.starHeading = w.faceYaw;
            w.starTip = static_cast<int>(u(w.rng) * 8.0f) % 8;
            w.starTarget = tip(w.starTip);
            w.starAtCenter = false;
            w.starStop = u(w.rng) < 0.5f;
            w.starHoldUntil = -1.0f;
        }
        Vec3 to = w.starTarget - here;
        to.z = 0.0f;
        float distance = std::hypot(to.x, to.y);
        if (w.time >= w.starHoldUntil && distance < 0.6f) {
            if (w.starStop) w.starHoldUntil = w.time + 0.8f + 0.7f * u(w.rng);
            if (!w.starAtCenter && u(w.rng) < 0.5f) {
                w.starTarget = w.starCenter;
                w.starAtCenter = true;
            } else {
                int next = static_cast<int>(u(w.rng) * 8.0f) % 8;
                if (next == w.starTip) next = (next + 1 + static_cast<int>(u(w.rng) * 7.0f)) % 8;
                w.starTip = next;
                w.starTarget = tip(next);
                w.starAtCenter = false;
            }
            w.starStop = u(w.rng) < 0.5f;
            to = w.starTarget - here;
            to.z = 0.0f;
            distance = std::hypot(to.x, to.y);
        }
        const float move = std::atan2(to.y, to.x);
        const float face = starDrill == 1 ? w.starHeading : move;
        const int sector = (static_cast<int>(std::lround(std::remainder(move - face, 2.0f * Pi) / (0.25f * Pi))) + 8) % 8;
        const bool front = sector == 0 || sector == 1 || sector == 7, side = sector == 2 || sector == 6;
        float speed = front ? (run ? 7.7f : 4.5f) : side ? 3.9f : 3.35f;
        if (w.starStop) speed = std::min(speed, std::sqrt(2.0f * 4.0f * std::max(0.0f, distance - 0.3f)));
        w.moveYaw = move;
        w.faceYaw = face;
        w.commandSpeed = w.time < w.starHoldUntil || speed < 0.3f ? 0.0f : speed;
    }

    // Uma decisao da sequencia de clipes do exercicio (sem busca): escolhe o clipe pela regra, avanca, troca com a transicao
    // nas juntas (0,3 s) e poe o corpo na pose, sem fisica.
    void sequenceStep(World& w) {
        constexpr std::size_t None = std::numeric_limits<std::size_t>::max();
        const float dt = TicksPerDecision * TickSeconds;
        static constexpr const char* Codes[8] { "f", "fl", "ll", "bl", "b", "br", "rr", "fr" };
        if (starDrill > 0) starPilot(w);
        const bool moving = w.commandSpeed > 0.05f;
        // (Fora da locomocao a vista e o alvo so no levantar correndo; no parado e a do exercicio - a direcao a um alvo que
        // nao existe girava sozinha e ele virava atras dela.)
        const float face = w.locomotion || w.startKind != 3 ? w.faceYaw : faceDirection(w), move = moveDirection(w);
        int sector = (static_cast<int>(std::lround(std::remainder(move - face, 2.0f * Pi) / (0.25f * Pi))) + 8) % 8;
        // (Num ciclo, a direcao so muda passando 10 graus da fronteira do setor - perto dela os dois ciclos se alternavam.)
        if (w.seqKind == 2 && w.seqSector >= 0 && sector != w.seqSector
            && std::abs(std::remainder(move - face - static_cast<float>(w.seqSector) * 0.25f * Pi, 2.0f * Pi)) < (22.5f + 10.0f) * Pi / 180.0f)
            sector = w.seqSector;
        // (07/11, ele: "a corrida a 6 m/s, sendo 5,5 para lateral e costas": os clipes corrida_* cobrem os 8 rumos - o sprint
        // da Unreal a 6,0 na frente e nas diagonais da frente, a corrida dela acelerada a 5,5 de lado e de costas.)
        const bool sprint = w.runBand || w.band >= 2;
        // (A virada da troca comeca quando o exercicio inicia a meia-volta dele - runTurn -, com o angulo pedido.)
        // (O giro do rumo pedido desde a decisao anterior - o giro parado.)
        const float faceRate = w.motionPreviousValid ? std::remainder(face - w.motionPreviousFace, 2.0f * Pi) / dt : 0.0f;
        w.motionPreviousFace = face;
        w.motionPreviousValid = true;
        // (O giro do rumo suavizado, meia-vida 0,3 s - o arcRate do percurso mira um ponto a partir da pelve, que balanca a
        // cada passada, e a curva trocava de arco para ciclo 17 vezes em 8 s.)
        w.seqTurnRate += (faceRate - w.seqTurnRate) * (1.0f - std::pow(0.5f, dt / 0.3f));
        float faceTurn = 0.0f;
        if (starDrill == 2 && w.mmPlaying && (w.seqKind == 1 || w.seqKind == 2) && std::abs(faceRate * dt) >= 0.5f)
            faceTurn = faceRate * dt;
        if (w.runTurn >= 0 && w.runTurnFrom != w.seqTurnFrom) {
            w.seqTurnFrom = w.runTurnFrom;
            const std::vector<float>& yaw = runTurnYaw(w.runTurn);
            faceTurn = (w.runTurn == 1 ? 1.0f : -1.0f) * w.runTurnScale * (yaw.empty() ? 0.0f : std::abs(yaw.back()));
        }
        // (O levantar correndo acaba correndo para a frente do clipe; o alvo noutro lado: a virada correndo.)
        const bool runGetUp = w.mmPlaying && w.seqKind == 8 && clipNames[w.mmGroup].find("getup_run") != std::string::npos;
        if (runGetUp && moving && w.time >= w.seqUntil) {
            const float error = std::remainder(face - w.mmRootYaw, 2.0f * Pi);
            if (std::abs(error) >= 0.6f) faceTurn = error;
        }
        const bool standingNow = w.seqKind <= 0 || w.seqKind == 4 || w.seqKind == 6 || w.seqKind == 7 || (w.seqKind == 8 && !runGetUp);
        float turnScale = 1.0f;
        // O clipe que a regra pede agora (e o tipo); a troca so quando ele muda.
        std::size_t want = None;
        int kind = w.seqKind;
        float startAt = 0.0f, until = -1.0f;
        const auto pick = [&](const std::string& name, int k) {
            const std::size_t c = clipIndex(name);
            if (c != None) {
                want = c;
                kind = k;
            }
            return c != None;
        };
        // (A partida e a virada vao ate o fim delas; a parada so enquanto nao ha pedido.)
        const bool busy = (w.seqKind == 3 || w.seqKind == 7 || w.seqKind == 8 || (w.seqKind == 1 && moving) || (w.seqKind == 4 && !moving))
            && w.time < w.seqUntil;
        if (moving && std::abs(faceTurn) >= 0.3f) {
            // A troca com virada do corpo: a virada do angulo mais perto, do lado do giro (comecando 0,35 s antes da metade dela).
            const SequenceTurn* best = nullptr;
            for (const SequenceTurn& t : sequenceTurns)
                if (t.left == (faceTurn > 0.0f) && t.fast == sprint && clipNames[t.clip].ends_with("_lfoot") && (best == nullptr || std::abs(t.degrees - std::abs(faceTurn) * 180.0f / Pi)
                        < std::abs(best->degrees - std::abs(faceTurn) * 180.0f / Pi)))
                    best = &t;
            if (best != nullptr) {
                want = best->clip;
                kind = 3;
                startAt = 0.0f;
                until = w.time + std::min(best->end, references[want]->durationSeconds() - 1.0f / 30.0f);
                // (O giro do clipe escalado ao angulo pedido - 70 graus pedidos num clipe de 90 giram 70.)
                const float clipTurn = std::abs(clipFacing(want, references[want]->durationSeconds(), 3));
                turnScale = clipTurn > 0.3f ? std::clamp(std::abs(faceTurn) / clipTurn, 0.7f, 1.4f) : 1.0f;
            }
        } else if (!busy) {
            if (!moving) {
                const float error = w.mmPlaying ? std::remainder(face - w.mmRootYaw, 2.0f * Pi) : 0.0f;
                if (w.seqKind == 1 || w.seqKind == 2 || w.seqKind == 3 || w.seqKind == 5 || (w.seqKind == 8 && runGetUp)) {
                    // (A parada e a do que esta correndo: o setor e o clipe - trote ou sprint - do de agora; o pedido parado ja
                    // aponta a perna seguinte da estrela. 07/11: parava "de costas" correndo para a frente, 45 cm de pulo.)
                    if (w.seqSector >= 0) sector = w.seqSector;
                    const bool running = clipNames[w.mmGroup].rfind("referencias_unreal/corrida", 0) == 0;
                    const bool picked = (running && pick(std::string("corrida_stop_") + Codes[sector] + "_lfoot", 4))
                        || pick(std::string("trote_run_stop_") + Codes[sector] + "_lfoot", 4);
                    if (picked) until = w.time + references[want]->durationSeconds() - 0.2f;
                } else if ((w.locomotion || w.lyingClip >= references.size() || w.startKind == 3)
                           && (std::abs(error) > 0.45f || (std::abs(w.seqTurnRate) > 0.3f && std::abs(error) > 0.2f))) {
                    // (Levantado de costas ou de brucos nao ha vista nem alvo: fica no parado.)
                    // (A virada parada da Unreal do angulo mais perto - 45, 90, 135 ou 180, com os passos dela -, o giro
                    // escalado ao angulo exato; com a vista girando, o angulo leva o que ela gira em 0,4 s, e as viradas
                    // se emendam enquanto ela gira. Os idle_turn vieram sem giro nenhum da conversao: so pisavam no lugar.)
                    const float aim = error + std::clamp(w.seqTurnRate, -3.0f, 3.0f) * 0.4f;
                    const float degrees = std::clamp(std::abs(aim), 0.6f, Pi) * 180.0f / Pi;
                    int best = 45;
                    for (int d : { 90, 135, 180 })
                        if (std::abs(static_cast<float>(d) - degrees) < std::abs(static_cast<float>(best) - degrees)) best = d;
                    char name[32];
                    std::snprintf(name, sizeof name, "stand_turn_%03d_%c", best, aim > 0.0f ? 'l' : 'r');
                    if (pick(name, 7)) {
                        until = w.time + turnDone(want, 7);
                        turnScale = std::clamp(degrees / static_cast<float>(best), 0.7f, 1.4f);
                    } else pick("stand_idle_loop", 0);
                } else pick("stand_idle_loop", 0);
            } else if (standingNow && w.mmPlaying && std::abs(std::remainder(face - w.mmRootYaw, 2.0f * Pi)) > 0.6f) {
                // (Parado e virado para outro lado - a estrela de peito livre: a partida virando, 90 ou 180 graus.)
                const float turn = std::remainder(face - w.mmRootYaw, 2.0f * Pi);
                const std::string name = std::string("run_reface_start_f_") + (turn > 0.0f ? "l_" : "r_")
                    + (std::abs(turn) > 2.36f ? "180" : "090");
                // (A partida virando comeca olhando a frente do clipe e acaba girada - o rumo dela e o da virada, tipo 3.)
                if ((sprint && pick("corrida_reface_start_f_" + name.substr(19), 3)) || pick("trote_" + name, 3)) {
                    // (Ela cede ao ciclo quando o giro chega a 90% do dela - o clipe inteiro corre 11 m e passava da ponta.)
                    until = w.time + turnDone(want, 3);
                    sector = 0;
                    const float clipTurn = std::abs(clipFacing(want, references[want]->durationSeconds(), 3));
                    if (clipTurn > 0.3f) turnScale = std::clamp(std::abs(turn) / clipTurn, 0.7f, 1.4f);
                }
            } else if (standingNow) {
                const bool started = sprint ? pick(std::string("corrida_start_") + Codes[sector] + "_lfoot", 1)
                                            : pick(std::string("trote_run_start_") + Codes[sector] + "_lfoot", 1);
                // (Nas curvas a partida cede logo ao arco - o percurso curva no primeiro segundo.)
                if (started) until = w.time + std::min(references[want]->durationSeconds() - 0.5f, isCurve(w) ? 1.0f : 1.0e9f);
            } else if (isCurve(w) && !sprint && std::abs(w.seqTurnRate) > (w.seqKind == 5 ? 0.12f : 0.25f)) {
                // (A curva pelo arco do trote, com folga para entrar e sair - entre 0,1 e 0,2 rad/s ficava trocando; no sprint
                // nao ha arco: o ciclo dele gira o rumo.)
                const auto& arcs = sequenceArcs[w.seqTurnRate > 0.0f ? 0 : 1];
                const float radius = w.commandSpeed / std::abs(w.seqTurnRate);
                std::size_t best = None;
                float gap = 1.0e9f;
                for (const auto& [clip, r] : arcs)
                    if (std::abs(r - radius) < gap) {
                        gap = std::abs(r - radius);
                        best = clip;
                    }
                // (Na mesma curva e no mesmo lado, so troca com o raio pedido 40% mais perto de outra.)
                if (best != None && w.seqKind == 5) {
                    float current = 1.0e9f;
                    for (const auto& [clip, r] : arcs)
                        if (clip == w.mmGroup) current = std::abs(r - radius);
                    if (current < 1.0e8f && gap > 0.6f * current) best = w.mmGroup;
                }
                if (best != None) {
                    want = best;
                    kind = 5;
                }
            } else {
                if (!(sprint && pick(std::string("corrida_loop_") + Codes[sector], 2)))
                    pick(std::string("trote_run_loop_") + Codes[sector], 2);
            }
        }
        bool wrapped = false;
        if (!w.mmPlaying) {
            MotionMatchingPose3D pose;
            motionReader.read(w.body->links, pose);
            w.mmRootPosition = pose.rootPosition;
            // (O personagem do clipe olha para a vista no comeco - o rumo da pelve do parado fica 20 graus torto.)
            w.mmRootYaw = face;
            if (w.lyingClip < references.size()) {
                // (Deitado: o clipe de levantar da Unreal do comeco, do quadro 0, exatamente onde o comeco o pos - o rumo e a
                // altura pela pelve do corpo contra a do clipe.)
                TrackingClipFrame3D raw;
                references[w.lyingClip]->frame(0.0f, {}, 0.0f, raw);
                const PhysicsBodyState3D& body = w.body->links[rootLink];
                const Vec3 x = (body.orientation * raw.links[rootLink].orientation.conjugate()).normalized().rotate({ 1.0f, 0.0f, 0.0f });
                w.mmRootYaw = std::atan2(x.y, x.x);
                w.mmRootPosition = { body.position.x, body.position.y, body.position.z - raw.links[rootLink].position.z };
                want = w.lyingClip;
                kind = 8;
                until = w.time + references[want]->durationSeconds() - 0.1f;
            }
            if (want == None) pick("stand_idle_loop", 0);
            if (want == None) return;
        } else {
            // Avanca o clipe tocando e a raiz com ele; o ciclo da a volta (as curvas, o parado e o giro tambem).
            const std::size_t ref = w.mmGroup;
            const float duration = references[ref]->durationSeconds();
            const bool cyclic = w.seqKind == 0 || w.seqKind == 2 || w.seqKind == 5 || w.seqKind == 6;
            const float t1 = cyclic && w.mmTime + dt > duration ? duration : std::min(w.mmTime + dt, duration);
            const auto [p0, y0] = clipRoot(ref, w.mmTime);
            const auto [p1, y1] = clipRoot(ref, t1);
            const float f0 = clipFacing(ref, w.mmTime, w.seqKind), f1 = clipFacing(ref, t1, w.seqKind);
            (void)y0;
            (void)y1;
            const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw - f0);
            Vec3 step = q.rotate(p1 - p0);
            step.z = 0.0f;
            // (Na partida e no ciclo, o deslocamento e o do clipe - o personagem dele anda exato no setor - girado devagar
            // pela diferenca entre o setor e o pedido (meia-vida 0,15 s; antes ia na direcao pedida na hora: a troca do pedido
            // virava a velocidade de uma vez, um tranco de 21 cm na pelve).)
            const float fix = moving && (w.seqKind == 1 || w.seqKind == 2) && w.seqSector >= 0
                ? std::clamp(std::remainder(move - w.mmRootYaw - static_cast<float>(w.seqSector) * 0.25f * Pi, 2.0f * Pi), -0.5f, 0.5f)
                : 0.0f;
            w.seqTravelFix += (fix - w.seqTravelFix) * (1.0f - std::pow(0.5f, dt / 0.15f));
            step = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.seqTravelFix).rotate(step);
            // (Na volta do ciclo o clipe so anda o resto ate o fim: o passo e esticado ao tempo inteiro da decisao - parava um
            // quadro, 18 cm de tranco.)
            const float played = t1 - w.mmTime;
            const float stretch = cyclic && w.mmTime + dt > duration && played > 1.0e-4f ? dt / played : 1.0f;
            w.mmRootPosition = w.mmRootPosition + step * stretch;
            // (Na partida, no ciclo e na curva em movimento o corpo gira com a vista - o giro dela nesta decisao, ate 0,15 rad,
            // mais a diferenca numa mola de meia-vida 0,2 s; antes so a mola: ficava 10 graus atras na curva e pulava a cada
            // volta do ciclo. Nas viradas, o giro do clipe escalado ao pedido.)
            const float yawBefore = w.mmRootYaw;
            const bool steered = moving && (w.seqKind == 1 || w.seqKind == 2 || w.seqKind == 5);
            if (!steered) w.mmRootYaw += (f1 - f0) * stretch * (w.seqKind == 3 || w.seqKind == 7 ? w.seqTurnScale : 1.0f);
            // (A volta do ciclo: os clipes da Unreal nao fecham exatos - os arcos menos ainda -, entao a volta e uma troca
            // para o proprio clipe, pela inercia e pelo pe; antes pulava 15-17 cm a cada volta.)
            wrapped = cyclic && w.mmTime + dt > duration;
            w.mmTime = t1;
            w.mmInertiaElapsed += dt;
            decaySequenceOffsets(w, dt);
            if (steered) {
                w.mmRootYaw += std::clamp(faceRate * dt, -0.15f, 0.15f);
                w.mmRootYaw += std::remainder(face - w.mmRootYaw, 2.0f * Pi) * (1.0f - std::pow(0.5f, dt / 0.2f));
            }
            // (O giro da curva para a inclinacao: o do rumo pedido, sem os saltos da vista - o do corpo saindo de uma virada
            // inclinava 16 graus numa reta.)
            const float k = 1.0f - std::pow(0.5f, dt / 0.08f);
            (void)yawBefore;
            const float curve = steered && std::abs(faceRate * dt) < 0.1f ? faceRate : 0.0f;
            w.seqYawRate += (curve - w.seqYawRate) * k;
            w.seqSpeed += (std::hypot(step.x, step.y) * stretch / dt - w.seqSpeed) * k;
        }
        if (wrapped && (want == None || want == w.mmGroup)) {
            want = w.mmGroup;
            kind = w.seqKind;
            until = w.seqUntil;
        }
        if (want != None && (!w.mmPlaying || want != w.mmGroup || wrapped)) {
            float t = startAt;
            TrackingClipFrame3D shown;
            SequenceRates shownRates;
            if (w.mmPlaying) {
                sequencePlaced(w, w.mmGroup, w.mmTime, shown, w.seqKind);
                applySequenceInertia(w, shown);
                // (A velocidade do mostrado: a do clipe no instante dele mais a das diferencas que restam - medir pelo quadro
                // anterior errava na volta do ciclo, em que o clipe so anda o resto ate o fim: 4 m/s de diferenca, 25 cm.)
                shownRates = sequenceRates(w, w.mmGroup, w.mmTime, w.seqKind);
                for (std::size_t i = 0; i < shownRates.coordinates.size() && i < w.seqOffsetCoordinateVelocity.size(); ++i)
                    shownRates.coordinates[i] = shownRates.coordinates[i] + w.seqOffsetCoordinateVelocity[i];
                shownRates.root = shownRates.root + w.seqOffsetRootVelocity;
                shownRates.spin = shownRates.spin + w.seqOffsetSpinVelocity;
                if (kind == 1) w.mmRootYaw = face;
                // (A entrada: o instante - e a variante do pe, nas partidas, paradas e viradas - em que os pes do clipe novo
                // estao mais parecidos com os de agora; o "pouso do pe" de antes, por medida.)
                std::vector<std::pair<std::size_t, std::pair<float, float>>> options;
                const auto window = [&](std::size_t c, float from, float to) { options.push_back({ c, { from, to } }); };
                const auto variant = [&](std::size_t c) {
                    const std::string& name = clipNames[c];
                    const std::size_t at = name.rfind("_lfoot");
                    return at == std::string::npos ? None : clipIndex(name.substr(19, at - 19) + "_rfoot");
                };
                const float length = references[want]->durationSeconds();
                // (Longe do fim: entrar a 0,03 s dele emendava duas transicoes.)
                if (kind == 2 || kind == 5) window(want, 0.0f, wrapped && want == w.mmGroup ? 0.5f * length : length - 0.3f);
                else if (kind == 1 || kind == 4) {
                    window(want, 0.0f, kind == 1 ? 0.15f : 0.3f);
                    if (const std::size_t other = variant(want); other != None) window(other, 0.0f, kind == 1 ? 0.15f : 0.3f);
                } else if (kind == 3) {
                    window(want, 0.0f, 0.3f);
                    if (const std::size_t other = variant(want); other != None) window(other, 0.0f, 0.3f);
                } else if (kind == 7) window(want, 0.0f, 0.1f);
                else if (kind == 0) window(want, 0.0f, wrapped && want == w.mmGroup ? 0.5f * length : length - 0.3f);
                else window(want, t, t);
                const auto feet = [&](const TrackingClipFrame3D& f, std::size_t i) {
                    return f.links[footLinks[i]].position - f.links[rootLink].position;
                };
                float bestCost = 1.0e9f;
                TrackingClipFrame3D candidate;
                for (const auto& [c, range] : options)
                    for (float u = range.first; u <= range.second + 1.0e-4f; u += 1.0f / 30.0f) {
                        sequencePlaced(w, c, u, candidate, kind);
                        float cost = 0.0f;
                        for (std::size_t i = 0; i < 2; ++i) {
                            cost += (feet(candidate, i) - feet(shown, i)).lengthSquared();
                            cost += 0.01f * (candidate.links[footLinks[i]].linearVelocity - shown.links[footLinks[i]].linearVelocity).lengthSquared();
                        }
                        if (cost < bestCost) {
                            bestCost = cost;
                            want = c;
                            t = u;
                        }
                    }
                if (kind == 3)
                    for (const SequenceTurn& turn : sequenceTurns)
                        if (turn.clip == want)
                            until = w.time + std::max(0.1f, std::min(turn.end, references[want]->durationSeconds() - 1.0f / 30.0f) - t);
            }
            // (Na partida, no ciclo e na parada o personagem do clipe olha para a vista desde a entrada - a partida virando
            // comeca virada para o lado de antes e gira ate ela.)
            if (kind == 1) w.mmRootYaw = face;
            // (A correcao do rumo continua do setor de antes ao novo - o deslocamento nao gira na troca; ela vai a do novo
            // devagar.)
            const int newSector = kind == 1 || kind == 2 || kind == 4 ? sector : -1;
            w.seqTravelFix = (kind == 1 || kind == 2) && w.seqSector >= 0 && newSector >= 0
                ? std::clamp(std::remainder(w.seqTravelFix + static_cast<float>(w.seqSector - newSector) * 0.25f * Pi, 2.0f * Pi), -0.8f, 0.8f)
                : 0.0f;
            if (w.mmPlaying) {
                TrackingClipFrame3D next;
                sequencePlaced(w, want, t, next, kind);
                // A mola parte das diferencas de pose e de velocidade entre o mostrado e o clipe novo na entrada.
                const SequenceRates nextRates = sequenceRates(w, want, t, kind);
                const std::size_t n = next.coordinates.size();
                w.mmOffsetCoordinates.assign(n, {});
                w.seqOffsetCoordinateVelocity.assign(n, {});
                for (std::size_t i = 0; i < n && i < shown.coordinates.size(); ++i) {
                    w.mmOffsetCoordinates[i] = shown.coordinates[i] - next.coordinates[i];
                    if (i < shownRates.coordinates.size()) w.seqOffsetCoordinateVelocity[i] = shownRates.coordinates[i] - nextRates.coordinates[i];
                }
                w.mmOffsetRootPosition = shown.links[rootLink].position - next.links[rootLink].position;
                w.seqOffsetRootVelocity = shownRates.root - nextRates.root;
                w.seqOffsetSpin = rotationVector((shown.links[rootLink].orientation * next.links[rootLink].orientation.conjugate()).normalized());
                w.seqOffsetSpinVelocity = shownRates.spin - nextRates.spin;
                w.mmInertiaElapsed = 0.0f;
            }
            w.mmGroup = want;
            w.mmTime = t;
            w.mmPlaying = true;
            w.seqKind = kind;
            w.seqTurnScale = kind == 3 || kind == 7 ? turnScale : 1.0f;
            w.seqSector = newSector;
            w.seqUntil = until;
            ++w.mmJumps;
        }
        TrackingClipFrame3D now;
        sequencePlaced(w, w.mmGroup, w.mmTime, now, w.seqKind);
        w.mmPose = now.links;
        applySequenceInertia(w, now);
        // (A regra dele: o peito sempre para a vista - o tronco de cima gira de volta em volta do abdome, ate 46 graus, como o
        // orientation warping da Unreal; menos na virada, na parada, no parado e no giro, que giram o corpo inteiro. O peso entra
        // e sai em 0,25 s e o giro anda no maximo 4 rad/s - sem isso, a entrada na virada soltava o tronco de uma vez e a vista
        // pulando para a ponta seguinte da estrela jogava a cabeca 60 cm num quadro.)
        const bool warp = w.seqKind == 1 || w.seqKind == 2 || w.seqKind == 5;
        w.seqWarpWeight = std::clamp(w.seqWarpWeight + (warp ? dt : -dt) / 0.25f, 0.0f, 1.0f);
        {
            const float chest = trackingBodyYaw3D(profile->links[motionReader.chest], now.links[motionReader.chest]);
            const float weight = w.seqWarpWeight * w.seqWarpWeight * (3.0f - 2.0f * w.seqWarpWeight);
            const float target = weight * std::clamp(std::remainder(face - chest, 2.0f * Pi), -0.8f, 0.8f);
            w.seqWarpAngle += std::clamp(target - w.seqWarpAngle, -4.0f * dt, 4.0f * dt);
        }
        if (std::abs(w.seqWarpAngle) > 1.0e-4f) {
            const float error = w.seqWarpAngle;
            const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, error);
            const Vec3 pivot = now.links[sequenceAbdomen].position;
            for (std::size_t i = 0; i < now.links.size(); ++i) {
                if (!sequenceUpper[i]) continue;
                PhysicsBodyState3D& l = now.links[i];
                l.position = pivot + q.rotate(l.position - pivot);
                l.orientation = (q * l.orientation).normalized();
                l.linearVelocity = q.rotate(l.linearVelocity);
                l.angularVelocity = q.rotate(l.angularVelocity);
            }
        }
        // (A inclinacao para dentro da curva, por cima da do clipe: a do giro e da velocidade de agora, atan(v w / g), menos a
        // que o clipe ja tem pelo giro dele - o arco tem a sua, o ciclo reto nenhuma; meia-vida 0,15 s, ate 29 graus. O corpo
        // inteiro gira em volta do chao sob a pelve no eixo do deslocamento e o pe mais baixo fica na altura dele.)
        {
            float target = 0.0f;
            if (moving && (w.seqKind == 2 || w.seqKind == 5))
                target = std::atan(w.seqSpeed * w.seqYawRate / 9.81f) - (w.seqKind == 5 ? clipRoll(w.mmGroup, 5) : 0.0f);
            w.seqLean += (std::clamp(target, -0.5f, 0.5f) - w.seqLean) * (1.0f - std::pow(0.5f, dt / 0.08f));
        }
        if (std::abs(w.seqLean) > 1.0e-3f) {
            const float travel = w.mmRootYaw + (w.seqSector >= 0 ? static_cast<float>(w.seqSector) * 0.25f * Pi : 0.0f);
            const Quaternion r = Quaternion::fromAxisAngle({ std::cos(travel), std::sin(travel), 0.0f }, -w.seqLean);
            const Vec3 pelvis = now.links[rootLink].position;
            const Vec3 pivot { pelvis.x, pelvis.y, w.mmRootPosition.z };
            // (O pe que fica: o mais baixo, num minimo suave de 4 cm - o minimo seco trocava de pe aos trancos, 22 cm.)
            const float z0 = now.links[footLinks[0]].position.z, z1 = now.links[footLinks[1]].position.z;
            const float low = std::min(z0, z1);
            const float w0 = std::exp(-(z0 - low) / 0.04f), w1 = std::exp(-(z1 - low) / 0.04f);
            for (PhysicsBodyState3D& l : now.links) {
                l.position = pivot + r.rotate(l.position - pivot);
                l.orientation = (r * l.orientation).normalized();
                l.linearVelocity = r.rotate(l.linearVelocity);
                l.angularVelocity = r.rotate(l.angularVelocity);
            }
            // (Suavizada, meia-vida 0,06 s: na passada o pe de baixo troca e a altura ia e voltava 10 cm.)
            const float lift = (w0 * (z0 - now.links[footLinks[0]].position.z) + w1 * (z1 - now.links[footLinks[1]].position.z)) / (w0 + w1);
            w.seqLift += (lift - w.seqLift) * (1.0f - std::pow(0.5f, dt / 0.06f));
            for (PhysicsBodyState3D& l : now.links) l.position.z += w.seqLift;
        } else w.seqLift = 0.0f;
        w.scene->resetRagdollState(w.handle, now.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }
    // As velocidades de um clipe da sequencia posto no mundo num instante (diferenca de 1/60 s ate ele): das juntas, da pelve
    // (com a correcao do rumo, na partida e no ciclo) e o giro da pelve.
    struct SequenceRates {
        std::vector<Vec3> coordinates;
        Vec3 root, spin;
    };
    SequenceRates sequenceRates(const World& w, std::size_t ref, float t, int kind) {
        constexpr float H = 1.0f / 60.0f;
        const float t0 = std::max(0.0f, std::min(t, references[ref]->durationSeconds()) - H), t1 = t0 + H;
        TrackingClipFrame3D a, b;
        references[ref]->frame(t0, {}, 0.0f, a);
        references[ref]->frame(t1, {}, 0.0f, b);
        const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw - clipFacing(ref, t, kind));
        const float fix = kind == 1 || kind == 2 ? w.seqTravelFix : 0.0f;
        SequenceRates out;
        out.coordinates.resize(b.coordinates.size());
        for (std::size_t i = 0; i < b.coordinates.size() && i < a.coordinates.size(); ++i) out.coordinates[i] = (b.coordinates[i] - a.coordinates[i]) / H;
        out.root = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, fix).rotate(q.rotate(b.links[rootLink].position - a.links[rootLink].position)) / H;
        out.spin = q.rotate(rotationVector((b.links[rootLink].orientation * a.links[rootLink].orientation.conjugate()).normalized())) / H;
        return out;
    }
    // O vetor de rotacao (eixo vezes angulo, o caminho curto) de um quaternio e a volta.
    static Vec3 rotationVector(Quaternion q) {
        if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
        const Vec3 v { q.x, q.y, q.z };
        const float s = v.length();
        return s < 1.0e-7f ? v * 2.0f : v * (2.0f * std::atan2(s, q.w) / s);
    }
    static Quaternion fromRotationVector(Vec3 v) {
        const float angle = v.length();
        return angle < 1.0e-7f ? Quaternion {} : Quaternion::fromAxisAngle(v / angle, angle);
    }
    // A mola criticamente amortecida das diferencas da troca (meia-vida 0,1 s; Holden, "spring-it-on"): a diferenca e a
    // velocidade dela vao a zero juntas, sem pulo de posicao nem de velocidade.
    static void decaySpring(Vec3& x, Vec3& v, float dt) {
        constexpr float HalfLife = 0.1f;
        const float y = 2.0f * 0.69314718f / HalfLife;
        const Vec3 j = v + x * y;
        const float e = std::exp(-y * dt);
        x = (x + j * dt) * e;
        v = (v - j * (y * dt)) * e;
    }
    void decaySequenceOffsets(World& w, float dt) const {
        for (std::size_t i = 0; i < w.mmOffsetCoordinates.size() && i < w.seqOffsetCoordinateVelocity.size(); ++i)
            decaySpring(w.mmOffsetCoordinates[i], w.seqOffsetCoordinateVelocity[i], dt);
        decaySpring(w.mmOffsetRootPosition, w.seqOffsetRootVelocity, dt);
        decaySpring(w.seqOffsetSpin, w.seqOffsetSpinVelocity, dt);
    }
    // A transicao: o quadro do clipe mais as diferencas que restam (juntas, pelve, giro da pelve).
    void applySequenceInertia(const World& w, TrackingClipFrame3D& frame) const {
        if (w.mmOffsetCoordinates.size() != frame.coordinates.size()) return;
        std::vector<Vec3> coordinates = frame.coordinates;
        for (std::size_t i = 0; i < coordinates.size(); ++i) coordinates[i] = coordinates[i] + w.mmOffsetCoordinates[i];
        const PhysicsBodyState3D root = frame.links[rootLink];
        const std::vector<PhysicsBodyState3D> velocities = frame.links;
        rlBodyLinkStates3D(*profile, root.position + w.mmOffsetRootPosition,
            (fromRotationVector(w.seqOffsetSpin) * root.orientation).normalized(),
            root.linearVelocity, root.angularVelocity, coordinates, std::vector<Vec3>(coordinates.size()), frame.links);
        for (std::size_t i = 0; i < frame.links.size() && i < velocities.size(); ++i) {
            frame.links[i].linearVelocity = velocities[i].linearVelocity;
            frame.links[i].angularVelocity = velocities[i].angularVelocity;
        }
        frame.coordinates = std::move(coordinates);
    }

    // Uma decisao do cinematico (como o no de Motion Matching do Game Animation Sample): o clipe avanca e a raiz anda com
    // ele; a busca (a consulta desta decisao, os bancos da escolha, o continuar, os quadros recentes e - o "nao reescolher" -
    // o resto do clipe que toca fora) troca de clipe quando outro custa menos; na troca, a inercia: a pose que estaria na
    // tela agora menos a do clipe novo, sumindo em 0,3 s (sem pulo, mesmo com trocas seguidas); o corpo vai a pose, sem fisica.
    void kinematicStep(World& w) {
        constexpr float InertiaSeconds = 0.3f, ReselectSeconds = 0.3f;
        const float dt = TicksPerDecision * TickSeconds;
        const auto decay = [](float elapsed) {
            const float x = std::clamp(elapsed / InertiaSeconds, 0.0f, 1.0f);
            return 1.0f - x * x * x * (10.0f - 15.0f * x + 6.0f * x * x);
        };
        // A pose com a inercia que resta: as coordenadas das juntas do clipe tocando mais a diferenca guardada, encolhendo, e a
        // pelve idem; o corpo remontado pelas juntas (a velocidade de cada link, a do clipe).
        const auto withInertia = [&](TrackingClipFrame3D& frame, float elapsed) {
            const float k = decay(elapsed);
            if (k <= 0.0f || w.mmOffsetCoordinates.size() != frame.coordinates.size()) return;
            const Quaternion yaw = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw);
            std::vector<Vec3> coordinates = frame.coordinates;
            for (std::size_t i = 0; i < coordinates.size(); ++i) coordinates[i] = coordinates[i] + w.mmOffsetCoordinates[i] * k;
            const PhysicsBodyState3D root = frame.links[rootLink];
            Quaternion o = w.mmOffsetRootRotation;
            if (o.w < 0.0f) o = { -o.x, -o.y, -o.z, -o.w };
            const Quaternion part = Quaternion { o.x * k, o.y * k, o.z * k, 1.0f + (o.w - 1.0f) * k }.normalized();
            const std::vector<PhysicsBodyState3D> velocities = frame.links;
            rlBodyLinkStates3D(*profile, root.position + yaw.rotate(w.mmOffsetRootPosition * k), (part * root.orientation).normalized(),
                root.linearVelocity, root.angularVelocity, coordinates, std::vector<Vec3>(coordinates.size()), frame.links);
            for (std::size_t i = 0; i < frame.links.size() && i < velocities.size(); ++i) {
                frame.links[i].linearVelocity = velocities[i].linearVelocity;
                frame.links[i].angularVelocity = velocities[i].angularVelocity;
            }
            frame.coordinates = std::move(coordinates);
        };
        if (!w.mmPlaying) {
            MotionMatchingPose3D pose;
            motionReader.read(w.body->links, pose);
            w.mmRootPosition = pose.rootPosition;
            w.mmRootYaw = pose.rootYaw;
        } else {
            const std::size_t ref = motionRefs[w.mmGroup][w.mmSource];
            const bool moving = w.commandSpeed > 0.05f || w.mmCapsuleVelocity.lengthSquared() > 0.04f;
            if (moving) {
                // Em movimento: a capsula anda pelo modelo de movimento (o mesmo da previsao) a partir do pedido; o rumo dela
                // vai ao pedido na hora, como o personagem do Game Animation Sample no chao.
                if (!w.mmCapsule) {
                    const auto [p0, y0] = clipRoot(ref, w.mmTime);
                    const auto [p1, y1] = clipRoot(ref, w.mmTime + dt);
                    w.mmCapsuleVelocity = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw - y0).rotate(p1 - p0) / dt;
                    w.mmCapsuleVelocity.z = 0.0f;
                }
                const bool commanded = w.commandSpeed > 0.05f;
                const float move = moveDirection(w);
                MotionMatchingMovementState3D state;
                state.position = w.mmRootPosition;
                state.velocity = w.mmCapsuleVelocity;
                state.facingYaw = w.mmRootYaw;
                state.desiredDirection = commanded ? Vec3 { std::cos(move), std::sin(move), 0.0f } : Vec3 {};
                state.desiredSpeed = commanded ? w.commandSpeed : 0.0f;
                state.hasDesiredFacing = true;
                state.desiredFacingYaw = faceDirection(w);
                const float step[1] { dt };
                const std::vector<MotionMatchingTrajectoryPoint3D> next = predictMotionTrajectory3D(motionMovement, state, step);
                if (!next.empty()) {
                    w.mmCapsuleYawRate = std::remainder(next[0].yaw - w.mmRootYaw, 2.0f * Pi) / dt;
                    w.mmRootPosition = { next[0].position.x, next[0].position.y, 0.0f };
                    w.mmRootYaw = next[0].yaw;
                    w.mmCapsuleVelocity = next[0].velocity;
                }
                w.mmCapsule = true;
            } else {
                // Parado e giro: a raiz segue o clipe (o giro no lugar gira pela raiz dele).
                const auto [p0, y0] = clipRoot(ref, w.mmTime);
                const auto [p1, y1] = clipRoot(ref, w.mmTime + dt);
                const Quaternion q = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, w.mmRootYaw - y0);
                w.mmRootPosition = w.mmRootPosition + q.rotate(p1 - p0);
                w.mmRootYaw += y1 - y0;
                w.mmCapsule = false;
        w.seqKind = -1;
        w.seqUntil = -1.0f;
        w.seqTurnFrom = -1.0f;
        w.starValid = false;
                w.mmCapsuleVelocity = {};
            }
            w.mmTime += dt;
            const float duration = references[ref]->durationSeconds();
            if (clips[ref]->loops && duration > 0.0f && w.mmTime >= duration) w.mmTime -= duration;
            w.mmInertiaElapsed += dt;
        }
        if (w.motionRaw.size() == motionDb->groupCount()) {
            std::erase_if(w.mmRecent, [&](const auto& r) { return w.time - r.second > ReselectSeconds; });
            std::vector<std::size_t> excluded;
            for (const auto& r : w.mmRecent) excluded.push_back(r.first);
            MotionMatchingSearchOptions3D options;
            options.groups = w.motionGroups;
            options.continuingIndex = w.mmPlaying ? motionDb->entryAt(w.mmGroup, w.mmSource, w.mmTime) : MotionMatchingResult3D::None;
            // (O "nao reescolher" do Game Animation Sample - em quase todo clipe de locomocao, menos o giro no lugar.)
            if (options.continuingIndex != MotionMatchingResult3D::None && motionCategory[w.mmGroup] != 1) {
                const auto [first, last] = motionDb->sourceEntries(w.mmGroup, w.mmSource);
                for (std::size_t i = first; i < last; ++i) excluded.push_back(i);
            }
            options.excluded = excluded;
            MotionMatchingResult3D found = motionDb->search(w.motionRaw, options);
            // (Diagnostico, temporario: MATTER_MM_SEM_TROCA=1 so troca quando o clipe acaba - a regua dos pulos.)
            if (std::getenv("MATTER_MM_SEM_TROCA") && options.continuingIndex != MotionMatchingResult3D::None)
                found = { options.continuingIndex, 0.0f, true };
            if (options.continuingIndex != MotionMatchingResult3D::None) w.mmRecent.push_back({ options.continuingIndex, w.time });
            if (found.valid() && !found.continuing) {
                const MotionMatchingEntry3D& e = motionDb->entries()[found.index];
                if (w.mmPlaying) {
                    // A inercia: a pose que estaria na tela agora (o clipe antigo com a inercia que resta) menos a do novo.
                    TrackingClipFrame3D shown, next;
                    placedFrame(w, w.mmGroup, w.mmSource, w.mmTime, shown);
                    withInertia(shown, w.mmInertiaElapsed);
                    placedFrame(w, e.group, e.source, e.timeSeconds, next);
                    const Quaternion back = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, -w.mmRootYaw);
                    w.mmOffsetCoordinates.assign(next.coordinates.size(), {});
                    for (std::size_t i = 0; i < next.coordinates.size() && i < shown.coordinates.size(); ++i)
                        w.mmOffsetCoordinates[i] = shown.coordinates[i] - next.coordinates[i];
                    w.mmOffsetRootPosition = back.rotate(shown.links[rootLink].position - next.links[rootLink].position);
                    w.mmOffsetRootRotation = (shown.links[rootLink].orientation * next.links[rootLink].orientation.conjugate()).normalized();
                    w.mmInertiaElapsed = 0.0f;
                }
                ++w.mmJumps;
                w.mmGroup = e.group;
                w.mmSource = e.source;
                w.mmTime = e.timeSeconds;
                w.mmPlaying = true;
                w.mmRecent.push_back({ found.index, w.time });
            }
        }
        if (!w.mmPlaying) return;
        TrackingClipFrame3D now;
        placedFrame(w, w.mmGroup, w.mmSource, w.mmTime, now);
        w.mmPose = now.links;
        withInertia(now, w.mmInertiaElapsed);
        w.scene->resetRagdollState(w.handle, now.links);
        w.body = w.scene->ragdollStateView(w.handle);
    }

    // (07/11) A consulta do Motion Matching do mundo agora: o corpo (a pelve no chao, os pes, os eixos), o passado gravado e o
    // futuro previsto pelo pedido (a direcao e a velocidade, o rumo pedido do corpo e o giro deles desde a decisao anterior); e
    // os bancos da escolha do Game Animation Sample (o chooser denso: parado/paradas/giro no lugar sem pedido; partidas, ciclos,
    // pivos e o giro de 180 em movimento - a corrida deles para o trote, o sprint para a corrida), nas velocidades do nosso corpo.
    void motionQuery(World& w) {
        const float dt = TicksPerDecision * TickSeconds;
        // (No cinematico, a pose do clipe que toca, sem a inercia - o UseContinuingPose do PoseSearch.)
        const std::span<const PhysicsBodyState3D> links = kinematicMotion && w.mmPose.size() == w.body->links.size()
            ? std::span<const PhysicsBodyState3D>(w.mmPose) : std::span<const PhysicsBodyState3D>(w.body->links);
        MotionMatchingPose3D pose;
        motionReader.read(links, pose);
        const PhysicsBodyState3D& pelvis = links[rootLink];
        MotionMatchingQueryInput3D q;
        q.rootPosition = pose.rootPosition;
        q.rootYaw = pose.rootYaw;
        q.rootVelocity = { pelvis.linearVelocity.x, pelvis.linearVelocity.y, 0.0f };
        q.rootYawRate = 0.5f * (pelvis.angularVelocity.z + links[motionReader.chest].angularVelocity.z);
        if (kinematicMotion && w.mmCapsule) {
            q.rootPosition = w.mmRootPosition;
            q.rootYaw = w.mmRootYaw;
            q.rootVelocity = w.mmCapsuleVelocity;
            q.rootYawRate = w.mmCapsuleYawRate;
        }
        q.points = pose.points;
        q.axes = pose.axes;
        const std::uint32_t pointLinks[3] { motionReader.feet[0], motionReader.feet[1], motionReader.pelvis };
        for (std::uint32_t link : pointLinks) {
            const PhysicsBodyState3D& s = links[link];
            const Vec3 com = rlLinkCenterOfMass3D(profile->links[link], s);
            q.pointVelocities.push_back(s.linearVelocity + cross(s.angularVelocity, com - s.position));
        }
        w.motionHistory.record(w.time, q.rootPosition, q.rootYaw, q.rootVelocity);
        float past = 0.0f, future = 0.0f;
        for (std::size_t g = 0; g < motionDb->groupCount(); ++g) {
            past = std::max(past, motionDb->group(g).schema.pastHorizonSeconds());
            future = std::max(future, motionDb->group(g).schema.futureHorizonSeconds());
        }
        std::vector<float> pastTimes;
        for (std::size_t g = 0; g < motionDb->groupCount(); ++g)
            for (const MotionMatchingTrajectorySample3D& s : motionDb->group(g).schema.trajectory)
                if (s.offsetSeconds < 0.0f) pastTimes.push_back(s.offsetSeconds);
        std::sort(pastTimes.begin(), pastTimes.end());
        pastTimes.erase(std::unique(pastTimes.begin(), pastTimes.end()), pastTimes.end());
        for (float t : pastTimes) {
            MotionMatchingTrajectoryPoint3D p;
            if (w.motionHistory.sample(t, p)) q.trajectory.push_back(p);
        }
        const bool commanded = w.commandSpeed > 0.05f;
        const float move = moveDirection(w), face = faceDirection(w);
        float moveRate = 0.0f, faceRate = 0.0f;
        if (w.motionPreviousValid) {
            moveRate = std::clamp(std::remainder(move - w.motionPreviousMove, 2.0f * Pi) / dt, -6.0f, 6.0f);
            faceRate = std::clamp(std::remainder(face - w.motionPreviousFace, 2.0f * Pi) / dt, -6.0f, 6.0f);
        }
        w.motionPreviousMove = move;
        w.motionPreviousFace = face;
        w.motionPreviousValid = true;
        MotionMatchingMovementState3D state;
        state.position = q.rootPosition;
        state.velocity = q.rootVelocity;
        state.facingYaw = q.rootYaw;
        state.desiredDirection = commanded ? Vec3 { std::cos(move), std::sin(move), 0.0f } : Vec3 {};
        state.desiredSpeed = commanded ? w.commandSpeed : 0.0f;
        state.desiredDirectionYawRate = commanded ? moveRate : 0.0f;
        state.hasDesiredFacing = true;
        state.desiredFacingYaw = face;
        state.desiredFacingYawRate = faceRate;
        // (Os instantes do futuro de todos os esquemas, mais 0,45 s: a velocidade futura do "esta arrancando".)
        std::vector<float> futureTimes { 0.45f };
        for (std::size_t g = 0; g < motionDb->groupCount(); ++g)
            for (const MotionMatchingTrajectorySample3D& s : motionDb->group(g).schema.trajectory)
                if (s.offsetSeconds > 0.0f) futureTimes.push_back(s.offsetSeconds);
        std::sort(futureTimes.begin(), futureTimes.end());
        futureTimes.erase(std::unique(futureTimes.begin(), futureTimes.end()), futureTimes.end());
        const std::vector<MotionMatchingTrajectoryPoint3D> ahead = predictMotionTrajectory3D(motionMovement, state, futureTimes);
        float futureSpeed = 0.0f;
        for (const MotionMatchingTrajectoryPoint3D& p : ahead) {
            if (std::abs(p.offsetSeconds - 0.45f) < 1.0e-4f) futureSpeed = p.velocity.length();
            else q.trajectory.push_back(p);
        }
        w.motionQuery.assign(motionQueryWidth, 0.0f);
        w.motionRaw.resize(motionDb->groupCount());
        for (std::size_t g = 0; g < motionDb->groupCount(); ++g) {
            std::vector<float>& raw = w.motionRaw[g];
            raw.assign(motionDb->featureSize(g), 0.0f);
            motionDb->queryFeatures(g, q, raw);
            motionDb->weighted(g, raw, std::span<float>(w.motionQuery).subspan(motionQueryOffset[g], raw.size()));
        }
        // (O passado so vale depois de gravado: o comeco do episodio fica fora do juiz por past segundos.)
        w.motionValid = w.time >= past - 1.0e-4f;
        // A escolha (o CHT_PoseSearchDatabases denso; as velocidades x 1,104, o nosso corpo sobre o manequim deles).
        const float speed = std::hypot(q.rootVelocity.x, q.rootVelocity.y);
        const float facingError = std::abs(std::remainder(face - q.rootYaw, 2.0f * Pi));
        const bool moving = commanded && speed > 0.01f;
        const bool sprint = w.runBand || w.band >= 2;
        std::uint32_t categories = 0;
        const auto allow = [&](int category) { categories |= 1u << category; };
        if (moving) {
            const bool starting = futureSpeed >= speed + 1.1f;
            const float velocityAngle = speed > 0.1f
                ? std::abs(std::remainder(move - std::atan2(q.rootVelocity.y, q.rootVelocity.x), 2.0f * Pi)) : 0.0f;
            const bool pivoting = velocityAngle >= 30.0f * Pi / 180.0f;
            if (!sprint) {
                allow(3);
                if (starting) allow(2);
                if (pivoting) allow(4);
                if (facingError >= 130.0f * Pi / 180.0f && speed >= 1.65f) allow(6);
            } else if (pivoting) {
                allow(9);
            } else {
                allow(8);
                allow(7);
            }
        } else {
            const bool turnInPlace = facingError >= 50.0f * Pi / 180.0f;
            if (speed < 0.22f && !turnInPlace) allow(0);
            if (speed >= 0.22f) allow(5);
            if (speed >= 6.05f) allow(10);
            if (turnInPlace) allow(1);
        }
        w.motionGroups = 0;
        for (std::size_t g = 0; g < motionCategory.size(); ++g)
            if (motionCategory[g] >= 0 && ((categories >> motionCategory[g]) & 1u)) w.motionGroups |= std::uint64_t { 1 } << g;
        if (w.motionGroups == 0) w.motionGroups = motionDb->groupCount() >= 64 ? ~std::uint64_t { 0 }
            : (std::uint64_t { 1 } << motionDb->groupCount()) - 1;
    }

    void begin(World& w) {
        w.motionHistory.clear();
        w.motionValid = false;
        w.motionPreviousValid = false;
        w.mmPlaying = false;
        w.mmRecent.clear();
        w.mmPose.clear();
        w.mmOffsetCoordinates.clear();
        w.mmInertiaElapsed = 1.0e9f;
        w.seqOffsetCoordinateVelocity.clear();
        w.seqOffsetRootVelocity = w.seqOffsetSpin = w.seqOffsetSpinVelocity = {};
        w.mmOffsetRootPosition = {};
        w.seqWarpWeight = w.seqWarpAngle = w.seqTravelFix = w.seqTurnRate = 0.0f;
        w.seqTurnScale = 1.0f;
        w.seqLean = w.seqYawRate = w.seqSpeed = w.seqLift = 0.0f;
        w.mmCapsule = false;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackTwist = 0.0f;
        w.runBand = false;
        w.altClip = std::numeric_limits<std::size_t>::max();
        w.maneuver = 0;
        w.nextManeuverAt = -1.0f;
        w.blendClip = std::numeric_limits<std::size_t>::max();
        w.refActive = false;
        w.refAccel = {};
        w.runTurn = -1;
        w.brakeTurn = 0.0f;
        w.brakeTurnAt = w.runAgainAt = -1.0f;
        w.turnAligned = false;
        w.measureAnchored = false;
        w.arcRate = w.arcLeft = 0.0f;
        w.straightUntil = -1.0f;
        w.improviseUntil = -1.0f;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        // (Antes da queda: ate 02/11 a parede e o peso do episodio anterior ficavam na cena enquanto a queda assentava.)
        clearScenery(w);
        w.forcedScenery = settings.evaluationScenery ? std::uniform_int_distribution<int>(0, 7)(w.rng) : -1;
        w.fallPending = false;
        w.fallActive = false;
        w.getUpGuided = false;
        w.idleAfterGetUp = false;
        w.sprintGuided = false;
        w.clipGetUp = false;
        w.getUpGo = false;
        w.goDistance = 0.0f;
        w.goArrivedAt = -1.0f;
        w.clipFarSeconds = w.clipFacingSeconds = 0.0f;
        w.clipHandedAt = -1.0f;
        w.handoffAnchorAt = -1.0f;
        w.clipHandSpeed = w.clipFlightSeconds = 0.0f;
        w.clipHeld = w.clipFreeSeconds = 0.0f;
        w.clipFreeUntil = -1.0f;
        w.clipDeliveredAt = -1.0f;
        w.clipRate = 1.0f;
        w.clipAgilityReward = 0.0f;
        w.getUpCovered = w.getUpTarget = -1.0f;
        w.fireKind = w.fireTiming = w.fireDirection = 0;
        w.fireFraction = w.fireSize = w.fireHoldScale = 0.0f;
        w.fireShoveAt = w.fireDownAt = -1.0f;
        w.riseFirmSeconds = 0.0f;
        w.riseFirmFrom = w.upAt = -1.0f;
        w.attemptFrom = w.retryBehindSeconds = w.attemptPhase = w.attemptPeakHead = 0.0f;
        w.attempts = 1;
        w.imitationAnchored = false;
        // (15/11 13h30, a revisao: o clipe da imitacao no ritmo ficava valido do episodio anterior - so o beginClipGetUp o punha - e o
        // levantar do comando (e a queda que vira comando) era medido contra o de pe do clipe velho sob o corpo deitado: o fim cedo
        // cortava em 0,4 s. No registro: o comando levantava 85-96% ate as 09h e 26-38% desde o fim cedo pela imitacao.)
        w.imitationReference = std::numeric_limits<std::size_t>::max();
        w.imitationLowSeconds = 0.0f;
        w.holdAnchored = w.holdAtHandoff = false;
        w.handoffCalmSeconds = 0.0f;
        w.fireShovesLeft = w.fireShoveDirection = w.fireExtras = 0;
        w.fireAfterStand = w.fireStandHandled = w.fireSlip = false;
        w.fireReleaseFrom = -1.0f;
        w.fireGrabLink = -1;
        w.fireGrabForce = {};
        w.fireGrabFrom = w.fireGrabUntil = -1.0f;
        w.fireWallAngle = 0.0f;
        w.fireWallDistance = w.fireNextTurnAt = -1.0f;
        w.fireShoveSize = w.fireShoveHead = w.fireAbovePitch = 0.0f;
        w.fireShoveForce = {};
        w.fireShoveLink = -1;
        w.fireShoveSeconds = 0.0f;
        w.fireShoveHeight = -1.0f;
        w.comAcceleration = w.comVelocityBefore = {};
        w.comVelocityKnown = false;
        w.fireHold = {};
        w.fireShoveDone = false;
        w.locoFire = false;
        w.locoFireLevel = w.firePerturb = 0.0f;
        w.locoFireNextAt = w.locoFireHoldUntil = -1.0f;
        w.locoFireEvents = 0;
        w.locoFireExamKind = -1;
        w.fireSlipCenter = {};
        w.fireSlipHalf = 0.0f;
        w.locoKeep = w.locoKeepSamples = 0;
        w.crooked = false;
        w.crookedFixedAt = -1.0f;
        w.crookedFixSeconds = 0.0f;
        w.clipFree = false;
        w.clipTerms = {};
        w.rollClip = std::numeric_limits<std::size_t>::max();
        w.rollAgain = 0.0f;
        // (Um sorteio so, como antes: com as fatias novas em 0, os mesmos comecos de antes.)
        // (Tambem na avaliacao, que nao usa o sorteio: as mesmas 64 quedas fixas de antes.)
        const float draw = u(w.rng);
        std::uint32_t kind = 0;
        if (settings.evaluation) {
            // (04/11 15h: o 8, o equilibrio parado com empurroes no nivel do curriculo, sem a mao.)
            // (07/11: o 9 e o 10, o levantar deitado de costas e de brucos - o comeco no clipe, deitado.)
            // (11/11 14h: do 11 ao 14, o levantar pelo clipe - de costas, de brucos, correndo de costas, correndo de brucos.)
            constexpr std::uint32_t Kinds[11] { 0u, 3u, 4u, 2u, 6u, 2u, 7u, 8u, 2u, 1u, 1u };
            kind = settings.evaluationKind >= 11 ? 9u : Kinds[settings.evaluationKind];
        } else {
            // (11/11 14h: o levantar pelo clipe primeiro; com a fatia dele em 0, o sorteio de antes.)
            float edge = settings.clipGetUpFraction;
            if (draw < edge) kind = 9;
            else if (draw < (edge += settings.standingStartFraction)) kind = 2;
            else if (draw < (edge += settings.clipStartFraction)) kind = 1;
            else if (draw < (edge += settings.avoidFraction)) kind = 4;
            else if (draw < (edge += settings.movingFraction)) kind = 3;
            else if (draw < (edge += settings.sprintStartFraction)) kind = 5;
            else if (draw < (edge += settings.locomotionFraction)) kind = 6;
            else if (draw < (edge += settings.turnFraction)) kind = 7;
            else if (draw < (edge += settings.locomotionFallFraction)) kind = 8;
        }
        if (kind == 5 && runStartIndices.empty()) kind = 3;
        if (kind == 9 && clipGetUpClips.empty()) kind = 0;
        // (O evitar ambientado: o ambiente garantido e o chao antes de ele ficar de pe; as paredes e os pilares depois.)
        w.controlledPreset = -1;
        if (kind == 4 && (settings.evaluation ? settings.evaluationScenery : u(w.rng) < settings.avoidSceneryFraction)) {
            constexpr int AvoidScenery[6] { 0, 1, 2, 5, 6, 7 };
            w.forcedScenery = AvoidScenery[std::uniform_int_distribution<int>(0, 5)(w.rng)];
            if (!controlledScenery()) placeAvoidTerrain(w);
        } else if (kind == 4) {
            w.forcedScenery = -1;
        }
        // Six get-up layouts (two corridor clearances), five avoid layouts; contacts get time to retry.
        w.controlledPreset = -1;
        if (controlledScenery()) {
            const bool ambient = settings.sceneryPreset >= 0 || (kind == 4 ? w.forcedScenery >= 0 : ((kind == 0 || kind == 3) && u(w.rng) < 0.65f));
            const int lastPreset = kind == 4 ? 4 : 5;
            if (ambient) w.controlledPreset = settings.sceneryPreset >= 0 ? std::clamp(settings.sceneryPreset, 0, lastPreset)
                : std::uniform_int_distribution<int>(0, lastPreset)(w.rng);
            constexpr int GetUpLayouts[6] { 2, 1, 5, 4, 7, 1 };
            constexpr int AvoidLayouts[5] { 0, 1, 5, 7, 2 };
            w.forcedScenery = ambient ? (kind == 4 ? AvoidLayouts[w.controlledPreset] : GetUpLayouts[w.controlledPreset]) : -1;
        }
        if (controlledScenery() && kind == 4 && w.forcedScenery >= 0) placeAvoidTerrain(w);
        if (kind == 8 && fallStarts.empty()) kind = 0;
        // (O evitar a queda com a parceira: ela de pe primeiro; sem a queda iminente dela, uma queda sorteada.)
        float partnerPushLeft = -1.0f;
        Vec3 partnerPushForce;
        if (kind == 4 && partner && settings.partnerAvoid) {
            w.pushTicks = 0;
            w.projectileTicks = 0;
            beginFromClip(w, true);
            if (!beginPartnerAvoid(w, partnerPushLeft, partnerPushForce)) {
                kind = 0;
                partnerPushLeft = -1.0f;
            }
        }
        // (O que vem do episodio anterior zerado antes do comeco: a queda assenta sem o resto de um empurrao.)
        w.pushTicks = 0;
        w.projectileTicks = 0;
        w.pushLinks = { 0u, 2u, 3u };
        w.pushShares = { 0.20f, 0.32f, 0.48f };
        w.commandSpeed = 0.0f;
        w.locomotion = kind == 6 || kind == 7;
        w.faceErrorSum = 0.0;
        w.faceErrorSamples = 0;
        w.firmSeconds = 0.0f;
        w.handoffAt = -1.0f;
        w.partnerDrove = -1.0f;
        w.partnerFell = false;
        w.touchedDown = false;
        w.protecting = false;
        w.supported = false;
        w.flexUntil = -1.0f;
        w.flexSum = 0.0;
        w.flexSamples = 0;
        w.footAir[0] = w.footAir[1] = 0.0f;
        w.footDown[0] = w.footDown[1] = true;
        w.protectiveStart = false;
        w.teacherSession.reset();
        w.taught = false;
        // (O musculo da aluna nao vaza para o comeco seguinte: a queda assenta sem a gravidade dela - a revisao de 03/11.)
        w.driveGravity = false;
        w.teacherEpisode = false;
        w.teacherUntil = -1.0f;
        w.chestError = 0.0f;
        w.lastTilt = -1.0f;
        w.teacherDecisions = 0;
        w.takeovers = 0;
        w.labeled = 0;
        w.teacherAssist = 0.0f;
        w.travelSum = {};
        w.trackClip = std::numeric_limits<std::size_t>::max();
        w.pendingClip = std::numeric_limits<std::size_t>::max();
        w.startKind = kind;
        // (04/11 15h: o equilibrio parado - o parado com empurroes, sem a mao; sem a fatia, sem sorteio: os comecos de antes.)
        w.balanceDrill = kind == 2 && !settings.evaluation && settings.balanceDrillFraction > 0.0f && u(w.rng) < settings.balanceDrillFraction;
        if (settings.evaluation && settings.evaluationKind == 8) w.balanceDrill = true;
        w.steps.reset();
        w.stepsOn = false;
        w.stepPoseValid = false;
        w.pushes = w.recoveryStepCount = w.settleStepCount = 0;
        w.finalStopAt = -1.0f;
        w.stopping = false;
        w.stopPhase = 0;
        w.stopError = -1.0f;
        w.stopClip = -1;
        w.unrealStart = -1;
        w.launchFootAir[0] = w.launchFootAir[1] = 0.0f;
        w.launchFootDown[0] = w.launchFootDown[1] = true;
        w.launchFootLift[0] = w.launchFootLift[1] = {};
        w.freeStop = false;
        w.freeTroca = false;
        // (08/11, a fase de fogo: no treino, a fatia dos episodios com empurroes; na avaliacao, a regua dela.)
        // (O equilibrio parado e o exercicio dos empurroes: sempre com eles.)
        w.fireEpisode = settings.evaluation || w.balanceDrill || settings.pushEpisodeShare >= 1.0f || u(w.rng) < settings.pushEpisodeShare;
        w.arrivalQuality = -1.0f;
        w.arrivalReward = 0.0f;
        w.worstCaptureOut = 0.0f;
        w.quietAgainAt = -1.0f;
        w.idleSince = -10.0f;
        w.idleFaceSet = false;
        w.basePrevDistance = -1.0f;
        w.baseFootDown[0] = w.baseFootDown[1] = true;
        // (O relogio do episodio zerado antes dos comecos: o giro e os rumos marcam o clipe a partir dele.)
        w.time = 0.0f;
        w.trackScale = 1.0f;
        w.rampFrom = -1.0f;
        w.trackingAverage = 1.0f;
        w.locomotionFeedback = {};
        w.headingError = 0.0f;
        w.headingBias = 0.0f;
        w.lateralBias = {};
        w.turnedSeconds = 0.0f;
        w.hasReferenceHeading = false;
        w.referenceTurningUntil = -1.0f;
        // (04/11 12h40: antes dos comecos - zerado depois deles, o fim da arrancada que o beginHeading marca se perdia e a
        // janela da arrancada da v10 nunca ligou.)
        w.launchEnd = -1.0f;
        w.rampSeconds = StartRampSeconds;
        w.accelFrom = -1.0f;
        w.accelRate = 0.0f;
        w.easeFrom = -1.0f;
        if (kind == 0 || kind == 3) placeTerrain(w);
        w.lyingClip = std::numeric_limits<std::size_t>::max();
        if (kind == 9) beginClipGetUp(w);
        else if (kind == 3 && !lyingRun.empty()) beginLying(w, lyingRun, -1);
        else if (kind == 0 || kind == 3) beginFall(w);
        else if (kind == 5) beginSprintStart(w);
        else if (kind == 6) beginHeading(w);
        else if (kind == 7) beginTurn(w);
        else if (kind == 8) beginLocomotionFall(w);
        else if (kind == 1 && !lyingStand.empty())
            beginLying(w, lyingStand, settings.evaluation ? static_cast<int>(settings.evaluationKind) - 9 : -1);
        else if (partnerPushLeft < 0.0f) beginFromClip(w, kind != 1);
        // (01/11 19h10, ele: "no exercicio do parado, faca ele se assemelhar ao maximo possivel com o IDLE parado, e
        // recompensar ele pela estabilidade e equilibrio e quietude": no parado o clipe seguido e o parado em alerta, do
        // instante e do rumo do comeco.)
        if ((kind == 2 || (kind == 4 && settings.avoidOnIdle)) && w.lastClip < references.size()) {
            w.trackClip = w.lastClip;
            w.trackStart = w.lastClipTime;
            w.trackYaw = w.lastClipYaw;
        }
        if (((kind == 2 && settings.taughtStanding) || kind == 7) && (teacher || settings.taughtHeadings)) {
            // (O parado: a vista no rumo do clipe parado - o do comeco; o giro: a do giro, ja posta em beginTurn.)
            if (kind == 2) w.faceYaw = w.moveYaw = w.lastClipYaw;
            beginTaughtStill(w);
        }
        const TrackingGetUpPosture3D start = posture->measure(w.body->links, floorUnder(w));
        w.stableSeconds = 0.0f;
        w.idleLostSeconds = 0.0f;
        w.bentSeconds = 0.0f;
        w.recoveryRough = w.recoveryLimits = 0.0f;
        w.idleBlendFrom = -1.0e9f;
        w.freeTarget.clear();
        w.lastRecoveryAction.clear();
        w.riseKnown = false;
        w.lastMargin = w.stumble = 0.0f;
        w.calmFrom = -1.0f;
        w.firmAir = {};
        w.locomotionControl = (w.locomotion || w.taught) && locomotionCommanded(w) && start.standing;
        {
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            w.smoothVelocity = { v.x, v.y, 0.0f };
            w.courseVelocity = w.smoothVelocity;
            w.straySeconds = 0.0f;
        }
        w.startSide = kind == 6 ? w.heading : start.chestFacingUp < -0.5f ? 0u : start.chestFacingUp > 0.5f ? 1u : 2u;
        w.startHead = start.headHeight;
        w.time = 0.0f;
        w.stoodAt = -1.0f;
        w.rewardSum = 0.0;
        w.decisions = 0;
        w.nextPushAt = -1.0f;
        w.lastPushEnd = -1.0e9f;
        w.reachedAt = -1.0f;
        w.towardSum = 0.0;
        w.towardSamples = 0;
        w.wantedSum = 0.0;
        w.imitationSum = w.stabilityDeficitSum = 0.0;
        w.crossStance = w.crossBad = 0;
        w.trackSamples = w.stabilitySamples = 0;
        w.avoidAt = -1.0f;
        w.avoidEnd = -1.0f;
        w.recomposedAt = -1.0f;
        w.idleAnchored = false;
        if (settings.protectiveFall && w.protectiveStart && !w.locomotion) w.fallPending = true;
        if (settings.getUpGuide && !w.locomotion && !w.fallPending && !start.standing && !w.clipGetUp) beginGetUpGuide(w);
        w.down = fallen(start, *w.body);
        w.fellAfterStanding = false;
        w.fallConverted = false;
        w.originalManeuver = 0;
        w.originalKind = 0;
        w.convertedAt = -1.0f;
        if (kind == 3 || kind == 5) {
            // (Na arrancada, o alvo a ate 30 graus da frente do clipe: ele ja sai virado para o alvo.)
            const float angle = kind == 5 ? w.sprintYaw + (2.0f * u(w.rng) - 1.0f) * (Pi / 6.0f) : 2.0f * Pi * u(w.rng);
            const float distance =
                settings.targetMinimumMeters + (settings.targetMaximumMeters - settings.targetMinimumMeters) * u(w.rng);
            const Vec3 root = w.body->links[rootLink].position;
            w.target = { root.x + std::cos(angle) * distance, root.y + std::sin(angle) * distance, 0.0f };
            w.commandSpeed = settings.moveMinimumSpeed + (settings.moveMaximumSpeed - settings.moveMinimumSpeed) * u(w.rng);
        }
        if (w.clipGetUp) {
            // (O levantar pelo clipe: parado, sem pedido; correndo, o pedido e a velocidade do fim do clipe, no rumo dele no mundo.)
            const Vec3 root = w.body->links[rootLink].position;
            if (clipRunExercise(w.clipExercise)) {
                const float direction = w.getUpGo ? std::atan2(w.goTarget.y - root.y, w.goTarget.x - root.x)
                                                  : w.clipYaw + clipGetUpDirection[w.clipPoseExercise];
                w.target = { root.x + std::cos(direction) * ClipRunTargetMeters, root.y + std::sin(direction) * ClipRunTargetMeters, 0.0f };
                w.commandSpeed = clipGetUpSpeed[w.clipPoseExercise];
            } else {
                w.target = { root.x, root.y, 0.0f };
                w.commandSpeed = 0.0f;
            }
        }
        w.startCommandSpeed = w.commandSpeed;
        if (kind == 4) w.avoidAt = 0.3f + 0.7f * u(w.rng);
        w.obstructed = false;
        beginLocomotionFire(w);
        placeObstacles(w);
        // (Todos os de locomocao, de pe: tambem o comando e o parado que comecam sem pedido - o locomotionControl so liga com ele.)
        if (settings.locomotionSceneryFraction > 0.0f && !settings.evaluation && w.locomotion && !w.down
            && std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng) < settings.locomotionSceneryFraction)
            placeLocomotionScenery(w);
        if (partnerPushLeft >= 0.0f) {
            w.avoidAt = -1.0f;
            w.avoidEnd = w.lastPushEnd = partnerPushLeft;
            if (partnerPushLeft > 0.0f) {
                w.pushForce = partnerPushForce;
                w.pushTicks = std::max(1, static_cast<int>(std::lround(partnerPushLeft / TickSeconds)));
            }
        }
    }

    // O empurrao de pe (31/10 09h, ele: depois de levantar falta o "jogo de pes de auto-equilibrio"): so no treino, nos
    // comecos sem alvo e sem o empurrao forte, depois de ja ter ficado de pe e enquanto esta de pe, um empurrao curto a
    // cada 1,5-3 s (o primeiro 1-2,5 s depois de ficar de pe), na faixa da configuracao por 0,1-0,2 s, num rumo
    // sorteado (150-450 N ate 12h35, 250-650 ate 15h05, 400-900 depois; cada degrau quando ele ja ficava de pe em ~100%).
    // Se cair, levanta de novo.
    // (31/10 21h25, ele: "botar um treino nele, para parado e aguentar empurroes medianos e fortes e ver como ele se sai
    // com auto-equilibrio": a faixa e o continuo da configuracao; na avaliacao do equilibrio parado, um so empurrao com a
    // regua do parado com empurrao forte do B21, para comparar as duas redes.)
    // (31/10 22h35, a locomocao: nos rumos e no giro, os empurroes em movimento, o primeiro de 1 a 3 s depois do comeco
    // e os outros a cada 2-4 s; na avaliacao, um so, de 2 a 3 s depois do comeco, com a velocidade ja pega.)
    // (08/11, ele: "garanta que ele fique muito bom com auto equilibrio... se recuperar das adversidades de forma muito
    // versatil") Com pushVariety, o ponto do empurrao sorteado: metade o de sempre (o tronco repartido), 20% so no peito alto
    // (o alto tomba), 15% so na pelve (o baixo escorrega a base), 15% num ombro (torce o tronco).
    // (08/11 12h30) O grupo do fogo de um episodio: 0 a frente, 1 os lados, 2 as costas (os retos pelo rumo), 3 o giro, 4 o
    // parado, 5 o resto (trocas, curvas, estrela, o de pe depois de levantar).
    std::size_t fireGroup(const World& w) const {
        if (w.startKind == 7) return 3;
        if (w.startKind == 2) return 4;
        if (w.startKind == 6 && w.maneuver == 0) {
            const std::uint32_t h = w.heading % 8u;
            return h == 2u || h == 6u ? 1 : (h >= 3u && h <= 5u) ? 2 : 0;
        }
        return 5;
    }
    void pickPushPoint(World& w) const {
        w.pushLinks = { 0u, 2u, 3u };
        w.pushShares = { 0.20f, 0.32f, 0.48f };
        if (!settings.pushVariety) return;
        const float draw = std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng);
        // (18h05, ele: "gostei dessa ideia de o local da pancada/empurrao ser variado": e as coxas - a entrada na perna do futebol.)
        if (draw < 0.4f) return;
        w.pushShares = { 1.0f, 0.0f, 0.0f };
        if (draw < 0.58f) w.pushLinks[0] = 3u;
        else if (draw < 0.73f) w.pushLinks[0] = 0u;
        else if (draw < 0.87f) w.pushLinks[0] = draw < 0.80f ? leftShoulderLink : rightShoulderLink;
        else w.pushLinks[0] = draw < 0.935f ? leftThighLink : rightThighLink;
        if (w.pushLinks[0] >= profile->links.size()) w.pushLinks[0] = 3u;
    }
    void maybePush(World& w, const TrackingGetUpPosture3D& posture) {
        // (12/11 16h30: com o fogo da locomocao, so ele empurra.)
        if (w.locoFire) return;
        // (O parado e um exercicio do IDLE, sem empurrao - a avaliacao 3 tambem; 04/11 15h: o equilibrio parado - no treino e na
        // avaliacao 8 - com.)
        if (w.startKind == 2 && !w.locomotion && !w.balanceDrill) return;
        // (08/11 14h55, ele: "no treino do comando nao precisa ter os empurroes".)
        if (isCommand(w)) return;
        // (18h05: com o fireShown - o laboratorio -, a avaliacao empurra como o treino.)
        const bool trainingLike = !settings.evaluation || settings.fireShown;
        const bool balanceTest = !trainingLike && settings.evaluationKind == 3;
        const bool movingTest = !trainingLike && w.locomotion;
        const bool movingPushes = settings.movingPushMaximumNewtons > 0.0f || settings.movingPushHoldMaximumNewtons > 0.0f
            || (settings.fireGrouped && trainingLike);
        if ((!trainingLike && !balanceTest && !movingTest && !w.balanceDrill) || (w.startKind >= 3 && !w.locomotion)
            || (w.locomotion && !movingPushes) || w.stoodAt < 0.0f || !posture.standing || w.pushTicks > 0 || !w.fireEpisode)
            return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        if (w.nextPushAt < 0.0f) {
            w.nextPushAt = w.time + (movingTest ? 2.0f + u(w.rng) : w.locomotion ? 1.0f + 2.0f * u(w.rng) : 1.0f + 1.5f * u(w.rng));
            return;
        }
        if (w.time < w.nextPushAt) return;
        const float angle = 2.0f * Pi * u(w.rng);
        float newtons = 0.0f, seconds = 0.0f;
        if (balanceTest) {
            if (u(w.rng) < 0.3f) newtons = 205.0f + 70.0f * u(w.rng), seconds = 1.0f + 2.0f * u(w.rng);
            else newtons = 900.0f + 300.0f * u(w.rng), seconds = 0.1f + 0.1f * u(w.rng);
        } else if (settings.fireGrouped && trainingLike) {
            // (08/11 12h30: o fogo por grupo - fireGroupBands: as faixas de cada grupo, no nivel dele.)
            const std::array<float, 4>& band = settings.fireGroupBands[fireGroup(w)];
            const float holdShare = w.locomotion ? settings.movingPushHoldFraction : settings.pushHoldFraction;
            if (u(w.rng) < holdShare) {
                newtons = band[2] + (band[3] - band[2]) * u(w.rng);
                seconds = settings.movingPushHoldMinimumSeconds
                    + (settings.movingPushHoldMaximumSeconds - settings.movingPushHoldMinimumSeconds) * u(w.rng);
            } else {
                newtons = band[0] + (band[1] - band[0]) * u(w.rng);
                seconds = 0.1f + 0.1f * u(w.rng);
            }
        } else if (w.locomotion) {
            if (u(w.rng) < settings.movingPushHoldFraction) {
                newtons = settings.movingPushHoldMinimumNewtons
                    + (settings.movingPushHoldMaximumNewtons - settings.movingPushHoldMinimumNewtons) * u(w.rng);
                seconds = settings.movingPushHoldMinimumSeconds
                    + (settings.movingPushHoldMaximumSeconds - settings.movingPushHoldMinimumSeconds) * u(w.rng);
            } else {
                newtons = settings.movingPushMinimumNewtons
                    + (settings.movingPushMaximumNewtons - settings.movingPushMinimumNewtons) * u(w.rng);
                seconds = 0.1f + 0.1f * u(w.rng);
            }
        } else if (u(w.rng) < settings.pushHoldFraction) {
            newtons = settings.pushHoldMinimumNewtons
                + (settings.pushHoldMaximumNewtons - settings.pushHoldMinimumNewtons) * u(w.rng);
            seconds = settings.pushHoldMinimumSeconds
                + (settings.pushHoldMaximumSeconds - settings.pushHoldMinimumSeconds) * u(w.rng);
        } else {
            newtons = settings.pushMinimumNewtons + (settings.pushMaximumNewtons - settings.pushMinimumNewtons) * u(w.rng);
            seconds = 0.1f + 0.1f * u(w.rng);
        }
        // (O equilibrio parado: o nivel do curriculo, ate a faixa inteira.)
        // (05/11 02h, ele: "o PORQUE ele nao esta conseguindo ficar em pe... e corrija isso": os empurroes de pe depois de
        // levantar vinham na faixa inteira, 400-1400 N por 0,1-0,2 s, 1-2,5 s depois de ficar de pe - 86% caiam depois de ja
        // de pe no treino, contra 3% sem empurrao na avaliacao. Agora no mesmo nivel do curriculo do equilibrio parado.)
        if (!w.locomotion && settings.balancePushLevel > 0.0f && !(settings.fireGrouped && trainingLike))
            newtons *= std::clamp(settings.balancePushLevel, 0.05f, 1.0f);
        ++w.pushes;
        w.quietAgainAt = -1.0f;
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        pickPushPoint(w);
        w.pushTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
        w.lastPushEnd = w.time + seconds;
        w.nextPushAt = balanceTest || movingTest ? 1.0e9f
            : w.time + seconds + (w.locomotion ? 2.0f + 2.0f * u(w.rng) : 1.5f + 1.5f * u(w.rng));
    }

    // O empurrao forte do evitar a queda (31/10 17h, ele: "comeca de pe, leva um empurrao que com certeza vai leva-lo ao
    // chao e ai ele deve treinar conseguir evitar aquela queda e recompor-se o mais rapido possivel"): uma vez, de 0,3 a
    // 1,0 s depois do comeco, o curto ou (numa fatia) o continuo, num rumo sorteado. Tambem na avaliacao.
    void maybeAvoidPush(World& w) {
        if (w.startKind != 4 || w.avoidAt < 0.0f || w.time < w.avoidAt || w.pushTicks > 0) return;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        const float angle = 2.0f * Pi * u(w.rng);
        float newtons = 0.0f, seconds = 0.0f;
        if (u(w.rng) < settings.avoidHoldFraction) {
            newtons = settings.avoidHoldMinimumNewtons
                + (settings.avoidHoldMaximumNewtons - settings.avoidHoldMinimumNewtons) * u(w.rng);
            seconds = settings.avoidHoldMinimumSeconds
                + (settings.avoidHoldMaximumSeconds - settings.avoidHoldMinimumSeconds) * u(w.rng);
        } else {
            newtons = settings.avoidMinimumNewtons + (settings.avoidMaximumNewtons - settings.avoidMinimumNewtons) * u(w.rng);
            seconds = 0.1f + 0.1f * u(w.rng);
        }
        w.pushForce = { std::cos(angle) * newtons, std::sin(angle) * newtons, 0.0f };
        pickPushPoint(w);
        w.pushTicks = std::max(1, static_cast<int>(std::lround(seconds / TickSeconds)));
        w.avoidEnd = w.time + seconds;
        w.lastPushEnd = w.avoidEnd;
        w.avoidAt = -1.0f;
    }

    // O rumo do alvo em relacao ao corpo (rad) e a distancia (m), no chao.
    std::pair<float, float> targetBearing(const World& w) const {
        const PhysicsBodyState3D& root = w.body->links[rootLink];
        const Vec3 to { w.target.x - root.position.x, w.target.y - root.position.y, 0.0f };
        const float yaw = trackingBodyYaw3D(profile->links[rootLink], root);
        return { wrapAngle(planarAngle(to) - yaw), std::hypot(to.x, to.y) };
    }

    // O rumo do deslocamento pedido e o do corpo (rad, no mundo): no levantar em movimento, os dois para o alvo; nos
    // rumos, o pedido e o do comeco; no giro, o que gira.
    float moveDirection(const World& w) const {
        if (w.locomotion) return w.moveYaw;
        const PhysicsBodyState3D& root = w.body->links[rootLink];
        return planarAngle({ w.target.x - root.position.x, w.target.y - root.position.y, 0.0f });
    }
    float faceDirection(const World& w) const { return w.locomotion ? w.faceYaw : moveDirection(w); }
    float bodyYaw(const World& w) const { return trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink]); }
    // (15/11, settings.getUpViewReward) O levantar de pe com vista, antes da troca: o levantar e ir de pe e o levantar do comando.
    bool getUpViewOn(const World& w) const {
        return settings.getUpViewReward > 0.0f && w.clipGetUp && w.clipHandedAt < 0.0f && !clipRunExercise(w.clipExercise)
            && (w.getUpGo || isCommand(w));
    }
    // (15/11, settings.getUpProneTimeScale) A escala do tempo do levantar deste mundo: so o de brucos de pe (o exercicio 1).
    float getUpTimeScale(const World& w) const {
        return w.clipExercise == 1u && !clipRunExercise(w.clipExercise) ? settings.getUpProneTimeScale : 1.0f;
    }
    // (01/11 00h, o Codex: no giro, max(0, cos) fica plano alem de 90 graus - o alvo gira e foge, e dai ele nao tinha
    // sinal nenhum para virar; o erro ficou em ~95 graus por 1,5 h. No giro, o alinhamento continuo (1 + cos) / 2.)
    // (01/11 22h, ele: "o da direita e esquerda, a maioria dos movimentos ele faz indo para diagonal... isso deveria ser
    // punido": a velocidade de lado ao rumo pedido, na proporcao da pedida, corta a nota - 30% de lado, 76%; 50%, 47%.)
    float sideDrift(const World& w, Vec3 v) const {
        const float toward = moveDirection(w);
        const float across = (-v.x * std::sin(toward) + v.y * std::cos(toward)) / std::max(0.5f, w.commandSpeed);
        return std::exp(-3.0f * across * across);
    }
    // A frente do corpo nos rumos: a pelve menos o rumo medio dela no clipe seguido (seguir o clipe nao custa rumo).
    float characterYaw(const World& w) const {
        return w.locomotion && w.trackClip < yawOffsets.size() ? wrapAngle(bodyYaw(w) - yawOffsets[w.trackClip]) : bodyYaw(w);
    }
    // O rumo do peito no plano (a frente do link do peito). (03/11 17h40, ele: "o peito precisa estar virado sempre para
    // o ponto de vista ... apontado para o alvo de visao" - na locomocao sem o rastreio a frente cobrada e a do peito,
    // sem desconto do clipe, e o portao e o cosseno a quarta: 20 graus, 78%; 45, 25%.)
    float chestYaw(const World& w) const {
        const std::size_t c = contract->chestLink();
        const Vec3 f = (w.body->links[c].orientation * profile->links[c].modelOrientation.conjugate()).normalized()
                           .rotate({ 1.0f, 0.0f, 0.0f });
        return std::atan2(f.y, f.x);
    }
    // (13/11 11h30) A forma da cabeca e do peito nos eixos do corpo (x a frente, z o alto): a cabeca contra o peito e o peito contra a pelve.
    GetUpFormSample upperForm(const std::vector<PhysicsBodyState3D>& links, float floor) const {
        const auto axes = [&](std::size_t k) { return (links[k].orientation * profile->links[k].modelOrientation.conjugate()).normalized(); };
        const std::size_t head = contract->headLink();
        const Quaternion p = axes(rootLink), c = axes(upperChestLink), h = axes(head);
        const Vec3 cx = c.rotate({ 1.0f, 0.0f, 0.0f }), cy = c.rotate({ 0.0f, 1.0f, 0.0f }), cz = c.rotate({ 0.0f, 0.0f, 1.0f });
        const Vec3 hz = h.rotate({ 0.0f, 0.0f, 1.0f }), hx = h.rotate({ 1.0f, 0.0f, 0.0f }), px = p.rotate({ 1.0f, 0.0f, 0.0f }),
                   py = p.rotate({ 0.0f, 1.0f, 0.0f }), pz = p.rotate({ 0.0f, 0.0f, 1.0f });
        constexpr float Deg = 180.0f / Pi;
        return { rlLinkCenterOfMass3D(profile->links[head], links[head]).z - floor, Deg * std::atan2(dot(hz, cx), dot(hz, cz)),
            Deg * std::atan2(dot(hz, cy), dot(hz, cz)), Deg * std::atan2(dot(cx, py), dot(cx, px)),
            Deg * std::atan2(dot(cz, px), dot(cz, pz)), Deg * std::asin(std::clamp(hx.z, -1.0f, 1.0f)),
            Deg * std::atan2(dot(cz, Vec3 { px.x, px.y, 0.0f }) / std::max(1.0e-4f, std::hypot(px.x, px.y)), cz.z) };
    }
    // A forma (0-1) contra a media do clipe na mesma altura da cabeca (+-0,1 m; senao a amostra mais perto), com folga de 8 graus: a cabeca
    // nao mais para tras que o clipe, nao mais de lado, o peito nao mais torcido.
    float getUpFormScore(const World& w, const std::vector<PhysicsBodyState3D>& links, float floor, bool standing) const {
        if (w.clipExercise >= clipGetUpForm.size() || clipGetUpForm[w.clipExercise].empty()) return 1.0f;
        const GetUpFormSample now = upperForm(links, floor);
        float pitch = 0.0f, roll = 0.0f, twist = 0.0f, lean = 0.0f, gaze = 0.0f, tilt = 0.0f, nearest = 1.0e9f;
        int n = 0;
        const GetUpFormSample* best = nullptr;
        for (const GetUpFormSample& s : clipGetUpForm[w.clipExercise]) {
            const float d = std::abs(s.head - now.head);
            if (d < nearest) nearest = d, best = &s;
            if (d <= 0.1f)
                pitch += s.pitch, roll += std::abs(s.roll), twist += std::abs(s.twist), lean += s.lean, gaze += s.gaze, tilt += s.tilt, ++n;
        }
        if (n == 0)
            pitch = best->pitch, roll = std::abs(best->roll), twist = std::abs(best->twist), lean = best->lean, gaze = best->gaze,
            tilt = best->tilt, n = 1;
        const float slack = 8.0f, back = std::max(0.0f, pitch / n - slack - now.pitch) / 20.0f,
                    side = std::max(0.0f, std::abs(now.roll) - roll / n - slack) / 15.0f,
                    turn = std::max(0.0f, std::abs(now.twist) - twist / n - slack) / 15.0f,
                    arch = std::max(0.0f, lean / n - slack - now.lean) / 15.0f, up = std::max(0.0f, now.gaze - gaze / n - slack) / 15.0f,
                    bent = std::max(0.0f, std::abs(now.tilt - tilt / n) - 10.0f) / 15.0f;
        const float score = 0.15f * std::exp(-back * back) + 0.15f * std::exp(-side * side) + 0.15f * std::exp(-turn * turn)
            + 0.15f * std::exp(-arch * arch) + 0.20f * std::exp(-up * up) + 0.20f * std::exp(-bent * bent);
        // (12h50) De pe nos levantares de pe: tambem o fim do clipe - o peito nao mais curvado (folga de 8 graus) e a cabeca na altura
        // (folga de 4 cm): ficar de pe agachado e curvado "travado" custa.
        if (!standing || clipRunExercise(w.clipExercise) || w.clipExercise >= clipGetUpStand.size()) return score;
        const std::array<float, 2>& end = clipGetUpStand[w.clipExercise];
        const float stoop = std::max(0.0f, now.tilt - end[1] - 8.0f) / 12.0f, low = std::max(0.0f, end[0] - 0.04f - now.head) / 0.05f;
        return 0.6f * score + 0.2f * std::exp(-stoop * stoop) + 0.2f * std::exp(-low * low);
    }
    // (14/11 20h10) A altura da cabeca e a da pelve e o joelho e o pe esquerdos contra a pelve, como se ve de fora (20h17: nos eixos da
    // pelve a inclinacao dela entrava junto - 20 graus mudavam o pe em ~25 cm com a perna igual): x a frente do corpo no plano (a de lado
    // da pelve, deitado ou de pe, girada no plano), y a esquerda no plano, z a vertical.
    std::array<float, 8> leftLegForm(const std::vector<PhysicsBodyState3D>& links, float floor) const {
        std::array<float, 8> out {};
        const std::size_t head = contract->headLink();
        const Vec3 base = rlLinkCenterOfMass3D(profile->links[rootLink], links[rootLink]);
        out[0] = rlLinkCenterOfMass3D(profile->links[head], links[head]).z - floor;
        out[1] = base.z - floor;
        const Quaternion p = (links[rootLink].orientation * profile->links[rootLink].modelOrientation.conjugate()).normalized();
        const Vec3 py = p.rotate({ 0.0f, 1.0f, 0.0f });
        const float side = std::max(1.0e-4f, std::hypot(py.x, py.y));
        const Vec3 left { py.x / side, py.y / side, 0.0f }, ahead { left.y, -left.x, 0.0f };
        for (std::size_t j = 0; j < 2; ++j) {
            const std::size_t k = leftLegLinks[j];
            if (k >= links.size()) continue;
            const Vec3 d = rlLinkCenterOfMass3D(profile->links[k], links[k]) - base;
            out[2 + 3 * j] = dot(d, ahead), out[3 + 3 * j] = dot(d, left), out[4 + 3 * j] = d.z;
        }
        return out;
    }
    // (14/11 20h20) O erro da perna esquerda (m) e o que ele comparou: a media do joelho e do pe contra a media do clipe na MESMA POSE de
    // alturas - a cabeca e a pelve a menos de 8 cm (no clipe inteiro, como a forma da cabeca: nao pelo relogio - a rede sobe em ~1,2 s e
    // o clipe em ~1,8 s; a pelve separa o sentado do ajoelhado); senao a amostra mais perto. out: o corpo (8), a media do clipe (6), n.
    // < 0: sem tabela.
    float getUpLegError(const World& w, const std::vector<PhysicsBodyState3D>& links, float floor, std::vector<float>* out = nullptr) const {
        if (w.clipExercise >= clipGetUpLeg.size() || clipGetUpLeg[w.clipExercise].empty()) return -1.0f;
        const std::vector<std::array<float, 8>>& table = clipGetUpLeg[w.clipExercise];
        const std::array<float, 8> now = leftLegForm(links, floor);
        std::array<float, 6> mean {};
        int n = 0;
        std::size_t best = 0;
        float nearest = 1.0e9f;
        for (std::size_t i = 0; i < table.size(); ++i) {
            const float d = std::hypot(table[i][0] - now[0], table[i][1] - now[1]);
            if (d < nearest) nearest = d, best = i;
            if (d <= 0.08f) {
                for (int c = 0; c < 6; ++c) mean[c] += table[i][2 + c];
                ++n;
            }
        }
        if (n == 0) {
            for (int c = 0; c < 6; ++c) mean[c] = table[best][2 + c];
            n = 1;
        }
        for (float& m : mean) m /= static_cast<float>(n);
        float err = 0.0f;
        for (int j = 0; j < 2; ++j) {
            const float dx = now[2 + 3 * j] - mean[3 * j], dy = now[3 + 3 * j] - mean[1 + 3 * j], dz = now[4 + 3 * j] - mean[2 + 3 * j];
            err += 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        if (out) {
            out->assign(now.begin(), now.end());
            out->insert(out->end(), mean.begin(), mean.end());
            out->push_back(static_cast<float>(n));
        }
        return err;
    }
    // (12h50, ele: "essa mao virada ai para se levantar de costas tambem nao gosto") A mao apoiada (o colisor a menos de 7 cm do chao) com
    // a palma para baixo (0-1, 1 sem mao apoiada): o -z do colisor (medido nos clipes: no de brucos as duas maos apoiam com ele para baixo;
    // no de costas o clipe apoia a direita de lado) nao pode apontar para cima - a rede apoiava a esquerda com o dorso no chao, ainda
    // ajoelhada (a cabeca abaixo de 0,6 m: vale a subida inteira, fora do deitado).
    float palmScore(const std::vector<PhysicsBodyState3D>& links, float floor) const {
        float palm = 1.0f;
        for (const std::size_t hand : handLinksForm) {
            if (hand >= links.size()) continue;
            const Quaternion q = (links[hand].orientation * profile->links[hand].collider.localOrientation).normalized();
            const Vec3 center = links[hand].position + links[hand].orientation.rotate(profile->links[hand].collider.localPosition);
            if (center.z - floor > 0.07f) continue;
            const float flip = std::max(0.0f, -q.rotate({ 0.0f, 0.0f, 1.0f }).z - 0.2f) / 0.4f;
            palm = std::min(palm, std::exp(-flip * flip));
        }
        return palm;
    }
    // (13/11 14h) A inclinacao de lado da pelve (graus): a linha das articulacoes dos quadris (a origem das coxas) contra o horizonte, + com
    // o quadril direito mais alto. (O eixo y do corpo da pelve dava 3,5 graus a mais que ela; a linha dos quadris da 0,0 no clipe do idle.)
    std::array<std::size_t, 2> thighLinks { std::numeric_limits<std::size_t>::max(), std::numeric_limits<std::size_t>::max() };
    float pelvisRoll(const std::vector<PhysicsBodyState3D>& links) const {
        if (thighLinks[0] >= links.size() || thighLinks[1] >= links.size()) return 0.0f;
        const Vec3 h = links[thighLinks[1]].position - links[thighLinks[0]].position;
        return 180.0f / Pi * std::atan2(h.z, std::hypot(h.x, h.y));
    }
    float facing(const World& w) const {
        const float c = std::cos(wrapAngle(faceDirection(w) - characterYaw(w)));
        return w.startKind == 7 ? 0.5f * (1.0f + c) : std::max(0.0f, c);
    }

    void observe(World& w) const {
        const std::size_t b = contract->bodyObservationSize();
        contract->observeBody(w.body->links, std::span<float>(w.observation).first(b));
        // (O contrato v3: o corpo contra o chao, depois do comando; o chao do treino e o plano z = 0.)
        contract->observeGround(w.body->links, w.body->interactions, floorUnder(w),
            std::span<float>(w.observation).subspan(b + CommandSize, TrackingGetUpContract3D::GroundObservationSize));
        {
            float* phase = w.observation.data() + b + CommandSize + GroundObservationSize;
            phase[0] = phase[1] = 0.0f;
            // (Nos ensinados, sem o relogio do clipe: o jogo nao tem fase - no parado ensinado o clipe parado so mede a pose.)
            if (w.clipGetUp && w.clipHandedAt < 0.0f) {
                // (11/11 14h: no levantar pelo clipe, meia volta no clipe inteiro - sen, cos de pi t/T: o fim nao repete o comeco; o
                // jogo conta o mesmo relogio desde que o levantar comeca.)
                const float duration = std::max(1.0e-3f, references[w.clipReference]->durationSeconds());
                const float t = std::min(duration, clipTime(w));
                phase[0] = std::sin(Pi * t / duration);
                phase[1] = std::cos(Pi * t / duration);
            } else if (clipTracked(w) && !w.taught && !freeStopNow(w) && !w.freeTroca && !pushFreeNow(w) && !(settings.idleLikeStop && idleHeld(w))) {
                const float duration = std::max(1.0e-3f, references[w.trackClip]->durationSeconds());
                const float t = trackTime(w, 0.0f);
                phase[0] = std::sin(2.0f * Pi * t / duration);
                phase[1] = std::cos(2.0f * Pi * t / duration);
            }
        }
        contract->observeEnvironment(w.scene.get(), w.handle, w.body->links, floorUnder(w),
            std::span<float>(w.observation).subspan(b + CommandSize + GroundObservationSize + PhaseSize, EnvironmentSize));
        if (settings.balanceObservation) {
            // (04/11 15h, o layout levantar-v7: o equilibrio e o estado do gerador de passos - zeros fora do idle da locomocao.)
            const std::size_t at = b + CommandSize + GroundObservationSize + PhaseSize + EnvironmentSize;
            contract->observeBalance(w.body->links, w.body->interactions, floorUnder(w),
                std::span<float>(w.observation).subspan(at, TrackingGetUpContract3D::BalanceObservationSize));
            const std::span<float> state = std::span<float>(w.observation).subspan(
                at + TrackingGetUpContract3D::BalanceObservationSize, TrackingBalanceSteps3D::FeatureCount);
            std::fill(state.begin(), state.end(), 0.0f);
            if (w.stepsOn && w.locomotionControl) w.steps.features(state);
        }
        const std::size_t fireInputs = settings.fireObservation ? FireObservationSize : 0u;
        if (settings.fireObservation) {
            // (06h40, settings.fireObservation - ver o .hpp: logo antes da habilidade.)
            float* f = &w.observation[w.observation.size() - 1 - FireObservationSize];
            const float yaw = trackingBodyYaw3D(profile->links[rootLink], w.body->links[rootLink]), c = std::cos(yaw), s = std::sin(yaw);
            Vec3 external = fireHoldNow(w) + (w.pushTicks > 0 ? w.pushForce : Vec3 {}) + (fireGrabActive(w) ? w.fireGrabForce : Vec3 {});
            // (17/11, settings.contactForceObservation - ver o .hpp) Mais a forca dos contatos que nao sao o apoio.
            if (settings.contactForceObservation) external += nonSupportContactForce(w.body->interactions);
            // (17/11, settings.projectileRate) O projetil enquanto age, como os empurroes.
            if (w.projectileTicks > 0) external += w.projectileForce;
            const float weight = std::max(1.0f, profile->totalMassKg * 9.81f);
            const auto put = [&](float* out, Vec3 v, float scale) {
                out[0] = std::clamp((c * v.x + s * v.y) / scale, -3.0f, 3.0f);
                out[1] = std::clamp((-s * v.x + c * v.y) / scale, -3.0f, 3.0f);
                out[2] = std::clamp(v.z / scale, -3.0f, 3.0f);
            };
            put(f, external, weight);
            put(f + 3, w.comAcceleration, 9.81f);
        }
        if (settings.clipVariantObservation) {
            // (18h05, settings.clipVariantObservation - ver o .hpp: logo antes da habilidade; com o fogo, antes dele.)
            w.observation[w.observation.size() - 2 - fireInputs] = w.clipGetUp && w.clipHandedAt < 0.0f && w.clipExercise < clipGetUpVariants.size()
                ? clipGetUpVariants[w.clipExercise] : 0.0f;
        }
        if (settings.idleAnchorObservation) {
            // (11/11, settings.idleAnchorObservation - ver o .hpp; com a variante, uma antes; com o fogo, antes dele.)
            float* a = &w.observation[w.observation.size() - (settings.clipVariantObservation ? 4 : 3) - fireInputs];
            a[0] = a[1] = 0.0f;
            if (w.idleAnchored && w.locomotionControl && idleTracked(w)) {
                const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
                const float h = pelvisHeading(pelvis), c = std::cos(h), s = std::sin(h);
                const float dx = w.idleAnchor.x - pelvis.position.x, dy = w.idleAnchor.y - pelvis.position.y;
                a[0] = std::clamp((c * dx + s * dy) / 0.1f, -3.0f, 3.0f);
                a[1] = std::clamp((-s * dx + c * dy) / 0.1f, -3.0f, 3.0f);
            }
        }
        w.observation.back() = settings.separateSkills && w.locomotionControl ? 1.0f : 0.0f;
        std::fill(w.clipAction.begin(), w.clipAction.end(), 0.0f);
        // (03/11 18h30: tambem na locomocao por AMP - os alvos do clipe da decisao para a imitacao supervisionada do
        // comeco, que ensina a cabeca da locomocao os movimentos de cada rumo; nao vao para os motores.)
        if (w.locomotion && clipTracked(w)) {
            referenceFrame(w, TicksPerDecision * TickSeconds, false, w.frame);
            contract->actionOf(w.frame.coordinates, w.clipAction);
            // (17/11, settings.imitationVelocityTargets - ver o .hpp) A velocidade-alvo do clipe na decisao: do quadro de agora ao da proxima,
            // por eixo nas juntas de um eixo e por junta (a velocidade angular no referencial do filho) nas de varios.
            if (settings.imitationVelocityTargets) {
                const float dt = TicksPerDecision * TickSeconds;
                referenceFrame(w, 0.0f, false, w.imitationNowFrame);
                w.clipActionNow.assign(w.clipAction.size(), 0.0f);
                contract->actionOf(w.imitationNowFrame.coordinates, w.clipActionNow);
                w.clipActionVelocity.assign(w.clipAction.size(), 0.0f);
                for (std::size_t k = 0; k < w.clipAction.size(); ++k) {
                    const float d = w.clipAction[k] - w.clipActionNow[k];
                    w.clipActionVelocity[k] = std::abs(d) < 1.0f ? d / dt : 0.0f;
                }
                jointVelocities(w.clipActionNow, w.clipAction, dt, false, false, 1.0f, w.clipActionVelocity, true);
            }
        } else if (w.clipGetUp && w.clipHandedAt < 0.0f && settings.getUpImitationMask != 0) {
            // (14/11 v1, settings.getUpImitationMask; ele: o levantar de costas "MUITO distante do clipe, movimentos bem feios... o mais
            // semelhante possivel") Nos exercicios da mascara, limpos (sem fogo): o clipe do COMECO no ritmo de getUpImitationPace s ate a
            // entrega, uma decisao a frente - nao o relogio adaptativo, que segue o corpo e, quando ele rola de brucos, troca para o clipe
            // de brucos (a imitacao ensinaria o caminho errado). Os outros levantares ficam sem alvo (sem imitacao).
            const bool chosen = w.clipExercise < 31u && ((settings.getUpImitationMask >> w.clipExercise) & 1) != 0 && w.fireKind == 0
                && w.imitationReference < references.size();
            if (chosen) {
                const TrackingClipReference3D& reference = *references[w.imitationReference];
                const float deliver = w.clipExercise < clipGetUpDeliver.size() && clipGetUpDeliver[w.clipExercise] > 0.0f
                    ? clipGetUpDeliver[w.clipExercise] : reference.durationSeconds();
                const float ahead = std::max(0.0f, w.time) + TicksPerDecision * TickSeconds;
                const float tau = std::min(deliver, w.imitationStart + ahead * deliver / std::max(0.3f, imitationPace(w)));
                thread_local TrackingClipFrame3D imitationFrame;
                reference.frame(tau, {}, 0.0f, imitationFrame);
                contract->actionOf(imitationFrame.coordinates, w.clipAction);
            }
        } else if (w.clipGetUp && w.clipHandedAt < 0.0f) {
            // (11/11 14h: no levantar pelo clipe, os alvos do clipe uma decisao a frente - so medem: a calibracao toca o professor
            // nos motores (build-profile/tmp/levantar_unreal_auditoria/calibrar.py); a recuperacao nao imita.)
            const TrackingClipReference3D& reference = *references[w.clipReference];
            reference.frame(std::min(reference.durationSeconds(), clipTime(w) + TicksPerDecision * TickSeconds), {}, 0.0f, w.frame);
            contract->actionOf(w.frame.coordinates, w.clipAction);
        }
        float* c = w.observation.data() + b;
        const bool moving = w.commandSpeed > 0.0f;
        // (04/11: tambem no rastreio - o erro 5 do balanco de 03/11, o pedido contra a pelve que gira trocava frente e lado.)
        if ((w.taught || w.locomotion) && w.locomotionControl) {
            // (So com a cabeca da locomocao: caido, a recuperacao levanta com o pedido de sempre - a revisao de 03/11.)
            // (03/11 20h35, ele: "o andar para frente ele esta tentando andar para as laterais": contra a pelve, que gira na
            // passada - ~22 graus no parado em alerta, mais nas laterais -, o rumo pedido nao aparecia direto e um rumo
            // parecia outro. Nos ensinados, o pedido no quadro do jogo, como o analogico: o deslocamento em relacao a vista
            // - constante no exercicio: frente 0, direita -90 graus - e a vista em relacao ao peito, para onde virar.)
            // (08/11 15h20: parando - a tecla solta -, o pedido ja e zero na observacao, como no jogo; a referencia freia por dentro.)
            const bool commanded = moving && !w.stopping;
            // (11h45, ele: "o tronco todo mira": no giro parado a vista contra o rumo do tronco inteiro, nao o do peito.)
            const float facedYaw = w.startKind == 7 ? trunkYawOf(w.body->links) : chestYaw(w);
            const float move = commanded ? wrapAngle(w.moveYaw - w.faceYaw) : 0.0f, face = wrapAngle(w.faceYaw - facedYaw);
            c[0] = std::cos(move);
            c[1] = std::sin(move);
            c[2] = commanded ? 1.0f : 0.0f;
            c[3] = commanded ? w.commandSpeed / 5.0f : 0.0f;
            c[4] = std::cos(face);
            c[5] = std::sin(face);
            return;
        }
        if (!moving && !w.locomotion) {
            c[0] = 1.0f, c[1] = 0.0f, c[2] = 0.0f, c[3] = 0.0f, c[4] = 1.0f, c[5] = 0.0f;
            // (settings.idleLikeStop: a vista contra o peito de verdade, como no parado da parada livre.)
            // (10/11 01h50: e no parado livre, desde que o alvo do rosto foi posto.)
            if (idleHeld(w) && (settings.idleLikeStop || (settings.freeIdle && w.idleFaceSet))) {
                const float face = wrapAngle(w.faceYaw - chestYaw(w));
                c[4] = std::cos(face);
                c[5] = std::sin(face);
            }
            // (15/11, settings.getUpViewReward) No levantar e ir de pe, a vista contra o rumo do corpo, como no levantar do comando.
            if (getUpViewOn(w)) {
                const float face = wrapAngle(w.faceYaw - bodyYaw(w));
                c[4] = std::cos(face);
                c[5] = std::sin(face);
            }
            return;
        }
        // (Parado com rumo pedido - o giro, o rumo depois de parar: o deslocamento como sem pedido, o rumo do corpo sim.)
        const float yaw = bodyYaw(w);
        const float move = moving ? wrapAngle(moveDirection(w) - yaw) : 0.0f, face = wrapAngle(faceDirection(w) - yaw);
        c[0] = std::cos(move);
        c[1] = std::sin(move);
        c[2] = !moving ? 0.0f : w.locomotion ? 1.0f : std::min(targetBearing(w).second, 10.0f) / 10.0f;
        c[3] = w.commandSpeed / 5.0f;
        c[4] = std::cos(face);
        c[5] = std::sin(face);
    }

    // O potencial do rumo com pedido: virar para o rumo pedido enquanto sobe (a altura vezes o alinhamento).
    float facePotential(const World& w, const TrackingGetUpPosture3D& posture) const {
        if (w.commandSpeed <= 0.0f && !w.locomotion) return 0.0f;
        // (O rastreio cobra o rumo pela pelve do clipe; o potencial pela media dela brigaria com o giro.)
        if (w.locomotion) return 0.0f;
        const float rise = std::clamp((posture.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
        return 5.0f * rise * facing(w);
    }

    // A nota de estado com o pedido de ir ao alvo, so de pe: a velocidade rumo ao alvo 0,6 e, na proporcao dela, virado
    // para ele 0,25 e sem andar de lado 0,15 (eram 0,5, 0,3 e 0,2 inteiros). (31/10 18h: a velocidade era exp(-(v - pedida)^2), que nao dava gradiente parado - pedindo
    // 2,5-5 m/s, parado ou a 1 m/s dava ~0 - e em 250 atualizacoes ele ficou parado virado para o alvo. Agora uma rampa:
    // 0 parado, 1 na pedida, caindo acima dela e negativa se afasta.)
    // (Nos rumos, o rumo do corpo e o do comeco: andar e trotar de costas e de lado sem virar para a direcao.)
    // (14h05, settings.runSettleSeconds) A janela da rede do levantar depois de ficar de pe: nos correndo a deles, se houver.
    // (12/11 19h50, ele: no levantar e ir de pe "demora um pouco para de fato trocar o brain... precisaria ser mais preciso" e "no treino do
    // Comando ele use essa mesma troca") A troca ja de pe tambem no levantar e ir e no comando: a medida no modo do laboratorio - de pe aos
    // 0,9-1,0 s, a troca aos 1,6-1,8 s (0,2 s firme + a janela de 0,6 s), e na janela a cabeca do levantar deixava o corpo tombar (cabeca de
    // 1,65 a 1,46 m) e metade caia 0,3 s depois da troca. Sem janela, e o firme de pe em StandHandoffFirm.
    bool handoffAtStand(const World& w) const {
        return settings.runHandoffAtStand && w.clipGetUp && (clipRunExercise(w.clipExercise) || w.getUpGo || isCommand(w));
    }
    static constexpr float StandHandoffFirm = 0.1f;
    float settleSecondsFor(const World& w) const {
        if (handoffAtStand(w)) return 0.0f;
        return w.clipGetUp && clipRunExercise(w.clipExercise) && settings.runSettleSeconds > 0.0f ? settings.runSettleSeconds
                                                                                                 : settings.adaptiveSettleSeconds;
    }
    // (14h05) A velocidade ao alvo sobre a pedida (0-1).
    float runSpeedRatio(const World& w) const {
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        const float toward = std::atan2(w.target.y - w.body->links[rootLink].position.y, w.target.x - w.body->links[rootLink].position.x);
        return std::clamp((v.x * std::cos(toward) + v.y * std::sin(toward)) / w.commandSpeed, 0.0f, 1.0f);
    }
    float moveReward(World& w, const TrackingGetUpPosture3D& posture) {
        // (01/11 23h30, ele: no levantar em movimento, "recompensar ele principalmente para durante o ato de levantar
        // ir-se virando ao alvo e se impulsionando ao objetivo enquanto levanta" - com o levantar guiado: ainda no chao,
        // a velocidade ao alvo e o rumo para ele na proporcao do quanto ja subiu; deitado vale zero, rastejar nao paga.)
        if (!posture.standing && settings.getUpGuide && !w.locomotion) {
            const float rise = std::clamp((posture.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            const float toward = moveDirection(w);
            const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
            return rise * (0.6f * std::clamp(along / w.commandSpeed, -0.5f, 1.0f) + 0.25f * facing(w));
        }
        if (!posture.standing) return 0.0f;
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        const float toward = moveDirection(w);
        const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
        const float across = -v.x * std::sin(toward) + v.y * std::cos(toward);
        w.towardSum += along;
        ++w.towardSamples;
        const float ratio = along / w.commandSpeed;
        // (01/11 00h05, ele: "nenhum passo ate agora": a rampa concava ate a pedida - a raiz -, para um passo pequeno ja
        // compensar o estilo e o tranco de um andar ainda desajeitado; para tras e acima da pedida, como antes.)
        const float speed = ratio < 0.0f ? std::max(-1.0f, ratio) : ratio <= 1.0f ? std::pow(ratio, settings.speedRampPower)
                                                                                  : std::max(0.0f, 2.0f - ratio);
        // (22h55, ele: "garantir que ele consiga ter o estimulo para se mover, e nao ficar apenas parado": parado com o
        // pedido, o rumo e o sem andar de lado davam 0,5 de 1 - nos rumos a velocidade caia, 0,136 -> 0,098 m/s em 160
        // atualizacoes, e no levantar em movimento ficou ~0 por horas. Agora a velocidade 0,6 e os outros dois na
        // proporcao dela: parado com o pedido vale 0, na pedida 1.)
        // (01/11, a rl_final - ele: "cuidar para ele nao trapacear, tipo andar de costas, ele dar meia volta e ir de
        // frente": com o rumo valendo 0,25 contra 0,6 da velocidade, dar meia volta e correr de frente pagava ~0,75 e
        // andar de costas devagar ~0,45. Agora o rumo do corpo e o portao de tudo: de costas para o rumo pedido, zero; a
        // 45 graus, 71%; ir para tras do pedido continua custando.)
        const float progress = std::clamp(ratio, 0.0f, 1.0f);
        const float gate = facing(w) * sideDrift(w, v);
        // (03/11 17h25, ele no laboratorio da locomocao por AMP: "ignorando completamente o trote lateral, indo para
        // frente apenas; ate o para tras ele ta indo para frente" - ir de lado do pedido nao custava, so nao pagava. Na
        // locomocao o deslocamento de lado do rumo pedido custa ate 0,3.)
        const float wrongWay = w.locomotion ? std::min(1.0f, std::abs(across) / w.commandSpeed) : 0.0f;
        return gate * (0.6f * std::max(0.0f, speed) + progress * (0.25f + 0.15f * std::exp(-across * across)))
            + 0.6f * std::min(0.0f, speed) - 0.3f * wrongWay;
    }

    // O idle seguido agora: no parado ou num rumo parado, com o parado em alerta como clipe.
    // (01/11 20h: com o idle nos motores durante o empurrao ele nao dava o passo de recuperar - o sem cair de pe com
    // empurrao caiu de 91% para ~40%. O idle so quieto: sem empurrao e 1,5 s depois do ultimo; dai a rede livre.)
    bool idleTracked(const World& w) const {
        return (w.trackClip == standingIndex || w.trackClip == standingMirrorIndex) && w.commandSpeed <= 0.0f
            && (w.startKind == 2 || (w.startKind == 4 && settings.avoidOnIdle) || w.locomotion || w.idleAfterGetUp)
            // (04/11 15h: com o gerador de passos o idle segue no empurrao - a locomocao o segura com o passo.)
            && (settings.idleThroughPushes || (w.pushTicks == 0 && w.time >= w.lastPushEnd + 1.5f));
    }
    // A locomocao com pedido: o deslocamento ou, com o rastreio, o giro parado.
    bool locomotionCommanded(const World& w) const {
        return w.taught || (w.locomotion && (w.commandSpeed > 0.0f || w.startKind == 7
            || (isCommand(w) && w.command.state == 3)));
    }
    // O clipe seguido agora (o do passo com o pedido de deslocamento, o do giro com o rastreio, ou o idle parado).
    bool clipTracked(const World& w) const {
        return w.trackClip < references.size()
            && (locomotionCommanded(w) || idleTracked(w) || w.getUpGuided || (w.clipGetUp && w.clipHandedAt < 0.0f));
    }
    // O instante do clipe seguido daqui a `ahead` s e o rumo do quadro: em ciclo nos passos, no giro e no parado - o clipe
    // tocado na proporcao trackScale; cada ciclo soma o giro do clipe ao rumo (o giro parado encadeado) -; no levantar
    // guiado, sem ciclo e acelerado, preso no fim.
    std::pair<float, float> trackSample(const World& w, float ahead) const {
        // (11/11, settings.idleStatic: o idle alerta parado no quadro 0 - ver o .hpp.)
        if (settings.idleStatic && (w.trackClip == standingIndex || w.trackClip == standingMirrorIndex)) return { 0.0f, w.trackYaw };
        const TrackingClipReference3D& reference = *references[w.trackClip];
        const float duration = std::max(1.0e-3f, reference.durationSeconds());
        if (w.getUpGuided || (w.clipGetUp && w.clipHandedAt < 0.0f))
            return { std::clamp(w.trackStart + (w.time + ahead - w.trackFrom) * w.trackScale, 0.0f, duration), w.trackYaw };
        const float u = w.trackStart + startRampTime(w, w.time + ahead) * w.trackScale;
        const float cycles = std::floor(u / duration);
        return { std::clamp(u - cycles * duration, 0.0f, duration), w.trackYaw + cycles * reference.clip().turnCycleRadians };
    }
    float trackTime(const World& w, float ahead) const { return trackSample(w, ahead).first; }
    // A largada de pe (rampFrom >= 0; 03/11 10h40: com o clipe na cadencia cheia desde o primeiro quadro as pernas
    // corriam a frente do corpo parado e ele caia de costas): a cadencia do clipe e a velocidade cobrada sobem juntas de
    // StartRampFloor a 1 em StartRampSeconds. startRamp e o fator no instante t; startRampTime, o tempo do clipe ate t (a
    // integral do fator; sem largada, o proprio t).
    // (04/11 12h35: com o pedido fixo a rampa dura o que leva a subir ate ele a StartAcceleration - nunca mais aguda que a
    // de antes, 4,4 m/s em 0,8 s -, rampSeconds; e a corrida, que sai da arrancada a 4,2 m/s, sobe dali ate o pedido na
    // mesma aceleracao.)
    static constexpr float StartRampSeconds = 0.8f, StartRampFloor = 0.3f, StartAcceleration = 5.0f;
    float startRamp(const World& w, float t) const {
        if (w.rampFrom < 0.0f) return 1.0f;
        return StartRampFloor + (1.0f - StartRampFloor) * std::clamp((t - w.rampFrom) / w.rampSeconds, 0.0f, 1.0f);
    }
    // (04/11 08h50: a velocidade cobrada na largada pela rampa sobe de 0 - com o piso de 0,3 da cadencia, de pe ja se pedia
    // 0,9 m/s no primeiro quadro, e a mao arrastava o corpo parado.)
    float speedRamp(const World& w, float t) const {
        if (w.rampFrom < 0.0f) return 1.0f;
        return std::clamp((t - w.rampFrom) / w.rampSeconds, 0.0f, 1.0f);
    }
    // O pedido no instante t: o fixo (ou o do clipe) na rampa da largada e, depois da arrancada da corrida, subindo.
    float commandNow(const World& w, float t) const {
        float speed = w.commandSpeed * speedRamp(w, t);
        if (w.accelFrom >= 0.0f)
            speed = std::min(speed, w.accelSpeed + (w.accelRate > 0.0f ? w.accelRate : StartAcceleration) * std::max(0.0f, t - w.accelFrom));
        return speed;
    }
    // O pedido fixo de uma faixa num rumo (m/s; 0: sem, o do clipe).
    float fixedSpeed(std::uint32_t band, std::uint32_t heading) const {
        if (band == 3) return heading % 8u == 0 ? settings.sprintSpeed : 0.0f;
        return band == 1 ? settings.trotSpeeds[heading % 8u] : band == 2 ? settings.runSpeeds[heading % 8u] : 0.0f;
    }
    float startRampTime(const World& w, float t) const {
        if (w.rampFrom < 0.0f) return t;
        const float e = std::max(0.0f, t - w.rampFrom), T = w.rampSeconds, f = StartRampFloor;
        const float ramped = e < T ? f * e + (1.0f - f) * e * e / (2.0f * T) : f * T + (1.0f - f) * T / 2.0f + (e - T);
        return w.rampFrom + ramped;
    }

    // As partes do rastreio de cada link pelo nome (a pelve; coxa, canela e pe de cada lado; bracos e maos; o resto,
    // o tronco: abdomen, peito, peito alto, pescoco e cabeca).
    void classifyTrackingLinks() {
        trackingParts.assign(profile->links.size(), PartTrunk);
        for (std::size_t i = 0; i < profile->links.size(); ++i) {
            const std::string& id = profile->links[i].id;
            const bool leg = id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos
                || id.find("Foot") != std::string::npos || id.find("Toe") != std::string::npos;
            if (i == rootLink) trackingParts[i] = PartRoot;
            else if (leg) trackingParts[i] = id.starts_with("Left") ? PartLeftLeg : PartRightLeg;
            else if (id.find("Arm") != std::string::npos || id.find("Hand") != std::string::npos
                || (settings.cliffForearm && id.find("Forearm") != std::string::npos)
                || id.find("Shoulder") != std::string::npos || id.find("Clavicle") != std::string::npos)
                trackingParts[i] = PartArms;
            // (04/11 04h25: o pescoco e a cabeca a parte - a cabeca cobrada sozinha na v7.)
            else if (id.find("Neck") != std::string::npos || id.find("Head") != std::string::npos)
                trackingParts[i] = PartHead;
        }
        trackingPartCount = {};
        for (std::uint8_t part : trackingParts) trackingPartCount[part] += 1.0f;
    }

    // A nota do rastreio (03/11, vault 04_locomocao/plano_rastreio_locomocao_03_11.md): o corpo contra o quadro do clipe
    // seguido no tempo dele, posto sob a pelve do corpo (x, y) no rumo do quadro. As portas - o que define trapacear -
    // multiplicam: a velocidade do centro de massa suavizada na passada (~0,25 s) contra a pedida (imitar parado zera),
    // a orientacao da pelve no mundo (rumo e inclinacao: andar torto, dar meia-volta) e a simetria das pernas (mancar:
    // uma perna errada e a outra certa corta a nota inteira). A forma do corpo soma - da sinal mesmo longe do clipe: a
    // pior perna 0,5, os pes no quadro da propria pelve (passada, largura e altura do pe; o pior) 0,2, o tronco 0,15 e os
    // bracos 0,15. (10h55: o produto de tudo, a primeira versao, quase zerava com os erros do comeco - a pelve a 1 rad^2
    // ja multiplicava por 0,007 - e sem sinal o treino andou para tras.)
    static constexpr float TrackSymmetry = 4.0f, TrackLegs = 3.0f, TrackFeet = 10.0f, TrackArms = 2.0f;
    // A v2: a velocidade (1 / (m/s)^2: 0,5 m/s fora 0,78, 1 m/s 0,37, parado pedindo 2,6 ~0), a pelve (1 / rad^2: 15 graus
    // 0,75) e o giro da pelve e da cabeca contra o do clipe (1 / (rad/s)^2) - os dois ultimos calibrados no cartao.
    // A v7: as reguas da coluna (abdomen, peito, peito alto), dos bracos e da cabeca (pescoco e cabeca), por parte (a soma
    // dos angulos ao quadrado, rad^2) - calibradas no cartao (04/11 04h25).
    // (Medido na rede da 282, trote para a frente: a coluna 5,4 graus por junta (a do clipe puro 3,6) -> 0,67 (0,84); os
    // bracos 12,6 (6,3) -> 0,57 (0,88); a cabeca 6,6 (1,4) -> 0,59 (0,98).)
    static constexpr float TrackSpine7 = 15.0f, TrackArms7 = 3.0f, TrackHead7 = 20.0f;
    // (05/11 06h) O desvio de rumo do deslocamento que tira ~63% da nota da velocidade (rad; 06h25, reforcado: 8,6 -> 5,7 graus).
    static constexpr float CourseSlack = 0.10f, CourseTailSlack = 0.60f;
    // (05/11 06h15) O tombo de lado do peito alem do do clipe que tira ~63% da semelhanca nos rumos retos (rad; 06h25,
    // reforcado: 7 -> 4,6 graus - 3 graus 65%, 5 graus 38%).
    static constexpr float RollSlack = 0.08f, RollTailSlack = 0.35f;
    // (06/11) As reguas largas das caudas sem os penhascos: o pe cruzado (m alem da margem) e o pulo da pelve (m).
    static constexpr float FootCrossTailMeters = 0.08f, HopTailMeters = 0.15f, LaunchRiseSlack = 0.03f;
    static constexpr float TrackVelocity2 = 1.0f, TrackHeadSpin = 0.25f;
    // (04/11 08h55, ele: "no quesito de estabilidade considerar tambem a ALTURA da pelve, para detectar os pulinhos dele
    // contraditorios ao se movimentar - parecendo um cabrito"): o pulo e a pelve SUBINDO acima da linha dela contra a do
    // clipe tocado (a altura do corpo menos a do clipe, menos a media movel disso em PelvisHopSeconds - o desvio fixo do
    // agachar ou do afundar da fisica fica de fora). Medido (09h25): a 100% de forca, clipe puro e rede, o p90 2-4 cm; a 10%,
    // a rede no trote 13 cm. Ate PelvisHopSlack paga cheio, 7 cm metade, 10 cm 0,12. (A velocidade vertical contra a do
    // clipe nao serve: o clipe puro ja erra 0,4-1 m/s.)
    static constexpr float PelvisHopSeconds = 0.6f, PelvisHopSlack = 0.03f;
    // (04/11 13h50, ele: "a estabilidade levar mais em conta esses saltinhos - tipo ao correr de lado quando tropeca da uns
    // saltos": medido, a corrida de lado a ~20% de forca salta p99 15-18 cm (max 21; acima de 5 cm 16-24% do tempo) e ainda
    // pagava 0,82-0,88 de estabilidade; o clipe puro com a mao cheia, p99 4 cm.) Na v11: 2 cm alem da folga ja pagam metade
    // (5 cm, quase nada), o pulo pesa 0,35 da estabilidade e multiplica a nota inteira (0,80 + 0,20 x ele), como a inclinacao.
    static constexpr float TrackPelvisHop11 = 1730.0f;
    // (04/11 14h35, ele: "na corrida lateral direita o pe direito sempre tropeca e fica se arrastando - nao ocorre na
    // esquerda": os clipes sao espelhos um do outro e o corpo e simetrico; medido, a rede escorrega o pe direito no chao 24%
    // do tempo na corrida direita, contra 13% do pe espelho na esquerda - e nada cobrava o pe arrastado.) Na v12: o pe em
    // contato com o chao andando no plano alem de FootSlideSlack (m/s) - 1 m/s a mais paga 0,3 -, na estabilidade (0,15) e
    // multiplicando a nota (0,85 + 0,15 x ele).
    static constexpr float FootSlideSlack = 0.3f, TrackFootSlide = 1.2f;
    // (04/11 11h05, ele: "cobrar mais a estabilidade de angulo horizontal da pelve para casos alem do padrao dos clipes - em
    // MUITOS movimentos ele esta inclinado demais o corpo todo, precisamos punir isso"): a inclinacao da pelve e do peito alto
    // (o eixo de cima contra a vertical, no plano) contra a do clipe tocado; alem de LeanSlack (rad, 6 graus; nas trocas, na
    // janela de improviso, mais 10), 8 graus a mais pagam metade. Entra na estabilidade (0,25) e multiplica a nota inteira
    // (0,75 + 0,25 x ela): o corpo torto custa ate um quarto de tudo.
    static constexpr float LeanSlack = 0.105f, LeanImproviseSlack = 0.175f, TrackLean = 35.4f;
    // (04/11 11h25, ele: "nas arrancadas para costas e lados ele luta muito para manter a coluna no clipe, deveria ser mais
    // flexivel - e normal jogar o corpo para a direcao para pegar inercia; recompensar melhor a pegada de velocidade": na
    // janela da arrancada (de pe ate o fim da rampa ou do clipe da arrancada, saindo em LaunchFadeSeconds) a inclinacao PARA
    // o rumo do deslocamento e perdoada ate LaunchLean (rad, 15 graus), a coluna e a cabeca cobradas pela metade (na regua e
    // na folga do tronco, +8 graus) e a velocidade vale 0,15 a mais (da semelhanca). A cabeca entra na inclinacao (o giro).)
    static constexpr float LaunchFadeSeconds = 0.4f, LaunchLean = 0.26f;
    // (04/11 16h30, ele: "refinar o que compreendemos por estabilidade - hoje e um conceito meio gambiarra: ha movimentos com
    // boa estabilidade que ele interpreta como pouca e vice versa; um conceito mensuravel que reflita a verdadeira
    // estabilidade, a confianca do movimento e a firmeza da base"): a v13, a FIRMEZA - a fisica do proprio corpo, nao a pose
    // do clipe (essa ja e a semelhanca) -, quatro pilares numa media geometrica (um pilar ruim derruba o todo, a soma
    // escondia):
    //  equilibrio: a margem de estabilidade de Hof - o ponto de captura contra a borda de FORA da base, no eixo de lado do
    //    deslocamento do corpo (cair para a frente e o andar; para o lado de fora do pe e a queda); devagar num pe so, a borda
    //    de fora no sentido do centro de massa para o pe; devagar nos dois, a base inteira; no voo, a do ultimo apoio;
    //  base: o pe apoiado sem escorregar - na SOLA (os cantos dela rente ao chao, a velocidade deles; de cada pe o
    //    canto mais parado: um ponto preso e o pe rolando do calcanhar para a ponta, natural - o arrasto da v12 media o
    //    tornozelo e acusava o rolar: o clipe puro "arrastava" 47-72% do tempo) - e sem tropeco (um pe que sai e volta ao
    //    chao em menos de StumbleSeconds);
    //  corpo: o momento angular em volta do centro de massa no plano (o que tomba o corpo: os bracos e o tronco se debatendo)
    //    alem do do clipe;
    //  ritmo: a pelve sem saltinhos e a cabeca sem chacoalhar (a cabeca firme e o sinal humano da confianca).
    static constexpr float MarginSlack = 0.02f, FirmMargin = 433.0f, MarginOuterGap = 0.03f;
    static constexpr float StumbleSeconds = 0.12f, StumbleFade = 0.3f;
    // (05/11 04h20, ele: "no corrida lateral direita ele sempre esta batendo um pe no outro, nao consegue fazer aquela
    // cruzada igual faz no lateral esquerdo": a rede a 29% juntava os pes a 12 cm no rumo 6 - e 47 cm no 2; o clipe puro,
    // 25 cm nos dois.) (04h55, ele: "punir quando um pe encosta no outro, ou uma canela encosta na outra... vale para todos
    // os sentidos": o contato de verdade, pelos colisores - pe com pe, canela com canela, pe com canela -, no lugar da
    // distancia dos centros, que punia a cruzada que so passava perto.) Encostando (legGap < LegTouch): a base pela metade
    // e LegClashPenalty na nota da decisao - de pe, fora do levantar pelo clipe (no chao as pernas se tocam).
    static constexpr float LegTouch = 0.01f, LegClashPenalty = 0.3f;
    // (Os colisores das pernas para o contato - initLegColliders, ao carregar: a casca do pe pela caixa justa dela; a
    // casca exata deixava o treino 26% mais lento.)
    std::array<RagdollCapsuleDefinition3D, 4> legColliders {};
    void initLegColliders() {
        const std::uint32_t links[4] { footLinks[0], footLinks[1], shinLinks[0], shinLinks[1] };
        for (int k = 0; k < 4; ++k) {
            legColliders[k] = profile->links[links[k]].collider;
            if (legColliders[k].shape == RagdollColliderShape3D::ConvexHull) {
                legColliders[k].shape = RagdollColliderShape3D::Box;
                legColliders[k].hullPoints.clear();
                legColliders[k].hullFaces.clear();
                legColliders[k].hullPlanes.clear();
            }
        }
    }
    float legGap(const World& w) const {
        // (0 e 1 os pes, 2 e 3 as canelas: legColliders.)
        const std::uint32_t links[4] { footLinks[0], footLinks[1], shinLinks[0], shinLinks[1] };
        const auto toWorld = [&](int k, Vec3 c) {
            const RagdollCapsuleDefinition3D& d = legColliders[k];
            const PhysicsBodyState3D& b = w.body->links[links[k]];
            return b.position + b.orientation.rotate(d.localPosition + d.localOrientation.rotate(c));
        };
        const auto toLocal = [&](int k, Vec3 p) {
            const RagdollCapsuleDefinition3D& d = legColliders[k];
            const PhysicsBodyState3D& b = w.body->links[links[k]];
            return d.localOrientation.conjugate().rotate(b.orientation.conjugate().rotate(p - b.position) - d.localPosition);
        };
        // (De um elo ao outro: a capsula pelo eixo, cinco pontos com o raio dela; a caixa pelos 8 cantos.)
        const auto from = [&](int a, int b) {
            const RagdollCapsuleDefinition3D& ca = legColliders[a];
            float best = 1.0e9f;
            if (ca.shape == RagdollColliderShape3D::Capsule) {
                const float half = ragdollCapsuleHalfSegment3D(ca);
                for (int k = 0; k <= 4; ++k) {
                    const Vec3 p { -half + 0.5f * half * static_cast<float>(k), 0.0f, 0.0f };
                    const float radius = -ragdollColliderClosestPoint3D(ca, p).distance;
                    best = std::min(best, ragdollColliderClosestPoint3D(legColliders[b], toLocal(b, toWorld(a, p))).distance - radius);
                }
            } else {
                const Vec3 h = ca.boxHalfExtents;
                for (const float x : { -h.x, h.x })
                    for (const float y : { -h.y, h.y })
                        for (const float z : { -h.z, h.z })
                            best = std::min(best, ragdollColliderClosestPoint3D(legColliders[b], toLocal(b, toWorld(a, { x, y, z }))).distance);
            }
            return best;
        };
        const int pairs[4][2] { { 0, 1 }, { 2, 3 }, { 0, 3 }, { 1, 2 } };
        float gap = 1.0e9f;
        for (const auto& pair : pairs) gap = std::min({ gap, from(pair[0], pair[1]), from(pair[1], pair[0]) });
        return gap;
    }
    static constexpr float MovingSpeed = 0.5f, SpinSlack = 0.10f, FirmSpin = 31.0f, SlipSlack = 0.15f, FirmSlip = 2.77f, SoleDown = 0.01f;
    struct Firmness {
        float balance = 1.0f, base = 1.0f, body = 1.0f, rhythm = 1.0f, margin = 0.0f, spinExcess = 0.0f, total = 1.0f,
              slip = 0.0f, stumble = 0.0f;
    };
    // (08/11) A fase do clipe presa ao passo do corpo (settings.phaseSync): no pouso do pe esquerdo depois de PhaseSyncAirSeconds
    // no ar, nos rumos retos com pedido, num ciclo sem giro, o relogio do clipe salta para o pouso esquerdo dele - o quadro
    // comparado e a fase observada passam a ser os do passo do corpo. Fora das manobras (as trocas e as curvas tem o alinhamento
    // delas) e do levantar.
    // (So corrige o descompasso grande - PhaseSyncGap do ciclo -: o que ja esta em fase fica; ressincronizar a cada passo
    // so trazia tremor - o trote de costas 0,24 -> 0,15. E o pe 0,08 s no ar: o trote de frente da rede quase nao voa.)
    static constexpr float PhaseSyncAirSeconds = 0.08f, PhaseSyncGap = 0.12f;
    // (17/11, settings.headingBiasWeight) A janela da media do rumo da pelve: ~1 ciclo da corrida.
    static constexpr float HeadingBiasSeconds = 0.6f;
    // (17/11, settings.straightCommand) O giro da vista abaixo do qual o comando conta como reta (rad/s; ~3 graus/s).
    static constexpr float StraightCommandTurn = 0.05f;
    static constexpr int PhaseSyncFrames = 32;
    static constexpr float FacingSlack = 0.35f, FacingWidth = 0.6f;
    void syncPhase(World& w) const {
        if (!w.locomotion || w.commandSpeed <= 0.0f || w.maneuver != 0 || w.getUpGuided || w.trackClip >= references.size()
            || !locomotionCommanded(w) || idleTracked(w) || w.stopping || unrealStarting(w)) return;
        const TrackingClipReference3D& reference = *references[w.trackClip];
        if (std::abs(reference.clip().turnCycleRadians) > 0.05f) return;
        // (15/11 13h30, a revisao) Na largada o clipe (idle_to_sprint) nao e um ciclo: pular a fase dele saltava o relogio da largada
        // 12-50% de 0,8 s - e a velocidade pedida (startSpeed) +-1,5 m/s - ou voltava ao quadro 0 (de pe).
        if (starting(w) || !reference.clip().loops) return;
        // (A fase pelo quadro do ciclo com as pernas mais parecidas com as do corpo agora - o canal de contato do clipe e o
        // pouso do corpo nao marcam o mesmo instante: o salto para o pouso esquerdo do clipe piorou o trote de costas, que
        // ja estava quase em fase, 0,24 -> 0,09.)
        const auto [tNow, yawNow] = trackSample(w, 0.0f);
        (void)tNow;
        const float duration = std::max(1.0e-3f, reference.durationSeconds());
        const Vec3 root = w.body->links[rootLink].position;
        float best = 1.0e30f;
        int bestFrame = -1;
        for (int k = 0; k < PhaseSyncFrames; ++k) {
            reference.frame(duration * static_cast<float>(k) / PhaseSyncFrames, { root.x, root.y, floorUnder(w) }, yawNow, w.altFrame,
                twist(w), pitch(w), legPitch(w));
            w.altErrors.assign(w.jointErrors.size(), 0.0f);
            contract->jointErrors(w.body->links, w.altFrame, w.altErrors);
            float legs = 0.0f;
            for (std::size_t q = 0; q < trackingParts.size() && q < w.altErrors.size(); ++q)
                if (trackingParts[q] == PartLeftLeg || trackingParts[q] == PartRightLeg) legs += w.altErrors[q];
            if (legs < best) {
                best = legs;
                bestFrame = k;
            }
        }
        if (bestFrame < 0) return;
        const float u = w.trackStart + startRampTime(w, w.time) * w.trackScale;
        float phase = u / duration;
        phase -= std::floor(phase);
        float shift = static_cast<float>(bestFrame) / PhaseSyncFrames - phase;
        shift -= std::round(shift);
        if (std::abs(shift) < PhaseSyncGap) return;
        w.trackStart += shift * duration;
    }
    Firmness firmness(World& w, float rate, float heightSteady, float headSpin) const {
        Firmness out;
        const float ground = floorUnder(w);
        const TrackingBalance3D now = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions,
            { footLinks[0], footLinks[1] }, ground);
        out.margin = w.lastMargin;
        const float moving = std::hypot(now.velocity.x, now.velocity.y);
        if (now.footDown[0] || now.footDown[1]) {
            // (settings.launchSideMargin) Na largada, devagar, o eixo de lado do rumo pedido.
            const bool launchSide = settings.launchSideMargin && moving <= MovingSpeed && starting(w) && w.commandSpeed > 0.0f;
            if (moving > MovingSpeed || launchSide) {
                // (O eixo de lado do deslocamento DE VERDADE - o do corpo; o pedido vira na hora nas trocas.)
                const Vec3 side = launchSide ? Vec3 { -std::sin(w.moveYaw), std::cos(w.moveYaw), 0.0f }
                                             : Vec3 { -now.velocity.y / moving, now.velocity.x / moving, 0.0f };
                const float capture = dot(now.capture - now.centerOfMass, side);
                float lo = 1.0e9f, hi = -1.0e9f, outer = 0.0f;
                int down = 0;
                for (int f = 0; f < 2; ++f) {
                    if (!now.footDown[f]) continue;
                    ++down;
                    for (const Vec3& k : now.corners[f]) {
                        const float s = dot(k - now.centerOfMass, side);
                        lo = std::min(lo, s);
                        hi = std::max(hi, s);
                    }
                    outer += dot(now.sole[f] - now.centerOfMass, side);
                }
                out.margin = down == 1 && outer > MarginOuterGap ? hi - capture
                    : down == 1 && outer < -MarginOuterGap ? capture - lo
                    : std::min(capture - lo, hi - capture);
            } else if (now.footDown[0] != now.footDown[1]) {
                // (Devagar num pe so - o passo no lugar, o giro -: so a borda de fora, no sentido do centro de massa para o pe;
                // o peso indo para o lado do outro pe e o passo.)
                const int f = now.footDown[0] ? 0 : 1;
                Vec3 u = now.sole[f] - now.centerOfMass;
                u.z = 0.0f;
                const float length = std::hypot(u.x, u.y);
                if (length > 1.0e-3f) {
                    u = u * (1.0f / length);
                    float far = -1.0e9f;
                    for (const Vec3& k : now.corners[f]) far = std::max(far, dot(k - now.centerOfMass, u));
                    out.margin = far - dot(now.capture - now.centerOfMass, u);
                } else {
                    out.margin = now.margin;
                }
            } else {
                out.margin = now.margin;
            }
            w.lastMargin = out.margin;
        }
        const float dt = TicksPerDecision * TickSeconds;
        // (O pe apoiado tambem na decisao de antes: na do pouso ele ainda vem com a velocidade do balanco.)
        const bool planted[2] { now.footDown[0] && w.firmAir[0] == 0, now.footDown[1] && w.firmAir[1] == 0 };
        w.stumble *= std::exp(-dt / StumbleFade);
        for (int f = 0; f < 2; ++f) {
            if (!now.footDown[f]) {
                ++w.firmAir[f];
                continue;
            }
            // (So andando: no giro e no passo no lugar os toques curtos sao o proprio passo - o clipe "tropecava" 0,24.)
            if (moving > MovingSpeed && w.firmAir[f] > 0 && static_cast<float>(w.firmAir[f]) * dt < StumbleSeconds) w.stumble = 1.0f;
            if ((f == 0 || settings.phaseSyncBothFeet) && settings.phaseSync && static_cast<float>(w.firmAir[f]) * dt >= PhaseSyncAirSeconds)
                syncPhase(w);
            w.firmAir[f] = 0;
        }
        for (int f = 0; f < 2; ++f) {
            // (Os cantos da sola a ate SoleDown do chao agora e a velocidade de cada um depois do passo da fisica; do pe, a do
            // canto mais parado. A lista de contatos junta os 4 passos da decisao - um pe que ja saiu ainda "tocava" - e a
            // velocidade dela e a de antes do solver.)
            if (!planted[f]) continue;
            const RagdollLinkDefinition3D& link = profile->links[footLinks[f]];
            const PhysicsBodyState3D& foot = w.body->links[footLinks[f]];
            const bool box = ragdollColliderHasBox3D(link.collider);
            const Vec3 half = box ? link.collider.boxHalfExtents : Vec3 { 0.13f, 0.05f, 0.0f };
            const Vec3 middle = box ? link.collider.localPosition : Vec3 {};
            const Quaternion local = box ? link.collider.localOrientation : Quaternion {};
            const Vec3 center = rlLinkCenterOfMass3D(link, foot);
            float least = -1.0f;
            // (Uma grade de 5 x 3 pontos da sola: girando na ponta do pe, o ponto do pivo fica parado e os cantos andam.)
            for (const float sx : { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f })
                for (const float sy : { -1.0f, 0.0f, 1.0f }) {
                    const Vec3 p = foot.position + foot.orientation.rotate(middle + local.rotate({ sx * half.x, sy * half.y, -half.z }));
                    if (p.z - ground > SoleDown) continue;
                    const Vec3 v = foot.linearVelocity + cross(foot.angularVelocity, p - center);
                    const float speed = std::hypot(v.x, v.y);
                    least = least < 0.0f ? speed : std::min(least, speed);
                }
            out.slip = std::max(out.slip, least);
        }
        out.stumble = w.stumble;
        const TrackingBalance3D clip = measureTrackingBalance3D(*profile, w.frame.links, {}, { footLinks[0], footLinks[1] }, ground);
        const Vec3 spin = now.spin - clip.spin * rate;
        out.spinExcess = std::hypot(spin.x, spin.y);
        const float outside = std::max(0.0f, -out.margin - MarginSlack);
        const float flail = std::max(0.0f, out.spinExcess - SpinSlack);
        out.balance = std::exp(-FirmMargin * outside * outside);
        const float slip = std::max(0.0f, out.slip - SlipSlack);
        // (12/11 16h30, settings.locoFireRelief) Sob o fogo, o passo rapido nao e tropeco e o giro alem do clipe nao custa.
        const float fireFree = fireFreedom(w);
        out.base = std::exp(-FirmSlip * slip * slip) * (1.0f - 0.5f * w.stumble * (1.0f - fireFree));
        if (w.legGap < LegTouch && !w.getUpGuided) out.base *= 0.5f;
        out.body = std::exp(-FirmSpin * flail * flail);
        out.rhythm = 0.6f * heightSteady + 0.4f * std::exp(-TrackHeadSpin * headSpin);
        out.total = std::pow(out.balance, 0.30f) * std::pow(out.base, 0.25f) * std::pow(out.body, 0.25f * (1.0f - fireFree))
            * std::pow(out.rhythm, 0.20f * (1.0f - 0.5f * fireFree));
        return out;
    }
    // (14/11 21h45, settings.getUpImitationPaceRun) O ritmo da imitacao do levantar deste mundo: o dos correndo a parte.
    float imitationPace(const World& w) const {
        return clipRunExercise(w.clipExercise) && settings.getUpImitationPaceRun > 0.0f ? settings.getUpImitationPaceRun
                                                                                          : settings.getUpImitationPace * getUpTimeScale(w);
    }
    float launchWeight(const World& w) const {
        if (w.launchEnd < 0.0f) return 0.0f;
        return std::clamp((w.launchEnd + LaunchFadeSeconds - w.time) / LaunchFadeSeconds, 0.0f, 1.0f);
    }
    // (14/11 20h40, settings.launchLeanWeight) A inclinacao (graus) no rumo do movimento, + para o rumo: {a do corpo, a do clipe seguido no
    // mesmo instante, posto como na imitacao}. (21h45: a do CORPO INTEIRO - do pe de apoio (o mais baixo; os dois juntos, o meio) ao centro
    // de massa. A de 20h40, do peito contra a pelve, deixava a rede agachar e saltitar curvada: mais "inclinada" e mais lenta.)
    std::array<float, 2> launchLeans(const World& w) const {
        thread_local TrackingClipFrame3D frame;
        const Vec3 root = w.body->links[rootLink].position;
        references[w.trackClip]->frame(trackTime(w, 0.0f), { root.x, root.y, 0.0f }, w.trackYaw, frame, twist(w));
        const Vec3 dir { std::cos(w.moveYaw), std::sin(w.moveYaw), 0.0f };
        const auto lean = [&](const std::vector<PhysicsBodyState3D>& links) {
            const Vec3 a = links[footLinks[0]].position, b = links[footLinks[1]].position;
            const Vec3 base = std::abs(a.z - b.z) < 0.03f ? (a + b) * 0.5f : (a.z < b.z ? a : b);
            const Vec3 d = rlBodyCenterOfMass3D(*profile, links) - base;
            return 180.0f / Pi * std::atan2(d.x * dir.x + d.y * dir.y, std::max(1.0e-3f, d.z));
        };
        return { lean(w.body->links), lean(frame.links) };
    }
    // (10/11) A largura da regua de cauda longa do tronco (o seno de ~10 graus).
    static constexpr float PostureTailWidth = 0.17f;
    float trackingScore(World& w, Vec3 velocity) {
        const Vec3 root = w.body->links[rootLink].position;
        referenceFrame(w, 0.0f, true, w.frame);
        contract->jointErrors(w.body->links, w.frame, w.jointErrors);
        float part[6] {};
        for (std::size_t j = 0; j < trackingParts.size() && j < w.jointErrors.size(); ++j) part[trackingParts[j]] += w.jointErrors[j];
        const float legs = std::max(part[PartLeftLeg], part[PartRightLeg]);
        const float trunkAll = part[PartTrunk] + part[PartHead];
        const auto degrees = [&](TrackingPart p) {
            return trackingPartCount[p] > 0.0f ? std::sqrt(part[p] / trackingPartCount[p]) * 180.0f / Pi : 0.0f;
        };
        const float asymmetry = part[PartLeftLeg] - part[PartRightLeg];
        // (Os pes no quadro da propria pelve - o rumo de cada uma tirado -: o rumo ja e cobrado pela pelve.) O rumo puro, a
        // frente da pelve no plano: 14h50, o trackingBodyYaw3D soma a frente e o alto, e com a pelve inclinada do trote ele
        // acusava "rumo virado" com o corpo de pe e alinhado - o fim por rumo matava os episodios em 0,5 s.
        const auto heading = [&](const PhysicsBodyState3D& pelvis) {
            const Vec3 f = (pelvis.orientation * profile->links[rootLink].modelOrientation.conjugate()).normalized()
                               .rotate({ 1.0f, 0.0f, 0.0f });
            return std::atan2(f.y, f.x);
        };
        const float bodyHeading = heading(w.body->links[rootLink]);
        const float referenceHeading = heading(w.frame.links[rootLink]);
        // (13/11 v1, settings.turnedGrace) A referencia girando rapido (> TurnGraceRate: o salto e o giro da vista no comando, a troca de
        // perna na estrela): o virado ganha settings.turnedGrace s - o corpo precisa de tempo para virar; a meia-volta sustentada segue barrada.
        if (settings.turnedGrace > 0.0f) {
            if (w.hasReferenceHeading
                && std::abs(wrapAngle(referenceHeading - w.lastReferenceHeading)) > TurnGraceRate * TicksPerDecision * TickSeconds)
                w.referenceTurningUntil = w.time + settings.turnedGrace;
            w.lastReferenceHeading = referenceHeading;
            w.hasReferenceHeading = true;
        }
        const auto local = [](Vec3 v, float heading) {
            const float c = std::cos(heading), s = std::sin(heading);
            return Vec3 { c * v.x + s * v.y, -s * v.x + c * v.y, v.z };
        };
        const Vec3 referenceRoot = w.frame.links[rootLink].position;
        float feet = 0.0f;
        for (std::uint32_t f : footLinks)
            feet = std::max(feet, (local(w.body->links[f].position - root, bodyHeading)
                                   - local(w.frame.links[f].position - referenceRoot, referenceHeading)).lengthSquared());
        w.smoothVelocity = w.smoothVelocity + (Vec3 { velocity.x, velocity.y, 0.0f } - w.smoothVelocity)
            * (TicksPerDecision * TickSeconds / 0.25f);
        if (settings.courseSmoothing > 0.0f)
            w.courseVelocity = w.courseVelocity + (Vec3 { velocity.x, velocity.y, 0.0f } - w.courseVelocity)
                * std::min(1.0f, TicksPerDecision * TickSeconds / settings.courseSmoothing);
        const Vec3 wanted = wantedVelocity(w, 0.0f);
        const float drift = (w.smoothVelocity - wanted).lengthSquared();
        // (04/11, a nota v2 para baixar a forca - o plano no cartao: as partes somam em vez de multiplicar - com a forca
        // descendo a velocidade cai e o produto zerava o sinal da forma -, a semelhanca e a estabilidade valem metade
        // parado - andar no lugar nunca paga como andar -, e a estabilidade compara o giro da pelve e da cabeca com o do
        // clipe tocado: o balanco que o clipe nao tem.)
        const float rate = w.trackScale * startRamp(w, w.time);
        const auto spin = [&](std::size_t link) {
            return (w.body->links[link].angularVelocity - w.frame.links[link].angularVelocity * rate).lengthSquared();
        };
        const float pelvisSpin = spin(rootLink), headSpin = spin(contract->headLink());
        // (04/11 03h25, ele: "recompensar melhor a arrancada... na corrida ele da uma patinada parado ate pegar no tranco" -
        // medido: a nota da velocidade no primeiro 1 s valia 0,02-0,07, o exp achatado longe da pedida nao dava sinal para
        // acelerar. Na v3, metade e o progresso: a velocidade no rumo pedido sobre a pedida, de 0 a 1, em linha reta.)
        const float wantedSpeed = std::hypot(wanted.x, wanted.y);
        // (05/11 06h, ele: "no trote ele esta andando torto para a direita... nas diagonais da frente, na corrida frente, ate o
        // sprint": o progresso so contava a velocidade ao longo do rumo - 10 graus de desvio custavam 1,5% (o cosseno) - e a mao
        // empurrando no rumo escondia o vies da rede; sem ela, o desvio medio chegou a -10 graus. Agora o progresso so vale no
        // rumo certo: vezes exp(-(angulo / CourseSlack)^2) - 2 graus 95%, 5 graus 71%, 10 graus 26%.)
        // (06h25, ele: "esse erro prejudicou todos os treinos, reforce isso ainda mais": o rumo vale na nota da velocidade
        // INTEIRA - antes so no progresso, a metade dela - e a folga caiu de 8,6 para 5,7 graus: 2 graus 89%, 5 graus 47%,
        // 10 graus 5%. E a nota da velocidade multiplica a da semelhanca e a da estabilidade na v13.)
        // (08/11 18h45, ele: "melhorar a velocidade das locomocoes LATERAIS, extremamente lentas"): a 0,25 s a passada lateral
        // oscila para a frente e para tras a cada passo - o desvio medio absoluto 10-29 graus nos laterais contra 3-7 na frente,
        // com o medio com sinal pequeno - e a folga de 5,7 graus zerava a velocidade: nos laterais o termo valia ~1/5 do da
        // frente, e correr de lado nao rendia. O vicio de rumo (o motivo dele) se mede na media de um passo: courseSmoothing.
        const Vec3 courseV = settings.courseSmoothing > 0.0f ? w.courseVelocity : w.smoothVelocity;
        float straight = 1.0f;
        if (wantedSpeed > 0.3f && std::hypot(courseV.x, courseV.y) > 0.3f) {
            const float course = std::atan2(wanted.x * courseV.y - wanted.y * courseV.x, wanted.x * courseV.x + wanted.y * courseV.y);
            straight = std::exp(-(course / CourseSlack) * (course / CourseSlack));
            const float tail = settings.courseTail;
            if (tail > 0.0f)
                straight = (1.0f - tail) * straight + tail * std::exp(-(course / CourseTailSlack) * (course / CourseTailSlack));
        }
        const float alongRumo = wantedSpeed > 0.3f
            ? std::clamp((w.smoothVelocity.x * wanted.x + w.smoothVelocity.y * wanted.y) / (wantedSpeed * wantedSpeed), 0.0f, 1.0f)
            : std::exp(-TrackVelocity2 * drift);
        const float progress = alongRumo * straight;
        // (07/11, v15 - a revisao geral, ele: "o importante e sairmos da situacao atual de estagnacao": com a largura de 1 m/s,
        // longe da pedida o exp valia ~0 - a 57% da corrida de 6 m/s, exp(-6,8) - e subir a 80% rendia ~0,03 na recompensa; a
        // velocidade parou desde 00h. A largura acompanha a pedida - 0,35 dela -, nunca menos que o 1 m/s de antes.)
        const float driftWidth = std::max(1.0f, settings.speedWidthFraction * wantedSpeed);
        const float driftTerm = std::exp(-TrackVelocity2 * drift / (driftWidth * driftWidth));
        float velocityTerm = (0.5f * driftTerm + 0.5f * alongRumo) * straight;
        // (05/11 06h, ele: "fazer o trote costas recompensar melhor pelo impulso inicial para engatar velocidade": na largada
        // do trote de costas e das diagonais de costas, o progresso vale inteiro - a nota da velocidade sobe em linha reta com ela.)
        // (10/11 06h35, settings.liveLaunch; ele: "nada de passinhos nem lentidao, o IMPULSO real da corrida"): o progresso inteiro
        // na largada de todos os rumos - medido, saindo de pe o termo valia 0,4 caindo a 0,1 com o corpo atrasado e acelerar nao rendia.)
        if ((!w.runBand && w.maneuver == 0 && w.heading >= 3 && w.heading <= 5) || settings.liveLaunch)
            velocityTerm += (std::max(velocityTerm, progress) - velocityTerm) * launchWeight(w);
        // (04/11 04h10, ele: "no giro parado nao esta sendo cobrada a semelhanca com o clipe, o que faz ele ficar numa pose
        // meio estranha": na v5, sem deslocamento pedido - o giro -, a regua da semelhanca duas vezes mais exigente.)
        const bool stationary = w.commandSpeed <= 0.0f;
        // (04/11 04h15, ele: "nao podemos cobrar tanta semelhanca com o clipe nos momentos em que ele troca de direcao, nao
        // temos clipes para isso, ele naturalmente vai precisar improvisar": na v6, na janela da troca - as de peito fixo e
        // a livre -, a semelhanca vale 0,20 numa regua com metade da exigencia; a estabilidade 0,30 e a velocidade 0,40.)
        // (12/11 16h30, settings.locoFireRelief) Sob o fogo da locomocao, a liberdade e a janela de improviso: a imitacao mais larga e
        // com menos peso, mais peso a velocidade e ao equilibrio, mais folga na inclinacao.
        const float fireFree = fireFreedom(w);
        const float improvise = std::max(((w.maneuver >= 1 && w.maneuver <= 4) || isStar(w) || isCommand(w))
                && w.improviseUntil >= 0.0f
            ? std::clamp((w.improviseUntil - w.time) / ImproviseFadeSeconds, 0.0f, 1.0f) : 0.0f, fireFree);
        // (08/11 13h50, o fogo: girando parado o giro so segurava 36% dos empurroes - a regua da imitacao parado e 2x e se apoiar
        // contra o empurrao foge da pose; ele mantinha a pose e caia. Durante o empurrao e ate 1,5 s depois, a regua pela metade
        // em tudo - recuperar vale mais que copiar.)
        const float pushRelief = settings.pushImitationRelief && (w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f) ? 0.5f : 1.0f;
        const float strict = (stationary ? 2.0f - fireFree : 1.0f - 0.5f * improvise) * pushRelief;
        // (12h: na corrida de lado as pernas e os pes cobrados pela metade quando o alcance delas foi aumentado.)
        // (05/11 05h55, ele: "no corrida lateral direita o pe fica esticado para frente se arrastando; no esquerdo faz correto":
        // com as pernas cobradas pela metade na corrida de lado - a permissividade de 04/11 12h - a perna dura da direita passava
        // (semelhanca 0,54 contra 0,69 da esquerda; o clipe puro igual nos dois). O alcance largo fica; a cobranca, inteira.)
        // (08/11 13h05, a inspecao: no trote e na corrida de lado uma perna chuta para tras - o joelho 81-108 graus contra 26-34
        // da outra, o clipe arrastando o pe baixo; com a regua larga da imitacao ele passa. Nos lados (2, 6) dos retos, as pernas
        // e os pes cobrados lateralLegStrict vezes mais.)
        const bool sideways = w.locomotion && w.maneuver == 0 && (w.heading % 8u == 2u || w.heading % 8u == 6u);
        const float strictLegs = strict * (sideways ? settings.lateralLegStrict : 1.0f);
        const float launch = launchWeight(w);
        // (13h55: na largada de costas com backStartLean a referencia ja joga o tronco para o rumo - a coluna e a cabeca pela
        // metade so deixavam ele ir para a frente; ali cobradas inteiras.)
        const float launchUpper = startLeaning(w) ? 0.0f : launch;
        // (09/11 19h15, settings.poseBoost: "arrancadas estranhas" - na arrancada a coluna e a cabeça cobradas inteiras.)
        const float strictUpper = strict * (1.0f - 0.5f * (settings.poseBoost > 0.0f ? 0.0f : launchUpper));
        // (04/11 04h25, ele: "mesmo com a recompensa da pose aumentada, os bracos, a cabeca e a coluna nao se reajustaram
        // o quanto queriamos": na v7 a coluna, os bracos e a cabeca cada um com o seu peso e a sua regua, calibrada pelo
        // erro medio por junta (ver Track*7).)
        // (08/11, settings.imitationLoosen: andando, a regua larga da imitacao.)
        const float loose = !stationary && settings.imitationLoosen > 1.0f ? 1.0f / settings.imitationLoosen : 1.0f;
        // (10/11 04h, ele: "cobrar a coluna correta, cabeca, bracos com mais rigor durante as locomocoes e parado") A regua da
        // coluna, da cabeca e dos bracos vezes upperRigor - andando, no giro e nas trocas; as pernas e os pes com a de sempre. 1: sem.
        const float rig = settings.upperRigor;
        float imitation =
            (0.30f * std::exp(-loose * strictLegs * TrackLegs * legs) + 0.15f * std::exp(-loose * strictLegs * TrackFeet * feet)
                + 0.20f * std::exp(-loose * strictUpper * rig * TrackSpine7 * part[PartTrunk])
                + 0.20f * std::exp(-loose * strict * rig * TrackArms7 * part[PartArms])
                + 0.15f * std::exp(-loose * strictUpper * rig * TrackHead7 * part[PartHead]))
                * std::exp(-loose * TrackSymmetry * asymmetry * asymmetry);
        if (w.altClip < references.size() && w.trackClip < references.size()
            && w.trackClip != w.altClip) {
            // (O outro jeito aceito: o quadro dele na mesma fase do pe esquerdo, no mesmo rumo e com a mesma cintura.)
            const auto [t, yaw] = trackSample(w, 0.0f);
            const float now = std::max(1.0e-3f, references[w.trackClip]->durationSeconds());
            float phase = t / now - travel[w.trackClip].leftLand + travel[w.altClip].leftLand;
            phase -= std::floor(phase);
            references[w.altClip]->frame(phase * references[w.altClip]->durationSeconds(), { root.x, root.y, floorUnder(w) }, yaw,
                w.altFrame, twist(w), pitch(w), legPitch(w));
            w.altErrors.assign(w.jointErrors.size(), 0.0f);
            contract->jointErrors(w.body->links, w.altFrame, w.altErrors);
            float other[6] {};
            for (std::size_t j = 0; j < trackingParts.size() && j < w.altErrors.size(); ++j) other[trackingParts[j]] += w.altErrors[j];
            const float otherHeading = heading(w.altFrame.links[rootLink]);
            const Vec3 otherRoot = w.altFrame.links[rootLink].position;
            float otherFeet = 0.0f;
            for (std::uint32_t f : footLinks)
                otherFeet = std::max(otherFeet, (local(w.body->links[f].position - root, bodyHeading)
                                                 - local(w.altFrame.links[f].position - otherRoot, otherHeading)).lengthSquared());
            const float otherLegs = std::max(other[PartLeftLeg], other[PartRightLeg]);
            const float otherAsymmetry = other[PartLeftLeg] - other[PartRightLeg];
            const float alternative = (0.30f * std::exp(-loose * strict * TrackLegs * otherLegs)
                + 0.15f * std::exp(-loose * strict * TrackFeet * otherFeet)
                + 0.20f * std::exp(-loose * strict * rig * TrackSpine7 * other[PartTrunk]) + 0.20f * std::exp(-loose * strict * rig * TrackArms7 * other[PartArms])
                + 0.15f * std::exp(-loose * strict * rig * TrackHead7 * other[PartHead]))
                * std::exp(-loose * TrackSymmetry * otherAsymmetry * otherAsymmetry);
            imitation = std::max(imitation, alternative);
        }
        // (05/11 06h15, ele: "a inclinacao e para as CURVAS; quando e para trotar reto ou correr reto, basicamente estar ereto,
        // nao deve": nos rumos retos e no sprint, o peito tombado de lado - no proprio rumo do corpo - alem do do clipe derruba
        // a semelhanca: exp(-(diferenca / RollSlack)^2) - 3 graus 65%, 5 graus 38%. A coluna por junta deixava passar.)
        if (settings.cliffCurveRoll && isCurve(w) && w.commandSpeed > 0.0f) {
            // (06/11, sem os penhascos: nas curvas a inclinacao de lado contra a do clipe, so na regua larga.)
            const std::uint32_t chest = static_cast<std::uint32_t>(contract->chestLink());
            const Vec3 side { -std::sin(bodyHeading), std::cos(bodyHeading), 0.0f };
            // (17/11, settings.referenceSideFix) O clipe medido no eixo de lado DELE (o rumo da referencia), o corpo no dele.
            const Vec3 clipSide = settings.referenceSideFix ? Vec3 { -std::sin(referenceHeading), std::cos(referenceHeading), 0.0f } : side;
            const auto tilt = [&](const PhysicsBodyState3D& link, const Vec3& axis) {
                const Vec3 up = (link.orientation * profile->links[chest].modelOrientation.conjugate()).normalized()
                                    .rotate({ 0.0f, 0.0f, 1.0f });
                return std::asin(std::clamp(dot(up, axis), -1.0f, 1.0f));
            };
            const float roll = tilt(w.body->links[chest], side) - tilt(w.frame.links[chest], clipSide);
            imitation *= std::exp(-(roll / RollTailSlack) * (roll / RollTailSlack));
        }
        // (15/11, settings.legAlignWeight) O fator das pernas alinhadas: (1 - peso) + peso x a media por elo (as coxas e os pes) de
        // 1/(1 + (erro / largura)^2), o erro sendo o rumo do eixo de lado do elo contra a pelve, menos o mesmo no clipe.
        const auto legAlignFactor = [&]() -> float {
            if (thighLinks[0] >= w.body->links.size() || thighLinks[1] >= w.body->links.size()) return 1.0f;
            const auto lateralYaw = [&](const PhysicsBodyState3D& state, std::size_t link) {
                const Vec3 a = (state.orientation * profile->links[link].modelOrientation.conjugate()).normalized()
                                   .rotate({ 0.0f, 1.0f, 0.0f });
                return std::atan2(a.y, a.x);
            };
            // (13h35, a checagem das 13h30: com o exp da soma dos quatro a 10 graus, acima de ~12 graus por perna a nota ja estava no
            // piso e nao dava direcao - a frente e a diagonal 7 alinharam, o trote, a de costas e os laterais (15-35 graus) pioraram.
            // Agora cada elo com cauda, 1/(1 + (erro / largura)^2), e a media: 5 graus 0,8; 10 graus 0,5; 20 graus 0,2; 30 graus 0,1.)
            float align = 0.0f;
            int counted = 0;
            const std::array<std::size_t, 4> aligned { thighLinks[0], thighLinks[1], static_cast<std::size_t>(footLinks[0]),
                static_cast<std::size_t>(footLinks[1]) };
            const float width = std::max(0.02f, settings.legAlignWidth);
            for (const std::size_t link : aligned) {
                if (link >= w.body->links.size() || link >= w.frame.links.size()) continue;
                const float d = wrapAngle(wrapAngle(lateralYaw(w.body->links[link], link) - bodyHeading)
                    - wrapAngle(lateralYaw(w.frame.links[link], link) - referenceHeading)) / width;
                align += 1.0f / (1.0f + d * d);
                ++counted;
            }
            align = counted > 0 ? align / static_cast<float>(counted) : 1.0f;
            return (1.0f - settings.legAlignWeight) + settings.legAlignWeight * align;
        };
        // (17/11, settings.straightCommand - ver o .hpp) A reta: a manobra 0 ou, ligado, o comando numa reta.
        const bool straightRun = w.commandSpeed > 0.0f && (w.maneuver == 0 || (settings.straightCommand && isCommand(w)
            && w.command.state == 0 && std::abs(w.arcRate) < StraightCommandTurn
            && !(w.blendClip < references.size() && w.time < w.blendFrom + w.blendSeconds)));
        // (17/11) O rumo medio da pelve recomeca a cada reta (fora dela, a curva e a manobra tem o rumo delas).
        if (!straightRun) {
            w.headingBias = 0.0f;
            w.lateralBias = {};
        }
        if (straightRun) {
            const std::uint32_t chest = static_cast<std::uint32_t>(contract->chestLink());
            const Vec3 side { -std::sin(bodyHeading), std::cos(bodyHeading), 0.0f };
            // (17/11, settings.referenceSideFix - a auditoria: o clipe medido no eixo de lado do CORPO; com a pelve virada delta, a
            // inclinacao do clipe para a frente (~31 graus na corrida) vazava como 'rolamento' do clipe - lambda x sen(delta), ~0,5 grau de
            // lado exigido por grau de rumo - e o corpo virado era premiado por pender. O clipe no eixo de lado DELE.)
            const Vec3 clipSide = settings.referenceSideFix ? Vec3 { -std::sin(referenceHeading), std::cos(referenceHeading), 0.0f } : side;
            const auto tilt = [&](const PhysicsBodyState3D& link, const Vec3& axis) {
                const Vec3 up = (link.orientation * profile->links[chest].modelOrientation.conjugate()).normalized()
                                    .rotate({ 0.0f, 0.0f, 1.0f });
                return std::asin(std::clamp(dot(up, axis), -1.0f, 1.0f));
            };
            const float roll = tilt(w.body->links[chest], side) - tilt(w.frame.links[chest], clipSide);
            imitation *= (1.0f - settings.rollTail) * std::exp(-(roll / RollSlack) * (roll / RollSlack))
                + settings.rollTail * std::exp(-(roll / RollTailSlack) * (roll / RollTailSlack));
            // (17/11, settings.headingBiasWeight - ver o .hpp) O rumo medio da pelve contra o do clipe.
            if (settings.headingBiasWeight > 0.0f) {
                const float rate = 1.0f - std::exp(-TicksPerDecision * TickSeconds / HeadingBiasSeconds);
                w.headingBias += (wrapAngle(bodyHeading - referenceHeading) - w.headingBias) * rate;
                const float d = w.headingBias / std::max(0.02f, settings.headingBiasWidth);
                imitation *= (1.0f - settings.headingBiasWeight) + settings.headingBiasWeight / (1.0f + d * d);
            }
            // (17/11, settings.lateralBiasWeight - ver o .hpp) O lado medio da cabeca, do peito alto e do meio dos pes contra o clipe.
            if (settings.lateralBiasWeight > 0.0f) {
                const Vec3 clipAxis { -std::sin(referenceHeading), std::cos(referenceHeading), 0.0f };
                const std::size_t head = static_cast<std::size_t>(contract->headLink());
                const auto lateral = [&](const auto& links, const Vec3& axis) {
                    const Vec3 pelvis = links[rootLink].position;
                    return Vec3 { dot(links[head].position - pelvis, axis), dot(links[upperChestLink].position - pelvis, axis),
                        0.5f * (dot(links[footLinks[0]].position - pelvis, axis) + dot(links[footLinks[1]].position - pelvis, axis)) };
                };
                if (head < w.body->links.size() && upperChestLink < w.body->links.size() && head < w.frame.links.size()
                    && upperChestLink < w.frame.links.size()) {
                    const float rate = 1.0f - std::exp(-TicksPerDecision * TickSeconds / HeadingBiasSeconds);
                    w.lateralBias += (lateral(w.body->links, side) - lateral(w.frame.links, clipAxis) - w.lateralBias) * rate;
                    const float rms = std::sqrt(w.lateralBias.lengthSquared() / 3.0f) / std::max(0.005f, settings.lateralBiasWidth);
                    imitation *= (1.0f - settings.lateralBiasWeight) + settings.lateralBiasWeight / (1.0f + rms * rms);
                }
            }
            // (15/11, settings.legAlignWeight) As pernas alinhadas: o rumo de cada coxa e de cada pe contra a pelve, como no clipe.
            if (settings.legAlignWeight > 0.0f) imitation *= legAlignFactor();
            // (15/11, settings.runWidthWeight) Correndo, os pes mais abertos que no clipe alem de 4 cm.
            if (settings.runWidthWeight > 0.0f && (w.runBand || w.sprintGuided) && w.heading != 2 && w.heading != 6) {
                const auto spread = [&](const auto& links, const Vec3& axis) {
                    return std::abs(dot(links[footLinks[0]].position - links[footLinks[1]].position, axis));
                };
                const float gap = std::max(0.0f, spread(w.body->links, side) - spread(w.frame.links, clipSide) - 0.04f) / 0.06f;
                imitation *= (1.0f - settings.runWidthWeight) + settings.runWidthWeight * std::exp(-gap * gap);
            }
            // (So no trote e fora os laterais - 2 e 6 -: de lado a distancia entre os pes e o proprio passo (o clipe puro caia
            // 0,85 -> 0,62); correndo e no sprint a largura do clipe no corpo fisico varia demais (o clipe puro 0,58 -> 0,47).)
            if (settings.footWidthSlack > 0.0f && !w.runBand && !w.sprintGuided && w.heading != 2 && w.heading != 6) {
                // (05/11 20h10) A largura entre os pes contra a do clipe no mesmo instante (ver footWidthSlack).
                const auto width = [&](const auto& links, const Vec3& axis) {
                    return dot(links[footLinks[0]].position - links[footLinks[1]].position, axis);
                };
                const float gap = (width(w.body->links, side) - width(w.frame.links, clipSide)) / settings.footWidthSlack;
                imitation *= std::exp(-gap * gap);
            }
            // (05/11 22h20, ele: "corrija a corrida, ele esta invertendo os passos da pegada ao pisar, cruzando passos"; 22h55: "e
            // ele AINDA esta cruzando") Na corrida para a frente e nas diagonais da frente: o pe APOIADO mais para dentro (de lado
            // no rumo pedido, a partir da pelve) que o mesmo pe do clipe, alem de FootCrossMargin, derruba a semelhanca:
            // exp(-(falta / footCrossSlack)^2). A primeira versao (22h04) cobrava a largura entre os dois pes: o pe de tras
            // longe a deixava boa, e em 23 min a rede ainda pousava os dois pes no meio (esquerdo +0,3 cm, direito -0,1 da pelve;
            // 43% dos pousos do lado errado; o clipe +-5). Calibrado nas capturas da r31850 com 1,5 e 2 cm: a corrida 0 0,47 no
            // apoio, as diagonais 0,95-0,98, o clipe puro 1,00 (ele apoia ~10 cm para fora).
            // (23h: e a medida do pe cruzado no episodio - o portao da forca da corrida a usa: ver footCross.)
            if (w.runBand && w.band != 3 && (w.heading == 0 || w.heading == 1 || w.heading == 7)) {
                const Vec3 across { -std::sin(w.moveYaw), std::cos(w.moveYaw), 0.0f };
                float inward = 0.0f;
                bool any = false;
                for (int f = 0; f < 2; ++f) {
                    const bool down = std::any_of(w.body->interactions.begin(), w.body->interactions.end(),
                        [&](const RagdollInteraction3D& c) { return c.linkIndex == footLinks[f] && c.normalWorld.z > 0.5f; });
                    if (!down) continue;
                    const float side = f == 0 ? 1.0f : -1.0f;
                    const float body = side * dot(w.body->links[footLinks[f]].position - w.body->links[rootLink].position, across);
                    const float clip = side * dot(w.frame.links[footLinks[f]].position - w.frame.links[rootLink].position, across);
                    inward = any ? std::max(inward, clip - body) : clip - body;
                    any = true;
                }
                if (any) {
                    ++w.crossStance;
                    if (inward > CrossedInward) ++w.crossBad;
                    if (settings.footCrossSlack > 0.0f) {
                        const float gap = std::max(0.0f, inward - FootCrossMargin) / settings.footCrossSlack;
                        const float wide = std::max(0.0f, inward - FootCrossMargin) / FootCrossTailMeters;
                        imitation *= settings.cliffFootCross ? 0.75f * std::exp(-gap * gap) + 0.25f * std::exp(-wide * wide)
                                                        : std::exp(-gap * gap);
                    }
                }
            }
        }
        // (15/11, settings.launchArmsCalm) Na largada, as maos contra o peito nao mais rapidas que no clipe.
        if (settings.launchArmsCalm > 0.0f && w.commandSpeed > 0.0f && upperChestLink < w.body->links.size()
            && upperChestLink < w.frame.links.size()) {
            const float lw = launchWeight(w);
            if (lw > 0.0f) {
                const auto hands = [&](const std::vector<PhysicsBodyState3D>& links, float scale) {
                    float sum = 0.0f;
                    for (const std::uint32_t hand : handLinks)
                        if (hand < links.size()) sum += (links[hand].linearVelocity - links[upperChestLink].linearVelocity).length() * scale;
                    return 0.5f * sum;
                };
                const float excess = std::max(0.0f, hands(w.body->links, 1.0f) - hands(w.frame.links, rate) - 0.15f) / 0.4f;
                const float k = settings.launchArmsCalm * lw;
                imitation *= (1.0f - k) + k / (1.0f + excess * excess);
            }
        }
        // (15/11, settings.legAlignCommand) No comando andando, as pernas alinhadas tambem (fora das misturas entre clipes).
        if (settings.legAlignCommand && settings.legAlignWeight > 0.0f && w.maneuver == CommandManeuver && w.commandSpeed > 0.0f
            && !(w.blendClip < references.size() && w.time < w.blendFrom + w.blendSeconds))
            imitation *= legAlignFactor();
        // (04/11 06h40, ele: "a estabilidade pegar o tronco também, com os limites de variação natural de folga": a coluna
        // contra o clipe - o erro das juntas além de 7° e o giro do peito alto além de 1 rad/s; dentro da folga paga cheio.)
        // (09/11 01h20, ele: "o alinhamento dos joelhos, para evitar dele correr com os joelhos 'colados'"): de frente, o joelho
        // para dentro da linha quadril-tornozelo (no eixo de lado da pelve), contra o clipe no mesmo instante. Medido na r67150:
        // a rede com os pes a 40-47 cm e os joelhos a 27-30 cm, 3-6 cm por dentro da linha (o "X"); o clipe na linha.
        float kneeExcess = 0.0f;
        if (settings.kneeInwardSlack > 0.0f && w.locomotion && w.commandSpeed > 0.0f && leftThighLink < profile->links.size()
            && rightThighLink < profile->links.size()) {
            const Vec3 side { -std::sin(bodyHeading), std::cos(bodyHeading), 0.0f };
            // (17/11, settings.referenceSideFix) O clipe no eixo de lado dele.
            const Vec3 clipSide = settings.referenceSideFix ? Vec3 { -std::sin(referenceHeading), std::cos(referenceHeading), 0.0f } : side;
            // (A junta do filho no mundo: a ancora do modelo levada pelo pai.)
            const auto anchor = [&](const auto& links, std::uint32_t child) {
                const RagdollLinkDefinition3D& d = profile->links[child];
                const RagdollLinkDefinition3D& p = profile->links[static_cast<std::size_t>(d.parentIndex)];
                const PhysicsBodyState3D& s = links[static_cast<std::size_t>(d.parentIndex)];
                return s.position + (s.orientation * p.modelOrientation.conjugate()).normalized()
                    .rotate(d.inboundJoint.anchorModelPosition - p.modelPosition);
            };
            const auto inward = [&](const auto& links, int leg, const Vec3& side) {
                const float sign = leg == 0 ? 1.0f : -1.0f;
                const Vec3 hip = anchor(links, leg == 0 ? leftThighLink : rightThighLink);
                const Vec3 knee = anchor(links, shinLinks[leg]), ankle = anchor(links, footLinks[leg]);
                const float upper = (knee - hip).length(), lower = (ankle - knee).length();
                const float t = upper / std::max(1.0e-3f, upper + lower);
                const float line = dot(hip, side) + (dot(ankle, side) - dot(hip, side)) * t;
                return sign * (line - dot(knee, side));
            };
            for (int leg = 0; leg < 2; ++leg)
                kneeExcess = std::max(kneeExcess, inward(w.body->links, leg, side) - inward(w.frame.links, leg, clipSide));
            constexpr float KneeInwardMargin = 0.02f;
            const float gap = std::max(0.0f, kneeExcess - KneeInwardMargin) / settings.kneeInwardSlack;
            imitation *= std::exp(-gap * gap);
        }
        const PhysicsBodyState3D& pelvisNow = w.body->links[rootLink];
        const PhysicsBodyState3D& pelvisClip = w.frame.links[rootLink];
        const float pelvisRise = pelvisNow.position.z - pelvisClip.position.z;
        if (!w.riseKnown) {
            w.riseMean = pelvisRise;
            w.riseKnown = true;
        }
        w.riseMean += (pelvisRise - w.riseMean) * std::min(1.0f, TicksPerDecision * TickSeconds / PelvisHopSeconds);
        const float pelvisHop = std::max(0.0f, pelvisRise - w.riseMean);
        const float hop = std::max(0.0f, pelvisHop - PelvisHopSlack);
        const float hopNarrow = std::exp(-TrackPelvisHop11 * hop * hop);
        const float heightSteady = settings.cliffHop
            ? 0.75f * hopNarrow + 0.25f * std::exp(-(hop / HopTailMeters) * (hop / HopTailMeters)) : hopNarrow;
        const auto upTilt = [&](const PhysicsBodyState3D& link, std::size_t index) {
            const Vec3 up = (link.orientation * profile->links[index].modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
            return Vec3 { up.x, up.y, 0.0f };
        };
        const Vec3 travel { std::cos(w.moveYaw), std::sin(w.moveYaw), 0.0f };
        const auto leanOf = [&](std::size_t index) {
            Vec3 d = upTilt(w.body->links[index], index) - upTilt(w.frame.links[index], index);
            // (Na arrancada, a inclinacao para o rumo do deslocamento ate LaunchLean nao conta.)
            if (launch > 0.0f && w.commandSpeed > 0.0f) d = d - travel * std::clamp(dot(d, travel), 0.0f, LaunchLean * launch);
            return d.length();
        };
        const float leanLinks = std::max({ leanOf(rootLink), leanOf(upperChestLink), leanOf(contract->headLink()) });
        float footSlide = 0.0f;
        for (int f = 0; f < 2; ++f) {
            const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                [&](const RagdollContactPoint3D& c) { return c.linkIndex == footLinks[f]; });
            const Vec3 v = w.body->links[footLinks[f]].linearVelocity;
            if (down) footSlide = std::max(footSlide, std::max(0.0f, std::sqrt(v.x * v.x + v.y * v.y) - FootSlideSlack));
        }
        const float slideSteady = std::exp(-TrackFootSlide * footSlide * footSlide);
        const float leanExcess = std::max(0.0f, leanLinks - LeanSlack - LeanImproviseSlack * improvise - fireLean(w));
        const float leanSteady = std::exp(-TrackLean * leanExcess * leanExcess);
        const Firmness firm = firmness(w, rate, heightSteady, headSpin);
        const float stability = firm.total;
        // (04/11 03h30, ele: "aumentar um pouco mais a recompensa por similaridade da pose com o clipe - quero que ele
        // aprenda mas por enquanto sem fazer movimentos feios": na v4 a semelhanca 0,40 -> 0,50, a velocidade 0,30 -> 0,25 e a
        // estabilidade 0,20 -> 0,15.)
        // (v5, o giro: a semelhanca 0,65 e a velocidade - parado, quase de graca - 0,10.)
        const float leanGate = (0.75f + 0.25f * leanSteady) * (0.80f + 0.20f * heightSteady) * (0.85f + 0.15f * slideSteady);
        // (06/11, a arrancada: a pelve acima da do clipe na janela dela, na corrida, corta a nota em linha reta - sem saturar.)
        // (09h35, o A/B: so na arrancada pelo impulso - a frente e as diagonais da frente; nas largadas de lado e de costas a
        // punicao piorava as corridas 2/6.)
        const std::uint32_t launchHeading = w.heading % 8u;
        const float riseCut = settings.launchRiseSpan > 0.0f && w.runBand && launch > 0.0f
            && (launchHeading == 0u || launchHeading == 1u || launchHeading == 7u)
            ? 1.0f - 0.6f * launch * std::clamp((pelvisRise - LaunchRiseSlack) / settings.launchRiseSpan, 0.0f, 1.0f) : 1.0f;
        // (08/11 03h, v18 - ele: "sem semelhanca alguma com os clipes": desde a v14 a forma saiu da nota andando e a semelhanca
        // medida caiu de 0,54 para 0,05 (as pernas 11x mais longe do clipe). A v18 e a composicao da v13 - a imitacao 0,50 no
        // relogio do clipe, as janelas de improviso das trocas - com a largura da velocidade da v15.)
        // (08/11 04h40, v19: a v18 com 0,15 da imitacao passado a velocidade - com a imitacao viva (a regua larga), a rede
        // trocava velocidade por forma: o trote de frente 84% -> 67% da meta desde a v18, com a velocidade a 0,25 contra 0,45
        // da v15.)
        // (10/11 06h35, settings.liveLaunch) Na largada, a pose contra o quadro mais parecido do clipe numa janela de +-LiveLaunchWindow s
        // do relogio - medido, a imitacao no relogio valia 0,00-0,12 a largada inteira (o corpo sai de fase no primeiro passo) e so a
        // estabilidade pagava: passos curtos e firmes.
        if (settings.liveLaunch && !stationary && w.trackClip < references.size()) {
            const float lw = launchWeight(w);
            if (lw > 0.0f) {
                constexpr float LiveLaunchWindow = 0.3f;
                const auto [tNow, yawNow] = trackSample(w, 0.0f);
                const TrackingClipReference3D& ref = *references[w.trackClip];
                const float duration = std::max(1.0e-3f, ref.durationSeconds());
                const float lo = settings.imitationLoosen > 1.0f ? 1.0f / settings.imitationLoosen : 1.0f;
                float nearest = 0.0f;
                for (int k = -9; k <= 9; ++k) {
                    const float t = std::clamp(tNow + LiveLaunchWindow * static_cast<float>(k) / 9.0f, 0.0f, duration);
                    ref.frame(t, { root.x, root.y, floorUnder(w) }, yawNow, w.altFrame, twist(w), pitch(w), legPitch(w));
                    w.altErrors.assign(w.jointErrors.size(), 0.0f);
                    contract->jointErrors(w.body->links, w.altFrame, w.altErrors);
                    float p[6] {};
                    for (std::size_t j = 0; j < trackingParts.size() && j < w.altErrors.size(); ++j) p[trackingParts[j]] += w.altErrors[j];
                    const float pLegs = std::max(p[PartLeftLeg], p[PartRightLeg]);
                    const float value = 0.40f * std::exp(-lo * TrackLegs * pLegs) + 0.25f * std::exp(-lo * TrackSpine7 * p[PartTrunk])
                        + 0.20f * std::exp(-lo * TrackArms7 * p[PartArms]) + 0.15f * std::exp(-lo * TrackHead7 * p[PartHead]);
                    nearest = std::max(nearest, value);
                }
                imitation += (std::max(imitation, nearest) - imitation) * lw;
            }
        }
        const float steady = stability;
        const float legacy = riseCut * leanGate * (stationary
            ? 0.10f + 0.10f * velocityTerm + ((0.65f - 0.45f * fireFree) * imitation + (0.15f + 0.45f * fireFree) * stability)
                * (0.5f + 0.5f * velocityTerm)
            : 0.10f + (0.40f + 0.10f * improvise + 0.10f * launch) * velocityTerm
                + ((0.35f - 0.25f * improvise - 0.10f * launch) * imitation
                   + (0.15f + 0.15f * improvise) * stability)
                    * (0.5f + 0.5f * velocityTerm));
        // (08/11, settings.facingGate: o peito para a vista cobrado antes do encerramento.)
        float facing = 1.0f;
        // (06h30: so nas trocas - nos rumos retos ele cobrava tambem o giro natural da pelve na passada longa, e a rede
        // compensava no tronco e nos bracos: bracos 1,06 -> 1,19, pernas 0,87 -> 1,0, tronco 0,52 -> 0,62 em 1 h. "Virar" so
        // acontece nas trocas.)
        if (settings.facingGate > 0.0f && !stationary && ((w.maneuver >= 1 && w.maneuver <= 4) || isStar(w) || isCommand(w))) {
            const float excess = std::max(0.0f, std::abs(wrapAngle(bodyHeading - referenceHeading)) - FacingSlack);
            facing = 1.0f - settings.facingGate + settings.facingGate * std::exp(-(excess / FacingWidth) * (excess / FacingWidth));
        }
        // (08/11 18h40, ele: "os nossos trotes e corridas diagonais nao sao diagonais... a pose dele fica como se fosse para frente
        // ou de costas"): nos diagonais retos a referencia leva a pelve no rumo do deslocamento e o peito alto girado ate a vista,
        // e a rede ignorava o giro - medido (r61000): o peito alto do corpo a 5-13 graus do deslocamento, o da referencia a
        // 36-45. O mesmo portao do peito para a vista, no peito alto do corpo contra o da referencia.
        if (settings.diagonalChestGate > 0.0f && !stationary && w.maneuver == 0 && w.locomotion && (w.heading % 2u) == 1u
            && upperChestLink < w.body->links.size() && upperChestLink < w.frame.links.size()) {
            const auto chest = [&](const PhysicsBodyState3D& link) {
                const Vec3 f = (link.orientation * profile->links[upperChestLink].modelOrientation.conjugate()).normalized()
                                   .rotate({ 1.0f, 0.0f, 0.0f });
                return std::atan2(f.y, f.x);
            };
            // (09/11 03h35, ele: "nas locomocoes diagonais, ele precisa estar mais com o peito apontado na direcao de olhar":
            // medido, o peito alto 20-29 graus menos girado que a referencia em todas as diagonais - com a folga das trocas (20
            // graus, largura 34) isso custava 2%. Regua propria: 5 graus de folga, largura 17 - 25 graus custam ~30%.)
            constexpr float DiagonalChestSlack = 0.09f, DiagonalChestWidth = 0.30f;
            const float excess = std::max(0.0f, std::abs(wrapAngle(chest(w.body->links[upperChestLink]) - chest(w.frame.links[upperChestLink])))
                - DiagonalChestSlack);
            facing *= 1.0f - settings.diagonalChestGate
                + settings.diagonalChestGate * std::exp(-(excess / DiagonalChestWidth) * (excess / DiagonalChestWidth));
        }
        // (08/11 15h20: parando, a precisao da parada contra o ponto de referencia.)
        const float stopFactor = w.stopping && w.stopError >= 0.0f
            ? 1.0f - StopPrecisionWeight + StopPrecisionWeight * std::exp(-(w.stopError / StopPrecisionWidth) * (w.stopError / StopPrecisionWidth))
            : 1.0f;
        // (08/11 23h55, ele: "todos os movimentos estao muito lentos... investigar muito mais fundo": de pe e devagar rendia quase o
        // mesmo que correr - o trote lateral a 1 m/s 0,18 de nota, e com o estilo e a multa do potencial o PPO via 0,07 por passo
        // contra 0,16 a 1,4 m/s, enquanto uma queda custava 26. A velocidade manda: a nota inteira vezes o portao dela; no empurrao
        // e ate 1,5 s depois, recuperar ainda paga.)
        float speedGateNow = 1.0f;
        if (settings.speedGate > 0.0f && !stationary && w.locomotion) {
            constexpr float SpeedGatePushed = 0.6f;
            speedGateNow = settings.speedGate + (1.0f - settings.speedGate) * std::clamp(velocityTerm, 0.0f, 1.0f);
            if (w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f) speedGateNow = std::max(speedGateNow, SpeedGatePushed);
            // (09/11 10h50, settings.stopGateFloor: freando - a parada final e o soltar da tecla -, o corpo mais rapido que a referencia
            // zerava a nota da velocidade e o portao cortava a nota inteira, a imitacao da passada de freio junto; nas folhas a rede
            // se curvava para a frente sem por o pe a frente e caia quando o idle chegava.)
            // (11h55, o A/B: com o clipe de freio as quedas de frente 51 -> 25 de 96; sem ele - de costas, a referencia e a passada em
            // re - 13 -> 30: sem a forma da freada para imitar, o piso so tirava o motivo para desacelerar. So com o clipe de parada.)
            if (settings.stopGateFloor > 0.0f && w.stopping && w.stopPhase == 1 && w.stopClip >= 0)
                speedGateNow = std::max(speedGateNow, settings.stopGateFloor);
        }
        // (12/11 17h20, settings.starPathWeight) A estrela sem o encerramento por sair do caminho: a nota andando vezes ficar perto dele
        // (cauda longa: nunca zero), fora da passagem pela ponta e da meia-volta.
        if (settings.starPathWeight > 0.0f && isStar(w) && !(w.runTurn >= 0 || w.brakeTurnAt >= 0.0f || w.time < w.star.graceUntil)) {
            const float d = w.courseError / std::max(0.1f, settings.starPathWidth);
            facing *= 1.0f - settings.starPathWeight + settings.starPathWeight / (1.0f + d * d);
        }
        float score = speedGateNow * stopFactor * facing * legacy;
        // (17/11, settings.imitationGate - ver o .hpp) Andando, a nota inteira vezes a semelhanca (a forma manda, como na literatura).
        if (settings.imitationGate > 0.0f && !stationary && w.locomotion)
            score *= (1.0f - settings.imitationGate)
                + settings.imitationGate * std::min(1.0f, imitation / std::max(0.05f, settings.imitationGateFull));
        // (09/11 19h15, settings.poseBoost) Andando, a pose multiplica a nota inteira: a coluna, a inclinação e a altura da pelve
        // contra o clipe - medido desde r44000: a coluna 3,3-5 -> 16,7 graus por junta, a inclinação 3-5 -> 18,5 graus, a pelve 2 ->
        // 8 cm abaixo, a semelhança 0,58-0,79 -> 0,14; o portão da inclinação de antes tirava no maximo 25%.
        if (settings.poseBoost > 0.0f && !stationary && w.locomotion) {
            const float spineEx = std::max(0.0f, degrees(PartTrunk) - PoseSpineSlack) / PoseSpineWidth;
            const float leanEx = leanExcess * 180.0f / Pi / PoseLeanWidth;
            const float dropEx = std::max(0.0f, -pelvisRise - PoseDropSlack) / PoseDropWidth;
            const float pose = (std::max(0.0f, 1.0f - spineEx) + std::max(0.0f, 1.0f - leanEx) + std::max(0.0f, 1.0f - dropEx)) / 3.0f;
            score *= 1.0f - settings.poseBoost + settings.poseBoost * pose;
        }
        // (12/11 01h10) No levantar pelo clipe, a semelhanca do episodio e so a do clipe: a locomocao depois da troca nao entra (ver o
        // levantar pelo clipe em clipGetUpState).
        if (!w.clipGetUp) {
            w.imitationSum += imitation;
            ++w.trackSamples;
        }
        if (!guidedRise(w)) {
            w.stabilityDeficitSum += (1.0f - steady) * (1.0f - steady);
            ++w.stabilitySamples;
        }
        const float along = w.smoothVelocity.x * std::cos(w.moveYaw) + w.smoothVelocity.y * std::sin(w.moveYaw);
        w.locomotionFeedback = { score, part[PartRoot], legs, trunkAll, part[PartArms], feet, drift, along,
            velocityTerm, imitation, steady, pelvisSpin, headSpin, w.handUp, w.handSide, w.handTurn, degrees(PartTrunk),
            degrees(PartHead), degrees(PartArms), pelvisRise, pelvisHop, leanExcess * 180.0f / Pi,
            isCurve(w) || isStar(w) ? w.courseError : 0.0f, footSlide, firm.balance, firm.base, firm.body, firm.rhythm, firm.margin,
            firm.spinExcess, firm.total, firm.slip, firm.stumble, speedGateNow, kneeExcess };
        // (O erro do rumo do episodio: a pelve do corpo contra a do clipe; a velocidade no rumo pedido.)
        w.headingError = wrapAngle(bodyHeading - referenceHeading);
        w.faceErrorSum += std::abs(w.headingError);
        ++w.faceErrorSamples;
        if (w.commandSpeed > 0.0f) {
            w.towardSum += along;
            ++w.towardSamples;
            // (07h45: o que a referencia pede no rumo, no mesmo instante - a coluna 10 do episodio no rastreio.)
            const Vec3 want = wantedVelocity(w, 0.0f);
            w.wantedSum += want.x * std::cos(w.moveYaw) + want.y * std::sin(w.moveYaw);
        }
        // (10/11, settings.postureTail: o tronco e a cabeca contra o clipe na regua de cauda longa - ver o .hpp.)
        if (settings.postureTail > 0.0f) {
            const float d = std::max(0.0f, leanLinks - fireLean(w)) / PostureTailWidth;
            score *= 1.0f - settings.postureTail + settings.postureTail / (1.0f + d * d);
        }
        // (14/11 22h10, settings.launchSpeedShare) A velocidade ganha na largada da corrida de frente, fora dos multiplicadores de forma.
        if (settings.launchSpeedShare > 0.0f && w.locomotion && w.runBand && (w.heading == 0 || w.heading == 1 || w.heading == 7)) {
            const float share = launchWeight(w) * settings.launchSpeedShare;
            if (share > 0.0f) {
                const Vec3 vc = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
                const Vec3 want = wantedVelocity(w, 0.0f);
                const float c = std::cos(w.moveYaw), sn = std::sin(w.moveYaw);
                const float p = std::clamp((vc.x * c + vc.y * sn) / std::max(1.0f, want.x * c + want.y * sn), 0.0f, 1.0f);
                score = (1.0f - share) * score + share * p;
            }
        }
        return score;
    }
    // (09/11 14h, settings.freeStop) A nota da parada livre: o objetivo, nunca o como. O progresso da velocidade do centro de
    // massa a zero desde a ordem (linear: cada m/s perdido paga), o tronco de pe (inclinar para frear e livre ate FreeLeanSlack),
    // o peito na vista (a regra dele) e, perto de parado, a qualidade do parado: a pelve na altura do idle, o peito alto reto,
    // a pose do idle como termo leve (sem prender os pes) e quieto. Cair encerra com a multa, como sempre.
    static constexpr float FreeLeanSlack = 0.35f, FreeLeanWidth = 0.20f, FreeFaceSlack = 0.35f, FreeFaceWidth = 0.50f;
    // (10/11 00h35, settings.freeStepHeight) A altura de repouso de cada pe (o idle no chao) e o quanto o pe mais alto esta acima
    // dela agora (m); e o fator das fases livres: o pe alem da folga custa ate 40% da nota.
    std::array<float, 2> footRest {};
    bool footRestKnown = false;
    float footLift(const World& w) {
        if (!footRestKnown) {
            TrackingClipFrame3D frame;
            references[standingIndex]->frame(0.0f, {}, 0.0f, frame);
            for (int f = 0; f < 2; ++f)
                footRest[static_cast<std::size_t>(f)] = footLinks[f] < frame.links.size() ? frame.links[footLinks[f]].position.z : 0.0f;
            footRestKnown = true;
        }
        const float ground = floorUnder(w);
        float lift = 0.0f;
        for (int f = 0; f < 2; ++f)
            if (footLinks[f] < w.body->links.size())
                lift = std::max(lift, w.body->links[footLinks[f]].position.z - ground - footRest[static_cast<std::size_t>(f)]);
        return std::max(0.0f, lift);
    }
    // (00h45: a folga cresce 5 cm por m/s - freando ainda rapido a passada de corrida segue livre; quase parado, o passo baixo. Medido
    // (r82180): na parada o pe subia 35-38 cm na mediana, p90 50-53, ~8,7 passos em 3 s - o "trotar parado".)
    float lowStepFactor(const World& w, float speed) {
        if (settings.freeStepHeight <= 0.0f) return 1.0f;
        const float over = std::max(0.0f, footLift(w) - settings.freeStepHeight - 0.05f * speed) / 0.06f;
        return 1.0f - 0.4f * (1.0f - std::exp(-over * over));
    }
    // (09/11 19h15, settings.poseBoost) As reguas do reforço das poses: a coluna por junta (graus; a folga e o erro do melhor
    // periodo, 3,3-5 graus ate r44000 - hoje 16,7), a inclinação alem da do clipe (graus; hoje 18,5) e a pelve abaixo da do clipe
    // (m; hoje 8 cm); no parado, a coluna contra a do idle.
    // (20h05: as larguras dobradas e a pose pela media dos tres - com o produto nas reguas estreitas (6 graus, 8 graus, 3 cm) a pose
    // de agora (coluna 17, inclinacao 18,5, pelve 8 cm abaixo) dava ~0 e melhorar 30% tambem: o portao ficava em 0,5 e nada ensinava;
    // em 450 atualizacoes a pose andando nao se mexeu. Agora cada parte da sinal de longe: 30% melhor vale ~17% de nota.)
    // (20h40: em rampas lineares - a coluna ate 15 graus alem da folga, a inclinacao ate 30, a pelve ate 10 cm alem de 3: nas folhas
    // da r79000 o corpo corre com o tronco inteiro 30-40 graus a frente do clipe, e na cauda da curva larga corrigir 10 graus valia ~3%
    // da nota - menos que a velocidade que a inclinacao da. Na rampa cada grau paga igual, de qualquer distancia.)
    static constexpr float PoseSpineSlack = 5.0f, PoseSpineWidth = 15.0f, PoseLeanWidth = 30.0f, PoseDropSlack = 0.03f, PoseDropWidth = 0.10f;
    static constexpr float IdleSpineSlack = 4.0f, IdleSpineWidth = 6.0f;
    // O erro da coluna (abdomen, peito, peito alto) por junta contra um quadro, em graus.
    float spineDegrees(World& w, const TrackingClipFrame3D& frame) {
        w.altErrors.assign(w.jointErrors.size(), 0.0f);
        contract->jointErrors(w.body->links, frame, w.altErrors);
        float sum = 0.0f;
        for (std::size_t j = 0; j < trackingParts.size() && j < w.altErrors.size(); ++j)
            if (trackingParts[j] == PartTrunk) sum += w.altErrors[j];
        return trackingPartCount[PartTrunk] > 0.0f ? std::sqrt(sum / trackingPartCount[PartTrunk]) * 180.0f / Pi : 0.0f;
    }
    static constexpr float FreeNearSpeed = 0.5f, FreeStopSettleSpeed = 0.5f, FreeQuietWeight = 2.0f;
    // (10/11, settings.basePosture) A postura base vale com o pedido de velocidade zero: freando (a ordem dada) ou parado.
    static constexpr float BasePostureNearSpeed = 1.0f, BasePostureStepCost = 0.02f;
    bool basePostureOn(const World& w) const {
        return settings.basePosture > 0.0f && w.locomotionControl && (w.stopping || idleHeld(w));
    }
    TrackingBasePosture3D basePosture(World& w) {
        // (Fora da locomocao o rumo da vista nunca era posto - ficava o do episodio anterior: o do peito quando o parado comeca.)
        if (!w.locomotion && !w.idleFaceSet) {
            w.faceYaw = chestYaw(w);
            w.idleFaceSet = true;
        }
        const Vec3 root = w.body->links[rootLink].position;
        const std::array<std::uint32_t, 2> feet { footLinks[0], footLinks[1] };
        const auto measure = [&](std::size_t clip) {
            const float t = w.trackClip == clip ? trackTime(w, 0.0f) : 0.0f;
            // (12h40: o espelho no mesmo rumo - os dois idles tem o peito na vista com o mesmo giro, medido +1,4 e -1,4 graus; pela pelve,
            // como o StopIdleMirror, o peito do espelho ficava 45 graus fora da vista e o atrator girava o corpo para la.)
            references[clip]->frame(t, { root.x, root.y, floorUnder(w) }, w.faceYaw, w.baseFrame);
            return measureTrackingBasePosture3D(*profile, w.body->links, w.baseFrame.links, rootLink, upperChestLink, feet);
        };
        // (Parado no idle, o clipe que ele segue; freando, o normal ou o espelho - o que estiver mais perto.)
        // (11/11, E5-10: depois do parkStop (stopPhase 2) tambem o que ele segue - o escolhido ali. Antes, o atrator seguia
        // escolhendo o mais perto a cada decisao e o rastreio o do parkStop: dois alvos, e o corpo assentava torcido entre eles,
        // a pelve no espelho e o peito 30-50 graus fora da vista, a coluna 12 graus de lado - medido na E5-9.)
        if (((idleHeld(w) && !w.stopping) || (w.stopping && w.stopPhase == 2)) && w.trackClip < references.size())
            return measure(w.trackClip);
        TrackingBasePosture3D best = measure(standingIndex);
        if (standingMirrorIndex < references.size() && standingMirrorIndex != standingIndex) {
            const TrackingBasePosture3D mirror = measure(standingMirrorIndex);
            if (mirror.distance() < best.distance()) best = mirror;
        }
        return best;
    }
    // (11/11 10h05, ele: "o correto e a pelvis com abdomen com peito com upperchest e neck... o alinhamento de todos"; 11h10: medido,
    // a reta entre os elos deixou o abdome 17 graus para tras do idle - a lombar arqueada) O alinhamento da coluna inteira contra a
    // da referencia: o eixo de cima de cada elo da cadeia Pelvis-Abdomen-Chest-UpperChest-Neck contra o do mesmo elo do quadro
    // seguido (no parado o idle alerta, no giro o clipe dele); cada um 1/(1+(angulo/5 graus)^2), a media dos cinco.
    mutable std::array<std::size_t, 5> spineChain {};
    mutable bool spineChainReady = false;
    void readySpineChain() const {
        if (spineChainReady) return;
        static constexpr const char* Chain[5] { "Pelvis", "Abdomen", "Chest", "UpperChest", "Neck" };
        for (std::size_t k = 0; k < 5; ++k) {
            spineChain[k] = profile->links.size();
            for (std::size_t i = 0; i < profile->links.size(); ++i)
                if (profile->links[i].id == Chain[k]) spineChain[k] = i;
        }
        spineChainReady = true;
    }
    // (11/11 11h45, ele: "nao pode nunca mais ser 'o peito mira', precisa ser o tronco todo mira, pois o problema dele estar torto e
    // justamente porque ele tenta deixar so o peito mirando") O rumo de cada elo da cadeia do tronco (a frente dele no plano) e o
    // rumo do tronco, a media circular dos cinco.
    float linkHeading(const PhysicsBodyState3D& link, std::size_t i) const {
        const Vec3 f = (link.orientation * profile->links[i].modelOrientation.conjugate()).normalized().rotate({ 1.0f, 0.0f, 0.0f });
        return std::atan2(f.y, f.x);
    }
    float trunkYawOf(std::span<const PhysicsBodyState3D> links) const {
        readySpineChain();
        float sx = 0.0f, sy = 0.0f;
        for (const std::size_t i : spineChain)
            if (i < links.size()) {
                const float h = linkHeading(links[i], i);
                sx += std::cos(h);
                sy += std::sin(h);
            }
        return std::atan2(sy, sx);
    }
    // O tronco inteiro mirando como no quadro seguido: cada elo da cadeia no rumo do mesmo elo do quadro, 1/(1+(erro/10 graus)^2), a media.
    float trunkAim(const World& w, const TrackingClipFrame3D& reference) const {
        readySpineChain();
        float sum = 0.0f;
        int count = 0;
        for (const std::size_t i : spineChain)
            if (i < w.body->links.size() && i < reference.links.size()) {
                const float e = std::abs(wrapAngle(linkHeading(w.body->links[i], i) - linkHeading(reference.links[i], i))) * 180.0f / Pi;
                sum += 1.0f / (1.0f + (e / 10.0f) * (e / 10.0f));
                ++count;
            }
        return count > 0 ? sum / static_cast<float>(count) : 1.0f;
    }
    float spineAlignment(const World& w, const TrackingClipFrame3D& reference) const {
        readySpineChain();
        const auto upOf = [&](const PhysicsBodyState3D& link, std::size_t i) {
            return (link.orientation * profile->links[i].modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
        };
        float sum = 0.0f;
        int count = 0;
        for (const std::size_t i : spineChain) {
            if (i >= w.body->links.size() || i >= reference.links.size()) continue;
            const float angle = std::acos(std::clamp(dot(upOf(w.body->links[i], i), upOf(reference.links[i], i)), -1.0f, 1.0f)) * 180.0f / Pi;
            sum += 1.0f / (1.0f + (angle / 5.0f) * (angle / 5.0f));
            ++count;
        }
        const float segments = count > 0 ? sum / static_cast<float>(count) : 1.0f;
        // (11/11 11h20, ele: "precisa ficar RETA em relacao ao mundo tambem": a linha da pelve ao pescoco contra a vertical do mundo -
        // cada grau alem da inclinacao da mesma linha no quadro seguido custa, 1/(1+(excesso/5 graus)^2). Metade da medida.)
        const std::size_t pelvis = spineChain[0], neck = spineChain[4];
        if (pelvis >= w.body->links.size() || neck >= w.body->links.size() || pelvis >= reference.links.size() || neck >= reference.links.size())
            return segments;
        const auto tiltOf = [](Vec3 line) {
            const float n = std::max(1.0e-4f, line.length());
            return std::acos(std::clamp(line.z / n, -1.0f, 1.0f)) * 180.0f / Pi;
        };
        const float excess = std::max(0.0f, tiltOf(w.body->links[neck].position - w.body->links[pelvis].position)
            - tiltOf(reference.links[neck].position - reference.links[pelvis].position));
        return 0.5f * segments + 0.5f / (1.0f + (excess / 5.0f) * (excess / 5.0f));
    }
    float freeStopScore(World& w, Vec3 velocity) {
        const Vec3 root = w.body->links[rootLink].position;
        w.smoothVelocity = w.smoothVelocity + (Vec3 { velocity.x, velocity.y, 0.0f } - w.smoothVelocity)
            * (TicksPerDecision * TickSeconds / 0.25f);
        const float speed = std::hypot(velocity.x, velocity.y);
        const float progress = std::clamp(1.0f - speed / w.freeFromSpeed, 0.0f, 1.0f);
        const float near = std::exp(-(speed / FreeNearSpeed) * (speed / FreeNearSpeed));
        // (O idle no rumo do proprio corpo: a forma e a altura, nao para onde olha.)
        const TrackingClipReference3D& idle = *references[standingIndex];
        const float t = w.trackClip == standingIndex ? trackTime(w, 0.0f) : 0.0f;
        idle.frame(t, {}, 0.0f, w.idleFrame);
        const float yaw = wrapAngle(pelvisHeading(w.body->links[rootLink]) - pelvisHeading(w.idleFrame.links[rootLink]));
        idle.frame(t, { root.x, root.y, 0.0f }, yaw, w.idleFrame);
        underPelvis(w.idleFrame, root.x, root.y);
        const float match = contract->reward(w.body->links, w.idleFrame, 1.0f).total;
        const float drop = w.idleFrame.links[rootLink].position.z - root.z;
        float tilt = 0.0f;
        if (upperChestLink < w.body->links.size() && upperChestLink < w.idleFrame.links.size()) {
            const auto up = [&](const PhysicsBodyState3D& link) {
                return (link.orientation * profile->links[upperChestLink].modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
            };
            tilt = std::acos(std::clamp(dot(up(w.body->links[upperChestLink]), up(w.idleFrame.links[upperChestLink])), -1.0f, 1.0f));
        }
        const float lean = std::max(0.0f, tilt - FreeLeanSlack) / FreeLeanWidth;
        const float upright = std::exp(-lean * lean);
        const float off = std::max(0.0f, std::abs(wrapAngle(w.faceYaw - chestYaw(w))) - FreeFaceSlack) / FreeFaceWidth;
        const float face = std::exp(-off * off);
        const float low = std::max(0.0f, drop - 0.015f) / 0.03f;
        float tall = std::exp(-low * low);
        const float bent = std::max(0.0f, tilt - 0.10f) / 0.17f;
        float straight = std::exp(-bent * bent);
        if (settings.idleWide) {
            // (settings.idleWide: o nucleo fino mais a rampa larga - 20 cm de agachamento, 0,60 rad (34 graus) de peito alem da folga.)
            tall = 0.5f * tall + 0.5f * std::clamp(1.0f - std::max(0.0f, drop - 0.015f) / 0.20f, 0.0f, 1.0f);
            straight = 0.5f * straight + 0.5f * std::clamp(1.0f - std::max(0.0f, tilt - 0.10f) / 0.60f, 0.0f, 1.0f);
        }
        double excess = 0.0;
        for (const PhysicsBodyState3D& link : w.body->links) {
            const float e = std::max(0.0f, link.angularVelocity.length() - IdleQuietSlack);
            excess += static_cast<double>(e) * e;
        }
        excess /= static_cast<double>(std::max<std::size_t>(1, w.body->links.size()));
        const float quiet = static_cast<float>(std::exp(-static_cast<double>(FreeQuietWeight) * excess));
        float stand = 0.35f * tall + 0.30f * straight + 0.20f * match + 0.15f * quiet;
        if (settings.freeIdleMatch > 0.0f) {
            // (settings.freeIdleMatch: a semelhanca com o idle na regua com suavidade 2 vale o peso dela; o resto, o complemento.)
            float matchSoft = contract->reward(w.body->links, w.idleFrame, 2.0f).total;
            if (settings.idleWide) matchSoft = 0.5f * matchSoft + 0.5f * contract->reward(w.body->links, w.idleFrame, 6.0f).total;
            stand = settings.freeIdleMatch * matchSoft
                + (1.0f - settings.freeIdleMatch) * (0.35f * tall + 0.30f * straight + 0.15f * quiet) / 0.80f;
        }
        float score = face * (0.10f + 0.45f * progress + 0.15f * upright + 0.30f * near * stand);
        if (basePostureOn(w)) {
            // (settings.basePosture: freado, o idle alerta pela regua que nao satura - a base dos pes, os giros, o peito, a altura.)
            const float attract = 0.85f * basePosture(w).attraction() + 0.15f * quiet;
            score = face * (0.10f + 0.30f * progress + 0.10f * upright + 0.50f * near * attract);
        }
        if (settings.stopIdlePose > 0.0f) {
            // (settings.stopIdlePose, E5-7 - ver o .hpp.)
            // (O nucleo do DeepMimic satura - longe da pose ele da ~0 e nao puxa de volta, diario 43; metade da pose numa regua de
            // cauda longa por junta, 1/(1+(erro/0,15 rad)^2), que nunca zera.)
            w.jointErrors.resize(trackingParts.size());
            contract->jointErrors(w.body->links, w.idleFrame, w.jointErrors);
            float tail = 0.0f;
            for (const float e : w.jointErrors) tail += 1.0f / (1.0f + e / (0.15f * 0.15f));
            tail /= static_cast<float>(std::max<std::size_t>(1, w.jointErrors.size()));
            const float k = settings.stopIdlePose * near;
            score = (1.0f - k) * score + k * face * (0.25f + 0.75f * (0.5f * match + 0.5f * tail));
        }
        float headTilt = 0.0f;
        if (settings.stopPosture > 0.0f && upperChestLink < w.body->links.size()) {
            // (settings.stopPosture) Frear ereto: o peito alto e a cabeca contra os do idle, na nota inteira e desde a ordem.
            std::size_t head = profile->links.size();
            for (std::size_t i = 0; i < profile->links.size(); ++i)
                if (profile->links[i].id == "Head") head = i;
            if (head < w.body->links.size() && head < w.idleFrame.links.size()) {
                const auto upOf = [&](const PhysicsBodyState3D& link, std::size_t i) {
                    return (link.orientation * profile->links[i].modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
                };
                headTilt = std::acos(std::clamp(dot(upOf(w.body->links[head], head), upOf(w.idleFrame.links[head], head)), -1.0f, 1.0f));
            }
            constexpr float ChestSlack = 0.10f, ChestWidth = 0.17f, HeadSlack = 0.15f, HeadWidth = 0.20f;
            const float chestBad = std::max(0.0f, tilt - ChestSlack) / ChestWidth;
            const float headBad = std::max(0.0f, headTilt - HeadSlack) / HeadWidth;
            // (10/11 14h05, settings.postureTail: no pos-freio tambem a regua de cauda longa - a gaussiana dava ~0 com o tronco
            // caido a 45 graus e nao puxava de volta; ele: "muito mole pos freiada, parece a coluna quebrada".)
            const float upright = settings.postureTail > 0.0f ? 1.0f / (1.0f + chestBad * chestBad + headBad * headBad)
                                                               : std::exp(-chestBad * chestBad - headBad * headBad);
            score *= 1.0f - settings.stopPosture + settings.stopPosture * upright;
        }
        // (settings.freeStandGate: parado, a pose de pe multiplica a nota - de pe ereto na altura do idle, nao curvado e agachado.)
        // (settings.poseBoost: e a coluna contra a do idle - a torção e a curva, que o peito alto sozinho nao ve.)
        const float spineIdle = settings.poseBoost > 0.0f
            ? std::exp(-std::pow(std::max(0.0f, spineDegrees(w, w.idleFrame) - IdleSpineSlack) / IdleSpineWidth, 2.0f)) : 1.0f;
        if (settings.freeStandGate > 0.0f) score *= 1.0f - settings.freeStandGate * near * (1.0f - tall * straight * spineIdle);
        // (settings.freeStepHeight: nada de "trotar parado" - o passo baixo e natural.)
        score *= lowStepFactor(w, speed);
        // (O retorno: a nota, a pelve contra o idle (m), a velocidade, o progresso, a pose do idle, a quietude e o peito alto em
        // graus; o portao da velocidade em zero - o estilo sem portao.)
        w.locomotionFeedback = {};
        w.locomotionFeedback[0] = score;
        w.locomotionFeedback[6] = -drop;
        w.locomotionFeedback[7] = speed;
        w.locomotionFeedback[8] = progress;
        w.locomotionFeedback[9] = match;
        w.locomotionFeedback[10] = quiet;
        w.locomotionFeedback[16] = tilt * 180.0f / Pi;
        w.locomotionFeedback[17] = headTilt * 180.0f / Pi;
        return score;
    }
    // (09/11 15h, settings.freeTroca) A nota da troca livre: o progresso da velocidade do centro de massa ate a pedida nova desde o
    // comeco (linear), o peito na vista (somado - na meia-volta ele comeca de costas para ela), de pe (o tronco livre ate
    // FreeLeanSlack) e perto da pedida com o peito na vista. Cair encerra com a multa; virar e sair do rumo nao encerram aqui.
    float freeTrocaScore(World& w, Vec3 velocity) {
        const Vec3 root = w.body->links[rootLink].position;
        w.smoothVelocity = w.smoothVelocity + (Vec3 { velocity.x, velocity.y, 0.0f } - w.smoothVelocity)
            * (TicksPerDecision * TickSeconds / 0.25f);
        const float gx = std::cos(w.moveYaw) * w.commandSpeed, gy = std::sin(w.moveYaw) * w.commandSpeed;
        const float e = std::hypot(velocity.x - gx, velocity.y - gy);
        const float progress = std::clamp(1.0f - e / w.freeTrocaError, 0.0f, 1.0f);
        const float width = 0.5f + 0.15f * w.commandSpeed;
        const float near = std::exp(-(e / width) * (e / width));
        float tilt = 0.0f;
        if (upperChestLink < w.body->links.size()) {
            const TrackingClipReference3D& idle = *references[standingIndex];
            idle.frame(0.0f, {}, 0.0f, w.idleFrame);
            const float yaw = wrapAngle(pelvisHeading(w.body->links[rootLink]) - pelvisHeading(w.idleFrame.links[rootLink]));
            idle.frame(0.0f, { root.x, root.y, 0.0f }, yaw, w.idleFrame);
            const auto up = [&](const PhysicsBodyState3D& link) {
                return (link.orientation * profile->links[upperChestLink].modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
            };
            if (upperChestLink < w.idleFrame.links.size())
                tilt = std::acos(std::clamp(dot(up(w.body->links[upperChestLink]), up(w.idleFrame.links[upperChestLink])), -1.0f, 1.0f));
        }
        const float lean = std::max(0.0f, tilt - FreeLeanSlack) / FreeLeanWidth;
        const float upright = std::exp(-lean * lean);
        const float off = std::max(0.0f, std::abs(wrapAngle(w.faceYaw - chestYaw(w))) - FreeFaceSlack) / FreeFaceWidth;
        const float face = std::exp(-off * off);
        const float score = 0.10f + 0.40f * progress + 0.20f * face + 0.15f * upright + 0.15f * near * face;
        w.locomotionFeedback = {};
        w.locomotionFeedback[0] = score;
        w.locomotionFeedback[7] = (velocity.x * gx + velocity.y * gy) / std::max(0.1f, w.commandSpeed);
        w.locomotionFeedback[8] = progress;
        w.locomotionFeedback[16] = tilt * 180.0f / Pi;
        return score;
    }
    // (10/11 00h20, settings.freePush) A janela livre do empurrao andando: o empurrao e PushFreeSeconds depois; a nota e voltar a
    // velocidade pedida (larga), de pe (o tronco livre ate FreeLeanSlack) e o peito na vista - sem imitacao do clipe.
    static constexpr float PushFreeSeconds = 1.5f;
    bool pushFreeNow(const World& w) const {
        return settings.freePush && w.locomotion && w.commandSpeed > 0.0f && !w.stopping && !w.freeTroca
            && (w.pushTicks > 0 || w.time < w.lastPushEnd + PushFreeSeconds);
    }
    float freePushScore(World& w, Vec3 velocity) {
        w.smoothVelocity = w.smoothVelocity + (Vec3 { velocity.x, velocity.y, 0.0f } - w.smoothVelocity)
            * (TicksPerDecision * TickSeconds / 0.25f);
        const float gx = std::cos(w.moveYaw) * w.commandSpeed, gy = std::sin(w.moveYaw) * w.commandSpeed;
        const float e = std::hypot(velocity.x - gx, velocity.y - gy);
        const float width = 0.5f + 0.25f * w.commandSpeed;
        const float back = std::exp(-(e / width) * (e / width));
        float tilt = 0.0f;
        if (upperChestLink < w.body->links.size()) {
            const Vec3 up = (w.body->links[upperChestLink].orientation * profile->links[upperChestLink].modelOrientation.conjugate())
                .normalized().rotate({ 0.0f, 0.0f, 1.0f });
            tilt = std::acos(std::clamp(up.z, -1.0f, 1.0f));
        }
        const float lean = std::max(0.0f, tilt - FreeLeanSlack) / FreeLeanWidth;
        const float upright = std::exp(-lean * lean);
        const float off = std::max(0.0f, std::abs(wrapAngle(w.faceYaw - chestYaw(w))) - FreeFaceSlack) / FreeFaceWidth;
        const float face = std::exp(-off * off);
        const float score = (0.10f + 0.45f * back + 0.25f * face + 0.20f * upright) * lowStepFactor(w, std::hypot(velocity.x, velocity.y));
        w.locomotionFeedback = {};
        w.locomotionFeedback[0] = score;
        w.locomotionFeedback[7] = (velocity.x * gx + velocity.y * gy) / std::max(0.1f, w.commandSpeed);
        w.locomotionFeedback[8] = back;
        w.locomotionFeedback[16] = tilt * 180.0f / Pi;
        return score;
    }
    // A distancia da pose do corpo a de um quadro: as coordenadas das juntas e o baixo (a gravidade) no quadro da pelve.
    float poseDistance(const RagdollState3D& body, const TrackingClipFrame3D& frame) const {
        const auto down = [&](const PhysicsBodyState3D& pelvis) {
            return (pelvis.orientation * profile->links[rootLink].modelOrientation.conjugate()).normalized().conjugate().rotate(
                { 0.0f, 0.0f, -1.0f });
        };
        float d = 4.0f * (down(body.links[rootLink]) - down(frame.links[rootLink])).lengthSquared();
        for (std::size_t i = 0; i < body.joints.size() && i < frame.coordinates.size() && i < profile->links.size(); ++i) {
            const Vec3 c = frame.coordinates[i];
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (!profile->links[i].inboundJoint.axes[axis].enabled) continue;
                const float e = body.joints[i].positionRadians[axis] - (axis == 0 ? c.x : axis == 1 ? c.y : c.z);
                d += e * e;
            }
        }
        return d;
    }
    // O levantar guiado (01/11 23h10): o clipe de levantar e o instante mais parecidos com a pose de agora (os quatro, a
    // cada 0,1 s nos primeiros 30% - a parte deitada; 02/11 00h: com 60% e refazendo no fim, ele aprendeu a ficar deitado
    // de bracos erguidos imitando o comeco do clipe de costas), o rumo dele alinhando a pelve dele a do corpo.
    void beginGetUpGuide(World& w) {
        float best = std::numeric_limits<float>::max(), bestTime = 0.0f;
        std::size_t bestClip = std::numeric_limits<std::size_t>::max();
        for (std::size_t c = 0; c < std::size(GetUpClips) && c < references.size(); ++c)
            for (float t = 0.0f; t <= 0.3f * references[c]->durationSeconds(); t += 0.1f) {
                references[c]->frame(t, {}, 0.0f, w.frame);
                const float d = poseDistance(*w.body, w.frame);
                if (d < best) {
                    best = d;
                    bestClip = c;
                    bestTime = t;
                }
            }
        if (bestClip >= references.size()) return;
        references[bestClip]->frame(bestTime, {}, 0.0f, w.frame);
        w.trackClip = bestClip;
        w.trackStart = bestTime;
        w.trackFrom = w.time;
        w.trackScale = settings.getUpSeconds > 0.0f ? references[bestClip]->durationSeconds() / settings.getUpSeconds : 1.0f;
        w.trackYaw = wrapAngle(bodyYaw(w) - trackingBodyYaw3D(profile->links[rootLink], w.frame.links[rootLink]));
        w.getUpGuided = true;
        w.idleAfterGetUp = false;
        w.sprintGuided = false;
    }
    // O rolar pelo clipe: o levantar de costas (ou o espelho) no instante dos primeiros 0,5 s mais parecido com a pose.
    static constexpr float RollEndSeconds = 0.75f;
    static constexpr float RollSpeed = 1.25f;
    static constexpr float RollBellyUp = 0.5f;
    static constexpr float RollPelvisHeight = 0.45f;
    void beginRoll(World& w) {
        float best = std::numeric_limits<float>::max();
        for (const std::size_t c : rollIndices)
            for (float t = 0.0f; t <= 0.5f; t += 0.05f) {
                references[c]->frame(t, {}, 0.0f, w.frame);
                const float d = poseDistance(*w.body, w.frame);
                if (d < best) {
                    best = d;
                    w.rollClip = c;
                    w.rollStart = t;
                }
            }
        w.rollFrom = w.time;
    }
    float rollTime(const World& w, float ahead) const {
        return w.rollStart + (w.time + ahead - w.rollFrom) * RollSpeed;
    }
    // No levantar em movimento guiado, agachado (crouchTimes): a arrancada agachada (crouched_to_sprint)
    // apontada para o alvo - ele, 23h30: "observa-se especial relevancia ao clipe do crouch to sprint".
    void beginSprintGuide(World& w) {
        const Vec3 root = w.body->links[rootLink].position;
        w.trackClip = sprintStartIndex;
        w.trackStart = 0.0f;
        w.trackFrom = w.time;
        w.trackScale = 1.0f;
        w.trackYaw = planarAngle({ w.target.x - root.x, w.target.y - root.y, 0.0f });
        w.sprintGuided = true;
    }
    // No fim do clipe de levantar: de pe, o parado em alerta (o normal ou o espelho, o mais perto da pose), na pelve do
    // corpo; com o pedido de deslocamento, a rede livre; ainda no chao, outro clipe de onde ele esta.
    void endGetUpGuide(World& w, bool standing) {
        // (02/11 00h: sem refazer o clipe - o alvo fica o ultimo quadro dele, de pe, ate ele ficar de pe; refazendo, ele
        // voltava ao comeco deitado e a imitacao pagava para sempre.)
        if (!standing) return;
        const bool tracked = guidedRise(w);
        w.getUpGuided = false;
        w.sprintGuided = false;
        w.trackClip = std::numeric_limits<std::size_t>::max();
        if (w.commandSpeed > 0.0f) return;
        startIdle(w);
        // (05/11 04h40: do fim do clipe de levantar ao idle, os alvos de agora indo aos do idle - a entrega de sempre.)
        if (tracked && settings.idleBlendSeconds > 0.0f && w.blendAction.size() == contract->actionSize()) {
            w.idleAfterGetUp = true;
            w.idleBlendFrom = w.time;
            w.idleBlendPose = w.blendAction;
        }
    }
    // (11/11 14h, settings.clipGetUpFraction) A nota do levantar pelo clipe (TrackingClipGetUp3D), o tempo longe dele (o clipe no fim
    // e o corpo ainda nao de pe tambem conta: o fim do clipe e de pe) e o voo de pe.
    // O clipe comeca de costas (o peito alto para cima: a frente acima de 0,5 no comeco).
    bool clipStartsSupine(std::size_t clip) {
        const std::lock_guard<std::mutex> lock(clipCacheMutex);
        if (clip >= supineStartCache.size()) supineStartCache.resize(clip + 1, -1);
        if (supineStartCache[clip] < 0) {
            TrackingClipFrame3D frame;
            references[clip]->frame(0.0f, {}, 0.0f, frame);
            supineStartCache[clip] = TrackingClipGetUp3D::chestFront(*contract, *profile, frame.links) > 0.5f ? 1 : 0;
        }
        return supineStartCache[clip] == 1;
    }

    // O relogio do clipe (o tempo que o rolamento livre esperou ou adiantou descontado).
    float clipTime(const World& w) const { return w.clipStart + w.time - w.clipHeld; }
    // (14/11 v1, settings.clipTrackMask) O levantar limpo dos exercicios da mascara volta ao rastreio do clipe (a nota e a semelhanca, o
    // fim quando longe - o de 11/11) mesmo com o adaptativo ligado; sob fogo e no comando seguem adaptativos.
    bool adaptiveFor(const World& w) const {
        if (!settings.clipGetUpAdaptive) return false;
        return !(settings.clipTrackMask != 0u && w.clipExercise < 32u && ((settings.clipTrackMask >> w.clipExercise) & 1u) != 0u
                 && w.fireKind == 0 && !isCommand(w));
    }
    bool clipSupineNow(const World& w) {
        thread_local TrackingClipFrame3D frame;
        references[w.clipReference]->frame(clipTime(w), {}, 0.0f, frame);
        return TrackingClipGetUp3D::chestFront(*contract, *profile, frame.links) > 0.0f;
    }

    // A primeira hora em que o peito alto do clipe fica abaixo de TrackingClipGetUp3D::FreeRollProne (o rolamento feito); -1 sem ela.
    float freeRollUntil(std::size_t clip) {
        const std::lock_guard<std::mutex> lock(clipCacheMutex);
        if (clip >= freeRollUntilCache.size()) freeRollUntilCache.resize(clip + 1, std::nanf(""));
        if (std::isnan(freeRollUntilCache[clip])) {
            const TrackingClipReference3D& reference = *references[clip];
            TrackingClipFrame3D frame;
            float found = -1.0f;
            for (float t = 0.0f; t <= reference.durationSeconds() && found < 0.0f; t += TicksPerDecision * TickSeconds) {
                reference.frame(t, {}, 0.0f, frame);
                if (TrackingClipGetUp3D::chestFront(*contract, *profile, frame.links) < TrackingClipGetUp3D::FreeRollProne) found = t;
            }
            freeRollUntilCache[clip] = found;
        }
        return freeRollUntilCache[clip];
    }

    // O clipe reancorado no corpo na hora t: o rumo (pelve -> cabeca no chao, que deitado e o comprido do corpo) e a posicao da pelve
    // no chao (a altura do chao do clipe fica).
    void clipReanchor(World& w, float t) {
        const TrackingClipReference3D& reference = *references[w.clipReference];
        const std::uint32_t head = contract->headLink();
        const auto heading = [&](std::span<const PhysicsBodyState3D> links) {
            const Vec3 d = links[head].position - links[rootLink].position;
            return std::atan2(d.y, d.x);
        };
        reference.frame(t, w.clipGround, w.clipYaw, w.frame);
        float turn = heading(w.body->links) - heading(w.frame.links);
        turn = std::atan2(std::sin(turn), std::cos(turn));
        w.clipYaw += turn;
        reference.frame(t, w.clipGround, w.clipYaw, w.frame);
        const Vec3 shift = w.body->links[rootLink].position - w.frame.links[rootLink].position;
        w.clipGround.x += shift.x;
        w.clipGround.y += shift.y;
    }

    // (12/11 08h20, settings.adaptiveRetry - ver o .hpp) O fracasso abre uma nova tentativa; devolve se abriu. w.frame e o quadro do
    // clipe de agora.
    static constexpr float RetryHeadMeters = 0.6f, RetrySeconds = 0.2f, RetryDropMeters = 0.35f;
    bool maybeRetry(World& w, const TrackingGetUpPosture3D& now) {
        const float deliverPoint = w.clipExercise < clipGetUpDeliver.size() && clipGetUpDeliver[w.clipExercise] > 0.0f
            ? clipGetUpDeliver[w.clipExercise] : references[w.clipReference]->durationSeconds();
        w.attemptPeakHead = std::max(w.attemptPeakHead, now.headHeight);
        // Perdeu o que ganhou (caiu de volta) ou o relogio ja passou da entrega e ele segue no chao.
        const bool fellBack = now.headHeight < RetryHeadMeters && w.attemptPeakHead - now.headHeight >= RetryDropMeters;
        const bool stalled = clipTime(w) >= deliverPoint - 1.0e-4f && now.headHeight < RetryHeadMeters;
        // (12/11 14h15, ele: "no correndo de brucos... se a forca fez ele cair de volta e ele caiu DE COSTAS... ele nao esta tendo essa
        // expertise de tentar se levantar de outra forma que nao de brucos") Virou para o outro lado no chao: deitado (cabeca abaixo de
        // 0,5 m) com o peito claramente do lado oposto ao do comeco do clipe de agora - a tentativa nova ja no clipe do lado certo.
        const float front = TrackingClipGetUp3D::chestFront(*contract, *profile, w.body->links);
        // (14/11 v1, settings.clipSideNow) Com o clipe que rola (o de costas que vira de brucos), o lado e o do clipe no quadro de agora.
        const bool clipSupine = settings.clipSideNow ? clipSupineNow(w) : clipStartsSupine(w.clipReference);
        const bool turnedOver = now.headHeight < 0.5f && (clipSupine ? front < -0.5f : front > 0.5f);
        w.retryBehindSeconds = fellBack || stalled || turnedOver ? w.retryBehindSeconds + TicksPerDecision * TickSeconds : 0.0f;
        if (w.retryBehindSeconds < RetrySeconds - 1.0e-4f) return false;
        const std::uint32_t chosen = adoptClipForPose(w);
        const bool run = clipRunExercise(w.clipExercise);
        w.attemptPhase = clipTime(w);
        w.attemptPeakHead = now.headHeight;
        // (08h25: correndo, o pedido pelo rumo do clipe novo, como no comeco - com o rumo de antes, a corrida apontava para um lado que
        // ele nunca viu deitado: na sonda, tentava de novo 30-48% e levantava 0-1%.)
        if (run && chosen < clipGetUpDirection.size()) {
            const Vec3 root = w.body->links[rootLink].position;
            const float direction = w.clipYaw + clipGetUpDirection[chosen];
            w.target = { root.x + std::cos(direction) * ClipRunTargetMeters, root.y + std::sin(direction) * ClipRunTargetMeters, 0.0f };
            if (chosen < clipGetUpSpeed.size()) w.commandSpeed = clipGetUpSpeed[chosen];
        }
        w.attemptFrom = w.time;
        w.retryBehindSeconds = 0.0f;
        ++w.attempts;
        w.riseFirmSeconds = 0.0f;
        w.riseFirmFrom = -1.0f;
        return true;
    }

    // (14h30) O clipe do lado em que ele esta (do mesmo tipo: de pe ou correndo, sem a variante), no trecho mais parecido com a pose de
    // agora (ate o ponto de entrega dele), reancorado nele; o relogio vai para esse trecho. Devolve o exercicio do clipe.
    std::uint32_t adoptClipForPose(World& w) {
        const bool supine = TrackingClipGetUp3D::chestFront(*contract, *profile, w.body->links) > 0.0f;
        const bool run = clipRunExercise(w.clipExercise);
        std::uint32_t chosen = w.clipExercise;
        for (std::uint32_t e = 0; e < clipGetUpClips.size(); ++e) {
            const bool variant = e < clipGetUpVariants.size() && clipGetUpVariants[e] > 0.5f;
            if (!variant && clipRunExercise(e) == run && clipStartsSupine(clipGetUpClips[e]) == supine) {
                w.clipReference = clipGetUpClips[e];
                chosen = e;
                break;
            }
        }
        // O trecho do clipe mais parecido com a pose de agora (ate o ponto de entrega do clipe novo), reancorado nele.
        const float chosenDeliver = chosen < clipGetUpDeliver.size() && clipGetUpDeliver[chosen] > 0.0f
            ? clipGetUpDeliver[chosen] : references[w.clipReference]->durationSeconds();
        const TrackingClipGetUp3D::Widths widths { settings.clipGetUpSoftness, settings.clipGetUpComVelocityWidth, settings.clipGetUpParts };
        float bestPhase = 0.0f, bestScore = -std::numeric_limits<float>::infinity(), bestYaw = w.clipYaw;
        Vec3 bestGround = w.clipGround;
        for (float t = 0.0f; t <= std::max(0.0f, chosenDeliver - 0.2f) + 1.0e-4f; t += 0.1f) {
            clipReanchor(w, t);
            const float score = TrackingClipGetUp3D::score(*contract, *profile, w.body->links, w.frame, widths, w.clipGround.z).total;
            if (score > bestScore) {
                bestScore = score;
                bestPhase = t;
                bestYaw = w.clipYaw;
                bestGround = w.clipGround;
            }
        }
        w.clipYaw = bestYaw;
        w.clipGround = bestGround;
        w.clipHeld = w.clipStart + w.time - bestPhase;
        w.clipRate = 1.0f;
        w.clipFree = false;
        w.clipFarSeconds = w.clipFacingSeconds = 0.0f;
        w.clipPoseExercise = chosen;
        return chosen;
    }

    float clipGetUpState(World& w, const TrackingGetUpPosture3D& now) {
        const TrackingClipReference3D& reference = *references[w.clipReference];
        const float duration = reference.durationSeconds();
        if (w.clipFree) {
            // (20h20, TrackingClipGetUp3D::FreeRollProne) O rolamento livre: so o peito virando para baixo; de brucos pelo caminho
            // dele, o relogio vai ao fim do rolamento do clipe, o clipe e reancorado no corpo e o rastreio segue dali.
            w.clipFreeSeconds += TicksPerDecision * TickSeconds;
            const float front = TrackingClipGetUp3D::chestFront(*contract, *profile, w.body->links);
            if (front < TrackingClipGetUp3D::FreeRollProne) {
                w.clipHeld = w.clipStart + w.time - w.clipFreeUntil;
                w.clipFree = false;
                clipReanchor(w, w.clipFreeUntil);
                w.clipFarSeconds = w.clipFacingSeconds = 0.0f;
            } else {
                w.clipHeld = std::max(w.clipHeld, w.clipStart + w.time - w.clipFreeUntil);
                reference.frame(std::min(duration, clipTime(w)), w.clipGround, w.clipYaw, w.frame);
                // (a semelhanca dos graficos continua medida contra o clipe)
                w.clipTerms = TrackingClipGetUp3D::score(*contract, *profile, w.body->links, w.frame,
                    { settings.clipGetUpSoftness, settings.clipGetUpComVelocityWidth, settings.clipGetUpParts }, w.clipGround.z);
                w.clipFacingSeconds = 0.0f;
                w.clipFarSeconds = w.clipFreeSeconds >= TrackingClipGetUp3D::FreeRollSeconds ? TrackingClipGetUp3D::FarSeconds : 0.0f;
                return TrackingClipGetUp3D::FreeRollWeight * std::clamp(0.5f * (1.0f - front), 0.0f, 1.0f);
            }
        }
        float t = std::min(duration, clipTime(w));
        reference.frame(t, w.clipGround, w.clipYaw, w.frame);
        if (settings.adaptiveRetry && adaptiveFor(w) && w.clipDeliveredAt < 0.0f && maybeRetry(w, now)) {
            t = std::min(references[w.clipReference]->durationSeconds(), clipTime(w));
            references[w.clipReference]->frame(t, w.clipGround, w.clipYaw, w.frame);
        }
        const float emphasis = w.clipExercise < clipGetUpEmphasis.size() ? clipGetUpEmphasis[w.clipExercise] : 0.0f;
        const TrackingClipGetUp3D::Widths widths { settings.clipGetUpSoftness, settings.clipGetUpComVelocityWidth, settings.clipGetUpParts,
            emphasis };
        const float agility = w.clipExercise < clipGetUpAgility.size() ? clipGetUpAgility[w.clipExercise] : 0.0f;
        const auto paced = [&](TrackingClipFrame3D& frame) {
            for (PhysicsBodyState3D& link : frame.links) {
                link.linearVelocity = link.linearVelocity * w.clipRate;
                link.angularVelocity = link.angularVelocity * w.clipRate;
            }
        };
        if (agility > 0.0f) paced(w.frame);
        w.clipTerms = TrackingClipGetUp3D::score(*contract, *profile, w.body->links, w.frame, widths, w.clipGround.z);
        w.clipAgilityReward = 0.0f;
        // (12/11 04h30, ele: "os levantares correndo estao terminando muito cedo, antes dele engatar a postura ereta de corrida") A
        // agilidade so ate o ponto de entrega (setClipGetUpDeliver): depois dele o clipe segue no tempo dele - no correndo, a corrida ereta
        // ate a troca; o ritmo volta a 1.
        const float deliverPoint = w.clipExercise < clipGetUpDeliver.size() ? clipGetUpDeliver[w.clipExercise] : 0.0f;
        if (agility > 0.0f && deliverPoint > 0.0f && t >= deliverPoint) w.clipRate += 0.25f * (1.0f - w.clipRate);
        else if (agility > 0.0f) {
            // (TrackingClipGetUp3D::AgilityMaxRate) O relogio acompanha o corpo: o primeiro degrau a frente que nao melhora a nota para.
            const float decision = TicksPerDecision * TickSeconds;
            const float stride = (TrackingClipGetUp3D::AgilityMaxRate - 1.0f) * decision / TrackingClipGetUp3D::AgilitySteps;
            thread_local TrackingClipFrame3D probe;
            float best = t;
            for (int k = 1; k <= TrackingClipGetUp3D::AgilitySteps && best < duration; ++k) {
                const float ahead = std::min(duration, t + stride * static_cast<float>(k));
                reference.frame(ahead, w.clipGround, w.clipYaw, probe);
                paced(probe);
                const TrackingClipGetUp3D::Terms terms
                    = TrackingClipGetUp3D::score(*contract, *profile, w.body->links, probe, widths, w.clipGround.z);
                if (!(terms.total > w.clipTerms.total)) break;
                best = ahead;
                w.clipTerms = terms;
                w.frame = probe;
            }
            const float gained = best - t;
            w.clipHeld -= gained;
            w.clipRate += 0.25f * (1.0f + gained / decision - w.clipRate);
            w.clipAgilityReward = agility * w.clipTerms.total * gained / decision;
            t = best;
        }
        const bool variant = w.clipExercise < clipGetUpVariants.size() && clipGetUpVariants[w.clipExercise] > 0.5f;
        // (22h10, desligado: em 1h30 a semelhanca do de costas caiu num degrau logo depois - 0,57 -> 0,52-0,54, a regua de antes - e
        // as maos e a cabeca quase nao se moveram (0,6-1,0 s 18/12/17 -> 15/7/13 cm; 1,0-1,6 s 15/24/32 -> 18/23/30 cm): a nota cortada
        // de uma vez empurrou a rede para longe do que ela fazia, sem achar o apoio do clipe. Fica para um reforco pela pose.)
        constexpr bool ReinforceBackGetUp = false;
        if (ReinforceBackGetUp && !variant && !clipRunExercise(w.clipExercise) && clipStartsSupine(w.clipReference)) {
            // (20h40, ele: "precisamos reforcar muito a questao da similaridade com clipe do levantar de costas, para corrigir
            // movimento estranho do braco e coluna que ela aprendeu errado"; medido r79400: de 0 a 0,6 s a cabeca, o pescoco, o
            // peito alto e o braco esquerdo a ~40 cm do clipe, de 0,6 a 1,6 s a mao direita a 44-51 cm - o termo das maos ja era
            // ~0 com 30 ou 50 cm, sem diferenca entre melhorar e nao.) No de costas, a nota vezes exp(-(pior/ReinforceWidth)^2): o
            // pior entre as duas maos e a cabeca, cada uma relativa a pelve (so a translacao), contra o clipe - 50 cm 0,06, 30 cm
            // 0,37, 15 cm 0,78. A semelhanca dos graficos fica na regua de antes.
            constexpr float ReinforceWidth = 0.3f;
            const std::uint32_t head = contract->headLink();
            const auto offset = [&](std::span<const PhysicsBodyState3D> links, std::uint32_t i) {
                return links[i].position - links[rootLink].position;
            };
            float worst = 0.0f;
            for (const std::uint32_t i : { handLinks[0], handLinks[1], head })
                worst = std::max(worst, (offset(w.body->links, i) - offset(w.frame.links, i)).length());
            const float x = worst / ReinforceWidth;
            w.clipTerms.total *= std::exp(-x * x);
        }
        const int farKind = TrackingClipGetUp3D::far(*contract, *profile, w.body->links, w.frame,
            variant ? TrackingClipGetUp3D::FarFacingVariant : TrackingClipGetUp3D::FarFacing);
        const bool far = farKind == 1 || (t >= duration - 1.0e-3f && now.headHeight <= ClipHandoffHead);
        const float dt = TicksPerDecision * TickSeconds;
        w.clipFarSeconds = far ? w.clipFarSeconds + dt : 0.0f;
        // (18h22: o peito virado tem a tolerancia dele - TrackingClipGetUp3D::FarFacingSeconds -, contada a parte; vencida, vira o longe.)
        w.clipFacingSeconds = farKind == 2 ? w.clipFacingSeconds + dt : 0.0f;
        if (w.clipFacingSeconds >= (variant ? TrackingClipGetUp3D::FarFacingVariantSeconds : TrackingClipGetUp3D::FarFacingSeconds))
            w.clipFarSeconds = std::max(w.clipFarSeconds, TrackingClipGetUp3D::FarSeconds);
        if (now.headHeight > 1.2f) {
            bool foot = false;
            for (const std::uint32_t f : footLinks)
                foot = foot || std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == f; });
            if (!foot) w.clipFlightSeconds += TicksPerDecision * TickSeconds;
        }
        // (17h40: a semelhanca dos graficos sempre na regua de antes - a nota do corpo inteiro -, mesmo com a nota por partes no
        // treino: o antes e o depois comparaveis.)
        // (12/11 03h25, ele: "flexibilizar a rigorosidade de semelhanca com o clipe, para permitir melhor as melhorias de movimento":
        // a nota do treino ficou mais larga; a regua dos graficos fica a de 11/11 - TrackingClipGetUp3D::RulerSoftness.)
        w.imitationSum += TrackingClipGetUp3D::score(*contract, *profile, w.body->links, w.frame,
            { TrackingClipGetUp3D::RulerSoftness, TrackingClipGetUp3D::RulerComVelocityWidth, false }, w.clipGround.z).total;
        ++w.trackSamples;
        return w.clipTerms.total;
    }
    // (11/11 14h) A troca do levantar pelo clipe no fim dele, de pe: parado, o idle da locomocao (o parado em alerta mais perto da
    // pose, com os alvos indo dos da recuperacao aos dele, como depois de qualquer levantar); correndo, a corrida dela - o trote de
    // frente no fixo dele, o clipe do trote na fase das pernas de agora, no rumo em que o corpo vai; o pedido saindo da velocidade do
    // corpo, subindo ate o fixo (ver o comentario no codigo). Dai a locomocao (congelada) comanda por clipGetUpAfterSeconds; cair nesse
    // tempo encerra o episodio - a entrega ruim custa o que a locomocao deixaria de ganhar.
    // (12/11 18h, ele: "no treino COMANDO (jogador emulado) vamos desde ja fazer a conexao entre os dois brains, o da locomocao e do
    // levantar, para vermos na pratica como eles agiriam em conjunto... em qual momento ha as trocas... em perfeita harmonia... e
    // garantir o aprendizado de ambos") settings.commandGetUp: caiu no comando, a cabeca do levantar assume no clipe de pe do lado em que
    // o corpo esta, no trecho mais parecido com a pose (adoptClipForPose, o mesmo da nova tentativa), sem fogo, com o levantar adaptativo
    // e o nunca desistir; o jogador espera (a tecla solta). De pe e firme (a entrega e a janela de firmeza), a troca: a locomocao no idle
    // alerta, o comando parado, e a proxima tecla retoma. Sem levantar no orcamento do levantar, o episodio acaba.
    void beginCommandGetUp(World& w, const TrackingGetUpPosture3D& now) {
        World::Command& c = w.command;
        std::uniform_real_distribution<float> u(0.0f, 1.0f);
        ++c.falls;
        c.riseFrom = w.time;
        // (12/11 18h, ele: "no treino de comando, vamos treinar tambem o levantar correndo (quando ele ta caido mas quer correr, ai ele
        // levanta ja correndo)") Andando, o jogador segue segurando a tecla em settings.commandRunGetUpShare das quedas - o levantar
        // correndo; senao solta - o de pe.
        const bool wantsRun = c.state == 0 && c.key >= 0 && u(w.rng) < settings.commandRunGetUpShare;
        if (!wantsRun) c.key = -1;
        c.state = 2;
        // (O fogo da locomocao pausa enquanto ele levanta: o continuo e o agarrao soltam, nada novo.)
        w.locoFireHoldUntil = std::min(w.locoFireHoldUntil, w.time);
        w.fireShovesLeft = 0;
        w.fireShoveDone = true;
        w.fireGrabUntil = std::min(w.fireGrabUntil, w.time);
        w.fireNextTurnAt = -1.0f;
        w.freeStop = w.freeTroca = w.stopping = false;
        w.stopPhase = 0;
        w.runTurn = -1;
        w.brakeTurnAt = w.runAgainAt = -1.0f;
        w.commandSpeed = 0.0f;
        // (13/11 v1, a revisao: a largada pendente do exercicio de antes - pendingClip, altClip, unrealStart - disparava depois da entrega e
        // trocava a referencia pela corrida antiga; e o virado contava com o rumo congelado de antes da queda.)
        w.altClip = w.pendingClip = std::numeric_limits<std::size_t>::max();
        w.unrealStart = -1;
        w.turnedSeconds = 0.0f;
        w.headingError = 0.0f;
        w.headingBias = 0.0f;
        w.lateralBias = {};
        w.hasReferenceHeading = false;
        w.referenceTurningUntil = -1.0f;
        const Vec3 root = w.body->links[rootLink].position;
        w.target = { root.x, root.y, 0.0f };
        w.clipGetUp = true;
        // (O clipe de pe - de costas - ou o correndo - de costas -; adoptClipForPose troca pelo lado do corpo na mesma familia.)
        std::uint32_t family = 0u;
        if (wantsRun)
            for (std::uint32_t e = 0; e < clipGetUpClips.size(); ++e)
                if (clipRunExercise(e)) { family = e; break; }
        w.clipExercise = family;
        w.clipReference = clipGetUpClips[family];
        w.clipStart = 0.0f;
        w.clipHeld = w.time;
        w.clipYaw = trunkYawOf(w.body->links);
        w.clipGround = { 0.0f, 0.0f, terrainHeight(w, {}) };
        w.clipDeliveredAt = w.clipHandedAt = -1.0f;
        w.clipFree = false;
        w.clipFreeUntil = -1.0f;
        w.clipFarSeconds = w.clipFacingSeconds = 0.0f;
        w.clipRate = 1.0f;
        w.getUpCovered = w.getUpTarget = -1.0f;
        w.riseFirmSeconds = 0.0f;
        w.riseFirmFrom = w.upAt = -1.0f;
        w.handoffCalmSeconds = 0.0f;
        w.attempts = 1;
        w.attemptFrom = w.time;
        w.retryBehindSeconds = 0.0f;
        w.attemptPeakHead = now.headHeight;
        w.fireKind = 0;
        w.fireHoldScale = 0.0f;
        w.clipExercise = adoptClipForPose(w);
        w.attemptPhase = clipTime(w);
        // (Correndo: o pedido pelo rumo do clipe escolhido, como na nova tentativa.)
        if (clipRunExercise(w.clipExercise) && w.clipExercise < clipGetUpDirection.size()) {
            const float direction = w.clipYaw + clipGetUpDirection[w.clipExercise];
            w.target = { root.x + std::cos(direction) * ClipRunTargetMeters, root.y + std::sin(direction) * ClipRunTargetMeters, 0.0f };
            if (w.clipExercise < clipGetUpSpeed.size()) w.commandSpeed = clipGetUpSpeed[w.clipExercise];
        }
        w.trackClip = w.clipReference;
        w.trackStart = clipTime(w) - w.time;
        w.trackScale = 1.0f;
        w.trackYaw = w.clipYaw;
        w.rampFrom = -1.0f;
        w.twistClip = std::numeric_limits<std::size_t>::max();
        w.trackTwist = 0.0f;
        w.locomotionControl = false;
    }
    void clipHandoff(World& w) {
        if (!clipRunExercise(w.clipExercise)) w.holdAtHandoff = true;
        // (15/11, ele: "o corpo do clipe NAO PODE SE DESLOCAR EM RELACAO AO MUNDO") O parado de depois nasce na pelve do clipe do levantar
        // agora (preso no mundo), nao na do corpo: o clipe nao pula na troca.
        if (settings.idleClipAnchor && !clipRunExercise(w.clipExercise) && w.clipReference < references.size()) {
            thread_local TrackingClipFrame3D handFrame;
            const TrackingClipReference3D& reference = *references[w.clipReference];
            reference.frame(std::min(reference.durationSeconds(), clipTime(w)), w.clipGround, w.clipYaw, handFrame);
            if (rootLink < handFrame.links.size()) {
                w.handoffAnchor = handFrame.links[rootLink].position;
                w.handoffAnchorAt = w.time;
            }
        }
        const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        const float speed = std::hypot(v.x, v.y);
        w.clipHandedAt = w.time;
        w.clipHandSpeed = speed;
        if (isCommand(w)) {
            // (12/11 18h, settings.commandGetUp) De volta ao comando: a locomocao no idle alerta (os alvos indo dos da recuperacao aos do
            // idle), a vista no rumo do corpo, o jogador parado - a proxima tecla retoma.
            World::Command& c = w.command;
            if (settings.idleBlendSeconds > 0.0f && w.lastRecoveryAction.size() == contract->actionSize()) {
                w.idleBlendFrom = w.time;
                w.idleBlendPose = w.lastRecoveryAction;
            }
            const bool ran = clipRunExercise(w.clipExercise) && c.key >= 0;
            w.clipGetUp = false;
            w.clipHandedAt = w.clipDeliveredAt = -1.0f;
            ++c.rises;
            if (ran) {
                // (O levantar correndo: o jogador segue segurando - a frente, na vista do rumo em que ele saiu correndo -; o pedido parte
                // da velocidade do corpo, como na troca do correndo, e sobe na StartAcceleration.)
                const float yaw = speed > 1.5f ? std::atan2(v.y, v.x) : trunkYawOf(w.body->links);
                w.faceYaw = yaw;
                c.target = yaw;
                c.key = 0;
                if (commandStart(w, c.run ? 2u : 1u, 0)) {
                    w.rampFrom = -1.0f;
                    w.launchEnd = -1.0f;
                    w.accelFrom = w.time;
                    w.accelSpeed = speed;
                    w.accelRate = StartAcceleration;
                }
                c.state = 0;
                ++c.runRises;
                w.stopping = false;
                w.stopPhase = 0;
            } else {
                // (15/11, settings.getUpViewReward: a vista de antes da queda fica - o corpo e que vira para ela.)
                if (settings.getUpViewReward <= 0.0f) w.faceYaw = trunkYawOf(w.body->links);
                w.moveYaw = w.faceYaw;
                c.target = w.faceYaw;
                commandIdle(w);
                w.stopping = true;
            }
            if (c.riseFrom >= 0.0f) c.riseSum += w.time - c.riseFrom;
            c.riseFrom = -1.0f;
            c.nextKeyAt = w.time + 0.6f + 0.8f * std::uniform_real_distribution<float>(0.0f, 1.0f)(w.rng);
            w.locomotionControl = true;
            return;
        }
        if (!clipRunExercise(w.clipExercise) && !w.getUpGo) {
            startIdle(w);
            if (settings.idleBlendSeconds > 0.0f && w.lastRecoveryAction.size() == contract->actionSize()) {
                w.idleBlendFrom = w.time;
                w.idleBlendPose = w.lastRecoveryAction;
            }
            w.locomotionControl = true;
            return;
        }
        if (w.getUpGo && !clipRunExercise(w.clipExercise)) {
            // (12/11 19h10, o levantar e ir de pe: a largada do trote como a do comando - a vista para onde o corpo esta virado, a
            // referencia misturada a partir da pose de agora, o pedido subindo do zero -; o steerGo vira para o ponto.)
            w.locomotion = true;
            w.startBand = 1u;
            w.maneuver = 0;
            w.speedScale = 1.0f;
            w.altClip = w.pendingClip = std::numeric_limits<std::size_t>::max();
            w.unrealStart = -1;
            w.refActive = false;
            // (15/11, settings.getUpViewReward: a vista fica no objetivo - o corpo e que vira para ela.)
            if (settings.getUpViewReward <= 0.0f) w.faceYaw = trunkYawOf(w.body->links);
            if (!commandStart(w, 1u, 0)) {
                w.trackClip = std::numeric_limits<std::size_t>::max();
                return;
            }
            w.startCommandSpeed = w.commandSpeed;
            const Vec3 root = w.body->links[rootLink].position;
            w.target = { root.x + std::cos(w.faceYaw) * ClipRunTargetMeters, root.y + std::sin(w.faceYaw) * ClipRunTargetMeters, 0.0f };
            w.locomotionControl = true;
            return;
        }
        // (12/11 19h10, o levantar e ir correndo: a corrida no lugar do trote; o rumo sai para onde o corpo vai e o steerGo vira para o
        // ponto.)
        const std::uint32_t band = w.getUpGo ? 2u : 1u;
        const Gait* gait = gaitFor(band, 0u);
        if (gait == nullptr) {
            w.trackClip = std::numeric_limits<std::size_t>::max();
            return;
        }
        const float yaw = speed > 1.5f ? std::atan2(v.y, v.x) : trunkYawOf(w.body->links);
        const float clipDirection = travel[gait->clip].direction;
        w.locomotion = true;
        w.band = w.startBand = band;
        w.heading = 0u;
        w.maneuver = 0;
        w.runBand = band >= 2u;
        w.speedScale = 1.0f;
        const float fixed = fixedSpeed(band, 0u);
        w.commandSpeed = w.startCommandSpeed = fixed > 0.0f ? fixed : std::max(0.1f, travel[gait->clip].speed);
        w.trackClip = gait->clip;
        w.trackScale = fixed > 0.0f ? 1.0f : w.commandSpeed / std::max(0.1f, travel[gait->clip].speed);
        w.trackTwist = wrapAngle(clipDirection);
        w.twistClip = gait->clip;
        w.altClip = std::numeric_limits<std::size_t>::max();
        w.pendingClip = std::numeric_limits<std::size_t>::max();
        w.unrealStart = -1;
        w.refActive = false;
        w.trackYaw = yaw - clipDirection;
        w.faceYaw = w.trackYaw + w.trackTwist;
        w.moveYaw = yaw;
        resumeTracking(w);
        // (16h55, a vigia: com a rampa da largada o pedido comecava em 0 - o corpo a 2,5 m/s recebia "fique parado" e caia para a
        // frente; o correndo de costas entregava 3% das trocas. O pedido comeca na velocidade do corpo e sobe ate o fixo a
        // StartAcceleration, como na saida da arrancada (accelFrom); a cadencia do clipe inteira.)
        w.rampFrom = -1.0f;
        w.launchEnd = -1.0f;
        w.accelFrom = w.time;
        w.accelSpeed = speed;
        w.accelRate = StartAcceleration;
        const Vec3 root = w.body->links[rootLink].position;
        w.target = { root.x + std::cos(yaw) * ClipRunTargetMeters, root.y + std::sin(yaw) * ClipRunTargetMeters, 0.0f };
        w.locomotionControl = true;
    }
    // O idle depois de ficar de pe: o parado em alerta (o normal ou o espelho, o mais perto da pose de agora) no rumo do
    // corpo, seguido daqui em diante.
    void startIdle(World& w) {
        references[standingIndex]->frame(0.0f, {}, 0.0f, w.frame);
        const float normal = poseDistance(*w.body, w.frame);
        references[standingMirrorIndex]->frame(0.0f, {}, 0.0f, w.frame);
        const std::size_t idle = poseDistance(*w.body, w.frame) < normal ? standingMirrorIndex : standingIndex;
        w.trackClip = idle;
        w.trackScale = 1.0f;
        w.trackStart = -w.time;
        w.rampFrom = -1.0f;
        w.trackYaw = wrapAngle(bodyYaw(w) - yawOffsets[idle]);
        w.idleAfterGetUp = true;
    }

    // A queda programada (a da locomocao fisica) ate assentar, sem a rede: no treino inteira dentro de uma decisao, no
    // laboratorio uma decisao por vez.
    void beginProtectiveFall(World& w) {
        w.fall->begin(static_cast<std::uint32_t>(w.rng()));
        w.fallActive = true;
        w.fallFromStart = w.fallPending;
        w.fallPending = false;
    }
    void advanceProtectiveFall(World& w) {
        for (int k = 0; k < TicksPerDecision; ++k) {
            w.fall->step(*w.body, 0.0f, TickSeconds, w.drives);
            settle(w, 1);
        }
        w.time += TicksPerDecision * TickSeconds;
        if (!w.fall->settled()) return;
        w.fallActive = false;
        if (settings.getUpGuide) beginGetUpGuide(w);
        // (Nos comecos ja caindo a queda nao foi da rede: o toque do tronco no chao nao conta contra ela.)
        if (w.fallFromStart) w.touchedDown = true;
    }

    void decide(World& w, std::size_t index, std::span<const float> action, float& reward, std::uint8_t& terminated,
        std::uint8_t& truncated) {
        // (04/11 15h: os canais do passo vem depois dos eixos; daqui em diante `action` sao so os eixos.)
        const std::size_t axesCount = contract->actionSize(), stepCount = settings.stepChannels ? 4 : 0;
        const std::span<const float> channels = action.size() > axesCount
            ? action.subspan(axesCount, std::min(stepCount, action.size() - axesCount)) : std::span<const float> {};
        // (06/11 17h40, velocityActions: depois dos canais do passo, a velocidade-alvo de cada eixo, rad/s.)
        const std::span<const float> velocityPart = settings.velocityActions && action.size() >= 2 * axesCount + stepCount
            ? action.subspan(axesCount + stepCount, axesCount) : std::span<const float> {};
        action = action.first(std::min(action.size(), axesCount));
        w.stepPoseValid = false;
        const std::size_t a = features->size();
        features->compute(w.body->links, std::span<float>(w.ampPair).first(a));
        if (motionDb) motionQuery(w);
        const TrackingGetUpPosture3D was = posture->measure(w.body->links, floorUnder(w));
        // (08/11 23h55: a cabeca que age nesta decisao e a do comeco dela; na troca de cabeca no meio, o passo e da que agiu.)
        const bool shaped = settings.locomotionShaping || !w.locomotionControl;
        const float before = shaped ? posture->risePotential(was) + posture->turnPotential(was) + facePotential(w, was) : 0.0f;
        maybePush(w, was);
        maybeAvoidPush(w);
        maybeGetUpFire(w, was);
        maybeLocomotionFire(w, was);
        trackLocomotionKeep(w, was);
        if (w.pendingClip < references.size() && w.trackClip < references.size()
            && w.trackStart + (w.time + TicksPerDecision * TickSeconds) * w.trackScale
                >= (w.unrealStart >= 0 ? startInfo[static_cast<std::size_t>(w.unrealStart)].exit : references[w.trackClip]->durationSeconds())) {
            // (04/11 08h50: o ciclo entra na fase do pe direito - a arrancada acaba com ele apoiado; no trote a fase 0 e a do
            // esquerdo e os pes trocavam na emenda.)
            const std::size_t from = w.trackClip, to = w.pendingClip;
            const bool fixed = fixedSpeed(w.band, w.heading) > 0.0f;
            float scale = fixed ? 1.0f : w.commandSpeed / std::max(0.1f, travel[to].speed);
            float u = 0.0f;
            const bool unreal = w.unrealStart >= 0;
            const bool launch = from == startClipIndex || from == launchExitIndex || unreal;
            if ((from == startClipIndex || unreal) && settings.runLaunchEaseSeconds > 0.0f && travel[to].speed > 0.0f) {
                // (06/11, a largada progressiva: o ciclo comeca na cadencia em que o impulso acaba e sobe ate a dele.)
                const float begin = std::clamp(startSpeed(w, 0.0f) / travel[to].speed, StartRampFloor, scale);
                w.easeFrom = w.time;
                w.easeScale = begin;
                w.easeTarget = scale;
                w.easeClip = to;
                w.easeLast = begin;
                scale = begin;
            }
            if (unreal) {
                // (A arrancada da Unreal: o ciclo entra no quadro com as pernas mais parecidas com as dela agora.)
                TrackingClipFrame3D now, candidate;
                references[from]->frame(w.trackStart + w.time * w.trackScale, {}, 0.0f, now);
                const float cycle = references[to]->durationSeconds();
                float best = std::numeric_limits<float>::max();
                for (int k = 0; static_cast<float>(k) / 60.0f < cycle; ++k) {
                    references[to]->frame(static_cast<float>(k) / 60.0f, {}, 0.0f, candidate);
                    float d = 0.0f;
                    for (std::size_t i = 0; i < now.coordinates.size() && i < candidate.coordinates.size() && i < trackingParts.size(); ++i)
                        if (trackingParts[i] == PartLeftLeg || trackingParts[i] == PartRightLeg)
                            d += (candidate.coordinates[i] - now.coordinates[i]).lengthSquared();
                    if (d < best) {
                        best = d;
                        u = static_cast<float>(k) / 60.0f;
                    }
                }
            } else if (launch && travel[from].rightLand > 0.0f && travel[to].rightLand > 0.0f) {
                const float since = std::max(0.0f, w.trackStart + w.time * w.trackScale
                    - travel[from].rightLand * references[from]->durationSeconds()) / std::max(0.1f, w.trackScale);
                u = travel[to].rightLand * references[to]->durationSeconds() + since * scale;
            }
            if (fixed && launch) {
                // (A velocidade em que a arrancada acaba, e dali o pedido sobe ate o fixo - ver commandNow.)
                w.accelSpeed = std::min(w.commandSpeed, startSpeed(w, 0.0f));
                w.accelFrom = w.time;
                w.accelRate = from == startClipIndex || unreal ? settings.runLaunchAcceleration : 0.0f;
            }
            if (w.refActive) w.refVelocity = wantedVelocity(w, 0.0f);
            if (launch) {
                // (09h35: a emenda misturada - o ultimo quadro da arrancada, parado, sai em StartBlendSeconds; sem ela a
                // semelhanca do clipe puro no trote caia a 0,60 por 0,3 s na troca do inclinado da arrancada para o trote.)
                w.blendClip = from;
                w.blendScale = 0.0f;
                w.blendStart = unreal ? std::max(0.0f, w.trackStart + w.time * w.trackScale)
                                      : std::max(0.0f, references[from]->durationSeconds() - 1.0e-3f);
                w.blendYaw = w.trackYaw;
                w.blendTwist = twist(w);
                w.blendPitch = pitch(w);
                w.blendLegPitch = legPitch(w);
                w.blendFrom = w.time;
                w.blendSeconds = StartBlendSeconds;
            }
            if (unreal) {
                // (Depois da emenda: o rumo e a cintura do ciclo de volta.)
                w.trackYaw = w.startCycleYaw;
                w.trackTwist = w.startCycleTwist;
                w.twistClip = w.startCycleTwistClip;
                w.unrealStart = -1;
            }
            w.trackClip = to;
            w.trackScale = scale;
            w.trackStart = u - w.time * w.trackScale;
            w.pendingClip = std::numeric_limits<std::size_t>::max();
        }
        if (w.easeFrom >= 0.0f) {
            // (A subida da cadencia, decisao a decisao, sem salto na fase do clipe; cancelada se o clipe ou a escala mudaram.)
            if (w.trackClip != w.easeClip || std::abs(w.trackScale - w.easeLast) > 1.0e-6f || w.rampFrom >= 0.0f) {
                w.easeFrom = -1.0f;
            } else {
                const float p = std::clamp((w.time - w.easeFrom) / settings.runLaunchEaseSeconds, 0.0f, 1.0f);
                const float s = w.easeScale + (w.easeTarget - w.easeScale) * p;
                const float clipTime = w.trackStart + w.time * w.trackScale;
                w.trackScale = s;
                w.trackStart = clipTime - w.time * s;
                w.easeLast = s;
                if (p >= 1.0f) w.easeFrom = -1.0f;
            }
        }
        // (A queda programada: a queda de verdade de quem ja ficou de pe no episodio, ou o comeco ja caindo. 23h35: o
        // "committing" da locomocao fisica - a pelve alem de ~50 graus e tombando - disparava na arrancada inclinada do
        // levantar em movimento; agora a cabeca descendo rapido abaixo de 1,1 m, ou a pelve deitada no ar - alem de 70
        // graus ainda alta.)
        const float tilt = ProtectiveFall3D::pelvisTilt(*profile, *w.body);
        const bool falling = (was.headHeight < 1.1f && w.body->links[contract->headLink()].linearVelocity.z < -1.2f)
            || (tilt > 1.22f && was.pelvisHeight > 0.5f * profile->standingRootHeightMeters);
        if (!w.fallActive && settings.protectiveFall && !w.locomotion && (w.fallPending || (w.stoodAt >= 0.0f && falling)))
            beginProtectiveFall(w);
        const bool programmed = w.fallActive;
        // (O rolar pelo clipe: deitado de costas e baixo, com a rede no comando, o trecho do rolamento do levantar de
        // costas; acaba no fim do trecho - dai a rede livre -, e outro so 1 s depois.)
        if (settings.rollBlend < 1.0f && !w.locomotion && !programmed && w.rollClip >= references.size()
            && w.time >= w.rollAgain && !rollIndices.empty()) {
            const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
            const Vec3 belly = (pelvis.orientation * profile->links[rootLink].modelOrientation.conjugate()).normalized()
                .rotate(rollBellyAxis);
            if (belly.z > RollBellyUp && pelvis.position.z - floorUnder(w) < RollPelvisHeight) beginRoll(w);
        }
        if (w.rollClip < references.size() && rollTime(w, 0.0f) >= RollEndSeconds) {
            w.rollClip = std::numeric_limits<std::size_t>::max();
            w.rollAgain = w.time + 1.0f;
        }
        updateSteps(w, channels);
        w.threatKnown = false;
        if (settings.idleThreatReach > 0.0f && idleHeld(w)) {
            const float margin = w.stepsOn ? w.balance.margin
                : measureTrackingBalance3D(*profile, w.body->links, w.body->interactions, { footLinks[0], footLinks[1] },
                      floorUnder(w)).margin;
            w.threat = std::clamp(1.0f - margin / 0.08f, 0.0f, 3.0f);
            w.threatKnown = true;
        }
        if (w.rollClip < references.size() && !programmed) {
            references[w.rollClip]->frame(rollTime(w, TicksPerDecision * TickSeconds), {}, 0.0f, w.frame);
            w.blendAction.assign(action.size(), 0.0f);
            contract->actionOf(w.frame.coordinates, w.blendAction);
            const float s = std::clamp(settings.rollBlend, 0.0f, 1.0f);
            for (std::size_t k = 0; k < action.size(); ++k)
                w.blendAction[k] = (1.0f - s) * w.blendAction[k] + s * (std::isfinite(action[k]) ? action[k] : w.blendAction[k]);
            contract->driveTargets(w.blendAction, w.drives);
        } else if (settings.locomotionResidual && settings.separateSkills && w.locomotionControl && clipTracked(w)) {
            composeResidual(w, action, w.blendAction, w.clipVelocity);
            contract->driveTargets(w.blendAction, w.clipVelocity, w.drives);
            if (settings.riseStiffness > 0.0f && guidedRise(w))
                for (RagdollDriveTarget3D& drive : w.drives) drive.stiffnessScale = settings.riseStiffness;
        } else if (settings.clipBlend < 1.0f && clipTracked(w) && (!settings.separateSkills || w.locomotionControl)) {
            // (Com as duas cabecas, o clipe nos motores e so da locomocao: a recuperacao age como foi treinada, sem ele -
            // 03/11, com a mistura no parado em alerta a cabeca da recuperacao caia parada, 12% sem cair.)
            const TrackingClipReference3D& reference = *references[w.trackClip];
            reference.frame(trackTime(w, TicksPerDecision * TickSeconds), {}, 0.0f, w.frame, twist(w));
            w.blendAction.assign(action.size(), 0.0f);
            contract->actionOf(w.frame.coordinates, w.blendAction);
            const float s = std::clamp(settings.clipBlend, 0.0f, 1.0f);
            for (std::size_t k = 0; k < action.size(); ++k)
                w.blendAction[k] = (1.0f - s) * w.blendAction[k] + s * (std::isfinite(action[k]) ? action[k] : w.blendAction[k]);
            contract->driveTargets(w.blendAction, w.drives);
        } else if ((settings.freeVelocityFeedforward || settings.velocityActions) && settings.separateSkills
                   && w.locomotionControl) {
            // (06/11, a locomocao livre: a velocidade-alvo pelo proprio alvo da rede - ver freeVelocityFeedforward.)
            const float dt = TicksPerDecision * TickSeconds;
            w.clipVelocity.assign(action.size(), 0.0f);
            if (settings.freeVelocityFeedforward && w.freeTarget.size() == action.size())
                for (std::size_t k = 0; k < action.size(); ++k) {
                    const float d = std::isfinite(action[k]) ? action[k] - w.freeTarget[k] : 0.0f;
                    w.clipVelocity[k] = std::abs(d) < 1.0f ? d / dt : 0.0f;
                }
            // (10/11, settings.spineVelocityFeedforward: nos eixos acoplados da coluna, a velocidade do proprio alvo - antes de ele mudar.)
            std::array<float, 8> spineFeed {};
            const bool spineFeedOn = settings.spineVelocityFeedforward && w.freeTarget.size() == action.size();
            if (spineFeedOn)
                for (std::size_t c = 0; c < coupledAxes.size() && c < spineFeed.size(); ++c) {
                    const std::size_t k = coupledAxes[c];
                    const float d = std::isfinite(action[k]) ? action[k] - w.freeTarget[k] : 0.0f;
                    spineFeed[c] = std::abs(d) < 1.0f ? d / dt : 0.0f;
                }
            w.freeTarget.assign(action.begin(), action.end());
            // (17h40: a rede livre da a velocidade-alvo de cada eixo - liberdade total nos motores, sem o clipe.)
            if (velocityPart.size() == action.size())
                for (std::size_t k = 0; k < action.size(); ++k)
                    w.clipVelocity[k] = std::isfinite(velocityPart[k]) ? std::clamp(velocityPart[k], -VelocityActionLimit, VelocityActionLimit) : 0.0f;
            if (settings.spineVelocityFeedforward)
                for (std::size_t c = 0; c < coupledAxes.size() && c < spineFeed.size(); ++c)
                    if (coupledAxes[c] < w.clipVelocity.size()) w.clipVelocity[coupledAxes[c]] = spineFeedOn ? spineFeed[c] : 0.0f;
            // (10/11 07h10, settings.launchClipShare; ele: "nao e melhor tu so tentar literalmente executar o movimento do clipe e
            // reforcar isso em largada? pelo menos ate ele aprender"): na janela da largada, o alvo e a velocidade-alvo de cada junta
            // misturados com os do clipe de largada - launchClipShare x a janela do clipe, o resto da rede; o treino desce a parcela.
            // (07h40: so com um clipe de largada de verdade tocando (starting) - nos rumos da rampa (laterais, costas, trote) o clipe e o
            // proprio ciclo e a ajuda piorava: a lateral 1,35 -> 0,87 m/s a 1 s.)
            const float ql = settings.launchClipShare > 0.0f && w.locomotion && starting(w) ? settings.launchClipShare * launchWeight(w) : 0.0f;
            if (ql > 0.0f) {
                clipTargets(w, 0.0f, w.launchClipNow);
                clipTargets(w, dt, w.launchClipNext);
                w.blendAction.assign(action.size(), 0.0f);
                for (std::size_t k = 0; k < action.size() && k < w.launchClipNext.size(); ++k) {
                    const float own = std::isfinite(action[k]) ? action[k] : w.launchClipNext[k];
                    const float d = w.launchClipNext[k] - w.launchClipNow[k];
                    // (14/11 22h40, o Codex: este ramo substituia a regra dos bracos - clipe + correcao x alcance - pela mistura; a rede
                    // aprendia os bracos com dois sentidos. Bracos e cabeca seguem a regra de sempre; a mistura so nas pernas e no tronco.)
                    const bool arm = k < armAxes.size() && armAxes[k] && settings.armClipReach >= 0.0f;
                    const bool headAxis = k < headAxes.size() && headAxes[k] && settings.headClipReach >= 0.0f;
                    if (arm || headAxis) {
                        const float reach = arm ? (settings.stopArmsReach >= 0.0f && w.stopping ? settings.stopArmsReach : settings.armClipReach)
                                                : settings.headClipReach;
                        const float residual = std::isfinite(action[k]) && k < actionMid.size() ? action[k] - actionMid[k] : 0.0f;
                        w.blendAction[k] = w.launchClipNext[k] + residual * reach;
                        w.clipVelocity[k] = std::abs(d) < 1.0f ? d / dt : 0.0f;
                        continue;
                    }
                    w.blendAction[k] = ql * w.launchClipNext[k] + (1.0f - ql) * own;
                    w.clipVelocity[k] = ql * (std::abs(d) < 1.0f ? d / dt : 0.0f) + (1.0f - ql) * w.clipVelocity[k];
                }
                // (17/11, settings.clipVelocityRotation) Os bracos e a cabeca da largada pelo clipe dela, por junta.
                if (settings.clipVelocityRotation)
                    jointVelocities(w.launchClipNow, w.launchClipNext, dt, settings.armClipReach >= 0.0f, settings.headClipReach >= 0.0f, 1.0f,
                        w.clipVelocity);
                contract->driveTargets(w.blendAction, w.clipVelocity, w.drives);
            } else if ((settings.armClipReach >= 0.0f || settings.headClipReach >= 0.0f) && clipTracked(w)) {
                // (08/11, settings.armClipReach: os bracos no clipe, na fase do passo, mais a correcao curta da rede; 10h40,
                // headClipReach: o pescoco e a cabeca tambem - a inspecao das 10h28: a cabeca girava a 163-267 graus/s, p95,
                // contra 38-55 do clipe.)
                const float dt = TicksPerDecision * TickSeconds;
                clipTargets(w, 0.0f, w.armClipNow);
                clipTargets(w, dt, w.armClipNext);
                w.blendAction.assign(action.begin(), action.end());
                // (10/11 08h20, settings.launchCalmArms; ele: "tirar os bracos estabanados na largada"): na janela da largada os bracos
                // vao da pose do idle a do clipe com a velocidade do corpo no rumo (cheios a partir de LaunchArmSpeed) - medido, o balanco
                // do clipe da largada corria no relogio dele enquanto as pernas davam os passos falsos.
                constexpr float LaunchArmSpeed = 1.5f;
                float calm = 1.0f;
                if (settings.launchCalmArms && w.locomotion && idleArmTargets.size() == action.size()) {
                    const float lw = launchWeight(w);
                    if (lw > 0.0f) {
                        const float along = w.smoothVelocity.x * std::cos(w.moveYaw) + w.smoothVelocity.y * std::sin(w.moveYaw);
                        const float x = std::clamp(along / LaunchArmSpeed, 0.0f, 1.0f);
                        calm = 1.0f - lw * (1.0f - x * x * (3.0f - 2.0f * x));
                    }
                }
                for (std::size_t k = 0; k < action.size() && k < armAxes.size() && k < w.armClipNext.size(); ++k) {
                    // (13/11 v1, settings.stopArmsReach: freando, a correcao da rede nos bracos cai para stopArmsReach - medido, com a
                    // correcao inteira ela jogava a mao ~50 cm acima do clipe no fim da freada em 36% das paradas; sem ela, 14% e 0 queda.)
                    const float armReach = settings.stopArmsReach >= 0.0f && w.stopping ? settings.stopArmsReach : settings.armClipReach;
                    const float reach = armAxes[k] && settings.armClipReach >= 0.0f ? armReach
                        : k < headAxes.size() && headAxes[k] && settings.headClipReach >= 0.0f ? settings.headClipReach : -1.0f;
                    if (reach < 0.0f) continue;
                    const float own = std::isfinite(action[k]) && k < actionMid.size() ? action[k] - actionMid[k] : 0.0f;
                    const float d = w.armClipNext[k] - w.armClipNow[k];
                    if (armAxes[k] && calm < 1.0f) {
                        w.blendAction[k] = idleArmTargets[k] + (w.armClipNext[k] - idleArmTargets[k]) * calm + own * reach;
                        w.clipVelocity[k] = calm * (std::abs(d) < 1.0f ? d / dt : 0.0f);
                        continue;
                    }
                    w.blendAction[k] = w.armClipNext[k] + own * reach;
                    w.clipVelocity[k] = std::abs(d) < 1.0f ? d / dt : 0.0f;
                }
                // (17/11, settings.clipVelocityRotation - ver o .hpp) A velocidade do clipe por junta nos bracos e na cabeca. No calmo da
                // largada os bracos vao do idle ao clipe: os alvos de agora e os proximos sao os misturados.
                if (settings.clipVelocityRotation) {
                    if (calm < 1.0f && idleArmTargets.size() == action.size()) {
                        w.rotationNow.assign(w.armClipNow.begin(), w.armClipNow.end());
                        w.rotationNext.assign(w.armClipNext.begin(), w.armClipNext.end());
                        for (std::size_t k = 0; k < armAxes.size() && k < w.rotationNow.size() && k < w.rotationNext.size(); ++k)
                            if (armAxes[k]) {
                                w.rotationNow[k] = idleArmTargets[k] + (w.armClipNow[k] - idleArmTargets[k]) * calm;
                                w.rotationNext[k] = idleArmTargets[k] + (w.armClipNext[k] - idleArmTargets[k]) * calm;
                            }
                        jointVelocities(w.rotationNow, w.rotationNext, dt, settings.armClipReach >= 0.0f, settings.headClipReach >= 0.0f, 1.0f,
                            w.clipVelocity);
                    } else {
                        jointVelocities(w.armClipNow, w.armClipNext, dt, settings.armClipReach >= 0.0f, settings.headClipReach >= 0.0f, 1.0f,
                            w.clipVelocity);
                    }
                }
                contract->driveTargets(w.blendAction, w.clipVelocity, w.drives);
            } else {
                contract->driveTargets(action, w.clipVelocity, w.drives);
            }
            if (settings.freeDampingScale > 0.0f)
                for (RagdollDriveTarget3D& drive : w.drives) drive.dampingScale = settings.freeDampingScale;
        } else {
            w.freeTarget.clear();
            // (06/11 17h55, ele: "garanta isso no treino de levantar tambem, essa autonomia de velocidade dos motores": com
            // recoveryVelocity, a cabeca da recuperacao tambem da a velocidade-alvo de cada eixo.)
            if (settings.recoveryVelocity && velocityPart.size() == action.size() && !w.locomotionControl) {
                w.clipVelocity.assign(action.size(), 0.0f);
                for (std::size_t k = 0; k < action.size(); ++k)
                    w.clipVelocity[k] = std::isfinite(velocityPart[k]) ? std::clamp(velocityPart[k], -VelocityActionLimit, VelocityActionLimit) : 0.0f;
                contract->driveTargets(action, w.clipVelocity, w.drives);
            } else {
                contract->driveTargets(action, w.drives);
            }
            // (06/11 17h25: o amortecimento dos motores da cabeca da locomocao sem clipe - sem a velocidade-alvo dele, o
            // amortecimento freia toda junta em movimento.)
            if (settings.freeDampingScale > 0.0f && settings.separateSkills && w.locomotionControl)
                for (RagdollDriveTarget3D& drive : w.drives) drive.dampingScale = settings.freeDampingScale;
            // (04/11 11h25, ele: "nos exercicios de levantar-se e de evitar queda, recompensar a SUAVIDADE dos movimentos... e o
            // evitamento de mover juntas aos limites - joelhos dobrados no limite, bracos esticados no limite": a aspereza e a
            // mudanca media dos alvos (na fracao da metade da faixa de cada eixo) de uma decisao para a outra, ao quadrado; o
            // limite, quanto cada alvo passa de 85% da metade da faixa, ate a borda.)
            if (actionHalf.size() == action.size()) {
                double rough = 0.0, limits = 0.0;
                const bool previous = w.lastRecoveryAction.size() == action.size();
                for (std::size_t k = 0; k < action.size(); ++k) {
                    if (!std::isfinite(action[k])) continue;
                    const float n = (action[k] - actionMid[k]) / actionHalf[k];
                    limits += std::clamp((std::abs(n) - 0.85f) / 0.15f, 0.0f, 1.0f);
                    if (previous) {
                        const float d = (action[k] - w.lastRecoveryAction[k]) / actionHalf[k];
                        rough += static_cast<double>(d) * d;
                    }
                }
                w.recoveryRough = static_cast<float>(rough / static_cast<double>(action.size()));
                w.recoveryLimits = static_cast<float>(limits / static_cast<double>(action.size()));
            }
            w.lastRecoveryAction.assign(action.begin(), action.end());
        }
        applyStudentActuator(w);
        w.labelValid = w.labelDrove = w.labelTakeover = false;
        // (Sem a sombra e sem retomada, entregue o volante a professora para de rodar.)
        if (w.teacherSession && !settings.teacherShadow && settings.teacherTakeoverSeconds <= 0.0f && !w.teacherEpisode
            && w.time >= w.teacherUntil)
            w.teacherSession.reset();
        // (Na queda programada a professora nao ajuda mais: a sessao acaba.)
        if (programmed) w.teacherSession.reset();
        if (programmed) {
            if (settings.protectiveFallStepwise) advanceProtectiveFall(w);
            else
                while (w.fallActive && w.time < settings.episodeSeconds - 1e-4f) advanceProtectiveFall(w);
        } else if (w.teacherSession) {
            // (O giro parado: o pedido dela com a vista de agora, que gira.)
            if (w.startKind == 7) teacher->command(*w.teacherSession, {}, w.faceYaw);
            settleTaught(w, w.teacherEpisode || w.time < w.teacherUntil);
            w.time += TicksPerDecision * TickSeconds;
        } else {
            settle(w, TicksPerDecision);
            w.time += TicksPerDecision * TickSeconds;
        }
        if (w.locomotion) {
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            w.travelSum = w.travelSum + Vec3 { v.x, v.y, 0.0f };
        }
        // A intervencao (a aluna no volante com a professora em sombra): no comeco do desvio ela retoma por
        // teacherTakeoverSeconds.
        if (w.teacherSession && !w.teacherEpisode && settings.teacherTakeoverSeconds > 0.0f) {
            const float tilt = GetUpPolicy3D::trunkTilt(*profile, w.body->links);
            const float rate = w.lastTilt >= 0.0f ? (tilt - w.lastTilt) / (TicksPerDecision * TickSeconds) : 0.0f;
            w.lastTilt = tilt;
            // (O rumo pedido do corpo: nos ensinados, a vista - tambem no parado, que nao e locomocao.)
            const float chest = std::abs(wrapAngle((w.taught ? w.faceYaw : faceDirection(w)) - chestYaw(w)));
            w.chestError += (chest - w.chestError) * 0.13f;
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            const float speed = std::hypot(v.x, v.y);
            // (O rumo so onde o pedido anda: no parado e no giro, deslizar ja aparece no peito e no tronco.)
            const float travelError = speed > 0.5f && w.commandSpeed > 0.0f ? std::abs(wrapAngle(std::atan2(v.y, v.x) - w.moveYaw)) : 0.0f;
            constexpr float Degrees = 180.0f / Pi;
            if (w.time >= w.teacherUntil
                && (tilt > settings.teacherTakeoverTilt || rate > settings.teacherTakeoverTiltRate
                    || w.chestError * Degrees > (w.startKind == 7 ? settings.teacherTakeoverTurnChest : settings.teacherTakeoverChest)
                    || travelError * Degrees > settings.teacherTakeoverTravel)) {
                w.teacherUntil = w.time + settings.teacherTakeoverSeconds;
                w.labelTakeover = true;
                ++w.takeovers;
            }
        }
        if (kinematicMotion && motionDb && (w.locomotion || w.startKind == 2)) kinematicStep(w);
        else if (clipSequence && (w.locomotion || w.startKind == 2 || w.lyingClip < references.size())) sequenceStep(w);
        features->compute(w.body->links, std::span<float>(w.ampPair).subspan(a, a));
        const TrackingGetUpPosture3D now = posture->measure(w.body->links, floorUnder(w));
        if (w.getUpGuided && !w.sprintGuided && w.commandSpeed > 0.0f && !w.locomotion
            && sprintStartIndex < references.size()
            && w.trackClip < crouchTimes.size() && trackTime(w, 0.0f) >= crouchTimes[w.trackClip])
            beginSprintGuide(w);
        if (w.getUpGuided && trackTime(w, 0.0f) >= references[w.trackClip]->durationSeconds() - 1.0e-3f)
            endGetUpGuide(w, now.standing);
        // O alvo alcancado: dai o pedido e ficar parado (a nota do parado, com a pose).
        if (!w.locomotion && w.commandSpeed > 0.0f && targetBearing(w).second < ArriveRadius) {
            w.commandSpeed = 0.0f;
            w.reachedAt = w.time;
        }
        advanceTurn(w);
        advanceManeuver(w);
        steerGo(w);
        advanceStop(w);
        // (Parado com rumo pedido: a nota do parado 0,7 e o rumo do corpo 0,3, de pe. A coluna so quieto: sem empurrao e
        // 1 s depois do ultimo.)
        const bool down = fallen(now, *w.body);
        // (04/11 04h50, ele: "nos atos de levantar-se, demora muito para se estabilizar apos ja estar em pe, passa muito tempo
        // arqueado - forcar o pos-levantar a ficar na pose de idle parado alerta": sem o levantar guiado nenhum idle era
        // seguido depois de levantar. Agora, de pe sem pedido e sem clipe, o IDLE ALERTA vira a referencia na hora.)
        if (settings.idleAfterStand && !w.locomotion && !w.taught && w.commandSpeed <= 0.0f && now.standing && !down
            && w.trackClip >= references.size() && !w.fallActive && w.rollClip >= references.size())
            startIdle(w);
        const bool movingRun = settings.movingHandoff && w.startKind == 3 && w.commandSpeed > 0.0f;
        // (11/11 14h: no levantar pelo clipe a recuperacao comanda ate o fim do clipe; ali, de pe, a troca - parado, o idle da
        // locomocao; correndo, a corrida dela.)
        const bool clipActive = w.clipGetUp && w.clipHandedAt < 0.0f;
        // (15h30, a calibracao: o levantar correndo termina inclinado, saindo para correr, com a cabeca a 1,40-1,45 m - o "de pe" geral
        // (cabeca 1,4, pelve 0,75, tronco a 37 graus) quase nunca valia e o fim do clipe correndo acabava como longe. A troca e pelo
        // proprio clipe, que termina de pe: no fim dele, junto dele (sem o longe) e com a cabeca acima de ClipHandoffHead.)
        // (12/11 00h35, a medida das 23h20: desde que o correndo acaba na troca, a pose de corrida do fim - a da imagem dele - se perdeu;
        // de 1,4 a 1,75 s os erros de 9-16 cm foram a 24-47 cm e nem o bonus da chegada nem 0,5 s de locomocao a traziam de volta)
        // A entrega no ponto (setClipGetUpDeliver: o corte dele, 1,70 e 1,57 s): de pe e junto do clipe ali, entregou - a estatistica
        // conta a troca ai e nada depois como queda -, e o clipe segue ate o fim (1,90 e 1,80 s), o trecho da pose de novo no meio.
        const float deliverAt = w.clipExercise < clipGetUpDeliver.size() ? clipGetUpDeliver[w.clipExercise] : 0.0f;
        bool deliveredNow = false;
        const bool adaptive = clipActive && adaptiveFor(w);
        if (!adaptive && clipActive && deliverAt > 0.0f && w.clipDeliveredAt < 0.0f && !w.clipFree && clipTime(w) >= deliverAt - 1.0e-3f
            && w.clipFarSeconds <= 0.0f && now.headHeight > ClipHandoffHead && !down) {
            w.clipDeliveredAt = w.time;
            deliveredNow = true;
        }
        if (!adaptive && clipActive && !w.clipFree && clipTime(w) >= references[w.clipReference]->durationSeconds() - 1.0e-3f
            && w.clipFarSeconds <= 0.0f && now.headHeight > ClipHandoffHead && !down)
            clipHandoff(w);
        if (adaptive) {
            // (12/11 04h40, settings.clipGetUpAdaptive) A entrega quando firme: de pe (o de pe da postura, o ponto de captura na base, o
            // centro de massa abaixo de 0,6 m/s) por AdaptiveFirmStand, ou correndo (a cabeca acima de 1,3 m, o tronco a menos de ~45
            // graus e ao rumo pedido a 60% da velocidade dele) por AdaptiveFirmRun; levantou quando o firme comecou. A troca na hora.
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            bool firm = false;
            if (clipRunExercise(w.clipExercise)) {
                const float toward = std::atan2(w.target.y - w.body->links[rootLink].position.y, w.target.x - w.body->links[rootLink].position.x);
                const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
                firm = now.headHeight > 1.3f && now.upright > 0.7f && (settings.runHandoffAtStand || along >= 0.6f * w.commandSpeed) && !down;
            } else {
                // (06h20, ele: "a partir do momento em que ele ficou de pe posturado, ele se levantou com sucesso") De pe na postura
                // (cabeca, pelve e tronco em pe), sem exigir o equilibrio parado - o empurrao continuo nao deixava o "firme" valer.
                firm = now.standing && !down;
            }
            const float dt = TicksPerDecision * TickSeconds;
            if (firm) {
                if (w.riseFirmFrom < 0.0f) w.riseFirmFrom = w.time;
                if (settings.fireReleaseAtFirm && w.fireReleaseFrom < 0.0f) w.fireReleaseFrom = w.time;
                w.riseFirmSeconds += dt;
            } else {
                w.riseFirmSeconds = 0.0f;
                w.riseFirmFrom = -1.0f;
            }
            constexpr float AdaptiveFirmStand = 0.2f, AdaptiveFirmRun = 0.2f;
            if (w.clipDeliveredAt < 0.0f
                && w.riseFirmSeconds >= (clipRunExercise(w.clipExercise) ? AdaptiveFirmRun : handoffAtStand(w) ? StandHandoffFirm : AdaptiveFirmStand)
                        - 1.0e-4f) {
                w.clipDeliveredAt = w.time;
                w.upAt = w.riseFirmFrom;
                deliveredNow = true;
                // (settings.adaptiveSpeedBonus) O premio da velocidade: o tempo do clipe dali ate o ponto de entrega contra o dele.
                // (09h45) Na tentativa nova, o tempo dela contra o trecho do clipe que faltava dali.
                // (13/11 18h20, settings.getUpPaceWeight) No limpo com o ritmo ligado, o tempo inteiro desde o comeco do levantar: o de costas
                // rolava de brucos em ~0,6 s, o "virou" abria uma tentativa nova e a faixa media so os ~0,6 s do trecho dali - a faixa de 1,5 s
                // nunca cobrava o levantar de ~1,05 s. Sob fogo as tentativas novas sao de verdade e seguem contando da tentativa.
                const bool whole = settings.getUpPaceWeight > 0.0f && w.fireKind == 0 && !isCommand(w);
                const float phase0 = whole ? w.clipStart : w.attempts > 1 ? w.attemptPhase : w.clipStart;
                const float clipSeconds = std::max(0.3f, deliverAt - phase0);
                const float taken = w.upAt - (whole ? 0.0f : w.attemptFrom);
                if (settings.adaptiveSpeedBonus > 0.0f && deliverAt - phase0 >= 0.5f) {
                    if (settings.adaptiveTimeMax > 0.0f) {
                        // (13h45, settings.adaptiveTimeMin/Max) A faixa do tempo, encolhida pelo trecho do clipe que falta.
                        const float share = std::clamp(clipSeconds / std::max(0.3f, deliverAt), 0.2f, 1.0f);
                        // (13/11, settings.adaptiveTimeMinRun/MaxRun) Os correndo com a faixa deles.
                        const bool runBand = clipRunExercise(w.clipExercise) && settings.adaptiveTimeMaxRun > 0.0f;
                        // (13/11 10h55, settings.adaptiveTimeFireSlack) Com fogo, so o teto (o maximo + a folga), sem piso.
                        const bool fireCeiling = settings.adaptiveTimeFireSlack >= 0.0f && w.fireKind != 0;
                        // (15/11, settings.getUpProneTimeScale: a faixa do de brucos de pe na escala dele.)
                        const float scale = runBand ? 1.0f : getUpTimeScale(w);
                        const float low = fireCeiling ? 0.0f : (runBand ? settings.adaptiveTimeMinRun : settings.adaptiveTimeMin) * share * scale,
                                    high = ((runBand ? settings.adaptiveTimeMaxRun : settings.adaptiveTimeMax) * scale
                                            + (fireCeiling ? settings.adaptiveTimeFireSlack : 0.0f)) * share,
                                    fall = settings.adaptiveTimeFall * share;
                        const float off = taken < low ? low - taken : taken > high ? taken - high : 0.0f;
                        // (15/11, settings.getUpStandBonusScale: nos de pe, o premio da chegada abaixo do ganho da semelhanca.)
                        w.arrivalReward += settings.adaptiveSpeedBonus * (runBand ? 1.0f : settings.getUpStandBonusScale)
                            * std::clamp(1.0f - off / fall, 0.0f, 1.0f);
                    } else {
                        w.arrivalReward += settings.adaptiveSpeedBonus * std::clamp((clipSeconds - taken) / clipSeconds, 0.0f, 1.0f);
                    }
                }
                if (settings.runImpulseBonus > 0.0f && clipRunExercise(w.clipExercise) && w.commandSpeed > 0.1f) {
                    // (13h45, settings.runImpulseBonus) O impulso para frente na entrega.
                    const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
                    const float toward = std::atan2(w.target.y - w.body->links[rootLink].position.y, w.target.x - w.body->links[rootLink].position.x);
                    const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
                    w.arrivalReward += settings.runImpulseBonus
                        * std::clamp((along - 0.6f * w.commandSpeed) / (0.4f * w.commandSpeed), 0.0f, 1.0f);
                }
                if (settleSecondsFor(w) <= 0.0f) clipHandoff(w);
            }
            // (settings.adaptiveSettleSeconds) A troca depois da firmeza. (15h05, settings.handoffCalmSpeed) So quando calmo, ou no maximo.
            if (w.clipDeliveredAt >= 0.0f && w.clipHandedAt < 0.0f && settleSecondsFor(w) > 0.0f
                && w.time >= w.clipDeliveredAt + settleSecondsFor(w) - 1.0e-4f) {
                bool handNow = true;
                if (settings.handoffCalmSpeed > 0.0f) {
                    const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
                    bool calm = false;
                    if (clipRunExercise(w.clipExercise) && w.commandSpeed > 0.1f) {
                        const float toward = std::atan2(w.target.y - w.body->links[rootLink].position.y,
                            w.target.x - w.body->links[rootLink].position.x);
                        const float along = v.x * std::cos(toward) + v.y * std::sin(toward);
                        const float across = -v.x * std::sin(toward) + v.y * std::cos(toward);
                        calm = now.headHeight > 1.3f && now.upright > 0.8f && along >= 0.8f * w.commandSpeed && std::abs(across) < 0.6f && !down;
                    } else {
                        calm = now.standing && std::hypot(v.x, v.y) < settings.handoffCalmSpeed && !down;
                    }
                    w.handoffCalmSeconds = calm ? w.handoffCalmSeconds + TicksPerDecision * TickSeconds : 0.0f;
                    const float most = clipRunExercise(w.clipExercise) ? settings.handoffMaxSecondsRun : settings.handoffMaxSeconds;
                    handNow = w.handoffCalmSeconds >= 0.1f - 1.0e-4f || w.time >= w.clipDeliveredAt + most - 1.0e-4f;
                }
                if (handNow) clipHandoff(w);
            }
        }
        // (12/11 03h20) O tempo para levantar: na entrega (ou na troca, sem ela), os segundos do clipe percorridos desde o comeco.
        if (clipActive && w.getUpCovered < 0.0f && (w.clipDeliveredAt >= 0.0f || w.clipHandedAt >= 0.0f)) {
            w.getUpTarget = w.clipDeliveredAt >= 0.0f ? deliverAt : references[w.clipReference]->durationSeconds();
            w.getUpCovered = std::max(0.0f, std::min(w.getUpTarget, clipTime(w)) - w.clipStart);
        }
        if (clipActive && w.clipHandedAt < 0.0f) {
            w.locomotionControl = false;
        } else if (settings.separateSkills && (w.locomotion || w.taught || movingRun)) {
            // (04/11: no rastreio a cabeca ate 0,9 m - a frenagem da troca agacha a 1,05-1,1 m e passava o controle para a
            // recuperacao no meio dela.)
            // (12/11 16h, settings.parkedOnLocomotion) O idle estacionado depois da parada segue na locomocao.
            const bool parkedHere = settings.parkedOnLocomotion && w.locomotion && w.stopping && w.stopPhase == 2
                && (w.trackClip == standingIndex || w.trackClip == standingMirrorIndex);
            if ((!locomotionCommanded(w) && !movingRun && !freeStopNow(w) && !parkedHere) || down || now.headHeight < LostHeadHeight
                || now.pelvisHeight < LostPelvisHeight)
                w.locomotionControl = false;
            else if (now.standing) w.locomotionControl = true;
        } else if (settings.separateSkills && settings.idleOnLocomotion) {
            // (04/11 04h50: o parado e o pos-levantar na cabeca da locomocao, com o clipe do idle nos motores - a receita da
            // locomocao; caido ou baixo, a recuperacao.)
            // (04/11 07h20: a entrega ao idle so com o corpo quieto ha IdleHandoffSeconds - antes, 1,5 s depois do empurrao do
            // evitar ou logo ao ficar de pe, o idle, quase sem alcance de correcao, pegava o corpo ainda balancando e caia: o
            // evitar 0% com qualquer cabeca, o de pe no fim do levantar ~50%. Ja no idle, segue ate cair ou baixar.)
            if (!now.standing) w.calmFrom = -1.0f;
            // (05/11 04h40: no levantar pelo clipe, a cabeca da locomocao do comeco ao fim do clipe - deitada inclusive.)
            if (guidedRise(w))
                w.locomotionControl = true;
            else if (!idleTracked(w) || down || now.headHeight < LostHeadHeight || now.pelvisHeight < LostPelvisHeight)
                w.locomotionControl = false;
            // (07h40: no comeco do parado - de pe no idle, sem empurrao ainda - a entrega e na hora: com o ruido do treino o
            // corpo quase nunca ficava quieto 0,3 s e o parado inteiro ia para a recuperacao, a semelhanca 0,77 -> 0,26.)
            else if ((now.standing && (w.locomotionControl || w.stableSeconds >= IdleHandoffSeconds
                                       || ((w.startKind == 2 || (w.startKind == 4 && settings.avoidOnIdle)) && w.lastPushEnd < -1.0e8f)
                                       || (w.idleAfterGetUp && settings.idleBlendSeconds > 0.0f && arrived(w) && calm(w))))
                     || risingIntoIdle(w, now)) {
                // (09h00: depois de levantar, na hora - com os alvos indo dos da recuperacao aos do idle, ver IdleBlendSeconds.)
                if (!w.locomotionControl && w.idleAfterGetUp && w.lastRecoveryAction.size() == contract->actionSize()) {
                    w.idleBlendFrom = w.time;
                    w.idleBlendPose = w.lastRecoveryAction;
                }
                // (A chegada do levantar: a qualidade do equilibrio na hora da entrega - credito da ultima acao da recuperacao.)
                if (!w.locomotionControl && w.idleAfterGetUp && w.arrivalQuality < 0.0f) {
                    const TrackingBalance3D b = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions,
                        { footLinks[0], footLinks[1] }, floorUnder(w));
                    const float out = std::max(0.0f, -b.margin), planar = b.velocity.x * b.velocity.x + b.velocity.y * b.velocity.y,
                                rise = std::max(0.0f, b.velocity.z);
                    // (15h50: somada - o produto dava ~0,006 com a chegada de hoje e nao ensinava nada.)
                    w.arrivalQuality = 0.35f * std::exp(-(out / 0.15f) * (out / 0.15f)) + 0.35f * std::exp(-planar / 0.25f)
                        + 0.15f * std::exp(-rise * rise / 0.25f) + 0.15f * (b.footDown[0] && b.footDown[1] ? 1.0f : b.footDown[0] || b.footDown[1] ? 0.5f : 0.0f);
                    w.arrivalReward += settings.arrivalBonus * w.arrivalQuality;
                }
                w.locomotionControl = true;
            }
        }
        const Vec3 comVelocity = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
        if (settings.fireObservation) {
            // (06h40) A aceleracao do centro de massa, suavizada em ~0,1 s (settings.fireObservation).
            const float dt = TicksPerDecision * TickSeconds;
            if (w.comVelocityKnown) w.comAcceleration = w.comAcceleration + ((comVelocity - w.comVelocityBefore) / dt - w.comAcceleration) * (dt / 0.1f);
            w.comVelocityBefore = comVelocity;
            w.comVelocityKnown = true;
        }
        const bool contactDisturbance = std::any_of(w.body->interactions.begin(), w.body->interactions.end(),
            [](const RagdollInteraction3D& c) { return c.otherMotion != RagdollContactMotion3D::Static || c.normalWorld.z < .5f; });
        const bool quiet = w.pushTicks == 0 && w.time >= w.lastPushEnd + 1.5f && now.standing
            && comVelocity.lengthSquared() < .04f && w.body->links[rootLink].angularVelocity.lengthSquared() < .36f
            && w.body->links[contract->chestLink()].angularVelocity.lengthSquared() < .36f && !contactDisturbance;
        w.stableSeconds = quiet && w.commandSpeed <= 0.0f ? w.stableSeconds + TicksPerDecision * TickSeconds : 0.0f;
        const bool settled = w.stableSeconds >= .5f;
        w.locomotionFeedback = {};
        w.legGap = legGap(w);
        const bool pushFree = pushFreeNow(w);
        const bool freeNow = freeStopNow(w) || w.freeTroca || pushFree;
        const float clipScore = clipActive ? clipGetUpState(w, now) : 0.0f;
        // (12/11 04h40, settings.clipGetUpAdaptive: a nota do levantar livre - correndo, a do deslocamento pedido; de pe, a de ficar de pe.)
        float state = clipActive && adaptiveFor(w)
            ? (w.commandSpeed > 0.0f ? moveReward(w, now) : posture->standingReward(now, w.body->links, settled))
                + (settings.runSpeedReward > 0.0f && w.clipDeliveredAt >= 0.0f && w.commandSpeed > 0.1f && clipRunExercise(w.clipExercise)
                    ? settings.runSpeedReward * runSpeedRatio(w) : 0.0f)
            : clipActive ? clipScore : freeStopNow(w) ? freeStopScore(w, comVelocity) : w.freeTroca ? freeTrocaScore(w, comVelocity)
            : pushFree ? freePushScore(w, comVelocity)
            : (locomotionCommanded(w) || guidedRise(w)) && clipTracked(w)
            ? trackingScore(w, comVelocity)
            : w.commandSpeed > 0.0f ? moveReward(w, now)
            : w.locomotion ? 0.7f * posture->standingReward(now, w.body->links, settled) + (now.standing ? 0.3f * facing(w) : 0.0f)
                           : posture->standingReward(now, w.body->links, settled);
        if (settings.settleCalmWeight > 0.0f && clipActive && adaptiveFor(w) && w.commandSpeed <= 0.0f
            && !clipRunExercise(w.clipExercise) && now.standing && !down && upperChestLink < w.body->links.size()) {
            // (13/11, settings.settleCalmWeight) A calma de pe: o centro de massa parado (0,3 m/s -> e^-1), o peito sem girar (1 rad/s),
            // o ponto de captura na base (0,15 m fora -> e^-1) e os dois pes no chao.
            const TrackingBalance3D b = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions,
                { footLinks[0], footLinks[1] }, floorUnder(w));
            const float out = std::max(0.0f, -b.margin);
            const float calm = 0.30f * std::exp(-(comVelocity.x * comVelocity.x + comVelocity.y * comVelocity.y) / 0.09f)
                + 0.30f * std::exp(-w.body->links[upperChestLink].angularVelocity.lengthSquared())
                + 0.25f * std::exp(-(out / 0.15f) * (out / 0.15f))
                + 0.15f * (b.footDown[0] && b.footDown[1] ? 1.0f : b.footDown[0] || b.footDown[1] ? 0.5f : 0.0f);
            state = (1.0f - settings.settleCalmWeight) * state + settings.settleCalmWeight * calm;
        }
        // (15/11, settings.getUpViewReward) O levantar com vista: subindo, o peito para a vista ((1 + cos)/2: com gradiente ate os 180 graus).
        if (getUpViewOn(w)) {
            const float rise = std::clamp((now.headHeight - 0.15f) / 1.35f, 0.0f, 1.0f);
            state += settings.getUpViewReward * rise * (0.5f + 0.5f * std::cos(wrapAngle(chestYaw(w) - w.faceYaw)));
        }
        if (settings.stillCost > 0.0f && w.startKind == 7 && w.locomotionControl && upperChestLink < w.body->links.size()) {
            // (settings.stillCost no giro parado: o tronco e a pelve tombando - o giro em volta do vertical e livre - e o pe escorregando.)
            const auto tilting = [&](std::size_t i) {
                const Vec3 a = w.body->links[i].angularVelocity;
                return a.x * a.x + a.y * a.y;
            };
            const float tilt = std::min(4.0f, tilting(upperChestLink) + tilting(rootLink)) / 4.0f;
            float slip = 0.0f;
            for (const std::uint32_t f : footLinks) {
                const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == f; });
                if (down) slip += std::hypot(w.body->links[f].linearVelocity.x, w.body->links[f].linearVelocity.y);
            }
            state = std::max(0.0f, state - settings.stillCost * (0.5f * tilt + 0.5f * std::min(1.0f, slip / 0.2f))
                * (settings.reviewFixes ? 1.0f - fireFreedom(w) : 1.0f));
        }
        if (settings.turnSpineReward > 0.0f && w.startKind == 7 && w.locomotionControl && upperChestLink < w.body->links.size()) {
            // (settings.turnSpineReward - ver o .hpp: a coluna toda reta no giro, o eixo de cima da pelve contra o do peito alto.)
            TrackingClipFrame3D turnFrame;
            if (clipTracked(w)) {
                referenceFrame(w, 0.0f, true, turnFrame);
                const float turnFree = settings.reviewFixes ? fireFreedom(w) : 0.0f;
                state += settings.turnSpineReward * (spineAlignment(w, turnFrame) * (1.0f - turnFree) + turnFree);
                // (11h45: o tronco todo mirando, cada elo como no clipe, multiplica a nota do giro - so o peito virado nao basta.)
                state *= 0.4f + 0.6f * (trunkAim(w, turnFrame) * (1.0f - turnFree) + turnFree);
            }
        }
        // O premio do passo (com o pedido de deslocamento, de pe): cada pe que pousa depois de um tempo no ar.
        if (settings.stepReward > 0.0f) {
            float steps = 0.0f;
            for (int f = 0; f < 2; ++f) {
                const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == footLinks[f]; });
                if (down && !w.footDown[f] && w.commandSpeed > 0.0f && now.standing)
                    steps += std::min(w.footAir[f], 0.5f) - 0.15f;
                w.footAir[f] = down ? 0.0f : w.footAir[f] + TicksPerDecision * TickSeconds;
                w.footDown[f] = down;
            }
            // (Com o mesmo portao do rumo: passo virado para o lado errado nao vale.)
            const float stepGate = facing(w);
            state += settings.stepReward * steps * (steps > 0.0f ? stepGate : 1.0f);
        }
        // (10/11 07h05, settings.launchStepReward; ele: "o maior ganho de velocidade inicial de arrancada com o menor numero de passos"):
        // na janela da largada, cada pe que pousa paga o tempo que ficou no ar menos 0,15 s - o passinho (medido: 4-5 pousos em 0,8 s
        // contra 2 do clipe) perde, a passada longa ganha. Fora da largada, nada.
        if (settings.launchStepReward > 0.0f || settings.launchInPlacePenalty > 0.0f) {
            // (10/11 08h20, settings.launchInPlacePenalty; ele: "se conseguir pelo menos tirar os passinhos no lugar"): medido, na corrida
            // de frente os dois primeiros pousos (o esquerdo aos 0,14 s em todos os mundos, 6 cm para tras; e o direito) nao avancam - o
            // passo falso que rearruma a base do idle para a da largada. Na largada, cada pe que pousa tendo andado menos de
            // LaunchStepMinimum no rumo pedido desde que saiu do chao custa ate launchInPlacePenalty (para tras, o maximo).
            constexpr float LaunchStepMinimum = 0.15f;
            const float lw = w.locomotion ? launchWeight(w) : 0.0f;
            const Vec3 dir { std::cos(w.moveYaw), std::sin(w.moveYaw), 0.0f };
            float steps = 0.0f, inPlace = 0.0f;
            for (int f = 0; f < 2; ++f) {
                const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == footLinks[f]; });
                const Vec3 foot = w.body->links[footLinks[f]].position;
                if (!down && w.launchFootDown[f]) w.launchFootLift[f] = foot;
                if (down && !w.launchFootDown[f] && lw > 0.0f && now.standing) {
                    steps += std::min(w.launchFootAir[f], 0.5f) - 0.15f;
                    const float along = (foot.x - w.launchFootLift[f].x) * dir.x + (foot.y - w.launchFootLift[f].y) * dir.y;
                    inPlace += std::clamp((LaunchStepMinimum - along) / LaunchStepMinimum, 0.0f, 1.0f);
                }
                w.launchFootAir[f] = down ? 0.0f : w.launchFootAir[f] + TicksPerDecision * TickSeconds;
                w.launchFootDown[f] = down;
            }
            state += (settings.launchStepReward * steps - settings.launchInPlacePenalty * inPlace) * lw;
        }
        // (14/11 20h40, settings.launchLeanWeight) A inclinacao do tronco na largada: menos inclinado que o clipe no mesmo instante custa.
        if (settings.launchLeanWeight > 0.0f && w.locomotion && now.standing && clipTracked(w)) {
            const float lw = launchWeight(w);
            if (lw > 0.0f) {
                const std::array<float, 2> lean = launchLeans(w);
                state -= settings.launchLeanWeight * std::min(1.5f, std::max(0.0f, lean[1] - lean[0] - 5.0f) / 15.0f) * lw;
            }
        }
        // (15/11 06h30, settings.launchImitationReward - ver o .hpp) A largada da corrida de frente pelo clipe dela, como o levantar.
        w.launchImitation = -1.0f;
        if (settings.launchImitationReward > 0.0f && w.locomotion && w.runBand && now.standing && starting(w)
            && w.trackClip < references.size() && (w.heading % 8u == 0u || w.heading % 8u == 1u || w.heading % 8u == 7u)) {
            const float lw = launchWeight(w);
            if (lw > 0.0f) {
                const auto [t, yaw] = trackSample(w, 0.0f);
                const TrackingClipReference3D& reference = *references[w.trackClip];
                const Vec3 root = w.body->links[rootLink].position;
                Vec3 ground { root.x, root.y, floorUnder(w) };
                thread_local TrackingClipFrame3D launchFrame;
                reference.frame(t, ground, yaw, launchFrame, twist(w), pitch(w), legPitch(w));
                ground.x += root.x - launchFrame.links[rootLink].position.x;
                ground.y += root.y - launchFrame.links[rootLink].position.y;
                reference.frame(t, ground, yaw, launchFrame, twist(w), pitch(w), legPitch(w));
                // As velocidades na cadencia tocada e o centro de massa no plano com a velocidade da rampa do clipe (no lugar, sem ela
                // a nota pagaria ficar parado).
                const float rate = startRamp(w, w.time) * w.trackScale;
                for (PhysicsBodyState3D& link : launchFrame.links) {
                    link.linearVelocity = link.linearVelocity * rate;
                    link.angularVelocity = link.angularVelocity * rate;
                }
                const Vec3 want = wantedVelocity(w, 0.0f);
                const Vec3 have = rlBodyCenterOfMassVelocity3D(*profile, launchFrame.links);
                const Vec3 shift { want.x - have.x, want.y - have.y, 0.0f };
                for (PhysicsBodyState3D& link : launchFrame.links) link.linearVelocity = link.linearVelocity + shift;
                const TrackingClipGetUp3D::Terms terms = TrackingClipGetUp3D::score(*contract, *profile, w.body->links, launchFrame,
                    { settings.clipGetUpSoftness, settings.clipGetUpComVelocityWidth, settings.clipGetUpParts, 0.0f }, floorUnder(w));
                w.launchImitation = terms.total;
                state += settings.launchImitationReward * terms.total * lw;
            }
        }
        if (!idleTracked(w)) w.idleAnchored = w.holdAnchored = false;
        // A imitacao do clipe do passo (com o pedido, de pe ou nao: cair sai caro pela pose).
        if (!freeNow && (settings.imitationWeight > 0.0f || (settings.idleImitationWeight > 0.0f && idleTracked(w))) && clipTracked(w)) {
            const TrackingClipReference3D& reference = *references[w.trackClip];
            const float t = trackTime(w, 0.0f);
            const Vec3 root = w.body->links[rootLink].position;
            // (settings.idleClipAnchor) No parado, o clipe preso na ancora do parado (posta ja aqui, no primeiro passo dele).
            const bool idleHold = settings.idleClipAnchor && idleTracked(w);
            if (idleHold && !w.idleAnchored) {
                w.idleAnchor = w.handoffAnchorAt >= 0.0f && w.time - w.handoffAnchorAt < 0.5f ? w.handoffAnchor : root;
                w.handoffAnchorAt = -1.0f;
                w.idleAnchored = true;
            }
            reference.frame(t, { root.x, root.y, 0.0f }, w.trackYaw, w.frame, twist(w));
            if (idleHold) pelvisTo(w.frame, w.idleAnchor.x, w.idleAnchor.y);
            else underPelvis(w.frame, root.x, root.y);
            // (01/11 17h: o quadro vem na posicao do corpo, entao imitar parado pagava - o sprint caiu de 2,0 para
            // 0,3 m/s. A imitacao vale na proporcao da velocidade feita sobre a pedida, no rumo pedido.)
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            if (idleTracked(w)) {
                // (Parado: a imitacao do idle inteira e a quietude - o centro de massa parado no chao e as juntas
                // quietas: a estabilidade, o equilibrio e a quietude que ele pediu.)
                double spin = 0.0;
                for (const PhysicsBodyState3D& link : w.body->links) spin += link.angularVelocity.lengthSquared();
                // (01/11 21h30: exigir aqui tambem o rumo do idle - 20h20, ele girava devagar no parado - derrubou o levantar
                // das quedas no treino de 97% para ~50% em meia hora; revertido.)
                // (01/11 22h, ele: "micromovimentos imperceptiveis nos pes que faz ele se arrastar para tras": a
                // velocidade baixa quase nao custava; a quietude tambem exige ficar no ponto do idle - 0,3 m fora, 41%.)
                if (!w.idleAnchored) {
                    w.idleAnchor = root;
                    w.idleAnchored = true;
                }
                const float dx = root.x - w.idleAnchor.x, dy = root.y - w.idleAnchor.y;
                const float still = std::exp(-25.0f * (v.x * v.x + v.y * v.y)) * std::exp(-10.0f * (dx * dx + dy * dy))
                    * static_cast<float>(std::exp(-0.02 * spin / std::max<std::size_t>(1, w.body->links.size())));
                // (07/11, ele: "no Parado, seria interessante que a quietude fosse recompensada, sem afetar e claro o equilibrio":
                // a de cima quase nao cobrava as juntas - 1 rad/s em todos os elos custava 2% - e nao via o centro de massa
                // subindo e descendo. Com idleQuiet: cada elo alem de IdleQuietSlack e o sobe-desce do centro de massa custam; o
                // equilibrio e os passos seguem pagos a parte, e cair continua encerrando. A medida do portao segue a de cima.)
                float quiet = still;
                if (settings.idleQuiet > 0.0f) {
                    double excess = 0.0;
                    for (const PhysicsBodyState3D& link : w.body->links) {
                        const float e = std::max(0.0f, link.angularVelocity.length() - IdleQuietSlack);
                        excess += static_cast<double>(e) * e;
                    }
                    excess /= static_cast<double>(std::max<std::size_t>(1, w.body->links.size()));
                    quiet = std::exp(-25.0f * (v.x * v.x + v.y * v.y + v.z * v.z)) * std::exp(-10.0f * (dx * dx + dy * dy))
                        * static_cast<float>(std::exp(-static_cast<double>(settings.idleQuiet) * excess));
                }
                // (05/11 21h10, idleReanchor) A estabilidade MEDIDA (a do portao do levantar e do parado): a mesma quietude, com
                // a ancora propria recomecando depois de cada empurrao e fora da janela dele - o empurrao e o passo de recuperar
                // sao do equilibrio, como o fim do levantar desde 05/11 05h20. Com idleThroughPushes a ancora da recompensa nunca
                // recomeca: recuperado do empurrao 0,4 m adiante, a quietude medida ficava ~0,2 o resto do episodio. A recompensa
                // segue com a de sempre; < 0: fora da medida.
                float measured = still;
                if (settings.idleReanchor) {
                    const bool pushWindow = w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f;
                    if (pushWindow) {
                        w.measureAnchored = false;
                        measured = -1.0f;
                    } else {
                        if (!w.measureAnchored) {
                            w.measureAnchor = root;
                            w.measureAnchored = true;
                        }
                        const float mx = root.x - w.measureAnchor.x, my = root.y - w.measureAnchor.y;
                        measured = std::exp(-25.0f * (v.x * v.x + v.y * v.y)) * std::exp(-10.0f * (mx * mx + my * my))
                            * static_cast<float>(std::exp(-0.02 * spin / std::max<std::size_t>(1, w.body->links.size())));
                    }
                }
                if (settings.idleImitationWeight > 0.0f) {
                    // (04/11 04h10, ele: "no exercicio parado alerta deveria ser muito cobrado a semelhanca com o parado idle
                    // alerta": a pose do idle na regua estrita do DeepMimic (suavidade 1) e com peso proprio - mas no rumo do
                    // proprio corpo: a forma, nao para onde olha (01/11 21h30: cobrar o rumo do idle derrubou o levantar).)
                    reference.frame(t, {}, 0.0f, w.idleFrame);
                    const float yaw = wrapAngle(pelvisHeading(w.body->links[rootLink]) - pelvisHeading(w.idleFrame.links[rootLink]));
                    reference.frame(t, { root.x, root.y, 0.0f }, yaw, w.idleFrame);
                    if (idleHold) pelvisTo(w.idleFrame, w.idleAnchor.x, w.idleAnchor.y);
                    else underPelvis(w.idleFrame, root.x, root.y);
                    float match = contract->reward(w.body->links, w.idleFrame, 1.0f).total;
                    // (Na cabeca da locomocao, a nota na escala dela: de pe 0,10 + a pose do idle 0,60 + a quietude 0,30.)
                    if (settings.idleOnLocomotion && w.locomotionControl) {
                        // (04/11 05h35, ele: "o clipe do IDLE parado nao pode ser o original, que tem a coluna torta" - medido: o
                        // clipe e o corrigido de 23/10, reto de lado (1,1 graus com o clipe puro); a coluna torta vinha da
                        // correcao da rede (9 graus antes de 05h23, 4,7 depois). A coluna reta entra na nota: o eixo de cada elo
                        // contra o do idle, exp(-soma dos angulos^2 / 0,1).)
                        const float spine = posture->idleSpineMatch(w.body->links);
                        // (07h40, ele: "a exigencia de semelhanca com o clipe muito mais alta, obrigatoriamente": a pose do idle e o
                        // lugar dos pes valem juntos - abrir a base derruba a semelhanca inteira -, com peso 0,55; e perder a pose
                        // encerra (IdleFeetLimit).)
                        float idleFeet = 0.0f;
                        {
                            const float bh = pelvisHeading(w.body->links[rootLink]), ih = pelvisHeading(w.idleFrame.links[rootLink]);
                            const auto toLocal = [](Vec3 v, float h) {
                                const float c = std::cos(h), sn = std::sin(h);
                                return Vec3 { c * v.x + sn * v.y, -sn * v.x + c * v.y, v.z };
                            };
                            for (std::uint32_t f : footLinks)
                                idleFeet = std::max(idleFeet, (toLocal(w.body->links[f].position - root, bh)
                                    - toLocal(w.idleFrame.links[f].position - w.idleFrame.links[rootLink].position, ih)).lengthSquared());
                        }
                        // (settings.basePosture: a base pela regua que nao satura - a 25 cm o nucleo de 7 cm dava ~0.)
                        const float feetHeld = settings.basePosture > 0.0f
                            ? 1.0f / (1.0f + idleFeet / (TrackingBasePosture3D::BaseWidth * TrackingBasePosture3D::BaseWidth))
                            : std::exp(-idleFeet / 0.005f);
                        const float drop = w.idleFrame.links[rootLink].position.z - root.z;
                        // (09/11 03h40, ele: "quando ele esta parado ... dificuldades de manter a pose" - medido: 4,1 cm mais baixo que
                        // o idle (2,0 antes da nota da velocidade), as pernas 25 graus por junta longe dele; agachar so contava em
                        // "perdeu a pose", a 10 cm. A pelve abaixo do idle alem de 1,5 cm derruba a semelhanca da pose.)
                        float tall = 1.0f;
                        if (settings.idleHeightSlack > 0.0f) {
                            const float low = std::max(0.0f, drop - 0.015f) / settings.idleHeightSlack;
                            tall = std::exp(-low * low);
                        }
                        match *= tall;
                        state = 0.10f + 0.55f * match * feetHeld + 0.15f * spine + 0.20f * quiet;
                        bool lost = std::sqrt(idleFeet) > IdleFeetLimit || drop > IdlePelvisDrop;
                        float unifiedBalance = 1.0f;
                        if (settings.balanceSteps && w.stepsOn) {
                            // (04/11 15h, ele: "os passos de recuperacao e estabilizacao... para o auto-equilibrio"; vault
                            // 04_locomocao/plano_passos_de_recuperacao_04_11.md) Com o gerador de passos: a semelhanca contra a
                            // referencia que os motores seguem (as pernas do passo no peso do gerador) e os pes julgados por ela
                            // enquanto a base nao e a do idle; o equilibrio (o ponto de captura dentro da base); a base do idle
                            // (o passo de estabilizacao a devolve); cada passo de recuperacao custa um pouco (so passo quando
                            // precisa). Perder a pose so com a base muito aberta ou a pelve muito baixa.
                            const float k = w.stepPoseValid ? w.stepPoseWeight : 0.0f;
                            float stepMatch = match;
                            if (k > 0.0f && w.stepPose.linkPositions.size() == w.idleFrame.links.size()) {
                                for (int side = 0; side < 2; ++side)
                                    for (std::size_t i : { legs.thigh[static_cast<std::size_t>(side)], legs.shin[static_cast<std::size_t>(side)],
                                             legs.foot[static_cast<std::size_t>(side)] })
                                        if (i < w.idleFrame.links.size()) {
                                            w.idleFrame.links[i].position = w.stepPose.linkPositions[i];
                                            w.idleFrame.links[i].orientation = w.stepPose.linkOrientations[i];
                                        }
                                stepMatch = contract->reward(w.body->links, w.idleFrame, 1.0f).total * tall;
                            }
                            const TrackingBalance3D b = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions,
                                { footLinks[0], footLinks[1] }, floorUnder(w));
                            const float out = std::max(0.0f, -b.margin);
                            const float balance = std::exp(-(out / 0.05f) * (out / 0.05f));
                            const float stanceError = w.steps.stanceError();
                            const float stance = std::exp(-stanceError * stanceError / 0.005f);
                            const float held = (1.0f - k) * feetHeld + k;
                            // (07/11, com idleQuiet: a quietude 0,15 -> 0,25, de 0,05 do de pe e 0,05 da pose.)
                            const bool quietOn = settings.idleQuiet > 0.0f;
                            unifiedBalance = balance;
                            state = (quietOn ? 0.05f : 0.10f) + (quietOn ? 0.35f : 0.40f) * stepMatch * held + 0.10f * spine
                                + (quietOn ? 0.25f : 0.15f) * quiet + 0.15f * balance + 0.10f * stance
                                - (w.steps.recoveryStarted() ? 0.05f : 0.0f) - (w.steps.settleStarted() ? 0.02f : 0.0f);
                            if (now.standing && b.footDown[0] && b.footDown[1]) w.worstCaptureOut = std::max(w.worstCaptureOut, out);
                            const bool quietNow = now.standing && b.margin > 0.03f && std::hypot(b.velocity.x, b.velocity.y) < 0.15f
                                && w.steps.swinging() < 0;
                            if (w.pushes > 0 && w.quietAgainAt < 0.0f && w.pushTicks == 0 && w.time > w.lastPushEnd && quietNow)
                                w.quietAgainAt = w.time - w.lastPushEnd;
                            lost = stanceError > 0.45f || drop > 2.0f * IdlePelvisDrop;
                            match = stepMatch;
                        }
                        if (settings.idleUnified) {
                            // (11/11, settings.idleUnified - ver o .hpp.) A forma: cada junta contra o idle (com as pernas do passo
                            // quando o gerador da um), na cauda longa de 0,15 rad; o lugar: o idle alerta na vista (basePosture).
                            w.jointErrors.resize(trackingParts.size());
                            contract->jointErrors(w.body->links, w.idleFrame, w.jointErrors);
                            float shape = 0.0f;
                            for (const float e : w.jointErrors) shape += 1.0f / (1.0f + e / (0.15f * 0.15f));
                            shape /= static_cast<float>(std::max<std::size_t>(1, w.jointErrors.size()));
                            const TrackingBasePosture3D bp = basePosture(w);
                            const auto tail = [](float x, float width) { const float u = x / width; return 1.0f / (1.0f + u * u); };
                            const float place = 0.2f * (tail(bp.baseMeters, TrackingBasePosture3D::BaseWidth)
                                + tail(bp.pelvisYaw, TrackingBasePosture3D::YawWidth) + tail(bp.chestYaw, TrackingBasePosture3D::YawWidth)
                                + tail(bp.chestTilt, TrackingBasePosture3D::TiltWidth) + tail(bp.drop, 0.03f));
                            const float steady = 0.5f + 0.5f * unifiedBalance * quiet;
                            match = shape * place;
                            float idleScore = match * steady;
                            if (settings.idleMotionCost > 0.0f || settings.idlePoseCost > 0.0f) {
                                // (settings.idleMotionCost/idlePoseCost - ver o .hpp.)
                                double spin2 = 0.0;
                                for (const PhysicsBodyState3D& link : w.body->links) spin2 += link.angularVelocity.lengthSquared();
                                spin2 /= static_cast<double>(std::max<std::size_t>(1, w.body->links.size()));
                                const float motion = static_cast<float>(spin2) + 10.0f * (v.x * v.x + v.y * v.y + v.z * v.z);
                                const float moving = 1.0f - std::exp(-motion / 0.25f);
                                float jointMean = 0.0f;
                                for (const float e : w.jointErrors) jointMean += e;
                                jointMean /= static_cast<float>(std::max<std::size_t>(1, w.jointErrors.size()));
                                const auto far = [](float x, float width) { return std::min(3.0f, x / width); };
                                const float distance = (far(std::sqrt(jointMean), 0.15f) + far(bp.baseMeters, TrackingBasePosture3D::BaseWidth)
                                    + far(bp.pelvisYaw, TrackingBasePosture3D::YawWidth) + far(bp.chestYaw, TrackingBasePosture3D::YawWidth)
                                    + far(bp.chestTilt, TrackingBasePosture3D::TiltWidth) + far(bp.drop, 0.03f)) / 18.0f;
                                idleScore = std::max(0.0f, 0.3f + 0.7f * idleScore - settings.idleMotionCost * moving
                                    - settings.idlePoseCost * distance);
                            }
                            if (settings.idleRewardPose > 0.0f) {
                                // (settings.idleRewardPose/idleRewardStill/idleSlideCost - ver o .hpp.)
                                const float close = (0.5f * shape + 0.5f * place) * (1.0f - 0.5f * fireFreedom(w)) + 0.5f * fireFreedom(w);
                                double spin2 = 0.0;
                                for (const PhysicsBodyState3D& link : w.body->links) spin2 += link.angularVelocity.lengthSquared();
                                spin2 /= static_cast<double>(std::max<std::size_t>(1, w.body->links.size()));
                                const float motion = static_cast<float>(spin2) + 10.0f * (v.x * v.x + v.y * v.y + v.z * v.z);
                                // (12/11 16h30, settings.locoFireRelief) Sob o fogo, a quietude nao e cobrada, a pose fica mais solta e
                                // o deslize e os passos nao custam; caindo p, a pose volta a ser cobrada inteira - a vontade de voltar a ela.
                                const float idleFree = fireFreedom(w);
                                const float still = 1.0f / (1.0f + motion / 0.25f) * (1.0f - idleFree) + idleFree;
                                // (12/11 17h, settings.crookedStartShare) O torto reajustou: perto da pose (shape e lugar) e quieto por 0,3 s; o
                                // premio da rapidez, cheio no comeco e zero em settings.crookedFixSpan s.
                                float fixBonus = 0.0f;
                                if (w.crooked && w.crookedFixedAt < 0.0f) {
                                    // (17h05, a sonda: a forma junta a junta do idle da rede ja fica ~24 graus do clipe no parado normal - o
                                    // reajuste e o lugar - a base, o giro, o peito na vertical, a altura - e a coluna alinhada, nao a forma.)
                                    const bool near = place >= settings.crookedFixClose && spineAlignment(w, w.idleFrame) >= 0.6f && motion < 0.25f;
                                    w.crookedFixSeconds = near ? w.crookedFixSeconds + TicksPerDecision * TickSeconds : 0.0f;
                                    if (w.crookedFixSeconds >= 0.3f - 1.0e-4f) {
                                        w.crookedFixedAt = std::max(0.0f, w.time - 0.3f);
                                        fixBonus = settings.crookedStartBonus * std::max(0.0f, 1.0f - w.crookedFixedAt / settings.crookedFixSpan);
                                    }
                                }
                                float slide = 0.0f;
                                for (const std::uint32_t f : footLinks) {
                                    const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                                        [&](const RagdollContactPoint3D& c) { return c.linkIndex == f; });
                                    if (down) slide += std::hypot(w.body->links[f].linearVelocity.x, w.body->links[f].linearVelocity.y);
                                }
                                // (11/11 09h45, ele: "a recompensa por ficar na pose nunca pode ser maior que o objetivo de nao cair"):
                                // de pe vale a base 0,5; a pose e a quietude sao o bonus ate 0,5 por cima; o deslize come so do
                                // bonus e da base, e a nota nunca fica negativa - cair (que encerra) nunca compensa.
                                // (11/11 09h55, ele: "no parado tambem, precisa recompensar por manter a coluna toda ALINHADA": a coluna
                                // alinhada - o eixo de cima da pelve contra o do peito alto, 1/(1+(angulo/8 graus)^2) - entra no bonus com o
                                // peso turnSpineReward, como no giro.)
                                float aligned = settings.turnSpineReward > 0.0f ? spineAlignment(w, w.idleFrame) : 0.0f;
                                if (settings.reviewFixes) aligned = aligned * (1.0f - idleFree) + idleFree;
                                // (13/11 v1, settings.idleStillBase) A quietude so vale inteira na base dos pes do idle (fora do fogo).
                                const float stillGate = settings.idleStillBase > 0.0f
                                    ? 1.0f - settings.idleStillBase * (1.0f - tail(bp.baseMeters, TrackingBasePosture3D::BaseWidth)) * (1.0f - idleFree) : 1.0f;
                                // (15/11, settings.idleHoldWeight) O lugar: o ponto onde o parado comecou, refeito so depois de empurrao ou fogo.
                                float hold = 1.0f;
                                if (settings.idleHoldWeight > 0.0f) {
                                    const bool shoved = w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f || idleFree > 0.0f;
                                    if (shoved) {
                                        w.holdAnchored = false;
                                    } else if (!w.holdAnchored && (w.holdAtHandoff || std::hypot(v.x, v.y) < 0.1f)) {
                                        w.holdAnchor = root;
                                        w.holdAnchored = true;
                                        w.holdAtHandoff = false;
                                    }
                                    if (w.holdAnchored) {
                                        const float u = std::hypot(root.x - w.holdAnchor.x, root.y - w.holdAnchor.y)
                                            / std::max(0.02f, settings.idleHoldWidth);
                                        hold = 1.0f / (1.0f + u * u);
                                    }
                                }
                                // (15/11, settings.idleWeightCenter) Onde esta o peso: o centro de massa contra o meio dos pes, como no clipe.
                                float centered = 1.0f;
                                if (settings.idleWeightCenter > 0.0f && rootLink < w.idleFrame.links.size()) {
                                    const auto offset = [&](const std::vector<PhysicsBodyState3D>& links, Vec3 com) {
                                        const Vec3 mid = (links[footLinks[0]].position + links[footLinks[1]].position) * 0.5f;
                                        const float h = pelvisHeading(links[rootLink]), cs = std::cos(h), sn = std::sin(h);
                                        const float dx = com.x - mid.x, dy = com.y - mid.y;
                                        return std::array<float, 2> { cs * dx + sn * dy, -sn * dx + cs * dy };
                                    };
                                    const auto b = offset(w.body->links, rlBodyCenterOfMass3D(*profile, w.body->links));
                                    const auto r = offset(w.idleFrame.links, w.idleFrame.centerOfMass);
                                    const float u = std::hypot(b[0] - r[0], b[1] - r[1]) / 0.025f;
                                    centered = (1.0f / (1.0f + u * u)) * (1.0f - idleFree) + idleFree;
                                }
                                const float bonus = (settings.idleRewardPose * close + settings.idleRewardStill * still * stillGate
                                    + settings.turnSpineReward * aligned) * ((1.0f - settings.idleHoldWeight) + settings.idleHoldWeight * hold)
                                    * ((1.0f - settings.idleWeightCenter) + settings.idleWeightCenter * centered);
                                float costs = 0.5f * settings.idleSlideCost * std::min(1.0f, slide);
                                if (settings.stillCost > 0.0f) {
                                    // (settings.stillCost - ver o .hpp: os custos de quietude do RL v1.)
                                    const float trunkSpin = std::min(4.0f, w.body->links[upperChestLink].angularVelocity.lengthSquared()
                                        + w.body->links[rootLink].angularVelocity.lengthSquared()) / 4.0f;
                                    float slip = 0.0f;
                                    for (const std::uint32_t f : footLinks) {
                                        const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                                            [&](const RagdollContactPoint3D& c) { return c.linkIndex == f; });
                                        if (down) {
                                            const PhysicsBodyState3D& foot = w.body->links[f];
                                            slip += std::hypot(foot.linearVelocity.x, foot.linearVelocity.y) + 0.1f * std::abs(foot.angularVelocity.z);
                                        }
                                    }
                                    const float comCost = std::min(1.0f, std::hypot(v.x, v.y) / 0.2f);
                                    costs = settings.stillCost * (0.4f * trunkSpin + 0.4f * std::min(1.0f, slip / 0.2f) + 0.2f * comCost);
                                }
                                idleScore = std::max(0.0f, 0.5f + 0.5f * bonus
                                    / std::max(1.0e-3f, settings.idleRewardPose + settings.idleRewardStill + settings.turnSpineReward)
                                    - costs * (1.0f - idleFree));
                                idleScore += fixBonus;
                                match = close;
                            }
                            state = idleScore - ((w.stepsOn && w.steps.recoveryStarted() ? 0.05f : 0.0f)
                                + (w.stepsOn && w.steps.settleStarted() ? 0.02f : 0.0f)) * (1.0f - fireFreedom(w));
                        }
                        // (08/11 11h25, o fogo: o passo de recuperacao muda a base do idle, e a pose perdida encerrava o parado
                        // empurrado como queda - 9% dos parados, 65% de sucesso com empurrao contra 100% sem. Durante o empurrao
                        // e ate IdleRecoverSeconds depois, nao conta; dai, ele tem de voltar a pose.)
                        const bool recovering = w.pushTicks > 0 || w.time < w.lastPushEnd + IdleRecoverSeconds;
                        // (09/11 06h15, ele: "O parado as vezes ainda fica todo estranho, abaixado e torto" - medido: 5-6 cm abaixo
                        // do idle (o pior 8,5), o peito alto 17-24 graus inclinado (o idle 1; o pior 44), a pelve 16-19 graus fora.
                        // A nota do idle pagava quieto e equilibrado (0,25 + 0,15 + 0,10) quase sem a pose - agachar curvado e o mais
                        // estavel. A nota inteira vezes a pose (a altura e o peito alto contra os do idle), fora do empurrao.)
                        // (10h05: e nao nos IdleSettleSeconds depois que o idle chega de uma parada - medido, as quedas da parada vem
                        // 0,6-1,4 s depois do idle, com o corpo ainda a 1,5-3 m/s; para assentar ele inclina e da passos.)
                        constexpr float IdleSettleSeconds = 2.0f;
                        if (settings.idlePoseGate > 0.0f && !settings.idleUnified && !recovering && w.time >= w.idleSince + IdleSettleSeconds
                            && upperChestLink < w.body->links.size()) {
                            const auto up = [&](const PhysicsBodyState3D& link) {
                                return (link.orientation * profile->links[upperChestLink].modelOrientation.conjugate()).normalized()
                                    .rotate({ 0.0f, 0.0f, 1.0f });
                            };
                            const float tilt = std::acos(std::clamp(dot(up(w.body->links[upperChestLink]), up(w.idleFrame.links[upperChestLink])),
                                -1.0f, 1.0f));
                            constexpr float IdleTiltSlack = 0.10f, IdleTiltWidth = 0.17f;
                            const float bent = std::max(0.0f, tilt - IdleTiltSlack) / IdleTiltWidth;
                            const float low = settings.idleHeightSlack > 0.0f ? std::max(0.0f, drop - 0.015f) / settings.idleHeightSlack : 0.0f;
                            // (settings.poseBoost: a coluna contra a do idle - a torção e a curva.)
                            const float spineBad = settings.poseBoost > 0.0f
                                ? std::max(0.0f, spineDegrees(w, w.idleFrame) - IdleSpineSlack) / IdleSpineWidth : 0.0f;
                            const float pose = std::exp(-bent * bent - low * low - spineBad * spineBad);
                            state *= 1.0f - settings.idlePoseGate + settings.idlePoseGate * pose;
                        }
                        // (10/11 12h, settings.basePosture: a nota do parado poe o idle no rumo do proprio corpo - medido, o corpo ficava
                        // 20-45 graus girado para fora da vista. Metade dela passa a ser o atrator do idle alerta de frente para a vista.)
                        if (settings.basePosture > 0.0f && w.locomotionControl && !settings.idleUnified) {
                            // (12h15: so a media nao segurava o rumo - o corpo girava 28-47 graus. De frente para a vista e condicao, como
                            // na parada livre: o peito fora da vista alem da folga derruba a nota inteira.)
                            const float attract = basePosture(w).attraction();
                            const float off = std::max(0.0f, std::abs(wrapAngle(w.faceYaw - chestYaw(w))) - FreeFaceSlack) / FreeFaceWidth;
                            state = (0.5f * state + 0.5f * attract) * std::exp(-off * off);
                        }
                        // (09/11 17h30, settings.freeIdle) O parado livre: a nota do parado da parada livre (o progresso a parado na
                        // regua de 1 m/s), sem o fim por pose perdida - recuperar de um empurrao com um passo e livre.
                        if (settings.freeIdle) {
                            // (10/11 01h50, o rosto do parado: fora da locomocao o alvo do rosto nunca era posto - ficava o do
                            // episodio anterior - e a nota do parado dava ~0 em 62% das linhas desde a primeira decisao, de pe: so
                            // o juiz moldava a pose, e ele prefere o corpo agachado. O rumo do peito quando o parado comeca.)
                            if (!w.locomotion && !w.idleFaceSet) {
                                w.faceYaw = chestYaw(w);
                                w.idleFaceSet = true;
                            }
                            w.freeFromSpeed = 1.0f;
                            state = freeStopScore(w, v);
                            lost = false;
                        }
                        w.idleLostSeconds = w.startKind == 2 && lost && !recovering && w.time >= IdleLockAfterSeconds
                            ? w.idleLostSeconds + TicksPerDecision * TickSeconds : 0.0f;
                        // (04/11 05h20: o parado tambem no retorno do rastreio - a nota, os erros por parte contra o idle e a
                        // semelhanca -, para a media do log nao misturar zeros e para ver o que foge do idle.)
                        contract->jointErrors(w.body->links, w.idleFrame, w.jointErrors);
                        float part[6] {};
                        for (std::size_t j = 0; j < trackingParts.size() && j < w.jointErrors.size(); ++j) part[trackingParts[j]] += w.jointErrors[j];
                        const auto degrees = [&](TrackingPart p) {
                            return trackingPartCount[p] > 0.0f ? std::sqrt(part[p] / trackingPartCount[p]) * 180.0f / Pi : 0.0f;
                        };
                        const float legs = std::max(part[PartLeftLeg], part[PartRightLeg]);
                        const float bodyHeading = pelvisHeading(w.body->links[rootLink]), idleHeading = pelvisHeading(w.idleFrame.links[rootLink]);
                        const auto local = [](Vec3 v, float h) {
                            const float c = std::cos(h), s = std::sin(h);
                            return Vec3 { c * v.x + s * v.y, -s * v.x + c * v.y, v.z };
                        };
                        float feet = 0.0f;
                        for (std::uint32_t f : footLinks)
                            feet = std::max(feet, (local(w.body->links[f].position - root, bodyHeading)
                                                   - local(w.idleFrame.links[f].position - w.idleFrame.links[rootLink].position, idleHeading)).lengthSquared());
                        w.locomotionFeedback = {};
                        w.locomotionFeedback[0] = state;
                        w.locomotionFeedback[1] = part[PartRoot];
                        w.locomotionFeedback[2] = legs;
                        w.locomotionFeedback[3] = part[PartTrunk] + part[PartHead];
                        w.locomotionFeedback[4] = part[PartArms];
                        w.locomotionFeedback[5] = feet;
                        w.locomotionFeedback[9] = match;
                        w.locomotionFeedback[10] = still;
                        w.locomotionFeedback[16] = degrees(PartTrunk);
                        w.locomotionFeedback[17] = degrees(PartHead);
                        w.locomotionFeedback[18] = degrees(PartArms);
                        // (Pernas e pelve em graus por junta, e a altura da pelve contra a do idle, m: o agachamento.)
                        w.locomotionFeedback[11] = std::sqrt(legs / std::max(1.0f, trackingPartCount[PartLeftLeg])) * 180.0f / Pi;
                        w.locomotionFeedback[12] = std::sqrt(part[PartRoot]) * 180.0f / Pi;
                        w.locomotionFeedback[6] = w.body->links[rootLink].position.z - w.idleFrame.links[rootLink].position.z;
                        w.locomotionFeedback[8] = spine;
                    } else
                        state += settings.idleImitationWeight * (match + 0.5f * still);
                    // (12/11 01h10) No levantar pelo clipe, o parado depois da troca nao entra na semelhanca do episodio (so o clipe).
                    if (!w.clipGetUp) w.imitationSum += match;
                    if (measured >= 0.0f) {
                        w.stabilityDeficitSum += (1.0f - measured) * (1.0f - measured);
                        ++w.stabilitySamples;
                    }
                    if (!w.clipGetUp) ++w.trackSamples;
                } else {
                    state += settings.imitationWeight * (contract->reward(w.body->links, w.frame, 4.0f).total + 0.5f * still);
                }
            } else if (w.getUpGuided) {
                // (O levantar guiado: a imitacao na proporcao da altura do corpo sobre a do clipe - 02/11 00h: inteira, deitado
                // imitando o comeco do clipe pagava tanto quanto levantar.)
                const float height = std::clamp(
                    rlBodyCenterOfMass3D(*profile, w.body->links).z / std::max(0.05f, w.frame.centerOfMass.z), 0.0f, 1.0f);
                state += settings.imitationWeight * height * contract->reward(w.body->links, w.frame, 4.0f).total;
            } else {
                const float toward = moveDirection(w);
                const float progress =
                    std::clamp((v.x * std::cos(toward) + v.y * std::sin(toward)) / w.commandSpeed, 0.0f, 1.0f)
                    * sideDrift(w, v);
                state += settings.imitationWeight * progress * contract->reward(w.body->links, w.frame, 4.0f).total;
            }
        }
        // (O gamma do treinador, 0,99, no potencial.)
        // (04/11 11h25) A naturalidade da recuperacao: a aspereza (ate RecoveryRoughScale, o p90 medido) e as juntas no limite
        // (ate metade dos alvos; medido: 26-31% em media, p90 55%), cada uma metade.
        // (11h55: como fracao da nota do estado - ate recoveryNatural dela -, nao multa por passo: subtraida, deitado quieto
        // custava menos que lutar para levantar e o levantar das avaliacoes caiu de 97% para 81-83%.)
        if (settings.recoveryNatural > 0.0f && settings.separateSkills && !w.locomotionControl && !w.fallActive && state > 0.0f)
            state *= 1.0f - std::min(1.0f, settings.recoveryNatural)
                * (0.5f * std::min(1.0f, w.recoveryRough / RecoveryRoughScale) + 0.5f * std::min(1.0f, 2.0f * w.recoveryLimits));
        reward = state + (shaped ? 0.99f * (posture->risePotential(now) + posture->turnPotential(now) + facePotential(w, now)) : 0.0f)
            - before;
        // (13/11 11h30, settings.getUpFormWeight) A forma da cabeca e do peito na subida e na firmeza: custo, nao premio - ficar no meio
        // da subida com a forma boa nao paga nada.
        if (settings.getUpFormWeight > 0.0f && clipActive && adaptiveFor(w) && !w.locomotionControl && !down) {
            const float form = now.headHeight > 0.6f ? getUpFormScore(w, w.body->links, floorUnder(w), now.standing) : 1.0f;
            const float palm = palmScore(w.body->links, floorUnder(w));
            // (A mao virada dura ~0,17 s - poucos passos -: com ela apoiada, metade da nota; com settings.getUpPalmWeight, peso proprio.)
            if (settings.getUpPalmWeight > 0.0f)
                reward -= settings.getUpFormWeight * (1.0f - form) + settings.getUpPalmWeight * (1.0f - palm);
            else
                reward -= settings.getUpFormWeight * (1.0f - (palm < 1.0f ? 0.5f * form + 0.5f * palm : form));
        }
        // (14/11 20h10, settings.getUpLegWeight) A forma da perna esquerda: limpo, na subida e na firmeza dos exercicios da mascara.
        if (settings.getUpLegWeight > 0.0f && clipActive && adaptiveFor(w) && !w.locomotionControl && !down && w.fireKind == 0
            && w.clipExercise < 32u && ((settings.getUpLegMask >> w.clipExercise) & 1u) != 0u && now.headHeight > 0.6f) {
            const float err = getUpLegError(w, w.body->links, floorUnder(w));
            if (err >= 0.0f) reward -= settings.getUpLegWeight * std::min(1.5f, std::max(0.0f, err - 0.05f) / 0.30f);
        }
        // (13/11 17h45, settings.getUpPaceWeight) O ritmo do levantar limpo de pe: a cabeca nao sobe antes da do clipe no ritmo pedido.
        // (18h55: ate o tempo do ritmo, nao ate a entrega - so ate a entrega, subir ainda mais rapido encurtava os passos cobrados; o de
        // brucos foi de ~1,30 a 1,21-1,24 s. Ja de pe antes da hora, segue pagando por estar adiantado; depois da troca tambem.)
        const float paceDeliver = w.clipExercise < clipGetUpDeliver.size() && clipGetUpDeliver[w.clipExercise] > 0.0f
            ? clipGetUpDeliver[w.clipExercise] : (w.clipGetUp ? references[w.clipReference]->durationSeconds() : 0.0f);
        const float paceEnd = paceDeliver > 0.0f ? std::max(0.0f, paceDeliver - w.clipStart) * settings.getUpPaceSeconds * getUpTimeScale(w) / paceDeliver : 0.0f;
        // (13/11 v1, a revisao: so ate a troca - o treino corta a trajetoria nela, e o custo de depois nunca chegava na subida; so ensinava
        // o idle do pos-levantar a abaixar a cabeca. Entre a entrega e a troca segue valendo: subir antes ainda paga ali.)
        if (settings.getUpPaceWeight > 0.0f && w.clipGetUp && adaptiveFor(w) && w.fireKind == 0 && w.clipHandedAt < 0.0f
            && !clipRunExercise(w.clipExercise) && w.time < paceEnd && w.clipExercise < clipGetUpForm.size()
            && !clipGetUpForm[w.clipExercise].empty()) {
            const std::vector<GetUpFormSample>& table = clipGetUpForm[w.clipExercise];
            const float deliver = w.clipExercise < clipGetUpDeliver.size() && clipGetUpDeliver[w.clipExercise] > 0.0f
                ? clipGetUpDeliver[w.clipExercise] : references[w.clipReference]->durationSeconds();
            // (18h20, medido: no de costas a rede rola de brucos em ~0,6 s e o "virou para o outro lado" abria uma tentativa nova no trecho
            // de 1,4 s do clipe - o ritmo pela tentativa saltava para a frente e nao cobrava nada. O ritmo do comeco do levantar, no clipe
            // do comeco: as tentativas novas nao o reiniciam.)
            const float tau = std::min(deliver, w.clipStart + std::max(0.0f, w.time) * deliver / std::max(0.3f, settings.getUpPaceSeconds * getUpTimeScale(w)));
            const float at = std::clamp(tau * 30.0f, 0.0f, static_cast<float>(table.size() - 1));
            const std::size_t i0 = static_cast<std::size_t>(at), i1 = std::min(i0 + 1, table.size() - 1);
            const float target = table[i0].head + (table[i1].head - table[i0].head) * (at - static_cast<float>(i0));
            reward -= settings.getUpPaceWeight * std::clamp((now.headHeight - target - 0.05f) / 0.25f, 0.0f, 1.0f);
        }
        // (14/11 v1, settings.getUpImitationReward; ele: o levantar de costas "o mais semelhante possivel" do clipe) A nota da semelhanca
        // da pose do corpo com o clipe do COMECO no ritmo de getUpImitationPace s (o relogio do custo do ritmo), nos levantares limpos da
        // mascara, ate a troca. O clipe e alinhado ao rumo (cabeca - raiz) e a raiz do corpo a cada passo: mede a pose, nao o lugar.
        if ((settings.getUpImitationReward > 0.0f || settings.getUpImitationFar > 0.0f) && settings.getUpImitationMask != 0 && w.clipGetUp
            && w.clipHandedAt < 0.0f
            && w.fireKind == 0 && w.clipExercise < 31u && ((settings.getUpImitationMask >> w.clipExercise) & 1) != 0
            && w.imitationReference < references.size()) {
            const TrackingClipReference3D& reference = *references[w.imitationReference];
            const float deliver = w.clipExercise < clipGetUpDeliver.size() && clipGetUpDeliver[w.clipExercise] > 0.0f
                ? clipGetUpDeliver[w.clipExercise] : reference.durationSeconds();
            const float rate = deliver / std::max(0.3f, imitationPace(w));
            const float tau = std::min(deliver, w.imitationStart + std::max(0.0f, w.time) * rate);
            // (15/11 11h50) Nos correndo, depois do ponto de entrega o quadro do clipe fica parado e o corpo segue correndo ate a troca: a nota
            // e o fim cedo param ali (medido: o fim cedo cortava 16/16 do correndo de brucos ja de pe, na saida correndo).
            const bool runPast = clipRunExercise(w.clipExercise) && w.imitationStart + std::max(0.0f, w.time) * rate >= deliver;
            thread_local TrackingClipFrame3D pacedFrame;
            float yaw = w.clipYaw;
            Vec3 ground = w.clipGround;
            const std::uint32_t head = contract->headLink();
            const auto heading = [&](std::span<const PhysicsBodyState3D> links) {
                const Vec3 d = links[head].position - links[rootLink].position;
                return std::atan2(d.y, d.x);
            };
            const bool anchored = settings.getUpImitationAnchor && !clipRunExercise(w.clipExercise);
            if (anchored && w.imitationAnchored) {
                yaw = w.imitationYaw;
                ground = w.imitationGround;
            } else {
                reference.frame(tau, ground, yaw, pacedFrame);
                // (getUpImitationAnchor) Preso: o rumo e o do comeco (o corpo nasceu deste quadro), so a raiz vai para baixo da do corpo.
                if (!anchored) yaw += wrapAngle(heading(w.body->links) - heading(pacedFrame.links));
                reference.frame(tau, ground, yaw, pacedFrame);
                ground.x += w.body->links[rootLink].position.x - pacedFrame.links[rootLink].position.x;
                ground.y += w.body->links[rootLink].position.y - pacedFrame.links[rootLink].position.y;
                if (anchored) {
                    w.imitationYaw = yaw;
                    w.imitationGround = ground;
                    w.imitationAnchored = true;
                }
            }
            reference.frame(tau, ground, yaw, pacedFrame);
            for (PhysicsBodyState3D& link : pacedFrame.links) {
                link.linearVelocity = link.linearVelocity * rate;
                link.angularVelocity = link.angularVelocity * rate;
            }
            const float emphasisNow = w.clipExercise < clipGetUpEmphasis.size() ? clipGetUpEmphasis[w.clipExercise] : 0.0f;
            const TrackingClipGetUp3D::Terms terms = TrackingClipGetUp3D::score(*contract, *profile, w.body->links, pacedFrame,
                { settings.clipGetUpSoftness, settings.clipGetUpComVelocityWidth, settings.clipGetUpParts, emphasisNow }, ground.z);
            if (!runPast) reward += settings.getUpImitationReward * terms.total;
            // (14/11 v1, settings.getUpImitationFar: o fim cedo - longe do clipe no ritmo por getUpImitationFarSeconds, acaba como o clipFar.)
            w.imitationLowSeconds = !runPast && terms.total < settings.getUpImitationFar ? w.imitationLowSeconds + TicksPerDecision * TickSeconds : 0.0f;
        }
        // (04/11 15h30, o levantar que chega equilibrado: o premio da chegada e a multa do pulinho - os dois pes no ar quase de pe,
        // com a recuperacao no comando e sem empurrao.)
        reward += w.arrivalReward;
        // (15/11 08h30, settings.handBehindWeight - ver o .hpp) A mao travada atras das costas, de pe, em todos os exercicios.
        if (settings.handBehindWeight > 0.0f && rootLink < w.body->links.size()) {
            const PhysicsBodyState3D& pelvis = w.body->links[rootLink];
            const Quaternion frame = (pelvis.orientation * profile->links[rootLink].modelOrientation.conjugate()).normalized();
            const Vec3 forward = frame.rotate({ 1.0f, 0.0f, 0.0f }), left = frame.rotate({ 0.0f, 1.0f, 0.0f });
            const bool upright = now.headHeight >= 1.3f;
            float stuck = 0.0f;
            for (int k = 0; k < 2; ++k) {
                const std::uint32_t h = handLinks[k];
                bool inside = false;
                if (upright && h < w.body->links.size()) {
                    const Vec3 d = w.body->links[h].position - pelvis.position;
                    inside = d.x * forward.x + d.y * forward.y + d.z * forward.z < -0.10f
                        && std::abs(d.x * left.x + d.y * left.y + d.z * left.z) < 0.15f;
                }
                w.handBehindSeconds[k] = inside ? w.handBehindSeconds[k] + TicksPerDecision * TickSeconds : 0.0f;
                stuck += std::clamp((w.handBehindSeconds[k] - 0.3f) / 0.5f, 0.0f, 1.0f);
            }
            reward -= settings.handBehindWeight * stuck;
        }
        // (13/11 14h, settings.idlePelvisLevelWeight) O quadril reto no parado.
        if (settings.idlePelvisLevelWeight > 0.0f && w.locomotionControl && w.commandSpeed <= 0.0f && now.standing && !down) {
            const float roll = pelvisRoll(w.body->links) / 6.0f;
            reward -= settings.idlePelvisLevelWeight * (1.0f - std::exp(-roll * roll)) * (settings.reviewFixes ? 1.0f - fireFreedom(w) : 1.0f);
        }
        w.arrivalReward = 0.0f;
        if (settings.footSlipCost > 0.0f && w.locomotionControl && w.commandSpeed <= 0.0f) {
            // (10/11, settings.footSlipCost: os pes apoiados deslizando ou girando, parado - ver o .hpp.)
            const float still = std::clamp(1.0f - std::hypot(comVelocity.x, comVelocity.y) / 0.5f, 0.0f, 1.0f);
            float slip = 0.0f;
            for (int f = 0; f < 2; ++f) {
                const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == footLinks[f]; });
                if (!down) continue;
                const PhysicsBodyState3D& foot = w.body->links[footLinks[f]];
                slip += std::min(2.0f, std::hypot(foot.linearVelocity.x, foot.linearVelocity.y) / 0.05f + std::abs(foot.angularVelocity.z) / 0.5f);
            }
            reward -= settings.footSlipCost * still * slip * (1.0f - fireFreedom(w));
        }
        if (settings.jamCost > 0.0f && w.locomotionControl) {
            // (10/11, settings.jamCost: o torque dos batentes da coluna - ver o .hpp.)
            constexpr float JamSlack = 20.0f, JamScale = 100.0f;
            float jam = 0.0f;
            for (std::size_t i = 0; i < spineLinks.size() && i < w.body->joints.size(); ++i)
                if (spineLinks[i])
                    for (int a = 0; a < 3; ++a)
                        jam += std::max(0.0f, std::abs(w.body->joints[i].constraintTorqueNewtonMeters[a]) - JamSlack) / JamScale;
            reward -= settings.jamCost * jam;
        }
        if (basePostureOn(w)) {
            // (settings.basePosture: o potencial -d do idle alerta, F = 0,99 Phi(s') - Phi(s), na proporcao do quanto ja freou; e cada
            // pe que pousa custa um pouco - o ajuste com o minimo de passos. Ver TrackingBasePosture3D.)
            const float speed = std::hypot(comVelocity.x, comVelocity.y) / BasePostureNearSpeed;
            const float near = std::exp(-speed * speed);
            const float d = basePosture(w).distance();
            if (w.basePrevDistance >= 0.0f) reward += settings.basePosture * near * (w.basePrevDistance - 0.99f * d);
            w.basePrevDistance = d;
            for (int f = 0; f < 2; ++f) {
                const bool down = std::any_of(w.body->contacts.begin(), w.body->contacts.end(),
                    [&](const RagdollContactPoint3D& c) { return c.linkIndex == footLinks[f]; });
                if (down && !w.baseFootDown[f]) reward -= BasePostureStepCost * near;
                w.baseFootDown[f] = down;
            }
        } else {
            w.basePrevDistance = -1.0f;
        }
        // (04h55: um pe ou uma canela encostando na outra perna, de pe na cabeca da locomocao - ver LegClashPenalty.)
        if (w.locomotionControl && !w.getUpGuided && w.legGap < LegTouch) reward -= LegClashPenalty;
        // (12/11 16h30) A liberdade sob o fogo desta decisao na ultima coluna do retorno da locomocao (o juiz nao aprende com ela alta).
        w.locomotionFeedback[TrackingGetUpAmpTraining3D::TrackingFeedbackSize - 1] = fireFreedom(w);
        if (settings.flightPenalty > 0.0f && !w.locomotionControl && !w.locomotion && w.startKind != 4 && now.headHeight > 1.2f
            && w.pushTicks == 0) {
            bool foot = false;
            for (const RagdollInteraction3D& c : w.body->interactions)
                foot = foot || c.linkIndex == footLinks[0] || c.linkIndex == footLinks[1];
            if (!foot) reward -= settings.flightPenalty;
        }
        // (08/11 10h35, a inspecao: no levantar de brucos o corpo inteiro sai do chao - a flexao explosiva, as pernas
        // chutadas para cima, 0,2-0,3 s no ar - antes de por os pes; o "pulinho" que ele ve. Depois de tocar o chao e ainda
        // baixo (a cabeca ate 1,2 m; acima, a multa do voo de cima), nenhum elo encostando custa riseFlightPenalty.)
        if (settings.riseFlightPenalty > 0.0f && !w.locomotionControl && !w.locomotion && w.startKind != 4 && w.pushTicks == 0
            && w.touchedDown && now.headHeight <= 1.2f && w.body->interactions.empty())
            reward -= settings.riseFlightPenalty;
        // (07/11, ele: "tentar fazer o nosso levantar nao dar esse pulinho no final quando se levanta" - a sonda: nos 32
        // levantares deitados o centro de massa passava 4-9 cm da altura final subindo a 1,2-1,6 m/s, os dois pes no ar
        // 0,17-0,30 s, com a recuperacao no comando; os clipes da Unreal chegam de pe sem passar (0-1 cm). Perto do topo,
        // subir alem de RiseTopSpeed custa - ela aprende a frear a subida como eles.)
        if (settings.riseOvershoot > 0.0f && !w.locomotionControl && !w.locomotion && w.startKind != 4 && w.pushTicks == 0) {
            const float top = rlBodyCenterOfMass3D(*profile, w.body->links).z - floorUnder(w);
            const float vz = rlBodyCenterOfMassVelocity3D(*profile, w.body->links).z;
            if (top > RiseTopHeight && vz > RiseTopSpeed) reward -= settings.riseOvershoot * (vz - RiseTopSpeed) * (vz - RiseTopSpeed);
        }
        // (04/11 16h40: a subida por cima dos pes - ver riseBalancePenalty.)
        if (settings.riseBalancePenalty > 0.0f && !w.locomotionControl && !w.locomotion && w.startKind != 4
            && now.headHeight > RiseHeadHeight && w.pushTicks == 0) {
            const TrackingBalance3D b = measureTrackingBalance3D(*profile, w.body->links, w.body->interactions,
                { footLinks[0], footLinks[1] }, floorUnder(w));
            const float out = std::clamp(-b.margin / 0.30f, 0.0f, 1.0f);
            const float drift = std::clamp(std::hypot(b.velocity.x, b.velocity.y), 0.0f, 1.0f);
            reward -= settings.riseBalancePenalty * (0.5f * out + 0.5f * drift);
        }
        // (O rastreio: perder o equilibrio - o tronco no chao, a cabeca abaixo de 1,1 m ou a pelve abaixo de 0,5 m - com
        // a queda encerrando custa a multa da queda e encerra; o rastreio ruim sustentado tambem - 03/11 10h20: sem a multa,
        // frear e travar o corpo ate o rastreio acabar saia de graca e cair tentando custava 10, o atalho de parar.)
        const bool trackingNow = locomotionCommanded(w) && !freeNow;
        const bool lostBalance = (trackingNow || freeNow) && settings.locomotionTermination
            && (down || now.headHeight < LostHeadHeight || now.pelvisHeight < LostPelvisHeight);
        if (lostBalance && !down && w.stoodAt >= 0.0f) {
            reward -= !shaped ? 2.0f * FallPenalty : FallPenalty;
            w.fellAfterStanding = true;
        }
        // (12/11 18h, settings.commandGetUp) No comando ligado ao levantar a queda nao encerra: a multa dela fica na locomocao e a cabeca
        // do levantar assume (beginCommandGetUp); levantando, o chao nao encerra.
        // (Sem os clipes do levantar no lote - as avaliacoes, as reguas -, o comando cai como antes.)
        const bool commandRising = settings.commandGetUp && isCommand(w) && w.clipGetUp && w.clipHandedAt < 0.0f;
        const bool commandFall = settings.commandGetUp && isCommand(w) && !clipGetUpClips.empty() && !commandRising && w.stoodAt >= 0.0f
            && (lostBalance || (settings.locomotionTermination && down));
        if (commandFall) {
            // (13/11 v1: a multa da queda deitada so no bloco "down && !w.down", como acima.)
            w.fellAfterStanding = true;
            beginCommandGetUp(w, now);
        }
        // (13/11, settings.fallToCommand) A queda nos outros exercicios de locomocao: o episodio vira comando ali (a tecla do rumo e o
        // shift da faixa se andava; parado ou girando, sem tecla) e levanta como no comando - o resto do episodio e do jogador emulado.
        const bool convertedFall = settings.fallToCommand && settings.commandGetUp && !commandFall && !commandRising && !isCommand(w)
            && w.locomotion && !w.clipGetUp && !clipGetUpClips.empty() && w.stoodAt >= 0.0f
            && (lostBalance || (settings.locomotionTermination && down));
        if (convertedFall) {
            // (13/11 v1, a revisao: a multa da queda deitada fica so no bloco "down && !w.down" la embaixo, como no fluxo que encerrava -
            // aqui ela era cobrada duas vezes, -30 em vez de -20.)
            w.fellAfterStanding = true;
            w.fallConverted = true;
            w.originalManeuver = w.maneuver;
            w.convertedAt = w.time;
            const bool moving = w.startKind == 6 && w.commandSpeed > 0.0f;
            // (13/11 v1, a revisao: o episodio vira comando de verdade - startKind 6 -; com o 7 o giro seguia girando o alvo
            // (advanceTurn), o custo de quietude e a coluna do giro valiam nas linhas do comando e o virado usava o rumo do giro. O
            // registro e o fogo seguem no tipo de antes, originalKind.)
            w.originalKind = w.startKind;
            w.startKind = 6u;
            w.maneuver = CommandManeuver;
            startCommand(w);
            if (!moving) w.command.key = -1;
            beginCommandGetUp(w, now);
        }
        if (trackingNow) w.trackingAverage += (w.locomotionFeedback[0] - w.trackingAverage) * (TicksPerDecision * TickSeconds / 0.5f);
        const bool strayed = !(clipSequence || kinematicMotion) && trackingNow && settings.trackingTermination > 0.0f && w.time > 1.0f
            && w.trackingAverage < settings.trackingTermination;
        if (strayed && !lostBalance && w.stoodAt >= 0.0f) reward -= FallPenalty;
        // (13h40, a folha do lateral esquerdo: ele virava o corpo para o rumo e corria de frente - a nota caia, mas so a
        // media movel encerrava, ~1,9 s depois, e nao cair pagava mais que tentar. O rumo da pelve fora do clipe por mais de
        // 45 graus - 60 no giro parado - por 0,2 s encerra como a queda.)
        // (04/11, ele: "reiniciam bem rapido as vezes, sem haver queda" - 45 graus por 0,2 s cortava o trote de pe na saida
        // do alerta; agora 60 graus por 0,3 s em tudo: a meia-volta segue barrada.)
        // (13/11 v1, a revisao: levantando - o correndo tem o pedido ligado - o rumo nao e atualizado e o virado encerrava no meio do levantar.)
        w.turnedSeconds = trackingNow && !(w.clipGetUp && w.clipHandedAt < 0.0f) && std::abs(w.headingError) > TurnedAwayRadians
            ? w.turnedSeconds + TicksPerDecision * TickSeconds : 0.0f;
        // (12/11 16h30: no fogo da locomocao, com forca agindo ou ate 1,5 s depois, 1 s a mais para voltar o rumo.)
        const float turnedLimit = TurnedAwaySeconds + (w.locoFire && (w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f) ? 1.0f : 0.0f)
            + (settings.turnedGrace > 0.0f && w.time < w.referenceTurningUntil ? settings.turnedGrace : 0.0f);
        const bool turnedAway = !(clipSequence || kinematicMotion) && trackingNow && settings.locomotionTermination && w.turnedSeconds > turnedLimit;
        if (turnedAway && !lostBalance && !strayed && w.stoodAt >= 0.0f) reward -= FallPenalty;
        // (04/11 14h: nas curvas, longe do percurso por OffCourseSeconds encerra como a queda - "fora do rumo".)
        // (08/11: na estrela tambem - no girando com 1 m a mais, a tecla de 45 em 45 graus nao segue a linha -, menos na
        // passagem pela ponta e na meia-volta; e, a fase de fogo, nao durante um empurrao nem 1,5 s depois dele.)
        const bool starGrace = isStar(w) && (w.runTurn >= 0 || w.brakeTurnAt >= 0.0f || w.time < w.star.graceUntil);
        const bool pushedNow = w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f;
        const float courseLimit = settings.courseLimit + (w.maneuver == 10 ? 1.0f : 0.0f);
        w.offCourseSeconds = trackingNow && settings.courseLimit > 0.0f && (isCurve(w) || (isStar(w) && settings.starPathWeight <= 0.0f))
                && !starGrace && !pushedNow
                && w.courseError > courseLimit
            ? w.offCourseSeconds + TicksPerDecision * TickSeconds : 0.0f;
        // (08/11 20h05, ele: "precisamos melhorar a velocidade das locomocoes LATERAIS": o lateral direito saiu 30-40 graus para
        // tras do rumo - a pelve e o peito certos, as pernas empurrando na diagonal -, a 0,8 de 2,5 m/s; com o portao do rumo a
        // nota da velocidade e zero ali e quase nao inclina, e ele ficou preso. O encerramento antecipado, como o do lento: nos
        // retos, o deslocamento (a velocidade de um passo, courseVelocity) mais que courseAngleLimit fora do pedido por
        // StraySeconds - fora do empurrao, da largada e da parada - encerra como a queda.)
        bool strayedCourse = false;
        // (12/11 19h20: no levantar e ir o rumo e o do ponto, re-mirado a cada decisao - quem nao chega acaba pelo prazo; a sonda: 259 de
        // 262 que nao chegaram eram este fim.)
        if (settings.courseAngleLimit > 0.0f && trackingNow && w.maneuver == 0 && w.locomotion && !w.stopping && !starting(w)
            && w.time > 1.5f && !pushedNow && settings.locomotionTermination && !w.getUpGo) {
            const Vec3 want = wantedVelocity(w, 0.0f), cv = settings.courseSmoothing > 0.0f ? w.courseVelocity : w.smoothVelocity;
            const float a = std::hypot(want.x, want.y), b = std::hypot(cv.x, cv.y);
            const float angle = a > 0.3f && b > 0.3f
                ? std::abs(std::atan2(want.x * cv.y - want.y * cv.x, want.x * cv.x + want.y * cv.y)) : 0.0f;
            w.straySeconds = angle > settings.courseAngleLimit ? w.straySeconds + TicksPerDecision * TickSeconds : 0.0f;
            strayedCourse = w.straySeconds > StraySeconds;
        } else {
            w.straySeconds = 0.0f;
        }
        const bool offCourse = !(clipSequence || kinematicMotion) && trackingNow && settings.locomotionTermination
            && (w.offCourseSeconds > OffCourseSeconds || strayedCourse);
        if (offCourse && !lostBalance && !strayed && !turnedAway && w.stoodAt >= 0.0f) reward -= FallPenalty;
        if (down && !w.down && w.stoodAt >= 0.0f) {
            // (01/11 23h35: no levantar em movimento a queda depois de ficar de pe custava 10 e, de pe, so a velocidade ao
            // alvo pagava - correr logo depois de levantar quase sempre caia, e ele aprendeu a nao levantar: 27% -> 1% em
            // 20 min. Com o levantar guiado, ali 2.)
            reward -= !shaped ? 2.0f * FallPenalty
                : settings.getUpGuide && (w.startKind == 3 || w.startKind == 5) ? 0.2f * FallPenalty : FallPenalty;
            w.fellAfterStanding = true;
        }
        // (A janela da protecao: nos comecos que ja vem caindo - as quedas da locomocao e as quedas com protecao -, antes
        // de ficar de pe; no evitar a queda, do empurrao ate se recompor. 01/11, a rl_final: sem a parceira o evitar comeca
        // de pe, e o "antes de ficar de pe" nunca valia - a protecao, o apoio, a flexao e o toque no chao ficavam de fora.)
        const bool window = w.startKind == 4 ? w.avoidEnd >= 0.0f && w.recomposedAt < 0.0f
                                             : (w.startKind == 8 || w.protectiveStart) && w.stoodAt < 0.0f;
        // (O evitar a queda de verdade: a primeira vez do tronco no chao na janela.)
        if (down && !w.touchedDown && window) reward -= settings.touchDownPenalty;
        // (A protecao: nesses comecos, antes do tronco no chao e de ficar de pe, o primeiro apoio das maos, dos bracos ou
        // dos joelhos vale uma vez.)
        const bool protecting = window && !w.touchedDown && !down;
        if (protecting && !w.supported) {
            for (const RagdollContactPoint3D& contact : w.body->contacts)
                if (contact.linkIndex < supportContact.size() && supportContact[contact.linkIndex]) w.supported = true;
            if (w.supported) reward += settings.supportBonus;
        }
        w.protecting = protecting;
        // (A flexao: na protecao e ate FlexAfterTouchSeconds depois do primeiro toque do tronco.)
        if (down && !w.touchedDown && window) w.flexUntil = w.time + FlexAfterTouchSeconds;
        if (protecting || (w.flexUntil >= 0.0f && w.time <= w.flexUntil)) {
            const float flex = flexion(*w.body);
            reward += settings.flexReward * flex;
            w.flexSum += flex;
            ++w.flexSamples;
        }
        w.touchedDown = w.touchedDown || down;
        w.down = down;
        if (w.startKind == 4 && w.avoidEnd >= 0.0f && w.time >= w.avoidEnd && w.recomposedAt < 0.0f && now.standing
            && !w.fellAfterStanding && posture->idlePoseMatch(w.body->links) > RecomposedPose) {
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            if (std::hypot(v.x, v.y) < RecomposedSpeed) w.recomposedAt = w.time;
        }
        // (11/11 14h: no levantar pelo clipe, com a recuperacao no comando, a nota e so a do clipe - nenhum termo do levantar antigo
        // (o potencial da subida, o pulinho, o voo, o toque, o equilibrio da subida).)
        // (12/11 01h05: sem bonus na troca - o bonus da chegada (22h34) nao trouxe a pose do fim do correndo de volta e, com o clipe
        // longo e nada depois da entrega, ensinou um bote para a frente: de 1,57 a 1,83 s o tronco quase deitado, as maos a 72-89 cm.)
        // (12/11 03h20) Mais o premio da agilidade (TrackingClipGetUp3D::AgilityMaxRate): adiantar o relogio do clipe parecido com ele.
        if (clipActive && !adaptiveFor(w)) reward = clipScore + w.clipAgilityReward;
        (void)deliveredNow;
        w.rewardSum += reward;
        ++w.decisions;
        if (w.stoodAt < 0.0f && now.standing) w.stoodAt = w.time;
        terminated = 0;
        // A troca de rede ao ficar de pe (so no treino): de pe e firme - a pelve perto da altura de pe do perfil, a
        // pelve em pe e quase parado, como a entrega no jogo - por 0,5 s, depois do chao ou do empurrao do evitar.
        if (settings.handoffOnStand && !settings.evaluation && !w.clipGetUp) {
            const bool eligible = w.startKind == 4 ? w.avoidEnd >= 0.0f && w.time >= w.avoidEnd : w.stoodAt >= 0.0f;
            const Vec3 v = rlBodyCenterOfMassVelocity3D(*profile, w.body->links);
            // (01/11, ele: no levantar em movimento, "a troca do modelo de RL assim que ele comeca a ficar em pe" e a da
            // locomocao corre para o alvo: de pe por MovingHandoffSeconds, mesmo andando; nos outros, de pe e firme.)
            const bool moving = w.startKind == 3 || w.startKind == 5;
            const bool firm = moving ? now.standing && !w.fellAfterStanding
                : now.pelvisHeight > 0.85f * profile->standingRootHeightMeters && now.upright > 0.85f
                    && std::hypot(v.x, v.y) < 0.5f && !w.fellAfterStanding;
            w.firmSeconds = eligible && firm ? w.firmSeconds + TicksPerDecision * TickSeconds : 0.0f;
            if (w.firmSeconds >= (moving ? MovingHandoffSeconds : 0.5f)) w.handoffAt = w.time;
        }
        // Depois da troca, a parceira dirige parada (01/11): se ela cair no tempo dela, a troca foi ruim - a punicao e o
        // fim do episodio -; senao, o corte de sempre, com o valor do estado da troca.
        const bool partnered = w.handoffAt >= 0.0f && partner && !settings.evaluation;
        if (partnered) {
            observe(w);
            w.finalObservation = w.observation;
            // (No levantar em movimento, ela corre ate o alvo na velocidade pedida - ate PartnerRunSpeed, a do treino dela.)
            Vec3 velocity;
            if (w.startKind == 3 || w.startKind == 5) {
                const Vec3 root = w.body->links[rootLink].position;
                const Vec3 to { w.target.x - root.x, w.target.y - root.y, 0.0f };
                const float distance = std::hypot(to.x, to.y);
                if (distance > ArriveRadius)
                    velocity = to * (std::min(w.startCommandSpeed, PartnerRunSpeed) / distance);
            }
            const LocomotionPartner3D::Result result =
                partner->drive(*w.scene, w.handle, w.drives, settings.partnerSeconds, nullptr, velocity);
            w.body = w.scene->ragdollStateView(w.handle);
            w.partnerDrove = result.seconds;
            w.partnerFell = result.fell;
            if (result.fell) reward -= settings.partnerFallPenalty;
        }
        // (A rl_final: nos rumos e no giro, o tronco no chao encerra o episodio - o early termination do DeepMimic: cair
        // custa o resto do episodio - e o tempo deles e o locomotionSeconds.)
        // (07h40: o parado que perde a pose do idle encerra como a queda - a regra obrigatoria.)
        const bool idleLost = w.startKind == 2 && idleHeld(w) && w.idleLostSeconds >= IdleLostSeconds;
        // (09/11 19h50, settings.poseBoost) O parado dobrado encerra como a queda: no exercicio de equilibrio da avaliacao (sem empurrao)
        // o corpo passava o episodio inteiro dobrado na cintura - o tronco quase horizontal, a cabeca a ~1,2 m - com a cabeca da
        // locomocao e o idle de referencia (folhas build-profile/tmp/parado_0911); o portao da pose ainda pagava 40% e nada encerrava
        // (a cabeca acima de 0,9 m). Parado (no idle da locomocao ou na parada livre, abaixo de 0,5 m/s), fora de IdleRecoverSeconds
        // depois de um empurrao, o peito alto alem de IdleBentLimit do idle por IdleBentSeconds encerra com a multa da queda.
        static constexpr float IdleBentLimit = 0.60f, IdleBentSeconds = 0.5f;
        if ((settings.poseBoost > 0.0f || settings.idleBentEnd) && (idleHeld(w) || freeStopNow(w)) && w.locomotionControl
            && upperChestLink < w.body->links.size()
            && upperChestLink < w.idleFrame.links.size()) {
            const auto up = [&](const PhysicsBodyState3D& link) {
                return (link.orientation * profile->links[upperChestLink].modelOrientation.conjugate()).normalized().rotate({ 0.0f, 0.0f, 1.0f });
            };
            const float tilt = std::acos(std::clamp(dot(up(w.body->links[upperChestLink]), up(w.idleFrame.links[upperChestLink])), -1.0f, 1.0f));
            const bool recovering = w.pushTicks > 0 || w.time < w.lastPushEnd + IdleRecoverSeconds;
            const bool still = std::hypot(comVelocity.x, comVelocity.y) < 0.5f;
            w.bentSeconds = !recovering && still && tilt > IdleBentLimit ? w.bentSeconds + TicksPerDecision * TickSeconds : 0.0f;
        } else {
            w.bentSeconds = 0.0f;
        }
        const bool bentLost = w.bentSeconds >= IdleBentSeconds;
        if (bentLost && !idleLost) reward -= FallPenalty;
        if (idleLost) reward -= FallPenalty;
        // (08/11 11h40, ele: "trabalhe para aumentar as velocidades". A sonda da paisagem: com a mao puxando a pelve, a corrida
        // de frente a 4,2 m/s em vez de 3,2 pagava 0,39 por passo em vez de 0,23 - a nota ja paga correr, a estabilidade nao
        // cai, ninguem cai; a rede esta no otimo local devagar e seguro (com o ruido e os empurroes, correr arrisca a queda).
        // O encerramento antecipado do DeepMimic na velocidade: nos retos, depois da largada, andar no rumo abaixo de slowLimit da
        // velocidade de referencia por SlowSeconds encerra como a queda - fora da janela do empurrao.)
        bool tooSlow = false;
        if (settings.slowLimit > 0.0f && trackingNow && w.locomotion && w.startKind == 6 && w.maneuver == 0 && w.commandSpeed > 0.0f
            && !w.stopping
            && !starting(w) && w.time >= SlowAfterSeconds && !pushedNow && !(clipSequence || kinematicMotion)) {
            const Vec3 want = wantedVelocity(w, 0.0f);
            const float wantedNow = std::hypot(want.x, want.y);
            const float along = wantedNow > 0.3f ? (w.smoothVelocity.x * want.x + w.smoothVelocity.y * want.y) / wantedNow : wantedNow;
            // (12h25: o piso por grupo de rumo - a frente (0, 1, 7), os lados (2, 6), as costas (3, 4, 5) -, cada um no ritmo dele.)
            const std::uint32_t h = w.heading % 8u;
            const float floor = settings.slowLimits[(h == 2u || h == 6u ? 1u : (h >= 3u && h <= 5u) ? 2u : 0u) + (w.runBand ? 3u : 0u)];
            w.slowSeconds = wantedNow > 0.3f && along < floor * wantedNow ? w.slowSeconds + TicksPerDecision * TickSeconds : 0.0f;
            tooSlow = settings.locomotionTermination && w.slowSeconds > SlowSeconds;
        } else {
            w.slowSeconds = 0.0f;
        }
        // (11/11 14h: o levantar pelo clipe longe do clipe por FarSeconds, ou caido na locomocao depois da troca.)
        const bool clipFar = (w.clipGetUp && !adaptiveFor(w) && w.clipHandedAt < 0.0f && w.clipFarSeconds >= TrackingClipGetUp3D::FarSeconds)
            || (settings.getUpImitationFar > 0.0f && w.clipGetUp && w.clipHandedAt < 0.0f && w.imitationLowSeconds >= settings.getUpImitationFarSeconds);
        // (12/11 04h40) O orcamento do levantar adaptativo (AdaptiveSpareSeconds): sem ficar firme ate ali, acaba - nao levantou.
        const float adaptiveDeliver = w.clipGetUp && w.clipExercise < clipGetUpDeliver.size() && clipGetUpDeliver[w.clipExercise] > 0.0f
            ? clipGetUpDeliver[w.clipExercise] : w.clipGetUp ? references[w.clipReference]->durationSeconds() : 0.0f;
        const bool adaptiveOver = w.clipGetUp && adaptiveFor(w) && w.clipHandedAt < 0.0f
            && w.time - w.attemptFrom >= adaptiveDeliver + AdaptiveSpareSeconds - 1.0e-4f;
        const bool clipFell = w.clipGetUp && w.clipHandedAt >= 0.0f && down;
        // (13/11, settings.fallToCommand: no instante da queda que virou comando nada encerra - o que a queda trouxe junto, o rastreio
        // ruim, o rumo ou o percurso, e da locomocao que acabou ali.)
        const bool fellMoving = !convertedFall && (clipFar || clipFell
            || (((settings.locomotionTermination && (w.locomotion || w.taught) && down) || lostBalance) && !commandFall && !commandRising
                && !convertedFall)
            || strayed || turnedAway
            || idleLost || bentLost || offCourse || tooSlow);
        const float limit = episodeLimit(w);
        terminated = w.partnerFell || fellMoving ? 1 : 0;
        // (12/11 09h00) Fora do mundo (caiu pela borda do chao - o continuo acima do atrito arrastava o corpo deitado 60 m): acaba.
        const bool offWorld = !w.body->links.empty() && w.body->links[rootLink].position.z < -1.0f;
        truncated = !terminated && (w.time >= limit - 1e-4f || w.handoffAt >= 0.0f || adaptiveOver || offWorld
                                       || (w.clipGetUp && w.clipHandedAt >= 0.0f
                                           && (w.getUpGo ? w.goArrivedAt >= 0.0f || w.time >= w.clipHandedAt + goBudget(w) - 1e-4f
                                               : w.time >= w.clipHandedAt + (settings.runAfterSeconds >= 0.0f && clipRunExercise(w.clipExercise)
                                                                              ? settings.runAfterSeconds : settings.clipGetUpAfterSeconds) - 1e-4f)))
            ? 1 : 0;
        if (truncated || terminated) {
            if (!partnered) {
                observe(w);
                w.finalObservation = w.observation;
            }
            TrackingGetUpAmpEpisode3D episode;
            episode.world = static_cast<std::uint32_t>(index);
            episode.endCause = !terminated ? 0u : down ? 1u : lostBalance ? 2u : turnedAway ? 3u : strayed || idleLost ? 4u
                : offCourse ? 6u : tooSlow ? 7u : clipFar ? 8u : 5u;
            episode.startKind = w.fallConverted ? w.originalKind : w.startKind;
            episode.startSide = w.startSide;
            episode.startHeadHeight = w.startHead;
            episode.seconds = w.time;
            episode.firstStandSeconds = w.stoodAt;
            episode.meanTaskReward = static_cast<float>(w.rewardSum / std::max<std::uint32_t>(1, w.decisions));
            episode.headHeight = now.headHeight;
            episode.pelvisHeight = now.pelvisHeight;
            episode.standing = now.standing;
            // (04/11 07h45, ele: "o quanto % ele acompanha o pedido": no rastreio, a media do que a referencia pede no rumo
            // nos mesmos instantes - a razao com a coluna 12 e o % do pedido; antes, o pedido do comeco, e a arrancada e as
            // frenagens das trocas contavam como lentidao.)
            episode.commandSpeed = w.locomotion && w.towardSamples > 0 && w.startCommandSpeed > 0.0f
                ? std::max(0.05f, static_cast<float>(w.wantedSum / w.towardSamples)) : w.startCommandSpeed;
            episode.reachedSeconds = w.reachedAt;
            episode.meanTowardSpeed = w.towardSamples ? static_cast<float>(w.towardSum / w.towardSamples) : 0.0f;
            episode.meanImitation = w.trackSamples ? static_cast<float>(w.imitationSum / w.trackSamples) : -1.0f;
            // (v13: a do episodio e 1 - a raiz da media do quadrado da falta - os tropecos raros pesam, a media os diluia.)
            episode.meanStability = !w.stabilitySamples ? -1.0f
                : 1.0f - static_cast<float>(std::sqrt(w.stabilityDeficitSum / w.stabilitySamples));
            episode.fellAfterStanding = w.fellAfterStanding;
            episode.recomposedSeconds = w.recomposedAt >= 0.0f ? w.recomposedAt - w.avoidEnd : -1.0f;
            episode.endPoseMatch = now.standing ? posture->idlePoseMatch(w.body->links) : 0.0f;
            episode.endSpineMatch = now.standing ? posture->idleSpineMatch(w.body->links) : 0.0f;
            episode.handoffSeconds = w.handoffAt;
            episode.partnerFell = w.partnerFell;
            episode.partnerSeconds = w.partnerDrove;
            episode.touchedDown = w.touchedDown;
            episode.supported = w.supported;
            episode.meanFlex = w.flexSamples ? static_cast<float>(w.flexSum / w.flexSamples) : 0.0f;
            episode.obstructed = w.obstructed;
            episode.band = w.startBand;
            // (04/11: nos rumos, a manobra do episodio - 0 nenhuma - no lugar do cenario.)
            episode.sceneryPreset = (w.fallConverted ? w.originalKind : w.startKind) == 6 ? (w.fallConverted ? w.originalManeuver : w.maneuver)
                                                                                          : w.controlledPreset;
            episode.meanFaceError = w.faceErrorSamples
                ? static_cast<float>(w.faceErrorSum / w.faceErrorSamples) * 180.0f / Pi : 0.0f;
            episode.teacherShare = static_cast<float>(w.teacherDecisions) / static_cast<float>(std::max<std::uint32_t>(1, w.decisions));
            episode.takeovers = w.takeovers;
            episode.labeled = w.labeled;
            episode.teacherAssist = w.teacherAssist;
            if (w.locomotion && w.travelSum.lengthSquared() > 1.0e-6f)
                episode.travelError = wrapAngle(std::atan2(w.travelSum.y, w.travelSum.x) - w.moveYaw) * 180.0f / Pi;
            episode.recoverySteps = w.recoveryStepCount;
            episode.settleSteps = w.settleStepCount;
            episode.balanceDrill = w.balanceDrill;
            episode.pushes = w.pushes;
            episode.worstCaptureOut = w.worstCaptureOut;
            episode.quietAgainSeconds = w.quietAgainAt;
            episode.arrivalQuality = w.arrivalQuality;
            episode.footCross = w.crossStance ? static_cast<float>(w.crossBad) / static_cast<float>(w.crossStance) : -1.0f;
            // (13/11 v1, a revisao: a estrela que caiu e virou comando grava as pernas e a fase dela, nao os eventos do comando.)
            const bool starEpisode = isStar(w) || (w.fallConverted && w.originalManeuver >= 8 && w.originalManeuver <= 10);
            episode.starLegs = starEpisode ? static_cast<std::uint32_t>(w.star.legs)
                : isCommand(w) ? static_cast<std::uint32_t>(w.command.events) : 0u;
            episode.starPhase = starEpisode ? w.star.phase : isCommand(w) ? w.command.phase : 0;
            if (starEpisode) {
                episode.starPathError = w.star.pathSamples > 0 ? static_cast<float>(w.star.pathErrorSum / w.star.pathSamples) : -1.0f;
                episode.starCurves = static_cast<float>(w.star.curves);
            }
            episode.fellInPush = terminated && w.pushes > 0 && (w.pushTicks > 0 || w.time <= w.lastPushEnd + 2.0f);
            episode.stopError = w.stopping ? w.stopError : -1.0f;
            if (isCommand(w)) {
                episode.commandFalls = static_cast<float>(w.command.falls);
                episode.commandRises = static_cast<float>(w.command.rises);
                episode.commandRiseSeconds = w.command.rises > 0 ? w.command.riseSum / static_cast<float>(w.command.rises) : -1.0f;
                episode.commandRunRises = static_cast<float>(w.command.runRises);
            }
            if (w.clipGetUp && !isCommand(w)) {
                // (11/11 14h, o levantar pelo clipe - ver TrackingClipGetUp3D: o exercicio no lugar do cenario; a troca e a locomocao
                // depois dela nas colunas da parceira; a velocidade do centro de massa na troca na do rumo; o voo de pe na da chegada;
                // a nota media do clipe na da imitacao.)
                episode.sceneryPreset = static_cast<decltype(episode.sceneryPreset)>(w.clipExercise);
                // (12/11 00h35) Com a entrega no ponto, a troca da estatistica e ela, e nada depois conta como queda.
                const float handed = w.clipDeliveredAt >= 0.0f ? w.clipDeliveredAt : w.clipHandedAt;
                episode.handoffSeconds = handed;
                episode.partnerFell = w.clipDeliveredAt < 0.0f && w.clipHandedAt >= 0.0f && terminated;
                episode.partnerSeconds = handed >= 0.0f ? w.time - handed : 0.0f;
                episode.meanTowardSpeed = w.clipHandSpeed;
                episode.arrivalQuality = w.clipFlightSeconds;
                episode.getUpCovered = w.getUpCovered;
                episode.getUpTarget = w.getUpTarget;
                episode.fireKind = static_cast<float>(w.fireKind);
                episode.fireTiming = static_cast<float>(w.fireTiming);
                episode.fireDirection = static_cast<float>(w.fireDirection);
                episode.fireFraction = w.fireFraction;
                episode.fireSize = w.fireKind == 3 ? w.fireShoveSize : w.fireSize;
                // (12/11 07h50: o "recuperou" contava quem levantou e caiu depois - medido: a recuperacao de verdade era ~0%.) 0 sem ir ao
                // chao; 1 foi ao chao e nao levantou depois; 2 foi ao chao e levantou depois (o firme comecou depois da queda); 3
                // levantou e foi ao chao depois.
                episode.fireDown = w.fireDownAt < 0.0f ? 0.0f : w.clipDeliveredAt < 0.0f ? 1.0f : w.upAt > w.fireDownAt ? 2.0f : 3.0f;
                episode.upSeconds = w.upAt;
                episode.clipStartSeconds = w.clipStart;
                episode.attempts = static_cast<float>(w.attempts);
                episode.retrySeconds = w.attempts > 1 && w.clipDeliveredAt >= 0.0f && w.upAt >= 0.0f ? w.upAt - w.attemptFrom : -1.0f;
                episode.fireExtras = static_cast<float>(w.fireExtras);
                if (w.getUpGo) {
                    episode.startKind = 11u;
                    episode.getUpGo = clipRunExercise(w.clipExercise) ? 2.0f : 1.0f;
                    episode.goArrivedAt = w.goArrivedAt;
                    episode.goDistance = w.goDistance;
                    episode.goLeft = std::hypot(w.goTarget.x - w.body->links[rootLink].position.x, w.goTarget.y - w.body->links[rootLink].position.y);
                }
            }
            if (w.crooked) {
                episode.crooked = 1.0f;
                episode.crookedFixedAt = w.crookedFixedAt;
            }
            if (w.locoFire || w.locoKeepSamples > 0) {
                // (12/11 16h30, o fogo da locomocao - ver TrackingGetUpAmpEpisode3D::locoFire; o seguir a tarefa tambem no limpo.)
                episode.locoFire = w.locoFire ? 1.0f : 0.0f;
                episode.locoFireLevel = w.locoFireLevel;
                episode.locoFireEvents = static_cast<float>(w.locoFireEvents);
                episode.locoFireKeep = w.locoKeepSamples > 0 ? static_cast<float>(w.locoKeep) / static_cast<float>(w.locoKeepSamples) : -1.0f;
                if (w.locoFire) {
                    episode.fireExtras = static_cast<float>(w.fireExtras);
                    episode.fireKind = static_cast<float>(w.fireKind);
                }
            }
            {
                std::lock_guard lock(episodeMutex);
                episodes.push_back(episode);
            }
            const auto finalFeedback = w.locomotionFeedback;
            begin(w);
            if (settings.finalRow) w.locomotionFeedback = finalFeedback;
        }
        observe(w);
    }
};

TrackingGetUpAmpTraining3D::TrackingGetUpAmpTraining3D(const TrackingGetUpAmpSettings3D& settings)
    : m_impl(std::make_unique<Impl>()) {
    Impl& impl = *m_impl;
    impl.settings = settings;
    impl.bundle = AthleteBundle3D::shared(settings.assetsDirectory);
    impl.profile = &impl.bundle->profile();
    std::vector<std::string> names(std::begin(GetUpClips), std::end(GetUpClips));
    names.emplace_back(StandingClip);
    impl.standingIndex = names.size() - 1;
    names.emplace_back(StandingMirrorClip);
    impl.standingMirrorIndex = names.size() - 1;
    // (O levantar v2: os clipes da arrancada e da corrida so entram com o levantar em movimento ligado - sem ele, o
    // estilo e as quedas de antes.)
    if (settings.movingFraction > 0.0f || settings.sprintStartFraction > 0.0f || settings.evaluationKind == 1)
        names.insert(names.end(), std::begin(MovingClips), std::end(MovingClips));
    // (12/11 19h40, ele no laboratorio: "nitidamente ha um problema na troca de brain, ele nao troca para o da locomocao, parece estar
    // tentando andar pelo brain do levantar" - nas avaliacoes do levantar pelo clipe (11+) as marchas nao eram carregadas: a troca do
    // correndo nao achava o trote (gaitFor nulo) e a cabeca do levantar seguia correndo ate o fim; no treino, com a locomocao, ia certo.)
    const bool clipGetUpEval = settings.evaluationKind >= 11;
    if (settings.locomotionFraction > 0.0f || settings.turnFraction > 0.0f || settings.evaluationKind == 4
        || settings.evaluationKind == 6 || clipGetUpEval) {
        names.insert(names.end(), std::begin(HeadingClips), std::end(HeadingClips));
        names.insert(names.end(), std::begin(LocomotionStyleClips), std::end(LocomotionStyleClips));
    }
    // (A rl_final: as quedas com protecao com o evitar a queda ou as quedas da locomocao ligados, no treino - 02/11 00h45:
    // so com a queda programada, que assume esses comecos ja caindo; sem ela, a rede se debatia na queda.)
    if (!settings.evaluation && settings.protectiveFall
        && (settings.avoidFraction > 0.0f || settings.locomotionFallFraction > 0.0f))
        names.insert(names.end(), std::begin(ProtectionClips), std::end(ProtectionClips));
    // (A rl_final: a corrida e o sprint so com as fatias deles - ou a avaliacao dessas faixas -, cada clipe uma vez.)
    if (settings.runFraction > 0.0f || settings.evaluationBand == 2 || clipGetUpEval) {
        names.insert(names.end(), std::begin(TrackedRunClips), std::end(TrackedRunClips));
    }
    // (05/11: o sprint para a frente - so o sprint_maximo, rastreado.)
    if (settings.sprintFraction > 0.0f || settings.evaluationBand == 3)
        names.emplace_back("sprint_maximo");
    // (04/11 08h50, ele: "nos exercicios de correr para frente ele da uma patinada no comeco, precisa aprender a pegar no
    // tranco... temos clipe exatamente para isso"): no rastreio a arrancada idle_to_sprint e a largada de pe do trote e
    // da corrida para a frente e nas diagonais da frente - so rastreada, fora do juiz da recuperacao (sem o levantar em
    // movimento ela nem era carregada: a largada era o ciclo na rampa).
    if (settings.locomotionFraction > 0.0f || settings.evaluationKind == 4) {
        // (08/11: os ciclos na meta, para o setGaitCycles.)
        names.insert(names.end(), std::begin(CycleClips), std::end(CycleClips));
        names.emplace_back("idle_to_sprint");
        // (23h55: a meia-volta das trocas livres, so rastreada.)
        names.emplace_back("running_turn_180");
        names.emplace_back("running_turn_180_esquerda");
    }
    // (04/11 09h50, ele: "o giro parado deveria ser feito do IDLE alerta, e ja temos os movimentos de giro do footplanner,
    // que ele DEVERIA usar como referencia, e nao esse atual inventado": no rastreio o giro segue o do modo padrao - o idle
    // alerta com os passos do StandTurnFeet3D, capturado do lider (tools/animation/clips/giro_idle.py) -; os da Mixamo
    // ficam so no juiz da recuperacao, como antes.)
    if (settings.turnFraction > 0.0f || settings.evaluationKind == 4 || settings.evaluationKind == 6) {
        names.emplace_back(TrackedTurnClips[0]);
        names.emplace_back(TrackedTurnClips[1]);
    } else if (settings.runFraction > 0.0f || settings.sprintFraction > 0.0f || settings.evaluationBand >= 2) {
        names.insert(names.end(), std::begin(RunClips), std::end(RunClips));
        names.insert(names.end(), std::begin(SprintClips), std::end(SprintClips));
        names.emplace_back("idle_to_sprint");
    }
    for (std::size_t i = 0; i < names.size(); ++i)
        if (std::find(names.begin(), names.begin() + static_cast<std::ptrdiff_t>(i), names[i])
            != names.begin() + static_cast<std::ptrdiff_t>(i))
            names.erase(names.begin() + static_cast<std::ptrdiff_t>(i--));
    // (05/11 23h20) A arrancada agachada da saida da meia-volta (turnLaunch): no fim da lista - nenhum indice muda -, so
    // quando o levantar em movimento nao a carrega; ai so rastreada (fora do juiz e da largada agachada).
    const bool movingClips = settings.movingFraction > 0.0f || settings.sprintStartFraction > 0.0f || settings.evaluationKind == 1;
    if (std::find(names.begin(), names.end(), "crouched_to_sprint") == names.end())
        names.emplace_back("crouched_to_sprint");
    for (const std::string& name : names) {
        impl.clips.push_back(std::make_unique<AnimationClip3D>(
            loadAnimationClip3D(settings.assetsDirectory + "/animations/clips/" + name + ".matteranim.json")));
        impl.references.push_back(std::make_unique<TrackingClipReference3D>(*impl.profile, *impl.clips.back()));
        impl.clipNames.push_back(name);
        if (name == "idle_to_sprint") {
            impl.startClipIndex = impl.references.size() - 1;
            impl.startClipTrackedOnly = settings.movingFraction <= 0.0f
                && settings.sprintStartFraction <= 0.0f && settings.evaluationKind != 1;
        }
        if (name == "turn_left" && impl.turnIndices[0] >= impl.references.size() - 1) impl.turnIndices[0] = impl.references.size() - 1;
        if (name == TrackedTurnClips[0]) impl.turnIndices[0] = impl.references.size() - 1;
        if (name == TrackedTurnClips[1]) impl.turnIndices[1] = impl.references.size() - 1;
        if (name == "run_backward_trote") impl.runBackIndex = impl.references.size() - 1;
        if (name == "running_turn_180") impl.runTurnIndices[0] = impl.references.size() - 1;
        if (name == "running_turn_180_esquerda") impl.runTurnIndices[1] = impl.references.size() - 1;
        if (name == "jog_backward") impl.jogBackIndex = impl.references.size() - 1;
        if (name == "turn_right" && impl.turnIndices[1] >= impl.references.size() - 1) impl.turnIndices[1] = impl.references.size() - 1;
        // (07/11: com o levantar correndo ligado - o --movimento - a arrancada da meia-volta das trocas continua; o uso confere
        // o turnLaunch.)
        if (name == "crouched_to_sprint") {
            if (movingClips) impl.sprintStartIndex = impl.references.size() - 1;
            impl.launchExitIndex = impl.references.size() - 1;
        }
        if (name == "stand_up_back_kimodo" || name == "stand_up_back_kimodo_espelho")
            impl.rollIndices.push_back(impl.references.size() - 1);
        if (std::find(std::begin(ProtectionClips), std::end(ProtectionClips), name) != std::end(ProtectionClips))
            impl.protectionIndices.push_back(impl.references.size() - 1);
        if (movingClips && std::find(std::begin(MovingClips), std::end(MovingClips), name) != std::end(MovingClips))
            impl.runStartIndices.push_back(impl.references.size() - 1);
        const bool heading = std::find(std::begin(HeadingClips), std::end(HeadingClips), name) != std::end(HeadingClips);
        Impl::Travel travel;
        const AnimationClip3D& clip = *impl.clips.back();
        travel.direction = clip.travelDirectionRadians;
        if (name == "idle_to_sprint" || name == "crouched_to_sprint")
            travel.rampDisplacement = std::hypot(clip.sourceRootDisplacementMeters.x,
            clip.sourceRootDisplacementMeters.y);
        else if (name == "mixamo_running" || name == "run_forward_medium" || name == "sprint_forward" || heading
            || std::find(std::begin(LocomotionStyleClips), std::end(LocomotionStyleClips), name)
                != std::end(LocomotionStyleClips)
            || std::find(std::begin(RunClips), std::end(RunClips), name) != std::end(RunClips)
            || std::find(std::begin(TrackedRunClips), std::end(TrackedRunClips), name) != std::end(TrackedRunClips)
            || std::find(std::begin(SprintClips), std::end(SprintClips), name) != std::end(SprintClips)
            || std::find(std::begin(CycleClips), std::end(CycleClips), name) != std::end(CycleClips))
            travel.speed = clip.loops ? clip.nominalSpeedMetersPerSecond : 0.0f;
        const auto land = [&](const AnimationScalarTrack3D& contact, float& out) {
            if (contact.keyframes.empty() || clip.durationSeconds <= 0.0f) return;
            float previous = sampleAnimationScalarTrack3D(contact, 0.0f, clip.durationSeconds, true);
            for (int k = 1; k <= 200; ++k) {
                const float t = clip.durationSeconds * static_cast<float>(k) / 200.0f;
                const float value = sampleAnimationScalarTrack3D(contact, t, clip.durationSeconds, true);
                if (previous < 0.5f && value >= 0.5f) {
                    out = static_cast<float>(k) / 200.0f;
                    break;
                }
                previous = value;
            }
        };
        land(clip.leftFootContact, travel.leftLand);
        land(clip.rightFootContact, travel.rightLand);
        impl.travel.push_back(travel);
        for (const GaitEntry& entry : GaitTable)
            if (name == entry.clip && travel.speed > 0.0f)
                impl.gaits[entry.band].push_back({ impl.references.size() - 1, entry.heading });
        for (std::uint32_t k = 0; k < 8; ++k)
            if (name == CycleClips[k] && travel.speed > 0.0f) {
                impl.cycleGaits[k < 4 ? 1u : 2u].push_back({ impl.references.size() - 1, 2u * (k % 4u) });
                if (k == 2) impl.cycleJogBack = impl.references.size() - 1;
                if (k == 6) impl.cycleRunBack = impl.references.size() - 1;
            }
    }
    // (Os passos de cada faixa na ordem da GaitTable: o primeiro e o da avaliacao, para a frente.)
    for (std::vector<Impl::Gait>& band : impl.gaits) {
        std::stable_sort(band.begin(), band.end(), [&](const Impl::Gait& a, const Impl::Gait& b) {
            const auto rank = [&](const Impl::Gait& g) {
                const std::string& clip = names[g.clip];
                for (std::size_t k = 0; k < std::size(GaitTable); ++k)
                    if (clip == GaitTable[k].clip) return k;
                return std::size(GaitTable);
            };
            return rank(a) < rank(b);
        });
    }
    impl.contract = std::make_unique<TrackingGetUpContract3D>(*impl.profile);
    {
        static const char* const Axes[3] = { "twist", "swing1", "swing2" };
        const std::vector<std::string>& names = impl.contract->actionNames();
        impl.spineLinks.assign(impl.profile->links.size(), false);
        impl.armLinks.assign(impl.profile->links.size(), false);
        for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
            const std::string& id = impl.profile->links[i].id;
            impl.spineLinks[i] = id == "Abdomen" || id == "Chest" || id == "UpperChest";
            impl.armLinks[i] = id.find("UpperArm") != std::string::npos || id.find("Forearm") != std::string::npos || id.find("Hand") != std::string::npos;
        }
        for (const RagdollJointCoupling3D& coupling : impl.profile->jointCouplings)
            for (const std::uint32_t link : coupling.links) {
                const std::string name = impl.profile->links[link].id + "." + Axes[static_cast<std::size_t>(coupling.axis)];
                const auto found = std::find(names.begin(), names.end(), name);
                if (found != names.end()) impl.coupledAxes.push_back(static_cast<std::size_t>(found - names.begin()));
            }
    }
    for (const std::string& name : impl.contract->actionNames()) {
        const std::string link = name.substr(0, name.find('.'));
        impl.legAxes.push_back(link.find("Thigh") != std::string::npos || link.find("Shin") != std::string::npos
            || link.find("Foot") != std::string::npos || link.find("Toe") != std::string::npos);
        impl.armAxes.push_back(link.find("Arm") != std::string::npos || link.find("arm") != std::string::npos
            || link.find("Hand") != std::string::npos);
        impl.headAxes.push_back(link.find("Neck") != std::string::npos || link.find("Head") != std::string::npos);
    }
    {
        // (17/11, settings.clipVelocityRotation) As juntas de varios eixos, pelos nomes da acao ("Elo.eixo").
        constexpr std::size_t None = std::numeric_limits<std::size_t>::max();
        const std::vector<std::string>& names = impl.contract->actionNames();
        std::vector<std::string> links;
        std::vector<Impl::ActionJoint> joints;
        for (std::size_t k = 0; k < names.size(); ++k) {
            const std::size_t dot = names[k].find('.');
            if (dot == std::string::npos) continue;
            const std::string link = names[k].substr(0, dot), axis = names[k].substr(dot + 1);
            const int a = axis == "twist" ? 0 : axis == "swing1" ? 1 : axis == "swing2" ? 2 : -1;
            if (a < 0) continue;
            auto found = std::find(links.begin(), links.end(), link);
            if (found == links.end()) {
                links.push_back(link);
                joints.push_back({});
                joints.back().index.fill(None);
                found = links.end() - 1;
            }
            Impl::ActionJoint& j = joints[static_cast<std::size_t>(found - links.begin())];
            j.index[static_cast<std::size_t>(a)] = k;
            j.arm = j.arm || impl.armAxes[k];
            j.head = j.head || impl.headAxes[k];
        }
        for (const Impl::ActionJoint& j : joints)
            if (std::count_if(j.index.begin(), j.index.end(), [&](std::size_t k) { return k != None; }) >= 2) impl.actionJoints.push_back(j);
    }
    {
        const std::vector<float> ranges = impl.contract->actionRanges();
        for (std::size_t k = 0; k + 1 < ranges.size(); k += 2) {
            impl.actionMid.push_back(0.5f * (ranges[k] + ranges[k + 1]));
            impl.actionHalf.push_back(std::max(1.0e-3f, 0.5f * (ranges[k + 1] - ranges[k])));
        }
    }
    impl.legs = resolveAthleteLegs3D(*impl.profile);
    {
        // (O ganho por parte, N.m/rad a 100%; o amortecimento 1/10 dele; o teto 0,55 dele: os 4 de cima primeiro.)
        struct Part { const char* id; float gain; };
        static constexpr Part Parts[] { { "UpperChest", 150.0f }, { "Head", 15.0f }, { "LeftUpperArm", 20.0f }, { "RightUpperArm", 20.0f },
            { "Abdomen", 150.0f }, { "Chest", 150.0f }, { "Neck", 20.0f }, { "LeftForearm", 8.0f }, { "RightForearm", 8.0f },
            { "LeftHand", 2.0f }, { "RightHand", 2.0f }, { "LeftThigh", 120.0f }, { "RightThigh", 120.0f }, { "LeftShin", 60.0f },
            { "RightShin", 60.0f }, { "LeftFoot", 15.0f }, { "RightFoot", 15.0f } };
        for (const Part& part : Parts)
            for (std::size_t i = 0; i < impl.profile->links.size(); ++i)
                if (impl.profile->links[i].id == part.id) {
                    impl.assistLinks.push_back(static_cast<std::uint32_t>(i));
                    impl.assistGain.push_back(part.gain);
                    impl.assistDamping.push_back(0.1f * part.gain);
                    impl.assistCap.push_back(0.55f * part.gain);
                    if (std::string(part.id) == "RightUpperArm") impl.upperHandCount = impl.assistLinks.size();
                }
    }
    impl.footGeometry = resolveAthleteFootGeometry3D(*impl.profile, impl.legs, 0.08f);
    impl.taskClipCount = impl.references.size();
    impl.features = std::make_unique<TrackingAmpFeatures3D>(*impl.profile);
    impl.posture = std::make_unique<TrackingGetUpPostureProbe3D>(*impl.profile);
    {
        TrackingClipFrame3D idle;
        impl.references[impl.standingIndex]->frame(0.0f, {}, 0.0f, idle);
        impl.posture->setIdlePose(idle.links);
        impl.references[impl.standingMirrorIndex]->frame(0.0f, {}, 0.0f, idle);
        impl.posture->addIdlePose(idle.links);
    }
    for (const auto& reference : impl.references) {
        TrackingClipFrame3D frame;
        float x = 0.0f, y = 0.0f;
        const std::uint32_t root = impl.contract->rootLink();
        for (int k = 0; k < 32; ++k) {
            reference->frame(reference->durationSeconds() * static_cast<float>(k) / 32.0f, {}, 0.0f, frame);
            const float yaw = trackingBodyYaw3D(impl.profile->links[root], frame.links[root]);
            x += std::cos(yaw);
            y += std::sin(yaw);
        }
        impl.yawOffsets.push_back(std::atan2(y, x));
    }
    // (O agachado de cada clipe de levantar: o quadro, de 0,05 em 0,05 s entre 30% e 95% dele, de juntas e baixo da pelve
    // mais perto do primeiro da arrancada agachada.)
    if (impl.sprintStartIndex < impl.references.size()) {
        TrackingClipFrame3D start, frame;
        impl.references[impl.sprintStartIndex]->frame(0.0f, {}, 0.0f, start);
        RagdollState3D pose;
        pose.links = start.links;
        pose.joints.resize(start.coordinates.size());
        for (std::size_t i = 0; i < start.coordinates.size(); ++i)
            pose.joints[i].positionRadians = { start.coordinates[i].x, start.coordinates[i].y, start.coordinates[i].z };
        for (std::size_t c = 0; c < std::size(GetUpClips) && c < impl.references.size(); ++c) {
            const float duration = impl.references[c]->durationSeconds();
            float best = std::numeric_limits<float>::max(), bestTime = 0.65f * duration;
            for (float t = 0.3f * duration; t <= 0.95f * duration; t += 0.05f) {
                impl.references[c]->frame(t, {}, 0.0f, frame);
                const float d = impl.poseDistance(pose, frame);
                if (d < best) {
                    best = d;
                    bestTime = t;
                }
            }
            impl.crouchTimes.push_back(bestTime);
        }
    }
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
        if (impl.profile->links[i].parentIndex < 0) impl.rootLink = static_cast<std::uint32_t>(i);
        if (impl.profile->links[i].id == "LeftUpperArm") impl.leftShoulderLink = static_cast<std::uint32_t>(i);
        if (impl.profile->links[i].id == "LeftThigh") impl.leftThighLink = static_cast<std::uint32_t>(i);
        if (impl.profile->links[i].id == "RightThigh") impl.rightThighLink = static_cast<std::uint32_t>(i);
        if (impl.profile->links[i].id == "RightUpperArm") impl.rightShoulderLink = static_cast<std::uint32_t>(i);
        const std::string& id = impl.profile->links[i].id;
        if (id == "LeftFoot") impl.footLinks[0] = static_cast<std::uint32_t>(i);
        if (id == "RightFoot") impl.footLinks[1] = static_cast<std::uint32_t>(i);
        if (id == "UpperChest") impl.upperChestLink = static_cast<std::uint32_t>(i);
        if (id == "LeftHand") impl.handLinks[0] = static_cast<std::uint32_t>(i);
        if (id == "RightHand") impl.handLinks[1] = static_cast<std::uint32_t>(i);
        if (id == "LeftShin") impl.shinLinks[0] = static_cast<std::uint32_t>(i);
        if (id == "RightShin") impl.shinLinks[1] = static_cast<std::uint32_t>(i);
        impl.allowedContact.push_back(id != "Pelvis" && id != "Abdomen" && id != "Chest" && id != "UpperChest"
            && id != "Neck" && id != "Head");
        impl.supportContact.push_back(impl.allowedContact.back() && id.find("Foot") == std::string::npos
            && id.find("Toe") == std::string::npos);
    }
    impl.initLegColliders();
    // (O rolar pelo clipe: o eixo da barriga, o da pelve que aponta para cima no primeiro quadro do levantar de costas.)
    if (!impl.rollIndices.empty()) {
        TrackingClipFrame3D first;
        impl.references[impl.rollIndices.front()]->frame(0.0f, {}, 0.0f, first);
        const Quaternion pelvis = (first.links[impl.rootLink].orientation
            * impl.profile->links[impl.rootLink].modelOrientation.conjugate()).normalized();
        impl.rollBellyAxis = pelvis.conjugate().rotate({ 0.0f, 0.0f, 1.0f });
    }
    // (Os nomes do corpo: os do levantar do DeepMimic sem a fase, a ultima; depois os do comando.)
    impl.observationNames.assign(impl.contract->observationNames().begin(), impl.contract->observationNames().end() - 1);
    for (const char* name : { "comando.alvo_cos", "comando.alvo_sen", "comando.distancia", "comando.velocidade",
             "comando.rumo_cos", "comando.rumo_sen" })
        impl.observationNames.emplace_back(name);
    for (const std::string& name : TrackingGetUpContract3D::groundObservationNames()) impl.observationNames.push_back(name);
    impl.observationNames.emplace_back("fase.sen");
    impl.observationNames.emplace_back("fase.cos");
    for (const std::string& name : TrackingGetUpContract3D::environmentObservationNames()) impl.observationNames.push_back(name);
    // (04/11 15h, o layout levantar-v7: o equilibrio e o estado do gerador de passos, antes da habilidade.)
    if (settings.balanceObservation) {
        for (const std::string& name : TrackingGetUpContract3D::balanceObservationNames()) impl.observationNames.push_back(name);
        for (const char* name : { "passo.no_ar_esq", "passo.no_ar_dir", "passo.progresso", "passo.pouso.frente", "passo.pouso.lado",
                 "passo.sugerido.frente", "passo.sugerido.lado", "passo.desde_pouso", "passo.base.frente", "passo.base.lado" })
            impl.observationNames.emplace_back(name);
    }
    if (settings.idleAnchorObservation)
        for (const char* name : { "parado.ancora.frente", "parado.ancora.lado" }) impl.observationNames.emplace_back(name);
    if (settings.clipVariantObservation) impl.observationNames.emplace_back("levantar.variante");
    if (settings.fireObservation)
        for (const char* name : { "fogo.forca.frente", "fogo.forca.lado", "fogo.forca.cima", "fogo.acel.frente", "fogo.acel.lado", "fogo.acel.cima" })
            impl.observationNames.emplace_back(name);
    impl.observationNames.emplace_back("habilidade.locomocao");
    impl.actionNames = impl.contract->actionNames();
    if (settings.stepChannels)
        for (const char* name : { "passo.vai", "passo.pouso_frente", "passo.pouso_esquerda", "passo.tempo" }) impl.actionNames.emplace_back(name);
    if (settings.locomotionFraction > 0.0f || settings.evaluationKind == 4)
        for (std::uint32_t h = 0; h < 8; h += 2)
            if (impl.gaitFor(1, h) == nullptr) throw std::runtime_error("rastreio: o trote sem o clipe de um rumo cardinal");
    impl.classifyTrackingLinks();
    // (05/11 21h50) As curvas do giro das meias-voltas aqui, antes das threads. Calculadas na primeira troca - dentro do passo
    // paralelo dos mundos -, dois mundos escreviam no mesmo vetor ao mesmo tempo: capturas identicas divergiam no passo da
    // primeira troca (61, 2,03 s), a meia-volta podia seguir uma curva corrompida e o heap corrompido aparecia na saida
    // ("double free", "free(): unaligned chunk").
    impl.runTurnYaw(0);
    impl.runTurnYaw(1);
    const std::uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    impl.workers = settings.threads > 0 ? settings.threads : std::max(1u, hardware > 2 ? hardware - 2 : 1u);
    impl.scheduler = std::make_shared<TaskScheduler>(TaskSchedulerSettings { std::max(1u, impl.workers - 1) });
    impl.engine = std::make_unique<PhysicsEngine3D>(impl.scheduler);
    impl.worlds.resize(std::max(1u, settings.environments));
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) impl.worlds[i].index = static_cast<std::uint32_t>(i);
    for (Impl::World& w : impl.worlds) {
        w.scene = impl.engine->createScene(trackingSceneSettings3D(), impl.materials);
        // (03/11: 60 m para cada lado - a 3 m/s em 8 s o corpo passava da borda dos 20.)
        w.ground = createTrackingGround3D(*w.scene, 60.0f);
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 1;
        spawn.active = true;
        spawn.pelvisPosition = { 0.0f, 0.0f, impl.profile->standingRootHeightMeters };
        // (13/11 v1, o estudo de leveza in-game - ele: "22 bots/players mais o jogador... performance e leveza": o solver das juntas e ~80%
        // do custo por jogador.) MATTER_RL_SOLVER="posicao,velocidade" troca as iteracoes das juntas do boneco; sem ela, as de sempre (20/20).
        if (const char* solver = std::getenv("MATTER_RL_SOLVER")) {
            unsigned position = 0, velocity = 0;
            if (std::sscanf(solver, "%u,%u", &position, &velocity) == 2) {
                spawn.solverPositionIterations = position;
                spawn.solverVelocityIterations = velocity;
            }
        }
        w.handle = w.scene->createRagdoll(*impl.profile, spawn);
        w.fall = std::make_unique<ProtectiveFall3D>(*impl.profile);
        w.observation.assign(observationSize(), 0.0f);
        w.finalObservation.assign(w.observation.size(), 0.0f);
        w.ampPair.assign(2 * impl.features->size(), 0.0f);
        w.pose.assign(impl.contract->actionSize(), 0.0f);
        w.clipAction.assign(impl.contract->actionSize(), 0.0f);
        w.jointErrors.assign(impl.profile->links.size(), 0.0f);
        w.gravity.prepare(*impl.profile);
        w.label.assign(impl.contract->actionSize(), 0.0f);
        w.labelRaw.assign(impl.contract->actionSize(), 0.0f);

    }
    reset(settings.seed);
}

TrackingGetUpAmpTraining3D::~TrackingGetUpAmpTraining3D() {
    if (m_impl) m_impl->worlds.clear();
}

std::size_t TrackingGetUpAmpTraining3D::environmentCount() const { return m_impl->worlds.size(); }
std::size_t TrackingGetUpAmpTraining3D::observationSize() const {
    return m_impl->contract->bodyObservationSize() + CommandSize + GroundObservationSize + PhaseSize + EnvironmentSize
        + balanceObservationSize() + (m_impl->settings.idleAnchorObservation ? 2 : 0) + (m_impl->settings.clipVariantObservation ? 1 : 0)
        + (m_impl->settings.fireObservation ? FireObservationSize : 0) + SkillModeSize;
}
std::size_t TrackingGetUpAmpTraining3D::balanceObservationSize() const {
    return m_impl->settings.balanceObservation ? TrackingGetUpContract3D::BalanceObservationSize + TrackingBalanceSteps3D::FeatureCount : 0;
}
std::size_t TrackingGetUpAmpTraining3D::stepChannelCount() const { return m_impl->settings.stepChannels ? 4 : 0; }
std::size_t TrackingGetUpAmpTraining3D::axisCount() const { return m_impl->contract->actionSize(); }
std::size_t TrackingGetUpAmpTraining3D::actionSize() const {
    return m_impl->contract->actionSize() * (m_impl->settings.velocityActions ? 2 : 1) + stepChannelCount();
}
std::size_t TrackingGetUpAmpTraining3D::ampSize() const { return m_impl->features->size(); }
const std::vector<std::string>& TrackingGetUpAmpTraining3D::observationNames() const {
    return m_impl->observationNames;
}
const std::vector<std::string>& TrackingGetUpAmpTraining3D::actionNames() const { return m_impl->actionNames; }
float TrackingGetUpAmpTraining3D::episodeSeconds() const {
    const TrackingGetUpAmpSettings3D& s = m_impl->settings;
    return std::max({ s.episodeSeconds, s.locomotionSeconds,
        s.starFraction > 0.0f || (s.evaluationManeuver >= 8 && s.evaluationManeuver <= 11) ? s.starSeconds : 0.0f,
        s.commandFraction > 0.0f || s.evaluationManeuver == 12 ? s.commandSeconds : 0.0f });
}
std::vector<float> TrackingGetUpAmpTraining3D::actionRanges() const {
    std::vector<float> out = m_impl->contract->actionRanges();
    // (Os canais do passo: crus, -1..1.)
    for (std::size_t k = 0; k < stepChannelCount(); ++k) {
        out.push_back(-1.0f);
        out.push_back(1.0f);
    }
    // (17h40: a velocidade-alvo de cada eixo, rad/s.)
    if (m_impl->settings.velocityActions)
        for (std::size_t k = 0; k < m_impl->contract->actionSize(); ++k) {
            out.push_back(-VelocityActionLimit);
            out.push_back(VelocityActionLimit);
        }
    return out;
}

void TrackingGetUpAmpTraining3D::setFallStarts(std::vector<std::vector<PhysicsBodyState3D>> states) {
    const std::size_t links = m_impl->profile->links.size();
    std::erase_if(states, [links](const std::vector<PhysicsBodyState3D>& s) { return s.size() != links; });
    m_impl->fallStarts = std::move(states);
}

std::size_t TrackingGetUpAmpTraining3D::fallStartCount() const { return m_impl->fallStarts.size(); }

void TrackingGetUpAmpTraining3D::protecting(std::span<std::uint8_t> out) const {
    for (std::size_t i = 0; i < m_impl->worlds.size() && i < out.size(); ++i) out[i] = m_impl->worlds[i].protecting ? 1 : 0;
}

void TrackingGetUpAmpTraining3D::locomoting(std::span<std::uint8_t> out) const {
    // (11/11, settings.idleUnified: o parado de depois de frear conta como parado - os pesos e o juiz do parado no treinador.)
    for (std::size_t i = 0; i < m_impl->worlds.size() && i < out.size(); ++i)
        out[i] = m_impl->worlds[i].locomotion && !(m_impl->settings.idleUnified && m_impl->parkedIdle(m_impl->worlds[i])) ? 1 : 0;
}

void TrackingGetUpAmpTraining3D::locomotionFeedback(std::span<float> out) const {
    if (out.size() < m_impl->worlds.size() * TrackingFeedbackSize) throw std::invalid_argument("rastreio: buffer pequeno");
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].locomotionFeedback.begin(), m_impl->worlds[i].locomotionFeedback.end(),
            out.begin() + i * TrackingFeedbackSize);
}

void TrackingGetUpAmpTraining3D::locomotionTargets(std::span<float> out) const {
    const auto width = actionSize();
    if (out.size() < environmentCount() * width) throw std::invalid_argument("alvos do clipe: buffer pequeno");
    // (17/11) A linha inteira em zero antes - o que nao e do clipe (os canais do passo; as velocidades sem imitationVelocityTargets).
    std::fill(out.begin(), out.begin() + environmentCount() * width, 0.0f);
    const std::size_t joints = m_impl->contract->actionSize();
    for (std::size_t i = 0; i < environmentCount(); ++i) {
        const Impl::World& w = m_impl->worlds[i];
        std::copy(w.clipAction.begin(), w.clipAction.end(), out.begin() + i * width);
        // (17/11, settings.imitationVelocityTargets) As velocidades do clipe depois dos canais do passo (a ordem das acoes: posicoes, passo,
        // velocidades); so nas linhas com o clipe de locomocao (as do levantar ficam sem).
        if (m_impl->settings.imitationVelocityTargets && m_impl->settings.velocityActions && w.locomotion && m_impl->clipTracked(w)
            && w.clipActionVelocity.size() == joints && joints + stepChannelCount() + joints <= width)
            std::copy(w.clipActionVelocity.begin(), w.clipActionVelocity.end(), out.begin() + i * width + joints + stepChannelCount());
    }
}

void TrackingGetUpAmpTraining3D::mirrorInfo(std::span<float> out) const {
    // (Os clipes periodicos com espelho, conferidos no clipe puro em 05/11: os que servem aos dois lados - o espelho e o
    // mesmo clipe meio ciclo depois - e os pares de lado - o espelho e o outro clipe no mesmo instante.)
    // (05/11 16h30 o run_forward_medium tinha saido - o clipe era assimetrico, o espelho errava a acao em 0,11 rad. 19h55:
    // voltou - o clipe foi simetrizado (tools/animation/simetrizar.py; ele: "a corrida esta com o personagem torto").)
    static constexpr const char* HalfCycle[] = { "jog", "jog_backward", "run_forward_medium", "run_backward_trote",
        "sprint_maximo" };
    static constexpr const char* Paired[] = { "jog_strafe_left", "jog_strafe_right", "sprint_strafe_left",
        "sprint_strafe_right" };
    // (10/11 02h30, settings.mirrorCycles: os clipes ciclo_* de agora.)
    static constexpr const char* HalfCycleCycles[] = { "ciclo_trote_frente", "ciclo_trote_costas", "ciclo_corrida_frente",
        "ciclo_corrida_costas" };
    static constexpr const char* PairedCycles[] = { "ciclo_trote_esquerda", "ciclo_trote_direita", "ciclo_corrida_esquerda",
        "ciclo_corrida_direita" };
    const Impl& impl = *m_impl;
    const std::size_t n = environmentCount();
    if (out.size() < n * MirrorInfoSize) throw std::invalid_argument("espelho: buffer pequeno");
    std::fill(out.begin(), out.begin() + n * MirrorInfoSize, 0.0f);
    for (std::size_t i = 0; i < n; ++i) {
        const Impl::World& w = impl.worlds[i];
        // (05/11 14h40: tambem nas manobras - a troca suave so caia indo para o lateral a esquerda; a terceira coluna diz
        // a manobra, e o treino escolhe se usa - as entradas da rede nao tem nada proprio da manobra alem do comando.)
        if (!w.locomotionControl || w.taught || w.startKind != 6 || impl.guidedRise(w)) continue;
        if (!impl.clipTracked(w) || impl.starting(w)) continue;
        if (w.blendClip < impl.references.size() && w.time < w.blendFrom + w.blendSeconds) continue;
        const std::string& name = impl.clipNames[w.trackClip];
        float shift = -1.0f;
        for (const char* c : HalfCycle)
            if (name == c) shift = Pi;
        for (const char* c : Paired)
            if (name == c) shift = 0.0f;
        if (impl.settings.mirrorCycles) {
            for (const char* c : HalfCycleCycles)
                if (name == c) shift = Pi;
            for (const char* c : PairedCycles)
                if (name == c) shift = 0.0f;
        }
        if (shift < 0.0f) continue;
        out[i * MirrorInfoSize] = 1.0f;
        out[i * MirrorInfoSize + 1] = shift;
        out[i * MirrorInfoSize + 2] = static_cast<float>(w.maneuver);
    }
}

void TrackingGetUpAmpTraining3D::setLocomotionPartner(std::shared_ptr<const LocomotionPartner3D> partner) {
    m_impl->partner = std::move(partner);
}

void TrackingGetUpAmpTraining3D::setLocomotionTeacher(std::shared_ptr<const LocomotionPartner3D> teacher) {
    m_impl->teacher = std::move(teacher);
    for (Impl::World& w : m_impl->worlds) w.teacherSession.reset();
}

void TrackingGetUpAmpTraining3D::setTeacherShadow(bool shadow) { m_impl->settings.teacherShadow = shadow; }

void TrackingGetUpAmpTraining3D::setTeacherLead(float minimum, float maximum) {
    m_impl->settings.teacherLeadMinimum = std::max(0.0f, minimum);
    m_impl->settings.teacherLeadMaximum = std::max(m_impl->settings.teacherLeadMinimum, maximum);
}

void TrackingGetUpAmpTraining3D::setTeacherOracle(int mode) { m_impl->teacherOracle = std::clamp(mode, 0, 4); }

void TrackingGetUpAmpTraining3D::setTeacherStillClip(int mode) { m_impl->settings.teacherStillClip = std::clamp(mode, 0, 2); }

void TrackingGetUpAmpTraining3D::setStudentActuator(std::span<const float> stiffness, std::span<const float> damping,
    std::span<const float> maximumTorque, bool gravity) {
    const std::size_t n = actionSize();
    if (stiffness.empty()) {
        m_impl->studentStiffness.clear();
        m_impl->studentDamping.clear();
        m_impl->studentMaximumTorque.clear();
        m_impl->studentGravity = false;
        return;
    }
    if (stiffness.size() != n || damping.size() != n || maximumTorque.size() != n)
        throw std::invalid_argument("musculo da aluna: um valor por eixo da acao");
    m_impl->studentStiffness.assign(stiffness.begin(), stiffness.end());
    m_impl->studentDamping.assign(damping.begin(), damping.end());
    m_impl->studentMaximumTorque.assign(maximumTorque.begin(), maximumTorque.end());
    m_impl->studentGravity = gravity;
}

std::vector<float> TrackingGetUpAmpTraining3D::teacherDriveStats() const {
    const std::size_t n = actionSize();
    std::vector<double> sum(5 * n, 0.0);
    std::uint64_t ticks = 0;
    for (const Impl::World& w : m_impl->worlds) {
        if (w.teacherStats.size() != sum.size()) continue;
        for (std::size_t k = 0; k < sum.size(); ++k) sum[k] += w.teacherStats[k];
        ticks += w.teacherTicks;
    }
    std::vector<float> out(5 * n, 0.0f);
    for (std::size_t k = 0; k < out.size(); ++k) out[k] = ticks ? static_cast<float>(sum[k] / static_cast<double>(ticks)) : 0.0f;
    return out;
}

void TrackingGetUpAmpTraining3D::setBalanceFeet(bool on) { m_impl->settings.balanceFeet = on; }
void TrackingGetUpAmpTraining3D::setBalanceSteps(bool steps, bool throughPushes) {
    m_impl->settings.balanceSteps = steps;
    m_impl->settings.idleThroughPushes = throughPushes;
}
void TrackingGetUpAmpTraining3D::setStepTuning(float loadLimit, float loadWeight, bool swingFromBody) {
    m_impl->settings.stepLoadLimit = loadLimit;
    m_impl->settings.stepLoadWeight = loadWeight;
    m_impl->settings.stepSwingFromBody = swingFromBody;
}
void TrackingGetUpAmpTraining3D::setIdleThreatReach(float reach) { m_impl->settings.idleThreatReach = std::clamp(reach, 0.0f, 3.0f); }
void TrackingGetUpAmpTraining3D::setAvoidOnIdle(bool on) { m_impl->settings.avoidOnIdle = on; }
void TrackingGetUpAmpTraining3D::setGetUpArrival(float bonus, float flightPenalty, float riseBalance, float calmHandoff) {
    m_impl->settings.calmHandoffSeconds = std::clamp(calmHandoff, 0.0f, 5.0f);
    m_impl->settings.arrivalBonus = std::max(0.0f, bonus);
    m_impl->settings.flightPenalty = std::max(0.0f, flightPenalty);
    m_impl->settings.riseBalancePenalty = std::max(0.0f, riseBalance);
}
void TrackingGetUpAmpTraining3D::setBalanceDrill(float fraction, float pushLevel) {
    m_impl->settings.balanceDrillFraction = std::clamp(fraction, 0.0f, 1.0f);
    m_impl->settings.balancePushLevel = std::clamp(pushLevel, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setFire(float episodeShare, std::array<float, 2> hitNewtons, std::array<float, 2> holdNewtons,
    std::array<float, 2> holdSeconds, float holdFraction) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.pushEpisodeShare = std::clamp(episodeShare, 0.0f, 1.0f);
    s.pushMinimumNewtons = s.movingPushMinimumNewtons = std::max(0.0f, hitNewtons[0]);
    s.pushMaximumNewtons = s.movingPushMaximumNewtons = std::max(s.pushMinimumNewtons, hitNewtons[1]);
    s.pushHoldMinimumNewtons = s.movingPushHoldMinimumNewtons = std::max(0.0f, holdNewtons[0]);
    s.pushHoldMaximumNewtons = s.movingPushHoldMaximumNewtons = std::max(s.pushHoldMinimumNewtons, holdNewtons[1]);
    s.pushHoldMinimumSeconds = s.movingPushHoldMinimumSeconds = std::max(0.05f, holdSeconds[0]);
    s.pushHoldMaximumSeconds = s.movingPushHoldMaximumSeconds = std::max(s.pushHoldMinimumSeconds, holdSeconds[1]);
    s.pushHoldFraction = s.movingPushHoldFraction = std::clamp(holdFraction, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setStar(float fraction, std::array<float, 6> weights, std::array<int, 6> phases, float seconds) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.starFraction = std::clamp(fraction, 0.0f, 1.0f);
    s.starWeights = weights;
    for (std::size_t k = 0; k < 6; ++k) s.starPhases[k] = std::clamp(phases[k], 1, 4);
    s.starSeconds = std::max(1.0f, seconds);
}
void TrackingGetUpAmpTraining3D::setPushVariety(bool on) { m_impl->settings.pushVariety = on; }
void TrackingGetUpAmpTraining3D::setFinalStop(float fraction, float earliest, float latest, float deceleration, bool inEvaluation) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.stopFraction = std::clamp(fraction, 0.0f, 1.0f);
    s.stopEarliest = std::max(0.5f, earliest);
    s.stopLatest = std::max(s.stopEarliest, latest);
    s.stopDeceleration = std::max(1.0f, deceleration);
    s.stopInEvaluation = inEvaluation;
}
void TrackingGetUpAmpTraining3D::setFireShown(bool on) { m_impl->settings.fireShown = on; }
void TrackingGetUpAmpTraining3D::setCourseAngleLimit(float radians) { m_impl->settings.courseAngleLimit = std::max(0.0f, radians); }
void TrackingGetUpAmpTraining3D::setLocomotionShaping(bool on) { m_impl->settings.locomotionShaping = on; }
void TrackingGetUpAmpTraining3D::setSpeedGate(float floor) { m_impl->settings.speedGate = std::clamp(floor, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setKneeInward(float slackMeters) { m_impl->settings.kneeInwardSlack = std::max(0.0f, slackMeters); }
void TrackingGetUpAmpTraining3D::setIdleHeight(float slackMeters) { m_impl->settings.idleHeightSlack = std::max(0.0f, slackMeters); }
void TrackingGetUpAmpTraining3D::setStopWaitBody(bool on) { m_impl->settings.stopWaitBody = on; }
void TrackingGetUpAmpTraining3D::setCourseSmoothing(float seconds) { m_impl->settings.courseSmoothing = std::max(0.0f, seconds); }
void TrackingGetUpAmpTraining3D::setDiagonalChest(float gate) { m_impl->settings.diagonalChestGate = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setStopClips(bool on) {
    Impl& impl = *m_impl;
    impl.settings.stopClips = on;
    if (on && impl.stopClipInfo.empty()) impl.loadStopClips();
}
void TrackingGetUpAmpTraining3D::setCommand(float fraction, int phase, float seconds) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.commandFraction = std::clamp(fraction, 0.0f, 1.0f);
    s.commandPhase = std::clamp(phase, 1, 4);
    s.commandSeconds = std::max(1.0f, seconds);
}
void TrackingGetUpAmpTraining3D::setPushImitationRelief(bool on) { m_impl->settings.pushImitationRelief = on; }
void TrackingGetUpAmpTraining3D::setLateralLegStrict(float factor) { m_impl->settings.lateralLegStrict = std::clamp(factor, 0.25f, 8.0f); }
void TrackingGetUpAmpTraining3D::setFireGroups(const std::vector<float>& bands) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.fireGrouped = bands.size() >= 24;
    for (std::size_t g = 0; g < 6 && s.fireGrouped; ++g)
        for (std::size_t k = 0; k < 4; ++k) s.fireGroupBands[g][k] = std::max(0.0f, bands[4 * g + k]);
}
void TrackingGetUpAmpTraining3D::setRiseFlight(float penalty) { m_impl->settings.riseFlightPenalty = std::max(0.0f, penalty); }
void TrackingGetUpAmpTraining3D::setSlowLimit(float fraction) { setSlowLimits({ fraction, fraction, fraction }); }
void TrackingGetUpAmpTraining3D::setSlowLimits(std::vector<float> fractions) {
    if (fractions.size() != 3 && fractions.size() != 6) throw std::invalid_argument("setSlowLimits: 3 ou 6 pisos");
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    for (std::size_t k = 0; k < 6; ++k) s.slowLimits[k] = std::clamp(fractions[k % fractions.size()], 0.0f, 0.95f);
    s.slowLimit = *std::max_element(s.slowLimits.begin(), s.slowLimits.end());
}
void TrackingGetUpAmpTraining3D::setTrackingReward(int version) { m_impl->settings.trackingReward = version; }
void TrackingGetUpAmpTraining3D::setGaitCycles(bool on) {
    Impl& impl = *m_impl;
    if (impl.baseGaits[1].empty() && impl.baseGaits[2].empty()) {
        impl.baseGaits = impl.gaits;
        impl.baseRunBack = impl.runBackIndex;
        impl.baseJogBack = impl.jogBackIndex;
    }
    // (So com os 4 de cada faixa carregados; senao, os de antes.)
    const bool ready = impl.cycleGaits[1].size() == 4 && impl.cycleGaits[2].size() == 4;
    impl.gaits = impl.baseGaits;
    impl.runBackIndex = impl.baseRunBack;
    impl.jogBackIndex = impl.baseJogBack;
    if (on && ready) {
        impl.gaits[1] = impl.cycleGaits[1];
        impl.gaits[2] = impl.cycleGaits[2];
        impl.runBackIndex = impl.cycleRunBack;
        impl.jogBackIndex = impl.cycleJogBack;
    }
}
void TrackingGetUpAmpTraining3D::setPhaseSync(bool on) { m_impl->settings.phaseSync = on; }
void TrackingGetUpAmpTraining3D::setImitationLoosen(float factor) { m_impl->settings.imitationLoosen = std::max(1.0f, factor); }
void TrackingGetUpAmpTraining3D::setFacingGate(float gate) { m_impl->settings.facingGate = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setArmClipReach(float reach) { m_impl->settings.armClipReach = reach; }
void TrackingGetUpAmpTraining3D::setHeadClipReach(float reach) { m_impl->settings.headClipReach = reach; }
void TrackingGetUpAmpTraining3D::setLinearHand(bool on) { m_impl->settings.linearHand = on; }
void TrackingGetUpAmpTraining3D::setUpperHand(bool on) { m_impl->settings.upperHand = on; }
void TrackingGetUpAmpTraining3D::setBodyHand(bool on) { m_impl->settings.bodyHand = on; }
void TrackingGetUpAmpTraining3D::setIdle(bool afterStand, bool onLocomotion, float residualScale) {
    m_impl->settings.idleAfterStand = afterStand;
    m_impl->settings.idleOnLocomotion = onLocomotion;
    m_impl->settings.idleResidualScale = std::clamp(residualScale, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setLateralHandPush(bool push) { m_impl->settings.lateralHandPush = push; }
void TrackingGetUpAmpTraining3D::setLateralRunLegReach(float scale) { m_impl->settings.lateralRunLegReach = std::clamp(scale, 0.0f, 4.0f); }
void TrackingGetUpAmpTraining3D::setFrontLean(float radians, float startRadians) {
    m_impl->settings.frontLean = std::clamp(radians, 0.0f, 0.6f);
    m_impl->settings.frontStartLean = std::clamp(startRadians, 0.0f, 0.8f);
}
void TrackingGetUpAmpTraining3D::setCurveLean(bool lean) { m_impl->settings.curveLean = lean; }

void TrackingGetUpAmpTraining3D::setFootWidth(float slackMeters) { m_impl->settings.footWidthSlack = std::max(0.0f, slackMeters); }
void TrackingGetUpAmpTraining3D::setRunTurnBrake(bool brake) { m_impl->settings.runTurnBrake = brake; }
void TrackingGetUpAmpTraining3D::setRunTurnAlign(bool align) { m_impl->settings.runTurnAlign = align; }
void TrackingGetUpAmpTraining3D::setTurnSettle(bool settle) { m_impl->settings.turnSettle = settle; }
void TrackingGetUpAmpTraining3D::setTurnLaunch(bool launch) { m_impl->settings.turnLaunch = launch; }
void TrackingGetUpAmpTraining3D::setMovingHandoff(bool handoff) { m_impl->settings.movingHandoff = handoff; }
void TrackingGetUpAmpTraining3D::setTrocaFixedChest(bool on) { m_impl->settings.trocaFixedChestNonForward = on; }
void TrackingGetUpAmpTraining3D::setTurnSettleBack(float speed) { m_impl->settings.turnSettleBackSpeed = std::max(0.0f, speed); }
void TrackingGetUpAmpTraining3D::setTurnSettleLateral(float speed) { m_impl->settings.turnSettleLateralSpeed = std::max(0.0f, speed); }
void TrackingGetUpAmpTraining3D::setTurnSettleFront(float speed) { m_impl->settings.turnSettleFrontSpeed = std::max(0.0f, speed); }
void TrackingGetUpAmpTraining3D::setStopGaitAll(bool on) { m_impl->settings.stopGaitAll = on; }
void TrackingGetUpAmpTraining3D::setStopClipsFront(bool on) { m_impl->settings.stopClipsFront = on; }
void TrackingGetUpAmpTraining3D::setStopClipsCommand(bool on) { m_impl->settings.stopClipsCommand = on; }
void TrackingGetUpAmpTraining3D::setStopGateFloor(float floor) { m_impl->settings.stopGateFloor = std::clamp(floor, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setFreeStop(bool on) { m_impl->settings.freeStop = on; }
void TrackingGetUpAmpTraining3D::setStopPosture(float gate) { m_impl->settings.stopPosture = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setIdleUnified(bool on) { m_impl->settings.idleUnified = on; }
void TrackingGetUpAmpTraining3D::setStopToIdle(bool on) { m_impl->settings.stopToIdle = on; }
void TrackingGetUpAmpTraining3D::setIdleAfterStop(bool on) { m_impl->settings.idleAfterStop = on; }
void TrackingGetUpAmpTraining3D::setParkedOnLocomotion(bool on) { m_impl->settings.parkedOnLocomotion = on; }
void TrackingGetUpAmpTraining3D::setLocomotionFire(float share, std::array<float, 6> levels, float relief) {
    m_impl->settings.locoFireShare = std::clamp(share, 0.0f, 1.0f);
    for (float& l : levels) l = std::clamp(l, 0.0f, 1.0f);
    m_impl->settings.locoFireLevels = levels;
    m_impl->settings.locoFireRelief = std::max(0.0f, relief);
}
void TrackingGetUpAmpTraining3D::setCrookedNoFire(bool on) { m_impl->settings.crookedNoFire = on; }
void TrackingGetUpAmpTraining3D::setCrookedFixSpan(float seconds) { m_impl->settings.crookedFixSpan = std::max(0.5f, seconds); }
void TrackingGetUpAmpTraining3D::setLocomotionFireGaps(float scale) { m_impl->settings.locoFireGapScale = std::clamp(scale, 0.1f, 10.0f); }
void TrackingGetUpAmpTraining3D::setCrookedStart(float share, float bonus, float close) {
    m_impl->settings.crookedStartShare = std::clamp(share, 0.0f, 1.0f);
    m_impl->settings.crookedStartBonus = std::max(0.0f, bonus);
    m_impl->settings.crookedFixClose = std::clamp(close, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setCommandGetUp(bool on, float runShare) {
    m_impl->settings.commandGetUp = on;
    m_impl->settings.commandRunGetUpShare = std::clamp(runShare, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setFallToCommand(bool on) { m_impl->settings.fallToCommand = on; }
void TrackingGetUpAmpTraining3D::setGetUpGo(float share, float minMeters, float maxMeters) {
    m_impl->settings.getUpGoShare = std::clamp(share, 0.0f, 1.0f);
    m_impl->settings.getUpGoMin = std::max(0.5f, minMeters);
    m_impl->settings.getUpGoMax = std::max(m_impl->settings.getUpGoMin, maxMeters);
}
void TrackingGetUpAmpTraining3D::setRunHandoffAtStand(bool on, float afterSeconds) {
    m_impl->settings.runHandoffAtStand = on;
    m_impl->settings.runAfterSeconds = afterSeconds;
}
void TrackingGetUpAmpTraining3D::setStarRework(float pathWeight, float pathWidth, float curveShare, float curveMin, float curveMax) {
    m_impl->settings.starPathWeight = std::clamp(pathWeight, 0.0f, 1.0f);
    m_impl->settings.starPathWidth = std::max(0.1f, pathWidth);
    m_impl->settings.starCurveShare = std::clamp(curveShare, 0.0f, 1.0f);
    m_impl->settings.starCurveMin = std::clamp(curveMin, 0.0f, 0.49f);
    m_impl->settings.starCurveMax = std::clamp(curveMax, 0.0f, 0.49f);
}
void TrackingGetUpAmpTraining3D::setLocomotionFireExam(bool on, float level) {
    m_impl->settings.locoFireExam = on;
    m_impl->settings.locoFireExamLevel = std::clamp(level, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setStillCost(float weight) { m_impl->settings.stillCost = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setIdleStatic(bool on) { m_impl->settings.idleStatic = on; }
void TrackingGetUpAmpTraining3D::setTurnVariety(bool on, float spineReward) {
    m_impl->settings.turnVariety = on;
    m_impl->settings.turnSpineReward = std::clamp(spineReward, 0.0f, 2.0f);
}
void TrackingGetUpAmpTraining3D::setClipSideNow(bool enabled) { m_impl->settings.clipSideNow = enabled; }
void TrackingGetUpAmpTraining3D::setClipTrackMask(int mask) { m_impl->settings.clipTrackMask = static_cast<std::uint32_t>(std::max(0, mask)); }
void TrackingGetUpAmpTraining3D::setGetUpImitationAnchor(bool enabled) { m_impl->settings.getUpImitationAnchor = enabled; }
void TrackingGetUpAmpTraining3D::setClipUnderPelvis(bool enabled) { m_impl->settings.clipUnderPelvis = enabled; }
void TrackingGetUpAmpTraining3D::setIdleClipAnchor(bool enabled) { m_impl->settings.idleClipAnchor = enabled; }
void TrackingGetUpAmpTraining3D::setReviewFixes(bool enabled) { m_impl->settings.reviewFixes = enabled; }
void TrackingGetUpAmpTraining3D::setLaunchSideMargin(bool enabled) { m_impl->settings.launchSideMargin = enabled; }
void TrackingGetUpAmpTraining3D::setRunWidth(float weight) { m_impl->settings.runWidthWeight = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setIdleWeightCenter(float weight) { m_impl->settings.idleWeightCenter = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setSpeedWidthFraction(float fraction) { m_impl->settings.speedWidthFraction = std::clamp(fraction, 0.05f, 1.0f); }
void TrackingGetUpAmpTraining3D::setLaunchArmsCalm(float weight) { m_impl->settings.launchArmsCalm = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setLegAlignCommand(bool enabled) { m_impl->settings.legAlignCommand = enabled; }
void TrackingGetUpAmpTraining3D::setReferenceSideFix(bool enabled) { m_impl->settings.referenceSideFix = enabled; }
void TrackingGetUpAmpTraining3D::setPhaseSyncBothFeet(bool enabled) { m_impl->settings.phaseSyncBothFeet = enabled; }
void TrackingGetUpAmpTraining3D::setHeadingBias(float weight, float width) {
    m_impl->settings.headingBiasWeight = std::clamp(weight, 0.0f, 1.0f);
    m_impl->settings.headingBiasWidth = std::max(0.02f, width);
}
void TrackingGetUpAmpTraining3D::setContactForceObservation(bool enabled) { m_impl->settings.contactForceObservation = enabled; }
void TrackingGetUpAmpTraining3D::setStraightCommand(bool enabled) { m_impl->settings.straightCommand = enabled; }
void TrackingGetUpAmpTraining3D::setLateralBias(float weight, float width) {
    m_impl->settings.lateralBiasWeight = std::clamp(weight, 0.0f, 1.0f);
    m_impl->settings.lateralBiasWidth = std::max(0.005f, width);
}
void TrackingGetUpAmpTraining3D::setImitationGate(float gate, float full) {
    m_impl->settings.imitationGate = std::clamp(gate, 0.0f, 1.0f);
    m_impl->settings.imitationGateFull = std::max(0.05f, full);
}
void TrackingGetUpAmpTraining3D::setGrassGroundFraction(float fraction) {
    // Os mundos de grama espalhados pelos indices (a razao aurea), para nao cair num bloco de exercicios.
    fraction = std::clamp(fraction, 0.0f, 1.0f);
    for (Impl::World& w : m_impl->worlds) {
        const float u = static_cast<float>(w.index) * 0.6180339887f;
        const bool grass = u - std::floor(u) < fraction;
        if (grass == w.grassGround) continue;
        w.scene->destroyBody(w.ground);
        w.ground = createTrackingGround3D(*w.scene, 60.0f, grass ? "grass" : "concrete");
        w.grassGround = grass;
    }
}
void TrackingGetUpAmpTraining3D::setLocomotionScenery(float fraction) { m_impl->settings.locomotionSceneryFraction = std::clamp(fraction, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setIdleHold(float weight, float widthMeters) {
    m_impl->settings.idleHoldWeight = std::clamp(weight, 0.0f, 1.0f);
    m_impl->settings.idleHoldWidth = std::max(0.02f, widthMeters);
}
void TrackingGetUpAmpTraining3D::setLegAlign(float weight, float widthDegrees) {
    m_impl->settings.legAlignWeight = std::clamp(weight, 0.0f, 1.0f);
    m_impl->settings.legAlignWidth = std::max(1.0f, widthDegrees) * Pi / 180.0f;
}
void TrackingGetUpAmpTraining3D::setGetUpImitationFar(float threshold, float seconds) {
    m_impl->settings.getUpImitationFar = std::max(0.0f, threshold);
    m_impl->settings.getUpImitationFarSeconds = std::max(0.1f, seconds);
}
void TrackingGetUpAmpTraining3D::setGetUpImitationReward(float weight) { m_impl->settings.getUpImitationReward = std::max(0.0f, weight); }
void TrackingGetUpAmpTraining3D::setGetUpImitationPaceRun(float seconds) { m_impl->settings.getUpImitationPaceRun = std::max(0.0f, seconds); }
void TrackingGetUpAmpTraining3D::setGetUpView(float weight) { m_impl->settings.getUpViewReward = std::clamp(weight, 0.0f, 4.0f); }
void TrackingGetUpAmpTraining3D::setGetUpProneTimeScale(float scale) { m_impl->settings.getUpProneTimeScale = std::clamp(scale, 0.5f, 1.5f); }
void TrackingGetUpAmpTraining3D::setHandBehind(float weight) { m_impl->settings.handBehindWeight = std::clamp(weight, 0.0f, 4.0f); }
void TrackingGetUpAmpTraining3D::setGetUpStandBonusScale(float scale) { m_impl->settings.getUpStandBonusScale = std::clamp(scale, 0.0f, 1.0f); }
std::vector<float> TrackingGetUpAmpTraining3D::getUpViewNow(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    if (w.body == nullptr || w.body->links.empty()) return { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    const std::uint32_t head = impl.contract->headLink();
    return { wrapAngle(impl.chestYaw(w) - w.faceYaw) * 180.0f / Pi, head < w.body->links.size() ? w.body->links[head].position.z : 0.0f,
             impl.getUpViewOn(w) ? 1.0f : 0.0f, w.clipHandedAt >= 0.0f ? 1.0f : 0.0f, w.getUpGo ? 1.0f : 0.0f, w.locomotion ? 1.0f : 0.0f };
}
void TrackingGetUpAmpTraining3D::setGetUpImitation(int mask, float paceSeconds) {
    m_impl->settings.getUpImitationMask = std::max(0, mask);
    m_impl->settings.getUpImitationPace = std::max(0.3f, paceSeconds);
}
void TrackingGetUpAmpTraining3D::setStopArmsReach(float reach) { m_impl->settings.stopArmsReach = reach; }
void TrackingGetUpAmpTraining3D::setTurnedGrace(float seconds) { m_impl->settings.turnedGrace = std::clamp(seconds, 0.0f, 3.0f); }
void TrackingGetUpAmpTraining3D::setIdleStillBase(float gate) { m_impl->settings.idleStillBase = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setIdleReward(float pose, float still, float slide) {
    m_impl->settings.idleRewardPose = std::clamp(pose, 0.0f, 2.0f);
    m_impl->settings.idleRewardStill = std::clamp(still, 0.0f, 2.0f);
    m_impl->settings.idleSlideCost = std::clamp(slide, 0.0f, 5.0f);
}
void TrackingGetUpAmpTraining3D::setIdleCosts(float motion, float pose) {
    m_impl->settings.idleMotionCost = std::clamp(motion, 0.0f, 1.0f);
    m_impl->settings.idlePoseCost = std::clamp(pose, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setStopIdlePose(float weight) { m_impl->settings.stopIdlePose = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setUpperRigor(float factor) { m_impl->settings.upperRigor = std::clamp(factor, 1.0f, 8.0f); }
void TrackingGetUpAmpTraining3D::setStopClipsBase(bool on) { m_impl->settings.stopClipsBase = on; }
void TrackingGetUpAmpTraining3D::setStopEntry(float blendSeconds, float windowSeconds) {
    m_impl->settings.stopEntryBlend = std::clamp(blendSeconds, 0.05f, 1.5f);
    m_impl->settings.stopEntryWindow = std::clamp(windowSeconds, 0.0f, 0.6f);
}
void TrackingGetUpAmpTraining3D::setStopFrontVariantMask(int mask) { m_impl->settings.stopFrontVariantMask = std::clamp(mask, 1, 15); }
void TrackingGetUpAmpTraining3D::setStopFrontVariants(bool on, int variant) {
    Impl& impl = *m_impl;
    impl.settings.stopFrontVariants = on;
    impl.settings.stopFrontVariant = std::clamp(variant, -1, 3);
    if (on) impl.loadMixamoStops();
}
void TrackingGetUpAmpTraining3D::setLiveLaunch(bool on) { m_impl->settings.liveLaunch = on; }
void TrackingGetUpAmpTraining3D::setLaunchHand(float level) { m_impl->settings.launchHand = std::clamp(level, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setLaunchInPlacePenalty(float weight) { m_impl->settings.launchInPlacePenalty = std::clamp(weight, 0.0f, 4.0f); }
void TrackingGetUpAmpTraining3D::setLaunchSpeedShare(float share) { m_impl->settings.launchSpeedShare = std::clamp(share, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setLaunchImitation(float weight) { m_impl->settings.launchImitationReward = std::clamp(weight, 0.0f, 4.0f); }
float TrackingGetUpAmpTraining3D::launchImitationNow(std::size_t world) const { return m_impl->worlds.at(world).launchImitation; }
void TrackingGetUpAmpTraining3D::setLaunchLean(float weight) { m_impl->settings.launchLeanWeight = std::clamp(weight, 0.0f, 4.0f); }
std::vector<float> TrackingGetUpAmpTraining3D::launchLeanNow(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    if (!w.locomotion || !impl.clipTracked(w)) return { std::nanf(""), std::nanf(""), 0.0f };
    const std::array<float, 2> lean = impl.launchLeans(w);
    return { lean[0], lean[1], impl.launchWeight(w) };
}
void TrackingGetUpAmpTraining3D::setSpineVelocityFeedforward(bool on) { m_impl->settings.spineVelocityFeedforward = on; }
void TrackingGetUpAmpTraining3D::setBasePosture(float weight) { m_impl->settings.basePosture = std::clamp(weight, 0.0f, 4.0f); }
void TrackingGetUpAmpTraining3D::setMotorTone(float tone) { m_impl->settings.motorTone = std::clamp(tone, 0.5f, 4.0f); }
void TrackingGetUpAmpTraining3D::setPostureTail(float share) { m_impl->settings.postureTail = std::clamp(share, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setSpineTone(float tone) { m_impl->settings.spineTone = std::clamp(tone, 0.5f, 4.0f); }
void TrackingGetUpAmpTraining3D::setArmTone(float tone) { m_impl->settings.armTone = std::clamp(tone, 0.5f, 6.0f); }
void TrackingGetUpAmpTraining3D::setClipVelocityRotation(bool enabled) { m_impl->settings.clipVelocityRotation = enabled; }
void TrackingGetUpAmpTraining3D::setImitationVelocityTargets(bool enabled) { m_impl->settings.imitationVelocityTargets = enabled; }
void TrackingGetUpAmpTraining3D::setProjectiles(float rate, float minimumImpulse, float maximumImpulse, float seconds) {
    m_impl->settings.projectileRate = std::max(0.0f, rate);
    m_impl->settings.projectileImpulse = { std::max(0.0f, minimumImpulse), std::max(minimumImpulse, maximumImpulse) };
    m_impl->settings.projectileSeconds = std::max(0.01f, seconds);
}
void TrackingGetUpAmpTraining3D::setJamCost(float weight) { m_impl->settings.jamCost = std::clamp(weight, 0.0f, 2.0f); }
void TrackingGetUpAmpTraining3D::setFootSlipCost(float weight) { m_impl->settings.footSlipCost = std::clamp(weight, 0.0f, 2.0f); }
void TrackingGetUpAmpTraining3D::setMotorGravity(bool on) { m_impl->settings.motorGravity = on; }
void TrackingGetUpAmpTraining3D::setLaunchCalmArms(bool on) {
    Impl& impl = *m_impl;
    impl.settings.launchCalmArms = on;
    if (on && impl.idleArmTargets.empty() && impl.standingIndex < impl.references.size()) {
        TrackingClipFrame3D frame;
        impl.references[impl.standingIndex]->frame(0.0f, {}, 0.0f, frame);
        impl.idleArmTargets.assign(impl.contract->actionSize(), 0.0f);
        impl.contract->actionOf(frame.coordinates, impl.idleArmTargets);
    }
}
void TrackingGetUpAmpTraining3D::setLaunchClipShare(float share) { m_impl->settings.launchClipShare = std::clamp(share, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setLaunchStepReward(float weight) { m_impl->settings.launchStepReward = std::clamp(weight, 0.0f, 4.0f); }
void TrackingGetUpAmpTraining3D::setUnrealStartScale(float scale) { m_impl->settings.unrealStartScale = std::clamp(scale, 0.0f, 1.5f); }
void TrackingGetUpAmpTraining3D::setUnrealStarts(bool on) {
    Impl& impl = *m_impl;
    impl.settings.unrealStarts = on;
    if (on && impl.startInfo.empty()) impl.loadUnrealStarts();
}
void TrackingGetUpAmpTraining3D::setFreeTroca(bool on) { m_impl->settings.freeTroca = on; }
void TrackingGetUpAmpTraining3D::setFreeStandGate(float gate) { m_impl->settings.freeStandGate = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setFreeIdle(bool on) { m_impl->settings.freeIdle = on; }
void TrackingGetUpAmpTraining3D::setPoseBoost(float gate) { m_impl->settings.poseBoost = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setFreePush(bool on) { m_impl->settings.freePush = on; }
void TrackingGetUpAmpTraining3D::setFreeIdleMatch(float weight) { m_impl->settings.freeIdleMatch = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setFreeStepHeight(float meters) { m_impl->settings.freeStepHeight = std::max(0.0f, meters); }
void TrackingGetUpAmpTraining3D::setIdleBentEnd(bool on) { m_impl->settings.idleBentEnd = on; }
void TrackingGetUpAmpTraining3D::setFinalRow(bool on) { m_impl->settings.finalRow = on; }
void TrackingGetUpAmpTraining3D::setIdleLikeStop(bool on) { m_impl->settings.idleLikeStop = on; }
void TrackingGetUpAmpTraining3D::setIdleWide(bool on) { m_impl->settings.idleWide = on; }
void TrackingGetUpAmpTraining3D::setMirrorCycles(bool on) { m_impl->settings.mirrorCycles = on; }
void TrackingGetUpAmpTraining3D::setIdlePoseGate(float gate) { m_impl->settings.idlePoseGate = std::clamp(gate, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setRunTurnTimeScale(float scale) { m_impl->settings.runTurnTimeScale = std::clamp(scale, 0.2f, 1.0f); }
void TrackingGetUpAmpTraining3D::setRiseStiffness(float scale) { m_impl->settings.riseStiffness = std::clamp(scale, 0.0f, 2.0f); }
void TrackingGetUpAmpTraining3D::setFreeVelocityFeedforward(bool on) { m_impl->settings.freeVelocityFeedforward = on; }
void TrackingGetUpAmpTraining3D::setFreeDampingScale(float scale) { m_impl->settings.freeDampingScale = std::clamp(scale, 0.0f, 2.0f); }
void TrackingGetUpAmpTraining3D::setRecoveryVelocity(bool on) { m_impl->settings.recoveryVelocity = on; }
void TrackingGetUpAmpTraining3D::setVelocityActions(bool on) {
    if (m_impl->settings.velocityActions == on) return;
    m_impl->settings.velocityActions = on;
    const std::vector<std::string>& axes = m_impl->contract->actionNames();
    if (on)
        for (const std::string& name : axes) m_impl->actionNames.push_back("vel." + name);
    else
        m_impl->actionNames.resize(m_impl->actionNames.size() - axes.size());
}
void TrackingGetUpAmpTraining3D::setLateralStart(float acceleration) {
    m_impl->settings.lateralStartAcceleration = std::max(0.0f, acceleration);
}
void TrackingGetUpAmpTraining3D::setLaunchRise(float spanMeters) { m_impl->settings.launchRiseSpan = std::max(0.0f, spanMeters); }
void TrackingGetUpAmpTraining3D::setCliffFree(bool on) { setCliffParts(on, on, on, on); }
void TrackingGetUpAmpTraining3D::setCliffParts(bool curveRoll, bool footCross, bool forearm, bool hop) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.cliffFree = curveRoll || footCross || forearm || hop;
    s.cliffCurveRoll = curveRoll;
    s.cliffFootCross = footCross;
    s.cliffForearm = forearm;
    s.cliffHop = hop;
    m_impl->classifyTrackingLinks();
}
void TrackingGetUpAmpTraining3D::setRollTail(float weight) { m_impl->settings.rollTail = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setCourseTail(float weight) { m_impl->settings.courseTail = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setSpeedRange(float low) { m_impl->settings.speedRangeLow = std::clamp(low, 0.1f, 1.0f); }
void TrackingGetUpAmpTraining3D::setIdleQuiet(float weight) { m_impl->settings.idleQuiet = std::max(0.0f, weight); }
void TrackingGetUpAmpTraining3D::setManeuverAcceleration(float acceleration) {
    m_impl->settings.maneuverAcceleration = std::clamp(acceleration, 1.0f, 12.0f);
}
void TrackingGetUpAmpTraining3D::setRiseOvershoot(float weight) { m_impl->settings.riseOvershoot = std::max(0.0f, weight); }
void TrackingGetUpAmpTraining3D::setRunLaunchEase(float scale, float easeSeconds, float acceleration) {
    m_impl->settings.runLaunchScale = std::max(0.0f, scale);
    m_impl->settings.runLaunchEaseSeconds = std::max(0.0f, easeSeconds);
    m_impl->settings.runLaunchAcceleration = std::max(0.0f, acceleration);
}
void TrackingGetUpAmpTraining3D::setTrocaPractice(bool practice, std::array<float, 8> headingWeights) {
    m_impl->settings.trocaPractice = practice;
    m_impl->settings.trocaHeadingWeights = headingWeights;
}
void TrackingGetUpAmpTraining3D::setFootCross(float slackMeters) { m_impl->settings.footCrossSlack = std::max(0.0f, slackMeters); }
void TrackingGetUpAmpTraining3D::setIdleReanchor(bool reanchor) { m_impl->settings.idleReanchor = reanchor; }
void TrackingGetUpAmpTraining3D::setAccelLean(float radians) { m_impl->settings.accelLean = std::clamp(radians, 0.0f, 0.6f); }
void TrackingGetUpAmpTraining3D::setSprint(float speed, float leanRadians, float legs) {
    m_impl->settings.sprintSpeed = std::clamp(speed, 0.0f, 12.0f);
    m_impl->settings.sprintLean = std::clamp(leanRadians, 0.0f, 0.6f);
    // (05/11 01h45: ate 2 - o dorso menos inclinado que as pernas.)
    m_impl->settings.sprintLeanLegs = std::clamp(legs, 0.0f, 2.0f);
}
void TrackingGetUpAmpTraining3D::setRunLean(float radians, float legs) {
    m_impl->settings.runLean = std::clamp(radians, 0.0f, 0.5f);
    // (05/11 01h55: ate 2, como o sprint - o dorso menos inclinado que as pernas.)
    m_impl->settings.runLeanLegs = std::clamp(legs, 0.0f, 2.0f);
}
void TrackingGetUpAmpTraining3D::setGetUpTracking(bool on) { m_impl->settings.getUpTracking = on; }
void TrackingGetUpAmpTraining3D::setRunBackLean(float radians, float legs) {
    m_impl->settings.runBackLean = std::clamp(radians, 0.0f, 0.5f);
    m_impl->settings.runBackLeanLegs = std::clamp(legs, 0.0f, 2.0f);
}
void TrackingGetUpAmpTraining3D::setCourseLimit(float meters) { m_impl->settings.courseLimit = std::clamp(meters, 0.0f, 20.0f); }
void TrackingGetUpAmpTraining3D::setBackStartLean(float radians) { m_impl->settings.backStartLean = std::clamp(radians, -0.8f, 0.0f); }
void TrackingGetUpAmpTraining3D::setGaitSpeeds(std::array<float, 8> trot, std::array<float, 8> run) {
    for (std::size_t k = 0; k < 8; ++k) {
        m_impl->settings.trotSpeeds[k] = std::clamp(trot[k], 0.0f, 12.0f);
        m_impl->settings.runSpeeds[k] = std::clamp(run[k], 0.0f, 12.0f);
    }
}
void TrackingGetUpAmpTraining3D::setRecoveryNatural(float weight) { m_impl->settings.recoveryNatural = std::max(0.0f, weight); }
void TrackingGetUpAmpTraining3D::setRiseHandoff(float headHeight) { m_impl->settings.riseHandoffHead = std::max(0.0f, headHeight); }
void TrackingGetUpAmpTraining3D::setIdleHandoff(float blendSeconds, float blendReach) {
    m_impl->settings.idleBlendSeconds = std::max(0.0f, blendSeconds);
    m_impl->settings.idleBlendReach = std::min(1.0f, blendReach);
}
void TrackingGetUpAmpTraining3D::setIdleImitation(float weight) { m_impl->settings.idleImitationWeight = std::max(0.0f, weight); }
void TrackingGetUpAmpTraining3D::setRunAssist(float assist) { m_impl->settings.runAssist = std::min(assist, 1.0f); }
void TrackingGetUpAmpTraining3D::setManeuverWeights(std::array<float, 7> weights) { m_impl->settings.maneuverWeights = weights; }
void TrackingGetUpAmpTraining3D::setManeuvers(float fraction, int evaluationManeuver) {
    m_impl->settings.maneuverFraction = std::clamp(fraction, 0.0f, 1.0f);
    m_impl->settings.evaluationManeuver = std::clamp(evaluationManeuver, -1, 12);
}
void TrackingGetUpAmpTraining3D::setDrillAssist(std::span<const float> levels) {
    for (std::size_t k = 0; k < m_impl->settings.drillAssist.size(); ++k)
        m_impl->settings.drillAssist[k] = k < levels.size() ? std::min(levels[k], 1.0f) : -1.0f;
}
void TrackingGetUpAmpTraining3D::setManeuverAssist(float assist) { m_impl->settings.maneuverAssist = std::min(assist, 1.0f); }

void TrackingGetUpAmpTraining3D::setLocomotionAssist(float assist) {
    m_impl->settings.locomotionAssist = std::clamp(assist, 0.0f, 1.0f);
}

void TrackingGetUpAmpTraining3D::setTeacherShare(float share) {
    m_impl->settings.teacherShare = std::clamp(share, 0.0f, 1.0f);
}

void TrackingGetUpAmpTraining3D::setTeacherSettings(std::array<float, 8> speeds, std::array<float, 8> studentSpeeds,
    std::array<float, 5> takeover) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.teacherSpeeds = speeds;
    s.teacherStudentSpeeds = studentSpeeds;
    s.teacherTakeoverTilt = takeover[0];
    s.teacherTakeoverTiltRate = takeover[1];
    s.teacherTakeoverChest = takeover[2];
    s.teacherTakeoverTravel = takeover[3];
    s.teacherTakeoverSeconds = takeover[4];
}

std::size_t TrackingGetUpAmpTraining3D::teacherTargetWidth() const { return 2 * actionSize() + 3; }

void TrackingGetUpAmpTraining3D::teacherTargets(std::span<float> out) const {
    const std::size_t width = teacherTargetWidth(), n = actionSize();
    if (out.size() < environmentCount() * width) throw std::invalid_argument("rotulos da professora: buffer pequeno");
    for (std::size_t i = 0; i < environmentCount(); ++i) {
        const Impl::World& w = m_impl->worlds[i];
        float* row = out.data() + i * width;
        for (std::size_t k = 0; k < n; ++k) {
            row[k] = k < w.label.size() ? w.label[k] : 0.0f;
            row[n + k] = k < w.labelRaw.size() ? w.labelRaw[k] : 0.0f;
        }
        row[2 * n] = w.labelValid ? 1.0f : 0.0f;
        row[2 * n + 1] = w.labelDrove ? 1.0f : 0.0f;
        row[2 * n + 2] = w.labelTakeover ? 1.0f : 0.0f;
    }
}

void TrackingGetUpAmpTraining3D::reset(std::uint64_t seed) {
    Impl& impl = *m_impl;
    trackingPullWorlds3D(*impl.scheduler, impl.workers, impl.worlds.size(), [&impl, seed](std::size_t i) {
        Impl::World& w = impl.worlds[i];
        w.rng.seed(static_cast<std::uint32_t>(seed * 2654435761ull + i * 40503ull + 911ull));
        impl.begin(w);
        impl.observe(w);
    });
    impl.episodes.clear();
}

void TrackingGetUpAmpTraining3D::observations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].observation.begin(), m_impl->worlds[i].observation.end(), out.begin() + i * size);
}

void TrackingGetUpAmpTraining3D::finalObservations(std::span<float> out) const {
    const std::size_t size = observationSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].finalObservation.begin(), m_impl->worlds[i].finalObservation.end(),
            out.begin() + i * size);
}

void TrackingGetUpAmpTraining3D::step(std::span<const float> actions, std::span<float> rewards,
    std::span<std::uint8_t> terminated, std::span<std::uint8_t> truncated) {
    Impl& impl = *m_impl;
    const std::size_t n = impl.worlds.size();
    const std::size_t a = actionSize();
    if (actions.size() < n * a || rewards.size() < n || terminated.size() < n || truncated.size() < n)
        throw std::invalid_argument("levantar livre: buffers pequenos no step");
    trackingPullWorlds3D(*impl.scheduler, impl.workers, n, [&](std::size_t i) {
        impl.decide(impl.worlds[i], i, actions.subspan(i * a, a), rewards[i], terminated[i], truncated[i]);
    });
}

void TrackingGetUpAmpTraining3D::ampPairs(std::span<float> out) const {
    const std::size_t size = 2 * ampSize();
    for (std::size_t i = 0; i < m_impl->worlds.size(); ++i)
        std::copy(m_impl->worlds[i].ampPair.begin(), m_impl->worlds[i].ampPair.end(), out.begin() + i * size);
}

std::size_t TrackingGetUpAmpTraining3D::buildMotionMatching(const std::vector<std::string>& names,
    const std::vector<std::string>& schemas, const std::vector<std::vector<std::string>>& clips,
    const std::vector<std::array<float, 3>>& biases, const std::vector<int>& categories, std::size_t mainGroup) {
    Impl& impl = *m_impl;
    const std::size_t count = names.size();
    if (schemas.size() != count || clips.size() != count || biases.size() != count || categories.size() != count || count == 0 || count > 24)
        throw std::invalid_argument("buildMotionMatching: listas de tamanhos diferentes (1 a 24 grupos)");
    impl.motionReader.profile = impl.profile;
    impl.motionReader.pelvis = impl.rootLink;
    impl.motionReader.feet[0] = impl.footLinks[0];
    impl.motionReader.feet[1] = impl.footLinks[1];
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i)
        if (impl.profile->links[i].id == "UpperChest") impl.motionReader.chest = static_cast<std::uint32_t>(i);
    impl.motionSources.clear();
    std::vector<MotionMatchingDatabaseGroup3D> groups;
    std::vector<std::vector<std::size_t>> references(count);
    for (std::size_t g = 0; g < count; ++g) {
        MotionMatchingDatabaseGroup3D group;
        group.name = names[g];
        group.schema = schemas[g] == "default" ? MotionMatchingSchema3D::gaspDefault()
            : schemas[g] == "stop" ? MotionMatchingSchema3D::gaspStop()
            : schemas[g] == "idle" ? MotionMatchingSchema3D::gaspIdle()
            : schemas[g] == "unreal" ? MotionMatchingSchema3D::unrealDefault()
            : throw std::invalid_argument("buildMotionMatching: esquema desconhecido " + schemas[g]);
        group.baseCostBias = biases[g][0];
        group.loopingCostBias = biases[g][1];
        group.continuingCostBias = biases[g][2];
        for (const std::string& clip : clips[g]) {
            const auto it = std::find(impl.clipNames.begin(), impl.clipNames.end(), clip);
            if (it == impl.clipNames.end()) throw std::invalid_argument("buildMotionMatching: clipe nao carregado: " + clip);
            const std::size_t r = static_cast<std::size_t>(it - impl.clipNames.begin());
            impl.motionSources.push_back(std::make_unique<TrackingMotionSource>(*impl.references[r], impl.clips[r]->loops, impl.motionReader));
            group.sources.push_back(impl.motionSources.back().get());
            references[g].push_back(r);
        }
        groups.push_back(std::move(group));
    }
    impl.motionDb = std::make_unique<MotionMatchingDatabase3D>(std::move(groups));
    impl.motionRefs = references;
    impl.motionCategory = categories;
    impl.motionMainGroup = std::min(mainGroup, count - 1);
    impl.motionQueryOffset.assign(count, 0);
    impl.motionQueryWidth = 0;
    for (std::size_t g = 0; g < count; ++g) {
        impl.motionQueryOffset[g] = impl.motionQueryWidth;
        impl.motionQueryWidth += impl.motionDb->featureSize(g);
    }
    // Os pares do juiz de cada quadro (o instante e uma decisao depois, como os pares dos clipes).
    const std::size_t a = ampSize();
    const float step = TicksPerDecision * TickSeconds;
    impl.motionPairs.assign(count, {});
    const std::size_t workers = std::max(1u, std::min(8u, std::thread::hardware_concurrency()));
    for (std::size_t g = 0; g < count; ++g) {
        std::vector<float>& pairs = impl.motionPairs[g];
        pairs.assign((impl.motionDb->groupEnd(g) - impl.motionDb->groupBegin(g)) * 2 * a, 0.0f);
        std::vector<std::thread> threads;
        for (std::size_t k = 0; k < workers; ++k) threads.emplace_back([&, g, k] {
        TrackingClipFrame3D first, second;
        for (std::size_t i = impl.motionDb->groupBegin(g) + k; i < impl.motionDb->groupEnd(g); i += workers) {
            const MotionMatchingEntry3D& e = impl.motionDb->entries()[i];
            const TrackingClipReference3D& reference = *impl.references[references[g][e.source]];
            reference.frame(e.timeSeconds, {}, 0.0f, first);
            reference.frame(e.timeSeconds + step, {}, 0.0f, second);
            float* out = pairs.data() + (i - impl.motionDb->groupBegin(g)) * 2 * a;
            impl.features->compute(first.links, std::span<float>(out, a));
            impl.features->compute(second.links, std::span<float>(out + a, a));
        }
        });
        for (std::thread& t : threads) t.join();
    }
    for (Impl::World& w : impl.worlds) {
        w.motionQuery.assign(impl.motionQueryWidth, 0.0f);
        w.motionValid = false;
    }
    return impl.motionDb->size();
}

void TrackingGetUpAmpTraining3D::setKinematicMotion(bool on) { m_impl->kinematicMotion = on; }

void TrackingGetUpAmpTraining3D::setStarDrill(int mode) { m_impl->starDrill = std::clamp(mode, 0, 2); }

void TrackingGetUpAmpTraining3D::setClipSequence(bool on) {
    Impl& impl = *m_impl;
    impl.clipSequence = on;
    if (!on) return;
    impl.motionReader.profile = impl.profile;
    impl.motionReader.pelvis = impl.rootLink;
    impl.motionReader.feet[0] = impl.footLinks[0];
    impl.motionReader.feet[1] = impl.footLinks[1];
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i)
        if (impl.profile->links[i].id == "UpperChest") impl.motionReader.chest = static_cast<std::uint32_t>(i);
    // As viradas (trote e corrida): o instante em que o rumo da raiz faz metade do giro total e o em que faz 95%.
    impl.sequenceTurns.clear();
    for (const char* gait : { "trote_run_turn_", "corrida_turn_" })
        for (const char* side : { "l", "r" })
            for (const char* angle : { "045", "090", "135", "180" })
            for (const char* foot : { "_lfoot", "_rfoot" }) {
                const std::size_t c = impl.clipIndex(std::string(gait) + side + "_" + angle + foot);
                if (c == std::numeric_limits<std::size_t>::max()) continue;
                const float duration = impl.references[c]->durationSeconds();
                const float y0 = impl.clipRoot(c, 0.0f).second, total = impl.clipRoot(c, duration).second - y0;
                float half = duration * 0.5f, end = duration;
                bool halfFound = false;
                for (int k = 1; k <= 120; ++k) {
                    const float t = duration * static_cast<float>(k) / 120.0f;
                    const float done = std::abs(total) > 1.0e-3f ? (impl.clipRoot(c, t).second - y0) / total : 1.0f;
                    if (!halfFound && done >= 0.5f) {
                        half = t;
                        halfFound = true;
                    }
                    if (done >= 0.95f) {
                        end = t;
                        break;
                    }
                }
                impl.sequenceTurns.push_back({ c, std::stof(angle), side[0] == 'l', half, end, gait[0] == 'c' });
            }
    // O tronco de cima (o abdome e tudo abaixo dele na arvore: peito, pescoco, cabeca, bracos) - o que gira para a vista.
    impl.sequenceUpper.assign(impl.profile->links.size(), false);
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i)
        if (impl.profile->links[i].id == "Abdomen") impl.sequenceAbdomen = i;
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i)
        for (int k = static_cast<int>(i); k >= 0; k = impl.profile->links[static_cast<std::size_t>(k)].parentIndex)
            if (static_cast<std::size_t>(k) == impl.sequenceAbdomen) impl.sequenceUpper[i] = true;
    // As curvas (trote): o raio = a velocidade sobre o giro da raiz no clipe.
    for (auto& arcs : impl.sequenceArcs) arcs.clear();
    for (const char* size : { "small", "tight", "wide" })
        for (int side = 0; side < 2; ++side) {
            const std::size_t c = impl.clipIndex(std::string("trote_run_arc_") + size + (side == 0 ? "_l" : "_r"));
            if (c == std::numeric_limits<std::size_t>::max()) continue;
            const float duration = impl.references[c]->durationSeconds();
            const auto a = impl.clipRoot(c, 0.0f), b = impl.clipRoot(c, duration);
            const Vec3 d = b.first - a.first;
            const float yaw = std::abs(b.second - a.second);
            // (A corda e o angulo: raio = corda / (2 sen(angulo/2)).)
            const float radius = yaw > 1.0e-3f ? std::hypot(d.x, d.y) / (2.0f * std::sin(0.5f * yaw)) : 1.0e3f;
            impl.sequenceArcs[side].push_back({ c, radius });
        }
}
std::size_t TrackingGetUpAmpTraining3D::kinematicJumps(std::size_t world) const { return m_impl->worlds.at(world).mmJumps; }
std::string TrackingGetUpAmpTraining3D::kinematicClip(std::size_t world) const {
    const Impl::World& w = m_impl->worlds.at(world);
    if (m_impl->clipSequence && w.mmPlaying && !m_impl->kinematicMotion)
    {
        // (Com MATTER_SEQ_DEPURAR, o tempo do clipe, o rumo do personagem, a vista, o giro do peito e as diferencas da troca.)
        if (std::getenv("MATTER_SEQ_DEPURAR") == nullptr)
            return "tipo " + std::to_string(w.seqKind) + ": " + m_impl->clipNames.at(w.mmGroup);
        char text[256];
        std::snprintf(text, sizeof text, " t %.2f rumo %.0f vista %.0f peito %.0f giro %.0f pos %.2f vel %.2f incl %.0f w %.0f",
            w.mmTime, w.mmRootYaw * 57.2958f, m_impl->faceDirection(w) * 57.2958f, w.seqWarpAngle * 57.2958f,
            w.seqOffsetSpin.length() * 57.2958f, w.mmOffsetRootPosition.length(), w.seqOffsetRootVelocity.length(),
            w.seqLean * 57.2958f, w.seqYawRate * 57.2958f);
        return "tipo " + std::to_string(w.seqKind) + ": " + m_impl->clipNames.at(w.mmGroup) + text;
    }
    if (!m_impl->motionDb || !w.mmPlaying) return {};
    return m_impl->motionDb->group(w.mmGroup).name + ":" + m_impl->clipNames[m_impl->motionRefs[w.mmGroup][w.mmSource]];
}

void TrackingGetUpAmpTraining3D::setMotionMovement(float maxAcceleration, float brakingDeceleration, float groundFriction,
    float brakingFriction, float rotationRadiansPerSecond) {
    MotionMatchingMovementSettings3D& m = m_impl->motionMovement;
    m.maxAcceleration = maxAcceleration;
    m.brakingDeceleration = brakingDeceleration;
    m.groundFriction = groundFriction;
    m.brakingFriction = brakingFriction;
    m.rotationRadiansPerSecond = rotationRadiansPerSecond;
}

std::size_t TrackingGetUpAmpTraining3D::motionGroupCount() const { return m_impl->motionDb ? m_impl->motionDb->groupCount() : 0; }
std::size_t TrackingGetUpAmpTraining3D::motionGroupEntries(std::size_t g) const {
    return m_impl->motionDb->groupEnd(g) - m_impl->motionDb->groupBegin(g);
}
std::size_t TrackingGetUpAmpTraining3D::motionFeatureSize(std::size_t g) const { return m_impl->motionDb->featureSize(g); }
std::size_t TrackingGetUpAmpTraining3D::motionQueryOffset(std::size_t g) const { return m_impl->motionQueryOffset.at(g); }
std::size_t TrackingGetUpAmpTraining3D::motionQueryWidth() const { return m_impl->motionQueryWidth; }
std::size_t TrackingGetUpAmpTraining3D::motionMainGroup() const { return m_impl->motionMainGroup; }

void TrackingGetUpAmpTraining3D::motionGroupFeatures(std::size_t g, std::span<float> out) const {
    const MotionMatchingDatabase3D& db = *m_impl->motionDb;
    const std::size_t d = db.featureSize(g);
    for (std::size_t i = db.groupBegin(g); i < db.groupEnd(g); ++i)
        db.weighted(g, db.features(i), out.subspan((i - db.groupBegin(g)) * d, d));
}
void TrackingGetUpAmpTraining3D::motionGroupPairs(std::size_t g, std::span<float> out) const {
    std::copy(m_impl->motionPairs.at(g).begin(), m_impl->motionPairs.at(g).end(), out.begin());
}
void TrackingGetUpAmpTraining3D::motionGroupEntryInfo(std::size_t g, std::span<float> addends, std::span<float> sources,
    std::span<float> times) const {
    const MotionMatchingDatabase3D& db = *m_impl->motionDb;
    for (std::size_t i = db.groupBegin(g); i < db.groupEnd(g); ++i) {
        const MotionMatchingEntry3D& e = db.entries()[i];
        addends[i - db.groupBegin(g)] = e.costAddend;
        sources[i - db.groupBegin(g)] = static_cast<float>(e.source);
        times[i - db.groupBegin(g)] = e.timeSeconds;
    }
}
void TrackingGetUpAmpTraining3D::motionQueries(std::span<float> out) const {
    const Impl& impl = *m_impl;
    const std::size_t width = impl.motionQueryWidth + 2;
    for (std::size_t i = 0; i < impl.worlds.size(); ++i) {
        const Impl::World& w = impl.worlds[i];
        float* row = out.data() + i * width;
        std::copy(w.motionQuery.begin(), w.motionQuery.end(), row);
        row[impl.motionQueryWidth] = static_cast<float>(w.motionGroups);
        row[impl.motionQueryWidth + 1] = w.motionValid ? 1.0f : 0.0f;
    }
}

void TrackingGetUpAmpTraining3D::setLyingStarts(const std::vector<std::string>& standClips,
    const std::vector<std::string>& runClips) {
    Impl& impl = *m_impl;
    const auto indices = [&](const std::vector<std::string>& names) {
        std::vector<std::size_t> out;
        for (const std::string& name : names) {
            const auto it = std::find(impl.clipNames.begin(), impl.clipNames.end(), name);
            if (it == impl.clipNames.end()) throw std::runtime_error("setLyingStarts: clipe nao carregado: " + name);
            out.push_back(static_cast<std::size_t>(it - impl.clipNames.begin()));
        }
        return out;
    };
    impl.lyingStand = indices(standClips);
    impl.lyingRun = indices(runClips);
}

void TrackingGetUpAmpTraining3D::setClipGetUp(const std::vector<std::string>& clips, float fraction, float midStartFraction,
    float afterSeconds, float startNoise, float softness, float comVelocityWidth) {
    Impl& impl = *m_impl;
    if (clips.empty()) throw std::runtime_error("setClipGetUp: sem clipes");
    impl.clipGetUpClips.clear();
    impl.clipGetUpDirection.assign(clips.size(), 0.0f);
    impl.clipGetUpSpeed.assign(clips.size(), 0.0f);
    impl.clipGetUpFacing.assign(clips.size(), 0.0f);
    TrackingClipFrame3D a, b;
    for (std::size_t k = 0; k < clips.size(); ++k) {
        const auto it = std::find(impl.clipNames.begin(), impl.clipNames.end(), clips[k]);
        if (it == impl.clipNames.end()) throw std::runtime_error("setClipGetUp: clipe nao carregado: " + clips[k]);
        const std::size_t r = static_cast<std::size_t>(it - impl.clipNames.begin());
        impl.clipGetUpClips.push_back(r);
        const TrackingClipReference3D& reference = *impl.references[r];
        const float d = reference.durationSeconds(), span = std::min(0.2f, std::max(1.0e-3f, d));
        reference.frame(d - span, {}, 0.0f, a);
        reference.frame(d, {}, 0.0f, b);
        impl.clipGetUpDirection[k] = std::atan2(b.rootPosition.y - a.rootPosition.y, b.rootPosition.x - a.rootPosition.x);
        impl.clipGetUpSpeed[k] = std::hypot(b.centerOfMass.x - a.centerOfMass.x, b.centerOfMass.y - a.centerOfMass.y) / span;
        impl.clipGetUpFacing[k] = trackingBodyYaw3D(impl.profile->links[impl.rootLink], b.links[impl.rootLink]);
    }
    TrackingGetUpAmpSettings3D& s = impl.settings;
    s.clipGetUpFraction = std::clamp(fraction, 0.0f, 1.0f);
    s.clipGetUpMidStartFraction = std::clamp(midStartFraction, 0.0f, 1.0f);
    s.clipGetUpAfterSeconds = std::max(0.0f, afterSeconds);
    s.clipGetUpStartNoise = std::max(0.0f, startNoise);
    s.clipGetUpSoftness = std::max(0.05f, softness);
    s.clipGetUpComVelocityWidth = std::max(0.05f, comVelocityWidth);
}
std::vector<float> TrackingGetUpAmpTraining3D::clipGetUpProbe(std::uint32_t exercise, float referenceTime, float bodyTime,
    float lift, float verticalSpeed, float twist, float pitch, float legs, float travelScale, float speedScale, float armOffset) const {
    const Impl& impl = *m_impl;
    if (exercise >= impl.clipGetUpClips.size()) throw std::runtime_error("clipGetUpProbe: sem o exercicio");
    const TrackingClipReference3D& reference = *impl.references[impl.clipGetUpClips[exercise]];
    TrackingClipFrame3D r, b, start;
    reference.frame(referenceTime, {}, 0.0f, r);
    reference.frame(bodyTime, {}, 0.0f, b, twist, pitch, legs);
    reference.frame(0.0f, {}, 0.0f, start);
    const Vec3 shift { (travelScale - 1.0f) * (b.rootPosition.x - start.rootPosition.x),
        (travelScale - 1.0f) * (b.rootPosition.y - start.rootPosition.y), lift };
    for (PhysicsBodyState3D& link : b.links) {
        link.position = link.position + shift;
        link.linearVelocity = { link.linearVelocity.x * speedScale, link.linearVelocity.y * speedScale,
            link.linearVelocity.z + verticalSpeed };
    }
    b.centerOfMass = b.centerOfMass + shift;
    if (armOffset != 0.0f) {
        // (O braco fora: o ombro direito deslocado armOffset nos eixos de balanco e o cotovelo metade disso; o corpo refeito pelas
        // coordenadas, com a velocidade das juntas zero.)
        std::vector<Vec3> coordinates = b.coordinates, zero(b.coordinates.size());
        for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
            if (impl.profile->links[i].id == "RightUpperArm") coordinates[i] = coordinates[i] + Vec3 { 0.0f, armOffset, armOffset };
            if (impl.profile->links[i].id == "RightForearm") coordinates[i] = coordinates[i] + Vec3 { 0.5f * armOffset, 0.0f, 0.0f };
        }
        const PhysicsBodyState3D& root = b.links[impl.rootLink];
        rlBodyLinkStates3D(*impl.profile, root.position, b.rootOrientation, root.linearVelocity, root.angularVelocity, coordinates,
            zero, b.links);
        b.centerOfMass = rlBodyCenterOfMass3D(*impl.profile, b.links);
    }
    const TrackingClipGetUp3D::Terms t = TrackingClipGetUp3D::score(*impl.contract, *impl.profile, b.links, r,
        { impl.settings.clipGetUpSoftness, impl.settings.clipGetUpComVelocityWidth, impl.settings.clipGetUpParts }, 0.0f);
    const int far = TrackingClipGetUp3D::far(*impl.contract, *impl.profile, b.links, r);
    return { t.pose, t.velocity, t.endEffector, t.centerOfMass, t.comVelocity, t.total, static_cast<float>(far), t.hop, t.trunk };
}
void TrackingGetUpAmpTraining3D::setClipGetUpParts(bool on) { m_impl->settings.clipGetUpParts = on; }
void TrackingGetUpAmpTraining3D::setClipGetUpVariants(const std::vector<float>& variants) { m_impl->clipGetUpVariants = variants; }
void TrackingGetUpAmpTraining3D::setClipGetUpDeliver(const std::vector<float>& seconds) { m_impl->clipGetUpDeliver = seconds; }
void TrackingGetUpAmpTraining3D::setClipGetUpEmphasis(const std::vector<float>& emphasis) { m_impl->clipGetUpEmphasis = emphasis; }
void TrackingGetUpAmpTraining3D::setClipGetUpAgility(const std::vector<float>& agility) { m_impl->clipGetUpAgility = agility; }
void TrackingGetUpAmpTraining3D::setClipGetUpAdaptive(bool on) { m_impl->settings.clipGetUpAdaptive = on; }
void TrackingGetUpAmpTraining3D::setAdaptiveSpeedBonus(float bonus) { m_impl->settings.adaptiveSpeedBonus = std::max(0.0f, bonus); }
void TrackingGetUpAmpTraining3D::setCalmHandoff(float calmSpeed, float maxSeconds, float maxSecondsRun) {
    m_impl->settings.handoffCalmSpeed = std::max(0.0f, calmSpeed);
    m_impl->settings.handoffMaxSeconds = std::max(0.1f, maxSeconds);
    m_impl->settings.handoffMaxSecondsRun = std::max(0.1f, maxSecondsRun);
}
void TrackingGetUpAmpTraining3D::setGetUpPoseVariety(float fraction) {
    m_impl->settings.getUpPoseVariety = std::clamp(fraction, 0.0f, 1.0f);
}
void TrackingGetUpAmpTraining3D::setRunPostRise(float speedReward, float settleSeconds) {
    m_impl->settings.runSpeedReward = std::max(0.0f, speedReward);
    m_impl->settings.runSettleSeconds = std::max(0.0f, settleSeconds);
}
void TrackingGetUpAmpTraining3D::setAdaptiveTiming(float timeMinimum, float timeMaximum, float runImpulse) {
    m_impl->settings.adaptiveTimeMin = std::max(0.0f, timeMinimum);
    m_impl->settings.adaptiveTimeMax = std::max(m_impl->settings.adaptiveTimeMin, timeMaximum);
    m_impl->settings.runImpulseBonus = std::max(0.0f, runImpulse);
}
void TrackingGetUpAmpTraining3D::setAdaptiveTimingRun(float timeMinimum, float timeMaximum) {
    m_impl->settings.adaptiveTimeMinRun = std::max(0.0f, timeMinimum);
    m_impl->settings.adaptiveTimeMaxRun = std::max(m_impl->settings.adaptiveTimeMinRun, timeMaximum);
}
void TrackingGetUpAmpTraining3D::setAdaptiveTimeFall(float seconds) { m_impl->settings.adaptiveTimeFall = std::max(0.1f, seconds); }
void TrackingGetUpAmpTraining3D::setAdaptiveTimeFire(float slack) { m_impl->settings.adaptiveTimeFireSlack = slack; }
void TrackingGetUpAmpTraining3D::setGetUpFireReleaseAtFirm(bool on) { m_impl->settings.fireReleaseAtFirm = on; }
void TrackingGetUpAmpTraining3D::setGetUpPalm(float weight) { m_impl->settings.getUpPalmWeight = std::max(0.0f, weight); }
void TrackingGetUpAmpTraining3D::setIdlePelvisLevel(float weight) {
    Impl& impl = *m_impl;
    impl.settings.idlePelvisLevelWeight = std::max(0.0f, weight);
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
        if (impl.profile->links[i].id == "LeftThigh") impl.thighLinks[0] = i;
        if (impl.profile->links[i].id == "RightThigh") impl.thighLinks[1] = i;
    }
}
std::vector<float> TrackingGetUpAmpTraining3D::getUpFormTable(std::uint32_t exercise) const {
    std::vector<float> out;
    if (exercise < m_impl->clipGetUpForm.size())
        for (const auto& s : m_impl->clipGetUpForm[exercise]) out.insert(out.end(), { s.head, s.pitch, s.roll, s.twist, s.lean, s.gaze, s.tilt });
    return out;
}
std::vector<float> TrackingGetUpAmpTraining3D::getUpFormNow(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    const auto s = impl.upperForm(w.body->links, impl.floorUnder(w));
    return { s.head, s.pitch, s.roll, s.twist, s.lean, s.gaze, s.tilt, impl.getUpFormScore(w, w.body->links, impl.floorUnder(w), s.head > 1.4f),
        impl.palmScore(w.body->links, impl.floorUnder(w)), impl.pelvisRoll(w.body->links) };
}
void TrackingGetUpAmpTraining3D::setGetUpForm(float weight) {
    Impl& impl = *m_impl;
    if (weight > 0.0f && impl.clipGetUpClips.empty()) throw std::runtime_error("setGetUpForm: chame setClipGetUp antes");
    impl.settings.getUpFormWeight = std::max(0.0f, weight);
    impl.clipGetUpForm.assign(impl.clipGetUpClips.size(), {});
    impl.clipGetUpStand.assign(impl.clipGetUpClips.size(), { 0.0f, 0.0f });
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
        if (impl.profile->links[i].id == "LeftHand") impl.handLinksForm[0] = i;
        if (impl.profile->links[i].id == "RightHand") impl.handLinksForm[1] = i;
    }
    TrackingClipFrame3D f;
    for (std::size_t k = 0; k < impl.clipGetUpClips.size() && weight > 0.0f; ++k) {
        const TrackingClipReference3D& reference = *impl.references[impl.clipGetUpClips[k]];
        for (float t = 0.0f; t <= reference.durationSeconds(); t += 1.0f / 30.0f) {
            reference.frame(t, {}, 0.0f, f);
            impl.clipGetUpForm[k].push_back(impl.upperForm(f.links, 0.0f));
        }
        const std::size_t tail = std::min<std::size_t>(9, impl.clipGetUpForm[k].size());
        for (std::size_t i = impl.clipGetUpForm[k].size() - tail; i < impl.clipGetUpForm[k].size(); ++i) {
            impl.clipGetUpStand[k][0] += impl.clipGetUpForm[k][i].head / static_cast<float>(tail);
            impl.clipGetUpStand[k][1] += impl.clipGetUpForm[k][i].tilt / static_cast<float>(tail);
        }
    }
}
void TrackingGetUpAmpTraining3D::setGetUpLegForm(float weight, std::uint32_t mask) {
    Impl& impl = *m_impl;
    if (weight > 0.0f && impl.clipGetUpClips.empty()) throw std::runtime_error("setGetUpLegForm: chame setClipGetUp antes");
    impl.settings.getUpLegWeight = std::max(0.0f, weight);
    impl.settings.getUpLegMask = mask;
    for (std::size_t i = 0; i < impl.profile->links.size(); ++i) {
        if (impl.profile->links[i].id == "LeftShin") impl.leftLegLinks[0] = i;
        if (impl.profile->links[i].id == "LeftFoot") impl.leftLegLinks[1] = i;
    }
    impl.clipGetUpLeg.assign(impl.clipGetUpClips.size(), {});
    TrackingClipFrame3D f;
    for (std::size_t k = 0; k < impl.clipGetUpClips.size() && weight > 0.0f; ++k) {
        const TrackingClipReference3D& reference = *impl.references[impl.clipGetUpClips[k]];
        for (float t = 0.0f; t <= reference.durationSeconds(); t += 1.0f / 30.0f) {
            reference.frame(t, {}, 0.0f, f);
            impl.clipGetUpLeg[k].push_back(impl.leftLegForm(f.links, 0.0f));
        }
    }
}
float TrackingGetUpAmpTraining3D::getUpLegErrorNow(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    return impl.getUpLegError(w, w.body->links, impl.floorUnder(w));
}
std::vector<float> TrackingGetUpAmpTraining3D::getUpLegFormNow(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    std::vector<float> out;
    const float err = impl.getUpLegError(w, w.body->links, impl.floorUnder(w), &out);
    out.push_back(err);
    return out;
}
void TrackingGetUpAmpTraining3D::setGetUpPace(float weight, float seconds) {
    Impl& impl = *m_impl;
    if (weight > 0.0f && impl.clipGetUpForm.empty()) throw std::runtime_error("setGetUpPace: chame setGetUpForm antes (a tabela da cabeca do clipe)");
    impl.settings.getUpPaceWeight = std::max(0.0f, weight);
    impl.settings.getUpPaceSeconds = std::max(0.3f, seconds);
}
void TrackingGetUpAmpTraining3D::setSettleCalm(float weight) { m_impl->settings.settleCalmWeight = std::clamp(weight, 0.0f, 1.0f); }
void TrackingGetUpAmpTraining3D::setAdaptiveRetry(bool on) { m_impl->settings.adaptiveRetry = on; }
void TrackingGetUpAmpTraining3D::setFireRunHeadwindCap(float fraction) { m_impl->settings.fireRunHeadwindCap = std::max(0.0f, fraction); }
void TrackingGetUpAmpTraining3D::setGetUpFireEarly(float share, float window) {
    m_impl->settings.fireEarlyShare = std::clamp(share, 0.0f, 1.0f);
    m_impl->settings.fireEarlyWindow = std::max(0.1f, window);
}
void TrackingGetUpAmpTraining3D::setGetUpFireExtras(int shoveSeries, float legShare, float wallShare, float wallMinimum,
    float wallMaximum, float turnShare, float turnSeconds, float afterStandShare, float slipShare, float grabShare, float grabFraction,
    float seriesGapMin, float seriesGapMax, float shoveSecondsMin, float shoveSecondsMax, float shoveNewtonsMin, float shoveNewtonsMax) {
    TrackingGetUpAmpSettings3D& s = m_impl->settings;
    s.fireShoveNewtonsMin = std::max(0.0f, shoveNewtonsMin);
    s.fireShoveNewtonsMax = std::max(s.fireShoveNewtonsMin, shoveNewtonsMax);
    s.fireShoveSecondsMin = std::max(0.02f, shoveSecondsMin);
    s.fireShoveSecondsMax = std::max(s.fireShoveSecondsMin, shoveSecondsMax);
    s.fireSeriesGapMin = std::max(0.05f, seriesGapMin);
    s.fireSeriesGapMax = std::max(s.fireSeriesGapMin, seriesGapMax);
    s.fireSlipShare = std::clamp(slipShare, 0.0f, 1.0f);
    s.fireGrabShare = std::clamp(grabShare, 0.0f, 1.0f);
    s.fireGrabFraction = std::max(0.0f, grabFraction);
    s.fireAfterStandShare = std::clamp(afterStandShare, 0.0f, 1.0f);
    s.fireShoveSeries = std::max(1, shoveSeries);
    s.fireLegShare = std::clamp(legShare, 0.0f, 1.0f);
    s.fireWallShare = std::clamp(wallShare, 0.0f, 1.0f);
    s.fireWallMin = std::max(0.0f, wallMinimum);
    s.fireWallMax = std::max(s.fireWallMin, wallMaximum);
    s.fireTurnShare = std::clamp(turnShare, 0.0f, 1.0f);
    s.fireTurnSeconds = std::max(0.3f, turnSeconds);
}
void TrackingGetUpAmpTraining3D::setGetUpArena(float minimumMeters, float maximumMeters) {
    m_impl->settings.getUpArenaMin = std::max(0.0f, minimumMeters);
    m_impl->settings.getUpArenaMax = std::max(m_impl->settings.getUpArenaMin, maximumMeters);
}
void TrackingGetUpAmpTraining3D::setAdaptiveSettle(float seconds, float release) {
    m_impl->settings.adaptiveSettleSeconds = std::max(0.0f, seconds);
    m_impl->settings.fireReleaseSeconds = std::max(0.02f, release);
}
void TrackingGetUpAmpTraining3D::setGetUpFire(const std::vector<float>& shares, const std::vector<float>& levels, bool exam) {
    Impl& impl = *m_impl;
    impl.getUpFireShares = { 1.0f, 0.0f, 0.0f, 0.0f };
    for (std::size_t k = 0; k < std::min<std::size_t>(4, shares.size()); ++k) impl.getUpFireShares[k] = std::max(0.0f, shares[k]);
    impl.getUpFireLevels = levels;
    impl.getUpFireExam = exam;
    impl.getUpFireAbove = shares.size() >= 5 ? std::clamp(shares[4], 0.0f, 1.0f) : 0.0f;
}
void TrackingGetUpAmpTraining3D::setClipGetUpStartBias(const std::vector<float>& bias) { m_impl->clipGetUpStartBias = bias; }
void TrackingGetUpAmpTraining3D::setClipGetUpWeights(const std::vector<float>& weights) {
    if (weights.size() != m_impl->clipGetUpClips.size()) throw std::runtime_error("setClipGetUpWeights: um peso por exercicio");
    float sum = 0.0f;
    m_impl->settings.clipGetUpWeights.assign(weights.size(), 0.0f);
    for (std::size_t k = 0; k < weights.size(); ++k) {
        m_impl->settings.clipGetUpWeights[k] = std::max(0.0f, weights[k]);
        sum += m_impl->settings.clipGetUpWeights[k];
    }
    if (sum <= 0.0f) m_impl->settings.clipGetUpWeights.clear();
}
std::vector<float> TrackingGetUpAmpTraining3D::clipGetUpEnds() const {
    std::vector<float> out;
    for (std::size_t k = 0; k < m_impl->clipGetUpClips.size(); ++k) {
        out.push_back(m_impl->references[m_impl->clipGetUpClips[k]]->durationSeconds());
        out.push_back(m_impl->clipGetUpSpeed[k]);
        out.push_back(m_impl->clipGetUpDirection[k]);
    }
    return out;
}

// (07/11, ele: "esses clipes do Unreal vao ser o nosso juiz de estilo a partir de agora") Os clipes so do juiz, no fim da
// lista (nenhum indice da tarefa muda; as quedas sorteiam so os da tarefa): com o deslocamento da raiz preservado, a velocidade
// dos pares sai da propria raiz do clipe (travel zerado).
std::size_t TrackingGetUpAmpTraining3D::addStyleClips(const std::vector<std::string>& names) {
    Impl& impl = *m_impl;
    // (Os arquivos lidos em paralelo - centenas de JSON; a lista montada na ordem.)
    std::vector<std::string> fresh;
    for (const std::string& name : names)
        if (std::find(impl.clipNames.begin(), impl.clipNames.end(), name) == impl.clipNames.end()
            && std::find(fresh.begin(), fresh.end(), name) == fresh.end())
            fresh.push_back(name);
    std::vector<std::unique_ptr<AnimationClip3D>> loaded(fresh.size());
    std::vector<std::string> errors(fresh.size());
    {
        const std::size_t workers = std::max(1u, std::min(8u, std::thread::hardware_concurrency()));
        std::vector<std::thread> threads;
        for (std::size_t k = 0; k < workers; ++k) threads.emplace_back([&, k] {
            for (std::size_t i = k; i < fresh.size(); i += workers) {
                try {
                    loaded[i] = std::make_unique<AnimationClip3D>(
                        loadAnimationClip3D(impl.settings.assetsDirectory + "/animations/clips/" + fresh[i] + ".matteranim.json"));
                } catch (const std::exception& error) {
                    errors[i] = fresh[i] + ": " + error.what();
                }
            }
        });
        for (std::thread& t : threads) t.join();
    }
    for (const std::string& error : errors)
        if (!error.empty()) throw std::runtime_error("addStyleClips: " + error);
    std::size_t added = 0;
    for (std::size_t i = 0; i < fresh.size(); ++i) {
        const std::string& name = fresh[i];
        impl.clips.push_back(std::move(loaded[i]));
        impl.references.push_back(std::make_unique<TrackingClipReference3D>(*impl.profile, *impl.clips.back()));
        impl.clipNames.push_back(name);
        // (07/11: os ciclos do juiz nas velocidades-meta - tools/animation/clips/ciclos_meta.py - sao no lugar: o par do juiz
        // leva a velocidade deles, como a dos passos da tarefa.)
        Impl::Travel travel;
        if (name.starts_with("ciclo_") && impl.clips.back()->loops) {
            travel.direction = impl.clips.back()->travelDirectionRadians;
            travel.speed = impl.clips.back()->nominalSpeedMetersPerSecond;
        }
        impl.travel.push_back(travel);
        ++added;
    }
    return added;
}

std::vector<float> TrackingGetUpAmpTraining3D::referencePairs() const { return referencePairsOf({}); }

std::vector<float> TrackingGetUpAmpTraining3D::referencePairsOf(const std::vector<std::string>& clips) const {
    const Impl& impl = *m_impl;
    const std::size_t a = ampSize();
    const float step = TicksPerDecision * TickSeconds;
    std::vector<float> out;
    std::vector<float> now(a), next(a);
    TrackingClipFrame3D first, second;
    for (std::size_t r = 0; r < impl.references.size(); ++r) {
        // (03/11 23h30: o juiz da locomocao so com os clipes dela - o unico juiz aprovava se jogar como numa queda.)
        if (!clips.empty() && std::find(clips.begin(), clips.end(), impl.clipNames[r]) == clips.end()) continue;
        if (clips.empty() && r >= impl.taskClipCount) continue;
        // (Os da corrida do rastreio ficam fora do juiz geral - o da recuperacao.)
        if (clips.empty()
            && std::find(std::begin(TrackedRunClips), std::end(TrackedRunClips), impl.clipNames[r]) != std::end(TrackedRunClips))
            continue;
        if (clips.empty() && r == impl.startClipIndex && impl.startClipTrackedOnly) continue;
        if (r == impl.launchExitIndex) continue;
        if (clips.empty() && (impl.clipNames[r] == TrackedTurnClips[0] || impl.clipNames[r] == TrackedTurnClips[1])) continue;
        const auto& reference = impl.references[r];
        const Impl::Travel& travel = impl.travel[r];
        const float duration = reference->durationSeconds();
        // (Os clipes no lugar: a velocidade do deslocamento que a fonte tinha, para a frente do clipe, somada a todos
        // os links - as caracteristicas usam a da raiz; as posicoes vao em relacao a ela.)
        const auto moving = [&](float t, TrackingClipFrame3D& frame) {
            float speed = travel.speed;
            if (travel.rampDisplacement > 0.0f && duration > 0.0f)
                speed = 2.0f * travel.rampDisplacement * std::clamp(t, 0.0f, duration) / (duration * duration);
            if (speed <= 0.0f) return;
            const Vec3 v { std::cos(travel.direction) * speed, std::sin(travel.direction) * speed, 0.0f };
            for (PhysicsBodyState3D& s : frame.links) s.linearVelocity = s.linearVelocity + v;
        };
        // (A arrancada agachada pesa mais: a referencia do levantar em movimento, "um levantar preparado ja indo".)
        const int copies = r == impl.sprintStartIndex ? SprintStartPairWeight : 1;
        for (float t = 0.0f; t + step <= duration + 1e-5f; t += 1.0f / 120.0f) {
            reference->frame(t, {}, 0.0f, first);
            reference->frame(t + step, {}, 0.0f, second);
            moving(t, first);
            moving(t + step, second);
            impl.features->compute(first.links, now);
            impl.features->compute(second.links, next);
            for (int c = 0; c < copies; ++c) {
                out.insert(out.end(), now.begin(), now.end());
                out.insert(out.end(), next.begin(), next.end());
            }
        }
    }
    return out;
}

std::vector<TrackingGetUpAmpEpisode3D> TrackingGetUpAmpTraining3D::takeEpisodes() {
    std::lock_guard lock(m_impl->episodeMutex);
    std::vector<TrackingGetUpAmpEpisode3D> out;
    out.swap(m_impl->episodes);
    return out;
}

const RagdollState3D& TrackingGetUpAmpTraining3D::body(std::size_t world) const {
    return *m_impl->worlds.at(world).body;
}
std::span<const RagdollDriveTarget3D> TrackingGetUpAmpTraining3D::driveTargets(std::size_t world) const {
    return m_impl->worlds.at(world).drives;
}

const RagdollProfile3D& TrackingGetUpAmpTraining3D::profile() const { return *m_impl->profile; }

TrackingGetUpAmpWorldView3D TrackingGetUpAmpTraining3D::worldView(std::size_t world) const {
    const Impl::World& w = m_impl->worlds.at(world);
    TrackingGetUpAmpWorldView3D view;
    view.startKind = w.startKind;
    view.hasTarget = w.startKind == 3 || w.startKind == 5;
    view.target = w.target;
    if (w.getUpGo && !w.body->links.empty()) {
        // (12/11 19h10, o levantar e ir: "ir ate um ponto proximo... marcado no mapa") O ponto, a linha ate ele (um ponto a cada ~1 m) e o
        // texto: o lado, o normal ou correndo, a distancia e o que falta.
        const Vec3 root = w.body->links[m_impl->rootLink].position;
        const float dx = w.goTarget.x - root.x, dy = w.goTarget.y - root.y, left = std::hypot(dx, dy);
        view.hasTarget = true;
        view.target = w.goTarget;
        const int marks = std::max(1, static_cast<int>(left));
        for (int k = 1; k <= marks; ++k)
            view.path.push_back({ root.x + dx * static_cast<float>(k) / static_cast<float>(marks),
                                  root.y + dy * static_cast<float>(k) / static_cast<float>(marks), 0.0f });
        static constexpr const char* Sides[4] { "de costas", "de bruços", "correndo de costas", "correndo de bruços" };
        char text[160];
        std::snprintf(text, sizeof text, "levantar e ir · %s · %s · ponto a %.1f m · falta %.1f m%s", Sides[std::min<std::size_t>(3, w.clipExercise)],
            m_impl->clipRunExercise(w.clipExercise) ? "depois correndo" : "depois no trote", w.goDistance, left,
            w.goArrivedAt >= 0.0f ? " · chegou" : w.clipHandedAt >= 0.0f ? " · locomoção" : " · levantar");
        view.commandText = text;
    }
    view.commandSpeed = w.commandSpeed;
    if (w.commandSpeed > 0.0f) {
        const float move = m_impl->moveDirection(w);
        view.moveVelocity = { std::cos(move) * w.commandSpeed, std::sin(move) * w.commandSpeed, 0.0f };
    }
    view.hasFace = w.locomotion || w.taught;
    view.faceYaw = w.faceYaw;
    view.pushing = w.pushTicks > 0;
    view.pushForce = w.pushForce;
    view.clipGetUp = w.clipGetUp;
    if (w.locoFire && !w.clipGetUp) {
        // (12/11 16h30) O fogo da locomocao para o desenho do laboratorio: os mesmos campos do levantar; o orcamento e o episodio.
        const Impl& impl = *m_impl;
        view.locoFire = true;
        view.locoFireLevel = w.locoFireLevel;
        // (16h45, o cartao seguia "empurrao continuo" o episodio todo) O tipo de agora: 2 o continuo agindo, 1 a pancada recente ou
        // pendente, 0 nenhum (o agarrao, se houver, vem no fireGrabLink).
        const bool holdNow = impl.fireHoldNow(w).length() > 1.0f;
        const bool shoveNow = !w.fireShoveDone || (w.fireShoveAt >= 0.0f && w.time - w.fireShoveAt < 1.5f);
        view.fireKind = (holdNow ? 2 : 0) | (shoveNow ? 1 : 0);
        view.fireDirection = w.fireDirection;
        view.fireSize = w.fireSize;
        view.fireShoveSize = w.fireShoveSize;
        view.fireHold = impl.fireHoldNow(w);
        view.fireShoveAt = w.fireShoveAt;
        view.fireShoveDone = w.fireShoveDone;
        view.fireShoveForce = w.fireShoveForce;
        view.fireShoveSeconds = w.fireShoveSeconds;
        view.fireShoveLink = w.fireShoveLink;
        const float yaw = trackingBodyYaw3D(impl.profile->links[impl.rootLink], w.body->links[impl.rootLink])
            + static_cast<float>(w.fireShoveDirection % 8) * Pi / 4.0f;
        view.fireAim = { std::cos(yaw), std::sin(yaw), 0.0f };
        view.time = w.time;
        view.fireBudget = impl.episodeLimit(w);
        view.downAt = w.fireDownAt;
        view.weight = impl.profile->totalMassKg * 9.81f;
        view.fireLevelHold = Impl::LocoFireHold0 + (Impl::LocoFireHold1 - Impl::LocoFireHold0) * w.locoFireLevel;
        view.fireLevelShove = impl.locoShoveNewtons(w, 1.0f) * 0.3f / std::max(1.0f, impl.profile->totalMassKg);
        view.fireGrabLink = impl.fireGrabActive(w) ? w.fireGrabLink : -1;
        view.fireGrabForce = impl.fireGrabActive(w) ? w.fireGrabForce : Vec3 {};
        view.fireSlip = w.fireSlip;
    }
    if (w.clipGetUp) {
        // (12/11 05h30) O fogo do levantar para o desenho do laboratorio - ver TrackingGetUpAmpWorldView3D::fireKind.
        const Impl& impl = *m_impl;
        view.fireKind = w.fireKind;
        view.fireTiming = w.fireTiming;
        view.fireDirection = w.fireDirection;
        view.fireAbovePitch = w.fireAbovePitch;
        view.fireFraction = w.fireFraction;
        view.fireSize = w.fireSize;
        view.fireShoveSize = w.fireShoveSize;
        view.fireHold = impl.fireHoldNow(w);
        view.fireShoveAt = w.fireShoveAt;
        view.fireShoveDone = w.fireShoveDone;
        view.fireShoveForce = w.fireShoveForce;
        view.fireShoveSeconds = w.fireShoveSeconds;
        view.fireShoveLink = w.fireShoveLink;
        const float yaw = trackingBodyYaw3D(impl.profile->links[impl.rootLink], w.body->links[impl.rootLink])
            + static_cast<float>(w.fireDirection % 8) * Pi / 4.0f;
        view.fireAim = { std::cos(yaw) * std::cos(w.fireAbovePitch), std::sin(yaw) * std::cos(w.fireAbovePitch), -std::sin(w.fireAbovePitch) };
        view.time = w.time;
        const float deliverAt = w.clipExercise < impl.clipGetUpDeliver.size() && impl.clipGetUpDeliver[w.clipExercise] > 0.0f
            ? impl.clipGetUpDeliver[w.clipExercise] : impl.references[w.clipReference]->durationSeconds();
        view.fireBudget = deliverAt + Impl::AdaptiveSpareSeconds;
        view.upAt = w.upAt;
        view.deliveredAt = w.clipDeliveredAt;
        view.downAt = w.fireDownAt;
        view.weight = impl.profile->totalMassKg * 9.81f;
        if (impl.getUpFireLevels.size() >= 4 * (w.clipExercise + 1)) {
            view.fireLevelHold = impl.getUpFireLevels[4 * w.clipExercise];
            view.fireLevelShove = w.fireTiming > 0 ? impl.getUpFireLevels[4 * w.clipExercise + static_cast<std::size_t>(w.fireTiming)] : 0.0f;
            view.fireGrabLink = impl.fireGrabActive(w) ? w.fireGrabLink : -1;
            view.fireGrabForce = impl.fireGrabActive(w) ? w.fireGrabForce : Vec3 {};
            view.fireSlip = w.fireSlip;
        }
    }
    if (Impl::isStar(w)) {
        view.star = true;
        view.starPoints.push_back(w.star.center);
        for (int k = 0; k < 8; ++k) view.starPoints.push_back(m_impl->starPoint(w, k));
        view.starFrom = w.star.at;
        view.starGoal = w.star.goal;
        view.starNext = w.star.next;
        view.starLegs = w.star.legs;
        view.starPhase = w.star.phase;
        view.starChest = w.maneuver;
        view.starTurnDegrees = w.star.nextTurn * 180.0f / Pi;
        view.starArrive = m_impl->starArrival(w, w.commandSpeed);
        Impl::starPath(w.star.from, m_impl->starPoint(w, w.star.goal), w.star.bend, view.starLegPath);
        Impl::starPath(m_impl->starPoint(w, w.star.goal), m_impl->starPoint(w, w.star.next), w.star.nextBend, view.starNextPath);
        view.starBend = w.star.bend;
        view.starNextBend = w.star.nextBend;
    }
    if (Impl::isCommand(w)) {
        static constexpr const char* Keys[8] { "W", "W+A", "A", "S+A", "S", "S+D", "D", "W+D" };
        static constexpr const char* States[4] { "andando", "freando", "parado", "girando no lugar" };
        const auto& c = w.command;
        char text[160];
        std::snprintf(text, sizeof text, "comando · %s%s · %s · vista %+.0f° do peito · fase %d/4 · %d eventos",
            c.key >= 0 ? Keys[c.key] : "sem tecla", c.key >= 0 && c.run ? " + Shift" : "", States[std::clamp(c.state, 0, 3)],
            wrapAngle(c.target - w.faceYaw) * 180.0f / Pi, c.phase, c.events);
        view.commandText = text;
    }
    if (Impl::isCurve(w) && w.commandSpeed > 0.0f && !w.course.empty()) {
        // (Fixos no mundo: os pontos do percurso que a referencia ainda nao passou - o quanto ela andou e a velocidade pedida
        // vezes o tempo - ate 14 m a frente.)
        // (Somem quando o CORPO passa: o ponto mais perto dele numa janela de 8 m em volta de onde a referencia esta - a curva em S
        // volta perto de si mesma.)
        // (04/11 14h: a janela ia pelo pedido vezes o tempo - lento, ele ficava atras dela e as marcas apareciam longe; agora
        // o ponto mais perto do corpo, o mesmo da medida.)
        const std::size_t nearest = w.courseAt;
        for (std::size_t k = nearest + 1; k < w.course.size() && k < nearest + 15; ++k) view.path.push_back(w.course[k]);
    }
    return view;
}

void TrackingGetUpAmpTraining3D::scenerySegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    out.clear();
    for (const Impl::World::SceneryBox& box : w.boxes) {
        if (box.arena || box.slip || !w.scene->contains(box.body)) continue;
        const PhysicsBodyState3D state = w.scene->bodyState(box.body);
        const Vec3 center = state.position + state.orientation.rotate(box.local);
        const Quaternion orientation = (state.orientation * box.orientation).normalized();
        const Vec3 half { std::min(box.half.x, 2.5f), std::min(box.half.y, 2.5f), box.half.z };
        const auto corner = [&](int k) {
            return center + orientation.rotate({ (k & 1) ? half.x : -half.x, (k & 2) ? half.y : -half.y, (k & 4) ? half.z : -half.z });
        };
        for (int k = 0; k < 8; ++k)
            for (const int bit : { 1, 2, 4 })
                if ((k & bit) == 0) out.emplace_back(corner(k), corner(k | bit));
    }
}

void TrackingGetUpAmpTraining3D::arenaSegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const {
    const Impl::World& w = m_impl->worlds.at(world);
    out.clear();
    for (const Impl::World::SceneryBox& box : w.boxes) {
        if (!box.arena || !w.scene->contains(box.body)) continue;
        const PhysicsBodyState3D state = w.scene->bodyState(box.body);
        const Quaternion q = state.orientation;
        // A face de dentro (para o nascimento: -x local), do chao ao topo.
        const Vec3 face = state.position + q.rotate({ -box.half.x, 0.0f, 0.0f });
        const float bottom = state.position.z - box.half.z + 0.5f, top = state.position.z + box.half.z;
        const auto at = [&](float along, float z) {
            const Vec3 p = face + q.rotate({ 0.0f, along, 0.0f });
            return Vec3 { p.x, p.y, z };
        };
        const float length = box.half.y;
        for (const float z : { bottom + 0.02f, bottom + 0.5f * (top - bottom), top })
            out.emplace_back(at(-length, z), at(length, z));
        for (float along = -length; along <= length + 1.0e-3f; along += 2.0f)
            out.emplace_back(at(along, bottom), at(along, top));
    }
}

void TrackingGetUpAmpTraining3D::slipSegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const {
    const Impl::World& w = m_impl->worlds.at(world);
    out.clear();
    for (const Impl::World::SceneryBox& box : w.boxes) {
        if (!box.slip || !w.scene->contains(box.body)) continue;
        const PhysicsBodyState3D state = w.scene->bodyState(box.body);
        const float z = state.position.z + box.half.z + 0.01f;
        for (float a = -box.half.x; a <= box.half.x + 1.0e-3f; a += 1.0f) {
            out.emplace_back(Vec3 { state.position.x + a, state.position.y - box.half.y, z }, Vec3 { state.position.x + a, state.position.y + box.half.y, z });
            out.emplace_back(Vec3 { state.position.x - box.half.x, state.position.y + a, z }, Vec3 { state.position.x + box.half.x, state.position.y + a, z });
        }
    }
}

void TrackingGetUpAmpTraining3D::sceneryBodies(std::size_t world, std::vector<const RagdollState3D*>& out) const {
    const Impl::World& w = m_impl->worlds.at(world);
    out.clear();
    for (const RagdollHandle3D extra : w.extras)
        if (w.scene->contains(extra))
            if (const RagdollState3D* other = w.scene->ragdollStateView(extra); other != nullptr) out.push_back(other);
}

bool TrackingGetUpAmpTraining3D::referencePose(std::size_t world, std::vector<PhysicsBodyState3D>& out) const {
    const Impl& impl = *m_impl;
    Impl::World& w = const_cast<Impl::World&>(impl.worlds.at(world));
    if (!impl.clipTracked(w)) return false;
    TrackingClipFrame3D frame;
    impl.referenceFrame(w, 0.0f, true, frame);
    out = frame.links;
    return out.size() == w.body->links.size();
}

std::string TrackingGetUpAmpTraining3D::referenceLines(std::size_t world) const {
    const Impl& impl = *m_impl;
    Impl::World& w = const_cast<Impl::World&>(impl.worlds.at(world));
    if (!impl.clipTracked(w)) return {};
    TrackingClipFrame3D frame;
    float time = impl.trackTime(w, 0.0f);
    if (impl.settings.idleClipAnchor) {
        // (15/11) O mesmo desenho do fantasma do laboratorio (referencePose do principal, com a ancora do proprio desenho).
        if (w.time < w.viewTime) w.viewAnchored = false, w.viewClipAt = -1.0f;
        w.viewTime = w.time;
        if (w.clipGetUp && w.clipHandedAt < 0.0f) {
            const TrackingClipReference3D& reference = *impl.references[w.clipReference];
            time = std::min(reference.durationSeconds(), impl.clipTime(w));
            reference.frame(time, w.clipGround, w.clipYaw, frame);
            if (impl.rootLink < frame.links.size()) {
                w.viewClipPelvis = frame.links[impl.rootLink].position;
                w.viewClipAt = w.time;
            }
            w.viewAnchored = false;
        } else {
            impl.referenceFrame(w, 0.0f, true, frame);
            if (impl.rootLink < frame.links.size()) {
                Vec3 at = w.body->links[impl.rootLink].position;
                if (impl.idleTracked(w)) {
                    if (w.pushTicks > 0 || w.time < w.lastPushEnd + 1.5f || w.fireKind != 0 || w.locoFire) {
                        w.viewAnchored = false;
                    } else {
                        if (!w.viewAnchored) {
                            w.viewAnchor = w.viewClipAt >= 0.0f && w.time - w.viewClipAt < 0.5f ? w.viewClipPelvis : at;
                            w.viewAnchored = true;
                            w.viewClipAt = -1.0f;
                        }
                        at = w.viewAnchor;
                    }
                } else {
                    w.viewAnchored = false;
                }
                impl.pelvisTo(frame, at.x, at.y);
            }
        }
    } else if (w.clipGetUp && w.clipHandedAt < 0.0f) {
        const TrackingClipReference3D& reference = *impl.references[w.clipReference];
        time = std::min(reference.durationSeconds(), impl.clipTime(w));
        const Vec3 root = w.body->links[impl.rootLink].position;
        if (impl.settings.clipUnderPelvis) reference.frame(time, w.clipGround, w.clipYaw, frame);
        else reference.frame(time, Vec3 { root.x, root.y, impl.floorUnder(w) }, w.clipYaw, frame);
    } else {
        impl.referenceFrame(w, 0.0f, true, frame);
    }
    std::ostringstream out;
    out << "Q " << impl.clipNames[w.trackClip] << ' ' << time << ' ' << impl.assistLevel(w) << ' '
        << (w.locomotionControl ? 1 : 0) << ' ' << w.commandSpeed << ' ' << w.moveYaw << '\n';
    for (std::size_t i = 0; i < impl.profile->links.size() && i < frame.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = impl.profile->links[i];
        const PhysicsBodyState3D& state = frame.links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    return out.str();
}

std::string TrackingGetUpAmpTraining3D::frameLines(std::size_t world) const {
    const Impl& impl = *m_impl;
    const Impl::World& w = impl.worlds.at(world);
    const RagdollState3D& body = *w.body;
    std::ostringstream out;
    for (std::size_t i = 0; i < impl.profile->links.size() && i < body.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = impl.profile->links[i];
        const PhysicsBodyState3D& state = body.links[i];
        const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
        const Quaternion orientation = (state.orientation * link.collider.localOrientation).normalized();
        out << (link.collider.shape == RagdollColliderShape3D::Box ? "B " : "C ") << link.id << ' ' << center.x << ' '
            << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' ' << orientation.z
            << ' ' << orientation.w << '\n';
    }
    for (const RagdollContactPoint3D& contact : body.contacts)
        out << "P " << contact.position.x << ' ' << contact.position.y << ' ' << contact.position.z << ' '
            << contact.linkIndex << ' ' << contact.normal.x << ' ' << contact.normal.y << ' ' << contact.normal.z << ' '
            << contact.normalImpulseNewtonSeconds << '\n';
    // (O cenario: as caixas, "O" - centro, orientacao e meias medidas -, e os corpos soltos, "D", um depois do outro.)
    for (const Impl::World::SceneryBox& box : w.boxes) {
        if (!w.scene->contains(box.body)) continue;
        const PhysicsBodyState3D state = w.scene->bodyState(box.body);
        const Vec3 center = state.position + state.orientation.rotate(box.local);
        const Quaternion orientation = (state.orientation * box.orientation).normalized();
        out << "O " << center.x << ' ' << center.y << ' ' << center.z << ' ' << orientation.x << ' ' << orientation.y << ' '
            << orientation.z << ' ' << orientation.w << ' ' << box.half.x << ' ' << box.half.y << ' ' << box.half.z << '\n';
    }
    for (const RagdollHandle3D extra : w.extras) {
        const RagdollState3D* other = w.scene->contains(extra) ? w.scene->ragdollStateView(extra) : nullptr;
        if (other == nullptr) continue;
        for (std::size_t i = 0; i < impl.profile->links.size() && i < other->links.size(); ++i) {
            const RagdollLinkDefinition3D& link = impl.profile->links[i];
            const PhysicsBodyState3D& state = other->links[i];
            const Vec3 center = state.position + state.orientation.rotate(link.collider.localPosition);
            out << "D " << link.id << ' ' << center.x << ' ' << center.y << ' ' << center.z << " 0 0 0 1\n";
        }
    }
    // (O alvo do levantar em movimento, como o "A" das folhas da locomocao.)
    if (w.startKind == 3 || w.startKind == 5)
        out << "A " << w.target.x << ' ' << w.target.y << " 0 " << ArriveRadius << '\n';
    return out.str();
}

} // namespace MatterEngine
