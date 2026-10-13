#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace MatterEngine {

// Pesos da recompensa, por decisao (30 Hz). As parcelas positivas sao notas
// entre 0 e 1; as negativas, penalidades. Os nomes batem com os do arquivo da
// tarefa e com as colunas do treinador.
struct TrainingRewardWeights3D {
    float alive = 1.0f;
    float upright = 0.5f;
    float headHeight = 0.5f;
    float comOverSupport = 0.5f;
    float still = 0.3f;
    // O tronco girando (penalidade): a velocidade angular da pelve e do peito
    // ao quadrado. O "still" so ve o centro de massa, e girar o tronco quase
    // nao o move. Quadratica (e nao uma nota em sino) para ensinar tambem com
    // o ruido do treino, que faz o tronco girar a ~1,5 rad/s. Uma parcela nova
    // comeca desligada (as tarefas antigas nao mudam sem pedir).
    float trunkTurn = 0.0f;
    float footSlip = 0.5f;
    float footDrift = 1.0f;
    float effort = 0.1f;
    float actionRate = 0.2f;
    float jointLimit = 0.2f;
    float posePrior = 0.1f;
    // A postura pedida junta por junta (nota em sino do erro quadratico medio
    // das juntas, sigma ~10 graus): numa postura comandada os bracos tambem
    // contam. O posePrior, normalizado pelo curso, quase nao cobra um antebraco
    // 40 graus fora. Desligada se a tarefa nao pede.
    float poseMatch = 0.0f;
    // Andar (comando de velocidade): seguir a velocidade pedida no plano (no
    // referencial do rumo) e o giro pedido, notas em sino de sigma 0,5; e o
    // tempo de cada pe no ar (o passo de verdade, nao arrastar), so com
    // comando de andar. Desligadas nas tarefas de ficar de pe.
    float velocity = 0.0f;
    float yawRate = 0.0f;
    float feetAirTime = 0.0f;
    // O relogio de passada (andando): o pe de balanco no ar na hora certa da
    // fase (alternando; no chao, ou o de apoio no ar, desconta) e subindo ate
    // a altura do passo. Parado, com os dois pes plantados, nunca paga.
    float gaitContact = 0.0f;
    float swingHeight = 0.0f;
    // A referencia de passada (gait.reference): as juntas das pernas perto do
    // alvo senoidal da fase, em sino da distancia (rad), menos um tanto dela.
    float gaitReference = 0.0f;
    // A imitacao (secao imitation, o DeepMimic): notas em sino de 0 a 1 contra
    // a referencia no mesmo instante. A pose (as juntas e a inclinacao da
    // pelve), a velocidade das juntas, cabeca, maos e pes em relacao a pelve
    // (no referencial do rumo) e a raiz (a altura e a velocidade da pelve).
    float imitationPose = 0.0f;
    float imitationVelocity = 0.0f;
    float imitationKeyBodies = 0.0f;
    float imitationRoot = 0.0f;
    // O alvo (secao target): a velocidade em direcao a ele (linear: 1 na
    // velocidade de referencia, ate o teto) e, ao chegar, a velocidade media
    // do trecho sobre a de referencia (uma vez por alvo).
    float targetSpeed = 0.0f;
    float targetReached = 0.0f;
    float fall = 10.0f;
    // O total de cada passo, menos a queda, nunca fica negativo (o
    // "only_positive_rewards" do legged_gym): tentar e errar nunca vale menos
    // que ficar parado, e cair nunca compensa as penalidades de seguir vivo.
    bool positiveOnly = false;
};

inline constexpr std::array<const char*, 26> TrainingRewardComponentNames3D {
    "alive", "upright", "headHeight", "comOverSupport", "still", "trunkTurn", "footSlip", "footDrift", "effort",
    "actionRate", "jointLimit", "posePrior", "poseMatch", "velocity", "yawRate", "feetAirTime", "gaitContact",
    "swingHeight", "gaitReference", "imitationPose", "imitationVelocity", "imitationKeyBodies", "imitationRoot",
    "targetSpeed", "targetReached", "fall"
};

struct TrainingRange3D {
    float minimum = 0.0f;
    float maximum = 0.0f;
};

// Uma postura que a tarefa pede (pelo id no corpo) e a fatia dos episodios
// de treino com ela. Uma habilidade nova mantem uma fatia das antigas (o
// ensaio: aprende a nova sem esquecer).
struct TrainingPostureWeight3D {
    std::string id;
    float weight = 1.0f;
    // So parado nesta postura (o ensaio de uma habilidade de ficar de pe numa
    // tarefa de andar: pedidos de andar nela a fariam esquecer o parado).
    bool standingOnly = false;
};

// Um trecho do comando de velocidade numa bateria: a partir de seconds, a
// velocidade pedida (m/s, para frente e para a esquerda, no referencial do
// rumo) e o giro (rad/s, anti-horario).
struct TrainingCommandStep3D {
    float seconds = 0.0f;
    float forward = 0.0f;
    float lateral = 0.0f;
    float yawRate = 0.0f;
};

// Uma tarefa de treino (assets/rl/tasks/<id>.json): o que se pede ao corpo,
// como o episodio comeca e acaba e o quanto o mundo varia.
struct TrainingTask3D {
    std::string id;
    std::string displayName;
    std::filesystem::path definitionPath;
    std::filesystem::path bodyPath;
    std::uint32_t actionRepeat = 4;
    float episodeSeconds = 20.0f;
    std::string groundMaterial = "grass";
    // As posturas pedidas; sem a lista, so a pose base ("natural").
    std::vector<TrainingPostureWeight3D> postures { { "natural", 1.0f, false } };
    // A distancia entre os mundos de uma cena (e o chao cobre cada um com
    // metade dela em volta): andando, ninguem sai do chao nem tromba no vizinho.
    float worldSpacingMeters = 16.0f;
    TrainingRewardWeights3D reward;

    // O comando de velocidade (andar). Sem ele, so parado. Os maximos sao o
    // alvo de cada direcao; os limites de agora sao os do curriculo (o
    // treinador os sobe conforme a politica acompanha) e nunca passam do
    // maximo. Costas, lados e giro ficam prontos: basta um maximo acima de 0.
    struct Commands {
        TrainingRange3D resampleSeconds { 4.0f, 6.0f };
        // A fatia dos sorteios que pede ficar parado (comando zero).
        float standingFraction = 1.0f;
        float forward = 0.0f;
        float backward = 0.0f;
        float lateral = 0.0f;
        float yawRate = 0.0f;
        struct Limits {
            float forward = -1.0f;
            float backward = -1.0f;
            float lateral = -1.0f;
            float yawRate = -1.0f;
        } limits;
        // Com comando de andar, as parcelas de "estar bem" (vivo, postura,
        // cabeca, pose, giro) valem multiplicadas pelo acompanhamento da
        // velocidade: parado sob um pedido de andar nao ganha nada.
        bool gateByTracking = false;
        // A fracao dos episodios de treino com pedido de andar que comeca com o
        // corpo ja na velocidade pedida: parado, ele cai para a frente, e o
        // primeiro passo vira uma reacao (como o de quem e empurrado). O resto
        // sai do parado (o primeiro passo de verdade). Na tarefa, true = 1.
        float startMoving = 0.0f;
        // Medido contra ficar parado: com pedido de andar, a velocidade (e o
        // giro, se pedido) vale 0 parado e 1 seguindo o pedido, e o "estar
        // bem" (gateByTracking) vai junto. Parado nao ganha nada do que paga
        // andar, em qualquer velocidade pedida (no sino puro, parado com 0,3
        // m/s pedidos levava 70%).
        bool relativeToStanding = false;
    } commands;

    // A tarefa tem a secao "teacher" ligada (o professor do aprendizado
    // guiado, em RL/Teacher; os mundos de treino nao a leem).
    bool hasTeacher = false;

    // O relogio de passada (so andando): a fase corre a frequency[0] +
    // frequency[1] * |v| passadas por segundo; cada pe fica no ar metade dela
    // (o esquerdo na primeira, o direito na segunda), com um apoio duplo de
    // doubleSupport (no seno da fase) que vira voo acima de flightSpeed m/s; o
    // pe de balanco sobe ate swingHeight metros.
    //
    // A referencia (receita do humanoid-gym): a perna de balanco segue um alvo
    // no ritmo da fase, a partir da postura; cada junta listada vai a postura
    // mais scale * hipFlexion * o balanco da perna dela (0 no apoio, 1 no meio
    // do voo). E o empurrao para o primeiro passo, nao o jeito de correr: o
    // peso some com a velocidade pedida entre fade[0] e fade[1] m/s.
    struct ReferenceJoint {
        std::string dof;
        std::uint32_t leg = 0;  // 0 esquerda, 1 direita
        float scale = 0.0f;
    };
    struct Gait {
        TrainingRange3D frequency { 0.9f, 0.13f };
        float doubleSupport = 0.1f;
        float flightSpeed = 2.5f;
        float swingHeight = 0.08f;
        float referenceHipFlexion = 0.0f;
        TrainingRange3D referenceFade { 2.0f, 3.5f };
        std::vector<ReferenceJoint> referenceJoints;
    } gait;

    // A imitacao (H3 v7): os trechos de clipe em sequencia, o ultimo em ciclo
    // (RL/Imitation). Metade dos episodios de treino (standingStart) sai do
    // primeiro quadro do primeiro trecho; o resto entra no ultimo numa fase
    // sorteada, ja com a pose e as velocidades do clipe (o RSI do DeepMimic).
    // Os sinos: nota = exp(-escala * erro ao quadrado somado).
    struct ImitationClip {
        std::string id;
        std::filesystem::path path;
    };
    struct Imitation {
        bool enabled = false;
        std::vector<ImitationClip> clips;
        float standingStart = 0.5f;
        float poseScale = 2.0f;
        float velocityScale = 0.1f;
        float keyBodyScale = 40.0f;
        float rootHeightScale = 40.0f;
        float rootVelocityScale = 0.5f;
    } imitation;

    // O alvo (H3 v7): um ponto a distance metros; chegando a radius dele,
    // nasce o proximo, distance adiante. O primeiro fica no rumo do trecho de
    // partida (no treino, sorteado ate firstAngle graus dele); o proximo, ate
    // nextAngle graus da direcao em que ele chegou. A avaliacao usa
    // evaluationAngles (graus, um por alvo, o ultimo se repete).
    struct Target {
        bool enabled = false;
        float distance = 10.0f;
        float radius = 1.0f;
        float firstAngleDegrees = 10.0f;
        float nextAngleDegrees = 30.0f;
        std::vector<float> evaluationAngles { 0.0f };
        float referenceSpeed = 6.0f;
        float speedCap = 1.2f;
    } target;

    struct Termination {
        float minimumHeadHeightFraction = 0.6f;
        float minimumPelvisHeightFraction = 0.5f;
        bool nonSupportContact = true;
        float maximumDistanceFromStart = 5.0f;  // m (o fim e truncado, nao queda)
    } termination;

    struct InitialState {
        float jointNoiseRadians = 0.05f;
        float jointVelocityNoise = 0.2f;
        float rootVelocityNoise = 0.1f;
        float rootAngularVelocityNoise = 0.1f;
        bool randomYaw = true;
    } initialState;

    struct Randomization {
        TrainingRange3D friction { 0.7f, 1.1f };
        TrainingRange3D mass { 0.95f, 1.05f };
        TrainingRange3D strength { 0.9f, 1.1f };
        float actionDelayProbability = 0.5f;
        float observationNoise = 1.0f;
    } randomization;

    struct Pushes {
        bool enabled = false;
        TrainingRange3D intervalSeconds { 2.0f, 4.0f };
        TrainingRange3D durationSeconds { 0.1f, 0.2f };
        TrainingRange3D forceNewtons { 50.0f, 150.0f };
        std::vector<std::string> links { "Pelvis", "UpperChest" };
    } pushes;

    std::uint64_t hash = 0;
};

[[nodiscard]] TrainingTask3D loadTrainingTask3D(const std::filesystem::path& definitionPath);

} // namespace MatterEngine
