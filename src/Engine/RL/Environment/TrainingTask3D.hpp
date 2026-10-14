#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
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
    // O giro: a velocidade de giro do corpo contra a pedida (o giro do clipe
    // mais o que alinha a direcao em que ele anda com o alvo), em sino.
    float targetTurn = 0.0f;
    // Segurar o rumo pedido (com o comando de olhar): o erro do link que fica
    // virado para ele (target.facingLink) em sino (target.facingSigma).
    float facingHold = 0.0f;
    // Acertar a velocidade pedida (o comando de velocidade): a velocidade em
    // direcao ao alvo contra a pedida (ja com a freada antes do ponto), em sino.
    float speedMatch = 0.0f;
    float fall = 10.0f;
    // O total de cada passo, menos a queda, nunca fica negativo (o
    // "only_positive_rewards" do legged_gym): tentar e errar nunca vale menos
    // que ficar parado, e cair nunca compensa as penalidades de seguir vivo.
    bool positiveOnly = false;
};

inline constexpr std::array<const char*, 29> TrainingRewardComponentNames3D {
    "alive", "upright", "headHeight", "comOverSupport", "still", "trunkTurn", "footSlip", "footDrift", "effort",
    "actionRate", "jointLimit", "posePrior", "poseMatch", "velocity", "yawRate", "feetAirTime", "gaitContact",
    "swingHeight", "gaitReference", "imitationPose", "imitationVelocity", "imitationKeyBodies", "imitationRoot",
    "targetSpeed", "targetReached", "targetTurn", "facingHold", "speedMatch", "fall"
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
    // Um exercicio (a metodologia das 3 areas: os exercicios de uma area
    // misturados no mesmo treino, cada mundo sorteando o seu pelo peso): a
    // tarefa base com o que ele muda por cima. Ele so muda o mundo (os alvos,
    // as notas, os empurroes, a duracao...), nunca o que a rede ve ou faz: a
    // mesma rede serve a todos, e ela nunca sabe o nome do exercicio.
    struct Drill {
        std::string id;
        float weight = 1.0f;
        std::shared_ptr<const TrainingTask3D> task;
    };
    std::vector<Drill> drills;

    std::string id;
    std::string displayName;
    // Aparece na lista de habilidades da tela ("menu": false esconde as tarefas
    // antigas, que ficam no repositorio para os testes e o historico).
    bool listed = true;
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
    // Com residualActions, a acao corrige a pose da referencia: o alvo de cada
    // junta e o da referencia no fim da decisao (e a velocidade dela) mais
    // actionScale vezes a acao. Acao zero segue o clipe com os motores de
    // sempre (nada empurra o corpo): a politica aprende o equilibrio.
    struct Imitation {
        bool enabled = false;
        std::vector<ImitationClip> clips;
        float standingStart = 0.5f;
        bool residualActions = false;
        // Uma biblioteca (todos os clipes em ciclo): o planejador escolhe a cada
        // decisao o clipe que anda mais perto da direcao do alvo, medida a partir
        // de para onde o corpo deve olhar (target.facing; sem ele, a frente do
        // corpo), com histerese e um tempo minimo em cada clipe. Com as 8
        // direcoes (frente, diagonais, lados, costas), e o andar do jogo: o
        // corpo olha para a camera e anda para qualquer lado.
        bool library = false;
        struct Planner {
            float hysteresisDegrees = 10.0f;
            float minimumSeconds = 0.4f;
            // A velocidade (com clipes de varias velocidades, como o trote e o
            // sprint): a pedida pela distancia ate a mira (pares [m, m/s] em
            // ordem, interpolados e presos nas pontas), e o peso dela na
            // escolha: speedWeight por unidade de |ln(v do clipe / v pedida)|,
            // somado ao erro de direcao (rad). 0 = so a direcao.
            std::vector<std::array<float, 2>> speedByDistance;
            float speedWeight = 0.0f;
            // O peso do erro de direcao no custo (1 = so ele). Abaixo de 1, longe
            // vale virar o corpo e correr de frente (o sprint) em vez de ir de
            // costas no trote; perto, a velocidade pedida e baixa e o clipe da
            // direcao continua valendo.
            float directionWeight = 1.0f;
            // Com o comando de olhar valendo: o peso da direcao (o clipe tem de
            // andar para onde o alvo esta em relacao ao peito); < 0 = o de cima.
            float directionWeightFacing = -1.0f;
            // A velocidade pedida (o comando de velocidade da tarefa, com a
            // freada antes do ponto) escolhe o passo; speedByDistance, se houver,
            // tem prioridade.
            bool speedFromCommand = false;
            // Sem o comando de olhar (o peito livre): os clipes de lado e de
            // costas (mais de 60 graus da frente) so valem com o alvo mais
            // perto que isto (m); mais longe ele vira e vai de frente, como um
            // jogador, e enquanto vira a velocidade pedida da escolha cai com
            // o cosseno do angulo ate a mira (no minimo a de chegada): vira
            // andando e acelera ao alinhar (0 = sempre valem).
            float freeBackwardDistance = 0.0f;
        } planner;
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
        // O centro do sorteio do proximo alvo (graus, a partir do rumo do trecho
        // que chega nele): 0 em volta da frente, 180 para voltar (o tiro com
        // volta), sorteado ate nextAngleDegrees dele.
        float nextAngleOffsetDegrees = 0.0f;
        std::vector<float> evaluationAngles { 0.0f };
        float referenceSpeed = 6.0f;
        float speedCap = 1.2f;
        // Passou do alvo: a distancia ate ele passa do mais perto que ele
        // chegou mais isto (m) e o episodio acaba (0 = desligado).
        float missMeters = 0.0f;
        // A distancia sorteada de cada alvo (com maximo acima de 0; senao a
        // distance). Na avaliacao, evaluationPerWorld da a cada mundo o angulo
        // (e a distancia) da sua posicao nas listas, e os alvos seguintes
        // seguem as listas dali.
        TrainingRange3D distanceRange { 0.0f, 0.0f };
        bool evaluationPerWorld = false;
        std::vector<float> evaluationDistances;
        // O giro pedido: turnGain por radiano entre a direcao em que o clipe anda
        // e o alvo, ate turnMaximum (rad/s); turnSigma e o sino da nota.
        float turnGain = 2.0f;
        float turnMaximum = 2.5f;
        float turnSigma = 0.5f;
        // Para onde olhar (o jogo: a camera), separado do alvo: com facing, o
        // episodio sorteia esse rumo (o do corpo no comeco), o giro pedido o
        // segura (turnGain por radiano de desvio) e a direcao do alvo e medida a
        // partir dele. facingDriftRadians (rad/s): o quanto ele passeia, como uma
        // camera virando devagar (0 = parado).
        bool facing = false;
        float facingDriftRadians = 0.0f;
        // O que fica virado para o comando: "pelvis" ou "chest" (o peito, o
        // UpperChest: o jogo de pes leva de lado e de costas com o peito para a
        // camera, e o quadril abre o que a passada pedir). facingFraction: a
        // fracao dos episodios de treino com o comando; nos outros ele anda
        // livre (o que ja sabe nao se perde). Na avaliacao, todos com o comando
        // (ou o que a bateria pedir).
        std::string facingLink = "pelvis";
        float facingFraction = 1.0f;
        float facingSigma = 0.5f;
        // Dos episodios com o comando: a fatia em que ele gira (facingDrift); o
        // resto fica fixo no rumo do comeco (o "norte").
        float facingDriftFraction = 1.0f;
        // O giro em trechos (o jogador girando a camera): com facingDriftTurns,
        // cada trecho sorteia quantas voltas dar (0,5 = meia) e a velocidade
        // (rad/s, facingDriftSpeed) e o sentido; acabou, sorteia o seguinte (o
        // sentido pode repetir: mais de uma volta seguida). Sem ele, o giro
        // antigo (ate facingDriftRadians, trocando a cada 2-4 s).
        TrainingRange3D facingDriftTurns { 0.0f, 0.0f };
        TrainingRange3D facingDriftSpeed { 0.4f, 1.2f };
        // O comando de velocidade: cada trecho ate um alvo tem uma velocidade
        // pedida, sorteada numa faixa (andar, trote, corrida, sprint), e a nota
        // e acertar ela (speedMatch), nao correr mais. A pedida desce antes do
        // ponto como uma freada (brakeAcceleration, m/s2) ate a velocidade de
        // chegada, que depende da virada para o proximo (cheia em linha reta,
        // minimumArrivalSpeed numa volta de 180 graus). Com o comando de olhar,
        // capByDirection limita a pedida ao que os clipes fazem naquela direcao
        // em relacao ao peito (nao se pede sprint de lado).
        struct SpeedBand {
            std::string name;
            TrainingRange3D speed;
            TrainingRange3D distance;
            float weight = 1.0f;
        };
        struct SpeedCommand {
            bool enabled = false;
            std::vector<SpeedBand> bands;
            float brakeAcceleration = 4.0f;
            float minimumArrivalSpeed = 1.0f;
            float sigma = 0.6f;
            float redrawOnReturn = 0.5f;
            bool capByDirection = true;
        } speedCommand;
        // O desenho dos alvos (biblioteca): "random" (cada um em volta do
        // anterior) ou "star" (a estrela: do centro a uma das 8 direcoes, de
        // volta ao centro, e assim por diante; a distancia da faixa sorteada).
        std::string pattern = "random";
        std::uint32_t starDirections = 8;
        float starJitterDegrees = 8.0f;
        // Os varios pontos (biblioteca): quantos alvos depois do de agora ja
        // estao no mapa e na observacao (0 a 3; com 0, cada um nasce so na
        // chegada do anterior). Com eles, cada alvo sai do anterior, pelo rumo do
        // trecho que chega nele.
        std::uint32_t lookahead = 0;
        // O tracado de corrida (com lookahead): o planejador e a nota do giro
        // miram o ponto do disco do alvo do lado do proximo, a racingLine
        // (fracao do raio) do centro. O ponto fica dentro do disco (seguir ate
        // ele e chegar), e ele ja entra virado para o seguinte. 0 = o centro.
        float racingLine = 0.0f;
        // Os percursos fixos da avaliacao: os pontos em metros no referencial do
        // comeco (x para a frente, y para a esquerda), em ordem; no fim, o
        // percurso recomeca deslocado do ultimo ponto (um fechado volta ao
        // comeco). Cada mundo pega um pela sua posicao.
        struct Course {
            std::string name;
            std::vector<std::array<float, 2>> points;
        };
        std::vector<Course> evaluationCourses;
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

    // As baterias da tarefa (os nomes das ligadas, na ordem do arquivo; a
    // tela mostra a primeira da habilidade).
    std::vector<std::string> evaluationBatteries;

    std::uint64_t hash = 0;
};

[[nodiscard]] TrainingTask3D loadTrainingTask3D(const std::filesystem::path& definitionPath);

} // namespace MatterEngine
