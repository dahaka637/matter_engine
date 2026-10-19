#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Tracking/TrackingLeader3D.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// O que o jogador virtual pede (DReCon 6.7): o prototipo do G1 (so de frente,
// de 0 ao teto do prototipo) ou o jogo inteiro (rumo, olhar, giro, velocidade
// e estilo pela tabela do padrao).
enum class TrackingScenario3D : std::uint8_t { Forward = 0, Full = 1 };

// A avaliacao (os portoes): de frente; os 8 rumos em relacao ao olhar (o mundo i
// no rumo i mod 8, de 45 em 45 graus); ou o protocolo de impacto do DReCon (7.1:
// a velocidade dada, trocando de rumo a cada switchSeconds).
enum class TrackingEvaluation3D : std::uint8_t { Forward = 0, Headings = 1, Switches = 2, Idle = 3, Turn = 4, FastTurn = 5 };

// Os exercicios parados (da escada antiga, locomocao_f3f, com as voltas
// completas que o usuario exige): ficar parado no idle com o olhar fixo;
// parado com giro suave (30 a 450 graus a 0,4 a 0,8 rad/s, pausas de 1,5 a
// 4 s); giro rapido parado (90 a 720 graus a 1 a 3,14 rad/s, pausas de 1 a
// 2 s). Move e o jogador virtual do DReCon.
enum class TrackingDrill3D : std::uint8_t { Move = 0, Idle = 1, Turn = 2, FastTurn = 3 };

struct TrackingTrainingSettings3D {
    std::string assetsDirectory;
    std::uint32_t environments = 512;
    // As threads (0: as do hardware menos duas); quem chama step() trabalha tambem.
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    TrackingScenario3D scenario = TrackingScenario3D::Forward;
    // O teto da velocidade no prototipo (m/s).
    float forwardMaximumSpeed = 3.0f;
    // A avaliacao (os portoes): de frente na velocidade dada, sem trocas; o
    // episodio acaba na queda ou no tempo.
    bool evaluation = false;
    float evaluationSpeed = 3.0f;
    TrackingEvaluation3D evaluationMode = TrackingEvaluation3D::Forward;
    float switchSeconds = 4.0f;
    // O jogo inteiro com a corrida (false: so o trote, o G2).
    bool allowSprint = true;
    // Os projeteis do DReCon (7.1, o G3): um cubo de 0,2 m a cada segundo, a 5
    // m/s, contra um ponto sorteado de -0,5 a 0,5 m na vertical do centro de
    // massa, com a massa sorteada entre os limites (o artigo: 0,01 a 8 kg; a
    // avaliacao da Fig. 11 fixa a massa). Declarado (o artigo nao diz): sai a
    // 2 m do corpo, num rumo sorteado, com a mira corrigindo a gravidade.
    bool projectiles = false;
    float projectileMinimumKg = 0.01f;
    float projectileMaximumKg = 8.0f;
    // DReCon: 20 s por episodio, o filtro y = beta a + (1 - beta) y.
    float episodeSeconds = 20.0f;
    float actionFilterBeta = 0.2f;
    // A parte dos episodios do treino em cada exercicio parado (o resto anda).
    // O custo de suavidade (desvio declarado do DReCon, 19/10: as pernas tremiam
    // 14 a 90x o lider parado): a nota vezes exp(-actionRateWeight * media((a - a_anterior)^2))
    // da acao crua. 0 = a receita do artigo.
    float actionRateWeight = 0.0f;
    // As pernas adaptaveis (desvio declarado do DReCon, 19/10, o usuario: "segue
    // a pose do clipe mas tropeca nela; precisa se adaptar"): fora de perigo as
    // pernas seguem o clipe com o peso cheio; em perigo (a cabeca se afastando
    // da do lider, de 0,15 a 0,5 m, ou um tropeco: o pe do corpo no chao com o
    // do lider no ar, por 0,5 s) o peso das pernas e dos pes cai a 0,3.
    bool adaptiveLegs = false;
    // Pernas livres (o usuario, 19/10: "muito mais liberdade para adaptar os
    // passos, para nao cair e depois manter a velocidade"): o peso das pernas e
    // dos pes na pose, sempre (1 = a receita), e o peso da velocidade do centro
    // de massa (1 = a receita).
    float legWeight = 1.0f;
    float centerOfMassVelocityWeight = 1.0f;
    // E a liberdade do resto: o peso dos bracos e do tronco na pose (1 = a receita).
    float armWeight = 1.0f;
    float trunkWeight = 1.0f;
    // Nao cair vale mais que a velocidade (o usuario, 19/10): um premio fixo por
    // decisao de pe (somado a nota; cair perde os de depois) e a escala dos
    // pedidos de velocidade no treino (1 = a tabela do padrao; sobe quando ele
    // parar de cair).
    float aliveBonus = 0.0f;
    float speedScale = 1.0f;
    float idleShare = 0.0f;
    float turnShare = 0.0f;
    float fastTurnShare = 0.0f;
};

// Um episodio que acabou.
struct TrackingEpisode3D {
    std::uint32_t world = 0;
    float seconds = 0.0f;
    // Acabou pela cabeca a mais de 1 m da do lider (a queda do DReCon).
    bool fell = false;
    // Desses, quais foram queda de verdade (a cabeca abaixo de 1,1 m no fim) e
    // quais o corpo de pe que ficou para tras (ou a frente) do lider.
    bool fellDown = false;
    float endHeadHeight = 0.0f;
    float meanReward = 0.0f;
    // A media por decisao do erro da velocidade do centro de massa no plano
    // (corpo contra lider, m/s), da velocidade do corpo e da pedida.
    float centerOfMassSpeedError = 0.0f;
    float bodySpeed = 0.0f;
    float requestedSpeed = 0.0f;
    float meanHeadDistance = 0.0f;
    // O rumo do pedido no fim do episodio em relacao ao olhar (rad; 0 frente,
    // pi/2 esquerda): os graficos por direcao.
    float relativeDirection = 0.0f;
    // O exercicio do episodio (TrackingDrill3D).
    std::uint8_t drill = 0;
};

// O lote de treino do rastreador (vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md): muitos
// mundos em paralelo, cada um com a sua cena, o ragdoll do jogo a 0% de ajuda e
// o lider (o modo padrao do jogo). Uma decisao sao 4 ticks de 1/120 s (30 Hz).
class TrackingTraining3D final {
public:
    static constexpr int TicksPerDecision = 4;
    static constexpr float TickSeconds = 1.0f / 120.0f;

    explicit TrackingTraining3D(const TrackingTrainingSettings3D& settings);
    ~TrackingTraining3D();
    TrackingTraining3D(const TrackingTraining3D&) = delete;
    TrackingTraining3D& operator=(const TrackingTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::uint32_t workerThreads() const;
    [[nodiscard]] const std::vector<std::string>& observationNames() const;
    [[nodiscard]] const std::vector<std::string>& actionNames() const;
    [[nodiscard]] const RagdollProfile3D& profile() const;

    // Recomeca todos os mundos (lider aquecido, corpo sobre ele).
    void reset(std::uint64_t seed);
    // A observacao de cada mundo (environments x observationSize).
    void observations(std::span<float> out) const;
    // Uma decisao: a acao de cada mundo (environments x actionSize, radianos);
    // a nota, se acabou pela queda (terminated) ou pelo tempo (truncated). Quem
    // acabou ja recomecou: a observacao nova e a do comeco seguinte, e a do fim
    // fica em finalObservations (para o valor de quem acabou pelo tempo).
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
        std::span<std::uint8_t> truncated);
    void finalObservations(std::span<float> out) const;
    // Os episodios que acabaram desde a ultima chamada.
    [[nodiscard]] std::vector<TrackingEpisode3D> takeEpisodes();

    // Para as previas: o lider e o corpo de um mundo.
    [[nodiscard]] const TrackingLeaderState3D& leader(std::size_t world) const;
    [[nodiscard]] const RagdollState3D& body(std::size_t world) const;
    // O pedido de um mundo na mao (o visualizador): a velocidade (m/s) e o rumo
    // em relacao ao olhar (rad); vale ate a proxima troca do jogador virtual (na
    // avaliacao, nao ha trocas).
    void setCommand(std::size_t world, float speed, float relativeDirectionRadians);
    // Um exercicio parado num mundo (o visualizador); Move volta ao pedido de setCommand.
    void setDrill(std::size_t world, TrackingDrill3D drill);
    // O olhar pedido de um mundo (rad, sem dar a volta: acumula os giros).
    [[nodiscard]] float lookYaw(std::size_t world) const;
    // Um empurrao (N, no mundo) num link de um mundo durante `seconds`.
    void push(std::size_t world, std::uint32_t link, Vec3 force, float seconds);

    // As linhas de um quadro para as folhas (MatterModelInspector quadros: os
    // colisores no mundo, os contatos, o pedido e a velocidade), do corpo ou do
    // lider.
    [[nodiscard]] std::string frameLines(std::size_t world, bool leader) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
