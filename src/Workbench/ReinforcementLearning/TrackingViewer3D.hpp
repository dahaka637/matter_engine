#pragma once

#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"
#include "Workbench/ReinforcementLearning/ViewerWorld3D.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <utility>

namespace MatterEngine {

// O lider indo atras do corpo (a folga, m; < 0 nao vai) e se so a queda real acaba o
// episodio, como no treino ("seguirCorpo" e "soQuedaReal" do tarefa.json; sem eles, os de
// 29/10 de madrugada: 0,15 e so a queda).
struct TrackingViewerConditions3D {
    float followSlack = 0.15f;
    bool realFallOnly = true;
    // O lider do treino ("liderLaboratorio"; sem a chave, o do laboratorio, como nos treinos de 29/10) e as condicoes
    // dele e do corpo da noite de 29-30/10 ("torcaoApoio", "ladoDeFrente", "respostaLider").
    bool labLeader = true;
    float stanceTwistTorque = 0.0f;
    bool leaderSideFromForward = false;
    float leaderResponsePercent = 0.0f;
    float terminationDistance = 1.0f;
    float fallHeadHeight = 0.0f;
    float captureStepSeconds = 0.0f;
    float leaderSideAmplitude = 0.0f;
};

// O visualizador de um treino do rastreador fisico (DReCon; vault
// 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md) na tela de
// Aprendizado: o ragdoll do jogo a 0% de ajuda seguindo o lider (o modo padrao
// do jogo), num dos 8 rumos na velocidade da tela, com a rede exportada pelo treino
// (a acao media, como no jogo).
class TrackingViewer3D final : public ViewerWorld3D {
public:
    // Os rumos de 45 em 45 graus a partir da frente, para a esquerda (como no portao).
    static constexpr std::array<const char*, 8> HeadingNames { "Frente", "Diagonal frente-esquerda", "Esquerda",
        "Diagonal tras-esquerda", "Tras", "Diagonal tras-direita", "Direita", "Diagonal frente-direita" };
    // O historico dos graficos de um treino do rastreador (ele nao grava no rl.db):
    // as rodadas do log.jsonl e, por rumo ("rumoN"), os episodios que nao chegaram ao fim nos
    // portoes de rumos na velocidade mais testada. Com sinceSeconds > 0, so as
    // rodadas desse tanto antes da ultima (sem tanto treino, desde o comeco).
    [[nodiscard]] static TrainingHistory3D readHistory(const std::filesystem::path& runDirectory,
        std::size_t maximumPoints, double sinceSeconds = 0.0);
    // Os exercicios parados (ids e nomes, na ordem da lista, depois dos rumos).
    static constexpr std::array<std::array<const char*, 2>, 6> StillDrills { { { "parado", "Parado" },
        { "giro", "Parado com giro suave" }, { "giro_rapido", "Giro rápido parado" },
        { "estrela", "Estrela, peito livre" }, { "estrela_fixo", "Estrela, peito fixo" },
        { "estrela_girando", "Estrela, peito girando" } } };
    // O rumo (rad) do exercicio "rumoN" (outro: a frente).
    [[nodiscard]] static float headingOf(const std::string& drill);

    // O treino com os exercicios de uma tarefa da locomocao (29/10): o arquivo dela (o
    // campo "tarefaLocomocao" do tarefa.json, relativo aos assets ou absoluto; vazio sem
    // ela) e a escala dos pedidos do treino ("escalaVelocidade"; 1 sem ela).
    [[nodiscard]] static std::filesystem::path locomotionTaskOf(const std::filesystem::path& trackingTask,
        const std::filesystem::path& assetsDirectory);
    [[nodiscard]] static float speedScaleOf(const std::filesystem::path& trackingTask);
    // O filtro da acao do treino ("filtroBeta"; 0,2, o do DReCon, sem ele).
    [[nodiscard]] static float filterBetaOf(const std::filesystem::path& trackingTask);
    using Conditions = TrackingViewerConditions3D;
    // O levantar do chao (o exercicio "levantar"): desde 31/10 a rede do levantar livre (uma so, de qualquer queda;
    // o treino levantar_livre, tools/rl/levantar_amp.py), a ultima exportada. Treinando: o log gravado nos ultimos
    // 10 min (ele treina a parte, numa rede propria).
    static constexpr const char* GetUpDrill = "levantar";
    // O levantar v2 (31/10 17h, o treino levantar_v2): o levantar em movimento (cai e levanta ja indo para um alvo,
    // em arrancada) e o evitar a queda (de pe, um empurrao forte), com a mesma rede, cada um pela avaliacao dele.
    static constexpr const char* MovingGetUpDrill = "levantar_movimento";
    static constexpr const char* AvoidFallDrill = "evitar_queda";
    // (07/11, ele: "vamos treinar o levantar de costas e de brucos") O levantar deitado, das levantadas da Unreal (as
    // avaliacoes 9 e 10 do treino; com o v2.deitado do run.json, o levantar em movimento tambem comeca deitado).
    static constexpr const char* LyingBackDrill = "levantar_costas";
    static constexpr const char* LyingFrontDrill = "levantar_brucos";
    // O equilibrio parado (31/10 21h25): de pe, um empurrao com a regua do parado com empurrao forte do B21.
    static constexpr const char* BalanceDrill = "equilibrio_parado";
    // (04/11 15h, os passos de recuperacao) O equilibrio parado com empurroes, sem a mao, no nivel do curriculo do treino (a
    // avaliacao 8): o idle com o gerador de passos e a autoridade sob ameaca.
    static constexpr const char* EquilibriumDrill = "lev_equilibrio";
    // O levantar nos ambientes (02/11): as quedas do levantar com o cenario do treino - o corredor, os pilares, a rampa,
    // o chao acidentado, os corpos soltos embaixo e em cima, a parede e o peso -, nas fatias do treino de agora.
    static constexpr const char* SceneryDrill = "levantar_ambientes";
    // O evitar a queda nos ambientes (02/11, a variante ambientada): o evitar com o cenario do evitar ambientado do treino.
    static constexpr const char* AvoidSceneryDrill = "evitar_queda_ambientes";
    static constexpr std::array<const char*, 6> ControlledGetUpDrills {
        "levantar_fixo_pilares", "levantar_fixo_corredor", "levantar_fixo_rampa", "levantar_fixo_corpo", "levantar_fixo_pilha", "levantar_fixo_corredor_leve" };
    static constexpr std::array<const char*, 5> ControlledAvoidDrills {
        "evitar_fixo_parede", "evitar_fixo_corredor", "evitar_fixo_rampa", "evitar_fixo_corpos1", "evitar_fixo_pilares" };
    [[nodiscard]] static int sceneryPresetOf(const std::string& drill);
    [[nodiscard]] static bool isAvoidSceneryDrill(const std::string& drill);
    // As quedas da locomocao (01/11, a interligacao das redes): o levantar do instante em que a rede da locomocao caiu
    // no treino dela - as 32 fixas da avaliacao do treino.
    static constexpr const char* LocomotionFallDrill = "quedas_locomocao";
    // A locomocao na rede do levantar (31/10 22h35, os basicos do B21; ele: so esta rede daqui em diante): o parado (sem
    // empurrao), o giro parado suave e os rumos (as diagonais com o lado sorteado pela semente), cada um pela avaliacao
    // dele.
    // (01/11, a rl_final: depois deles, as faixas - andar, trote, corrida e sprint -, para a frente na velocidade do
    // clipe da faixa, cada uma pela avaliacao dela.)
    static constexpr std::array<const char*, 12> LocomotionDrills { "lev_parado", "lev_giro", "lev_frente", "lev_tras",
        "lev_direita", "lev_esquerda", "lev_diagonal_frente", "lev_diagonal_costas", "lev_andar", "lev_trote",
        "lev_corrida", "lev_sprint" };
    // A faixa de um exercicio de faixa (0 andar a 3 sprint; -1 se nao e uma).
    [[nodiscard]] static int locomotionBandOf(const std::string& drill);
    // Se o treino do levantar de agora treina a locomocao (a rl_final: os exercicios dela na lista).
    [[nodiscard]] static bool getUpLocomotion(const std::filesystem::path& runDirectory = {});
    // O rumo pedido de um exercicio de rumo (0 a frente, de 45 em 45 graus para a esquerda; a diagonal da direita com a
    // semente impar; -1 se nao e um rumo).
    [[nodiscard]] static int locomotionHeadingOf(const std::string& drill, std::uint64_t seed = 0);
    // (07/11, ele: "uma seccao de exercicios compostos... a estrela de 8 pontas, peito livre e peito fixo, trote e corrida...
    // nao entram no treino ainda, so para observar") Os compostos: so com a sequencia de clipes.
    static constexpr std::array<const char*, 4> CompositeDrills { "comp_estrela_trote_fixo", "comp_estrela_trote_livre",
        "comp_estrela_corrida_fixo", "comp_estrela_corrida_livre" };
    // (08/11, ele: "6 exercicios novos, da estrela de oito pontas... bem elaborados e visualmente organizados no laboratorio")
    // A estrela no treino: trote e corrida, peito fixo, livre e girando (as manobras 8-10).
    static constexpr std::array<const char*, 6> StarDrills { "lev_estrela_trote_fixo", "lev_estrela_trote_livre",
        "lev_estrela_trote_girando", "lev_estrela_corrida_fixo", "lev_estrela_corrida_livre", "lev_estrela_corrida_girando" };
    // O modo da estrela de um composto (1 peito fixo, 2 peito livre; 0 nao e) e a faixa dele (1 trote, 2 corrida).
    [[nodiscard]] static int starModeOf(const std::string& drill);
    [[nodiscard]] static int starBandOf(const std::string& drill);
    [[nodiscard]] static bool isLocomotionDrill(const std::string& drill);
    // As manobras do rastreio (04/11): "lev_troca_suave" ... "lev_curva_fechada" -> o codigo (1-7); -1 nao e manobra.
    [[nodiscard]] static int maneuverOf(const std::string& drill);
    [[nodiscard]] static bool isGetUpDrill(const std::string& drill);
    // Se o tarefa.json e o de um treino do levantar (esquema matter-getup-amp-1; o levantar_v2, a rede principal desde
    // 31/10 22h35).
    [[nodiscard]] static bool isGetUpTask(const std::filesystem::path& taskPath);
    // Se o treino do levantar de agora e o v2 ou a rl_final (os dois exercicios novos existem neles).
    [[nodiscard]] static bool getUpVersion2(const std::filesystem::path& runDirectory = {});
    [[nodiscard]] static std::filesystem::path getUpExport(const std::filesystem::path& runDirectory = {});
    [[nodiscard]] static std::filesystem::path getUpLog(const std::filesystem::path& runDirectory = {});
    [[nodiscard]] static bool getUpReady(const std::filesystem::path& runDirectory = {});
    [[nodiscard]] static bool getUpTraining(const std::filesystem::path& runDirectory = {});
    // No levantar: o que deu certo e quantos episodios acabaram desde o comeco - no levantar do chao, as quedas que
    // ele levantou (ficou de pe alguma vez); no em movimento, as que chegaram ao alvo; no evitar e no equilibrio parado,
    // as que nao cairam.
    [[nodiscard]] std::pair<std::uint32_t, std::uint32_t> getUpTally() const;
    // A troca para a rede da locomocao (01/11): se quem dirige agora e ela (o mundo do treino com o "getUpHandoff",
    // depois de a rede do levantar ficar de pe e firme; no evitar a queda, desde o comeco) e o texto da rede dela.
    [[nodiscard]] bool drivingLocomotion() const;
    // (07/11) Na rede de agora (cabecas separadas), se a cabeca da locomocao dirige: a ultima entrada da observacao.
    [[nodiscard]] bool locomotionHeadDriving() const;
    // (06/11, a locomocao livre) A professora do treino na parcela dela nos motores, como no treino (ligada por padrao), e a
    // parcela de agora (0: sem professora no treino).
    void setTeacherShown(bool shown);
    // (07/11, temporario) Os exercicios de forma cinematica pelo Motion Matching do juiz (sem fisica, sem a rede).
    void setKinematicMotionMatching(bool on);
    [[nodiscard]] bool kinematicMotionMatching() const;
    [[nodiscard]] const std::string& kinematicError() const;
    [[nodiscard]] std::string kinematicClip() const;
    [[nodiscard]] std::size_t kinematicJumps() const;
    // (07/11, so para ver) A sequencia de clipes da Unreal de cada exercicio, sem fisica (ligada por padrao).
    void setClipSequence(bool on);
    [[nodiscard]] bool clipSequence() const;
    // (08/11) Os empurroes do treino (o nivel de cada grupo, os pontos variados) em todo episodio; desligado por padrao.
    void setPushes(bool on);
    [[nodiscard]] bool pushes() const;
    [[nodiscard]] bool teacherShown() const;
    [[nodiscard]] float teacherShare() const;
    [[nodiscard]] const std::string& partnerText() const;
    [[nodiscard]] static Conditions conditionsOf(const std::filesystem::path& trackingTask);

    // drill: "rumoN" (andar no rumo N) ou um dos parados; com locomotionTask, o id de um
    // exercicio dela (vazio: o primeiro com peso), levado pelo piloto dele.
    TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed, const std::string& drill = {},
        const std::filesystem::path& locomotionTask = {}, float speedScale = 1.0f, float filterBeta = 0.2f,
        TrackingViewerConditions3D conditions = {}, const std::filesystem::path& getUpDirectory = {});
    ~TrackingViewer3D() override;
    TrackingViewer3D(const TrackingViewer3D&) = delete;
    TrackingViewer3D& operator=(const TrackingViewer3D&) = delete;

    // Se o tarefa.json e de um treino do rastreador (esquema matter-tracking-1).
    [[nodiscard]] static bool isTrackingTask(const std::filesystem::path& taskPath);

    void setPolicy(const std::filesystem::path& file, std::string& label) override;
    void clearPolicy() override;
    [[nodiscard]] bool hasPolicy() const override;
    void step() override;
    [[nodiscard]] float policyStepSeconds() const override { return 1.0f / 30.0f; }
    void reset(std::uint64_t seed) override;
    void setManualSpeed(std::optional<float> metersPerSecond) override;
    void push(Vec3 force, float seconds) override;
    [[nodiscard]] BodyWorldView3D view() const override;
    [[nodiscard]] const RagdollProfile3D& profile() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
