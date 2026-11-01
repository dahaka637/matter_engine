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
    // O equilibrio parado (31/10 21h25): de pe, um empurrao com a regua do parado com empurrao forte do B21.
    static constexpr const char* BalanceDrill = "equilibrio_parado";
    [[nodiscard]] static bool isGetUpDrill(const std::string& drill);
    // Se o treino do levantar de agora e o v2 (os dois exercicios novos existem nele).
    [[nodiscard]] static bool getUpVersion2();
    [[nodiscard]] static std::filesystem::path getUpExport();
    [[nodiscard]] static std::filesystem::path getUpLog();
    [[nodiscard]] static bool getUpReady();
    [[nodiscard]] static bool getUpTraining();
    // No levantar: o que deu certo e quantos episodios acabaram desde o comeco - no levantar do chao, as quedas que
    // ele levantou (ficou de pe alguma vez); no em movimento, as que chegaram ao alvo; no evitar e no equilibrio parado,
    // as que nao cairam.
    [[nodiscard]] std::pair<std::uint32_t, std::uint32_t> getUpTally() const;
    [[nodiscard]] static Conditions conditionsOf(const std::filesystem::path& trackingTask);

    // drill: "rumoN" (andar no rumo N) ou um dos parados; com locomotionTask, o id de um
    // exercicio dela (vazio: o primeiro com peso), levado pelo piloto dele.
    TrackingViewer3D(const std::filesystem::path& assetsDirectory, std::uint64_t seed, const std::string& drill = {},
        const std::filesystem::path& locomotionTask = {}, float speedScale = 1.0f, float filterBeta = 0.2f,
        TrackingViewerConditions3D conditions = {});
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
