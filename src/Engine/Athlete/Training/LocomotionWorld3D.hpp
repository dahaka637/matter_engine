#pragma once

#include "Engine/Athlete/AthleteRuntime3D.hpp"
#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace MatterEngine {

class MaterialLibrary;

// O chao dos mundos de treino e da sonda: uma laje de concreto de 120 x 120 m
// com o topo em z = 0.
// halfExtentMeters: a meia-largura do chao (60 m: o de sempre; a corrida a 8 m/s pede mais).
void createLocomotionGround3D(PhysicsScene3D& scene, float halfExtentMeters = 60.0f);

// Uma caixa parada no mundo (parede, degrau, rampa, a barra escondida): o
// centro, a meia-dimensao e a orientacao; de concreto.
struct LocomotionStaticBox3D {
    Vec3 center;
    Vec3 halfExtents;
    Quaternion orientation;
};

void createLocomotionStaticBox3D(PhysicsScene3D& scene, const LocomotionStaticBox3D& box);

// Uma caixa de madeira arremessada (a pancada de verdade: contato, nao
// forca): onde nasce, a velocidade, a massa e a meia-dimensao.
struct LocomotionThrow3D {
    Vec3 position;
    Vec3 velocity;
    float massKg = 20.0f;
    Vec3 halfExtents { 0.2f, 0.2f, 0.2f };
};

void createLocomotionThrow3D(PhysicsScene3D& scene, const LocomotionThrow3D& item);

// O corpo de um atleta de pe na origem (a pelve na altura de pe do perfil, a
// pose de ligacao), como a sonda e o laboratorio o criam.
[[nodiscard]] RagdollHandle3D spawnLocomotionAthlete3D(PhysicsScene3D& scene, const RagdollProfile3D& profile,
    std::uint64_t entityId = 1);

struct LocomotionWorldSettings3D {
    // Os ajustes do jogo (a ajuda, a tabela de velocidades, as chaves).
    AthleteRuntimeSettings3D runtime;
    // A meia-largura do chao (m).
    float groundHalfExtentMeters = 60.0f;
    // O terreno alem do chao (criado depois dele, antes do corpo).
    std::vector<LocomotionStaticBox3D> statics;
    // A cena: as threads do passo (o lote roda uma cena por thread, inline).
    PhysicsSceneSettings3D scene;
    // O tick do jogo (s).
    float tickSeconds = 1.0f / 120.0f;
};

// Um mundo de treino da locomocao (a L0 do plano do RL, vault
// 04_locomocao/rl_sobre_a_locomocao.md, secao 2): uma cena sem janela com o
// chao e um atleta comandado pelo runtime do jogo - o mesmo passo, a mesma
// maquina de modos, a mesma ajuda e os mesmos ganchos do RL (as correcoes do
// atleta). O tick e o do chamador da sonda: o comando, o passo do runtime, a
// fisica e o corpo publicado. Sem a capsula do jogador: o atleta comandado
// tem a capsula virtual dele (AthleteSlot3D::command).
//
// Hoje o atleta comandado so roda no fisico (o padrao e do controlado com a
// capsula): os comecos "no padrao" do plano esperam o padrao comandado. Por
// isso o recomeco (athleteRestore3D) basta: no fisico o passo reescreve os
// alvos, a guia e o atrito a cada tick, e o guiado volta junto.
//
// O motor fisico, os materiais e os recursos (perfil, clipes) sao de quem
// chama e precisam viver mais que o mundo.
class LocomotionWorld3D final {
public:
    LocomotionWorld3D(PhysicsEngine3D& engine, const MaterialLibrary& materials, const AthleteRuntimeAssets3D& assets,
        const LocomotionWorldSettings3D& settings);
    ~LocomotionWorld3D();
    LocomotionWorld3D(const LocomotionWorld3D&) = delete;
    LocomotionWorld3D& operator=(const LocomotionWorld3D&) = delete;

    // Comeca de novo numa cena nova: o corpo de pe na origem, no modo pedido.
    void reset(AthleteStartMode3D mode);
    // Comeca de novo numa cena nova, num instante guardado.
    void restore(const AthleteSnapshot3D& snapshot);
    [[nodiscard]] AthleteSnapshot3D snapshot() const;

    // Uma forca num link no proximo tick, entre o passo do atleta e a fisica
    // (onde a sonda e o painel de impacto empurram).
    void push(std::uint32_t link, Vec3 force);
    // Uma caixa arremessada no proximo tick (no mesmo lugar, antes dos
    // empurroes, como a sonda).
    void throwBox(const LocomotionThrow3D& item);
    // O terreno do proximo comeco (reset ou restore).
    void setStatics(std::vector<LocomotionStaticBox3D> statics);
    // Os recursos do proximo comeco (o corpo variavel: o perfil, os clipes e a
    // capsula de outro corpo); valem do reset ou restore seguinte em diante.
    void setAssets(const AthleteRuntimeAssets3D& assets);

    // Um tick: o comando, o passo do atleta pelo runtime, os empurroes e a
    // fisica.
    void step(const AthleteCommand3D& command);

    // O atleta (as correcoes do RL moram nele) e o corpo publicado.
    [[nodiscard]] AthleteState3D& athlete();
    [[nodiscard]] const AthleteState3D& athlete() const;
    [[nodiscard]] const RagdollState3D& body() const;
    [[nodiscard]] AthleteTelemetry3D telemetry() const;
    [[nodiscard]] PhysicsScene3D& scene();
    [[nodiscard]] RagdollHandle3D handle() const;
    [[nodiscard]] const LocomotionWorldSettings3D& settings() const;
    // Os ajustes do jogo deste mundo (o nivel da ajuda de um episodio; valem
    // do proximo tick).
    [[nodiscard]] AthleteRuntimeSettings3D& runtimeSettings();
    // Os ticks desde o comeco (um recomeco num instante guardado zera).
    [[nodiscard]] std::uint64_t ticks() const;
    // O custo do ultimo tick (s): o passo do atleta e a fisica.
    [[nodiscard]] double lastAthleteSeconds() const;
    [[nodiscard]] double lastPhysicsSeconds() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
