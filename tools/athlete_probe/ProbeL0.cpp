// Os modos da L0 do plano do RL na sonda: a paridade (--paridade) e a linha
// de base do classico por nivel de ajuda (--linha-de-base). O roteiro e as
// medidas sao os da sonda (ProbeShared.hpp).
#include "athlete_probe/ProbeShared.hpp"

#include "Engine/Athlete/Training/LocomotionBatch3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace MatterEngine::Probe {

std::vector<float> parityAction(const LocomotionContract3D& contract, int decision) {
    std::vector<float> action(contract.actionSize());
    for (std::size_t i = 0; i < action.size(); ++i)
        action[i] = 0.8f * std::sin(static_cast<float>(decision) * 0.37f + static_cast<float>(i) * 1.3f);
    return action;
}

// A impressao da entrada (FNV-1a sobre os bits).
std::uint64_t observationPrint(std::span<const float> values) {
    std::uint64_t hash = 1469598103934665603ull;
    for (float v : values) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        for (int k = 0; k < 4; ++k) {
            hash ^= (bits >> (8 * k)) & 0xFFu;
            hash *= 1099511628211ull;
        }
    }
    return hash;
}

// Antes do passo: a decisao (a cada TicksPerDecision ticks) e a rampa.
void contractBefore(ContractHooks* hooks, AthleteState3D& athlete, int step) {
    if (hooks == nullptr || hooks->contract == nullptr || !hooks->act) return;
    if (step % LocomotionContract3D::TicksPerDecision == 0)
        hooks->contract->decide(athlete, parityAction(*hooks->contract, step / LocomotionContract3D::TicksPerDecision));
    hooks->contract->follow(athlete, hooks->authority);
}

// Depois da fisica: a entrada da rede deste instante.
void contractAfter(ContractHooks* hooks, const AthleteState3D& athlete, const RagdollState3D& body,
    const AthleteCommand3D& command) {
    if (hooks == nullptr || hooks->contract == nullptr || hooks->observations == nullptr) return;
    hooks->buffer.resize(hooks->contract->observationSize());
    hooks->nonFinite += hooks->contract->observe(body, command, athleteControlView3D(athlete), athlete.policyHistory,
        hooks->forecast, hooks->buffer).nonFinite;
    hooks->observations->push_back(observationPrint(hooks->buffer));
}

// Um cenario do comeco ao fim (a cena nova, o boneco, as teclas, as medidas).
// O comando do roteiro num tick, como a sonda o monta (o analogico virado
// pela camera, sem a virada automatica do manejo).
AthleteCommand3D scenarioCommand(const ProbeContext& context, const AthleteRuntimeSettings3D& settings, const Keys& keys,
    float terrainSpeedLimit) {
    const Vec3 forward { std::cos(keys.cameraYaw), std::sin(keys.cameraYaw), 0.0f };
    const Vec3 right { forward.y, -forward.x, 0.0f };
    AthleteStickInput3D stick;
    stick.moveWorld = forward * keys.forward - right * keys.left;
    stick.cameraYawRadians = keys.cameraYaw;
    stick.lookPitchRadians = -0.2f;
    stick.sprint = keys.sprint;
    stick.crouch = keys.crouch;
    return athleteStickCommand3D(stick, settings, context.gaitSpeeds, terrainSpeedLimit);
}

// Um tick do roteiro num mundo de treino: o comando, os empurroes da sonda e
// (na paridade do contrato) a acao fixa e a entrada da rede.
void stepScenarioWorld(const ProbeContext& context, const AthleteRuntimeSettings3D& settings, const Scenario& scenario,
    LocomotionWorld3D& world, int step, ContractHooks* hooks) {
    const float t = static_cast<float>(step) * Dt;
    const AthleteCommand3D command = scenarioCommand(context, settings, scenario.keys(t),
        world.telemetry().terrainSpeedLimit);
    contractBefore(hooks, world.athlete(), step);
    for (const Throw& item : scenario.throws)
        if (t >= item.at && t < item.at + Dt) world.throwBox({ item.start, item.velocity, item.massKg, item.halfExtents });
    for (const Push& push : scenario.pushes)
        if (t >= push.at && t < push.at + push.seconds) {
            world.push(0, push.force * 0.20f);
            world.push(2, push.force * 0.32f);
            world.push(3, push.force * 0.48f);
        }
    world.step(command);
    contractAfter(hooks, world.athlete(), world.body(), command);
}

// O terreno de um roteiro (os degraus, a rampa, a parede) para um mundo.
std::vector<LocomotionStaticBox3D> scenarioStatics(const Scenario& scenario) {
    std::vector<LocomotionStaticBox3D> statics;
    for (const StaticBox& box : scenario.statics) statics.push_back({ box.center, box.half, box.orientation });
    if (scenario.wallHalf.lengthSquared() > 0.0f) statics.push_back({ scenario.wallCenter, scenario.wallHalf, {} });
    return statics;
}

// Onde duas trilhas de impressoes se separam: o primeiro tick e a parte (o
// corpo, os alvos, a ajuda, as decisoes); -1 iguais.
struct TraceSplit {
    int tick = -1;
    const char* part = "";
};

TraceSplit splitOf(const std::vector<AthleteTickDigest3D>& a, const std::vector<AthleteTickDigest3D>& b) {
    TraceSplit split;
    const std::size_t count = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < count; ++i) {
        if (a[i] == b[i]) continue;
        split.tick = static_cast<int>(i);
        // (A ordem do tick: as decisoes e os alvos saem antes da fisica.)
        split.part = a[i].decisions != b[i].decisions ? "decisoes"
            : a[i].navigation != b[i].navigation     ? "navegacao"
            : a[i].targets != b[i].targets           ? "alvos"
            : a[i].assist != b[i].assist             ? "ajuda"
                                                     : "corpo";
        return split;
    }
    if (a.size() != b.size()) {
        split.tick = static_cast<int>(count);
        split.part = "tamanho";
    }
    return split;
}

// A paridade (R1 da reescrita, o portao da L0 do RL; vault
// 04_locomocao/reescrita_da_locomocao.md):
//  1. cada roteiro sem bola nem objetos, do comeco, pelo caminho da sonda (o
//     chamador do jogo, comandado sem capsula, no fisico) e por um mundo de
//     treino, comparados tick a tick bit a bit (corpo, alvos, ajuda,
//     decisoes);
//  2. os recomecos: instantes guardados no meio de roteiros (1 e 2 s),
//     recolocados em dois mundos novos e seguidos por 2 s - iguais bit a bit;
//  3. o recomeco contra a continuacao (o solver perde o arranque quente): so
//     a medida, em mm da pelve depois de 0,1 e 1 s;
//  4. a cena inline de uma thread (como o lote roda) contra a com threads (a
//     do jogo), bit a bit, e o custo do tick nela (p50/p95/p99).
// O degrau 1 (a entrada e a saida da rede) espera o contrato da rede. limit:
// os roteiros da parte 1 (0 todos); restores: os da parte 2 (0 todos).
int runParity(const ProbeContext& context, int limit, int restores) {
    AthleteRuntimeSettings3D settings;
    settings.defaultLocomotion = false;
    // (Com a referencia de estilo a sombra: a entrada da rede completa.)
    settings.physicalReference = true;
    ProbeContext commanded = context;
    commanded.noCapsule = true;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    LocomotionWorldSettings3D worldSettings;
    worldSettings.runtime = settings;
    int failures = 0;
    std::printf("paridade: configuracao fisico total, comandado sem capsula, referencia a sombra, tick %.6f s, cena %u threads%s\n", Dt,
        worldSettings.scene.workerThreadCount, worldSettings.scene.runJobsInline ? " inline" : " (todas)");
    const LocomotionContract3D contract(*context.profile);
    std::printf("contrato da rede v%u: %zu entradas, %zu saidas, %zu eixos atuados, impressao %016llx\n",
        LocomotionContract3D::Version, contract.observationSize(), contract.actionSize(), contract.actuatedAxes().size(),
        static_cast<unsigned long long>(contract.hash()));
    {
        // O espelho (a simetria do treinador): os eixos das juntas do outro
        // lado devem ser o reflexo exato (alinhamento 1).
        const LocomotionMirror3D mirror = contract.mirror();
        std::printf("  o espelho: o pior alinhamento dos eixos refletidos %.3f (%s)%s\n", mirror.worstAxisAlignment,
            mirror.worstAxis.c_str(), mirror.worstAxisAlignment > 0.95f ? "" : " - EIXOS ASSIMETRICOS");
        if (mirror.worstAxisAlignment <= 0.95f) ++failures;
    }
    std::printf("1. sonda contra mundo de treino, do comeco (todos os roteiros sem bola: terreno, caixas, empurroes), com a entrada da rede\n");
    int scenariosRun = 0, ticksCompared = 0;
    std::size_t nonFinite = 0;
    int observationFailures = 0;
    const std::vector<Scenario> list = scenarios();
    for (const Scenario& scenario : list) {
        // (Sem bola: a bola do treino vem com o RL dela.)
        if (scenario.ball) continue;
        if (limit > 0 && scenariosRun >= limit) break;
        ++scenariosRun;
        std::vector<AthleteTickDigest3D> probe, trained;
        std::vector<std::uint64_t> probeObservations, trainedObservations;
        ContractHooks probeHooks, trainedHooks;
        probeHooks.contract = trainedHooks.contract = &contract;
        probeHooks.observations = &probeObservations;
        trainedHooks.observations = &trainedObservations;
        (void)runScenario(commanded, scenario, settings, true, nullptr, 1, false, &probe, &probeHooks);
        LocomotionWorldSettings3D terrain = worldSettings;
        terrain.statics = scenarioStatics(scenario);
        LocomotionWorld3D world(engine, materials, context.runtimeAssets, terrain);
        world.reset(AthleteStartMode3D::Physical);
        const int ticks = static_cast<int>(scenario.seconds / Dt);
        for (int step = 0; step < ticks; ++step) {
            stepScenarioWorld(context, settings, scenario, world, step, &trainedHooks);
            trained.push_back(athleteTickDigest3D(world.athlete(), world.body()));
        }
        const TraceSplit split = splitOf(probe, trained);
        ticksCompared += static_cast<int>(probe.size());
        nonFinite += probeHooks.nonFinite + trainedHooks.nonFinite;
        if (scenario.name == "trota" && settings.physicalReference) {
            // A referencia de estilo trotando: vale, anda (a fase e o plano a
            // frente) e o erro das partes-chave e de centimetros.
            const auto& channels = contract.observationLayout().channels;
            const auto value = [&](const std::vector<float>& observation, const std::string& name) {
                for (std::size_t c = 0; c < channels.size(); ++c)
                    if (channels[c].name == name) return observation[c];
                return std::numeric_limits<float>::quiet_NaN();
            };
            std::vector<float> observation(contract.observationSize());
            BodyForecast3D forecast;
            (void)contract.observe(world.body(), scenarioCommand(context, settings, scenario.keys(scenario.seconds - Dt),
                    world.telemetry().terrainSpeedLimit),
                athleteControlView3D(world.athlete()), world.athlete().policyHistory, forecast, observation);
            const float valid = value(observation, "ref.valida"), ahead = value(observation, "ref.plano.0_50.pos.x");
            const float footError = std::hypot(value(observation, "ref.estilo_erro.pe_esq.x"),
                value(observation, "ref.estilo_erro.pe_esq.y"));
            const bool sane = valid > 0.5f && ahead > 0.5f && footError < 0.6f;
            std::printf("  a referencia trotando: valida %.0f, o plano a 0,5 s %.2f m a frente, a fase (%.2f, %.2f), o pe esquerdo %.2f m do estilo (%s)\n",
                valid, ahead, value(observation, "ref.fase.sen"), value(observation, "ref.fase.cos"), footError,
                sane ? "ok" : "FALHOU");
            if (!sane) ++failures;
        }
        if (scenario.name == "parado") {
            // O referencial da entrada: a frente da pelve sai como +x, e o
            // olhar relativo e o olhar menos o giro da pelve medido a parte
            // (parado, a base do classico deixa a pelve uns 20 graus virada;
            // o peito de cima olha para a camera).
            const auto& channels = contract.observationLayout().channels;
            const auto channel = [&](const std::string& name) {
                for (std::size_t c = 0; c < channels.size(); ++c)
                    if (channels[c].name == name) return c;
                return channels.size();
            };
            const std::string pelvisId = context.profile->links.front().id;
            const std::size_t sine = channel("comando.olhar.sen"), cosine = channel("comando.olhar.cos");
            const std::size_t forwardX = channel("corpo." + pelvisId + ".frente.x");
            const std::size_t forwardY = channel("corpo." + pelvisId + ".frente.y");
            std::vector<float> observation(contract.observationSize());
            BodyForecast3D forecast;
            (void)contract.observe(world.body(), scenarioCommand(context, settings, Keys {}, 1.0e9f),
                athleteControlView3D(world.athlete()), world.athlete().policyHistory, forecast, observation);
            const Quaternion pelvisRotation = (world.body().links.front().orientation
                * context.profile->links.front().modelOrientation.conjugate()).normalized();
            const Vec3 pelvisForward = pelvisRotation.rotate({ 1.0f, 0.0f, 0.0f });
            const float pelvisYaw = std::atan2(pelvisForward.y, pelvisForward.x);
            const bool found = sine < channels.size() && cosine < channels.size() && forwardX < channels.size()
                && forwardY < channels.size();
            const bool frame = found && observation[forwardX] > 0.95f && std::abs(observation[forwardY]) < 0.05f
                && std::abs(observation[sine] - std::sin(-pelvisYaw)) < 0.02f
                && std::abs(observation[cosine] - std::cos(-pelvisYaw)) < 0.02f;
            std::printf("  o referencial parado: a frente da pelve (%.3f, %.3f); o olhar relativo %.1f graus, a pelve virada %.1f (%s)\n",
                found ? observation[forwardX] : 0.0f, found ? observation[forwardY] : 0.0f,
                found ? std::atan2(observation[sine], observation[cosine]) * 57.2958f : 0.0f, pelvisYaw * 57.2958f,
                frame ? "ok" : "FALHOU: o referencial do rumo esta errado");
            if (!frame) ++failures;
        }
        if (split.tick >= 0) {
            ++failures;
            std::printf("  FALHOU %-22s separa no tick %d (%s)\n", scenario.name.c_str(), split.tick, split.part);
        }
        if (probeObservations != trainedObservations) {
            ++observationFailures;
            std::size_t first = 0;
            while (first < std::min(probeObservations.size(), trainedObservations.size())
                && probeObservations[first] == trainedObservations[first])
                ++first;
            std::printf("  FALHOU %-22s a entrada da rede separa no tick %zu\n", scenario.name.c_str(), first);
        }
    }
    failures += observationFailures;
    if (nonFinite > 0) ++failures;
    if (settings.physicalReference) {
        // O erro dos pes contra o estilo em cruzeiro (de 1,5 s ao fim; a
        // media dos dois pes, no plano): com a fase da referencia seguindo o
        // corpo ele cai (a fase diferente soma meia passada).
        const auto& channels = contract.observationLayout().channels;
        std::size_t left = channels.size(), right = channels.size(), valid = channels.size();
        for (std::size_t c = 0; c < channels.size(); ++c) {
            if (channels[c].name == "ref.estilo_erro.pe_esq.x") left = c;
            if (channels[c].name == "ref.estilo_erro.pe_dir.x") right = c;
            if (channels[c].name == "ref.valida") valid = c;
        }
        std::printf("  o erro dos pes contra o estilo em cruzeiro (m, media dos dois pes):");
        for (const char* name : { "anda", "trota", "sprint", "costas", "diagonal", "lados" }) {
            const Scenario* found = nullptr;
            for (const Scenario& scenario : list)
                if (scenario.name == name) found = &scenario;
            if (found == nullptr || left >= channels.size() || right >= channels.size()) continue;
            LocomotionWorld3D world(engine, materials, context.runtimeAssets, worldSettings);
            world.reset(AthleteStartMode3D::Physical);
            std::vector<float> observation(contract.observationSize());
            BodyForecast3D forecast;
            double sum = 0.0;
            int samples = 0;
            const int ticks = static_cast<int>(found->seconds / Dt);
            for (int step = 0; step < ticks; ++step) {
                stepScenarioWorld(context, settings, *found, world, step);
                if (static_cast<float>(step) * Dt < 1.5f) continue;
                (void)contract.observe(world.body(), scenarioCommand(context, settings, found->keys(static_cast<float>(step) * Dt),
                        world.telemetry().terrainSpeedLimit),
                    athleteControlView3D(world.athlete()), world.athlete().policyHistory, forecast, observation);
                if (observation[valid] < 0.5f) continue;
                sum += 0.5 * (std::hypot(observation[left], observation[left + 1])
                    + std::hypot(observation[right], observation[right + 1]));
                ++samples;
            }
            std::printf(" %s %.2f", name, samples > 0 ? sum / samples : -1.0);
        }
        std::printf("\n");
    }
    std::printf("  %d roteiros, %d ticks: %s; a entrada da rede %s e %s\n", scenariosRun, ticksCompared,
        failures == 0 ? "iguais bit a bit" : "com diferencas", observationFailures == 0 ? "igual bit a bit" : "DIFERE",
        nonFinite == 0 ? "sempre finita" : "com numeros nao finitos");
    std::printf("2. recomecos de instantes guardados (dois mundos novos) e 3. contra a continuacao\n");
    int states = 0, restoreFailures = 0;
    const std::vector<Scenario> all = scenarios();
    int restoreScenarios = 0;
    for (const char* name : { "trota", "sprint", "zigue", "corte", "inverte", "costas", "diagonal", "para_sprint",
             "queda_lado_1400", "queda_levanta", "mov_corre_090_045", "mov_anda_225_180" }) {
        if (restores > 0 && restoreScenarios++ >= restores) break;
        const Scenario* found = nullptr;
        for (const Scenario& scenario : all)
            if (scenario.name == name) found = &scenario;
        if (found == nullptr) continue;
        const Scenario& scenario = *found;
        for (const float at : { 1.0f, 2.0f }) {
            const int startTick = static_cast<int>(at / Dt);
            const int ticks = std::min(static_cast<int>(2.0f / Dt), static_cast<int>(scenario.seconds / Dt) - startTick);
            if (ticks <= 0) continue;
            LocomotionWorld3D original(engine, materials, context.runtimeAssets, worldSettings);
            original.reset(AthleteStartMode3D::Physical);
            for (int step = 0; step < startTick; ++step)
                stepScenarioWorld(context, settings, scenario, original, step);
            const AthleteSnapshot3D snapshot = original.snapshot();
            std::vector<AthleteTickDigest3D> traces[3];
            std::vector<Vec3> pelvis[3];
            LocomotionWorld3D first(engine, materials, context.runtimeAssets, worldSettings);
            LocomotionWorld3D second(engine, materials, context.runtimeAssets, worldSettings);
            first.restore(snapshot);
            second.restore(snapshot);
            LocomotionWorld3D* worlds[3] { &original, &first, &second };
            for (int step = 0; step < ticks; ++step) {
                for (int k = 0; k < 3; ++k) {
                    stepScenarioWorld(context, settings, scenario, *worlds[k], startTick + step);
                    traces[k].push_back(athleteTickDigest3D(worlds[k]->athlete(), worlds[k]->body()));
                    pelvis[k].push_back(worlds[k]->body().links.front().position);
                }
            }
            ++states;
            const TraceSplit twin = splitOf(traces[1], traces[2]);
            if (twin.tick >= 0) ++restoreFailures;
            const auto deviation = [&](int tick) {
                tick = std::min(tick, static_cast<int>(pelvis[0].size()) - 1);
                return (pelvis[0][static_cast<std::size_t>(tick)] - pelvis[1][static_cast<std::size_t>(tick)]).length()
                    * 1000.0f;
            };
            const AthleteTelemetry3D end = athleteTelemetry3D(first.athlete());
            const std::string twinText = twin.tick < 0 ? std::string("iguais")
                                                       : "DIFEREM no tick " + std::to_string(twin.tick) + " (" + twin.part + ")";
            std::printf("  %-18s em %.1f s: recomecos %s | contra a continuacao: pelve %.2f mm em 0,1 s, %.1f mm em 1 s (%s)\n",
                name, at, twinText.c_str(), deviation(12), deviation(120),
                end.fallen ? "caido" : bodyModeName3D(end.mode));
        }
    }
    failures += restoreFailures;
    std::printf("  %d instantes: %s\n", states, restoreFailures == 0 ? "recomecos iguais bit a bit" : "recomecos diferem");
    std::printf("4. a cena do lote (inline, reservas pequenas) contra a do jogo, e o custo do tick nela\n");
    // (A cena exata do lote: inline, uma thread, as reservas pequenas.)
    LocomotionWorldSettings3D inlineSettings = worldSettings;
    inlineSettings.scene = locomotionBatchScene3D();
    std::vector<double> athleteCost, physicsCost, locomotionCost, assistCost, referenceCost;
    std::vector<std::vector<double>> sectionCost(CharacterLocomotion3D::SectionCount);
    int inlineFailures = 0;
    for (const char* name : { "trota", "sprint", "zigue", "queda_lado_1400" }) {
        const Scenario* found = nullptr;
        for (const Scenario& scenario : all)
            if (scenario.name == name) found = &scenario;
        if (found == nullptr) continue;
        const Scenario& scenario = *found;
        LocomotionWorld3D pooled(engine, materials, context.runtimeAssets, worldSettings);
        LocomotionWorld3D single(engine, materials, context.runtimeAssets, inlineSettings);
        pooled.reset(AthleteStartMode3D::Physical);
        single.reset(AthleteStartMode3D::Physical);
        std::vector<AthleteTickDigest3D> a, b;
        const int ticks = static_cast<int>(scenario.seconds / Dt);
        for (int step = 0; step < ticks; ++step) {
            stepScenarioWorld(context, settings, scenario, pooled, step);
            stepScenarioWorld(context, settings, scenario, single, step);
            a.push_back(athleteTickDigest3D(pooled.athlete(), pooled.body()));
            b.push_back(athleteTickDigest3D(single.athlete(), single.body()));
            athleteCost.push_back(single.lastAthleteSeconds());
            physicsCost.push_back(single.lastPhysicsSeconds());
            locomotionCost.push_back(single.athlete().locomotionSeconds);
            assistCost.push_back(single.athlete().assistSeconds);
            referenceCost.push_back(single.athlete().referenceSeconds);
            for (std::size_t k = 0; k < CharacterLocomotion3D::SectionCount; ++k)
                sectionCost[k].push_back(single.athlete().legacy.locomotion.sectionSeconds()[k]);
        }
        const TraceSplit split = splitOf(a, b);
        if (split.tick >= 0) {
            ++inlineFailures;
            std::printf("  FALHOU %-22s inline separa no tick %d (%s)\n", name, split.tick, split.part);
        }
    }
    failures += inlineFailures;
    // 5. O contrato da rede com as correcoes: a acao fixa (rampa de 4 ticks)
    // com autoridade 0,5 nas quatro cabecas, pela sonda e pelo mundo (o gancho
    // nos dois caminhos); e com autoridade zero, igual ao classico.
    std::printf("5. as correcoes do contrato nos dois caminhos (autoridade 0,5) e a correcao zero igual ao classico\n");
    int hookFailures = 0, zeroFailures = 0, hookScenarios = 0;
    for (const Scenario& scenario : list) {
        if (scenario.ball) continue;
        if (limit > 0 && hookScenarios >= limit) break;
        ++hookScenarios;
        LocomotionWorldSettings3D terrain = worldSettings;
        terrain.statics = scenarioStatics(scenario);
        const int ticks = static_cast<int>(scenario.seconds / Dt);
        // Com autoridade.
        std::vector<AthleteTickDigest3D> probe, trained;
        std::vector<std::uint64_t> probeObservations, trainedObservations;
        ContractHooks probeHooks, trainedHooks;
        for (ContractHooks* hooks : { &probeHooks, &trainedHooks }) {
            hooks->contract = &contract;
            hooks->act = true;
            hooks->authority = { 0.5f, 0.5f, 0.5f, 0.5f };
        }
        probeHooks.observations = &probeObservations;
        trainedHooks.observations = &trainedObservations;
        (void)runScenario(commanded, scenario, settings, true, nullptr, 1, false, &probe, &probeHooks);
        LocomotionWorld3D world(engine, materials, context.runtimeAssets, terrain);
        world.reset(AthleteStartMode3D::Physical);
        for (int step = 0; step < ticks; ++step) {
            stepScenarioWorld(context, settings, scenario, world, step, &trainedHooks);
            trained.push_back(athleteTickDigest3D(world.athlete(), world.body()));
        }
        const TraceSplit split = splitOf(probe, trained);
        if (split.tick >= 0 || probeObservations != trainedObservations) {
            ++hookFailures;
            std::printf("  FALHOU %-22s com correcoes: separa no tick %d (%s)%s\n", scenario.name.c_str(), split.tick,
                split.part, probeObservations != trainedObservations ? ", a entrada difere" : "");
        }
        // Autoridade zero: a mesma acao nao muda nada.
        ContractHooks zeroHooks;
        zeroHooks.contract = &contract;
        zeroHooks.act = true;
        LocomotionWorld3D plain(engine, materials, context.runtimeAssets, terrain);
        LocomotionWorld3D zero(engine, materials, context.runtimeAssets, terrain);
        plain.reset(AthleteStartMode3D::Physical);
        zero.reset(AthleteStartMode3D::Physical);
        std::vector<AthleteTickDigest3D> plainTrace, zeroTrace;
        for (int step = 0; step < ticks; ++step) {
            stepScenarioWorld(context, settings, scenario, plain, step);
            stepScenarioWorld(context, settings, scenario, zero, step, &zeroHooks);
            plainTrace.push_back(athleteTickDigest3D(plain.athlete(), plain.body()));
            zeroTrace.push_back(athleteTickDigest3D(zero.athlete(), zero.body()));
        }
        const TraceSplit zeroSplit = splitOf(plainTrace, zeroTrace);
        if (zeroSplit.tick >= 0) {
            ++zeroFailures;
            std::printf("  FALHOU %-22s autoridade zero muda o classico no tick %d (%s)\n", scenario.name.c_str(),
                zeroSplit.tick, zeroSplit.part);
        }
    }
    failures += hookFailures + zeroFailures;
    std::printf("  %d roteiros: com correcoes, sonda e mundo %s; autoridade zero %s\n", hookScenarios,
        hookFailures == 0 ? "iguais bit a bit (trajetoria e entrada)" : "DIFEREM",
        zeroFailures == 0 ? "igual ao classico bit a bit" : "MUDA o classico");
    const auto percentile = [](std::vector<double> values, double p) {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        return values[std::min(values.size() - 1, static_cast<std::size_t>(p * static_cast<double>(values.size())))]
            * 1.0e6;
    };
    std::printf("  inline: %s; o tick (us) atleta p50 %.0f p95 %.0f p99 %.0f | fisica p50 %.0f p95 %.0f p99 %.0f (%zu ticks)\n",
        inlineFailures == 0 ? "igual bit a bit" : "DIFERE", percentile(athleteCost, 0.5), percentile(athleteCost, 0.95),
        percentile(athleteCost, 0.99), percentile(physicsCost, 0.5), percentile(physicsCost, 0.95),
        percentile(physicsCost, 0.99), athleteCost.size());
    std::printf("  no passo do atleta: controle fisico p50 %.0f p95 %.0f p99 %.0f | ajuda p50 %.0f p95 %.0f p99 %.0f | referencia a sombra p50 %.0f p95 %.0f p99 %.0f\n",
        percentile(locomotionCost, 0.5), percentile(locomotionCost, 0.95), percentile(locomotionCost, 0.99),
        percentile(assistCost, 0.5), percentile(assistCost, 0.95), percentile(assistCost, 0.99),
        percentile(referenceCost, 0.5), percentile(referenceCost, 0.95), percentile(referenceCost, 0.99));
    std::printf("  no controle fisico, por bloco (p50/p99 us):");
    for (std::size_t k = 0; k < CharacterLocomotion3D::SectionCount; ++k)
        std::printf(" %s %.0f/%.0f", CharacterLocomotion3D::sectionName(k), percentile(sectionCost[k], 0.5),
            percentile(sectionCost[k], 0.99));
    std::printf("\n");
    std::printf(failures == 0 ? "paridade: degraus 1 e 2 passam (contrato v1)\n" : "paridade: %d falha(s)\n", failures);
    return failures == 0 ? 0 : 1;
}

// A linha de base do classico por nivel de ajuda (a L0 do plano do RL: o que
// o RL tem de bater) e a vazao do lote: a bateria sem bola - retas em 8 rumos
// e 4 marchas, as manobras, as pancadas e o terreno - num lote de mundos de
// treino, um mundo por exercicio e nivel (as ajudas parado e andando no
// nivel). Mede as quedas, a velocidade contra a pedida nas retas (de 2 a 4 s,
// no rumo pedido) e a forca da ajuda na pelve (fracao do peso).
int runBaseline(const ProbeContext& context, int threads) {
    struct Drill {
        std::string group;
        Scenario scenario;
    };
    std::vector<Drill> drills;
    static const struct { const char* key; float amount; bool sprint; } gaits[4] = {
        { "anda", 0.33f, false }, { "trota", 0.5f, false }, { "corre", 1.0f, false }, { "sprint", 1.0f, true } };
    for (const auto& gait : gaits)
        for (int k = 0; k < 8; ++k) {
            char name[48];
            std::snprintf(name, sizeof(name), "reta_%s_%03d", gait.key, k * 45);
            const float angle = static_cast<float>(k) * Pi * 0.25f;
            const float amount = gait.amount;
            const bool sprint = gait.sprint;
            drills.push_back({ std::string("reta_") + gait.key, Scenario { name, 4.0f, [angle, amount, sprint](float t) {
                Keys keys;
                if (t > 0.3f) {
                    keys.forward = std::cos(angle) * amount;
                    keys.left = std::sin(angle) * amount;
                    keys.sprint = sprint;
                }
                return keys; } } });
        }
    const std::vector<Scenario> all = scenarios();
    const auto add = [&](const char* group, std::initializer_list<const char*> names) {
        for (const char* name : names)
            for (const Scenario& scenario : all)
                if (scenario.name == name) {
                    Scenario capped = scenario;
                    // (A pancada decide nos primeiros segundos; o levantar nao
                    // entra na linha de base.)
                    capped.seconds = std::min(capped.seconds, 6.0f);
                    drills.push_back({ group, capped });
                }
    };
    add("manobras", { "zigue", "inverte", "corte", "para", "para_sprint", "lados", "gira", "gira_rapido", "anda_gira",
                        "corre_gira", "sprint_gira", "lado_gira", "diagonal_gira", "corre_vira180", "anda_vira180" });
    add("pancadas", { "queda_lado_700", "queda_lado_1000", "queda_lado_1400", "queda_lado_1900", "queda_000", "queda_090",
                        "queda_180", "queda_270", "quedacorre_000", "quedacorre_090", "quedacorre_180", "quedacorre_270",
                        "empurrao_leve", "empurrao_medio", "empurrao_forte", "empurrao_correndo", "encostao_andando",
                        "encostao_correndo", "caixa_leve", "caixa_media", "caixa_forte", "caixa_correndo" });
    add("terreno", { "degrau_10_anda", "degrau_18_anda", "degrau_25_anda", "degrau_35_anda", "degrau_10_corre",
                       "degrau_18_corre", "degrau_25_corre", "degrau_35_corre", "escada_anda", "escada_corre", "rampa_anda",
                       "rampa_corre" });
    const float levels[5] = { 100.0f, 70.0f, 50.0f, 30.0f, 0.0f };
    const std::size_t count = drills.size() * 5;
    AthleteRuntimeSettings3D base;
    base.defaultLocomotion = false;
    LocomotionBatchSettings3D batchSettings;
    batchSettings.worlds = static_cast<std::uint32_t>(count);
    batchSettings.threads = static_cast<std::uint32_t>(std::max(0, threads));
    batchSettings.world.runtime = base;
    LocomotionBatch3D batch(context.runtimeAssets, batchSettings);
    struct Tally {
        bool fell = false;
        double speedRatio = 0.0;
        int speedSamples = 0;
        double assist = 0.0;
        double feetAssist = 0.0;
        int ticks = 0;
        // A arrancada (s ate 90% da velocidade pedida, nas retas; -1 nunca),
        // o escorregar do pe apoiado (m/s), a potencia dos motores (W/kg) e a
        // maior inclinacao do peito (graus).
        float reach = -1.0f;
        int reachTicks = 0;
        float reachStart = -1.0f;
        std::array<int, 2> contactTicks { 0, 0 };
        double slide = 0.0;
        int slideSamples = 0;
        double power = 0.0;
        float tilt = 0.0f;
    };
    std::vector<Tally> tally(count);
    float weight = 9.81f * context.profile->totalMassKg;
    const RagdollProfile3D& profile = *context.profile;
    std::array<std::size_t, 2> feet { profile.links.size(), profile.links.size() };
    std::size_t chest = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftFoot") feet[0] = i;
        if (profile.links[i].id == "RightFoot") feet[1] = i;
        if (profile.links[i].id == "Chest") chest = i;
    }
    for (std::size_t i = 0; i < count; ++i) {
        const Drill& drill = drills[i / 5];
        LocomotionWorld3D& world = batch.world(i);
        world.runtimeSettings().e5StandingAssistPercent = levels[i % 5];
        world.runtimeSettings().e5WalkingAssistPercent = levels[i % 5];
        // (XPES_NO_NIVEL=1: a ajuda nos pes tambem no nivel - hoje ela fica
        // em 100% mesmo com as outras em zero.)
        if (std::getenv("XPES_NO_NIVEL")) world.runtimeSettings().physicalFeetAssistPercent = levels[i % 5];
        world.setStatics(scenarioStatics(drill.scenario));
        world.reset(AthleteStartMode3D::Physical);
    }
    float longest = 0.0f;
    for (const Drill& drill : drills) longest = std::max(longest, drill.scenario.seconds);
    const int ticks = static_cast<int>(longest / Dt);
    double stepSeconds = 0.0;
    std::size_t worldTicks = 0;
    for (int step = 0; step < ticks; ++step) {
        const float t = static_cast<float>(step) * Dt;
        for (std::size_t i = 0; i < count; ++i) {
            const Scenario& scenario = drills[i / 5].scenario;
            LocomotionWorld3D& world = batch.world(i);
            const AthleteRuntimeSettings3D& settings = world.runtimeSettings();
            const bool running = t < scenario.seconds;
            batch.commands()[i] = running
                ? scenarioCommand(context, settings, scenario.keys(t), world.telemetry().terrainSpeedLimit)
                : scenarioCommand(context, settings, Keys {}, world.telemetry().terrainSpeedLimit);
            if (!running) continue;
            for (const Throw& item : scenario.throws)
                if (t >= item.at && t < item.at + Dt) world.throwBox({ item.start, item.velocity, item.massKg, item.halfExtents });
            for (const Push& push : scenario.pushes)
                if (t >= push.at && t < push.at + push.seconds) {
                    world.push(0, push.force * 0.20f);
                    world.push(2, push.force * 0.32f);
                    world.push(3, push.force * 0.48f);
                }
        }
        const auto begin = std::chrono::steady_clock::now();
        batch.step(1);
        stepSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        worldTicks += count;
        for (std::size_t i = 0; i < count; ++i) {
            const Scenario& scenario = drills[i / 5].scenario;
            if (t >= scenario.seconds || batch.failed(i)) continue;
            LocomotionWorld3D& world = batch.world(i);
            Tally& item = tally[i];
            const AthleteTelemetry3D telemetry = world.telemetry();
            if (telemetry.fallen || telemetry.gettingUp) item.fell = true;
            item.assist += world.athlete().legacy.adaptivePhysics.output().rootForceWorld.length() / weight;
            // (A ajuda nos pes entregue: as forcas dela sao pares internos, a
            // soma delas e zero - a medida e a da sonda, XAJUDA.)
            item.feetAssist += world.athlete().feetAssistOutput.deliveredForce.length() / weight;
            ++item.ticks;
            const Vec3 asked = batch.commands()[i].intent.requestedVelocityWorld;
            const float askedSpeed = std::hypot(asked.x, asked.y);
            if (drills[i / 5].group.rfind("reta_", 0) == 0 && t >= 2.0f && askedSpeed > 0.1f && !item.fell) {
                const Vec3 velocity = world.body().links.front().linearVelocity;
                item.speedRatio += (velocity.x * asked.x + velocity.y * asked.y) / (askedSpeed * askedSpeed);
                ++item.speedSamples;
            }
            const RagdollState3D& body = world.body();
            // (A arrancada: 90% da velocidade da tabela por 0,1 s seguidos -
            // contra a da tabela, nao a pedida: o limite do terreno do
            // controle fisico sobe em rampa na largada e a pedida vai junto.)
            const Vec3 nominal = scenarioCommand(context, world.runtimeSettings(), scenario.keys(t), 1.0e9f)
                                     .intent.requestedVelocityWorld;
            const float nominalSpeed = std::hypot(nominal.x, nominal.y);
            if (drills[i / 5].group.rfind("reta_", 0) == 0 && item.reach < 0.0f && nominalSpeed > 0.1f && !item.fell) {
                const Vec3 velocity = body.links.front().linearVelocity;
                if ((velocity.x * nominal.x + velocity.y * nominal.y) / nominalSpeed >= 0.9f * nominalSpeed) {
                    if (item.reachTicks++ == 0) item.reachStart = t;
                    if (item.reachTicks >= 12) item.reach = item.reachStart - 0.3f;
                } else {
                    item.reachTicks = 0;
                }
            }
            if (!item.fell) {
                for (std::size_t side = 0; side < 2; ++side) {
                    bool touching = false;
                    for (const RagdollInteraction3D& contact : body.interactions)
                        touching |= contact.linkIndex == feet[side] && contact.normalWorld.z > 0.5f;
                    // (O pe no chao ha 0,05 s: sem o impacto do pouso; o rolar do
                    // pe ainda conta - a medida e relativa, a mesma para o RL.)
                    item.contactTicks[side] = touching ? item.contactTicks[side] + 1 : 0;
                    if (item.contactTicks[side] < 6 || feet[side] >= body.links.size()) continue;
                    const Vec3 v = body.links[feet[side]].linearVelocity;
                    item.slide += std::hypot(v.x, v.y);
                    ++item.slideSamples;
                }
                double watts = 0.0;
                for (const RagdollJointState3D& joint : body.joints)
                    for (std::size_t axis = 0; axis < 3; ++axis)
                        watts += std::abs(joint.motorTorqueNewtonMeters[axis] * joint.velocityRadiansPerSecond[axis]);
                item.power += watts / std::max(1.0f, profile.totalMassKg);
                if (chest < body.links.size()) {
                    const Quaternion r = (body.links[chest].orientation
                        * profile.links[chest].modelOrientation.conjugate()).normalized();
                    const Vec3 up = r.rotate({ 0.0f, 0.0f, 1.0f });
                    item.tilt = std::max(item.tilt, std::acos(std::clamp(up.z, -1.0f, 1.0f)) * 57.2958f);
                }
            }
        }
    }
    int failedWorlds = 0;
    for (std::size_t i = 0; i < count; ++i) failedWorlds += batch.failed(i) ? 1 : 0;
    std::printf("linha de base: %zu exercicios x 5 niveis = %zu mundos, %u threads; fisico total, comandado, ajuda nos pes %s\n",
        drills.size(), count, batch.threads(), std::getenv("XPES_NO_NIVEL") ? "no nivel" : "100%");
    std::printf("vazao: %.0f ticks de mundo/s = %.0f decisoes/s a 30 Hz (%zu ticks de mundo em %.1f s de lote; %d mundos com erro)\n",
        static_cast<double>(worldTicks) / stepSeconds, static_cast<double>(worldTicks) / stepSeconds / 4.0, worldTicks,
        stepSeconds, failedWorlds);
    std::printf("nivel | velocidade/pedida nas retas (8 rumos: frente diag lado diag-tras tras) por marcha | quedas: retas manobras pancadas terreno | ajuda na pelve (peso; e nos pes)\n");
    for (int level = 0; level < 5; ++level) {
        std::printf("%4.0f%% |", levels[level]);
        int falls[4] = {}, totals[4] = {};
        double assist = 0.0, feetAssist = 0.0;
        int assistCount = 0;
        for (const auto& gait : gaits) {
            // (Por rumo: 0 frente, 45 e 315 diagonais, 90 e 270 lados, 135 e
            // 225 diagonais de tras, 180 tras.)
            double sums[5] = {};
            int counts[5] = {};
            for (std::size_t d = 0; d < drills.size(); ++d) {
                if (drills[d].group != std::string("reta_") + gait.key) continue;
                const Tally& item = tally[d * 5 + static_cast<std::size_t>(level)];
                const int k = std::atoi(drills[d].scenario.name.c_str() + drills[d].scenario.name.size() - 3) / 45;
                const int bucket = k == 0 ? 0 : (k == 1 || k == 7) ? 1 : (k == 2 || k == 6) ? 2 : (k == 3 || k == 5) ? 3 : 4;
                if (item.speedSamples > 0) {
                    sums[bucket] += item.speedRatio / item.speedSamples;
                    ++counts[bucket];
                }
            }
            std::printf(" %s", gait.key);
            for (int b = 0; b < 5; ++b)
                std::printf(counts[b] > 0 ? " %.2f" : "  -  ", counts[b] > 0 ? sums[b] / counts[b] : 0.0);
            std::printf(" |");
        }
        for (std::size_t d = 0; d < drills.size(); ++d) {
            const Tally& item = tally[d * 5 + static_cast<std::size_t>(level)];
            const std::string& group = drills[d].group;
            const int g = group.rfind("reta_", 0) == 0 ? 0 : group == "manobras" ? 1 : group == "pancadas" ? 2 : 3;
            ++totals[g];
            falls[g] += item.fell ? 1 : 0;
            if (item.ticks > 0) {
                assist += item.assist / item.ticks;
                feetAssist += item.feetAssist / item.ticks;
                ++assistCount;
            }
        }
        std::printf(" quedas %d/%d %d/%d %d/%d %d/%d | %.2f (pes %.2f)\n", falls[0], totals[0], falls[1], totals[1], falls[2],
            totals[2], falls[3], totals[3], assistCount > 0 ? assist / assistCount : 0.0,
            assistCount > 0 ? feetAssist / assistCount : 0.0);
    }
    // As medidas separadas do plano (a regua do RL): a arrancada nas retas de
    // corrida e sprint, o escorregar do pe apoiado, a energia e a
    // estabilidade (a maior inclinacao do peito; quase queda: alem de 35
    // graus sem cair).
    std::printf("nivel | arrancada ate 90%% por 0,1 s (s; corre, sprint) | o pe no chao (m/s, depois de 0,05 s de apoio) | energia dos motores (W/kg) | peito: inclinacao max media (graus), quase quedas\n");
    for (int level = 0; level < 5; ++level) {
        double reachRun = 0.0, reachSprint = 0.0, slide = 0.0, power = 0.0, tilt = 0.0;
        int reachRunCount = 0, reachSprintCount = 0, slideCount = 0, powerCount = 0, tiltCount = 0, nearFalls = 0;
        for (std::size_t d = 0; d < drills.size(); ++d) {
            const Tally& item = tally[d * 5 + static_cast<std::size_t>(level)];
            const std::string& group = drills[d].group;
            if (item.reach >= 0.0f && group == "reta_corre") { reachRun += item.reach; ++reachRunCount; }
            if (item.reach >= 0.0f && group == "reta_sprint") { reachSprint += item.reach; ++reachSprintCount; }
            if (group == "pancadas") continue;
            if (item.slideSamples > 0) { slide += item.slide / item.slideSamples; ++slideCount; }
            if (item.ticks > 0) { power += item.power / item.ticks; ++powerCount; }
            tilt += item.tilt;
            ++tiltCount;
            nearFalls += !item.fell && item.tilt > 35.0f ? 1 : 0;
        }
        const auto mean = [](double sum, int n) { return n > 0 ? sum / n : -1.0; };
        std::printf("%4.0f%% | %.2f (%d/8), %.2f (%d/8) | %.3f | %.1f | %.1f, %d de %d\n", levels[level],
            mean(reachRun, reachRunCount), reachRunCount, mean(reachSprint, reachSprintCount), reachSprintCount,
            mean(slide, slideCount), mean(power, powerCount), mean(tilt, tiltCount), nearFalls, tiltCount);
    }
    // As quedas por exercicio (onde o nivel derruba).
    std::printf("quem cai (niveis 100 70 50 30 0):\n");
    for (std::size_t d = 0; d < drills.size(); ++d) {
        bool any = false;
        for (int level = 0; level < 5; ++level) any = any || tally[d * 5 + static_cast<std::size_t>(level)].fell;
        if (!any) continue;
        std::printf("  %-20s", drills[d].scenario.name.c_str());
        for (int level = 0; level < 5; ++level)
            std::printf(" %s", tally[d * 5 + static_cast<std::size_t>(level)].fell ? "CAI" : " - ");
        std::printf("\n");
    }
    return failedWorlds == 0 ? 0 : 1;
}

} // namespace MatterEngine::Probe
