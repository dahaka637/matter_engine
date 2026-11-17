// (16/11) A sonda sem janela do cerebro unico no corpo do jogo, com a fisica do jogo: uma cena como a do laboratorio in-game (as
// configuracoes do jogo, o chao de concreto, uma bola de 0,43 kg), o ragdoll do jogador nela e o mesmo caminho do Fisico com RL - a
// cada tick de 1/120 s os alvos antes de simular, a decisao a cada 4 ticks, o resto dela depois do 4o. Mede a velocidade, o rumo contra
// o pedido, a altura da cabeca, a bola e as quedas sob forcas de verdade aplicadas no peito.
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingGetUpAmpTraining3D.hpp"
#include "Engine/Tracking/TrackingScene3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <thread>
#include <memory>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {
std::string procStatus(const char* key) {
    std::ifstream in("/proc/self/status");
    std::string line;
    while (std::getline(in, line))
        if (line.rfind(key, 0) == 0) return line;
    return {};
}

// (Os 22 jogadores: uma cena so, um cerebro por corpo, cada um com as suas teclas - o custo e as quedas de verdade.)
int crowd(int count, float spacing, const RagdollState3D& standing, const std::filesystem::path& run, const std::filesystem::path& policy) {
    namespace fs = std::filesystem;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    auto scene = engine.createScene(sceneSettings, materials);
    createTrackingGround3D(*scene, 2000.0f);
    std::vector<std::unique_ptr<TrackingViewer3D>> brains;
    std::vector<RagdollHandle3D> bodies;
    const auto built = std::chrono::steady_clock::now();
    for (int i = 0; i < count; ++i) {
        auto brain = std::make_unique<TrackingViewer3D>(fs::path(MATTERENGINE_ASSETS_DIR), 61 + i, "lev_comando", fs::path {}, 1.0f, 0.2f,
            TrackingViewerConditions3D {}, run, scene.get());
        std::string label;
        brain->setPolicy(policy, label);
        const Vec3 at { spacing * static_cast<float>(i % 6), spacing * static_cast<float>(i / 6), 0.0f };
        std::vector<PhysicsBodyState3D> links = standing.links;
        const Vec3 origin { standing.links.front().position.x, standing.links.front().position.y, 0.0f };
        for (PhysicsBodyState3D& l : links) l.position = l.position - origin + at;
        RagdollSpawnDefinition3D spawn;
        spawn.entityId = 100 + static_cast<std::uint64_t>(i);
        spawn.active = true;
        spawn.pelvisPosition = links.front().position;
        const RagdollHandle3D body = scene->createRagdoll(brain->profile(), spawn);
        scene->resetRagdollState(body, links);
        brain->adoptExternalBody(body);
        brains.push_back(std::move(brain));
        bodies.push_back(body);
    }
    std::printf("%d cerebros montados em %.2f s | %s | %s\n", count,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - built).count(), procStatus("VmRSS").c_str(),
        procStatus("Threads").c_str());
    std::uint32_t head = 0;
    for (std::size_t i = 0; i < brains.front()->profile().links.size(); ++i)
        if (brains.front()->profile().links[i].id == "Head") head = static_cast<std::uint32_t>(i);
    const float tick = TrackingGetUpAmpTraining3D::TickSeconds;
    double brainSeconds = 0.0, physicsSeconds = 0.0;
    std::vector<int> falls(count, 0);
    std::vector<bool> down(count, false);
    int tickInDecision = 0;
    float t = 0.0f;
    // (As teclas: cada um no seu tempo - parado, trote para um rumo, corrida, troca de rumo, para.)
    const auto keys = [](int i, float time, int& key, bool& sprint) {
        const float local = std::fmod(time + 0.7f * static_cast<float>(i), 12.0f);
        key = local < 2.0f ? -1 : local < 5.0f ? (i % 8) : local < 8.0f ? (i % 8) : local < 10.0f ? ((i + 3) % 8) : -1;
        sprint = local >= 5.0f && local < 10.0f;
    };
    while (t < 24.0f) {
        const auto a = std::chrono::steady_clock::now();
        for (int i = 0; i < count; ++i) {
            if (tickInDecision == 0) {
                int key = -1;
                bool sprint = false;
                keys(i, t, key, sprint);
                brains[i]->setPlayerInput(key, sprint, 0.0f);
                brains[i]->step();
            }
            brains[i]->applyExternalDrives();
        }
        const auto b = std::chrono::steady_clock::now();
        scene->simulate(tick);
        const auto c = std::chrono::steady_clock::now();
        if (++tickInDecision >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
            tickInDecision = 0;
            for (auto& brain : brains) brain->finishExternalDecision();
        }
        const auto d = std::chrono::steady_clock::now();
        brainSeconds += std::chrono::duration<double>(b - a).count() + std::chrono::duration<double>(d - c).count();
        physicsSeconds += std::chrono::duration<double>(c - b).count();
        t += tick;
        for (int i = 0; i < count; ++i) {
            const float h = scene->ragdollStateView(bodies[i])->links[head].position.z;
            if (!down[i] && h < 0.7f) {
                down[i] = true;
                ++falls[i];
            } else if (down[i] && h > 1.5f) {
                down[i] = false;
            }
        }
    }
    int total = 0, stillDown = 0;
    for (int i = 0; i < count; ++i) {
        if (falls[i] > 0) std::printf("  jogador %d (rumo %d -> %d): %d quedas%s\n", i, i % 8, (i + 3) % 8, falls[i], down[i] ? " (caido no fim)" : "");
        total += falls[i];
        if (down[i]) ++stillDown;
    }
    std::printf("%d jogadores, %.0f s: quedas %d (caidos no fim %d) | cerebros %.1f ms e fisica %.1f ms por segundo de jogo\n", count,
        t, total, stillDown, 1000.0 * brainSeconds / t, 1000.0 * physicsSeconds / t);
    return 0;
}
// (Os obstaculos do laboratorio: um corredor de paredes a 0,7 m do caminho, uma parede a frente para parar diante dela e um meio-fio de
// 20 cm para cruzar - quantas quedas perto deles, sem empurrao nenhum.)
int obstacles(const RagdollState3D& standing, const std::filesystem::path& run, const std::filesystem::path& policy) {
    namespace fs = std::filesystem;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    auto scene = engine.createScene(sceneSettings, materials);
    createTrackingGround3D(*scene, 2000.0f);
    const auto box = [&](Vec3 center, Vec3 half) {
        PhysicsShape3D shape;
        shape.type = PhysicsShapeType3D::Box;
        shape.halfExtents = half;
        shape.materialId = "concrete";
        PhysicsBodyDefinition3D def;
        def.motionType = PhysicsMotionType3D::Static;
        def.position = center;
        def.materialId = "concrete";
        (void)scene->createBody(def, std::span<const PhysicsShape3D>(&shape, 1));
    };
    const Vec3 p0 = standing.links.front().position;
    // O corredor: paredes dos dois lados, de x+2 a x+14, a 0,7 m da linha do corpo.
    box({ p0.x + 8.0f, p0.y + 0.85f, 1.0f }, { 6.0f, 0.15f, 1.0f });
    box({ p0.x + 8.0f, p0.y - 0.85f, 1.0f }, { 6.0f, 0.15f, 1.0f });
    // O meio-fio de 20 cm em x+20 e a parede final em x+30.
    box({ p0.x + 20.0f, p0.y, 0.10f }, { 0.3f, 3.0f, 0.10f });
    box({ p0.x + 30.0f, p0.y, 1.0f }, { 0.15f, 3.0f, 1.0f });
    TrackingViewer3D brain(fs::path(MATTERENGINE_ASSETS_DIR), 61, "lev_comando", fs::path {}, 1.0f, 0.2f, TrackingViewerConditions3D {}, run,
        scene.get());
    std::string label;
    brain.setPolicy(policy, label);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 7;
    spawn.active = true;
    spawn.pelvisPosition = p0;
    const RagdollHandle3D body = scene->createRagdoll(brain.profile(), spawn);
    scene->resetRagdollState(body, standing.links);
    brain.adoptExternalBody(body);
    std::uint32_t head = 0;
    for (std::size_t i = 0; i < brain.profile().links.size(); ++i)
        if (brain.profile().links[i].id == "Head") head = static_cast<std::uint32_t>(i);
    struct Phase { const char* name; int key; bool run; float seconds; };
    const Phase phases[] = { { "parado entre as paredes?", -1, false, 1.0f }, { "trote pelo corredor", 0, false, 4.5f },
        { "corrida, meio-fio", 0, true, 3.0f }, { "trote ate a parede", 0, false, 2.5f }, { "parado diante da parede", -1, false, 3.0f } };
    const float tick = TrackingGetUpAmpTraining3D::TickSeconds;
    int tickInDecision = 0, falls = 0;
    bool down = false;
    float t = 0.0f;
    for (const Phase& ph : phases) {
        float phaseTime = 0.0f, shown = 0.0f;
        while (phaseTime < ph.seconds - 1.0e-4f) {
            if (tickInDecision == 0) {
                brain.setPlayerInput(ph.key, ph.run, 0.0f);
                brain.step();
            }
            brain.applyExternalDrives();
            scene->simulate(tick);
            if (++tickInDecision >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
                tickInDecision = 0;
                brain.finishExternalDecision();
            }
            t += tick;
            phaseTime += tick;
            const RagdollState3D& s = *scene->ragdollStateView(body);
            const float h = s.links[head].position.z;
            if (!down && h < 0.7f) {
                down = true;
                ++falls;
                std::printf("  QUEDA t=%.2f em x=%.1f (%s)\n", t, s.links.front().position.x - p0.x, ph.name);
            } else if (down && h > 1.5f) {
                down = false;
            }
            if ((shown += tick) >= 0.5f - 1.0e-4f) {
                shown = 0.0f;
                const Vec3 v = rlBodyCenterOfMassVelocity3D(brain.profile(), s.links);
                std::printf("t=%5.1f %-26s x=%5.1f y=%5.2f vel %4.2f cabeca %.2f\n", t, ph.name, s.links.front().position.x - p0.x,
                    s.links.front().position.y - p0.y, std::hypot(v.x, v.y), h);
            }
        }
    }
    std::printf("obstaculos: quedas %d\n", falls);
    return 0;
}
// (As pancadas com ele andando: trotando e correndo com a tecla segurada, uma pancada de cada forca e rumo; depois de cada uma, se caiu,
// quanto demorou para voltar a andar - acima de 70% da velocidade de antes - e o estado do cerebro ao longo.)
int impacts(const RagdollState3D& standing, const std::filesystem::path& run, const std::filesystem::path& policy) {
    namespace fs = std::filesystem;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    auto scene = engine.createScene(sceneSettings, materials);
    createTrackingGround3D(*scene, 2000.0f);
    TrackingViewer3D brain(fs::path(MATTERENGINE_ASSETS_DIR), 61, "lev_comando", fs::path {}, 1.0f, 0.2f, TrackingViewerConditions3D {}, run,
        scene.get());
    std::string label;
    brain.setPolicy(policy, label);
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 7;
    spawn.active = true;
    spawn.pelvisPosition = standing.links.front().position;
    const RagdollHandle3D body = scene->createRagdoll(brain.profile(), spawn);
    scene->resetRagdollState(body, standing.links);
    brain.adoptExternalBody(body);
    // (MATTER_PROBE_ASSIST=0..1: a regua da ajuda auxiliar do painel, nas tres.)
    if (const char* assist = std::getenv("MATTER_PROBE_ASSIST"); assist != nullptr) {
        const float level = static_cast<float>(std::atof(assist));
        brain.setExternalAssist(level, level, level);
    }
    std::uint32_t head = 0, chest = 0;
    for (std::size_t i = 0; i < brain.profile().links.size(); ++i) {
        if (brain.profile().links[i].id == "Head") head = static_cast<std::uint32_t>(i);
        if (brain.profile().links[i].id == "UpperChest") chest = static_cast<std::uint32_t>(i);
    }
    const float tick = TrackingGetUpAmpTraining3D::TickSeconds;
    int tickInDecision = 0;
    float t = 0.0f;
    const auto advance = [&](int key, bool sprint, Vec3 force, float forceSeconds, float seconds, bool print) {
        float local = 0.0f, shown = 0.0f;
        while (local < seconds - 1.0e-4f) {
            if (tickInDecision == 0) {
                brain.setPlayerInput(key, sprint, 0.0f);
                brain.step();
            }
            brain.applyExternalDrives();
            if (local < forceSeconds) {
                // (Repartida como a pancada do laboratorio: pelve 20%, peito 32%, peito alto 48%.)
                scene->applyRagdollLinkForce(body, 0, force * 0.20f, {});
                scene->applyRagdollLinkForce(body, 2, force * 0.32f, {});
                scene->applyRagdollLinkForce(body, chest, force * 0.48f, {});
            }
            scene->simulate(tick);
            if (++tickInDecision >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
                tickInDecision = 0;
                brain.finishExternalDecision();
            }
            t += tick;
            local += tick;
            if (print && (shown += tick) >= 0.25f - 1.0e-4f && static_cast<int>(std::lround(t * 4.0f)) % 2 == 0) {
                shown = 0.0f;
                const RagdollState3D& s = *scene->ragdollStateView(body);
                const Vec3 v = rlBodyCenterOfMassVelocity3D(brain.profile(), s.links);
                std::printf("    +%.2f s vel %4.2f cabeca %.2f | %s\n", local, std::hypot(v.x, v.y), s.links[head].position.z,
                    brain.externalState().c_str());
            }
        }
    };
    const auto speedNow = [&]() {
        const RagdollState3D& s = *scene->ragdollStateView(body);
        const Vec3 v = rlBodyCenterOfMassVelocity3D(brain.profile(), s.links);
        return std::hypot(v.x, v.y);
    };
    struct Hit { bool sprint; float newtons; float side; };
    const Hit hits[] = { { false, 600.0f, 1.0f }, { false, 1200.0f, 1.0f }, { false, 1200.0f, 0.0f }, { true, 600.0f, 1.0f },
        { true, 1200.0f, 1.0f }, { true, 1200.0f, -1.0f }, { true, 2000.0f, 1.0f }, { false, 2000.0f, 0.0f } };
    int stuck = 0;
    for (const Hit& h : hits) {
        advance(0, h.sprint, {}, 0.0f, 3.0f, false);
        const float before = speedNow();
        // (A pancada: de lado - +y, a esquerda do rumo +x - ou de frente, contra o rumo; 0,2 s.)
        const Vec3 force = h.side != 0.0f ? Vec3 { 0.0f, h.newtons * h.side, 0.0f } : Vec3 { -h.newtons, 0.0f, 0.0f };
        std::printf("%s %4.0f N %s (antes %.2f m/s):\n", h.sprint ? "CORRENDO" : "TROTANDO", h.newtons,
            h.side > 0.0f ? "de lado" : h.side < 0.0f ? "do outro lado" : "de frente", before);
        float back = -1.0f, low = 10.0f;
        float elapsed = 0.0f;
        while (elapsed < 6.0f) {
            advance(0, h.sprint, force, elapsed < 0.2f ? 0.2f - elapsed : 0.0f, 0.25f, elapsed < 6.0f);
            elapsed += 0.25f;
            low = std::min(low, scene->ragdollStateView(body)->links[head].position.z);
            if (back < 0.0f && elapsed > 0.3f && speedNow() > 0.7f * before && brain.externalStanding()
                && scene->ragdollStateView(body)->links[head].position.z > 1.4f) back = elapsed;
            if (back >= 0.0f && elapsed > back + 0.5f) break;
        }
        if (back < 0.0f) ++stuck;
        std::printf("  -> %s, voltou a andar em %s\n", low < 0.7f ? "CAIU" : "de pe", back >= 0.0f ? (std::to_string(back) + " s").c_str() : "NAO VOLTOU (6 s)");
        // (Volta ao rumo +x: solta e para.)
        advance(-1, false, {}, 0.0f, 2.0f, false);
    }
    std::printf("pancadas: %d sem voltar a andar\n", stuck);
    return 0;
}
// (Os bonecos que nascem: um cerebro dirigindo na thread do jogo enquanto outros 8 carregam em segundo plano - como no jogo - e assumem
// corpos novos quando prontos; o pior tick da thread do jogo e as quedas.)
// (16/11, a volta dos treinos - o plano de melhoria do RL: as reguas do jogo para os focos dele) Com a rede de um treino (MATTER_PROBE_RUN,
// senao o cerebro_unico), na fisica do jogo:
// - a suavidade: as aceleracoes dos elos pelas posicoes (a segunda diferenca; o "corpo de cima" e a media dos elos fora das pernas, as maos
//   e a cabeca) - p95 e maximo - no cruzeiro de cada andadura e nas trocas de rumo correndo, com a velocidade de cada uma;
// - as pancadas: parado (4 lados x 400/900/1500 N), trotando e correndo (de lado, de frente e por tras x 600/1200/2000 N), 0,2 s no peito
//   como no laboratorio, cada uma do corpo de pe recomecado: caiu (cabeca < 0,7 m) e quanto levou para voltar (parado: assentado; andando,
//   70% da velocidade de antes).
// A ultima linha, "REGUA ...", tem os numeros do portao.
int ruler(const RagdollState3D& standing, const std::filesystem::path& run, const std::filesystem::path& policy) {
    namespace fs = std::filesystem;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    auto scene = engine.createScene(sceneSettings, materials);
    createTrackingGround3D(*scene, 2000.0f);
    TrackingViewer3D brain(fs::path(MATTERENGINE_ASSETS_DIR), 61, "lev_comando", fs::path {}, 1.0f, 0.2f, TrackingViewerConditions3D {}, run,
        scene.get());
    std::string label;
    brain.setPolicy(policy, label);
    std::printf("rede: %s\n", label.c_str());
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 7;
    spawn.active = true;
    spawn.pelvisPosition = standing.links.front().position;
    const RagdollHandle3D body = scene->createRagdoll(brain.profile(), spawn);
    const RagdollProfile3D& profile = brain.profile();
    std::uint32_t head = 0, chest = 0;
    std::vector<char> upper(profile.links.size(), 1);
    std::vector<std::uint32_t> hands;
    std::vector<std::uint32_t> feet[2];
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id == "Head") head = static_cast<std::uint32_t>(i);
        if (id == "UpperChest") chest = static_cast<std::uint32_t>(i);
        if (id == "LeftHand" || id == "RightHand") hands.push_back(static_cast<std::uint32_t>(i));
        if (id.rfind("Left", 0) == 0 && (id.find("Foot") != std::string::npos || id.find("Toe") != std::string::npos))
            feet[0].push_back(static_cast<std::uint32_t>(i));
        if (id.rfind("Right", 0) == 0 && (id.find("Foot") != std::string::npos || id.find("Toe") != std::string::npos))
            feet[1].push_back(static_cast<std::uint32_t>(i));
        if (id.find("Thigh") != std::string::npos || id.find("Shin") != std::string::npos || id.find("Foot") != std::string::npos
            || id.find("Toe") != std::string::npos)
            upper[i] = 0;
    }
    const float tick = TrackingGetUpAmpTraining3D::TickSeconds;
    int tickInDecision = 0;
    // O corpo de pe recomecado na origem, adotado do zero.
    const auto restart = [&]() {
        scene->resetRagdollState(body, standing.links);
        brain.adoptExternalBody(body);
        tickInDecision = 0;
    };
    struct Window {
        std::vector<float> upperA, handsA, headA;
        // (16/11 18h05) As mesmas aceleracoes com as posicoes de 30 em 30 Hz (a cada decisao, como a regua do treino): a de 120 Hz mede
        // sobretudo a vibracao de milimetros entre as decisoes (100 m/s2 a 30 Hz sao ~3 mm), a de 30 Hz o tranco que se ve.
        std::vector<float> upper30, hands30, head30;
        // (16/11 21h, ele: "corrida e trote ele esta bem torto... e antigo") O torto: os pousos de cada pe (o contato comecando: tempo, lado,
        // posicao) e a inclinacao de lado do peito e da pelve contra o rumo andado, a cada tick.
        struct Touch { float t; int side; Vec3 at; };
        std::vector<Touch> touches;
        double chestRoll = 0.0, pelvisRoll = 0.0;
        int rollN = 0;
        bool footDown[2] { true, true };
        float clock = 0.0f;
        double speed = 0.0;
        int n = 0;
    };
    std::vector<Vec3> p1, p2, q1, q2;
    const auto advance = [&](int key, bool sprint, Vec3 force, float forceSeconds, float seconds, Window* window) {
        float local = 0.0f;
        while (local < seconds - 1.0e-4f) {
            if (tickInDecision == 0) {
                brain.setPlayerInput(key, sprint, 0.0f);
                brain.step();
            }
            brain.applyExternalDrives();
            if (local < forceSeconds) {
                scene->applyRagdollLinkForce(body, 0, force * 0.20f, {});
                scene->applyRagdollLinkForce(body, 2, force * 0.32f, {});
                scene->applyRagdollLinkForce(body, chest, force * 0.48f, {});
            }
            scene->simulate(tick);
            if (++tickInDecision >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
                tickInDecision = 0;
                brain.finishExternalDecision();
            }
            local += tick;
            const RagdollState3D& s = *scene->ragdollStateView(body);
            if (tickInDecision == 0) {
                if (window != nullptr && q2.size() == s.links.size()) {
                    const float dt = tick * static_cast<float>(TrackingGetUpAmpTraining3D::TicksPerDecision);
                    float sum = 0.0f, handMax = 0.0f;
                    int count = 0;
                    for (std::size_t i = 0; i < s.links.size(); ++i) {
                        const float a = (s.links[i].position - q1[i] * 2.0f + q2[i]).length() / (dt * dt);
                        if (upper[i]) { sum += a; ++count; }
                        for (const std::uint32_t h : hands) if (h == i) handMax = std::max(handMax, a);
                    }
                    window->upper30.push_back(count > 0 ? sum / static_cast<float>(count) : 0.0f);
                    window->hands30.push_back(handMax);
                    window->head30.push_back((s.links[head].position - q1[head] * 2.0f + q2[head]).length() / (dt * dt));
                }
                q2 = q1;
                q1.resize(s.links.size());
                for (std::size_t i = 0; i < s.links.size(); ++i) q1[i] = s.links[i].position;
            }
            if (window != nullptr) {
                window->clock += tick;
                bool down[2] { false, false };
                for (const RagdollContactPoint3D& c : s.contacts)
                    for (int side = 0; side < 2; ++side)
                        for (const std::uint32_t f : feet[side]) if (c.linkIndex == f) down[side] = true;
                for (int side = 0; side < 2; ++side) {
                    if (down[side] && !window->footDown[side])
                        window->touches.push_back({ window->clock, side, s.links[feet[side].front()].position });
                    window->footDown[side] = down[side];
                }
                const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, s.links);
                const float planar = std::hypot(v.x, v.y);
                if (planar > 1.0f) {
                    // (De lado: o eixo de cima do elo contra a esquerda do rumo andado; + inclinado para a esquerda.)
                    const Vec3 left { -v.y / planar, v.x / planar, 0.0f };
                    const auto rollOf = [&](std::size_t k) {
                        const Vec3 up = (s.links[k].orientation * profile.links[k].modelOrientation.conjugate()).rotate({ 0.0f, 0.0f, 1.0f });
                        return std::asin(std::clamp(dot(up, left), -1.0f, 1.0f)) * 57.2958f;
                    };
                    window->chestRoll += rollOf(chest);
                    window->pelvisRoll += rollOf(0);
                    ++window->rollN;
                }
            }
            if (window != nullptr && p2.size() == s.links.size()) {
                float sum = 0.0f, handMax = 0.0f;
                int count = 0;
                for (std::size_t i = 0; i < s.links.size(); ++i) {
                    const float a = (s.links[i].position - p1[i] * 2.0f + p2[i]).length() / (tick * tick);
                    if (upper[i]) { sum += a; ++count; }
                    for (const std::uint32_t h : hands) if (h == i) handMax = std::max(handMax, a);
                }
                window->upperA.push_back(count > 0 ? sum / static_cast<float>(count) : 0.0f);
                window->handsA.push_back(handMax);
                window->headA.push_back((s.links[head].position - p1[head] * 2.0f + p2[head]).length() / (tick * tick));
                const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, s.links);
                window->speed += std::hypot(v.x, v.y);
                ++window->n;
            }
            p2 = p1;
            p1.resize(s.links.size());
            for (std::size_t i = 0; i < s.links.size(); ++i) p1[i] = s.links[i].position;
        }
    };
    const auto percentile = [](std::vector<float> v, float q) {
        if (v.empty()) return 0.0f;
        std::sort(v.begin(), v.end());
        return v[std::min(v.size() - 1, static_cast<std::size_t>(q * static_cast<float>(v.size() - 1)))];
    };
    // A suavidade.
    struct Gait { const char* name; int key; bool sprint; };
    const Gait gaits[] = { { "trote frente", 0, false }, { "corrida frente", 0, true }, { "corrida diagonal", 1, true }, { "trote lateral", 2, false },
        { "trote costas", 4, false }, { "parado", -1, false } };
    double upperSum = 0.0, handsSum = 0.0, headSum = 0.0, upper30Sum = 0.0, hands30Sum = 0.0, head30Sum = 0.0;
    int segments = 0;
    const auto at30 = [&](const char* name, const Window& w) {
        const float cima = percentile(w.upper30, 0.95f), maos = percentile(w.hands30, 0.95f), cabeca = percentile(w.head30, 0.95f);
        std::printf("suave30 %-16s | p95 corpo de cima %5.1f maos %5.1f cabeca %5.1f m/s2 (30 Hz)\n", name, cima, maos, cabeca);
        upper30Sum += cima; hands30Sum += maos; head30Sum += cabeca;
    };
    std::string speeds, tortoLine;
    for (const Gait& g : gaits) {
        restart();
        p1.clear(); p2.clear(); q1.clear(); q2.clear();
        advance(g.key, g.sprint, {}, 0.0f, 3.0f, nullptr);
        Window w;
        advance(g.key, g.sprint, {}, 0.0f, 4.0f, &w);
        const float cima = percentile(w.upperA, 0.95f), maos = percentile(w.handsA, 0.95f), cabeca = percentile(w.headA, 0.95f);
        const float vel = w.n > 0 ? static_cast<float>(w.speed / w.n) : 0.0f;
        std::printf("suave %-18s vel %4.2f m/s | p95 corpo de cima %5.1f maos %5.1f cabeca %5.1f m/s2 | max corpo de cima %5.1f\n", g.name, vel, cima,
            maos, cabeca, w.upperA.empty() ? 0.0f : *std::max_element(w.upperA.begin(), w.upperA.end()));
        upperSum += cima; handsSum += maos; headSum += cabeca; ++segments;
        at30(g.name, w);
        if (g.key == 0) {
            // (O torto, nas retas de frente: o tempo e o comprimento do passo de cada lado - de um pouso do outro pe ao dele - e a
            // assimetria deles; a inclinacao de lado media do peito e da pelve.)
            double time[2] = { 0.0, 0.0 }, length[2] = { 0.0, 0.0 };
            int count[2] = { 0, 0 };
            for (std::size_t k = 1; k < w.touches.size(); ++k) {
                const auto& a = w.touches[k - 1];
                const auto& b = w.touches[k];
                if (a.side == b.side) continue;
                time[b.side] += b.t - a.t;
                length[b.side] += std::hypot(b.at.x - a.at.x, b.at.y - a.at.y);
                ++count[b.side];
            }
            for (int side = 0; side < 2; ++side) if (count[side] > 0) { time[side] /= count[side]; length[side] /= count[side]; }
            const double tm = 0.5 * (time[0] + time[1]), lm = 0.5 * (length[0] + length[1]);
            const double at = tm > 1e-6 ? 100.0 * std::abs(time[0] - time[1]) / tm : 0.0, al = lm > 1e-6 ? 100.0 * std::abs(length[0] - length[1]) / lm : 0.0;
            const double cr = w.rollN > 0 ? w.chestRoll / w.rollN : 0.0, pr = w.rollN > 0 ? w.pelvisRoll / w.rollN : 0.0;
            std::printf("torto %-18s | passo E/D %.3f/%.3f s compr E/D %.2f/%.2f m (%d/%d passos) | assimetria tempo %.0f%% compr %.0f%% | de lado peito %+.1f pelve %+.1f graus\n",
                g.name, time[0], time[1], length[0], length[1], count[0], count[1], at, al, cr, pr);
            tortoLine += std::string(tortoLine.empty() ? "" : " | ") + (g.sprint ? "corrida" : "trote");
            char item[96];
            std::snprintf(item, sizeof item, " assim %.0f%% peito %+.1f", 0.5 * (at + al), cr);
            tortoLine += item;
        }
        char item[48];
        std::snprintf(item, sizeof item, "%s%.2f", speeds.empty() ? "" : "/", vel);
        speeds += item;
    }
    {
        // As trocas de rumo correndo: frente -> esquerda -> frente -> direita, 1 s cada.
        restart();
        p1.clear(); p2.clear(); q1.clear(); q2.clear();
        advance(0, true, {}, 0.0f, 3.0f, nullptr);
        Window w;
        for (const int key : { 2, 0, 6, 0 }) advance(key, true, {}, 0.0f, 1.0f, &w);
        const float cima = percentile(w.upperA, 0.95f), maos = percentile(w.handsA, 0.95f), cabeca = percentile(w.headA, 0.95f);
        std::printf("suave %-18s vel %4.2f m/s | p95 corpo de cima %5.1f maos %5.1f cabeca %5.1f m/s2\n", "trocas correndo",
            w.n > 0 ? static_cast<float>(w.speed / w.n) : 0.0f, cima, maos, cabeca);
        upperSum += cima; handsSum += maos; headSum += cabeca; ++segments;
        at30("trocas correndo", w);
    }
    std::printf("SUAVE30 cima %.1f maos %.1f cabeca %.1f\n", upper30Sum / segments, hands30Sum / segments, head30Sum / segments);
    std::printf("TORTO %s\n", tortoLine.c_str());
    // As pancadas.
    struct Hit { const char* group; int key; bool sprint; Vec3 direction; float newtons; };
    std::vector<Hit> hits;
    for (const float n : { 400.0f, 900.0f, 1500.0f })
        for (const Vec3 d : { Vec3 { 1, 0, 0 }, Vec3 { -1, 0, 0 }, Vec3 { 0, 1, 0 }, Vec3 { 0, -1, 0 } }) hits.push_back({ "parado", -1, false, d, n });
    for (const bool sprint : { false, true })
        for (const float n : { 600.0f, 1200.0f, 2000.0f })
            for (const Vec3 d : { Vec3 { 0, 1, 0 }, Vec3 { -1, 0, 0 }, Vec3 { 1, 0, 0 } }) hits.push_back({ sprint ? "corrida" : "trote", 0, sprint, d, n });
    int upright = 0, total = 0, groupUp[3] = { 0, 0, 0 }, groupN[3] = { 0, 0, 0 };
    double backSum = 0.0;
    int backN = 0;
    for (const Hit& h : hits) {
        restart();
        p1.clear(); p2.clear(); q1.clear(); q2.clear();
        advance(h.key, h.sprint, {}, 0.0f, h.key < 0 ? 2.0f : 3.0f, nullptr);
        const RagdollState3D& s0 = *scene->ragdollStateView(body);
        const Vec3 v0 = rlBodyCenterOfMassVelocity3D(profile, s0.links);
        const float before = std::hypot(v0.x, v0.y);
        float low = 10.0f, back = -1.0f, elapsed = 0.0f;
        while (elapsed < 5.0f) {
            advance(h.key, h.sprint, h.direction * h.newtons, elapsed < 0.2f ? 0.2f - elapsed : 0.0f, 0.1f, nullptr);
            elapsed += 0.1f;
            const RagdollState3D& s = *scene->ragdollStateView(body);
            low = std::min(low, s.links[head].position.z);
            const Vec3 v = rlBodyCenterOfMassVelocity3D(profile, s.links);
            const bool settled = h.key < 0 ? std::hypot(v.x, v.y) < 0.3f : std::hypot(v.x, v.y) > 0.7f * before;
            if (back < 0.0f && elapsed > 0.3f && settled && brain.externalStanding() && s.links[head].position.z > 1.4f) back = elapsed;
            if (back >= 0.0f && elapsed > back + 0.3f) break;
        }
        const bool fell = low < 0.7f;
        const int g = h.key < 0 ? 0 : h.sprint ? 2 : 1;
        ++total; ++groupN[g];
        if (!fell) { ++upright; ++groupUp[g]; }
        if (back >= 0.0f) { backSum += back; ++backN; }
        std::printf("pancada %-8s %4.0f N (%+.0f, %+.0f): %s, voltou %s\n", h.group, h.newtons, h.direction.x, h.direction.y, fell ? "CAIU" : "de pe",
            back >= 0.0f ? (std::to_string(back).substr(0, 4) + " s").c_str() : "nao (5 s)");
    }
    std::printf("REGUA suave p95 cima %.1f maos %.1f cabeca %.1f | vel %s | pancadas de pe %d/%d (parado %d/%d trote %d/%d corrida %d/%d) volta %.2f s\n",
        upperSum / segments, handsSum / segments, headSum / segments, speeds.c_str(), upright, total, groupUp[0], groupN[0], groupUp[1], groupN[1],
        groupUp[2], groupN[2], backN > 0 ? backSum / backN : -1.0);
    return 0;
}

int asyncSpawn(const RagdollState3D& standing, const std::filesystem::path& run, const std::filesystem::path& policy) {
    namespace fs = std::filesystem;
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    auto scene = engine.createScene(sceneSettings, materials);
    createTrackingGround3D(*scene, 2000.0f);
    struct Bot {
        std::unique_ptr<TrackingViewer3D> brain;
        std::future<std::unique_ptr<TrackingViewer3D>> loading;
        RagdollHandle3D body;
        bool adopted = false;
        int tick = 0;
    };
    std::vector<Bot> bots(9);
    PhysicsScene3D* raw = scene.get();
    for (std::size_t i = 0; i < bots.size(); ++i)
        bots[i].loading = std::async(std::launch::async, [run, policy, raw, i]() {
            auto brain = std::make_unique<TrackingViewer3D>(fs::path(MATTERENGINE_ASSETS_DIR), 61 + i, "lev_comando", fs::path {}, 1.0f, 0.2f,
                TrackingViewerConditions3D {}, run, raw);
            std::string label;
            brain->setPolicy(policy, label);
            return brain;
        });
    const float tick = TrackingGetUpAmpTraining3D::TickSeconds;
    double worst = 0.0;
    float t = 0.0f, firstReady = -1.0f, allReady = -1.0f;
    std::uint32_t head = 0;
    int falls = 0;
    std::vector<bool> down(bots.size(), false);
    // (No tempo do jogo: um tick a cada 1/120 s de relogio, como o jogo - o carregamento corre em paralelo de verdade.)
    auto next = std::chrono::steady_clock::now();
    while (t < 12.0f) {
        std::this_thread::sleep_until(next);
        next += std::chrono::microseconds(8333);
        const auto a = std::chrono::steady_clock::now();
        int ready = 0;
        for (std::size_t i = 0; i < bots.size(); ++i) {
            Bot& b = bots[i];
            if (!b.brain && b.loading.valid() && b.loading.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                b.brain = b.loading.get();
                for (std::size_t k = 0; k < b.brain->profile().links.size(); ++k)
                    if (b.brain->profile().links[k].id == "Head") head = static_cast<std::uint32_t>(k);
                std::vector<PhysicsBodyState3D> links = standing.links;
                const Vec3 origin { standing.links.front().position.x, standing.links.front().position.y, 0.0f };
                const Vec3 at { 30.0f * static_cast<float>(i % 3), 30.0f * static_cast<float>(i / 3), 0.0f };
                for (PhysicsBodyState3D& l : links) l.position = l.position - origin + at;
                RagdollSpawnDefinition3D spawn;
                spawn.entityId = 100 + i;
                spawn.active = true;
                spawn.pelvisPosition = links.front().position;
                b.body = scene->createRagdoll(b.brain->profile(), spawn);
                scene->resetRagdollState(b.body, links);
                b.brain->setPlayerInput(-1, false, 0.0f);
                b.brain->adoptExternalBody(b.body);
                b.adopted = true;
            }
            if (!b.adopted) continue;
            ++ready;
            if (b.tick == 0) {
                const bool moving = std::fmod(t + static_cast<float>(i), 6.0f) > 3.0f;
                b.brain->setPlayerInput(moving ? static_cast<int>(i % 8) : -1, false, 0.0f);
                b.brain->step();
            }
            b.brain->applyExternalDrives();
        }
        scene->simulate(tick);
        for (std::size_t i = 0; i < bots.size(); ++i) {
            Bot& b = bots[i];
            if (!b.adopted) continue;
            if (++b.tick >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
                b.tick = 0;
                b.brain->finishExternalDecision();
            }
            const float h = scene->ragdollStateView(b.body)->links[head].position.z;
            if (!down[i] && h < 0.7f) { down[i] = true; ++falls; }
            else if (down[i] && h > 1.5f) down[i] = false;
        }
        worst = std::max(worst, std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count());
        if (ready > 0 && firstReady < 0.0f) firstReady = t;
        if (ready == static_cast<int>(bots.size()) && allReady < 0.0f) allReady = t;
        t += tick;
    }
    std::printf("async: o primeiro pronto em %.2f s, todos em %.2f s | pior tick da thread do jogo %.2f ms | quedas %d\n", firstReady, allReady,
        worst * 1000.0, falls);
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    // (MATTER_PROBE_RUN: a rede de outro treino - o braco do A/B; senao a do cerebro_unico.)
    const char* runEnv = std::getenv("MATTER_PROBE_RUN");
    const fs::path run = runEnv != nullptr && *runEnv != '\0' ? fs::path(runEnv)
                                                               : fs::path(std::getenv("HOME")) / ".local/share/matter-engine/rl/runs/cerebro_unico";
    const fs::path policy = run / "exports" / "policy_latest.mrlpolicy";
    // Uma pose de pe: o cerebro no mundo do treino por 1 s (so para a pose de comeco da sonda).
    RagdollState3D standing;
    {
        TrackingViewer3D warm(fs::path(MATTERENGINE_ASSETS_DIR), 61, "lev_comando", fs::path {}, 1.0f, 0.2f, TrackingViewerConditions3D {}, run);
        std::string label;
        warm.setPolicy(policy, label);
        for (int k = 0; k < 120; ++k) {
            warm.setPlayerInput(-1, false, 0.0f);
            warm.step();
        }
        standing = *warm.view().state;
    }
    if (argc > 1 && std::string(argv[1]) == "obst") return obstacles(standing, run, policy);
    if (argc > 1 && std::string(argv[1]) == "impacto") return impacts(standing, run, policy);
    if (argc > 1 && std::string(argv[1]) == "regua") return ruler(standing, run, policy);
    if (argc > 1 && std::string(argv[1]) == "async") return asyncSpawn(standing, run, policy);
    if (argc > 1) return crowd(std::atoi(argv[1]), argc > 2 ? static_cast<float>(std::atof(argv[2])) : 6.0f, standing, run, policy);
    // A cena do jogo (WorkbenchApp::onStart) e o chao de concreto do laboratorio.
    PhysicsEngine3D engine;
    MaterialLibrary materials;
    PhysicsSceneSettings3D sceneSettings;
    sceneSettings.enableContinuousCollision = true;
    sceneSettings.enableStabilization = true;
    auto scene = engine.createScene(sceneSettings, materials);
    createTrackingGround3D(*scene, 200.0f);
    TrackingViewer3D brain(fs::path(MATTERENGINE_ASSETS_DIR), 61, "lev_comando", fs::path {}, 1.0f, 0.2f, TrackingViewerConditions3D {}, run,
        scene.get());
    std::string label;
    brain.setPolicy(policy, label);
    std::printf("rede: %s | hasPolicy %d\n", label.c_str(), brain.hasPolicy() ? 1 : 0);
    const RagdollProfile3D& profile = brain.profile();
    std::uint32_t head = 0, chest = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "Head") head = static_cast<std::uint32_t>(i);
        if (profile.links[i].id == "UpperChest") chest = static_cast<std::uint32_t>(i);
    }
    // O corpo do jogador como o jogo cria (WorkbenchApp::adoptRlBrainBody): o perfil do cerebro, na pose e nas velocidades de agora.
    RagdollSpawnDefinition3D spawn;
    spawn.entityId = 7;
    spawn.active = true;
    spawn.pelvisPosition = standing.links.front().position;
    const RagdollHandle3D body = scene->createRagdoll(profile, spawn);
    scene->resetRagdollState(body, standing.links);
    // (MATTER_PROBE_TENSION=0: o corpo sem a tensao dos limites, como o jogo criava.)
    if (const char* tension = std::getenv("MATTER_PROBE_TENSION"); tension != nullptr) scene->setRagdollLimitTension(body, tension[0] != '0');
    brain.adoptExternalBody(body);
    // A bola: 0,43 kg, raio 0,11 m, a 6 m a frente do corpo no rumo +x.
    PhysicsShape3D ballShape;
    ballShape.type = PhysicsShapeType3D::Sphere;
    ballShape.radius = 0.11f;
    ballShape.materialId = "rubber";
    PhysicsBodyDefinition3D ballDef;
    ballDef.massKg = 0.43f;
    ballDef.materialId = "rubber";
    const Vec3 pelvis0 = standing.links.front().position;
    ballDef.position = { pelvis0.x + 6.0f, pelvis0.y, 0.11f };
    ballDef.collisionMode = PhysicsCollisionMode3D::Continuous;
    PhysicsBodyHandle3D ball = scene->createBody(ballDef, std::span<const PhysicsShape3D>(&ballShape, 1));

    struct Phase { const char* name; int key; bool run; float seconds; float pushNewtons; bool adopt = false; };
    const Phase phases[] = {
        { "parado", -1, false, 2.0f, 0.0f },
        { "trote frente (bola a 6 m)", 0, false, 3.0f, 0.0f },
        { "corrida frente", 0, true, 3.0f, 0.0f },
        { "ASSUME correndo (padrao->fisico)", 0, true, 2.0f, 0.0f, true },
        { "solta (para)", -1, false, 2.0f, 0.0f },
        { "empurrao 400 N lado 0,25 s", -1, false, 2.5f, 400.0f },
        { "corrida esquerda", 2, true, 2.5f, 0.0f },
        { "ASSUME correndo esquerda", 2, true, 1.5f, 0.0f, true },
        { "meia-volta corrida tras", 4, true, 2.5f, 0.0f },
        { "corrida diagonal", 1, true, 2.5f, 0.0f },
        { "solta", -1, false, 2.0f, 0.0f },
        { "pancada 2500 N frente 0,2 s", -1, false, 1.2f, 2500.0f },
        { "levantando de pe: aperta W+shift", 0, true, 4.0f, 0.0f },
        { "corre", 0, true, 2.0f, 0.0f },
        { "solta", -1, false, 2.5f, 0.0f } };
    const float tick = TrackingGetUpAmpTraining3D::TickSeconds;
    Vec3 ballStart = scene->bodyState(ball).position;
    float t = 0.0f, lowest = 10.0f;
    int tickInDecision = 0, falls = 0;
    bool down = false;
    double brainSeconds = 0.0, physicsSeconds = 0.0;
    for (const Phase& ph : phases) {
        float shown = 0.0f, phaseTime = 0.0f;
        if (ph.adopt) {
            // (A troca do padrao para o fisico no jogo: o cerebro assume o corpo como ele esta, no meio da decisao.)
            if (tickInDecision != 0) brain.finishExternalDecision();
            brain.setPlayerInput(ph.key, ph.run, 0.0f);
            brain.adoptExternalBody(body);
            tickInDecision = 0;
        }
        if (&ph == &phases[2]) {
            // (A bola 4 m a frente da pelve, no rumo +x do trote.)
            const Vec3 p = scene->ragdollStateView(body)->links.front().position;
            scene->destroyBody(ball);
            ballDef.position = { p.x + 4.0f, p.y, 0.11f };
            ball = scene->createBody(ballDef, std::span<const PhysicsShape3D>(&ballShape, 1));
            ballStart = ballDef.position;
        }
        while (phaseTime < ph.seconds - 1.0e-4f) {
            const auto a = std::chrono::steady_clock::now();
            if (tickInDecision == 0) {
                brain.setPlayerInput(ph.key, ph.run, 0.0f);
                brain.step();
            }
            brain.applyExternalDrives();
            const auto b = std::chrono::steady_clock::now();
            if (ph.pushNewtons > 0.0f && phaseTime < (ph.pushNewtons > 1000.0f ? 0.2f : 0.25f)) {
                const Vec3 force = ph.pushNewtons > 1000.0f ? Vec3 { ph.pushNewtons, 0.0f, 0.0f } : Vec3 { 0.0f, ph.pushNewtons, 0.0f };
                scene->applyRagdollLinkForce(body, chest, force, {});
            }
            scene->simulate(tick);
            const auto c = std::chrono::steady_clock::now();
            if (++tickInDecision >= TrackingGetUpAmpTraining3D::TicksPerDecision) {
                tickInDecision = 0;
                brain.finishExternalDecision();
            }
            const auto d = std::chrono::steady_clock::now();
            brainSeconds += std::chrono::duration<double>(b - a).count() + std::chrono::duration<double>(d - c).count();
            physicsSeconds += std::chrono::duration<double>(c - b).count();
            t += tick;
            phaseTime += tick;
            const RagdollState3D& s = *scene->ragdollStateView(body);
            const float h = s.links[head].position.z;
            lowest = std::min(lowest, h);
            if (!down && h < 0.7f) {
                down = true;
                ++falls;
            } else if (down && h > 1.5f) {
                down = false;
            }
            if ((shown += tick) >= 0.5f - 1.0e-4f) {
                shown = 0.0f;
                const Vec3 vel = rlBodyCenterOfMassVelocity3D(profile, s.links);
                const float speed = std::hypot(vel.x, vel.y);
                const float course = speed > 0.3f ? std::atan2(vel.y, vel.x) * 57.2958f : 0.0f;
                const float wanted = ph.key >= 0 ? 45.0f * ph.key : 0.0f;
                const Vec3 bp = scene->bodyState(ball).position;
                const Vec3 pp = s.links.front().position;
                std::printf("t=%5.1f %-30s vel %4.2f m/s rumo %6.1f (pedido %6.1f) cabeca %.2f m pelve (%.1f, %.1f) bola (%.1f, %.1f) andou %.2f m\n",
                    t, ph.name, speed, course, ph.key >= 0 ? (wanted > 180 ? wanted - 360 : wanted) : 0.0f, h, pp.x, pp.y, bp.x, bp.y,
                    std::hypot(bp.x - ballStart.x, bp.y - ballStart.y));
            }
        }
    }
    std::printf("quedas %d | cabeca mais baixa %.2f m | cerebro %.2f ms e fisica %.2f ms por segundo de jogo (%.1f s)\n", falls, lowest,
        1000.0 * brainSeconds / static_cast<double>(t), 1000.0 * physicsSeconds / static_cast<double>(t), t);
    return 0;
}
