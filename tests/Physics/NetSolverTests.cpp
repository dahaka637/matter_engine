// A rede de fios sozinha (Engine/Physics/Net, sem o motor de fisica): um pano
// de rede de gol (malha de 12 cm, fio de 4 mm, 4% de folga) preso nas bordas.
// Parada ela pendura sem esticar os fios; a bola a 35 m/s em qualquer ponto
// (meio, canto, embaixo perto da trave) estufa a rede e nunca a atravessa, e
// para numa profundidade plausivel; o mesmo chute da o mesmo resultado; a rede
// dorme depois e, dormindo, segura a bola apoiada nela.

#include "Engine/Physics/Net/NetSolver3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace MatterEngine;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

constexpr float Mesh = 0.12f;
constexpr float Slack = 0.04f;
constexpr float TwineDensity = 0.007f;
constexpr float Tick = 1.0f / 120.0f;
constexpr std::uint32_t Substeps = 16;

// Um pano retangular: largura ao longo de u, altura ao longo de v, a partir de
// origin; todas as bordas presas.
NetDescription3D panel(Vec3 origin, Vec3 u, Vec3 v, float width, float height) {
    NetDescription3D net;
    const int columns = std::max(1, static_cast<int>(std::lround(width / Mesh)));
    const int rows = std::max(1, static_cast<int>(std::lround(height / Mesh)));
    const float du = width / static_cast<float>(columns), dv = height / static_cast<float>(rows);
    const auto index = [&](int c, int r) { return static_cast<std::uint32_t>(r * (columns + 1) + c); };
    for (int r = 0; r <= rows; ++r)
        for (int c = 0; c <= columns; ++c) {
            net.positions.push_back(origin + u * (du * static_cast<float>(c)) + v * (dv * static_cast<float>(r)));
            net.pinned.push_back(c == 0 || r == 0 || c == columns || r == rows ? 1 : 0);
        }
    net.masses.assign(net.positions.size(), 0.0f);
    const auto link = [&](std::uint32_t a, std::uint32_t b, float span) {
        const float rest = span * (1.0f + Slack);
        net.segments.push_back({ a, b, rest, 10000.0f });
        net.masses[a] += 0.5f * rest * TwineDensity;
        net.masses[b] += 0.5f * rest * TwineDensity;
    };
    for (int r = 0; r <= rows; ++r)
        for (int c = 0; c <= columns; ++c) {
            if (c < columns) link(index(c, r), index(c + 1, r), du);
            if (r < rows) link(index(c, r), index(c, r + 1), dv);
        }
    // As celulas (a diagonal alternando).
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < columns; ++c) {
            const std::uint32_t a = index(c, r), b = index(c + 1, r), d = index(c + 1, r + 1), e = index(c, r + 1);
            if ((r + c) % 2 == 0) {
                net.faces.push_back({ a, b, d });
                net.faces.push_back({ a, d, e });
            } else {
                net.faces.push_back({ a, b, e });
                net.faces.push_back({ b, d, e });
            }
        }
    net.settings.ground = false;
    return net;
}

struct Ball {
    NetBody3D body;
    NetProxy3D proxy;
};

Ball ball(Vec3 position, Vec3 velocity) {
    Ball result;
    constexpr float Mass = 0.43f, Radius = 0.11f;
    result.body.position = position;
    result.body.velocity = velocity;
    result.body.inverseMass = 1.0f / Mass;
    result.body.inverseInertia = 1.0f / (2.0f / 3.0f * Mass * Radius * Radius);
    result.proxy.shape = NetProxyShape3D::Sphere;
    result.proxy.radius = Radius;
    result.proxy.friction = 0.45f;
    return result;
}

struct ShotResult {
    float deepest = 0.0f;
    float finalX = 0.0f;
    bool passedThrough = false;
    float microsecondsPerSubstep = 0.0f;
    std::vector<Vec3> finalNet;
};

// O tick como o backend faz: a bola integra a gravidade, depois a rede acopla.
void tick(NetSolver3D& net, Ball& b, bool withBall, double& contactMicroseconds, int& contactSubsteps) {
    std::span<NetBody3D> bodies(&b.body, withBall ? 1 : 0);
    std::span<const NetProxy3D> proxies(&b.proxy, withBall ? 1 : 0);
    net.beginTick(Tick, bodies, proxies);
    const float h = Tick / static_cast<float>(Substeps);
    for (std::uint32_t k = 0; k < Substeps; ++k) {
        if (withBall) {
            b.body.velocity = b.body.velocity + Vec3 { 0.0f, 0.0f, -9.81f } * h;
            b.body.position = b.body.position + b.body.velocity * h;
        }
        const auto start = std::chrono::steady_clock::now();
        net.substep(h, bodies, proxies);
        if (withBall && b.body.contacts > 0) {
            contactMicroseconds += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
            ++contactSubsteps;
        }
    }
    net.endTick(Tick, bodies);
}

// A rede vertical no plano x = -1 (de y -3,7 a 3,7, z 0 a 2,5); a bola sai
// de x = 4 rumo ao ponto (y, z) a 35 m/s.
ShotResult shoot(float y, float z, float speed) {
    NetSolver3D net(panel({ -1.0f, -3.7f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 7.4f, 2.5f));
    net.settle(4.0f);
    Ball b = ball({ 4.0f, y, z }, { -speed, 0.0f, 0.0f });
    // (Sem gravidade a mira e reta: compensa a queda ate a rede.)
    const float flight = 5.0f / speed;
    b.body.velocity.z = 0.5f * 9.81f * flight;
    ShotResult result;
    double micro = 0.0;
    int substeps = 0;
    float deepest = 1e9f;
    for (int t = 0; t < 360; ++t) {
        b.body.contacts = 0;
        tick(net, b, true, micro, substeps);
        deepest = std::min(deepest, b.body.position.x);
        float netMinimum = 1e9f;
        for (const Vec3& p : net.positions()) netMinimum = std::min(netMinimum, p.x);
        // Atravessou: o centro da bola atras de todos os nos.
        if (b.body.position.x < netMinimum - 0.02f) result.passedThrough = true;
    }
    result.deepest = -1.0f - deepest;
    result.finalX = b.body.position.x;
    result.microsecondsPerSubstep = substeps > 0 ? static_cast<float>(micro / substeps) : 0.0f;
    result.finalNet.assign(net.positions().begin(), net.positions().end());
    return result;
}

void testRest() {
    NetSolver3D net(panel({ -1.0f, -3.7f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 7.4f, 2.5f));
    const float used = net.settle(6.0f);
    require(net.sleeping(), "a rede pendurada dorme (" + std::to_string(used) + " s)");
    double micro = 0.0;
    int substeps = 0;
    Ball none = ball({}, {});
    tick(net, none, false, micro, substeps);
    // Os fios parados quase no comprimento (a folga absorve o peso).
    float strain = 0.0f;
    for (const NetSegment3D& s : net.segments())
        strain = std::max(strain, (net.positions()[s.b] - net.positions()[s.a]).length() / s.restLength - 1.0f);
    std::printf("  repouso: dormiu em %.2f s, fio mais esticado %.2f%%, massa %.2f kg, %zu nos, %zu fios\n", used,
        strain * 100.0f, net.totalMass(), net.positions().size(), net.segments().size());
    require(strain < 0.03f, "parada, nenhum fio estica mais de 3%");
    require(net.totalMass() > 1.0f && net.totalMass() < 4.0f, "a massa do pano (fio de 7 g/m)");
}

void testShots() {
    struct Aim {
        const char* name;
        float y;
        float z;
    };
    const Aim aims[] = { { "meio", 0.0f, 1.25f }, { "canto alto", 3.3f, 2.2f }, { "baixo perto da trave", -3.4f, 0.3f },
        { "alto no meio", 0.4f, 2.3f } };
    for (const Aim& aim : aims) {
        const ShotResult shot = shoot(aim.y, aim.z, 35.0f);
        std::printf("  chute 35 m/s no %s: estufou %.2f m, bola termina em x = %.2f, %.1f us por subpasso com contato\n",
            aim.name, shot.deepest, shot.finalX, shot.microsecondsPerSubstep);
        require(!shot.passedThrough, std::string("a bola nao atravessa a rede (") + aim.name + ")");
        require(shot.deepest > 0.05f && shot.deepest < 1.6f, std::string("estufa numa profundidade plausivel (") + aim.name + ")");
        require(shot.finalX > -1.0f - 1.6f, std::string("a bola fica (") + aim.name + ")");
    }
    const ShotResult center = shoot(0.0f, 1.25f, 35.0f);
    require(center.deepest > 0.25f, "no meio, a 35 m/s, a rede estufa mais de 25 cm");
    // O mesmo chute, o mesmo resultado.
    const ShotResult again = shoot(0.0f, 1.25f, 35.0f);
    bool same = again.finalNet.size() == center.finalNet.size();
    for (std::size_t i = 0; same && i < center.finalNet.size(); ++i)
        same = center.finalNet[i].x == again.finalNet[i].x && center.finalNet[i].y == again.finalNet[i].y
            && center.finalNet[i].z == again.finalNet[i].z;
    require(same, "deterministico");
}

void testResting() {
    // Uma rede deitada (rede de pesca esticada a 1 m do chao): a bola cai
    // nela, para, a rede dorme e continua segurando a bola.
    NetSolver3D net(panel({ -1.2f, -1.2f, 1.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, 2.4f, 2.4f));
    net.settle(4.0f);
    Ball b = ball({ 0.05f, 0.03f, 2.0f }, {});
    double micro = 0.0;
    int substeps = 0;
    bool slept = false;
    float lowest = 1e9f;
    for (int t = 0; t < 120 * 8; ++t) {
        b.body.contacts = 0;
        tick(net, b, true, micro, substeps);
        if (net.sleeping()) slept = true;
        if (slept) lowest = std::min(lowest, b.body.position.z);
    }
    float netLowest = 1e9f;
    for (const Vec3& p : net.positions()) netLowest = std::min(netLowest, p.z);
    std::printf("  bola apoiada: rede dormiu %s, bola em z = %.3f (fundo da rede %.3f), a mais baixa dormindo %.3f\n",
        slept ? "sim" : "nao", b.body.position.z, netLowest, lowest);
    require(slept, "a rede dorme com a bola apoiada");
    require(b.body.position.z > netLowest, "a bola continua sobre a rede");
    require(lowest > netLowest - 0.005f, "dormindo, a rede segura a bola (nao cai por ela)");
    require(std::abs(b.body.velocity.z) < 0.05f, "a bola parada");
}

// Um membro (capsula) contra o pano em pe: a rede nao e atravessada. O
// cinematico (o boneco guiado, que nao cede) empurra a rede a frente dele; o
// dinamico (uma perna de 6 kg atirada) e parado por ela.
void testCapsules() {
    // (O guiado so encosta, ate 25 cm dentro da folga: no jogo, quando a rede
    // esticada o empurra, o atleta vira fisico e ela o para como a perna.)
    struct Case {
        const char* name;
        float radius;
        float speed;
        float inverseMass;
        bool guided;
        float stop;
    };
    const Case cases[] = { { "braco fino de 3 kg a 3 m/s, guiado", 0.035f, 3.0f, 1.0f / 3.0f, true, -1.25f },
        { "braco fino de 3 kg a 12 m/s, guiado", 0.035f, 12.0f, 1.0f / 3.0f, true, -1.25f },
        { "cinematico a 6 m/s", 0.05f, 6.0f, 0.0f, false, -1.25f }, { "perna de 6 kg a 15 m/s", 0.06f, 15.0f, 1.0f / 6.0f, false, -9.0f },
        { "perna de 6 kg a 30 m/s", 0.06f, 30.0f, 1.0f / 6.0f, false, -9.0f } };
    for (const Case& test : cases) {
        NetSolver3D net(panel({ -1.0f, -3.7f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 7.4f, 2.5f));
        net.settle(4.0f);
        NetBody3D body;
        // De pe (o eixo em z), no meio de uma celula, vindo de x = 1.
        body.position = { 1.0f, 0.06f, 1.26f };
        body.velocity = { -test.speed, 0.0f, 0.0f };
        body.inverseMass = test.inverseMass;
        body.guided = test.guided;
        body.advance = true;
        NetProxy3D proxy;
        proxy.shape = NetProxyShape3D::Capsule;
        proxy.radius = test.radius;
        proxy.halfHeight = 0.2f;
        // O guiado e o cinematico param onde o caso diz; o dinamico, quando
        // a rede o para. O guiado relata o impulso (o que vai ao boneco).
        float deepest = 0.0f;
        bool through = false;
        float reported = 0.0f;
        for (int t = 0; t < 240; ++t) {
            if (body.position.x < test.stop) body.velocity = {};
            std::span<NetBody3D> bodies(&body, 1);
            std::span<const NetProxy3D> proxies(&proxy, 1);
            net.beginTick(Tick, bodies, proxies);
            for (std::uint32_t k = 0; k < Substeps; ++k) net.substep(Tick / static_cast<float>(Substeps), bodies, proxies);
            net.endTick(Tick, bodies);
            reported = std::max(reported, body.impulse.length());
            body.impulse = {};
            deepest = std::min(deepest, body.position.x);
            // Atravessou: algum no do pano na faixa do membro ficou do lado de
            // ca dele (entre o membro e de onde ele veio).
            for (const Vec3& q : net.positions())
                if (std::abs(q.y - body.position.y) < 0.03f && std::abs(q.z - body.position.z) < 0.15f && q.x > body.position.x + test.radius + 0.01f)
                    through = true;
        }
        std::printf("  capsula, %s: foi ate x = %.2f, %s, impulso no tick ate %.2f N s\n", test.name, deepest,
            through ? "ATRAVESSOU" : "a rede segurou", reported);
        require(!through, std::string("a capsula nao atravessa a rede (") + test.name + ")");
        if (!test.guided && test.inverseMass > 0.0f) require(deepest > -1.0f - 1.6f, std::string("a rede para o membro (") + test.name + ")");
        if (test.guided) require(reported > 0.0f, std::string("o guiado relata o empurrao da rede (") + test.name + ")");
    }
}

// O guiado empurrando fundo (o boneco que ainda nao reagiu): o impulso
// relatado e o de parar o membro, para o atleta virar fisico.
void testGuidedPush() {
    NetSolver3D net(panel({ -1.0f, -3.7f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 7.4f, 2.5f));
    net.settle(4.0f);
    NetBody3D body;
    body.position = { 1.0f, 0.06f, 1.26f };
    body.velocity = { -3.0f, 0.0f, 0.0f };
    body.inverseMass = 1.0f / 3.0f;
    body.guided = true;
    body.advance = true;
    NetProxy3D proxy;
    proxy.shape = NetProxyShape3D::Capsule;
    proxy.radius = 0.035f;
    proxy.halfHeight = 0.2f;
    float slack = 0.0f, taut = 0.0f;
    for (int t = 0; t < 120 && body.position.x > -1.5f; ++t) {
        std::span<NetBody3D> bodies(&body, 1);
        std::span<const NetProxy3D> proxies(&proxy, 1);
        net.beginTick(Tick, bodies, proxies);
        for (std::uint32_t k = 0; k < Substeps; ++k) net.substep(Tick / static_cast<float>(Substeps), bodies, proxies);
        net.endTick(Tick, bodies);
        const float impulse = body.impulse.length();
        body.impulse = {};
        if (body.position.x > -1.1f) slack = std::max(slack, impulse);
        else if (body.position.x < -1.35f) taut = std::max(taut, impulse);
    }
    std::printf("  guiado de 3 kg a 3 m/s: impulso por tick na rede frouxa ate %.2f N s, esticada ate %.2f N s (parar o braco: 9 N s)\n", slack, taut);
    require(taut > 4.0f * std::max(slack, 0.01f), "a rede esticada empurra o guiado bem mais que a frouxa");
    require(taut > 3.0f && taut < 40.0f, "o impulso da rede esticada e o de parar o braco");
}

} // namespace

// A recuperacao de uma penetracao funda nao apaga o caminho que os tubos conferem (a revisao de
// 26/10): a bola parada encostada fundo no pano logo na frente de um tubo empurra os nos so pela
// recuperacao (sem aproximacao, nada vira velocidade); a referencia da velocidade anda junto, mas o
// tubo confere o caminho geometrico do subpasso e o no volta para o lado de onde veio. Com o caminho
// encurtado, o no terminava do outro lado do tubo, preso la.
void testRecoveryKeepsTubeSide() {
    // O pano no plano x = 0 (y e z de 0 a 0,48) e o tubo atras dele, ao longo de y, a 4 cm.
    NetDescription3D description = panel({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 0.48f, 0.48f);
    description.settings.twineRadius = 0.003f;
    const float tubeX = -0.04f, tubeRadius = 0.025f;
    description.colliders.push_back({ { tubeX, -0.1f, 0.24f }, { tubeX, 0.58f, 0.24f }, tubeRadius });
    NetSolver3D net(description);
    // A bola parada, o centro 5 cm na frente do pano, na frente do tubo.
    Ball b = ball({ 0.05f, 0.24f, 0.24f }, {});
    std::span<NetBody3D> bodies(&b.body, 1);
    std::span<const NetProxy3D> proxies(&b.proxy, 1);
    net.beginTick(Tick, bodies, proxies);
    net.substep(Tick / static_cast<float>(Substeps), bodies, proxies);
    net.endTick(Tick, bodies);
    float worst = 1e9f;
    for (std::size_t i = 0; i < net.positions().size(); ++i) {
        const Vec3 p = net.positions()[i];
        if (std::abs(p.z - 0.24f) > 0.07f) continue;
        worst = std::min(worst, p.x);
    }
    std::printf("  recuperacao junto ao tubo: o no mais fundo em x = %.3f (o eixo do tubo em %.3f, o lado de tras dele em %.3f)\n", worst,
        tubeX, tubeX - tubeRadius);
    require(worst > tubeX, "a recuperacao funda nao passa o no para o outro lado do tubo");
}

// O fio isolado (26/10, a revisao do Codex): repetir a passada nao endurece o fio. Esticado 10% entre
// dois nos soltos, sem gravidade, ar ou amortecimento, um subpasso deixa o alongamento do XPBD (a
// complacencia do fio contra as massas), com 0 ou 4 passadas a mais do cabo; esticado 30%, o que passa
// dos 15% sai inteiro e o resto pelo XPBD, igual com 1 ou 5 passadas. (Sem o multiplicador acumulado,
// cada passada encolhia o que sobrou na mesma fracao: o fio ficava mais rigido.)
NetDescription3D strands(std::vector<Vec3> positions, std::vector<std::uint8_t> pinned, std::uint32_t passes) {
    NetDescription3D description;
    description.positions = std::move(positions);
    description.masses.assign(description.positions.size(), 0.01f);
    description.pinned = std::move(pinned);
    description.settings.gravity = {};
    description.settings.dragCoefficient = 0.0f;
    description.settings.stretchDampingTime = 0.0f;
    description.settings.ropeIterations = passes;
    return description;
}

void testRepeatedPassesKeepCompliance() {
    const auto stretch = [](float length, std::uint32_t passes) {
        NetDescription3D description = strands({ { 0.0f, 0.0f, 1.0f }, { length, 0.0f, 1.0f } }, { 0, 0 }, passes);
        description.segments.push_back({ 0, 1, 0.1f, 10000.0f, true });
        NetSolver3D net(description);
        net.beginTick(Tick, {}, {});
        net.substep(Tick / static_cast<float>(Substeps), {}, {});
        return std::sqrt((net.positions()[1] - net.positions()[0]).lengthSquared()) - 0.1f;
    };
    const float h = Tick / static_cast<float>(Substeps);
    const float soft = 0.1f / 10000.0f / (h * h);
    const float fraction = soft / (200.0f + soft);
    const float once = stretch(0.11f, 0), repeated = stretch(0.11f, 4);
    std::printf("  fio isolado: alongamento %.6f m com 1 passada, %.6f com 5 (o XPBD: %.6f)\n", once, repeated, 0.01f * fraction);
    require(std::abs(once - 0.01f * fraction) < 1e-6f && std::abs(repeated - once) < 1e-7f, "repetir a passada nao endurece o fio");
    const float rigidOnce = stretch(0.13f, 0), rigidRepeated = stretch(0.13f, 4);
    std::printf("  fio isolado alem do limite (30%%): %.6f m com 1 passada, %.6f com 5 (o XPBD: %.6f)\n", rigidOnce, rigidRepeated,
        0.015f * fraction);
    require(std::abs(rigidOnce - 0.015f * fraction) < 1e-6f && std::abs(rigidRepeated - rigidOnce) < 1e-7f,
        "alem do limite, o excesso sai inteiro e o resto nao endurece com as passadas");
}

// O fio que afrouxa no mesmo subpasso devolve o que puxou: A-B esticado puxa os dois; depois o fio B-D,
// preso em D, leva B para longe de A e o deixa frouxo. Na passada seguinte do cabo, A volta para onde
// estava (o fio frouxo nao fica com forca). Sem a devolucao, A ficava 4,2 mm puxado por um fio frouxo.
void testSlackStrandReleases() {
    NetDescription3D description = strands({ { 0.0f, 0.0f, 1.0f }, { 0.11f, 0.0f, 1.0f }, { -0.2f, 0.0f, 1.0f } }, { 0, 0, 1 }, 1);
    description.segments.push_back({ 0, 1, 0.1f, 10000.0f, true });
    description.segments.push_back({ 2, 1, 0.25f, 10000.0f, true });
    NetSolver3D net(description);
    net.beginTick(Tick, {}, {});
    net.substep(Tick / static_cast<float>(Substeps), {}, {});
    const Vec3 a = net.positions()[0], b = net.positions()[1];
    std::printf("  fio que afrouxa: A em x = %.7f (comecou em 0), A-B com %.4f m (solto 0,1)\n", a.x, b.x - a.x);
    require(b.x - a.x < 0.1f && std::abs(a.x) < 1e-6f, "o fio que afrouxou no subpasso devolve o que puxou");
}

int main() {
    try {
        testRepeatedPassesKeepCompliance();
        testSlackStrandReleases();
        testRecoveryKeepsTubeSide();
        testRest();
        testShots();
        testResting();
        testCapsules();
        testGuidedPush();
    } catch (const std::exception& exception) {
        std::cerr << "MatterNetSolver test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterNetSolver tests passed (" << checks << " checagens)\n";
    return 0;
}
