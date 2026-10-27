// A rede da trave sob medida: o custo por tick de cada cenario pela cena de
// verdade (o Jolt, a bola inflada, a rede nos subpassos dela) e o desenho dela
// na tela Objetos (o renderizador de inspecao, com os fios). O tick e o de 120
// Hz do jogo (8,33 ms de orcamento).
//
// uso: MatterNetBenchmark [--grade-forte] [parametro valor]...  (os da receita
// da trave: "malha_fisica 1" compara com a fisica na malha de verdade;
// --grade-forte: so a grade de chutes fortes, procurando a bola que escapa, e
// onde os gols terminam). Cada linha termina com a impressao digital da cena
// ate ali ([...]: os nos e os corpos a cada tick): a otimizacao que nao muda
// nada a deixa igual.
// A linha "a rede parada" da a flecha do cabo do alto, quanto o teto desceu ao assentar e a dobra do pano
// no cabo (no meio do vao).
// O lance tick a tick: XCHUTE="velocidade distancia mira_lado mira_altura
// angulo efeito_lado efeito_cima" (a velocidade e o giro de verdade da bola, a
// rede acordada ou dormindo), XTICKS (quantos; 200) e XNOS="a b c" (onde esses
// nos da fisica terminam, contra o repouso).

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"
#include "Engine/Objects/ObjectNetLab3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <span>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point begin) {
    return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}

// A impressao digital de tudo o que a cena fez (os nos da rede e os corpos a
// cada tick): uma otimizacao que nao muda nada deixa ela igual.
std::uint64_t g_digest = 1469598103934665603ull;

struct Series {
    std::vector<float> net;
    std::vector<float> scene;
};

void report(const char* name, const Series& series, const char* note = "") {
    const auto line = [](std::vector<float> values) {
        if (values.empty()) return std::string("-");
        std::sort(values.begin(), values.end());
        double sum = 0.0;
        for (const float v : values) sum += v;
        char text[96];
        std::snprintf(text, sizeof(text), "%6.3f %6.3f %6.3f %6.3f", sum / static_cast<double>(values.size()),
            values[values.size() / 2], values[std::min(values.size() - 1, values.size() * 95 / 100)], values.back());
        return std::string(text);
    };
    std::printf("%-34s %5zu | %s | %s %s [%016llx]\n", name, series.net.size(), line(series.net).c_str(), line(series.scene).c_str(), note,
        static_cast<unsigned long long>(g_digest));
}

void digest(const ObjectNetLab3D& lab) {
    const auto add = [](Vec3 p) {
        for (const float f : { p.x, p.y, p.z }) {
            std::uint32_t bits = 0;
            std::memcpy(&bits, &f, sizeof(bits));
            g_digest = (g_digest ^ bits) * 1099511628211ull;
        }
    };
    for (const Vec3& p : lab.netPositions()) add(p);
    for (const NetLabBody3D& body : lab.bodies()) add(body.position);
}

// Um tick medido: a rede (o que ela relata) e a cena inteira (Jolt, bolas, rede).
void tick(ObjectNetLab3D& lab, Series& series) {
    const auto begin = Clock::now();
    lab.step();
    series.scene.push_back(static_cast<float>(millisecondsSince(begin)));
    series.net.push_back(lab.netStatus().stepMilliseconds);
    digest(lab);
}

// Ticks ate a condicao (ou o limite), medindo so os que a rede estava ativa
// (acordada ou segurando algo).
Series runActive(ObjectNetLab3D& lab, int limit, const std::function<bool()>& done) {
    Series series;
    for (int t = 0; t < limit && !done(); ++t) {
        Series one;
        tick(lab, one);
        if (!lab.netStatus().sleeping || lab.netStatus().contacts > 0) {
            series.net.push_back(one.net.front());
            series.scene.push_back(one.scene.front());
        }
    }
    return series;
}

Modeling::ProceduralObject load(const char* name, std::span<char*> overrides = {}) {
    std::string error;
    auto recipe = Modeling::readObjectRecipe(std::string(MATTERENGINE_ASSETS_DIR) + "/objects/" + name, &error);
    for (std::size_t k = 0; recipe && k + 1 < overrides.size(); k += 2) recipe->numbers[overrides[k]] = std::strtof(overrides[k + 1], nullptr);
    const auto object = recipe ? Modeling::generateObject(*recipe, &error) : std::nullopt;
    if (!object) {
        std::fprintf(stderr, "%s: %s\n", name, error.c_str());
        std::exit(1);
    }
    return *object;
}

// A grade forte: chutes de 30 a 45 m/s de 6 a 16 m, pela largura toda, do
// chao ao canto do teto, de angulo (dos dois lados) e com efeito. Cada bola
// que atravessa a rede (alem da estufada maxima atras, pelo lado ou pelo
// teto) aparece; as que voltam pela boca so contam.
// A bola atras da rede, pelo pano deformado (independente das caixas fixas e do solver): o segmento
// do centro dela ate um ponto da boca do gol corta as celulas da fisica um numero impar de vezes.
// Cada cruzamento conta uma vez: os nos vao para o plano perpendicular ao segmento (uma vez cada, as
// celulas vizinhas veem os mesmos numeros), e o segmento que passa exatamente por uma aresta ou um
// vertice comum e desempatado como se estivesse deslocado (eps, eps^2) nesse plano, o que o poe dentro
// de exatamente uma das celulas em volta. (A conta de antes aceitava a borda nas duas celulas: na
// diagonal comum contava 2, par, e a bola atras do pano passava por dentro.)
bool behindNet(std::span<const std::array<std::uint32_t, 3>> faces, std::span<const Vec3> knots, Vec3 ball, Vec3 mouth) {
    const Vec3 d = mouth - ball;
    const float length = std::sqrt(d.lengthSquared());
    if (!(length > 1e-6f)) return false;
    const Vec3 w = d * (1.0f / length);
    const Vec3 helper = std::abs(w.x) < 0.9f ? Vec3 { 1.0f, 0.0f, 0.0f } : Vec3 { 0.0f, 1.0f, 0.0f };
    Vec3 u = cross(helper, w);
    u = u * (1.0f / std::sqrt(u.lengthSquared()));
    const Vec3 v = cross(w, u);
    std::vector<std::array<float, 3>> at(knots.size());
    for (std::size_t i = 0; i < knots.size(); ++i) {
        const Vec3 r = knots[i] - ball;
        at[i] = { dot(r, u), dot(r, v), dot(r, w) };
    }
    // O lado do segmento (a origem do plano) em relacao a aresta a -> b, exato (os produtos de float em
    // double); no zero, o do ponto deslocado. Visto da celula vizinha (b -> a), o oposto.
    const auto side = [&](std::uint32_t a, std::uint32_t b, double& e) {
        e = static_cast<double>(at[a][0]) * at[b][1] - static_cast<double>(at[a][1]) * at[b][0];
        if (e != 0.0) return e > 0.0 ? 1 : -1;
        const float dy = at[b][1] - at[a][1], dx = at[b][0] - at[a][0];
        if (dy != 0.0f) return dy < 0.0f ? 1 : -1;
        return dx > 0.0f ? 1 : dx < 0.0f ? -1 : 0;
    };
    int crossings = 0;
    for (const std::array<std::uint32_t, 3>& f : faces) {
        double ab = 0.0, bc = 0.0, ca = 0.0;
        const int s1 = side(f[0], f[1], ab), s2 = side(f[1], f[2], bc), s3 = side(f[2], f[0], ca);
        if (s1 == 0 || s1 != s2 || s2 != s3) continue;
        const double area = ab + bc + ca;
        if (area == 0.0) continue;
        const double depth = (bc * at[f[0]][2] + ca * at[f[1]][2] + ab * at[f[2]][2]) / area;
        if (depth > 0.0 && depth < length) ++crossings;
    }
    return (crossings & 1) != 0;
}

// A paridade conferida antes da grade: o quadrado em dois triangulos e o leque de quatro em volta de
// um vertice, no plano x = 0, cruzados de x = 1 a x = -1 pelo meio de um triangulo, pela diagonal
// comum, pelo vertice comum, por uma aresta do leque e por pontos a 1e-6 deles: um cruzamento cada;
// fora do pano, nenhum.
bool parityHolds() {
    const std::vector<Vec3> square { { 0.0f, -1.0f, -1.0f }, { 0.0f, 1.0f, -1.0f }, { 0.0f, 1.0f, 1.0f }, { 0.0f, -1.0f, 1.0f } };
    const std::vector<std::array<std::uint32_t, 3>> halves { { 0, 1, 2 }, { 0, 2, 3 } };
    const std::vector<Vec3> fan { {}, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, -1.0f } };
    const std::vector<std::array<std::uint32_t, 3>> around { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 1 } };
    const auto crosses = [](const std::vector<std::array<std::uint32_t, 3>>& faces, const std::vector<Vec3>& knots, float y, float z) {
        return behindNet(faces, knots, { 1.0f, y, z }, { -1.0f, y, z });
    };
    for (const float e : { 0.0f, 1e-6f, -1e-6f }) {
        if (!crosses(halves, square, 0.5f + e, -0.2f) || !crosses(halves, square, 0.3f + e, 0.3f) || !crosses(halves, square, 0.3f, 0.3f + e))
            return false;
        if (!crosses(around, fan, e, 0.0f) || !crosses(around, fan, 0.0f, e) || !crosses(around, fan, e, e) || !crosses(around, fan, 0.5f, e))
            return false;
    }
    return !crosses(halves, square, 1.5f, 0.0f) && !crosses(around, fan, 0.6f, 0.6f);
}

// Um numero que nao e numero na bola (a posicao e as velocidades) ou na rede: falha, dentro ou fora do gol.
bool brokenScene(ObjectNetLab3D& lab) {
    const auto finite = [](Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); };
    for (const InflatedBallHandle3D ball : lab.balls()) {
        const InflatedBallState3D state = lab.scene()->inflatedBallState(ball);
        if (!finite(state.position) || !finite(state.linearVelocity) || !finite(state.angularVelocity)) return true;
    }
    for (const Vec3& p : lab.netPositions())
        if (!finite(p)) return true;
    return false;
}

int hardGrid(ObjectNetLab3D& lab, const Modeling::ProceduralObject& goal) {
    if (!parityHolds()) {
        std::printf("grade forte: a paridade do pano falhou na conferencia\n");
        return 1;
    }
    const float backLimit = -goal.size.x - 0.8f, sideLimit = 0.5f * goal.size.y + 0.3f, roofLimit = goal.size.z + 0.3f;
    // Um ponto da boca, um pouco para dentro da linha, no meio do vao.
    const Vec3 mouth { lab.goalLineX() - 0.3f, 0.0f, 0.5f * goal.size.z };
    int shots = 0, escaped = 0, goals = 0, outAgain = 0;
    // Onde os gols terminam (x alem da linha: positivo e fora do gol).
    std::vector<float> ends;
    const float speeds[] { 30.0f, 35.0f, 40.0f, 45.0f };
    const float distances[] { 4.0f, 6.0f, 11.0f, 16.0f };
    const float acrosses[] { -0.95f, -0.6f, -0.2f, 0.2f, 0.6f, 0.95f };
    const float heights[] { 0.05f, 0.25f, 0.5f, 0.8f, 0.97f };
    for (int si = 0; si < 4; ++si)
        for (int di = 0; di < 4; ++di)
            for (int ai = 0; ai < 6; ++ai)
                for (int hi = 0; hi < 5; ++hi) {
                    lab.reset();
                    NetShotSettings3D settings;
                    settings.speed = speeds[si];
                    settings.distance = distances[di];
                    settings.aimAcross = acrosses[ai];
                    settings.aimHeight = heights[hi];
                    // (Metade cruzando: de um lado para o canto do outro.)
                    settings.angleDegrees = ((hi + di) % 2 == 0 ? 1.0f : -1.0f) * acrosses[ai] * 30.0f;
                    settings.sideSpin = ((ai + si) % 3 - 1) * 10.0f;
                    settings.topSpin = (hi + si) % 2 == 0 ? 0.0f : 15.0f;
                    lab.shoot(settings);
                    bool through = false, inside = false;
                    Vec3 worst;
                    for (int t = 0; t < 480; ++t) {
                        lab.step();
                        digest(lab);
                        const Vec3 p = lab.bodies().back().position;
                        const bool out = p.x < backLimit || (p.x < -0.2f && (std::abs(p.y) > sideLimit || p.z > roofLimit));
                        // (So a que esteve dentro do gol: a do poste que sai rolando pela linha nao.)
                        inside = inside || (p.x < -0.3f && std::abs(p.y) < 0.5f * goal.size.y && p.z < goal.size.z);
                        // (A que voltou pela boca e saiu na frente do gol nao esta mais dentro.)
                        if (p.x > lab.goalLineX() + 0.3f) inside = false;
                        // (Atras do pano deformado, dentro do gol antes; um numero que nao e numero, sempre.)
                        const bool failed = brokenScene(lab) || (inside && (out || behindNet(goal.net->faces, lab.netPositions(), p, mouth)));
                        if (failed && !through) worst = p;
                        through = through || failed;
                    }
                    const Vec3 end = lab.bodies().back().position;
                    ++shots;
                    goals += lab.metrics().goal ? 1 : 0;
                    if (lab.metrics().goal && end.x > 0.5f) ++outAgain;
                    if (lab.metrics().goal) ends.push_back(end.x);
                    if (through) {
                        ++escaped;
                        std::printf("  ESCAPOU: %.0f m/s de %.0f m, mira (%.2f, %.2f), angulo %.0f, efeito %.0f/%.0f: passou em (%.2f %.2f %.2f), "
                                    "termina em (%.2f %.2f %.2f)\n",
                            settings.speed, settings.distance, settings.aimAcross, settings.aimHeight, settings.angleDegrees, settings.sideSpin,
                            settings.topSpin, worst.x, worst.y, worst.z, end.x, end.y, end.z);
                    }
                }
    // De muito perto (2 e 3 m) e bem de lado (45 e 60 graus, dos dois lados),
    // nas duas redes do lado: o chute que ele descreveu (40 m/s, na diagonal).
    for (const float speed : { 35.0f, 40.0f, 45.0f })
        for (const float distance : { 2.0f, 3.0f })
            for (const float angle : { -60.0f, -45.0f, 45.0f, 60.0f })
                for (const float across : { -0.95f, -0.5f, 0.5f, 0.95f })
                    for (const float height : { 0.2f, 0.5f, 0.8f }) {
                        lab.reset();
                        NetShotSettings3D settings;
                        settings.speed = speed;
                        settings.distance = distance;
                        settings.angleDegrees = angle;
                        settings.aimAcross = across;
                        settings.aimHeight = height;
                        settings.sideSpin = angle > 0.0f ? 8.0f : -8.0f;
                        lab.shoot(settings);
                        bool through = false, inside = false;
                        Vec3 worst;
                        for (int t = 0; t < 480; ++t) {
                            lab.step();
                            digest(lab);
                            const Vec3 p = lab.bodies().back().position;
                            const bool out = p.x < backLimit || (p.x < -0.2f && (std::abs(p.y) > sideLimit || p.z > roofLimit));
                            inside = inside || (p.x < -0.3f && std::abs(p.y) < 0.5f * goal.size.y && p.z < goal.size.z);
                            // (A que voltou pela boca e saiu na frente do gol nao esta mais dentro.)
                            if (p.x > lab.goalLineX() + 0.3f) inside = false;
                            const bool failed = brokenScene(lab) || (inside && (out || behindNet(goal.net->faces, lab.netPositions(), p, mouth)));
                            if (failed && !through) worst = p;
                            through = through || failed;
                        }
                        const Vec3 end = lab.bodies().back().position;
                        ++shots;
                        goals += lab.metrics().goal ? 1 : 0;
                        if (lab.metrics().goal && end.x > 0.5f) ++outAgain;
                        if (lab.metrics().goal) ends.push_back(end.x);
                        if (through) {
                            ++escaped;
                            std::printf("  ESCAPOU (de perto): %.0f m/s de %.0f m, angulo %.0f, mira (%.2f, %.2f): passou em (%.2f %.2f %.2f), termina em (%.2f %.2f %.2f)\n",
                                speed, distance, angle, across, height, worst.x, worst.y, worst.z, end.x, end.y, end.z);
                        }
                    }
    // A segunda bola na rede ja estufada pela primeira (6 ticks depois, no mesmo lugar e um pouco ao
    // lado): as duas conferidas pelas caixas e pelo pano deformado.
    for (const float speed : { 30.0f, 45.0f })
        for (const float distance : { 4.0f, 11.0f })
            for (const float across : { -0.6f, 0.0f, 0.95f })
                for (const float height : { 0.25f, 0.8f }) {
                    lab.reset();
                    NetShotSettings3D first;
                    first.speed = speed;
                    first.distance = distance;
                    first.aimAcross = across;
                    first.aimHeight = height;
                    lab.shoot(first);
                    for (int t = 0; t < 6; ++t) lab.step();
                    NetShotSettings3D second = first;
                    second.aimAcross = std::clamp(across + 0.1f, -0.95f, 0.95f);
                    lab.shoot(second);
                    const std::size_t balls = lab.bodies().size();
                    std::vector<char> inside(balls, 0), through(balls, 0);
                    for (int t = 0; t < 480; ++t) {
                        lab.step();
                        digest(lab);
                        const bool broken = brokenScene(lab);
                        for (std::size_t b = 0; b < balls && b < lab.bodies().size(); ++b) {
                            const Vec3 p = lab.bodies()[b].position;
                            const bool out = p.x < backLimit || (p.x < -0.2f && (std::abs(p.y) > sideLimit || p.z > roofLimit));
                            inside[b] = inside[b] || (p.x < -0.3f && std::abs(p.y) < 0.5f * goal.size.y && p.z < goal.size.z);
                            if (p.x > lab.goalLineX() + 0.3f) inside[b] = 0;
                            if (!through[b] && (broken || (inside[b] && (out || behindNet(goal.net->faces, lab.netPositions(), p, mouth))))) {
                                through[b] = 1;
                                ++escaped;
                                std::printf("  ESCAPOU (segunda bola): %.0f m/s de %.0f m, mira (%.2f, %.2f), bola %zu em (%.2f %.2f %.2f)\n", speed, distance,
                                    across, height, b, p.x, p.y, p.z);
                            }
                        }
                    }
                    shots += 2;
                }
    std::sort(ends.begin(), ends.end());
    const auto at = [&](float q) { return ends.empty() ? 0.0f : ends[static_cast<std::size_t>(q * static_cast<float>(ends.size() - 1))]; };
    const auto beyond = [&](float x) { return static_cast<long>(ends.end() - std::upper_bound(ends.begin(), ends.end(), x)); };
    std::printf("onde os gols terminam (m alem da linha): p50 %.2f p75 %.2f p90 %.2f max %.2f; mais de 2 m: %ld, de 4 m: %ld, de 8 m: %ld\n", at(0.5f),
        at(0.75f), at(0.9f), at(1.0f), beyond(2.0f), beyond(4.0f), beyond(8.0f));
    std::printf("grade forte: %d chutes, %d gols, %d atravessaram a rede, %d gols voltaram pela boca [%016llx]\n", shots, goals, escaped, outAgain,
        static_cast<unsigned long long>(g_digest));
    return escaped == 0 ? 0 : 1;
}

} // namespace

int main(int argc, char** argv) {
    PhysicsEngine3D engine;
    const MaterialLibrary materials;
    const bool hard = argc > 1 && std::string(argv[1]) == "--grade-forte";
    if (hard) {
        --argc;
        ++argv;
    }
    const Modeling::ProceduralObject goal = load("trave.json", std::span<char*>(argv + 1, static_cast<std::size_t>(argc - 1)));
    if (hard) {
        ObjectNetLab3D lab(engine, materials);
        lab.start(goal);
        return hardGrid(lab, goal);
    }
    // (XCHUTE="velocidade distancia mira_lado mira_altura angulo efeito_lado
    // efeito_cima": o lance tick a tick.)
    if (const char* drop = std::getenv("XQUEDA")) {
        // (XQUEDA="dx y z": a bola largada parada, dx metros atras da linha do gol; o lance tick a tick.)
        ObjectNetLab3D lab(engine, materials);
        lab.start(goal);
        Vec3 at;
        std::sscanf(drop, "%f %f %f", &at.x, &at.y, &at.z);
        InflatedBallDefinition3D ball;
        ball.position = { lab.goalLineX() - at.x, at.y, at.z };
        const InflatedBallHandle3D handle = lab.scene()->createInflatedBall(ball);
        for (int t = 0; t < 360; ++t) {
            lab.step();
            const InflatedBallState3D b = lab.scene()->inflatedBallState(handle);
            const NetStatus3D st = lab.netStatus();
            const Vec3 mouth { lab.goalLineX() - 0.3f, 0.0f, 0.5f * goal.size.z };
            std::printf("t=%3d bola (%.3f %.3f %.3f) v=(%+.2f %+.2f %+.2f) | rede: %s subpassos %u contatos %u | atras do pano %d\n", t, b.position.x - lab.goalLineX(), b.position.y, b.position.z, b.linearVelocity.x, b.linearVelocity.y, b.linearVelocity.z,
                st.sleeping ? "dormindo" : "acordada", st.substeps, st.contacts, behindNet(goal.net->faces, lab.netPositions(), b.position, mouth) ? 1 : 0);
        }
        return 0;
    }
    if (const char* trace = std::getenv("XCHUTE")) {
        ObjectNetLab3D lab(engine, materials);
        lab.start(goal);
        NetShotSettings3D s;
        std::sscanf(trace, "%f %f %f %f %f %f %f", &s.speed, &s.distance, &s.aimAcross, &s.aimHeight, &s.angleDegrees, &s.sideSpin, &s.topSpin);
        lab.shoot(s);
        Vec3 last = lab.bodies().back().position;
        std::vector<Vec3> knots(lab.netPositions().begin(), lab.netPositions().end());
        // (XTICKS: quantos ticks; 200 sem ele.)
        const int ticks = std::getenv("XTICKS") ? std::atoi(std::getenv("XTICKS")) : 200;
        for (int t = 0; t < ticks; ++t) {
            lab.step();
            const NetLabBody3D& b = lab.bodies().back();
            const NetStatus3D st = lab.netStatus();
            // (O no que mais andou no tick: onde ele esta.)
            std::size_t fastest = 0;
            float moved = 0.0f;
            const std::span<const Vec3> now = lab.netPositions();
            for (std::size_t i = 0; i < now.size() && i < knots.size(); ++i)
                if ((now[i] - knots[i]).length() > moved) {
                    moved = (now[i] - knots[i]).length();
                    fastest = i;
                }
            const InflatedBallState3D ball = lab.scene()->inflatedBallState(lab.balls().back());
            std::printf("t=%3d bola (%.3f %.3f %.3f) v=%.2f vx=%+.2f giro=%.1f | rede: %s subpassos %u contatos %u vmax %.2f fio %.1f%% | no %zu (%.3f %.3f %.3f) andou %.4f\n", t,
                b.position.x, b.position.y, b.position.z, ball.linearVelocity.length(), ball.linearVelocity.x, ball.angularVelocity.length(),
                st.sleeping ? "dormindo" : "acordada", st.substeps, st.contacts, st.maximumSpeed, st.maximumStrain * 100.0f, fastest, now[fastest].x,
                now[fastest].y, now[fastest].z, moved);
            knots.assign(now.begin(), now.end());
            last = b.position;
        }
        if (const char* list = std::getenv("XNOS")) {
            // (XNOS="a b c": onde esses nos terminam, e onde estavam em repouso.)
            for (const char* p = list; *p;) {
                char* end = nullptr;
                const auto k = static_cast<std::size_t>(std::strtol(p, &end, 10));
                if (end == p) break;
                p = end;
                const Vec3 now = lab.netPositions()[k], rest = lab.netRest()[k];
                std::printf("no %zu: agora (%.3f %.3f %.3f), repouso (%.3f %.3f %.3f)\n", k, now.x, now.y, now.z, rest.x, rest.y, rest.z);
            }
        }
        return 0;
    }
    const Modeling::ProceduralObject crate = load("wood_crate.json");
    std::printf("rede: %zu nos, %zu fios (a fisica: %zu nos, %zu fios, %zu celulas) | tick de 120 Hz (orcamento 8,33 ms)\n",
        goal.net->drawnKnots.size(), goal.net->drawnSegments.size(), goal.net->knots.size(), goal.net->segments.size(),
        goal.net->faces.size());
    std::printf("%-34s %5s | %-27s | %-27s\n", "cenario", "ticks", "rede ms: media p50 p95 max", "cena ms: media p50 p95 max");

    ObjectNetLab3D lab(engine, materials);
    // Montar a cena (a rede assentando sozinha ate dormir).
    {
        const auto begin = Clock::now();
        lab.start(goal);
        std::printf("%-34s montou em %.0f ms\n", "montar a cena (assentar a rede)", millisecondsSince(begin));
        // A rede parada: a flecha do cabo do alto (o no dele mais longe da reta entre as pontas) e o quanto
        // o teto desceu ao assentar (o no do teto que mais baixou).
        const Modeling::ObjectNet& net = *goal.net;
        const std::span<const Vec3> rest = lab.netRest();
        std::vector<int> cordLinks(net.knots.size(), 0);
        for (const auto& segment : net.segments)
            if (segment.cord) ++cordLinks[segment.a], ++cordLinks[segment.b];
        std::vector<std::size_t> ends;
        for (std::size_t i = 0; i < cordLinks.size(); ++i)
            if (cordLinks[i] == 1) ends.push_back(i);
        float sag = 0.0f;
        if (ends.size() == 2) {
            const Vec3 a = rest[ends[0]], along = rest[ends[1]] - a;
            for (std::size_t i = 0; i < cordLinks.size(); ++i) {
                if (cordLinks[i] == 0) continue;
                const float t = std::clamp(dot(rest[i] - a, along) / along.lengthSquared(), 0.0f, 1.0f);
                sag = std::max(sag, std::sqrt((rest[i] - (a + along * t)).lengthSquared()));
            }
        }
        float top = -1e9f;
        for (const Vec3& k : net.knots) top = std::max(top, k.z);
        std::size_t pin = 0;
        while (pin + 1 < net.pinned.size() && net.pinned[pin] == 0) ++pin;
        const float shift = net.knots[pin].z - rest[pin].z;
        float drop = 0.0f;
        for (std::size_t i = 0; i < net.knots.size(); ++i)
            if (net.knots[i].z > top - 1e-4f) drop = std::max(drop, net.knots[i].z - rest[i].z - shift);
        // A dobra do pano no cabo, no meio do vao: quanto o caminho do fio do teto que chega ao no do cabo
        // mais perto de y = 0 vira ao seguir pelo fio do fundo (0: reto).
        std::size_t middle = ends.empty() ? 0 : ends[0];
        for (std::size_t i = 0; i < cordLinks.size(); ++i)
            if (cordLinks[i] > 0 && std::abs(rest[i].y) < std::abs(rest[middle].y)) middle = i;
        Vec3 fromRoof, intoBack;
        for (const auto& segment : net.segments) {
            if (segment.cord || (segment.a != middle && segment.b != middle)) continue;
            const Vec3 d = rest[segment.a == middle ? segment.b : segment.a] - rest[middle];
            if (std::abs(d.y) > 0.5f * std::sqrt(d.lengthSquared())) continue;
            // (O do teto vai para o travessao, quase na horizontal; o do fundo desce.)
            if (d.x > std::abs(d.z)) fromRoof = d * -1.0f;
            else intoBack = d;
        }
        const float turn = fromRoof.lengthSquared() > 0.0f && intoBack.lengthSquared() > 0.0f
            ? std::acos(std::clamp(dot(fromRoof, intoBack) / std::sqrt(fromRoof.lengthSquared() * intoBack.lengthSquared()), -1.0f, 1.0f)) * 57.29578f
            : -1.0f;
        std::printf("%-34s cabo: flecha %.1f mm (%zu pontas); teto: desceu %.1f mm; a dobra no cabo: %.1f graus\n", "a rede parada", 1000.0f * sag,
            ends.size(), 1000.0f * drop, turn);
    }
    {
        Series series;
        for (int t = 0; t < 600; ++t) tick(lab, series);
        report("rede parada (dormindo)", series);
    }
    const auto shot = [&](float speed, float across, float height, float angle) {
        NetShotSettings3D settings;
        settings.speed = speed;
        settings.distance = 11.0f;
        settings.aimAcross = across;
        settings.aimHeight = height;
        settings.angleDegrees = angle;
        return settings;
    };
    {
        lab.reset();
        lab.shoot(shot(25.0f, 0.0f, 0.5f, 0.0f));
        // O lance por fase (pelos subpassos da rede no tick: 16 o impacto, 8 a
        // bola devagar, 4 encostada ou a rede sozinha) e o quanto ela demora a
        // dormir.
        Series series;
        double phaseCost[17] {};
        int phaseTicks[17] {};
        int awake = 0;
        for (int t = 0; t < 1200 && !(lab.metrics().stopTime >= 0.0f && lab.netStatus().sleeping); ++t) {
            Series one;
            tick(lab, one);
            const NetStatus3D status = lab.netStatus();
            if (!status.sleeping || status.contacts > 0) {
                series.net.push_back(one.net.front());
                series.scene.push_back(one.scene.front());
            }
            if (!status.sleeping) awake = t + 1;
            const int phase = std::min<int>(16, static_cast<int>(status.substeps));
            phaseCost[phase] += one.net.front();
            ++phaseTicks[phase];
        }
        report("chute 25 m/s no meio", series, "(ate a rede dormir)");
        std::printf("  por fase:");
        for (const int phase : { 16, 8, 4, 2, 1, 0 })
            if (phaseTicks[phase] > 0) std::printf("  %d subpassos: %d ticks, %.1f ms", phase, phaseTicks[phase], phaseCost[phase]);
        const NetShotMetrics3D& metrics = lab.metrics();
        const Vec3 ball = lab.bodies().back().position;
        std::printf("\n  realismo: estufou %.2f m, parou em %.2f s em (%.2f %.2f), fio no maximo %.1f%%, rede dormiu %.2f s apos o chute\n",
            metrics.bulge, metrics.stopTime, ball.x, ball.y, metrics.peakStrain * 100.0f, static_cast<float>(awake) / 120.0f);
        Series resting;
        for (int t = 0; t < 600; ++t) tick(lab, resting);
        report("bola parada encostada na rede", resting, lab.netStatus().sleeping ? "(rede dormindo)" : "(rede acordada)");
    }
    {
        lab.reset();
        lab.shoot(shot(35.0f, 0.9f, 0.9f, 20.0f));
        report("chute 35 m/s no canto alto", runActive(lab, 1200, [&] { return lab.metrics().stopTime >= 0.0f && lab.netStatus().sleeping; }));
    }
    {
        lab.reset();
        // (Cada uma de um angulo: saindo do mesmo ponto, uma dentro da outra,
        // elas se explodiam antes de chegar.)
        const float aims[6][2] { { -0.8f, 0.3f }, { -0.4f, 0.8f }, { 0.0f, 0.4f }, { 0.3f, 0.9f }, { 0.6f, 0.2f }, { 0.9f, 0.6f } };
        for (const auto& aim : aims) lab.shoot(shot(28.0f, aim[0], aim[1], aim[0] * 30.0f));
        report("rajada de 6 bolas juntas", runActive(lab, 1500, [&] { return lab.netStatus().sleeping && lab.metrics().elapsed > 3.0f; }));
    }
    {
        lab.reset();
        NetShotSettings3D toss = shot(9.0f, 0.0f, 0.4f, 0.0f);
        toss.distance = 6.0f;
        lab.throwObject(crate, toss);
        report("caixa atirada", runActive(lab, 1200, [&] { return lab.netStatus().sleeping && lab.metrics().elapsed > 2.0f; }));
    }
    {
        // O boneco guiado (o modo padrao) dentro do gol: parado sem encostar,
        // encostado no fundo balancando de leve (o parado de verdade) e
        // correndo a 9 m/s rente ao fundo, por dentro (os bracos raspando na
        // rede; atravessar a rede do lado, a barreira nao deixa).
        lab.reset();
        PhysicsScene3D& scene = *lab.scene();
        const RagdollProfile3D profile = loadRagdollProfile3D(std::string(MATTERENGINE_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
        RagdollSpawnDefinition3D spawn;
        spawn.pelvisPosition = { -0.7f, 0.2f, profile.standingRootHeightMeters };
        spawn.orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 3.14159265f);
        const RagdollHandle3D doll = scene.createRagdoll(profile, spawn);
        scene.setRagdollGuided(doll, true);
        std::vector<Vec3> base;
        std::vector<Quaternion> turns;
        for (const PhysicsBodyState3D& link : scene.ragdollState(doll).links) {
            base.push_back(link.position);
            turns.push_back(link.orientation);
        }
        std::vector<Vec3> pose(base.size());
        Vec3 at;
        const auto drive = [&](Vec3 offset, Series& series) {
            at = offset;
            for (std::size_t i = 0; i < base.size(); ++i) pose[i] = base[i] + offset;
            scene.setRagdollGuidePose(doll, pose, turns);
            tick(lab, series);
        };
        // (De onde esta ate o ponto, a velocidade dada.)
        const auto walkTo = [&](Vec3 target, float speed, Series& series) {
            const Vec3 from = at;
            const int ticks = std::max(1, static_cast<int>((target - from).length() / speed * 120.0f));
            for (int t = 1; t <= ticks; ++t) drive(from + (target - from) * (static_cast<float>(t) / static_cast<float>(ticks)), series);
        };
        Series idle, walk, leaning, running;
        for (int t = 0; t < 240; ++t) drive({}, idle);
        report("boneco parado no gol, sem encostar", idle, lab.netStatus().sleeping ? "(rede dormindo)" : "(rede acordada)");
        walkTo({ -1.1f, 0.0f, 0.0f }, 1.5f, walk);
        for (int t = 0; t < 600; ++t) drive({ -1.1f + 0.02f * std::sin(static_cast<float>(t) * 0.026f), 0.0f, 0.0f }, leaning);
        report("boneco encostado no fundo (5 s)", leaning, lab.netStatus().sleeping ? "(rede dormindo)" : "(rede acordada)");
        walkTo({ -0.6f, -2.8f, 0.0f }, 4.0f, walk);
        walkTo({ -1.0f, -2.8f, 0.0f }, 2.0f, walk);
        walkTo({ -1.0f, 2.8f, 0.0f }, 9.0f, running);
        report("boneco correndo 9 m/s rente ao fundo", running);
        scene.destroyRagdoll(doll);
    }
    {
        // Duas traves (as duas do campo), um chute em cada ao mesmo tempo.
        ObjectNetLab3D other(engine, materials);
        other.start(goal);
        lab.reset();
        lab.shoot(shot(25.0f, 0.0f, 0.5f, 0.0f));
        other.shoot(shot(25.0f, 0.3f, 0.6f, 0.0f));
        Series both;
        for (int t = 0; t < 360; ++t) {
            const auto begin = Clock::now();
            lab.step();
            other.step();
            digest(lab);
            digest(other);
            both.scene.push_back(static_cast<float>(millisecondsSince(begin)));
            both.net.push_back(lab.netStatus().stepMilliseconds + other.netStatus().stepMilliseconds);
        }
        report("duas traves, um chute em cada", both, "(3 s)");
    }
    // O desenho na tela Objetos: a trave e os fios no renderizador de
    // inspecao, no tamanho do visualizador (sem e com o antisserrilhado).
    {
        lab.reset();
        Modeling::Mesh frame = goal.mesh;
        for (Vec3& p : frame.positions) p = p + lab.objectPosition();
        std::vector<Vec3> knots;
        Modeling::drawnNetKnots(*goal.net, lab.netPositions(), knots);
        std::vector<Vec3> wires;
        for (const auto& [a, b] : goal.net->drawnSegments) {
            wires.push_back(knots[a]);
            wires.push_back(knots[b]);
        }
        Modeling::InspectionWires net;
        net.points = wires;
        net.color = goal.net->color;
        net.widthMeters = goal.net->twineDiameter;
        Modeling::Skeleton ground;
        ground.groundHeight = 0.0f;
        Modeling::InspectionScene scene;
        scene.skeleton = &ground;
        scene.mesh = &frame;
        scene.wires = std::span<const Modeling::InspectionWires>(&net, 1);
        Modeling::OrbitCamera camera;
        camera.yaw = 0.6f;
        camera.pitch = 0.38f;
        Modeling::Box3 box;
        box.include(lab.objectPosition() - goal.size * 0.5f);
        box.include(lab.objectPosition() + goal.size * 0.5f);
        Modeling::frameBox(camera, box, 1560.0f / 830.0f, 1.3f);
        for (const int supersample : { 1, 2 }) {
            Modeling::InspectionStyle style;
            style.skeleton = false;
            style.joints = false;
            style.hud = false;
            style.gridExtent = 6.0f;
            style.gridStep = 0.5f;
            style.supersample = supersample;
            std::vector<float> times;
            for (int k = 0; k < 20; ++k) {
                const auto begin = Clock::now();
                const Modeling::Image image = Modeling::renderInspection(scene, style, camera, 1560, 830);
                times.push_back(static_cast<float>(millisecondsSince(begin)));
            }
            std::sort(times.begin(), times.end());
            std::printf("desenho na tela Objetos 1560x830, %s: %.1f ms por quadro (p50)\n",
                supersample == 1 ? "rapido (mexendo)" : "parado (antisserrilhado 2x)", times[times.size() / 2]);
        }
    }
    return 0;
}
