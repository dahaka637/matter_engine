// Testes dos modulos do jogador montado (MatterModeling): o corte da malha,
// a importacao glTF, os ajustes de forma (os puxoes, o pescoco mais curto),
// os pesos pelo calor dos ossos e o espalhamento deles, o pano do uniforme,
// os reparos da triangulacao, o atlas de textura (o arranjo das ilhas pelo
// formato), o bake e o rosto pintado.
// Linkado so contra MatterModeling (sem a engine).

#include "Engine/Modeling/Assembly/FacePaint.hpp"
#include "Engine/Modeling/Assembly/Garment.hpp"
#include "Engine/Modeling/Assembly/MeshEdit.hpp"
#include "Engine/Modeling/Assembly/NeckBand.hpp"
#include "Engine/Modeling/Assembly/NeckShortening.hpp"
#include "Engine/Modeling/Import/GltfImport.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshCut.hpp"
#include "Engine/Modeling/Mesh/MeshRepair.hpp"
#include "Engine/Modeling/Mesh/MeshSculpt.hpp"
#include "Engine/Modeling/Mesh/MeshSkinning.hpp"
#include "Engine/Modeling/Mesh/MeshSubdivision.hpp"
#include "Engine/Modeling/Mesh/MeshTopology.hpp"
#include "Engine/Modeling/Skin/BoneHeatSkin.hpp"
#include "Engine/Modeling/Skin/SkinSmoothing.hpp"
#include "Engine/Modeling/Texture/IslandPacking.hpp"
#include "Engine/Modeling/Texture/TextureBake.hpp"
#include "Engine/Modeling/Texture/UvAtlas.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

using namespace MatterEngine;
using namespace MatterEngine::Modeling;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

// Um tubo fechado ao longo de z (de "from" a "to"), "rings" aneis, com o
// eixo em y = "side".
Mesh tubeMesh(float from, float to, float radius, int rings = 9, int segments = 16, std::string_view part = "corpo", float side = 0.0f) {
    Mesh mesh;
    MeshBuilder builder(mesh);
    builder.part(part).material("pele", rgb(200, 150, 110));
    std::vector<std::vector<std::uint32_t>> loops;
    for (int i = 0; i < rings; ++i) {
        const float z = from + (to - from) * static_cast<float>(i) / static_cast<float>(rings - 1);
        loops.push_back(builder.ellipseRing({ { 0.0f, side, z }, frameAlong({ 0.0f, 0.0f, to > from ? 1.0f : -1.0f }, { 1.0f, 0.0f, 0.0f }) },
            radius, radius, segments));
    }
    builder.loft(loops, Cap::Flat, Cap::Flat);
    return mesh;
}

std::size_t openEdges(const Mesh& mesh) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> uses;
    for (const Triangle& t : mesh.triangles)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = t[static_cast<std::size_t>(i)], b = t[static_cast<std::size_t>((i + 1) % 3)];
            ++uses[{ std::min(a, b), std::max(a, b) }];
        }
    std::size_t open = 0;
    for (const auto& [edge, count] : uses) open += count == 1 ? 1 : 0;
    return open;
}

float weightSum(const SkinWeights& skin) {
    float total = 0.0f;
    for (const float w : skin.weights) total += w;
    return total;
}

// Uma cadeia de tres juntas em z (o "braco": ombro, cotovelo, pulso) e a ponta.
Skeleton chainSkeleton() {
    Skeleton skeleton;
    skeleton.joints.push_back({ "Ombro", -1, { { 0, 0, 0.5f }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "Cotovelo", 0, { { 0, 0, 0.0f }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "Pulso", 1, { { 0, 0, -0.5f }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "Ponta", 2, { { 0, 0, -0.6f }, {} }, JointKind::Tip, Side::Center, 0.1f });
    return skeleton;
}

void testCut() {
    // O plano corta o tubo numa volta so; o que fica e o lado da normal.
    Mesh tube = tubeMesh(-0.5f, 0.5f, 0.1f);
    const CutResult cut = cutByPlane(tube, { { 0.0f, 0.0f, 0.05f }, { 0.0f, 0.0f, 1.0f } });
    require(cut.ok && cut.borders.size() == 1 && cut.borders.front().closed, "o plano deu uma volta");
    for (std::size_t t = 0; t < tube.triangles.size(); ++t) require(tube.center(t).z > 0.05f - 1e-4f, "so o lado da normal fica");
    for (const std::uint32_t v : cut.borders.front().vertices) require(std::abs(tube.positions[v].z - 0.05f) < 1e-5f, "a volta no plano");
    require(boundaryLoops(tube).size() == 1, "uma borda aberta, a do corte");
    // O de perto divide sem tirar: a superficie continua fechada; dois cortes
    // e o pedaco entre eles sai.
    Mesh band = tubeMesh(-0.5f, 0.5f, 0.1f);
    const std::size_t before = band.triangles.size();
    const CutResult upper = splitNear(band, { { 0.0f, 0.0f, 0.2f }, { 0.0f, 0.0f, 1.0f } }, { 0.0f, 0.0f, 0.2f }, 0.15f);
    const CutResult lower = splitNear(band, { { 0.0f, 0.0f, -0.2f }, { 0.0f, 0.0f, 1.0f } }, { 0.0f, 0.0f, -0.2f }, 0.15f);
    require(upper.ok && lower.ok && upper.borders.size() == 1 && lower.borders.size() == 1, "cada corte local deu uma volta");
    require(band.triangles.size() > before && openEdges(band) == 0, "dividir nao abre a superficie");
    const std::vector<CutBorder> barriers { upper.borders.front(), lower.borders.front() };
    require(removeRegion(band, { 0.1f, 0.0f, 0.0f }, barriers) > 0, "o pedaco do meio sai");
    for (std::size_t t = 0; t < band.triangles.size(); ++t) require(std::abs(band.center(t).z) > 0.2f - 1e-4f, "so o meio saiu");
    require(boundaryLoops(band).size() == 2, "duas bordas: as voltas dos cortes");
    // Um corte por superficie (a esfera de raio 0,3 em volta da origem): a
    // volta segue a curva inteira, mesmo com o centro longe dela.
    Mesh ball = tubeMesh(-0.5f, 0.5f, 0.1f, 17);
    const CutField sphere = [](Vec3 p) { return 0.3f - p.length(); };
    const CutResult curved = splitNear(ball, sphere, { 0.0f, 0.1f, 0.28f }, 0.05f);
    require(curved.ok && curved.borders.size() == 1 && curved.borders.front().closed, "a curva seguida inteira");
    for (const std::uint32_t v : curved.borders.front().vertices) require(std::abs(ball.positions[v].length() - 0.3f) < 2e-3f, "a volta na superficie");
}

void testImport() {
    GltfImportSettings settings;
    settings.part = "cabeca";
    GltfImportResult head = importGltfMesh(std::string(MATTERMODELING_BASE_DIR) + "/cabeca_base.glb", settings);
    require(head.ok && head.mesh.triangles.size() == 344, "a cabeca base: 344 triangulos (" + head.error + ")");
    require(head.mesh.parts.size() == 1 && head.mesh.parts.front().name == "cabeca" && head.mesh.parts.front().root, "a parte e raiz");
    // A fresta da nuca (0,14 mm) so fecha com a solda maior.
    require(openEdges(head.mesh) > 0, "a fresta da nuca na malha original");
    settings.weldTolerance = 5e-4f;
    head = importGltfMesh(std::string(MATTERMODELING_BASE_DIR) + "/cabeca_base.glb", settings);
    require(head.ok && openEdges(head.mesh) == 0, "soldada, a cabeca fecha");
    require(head.mesh.signedVolume() > 0.0f, "orientada para fora (z para cima, x para a frente)");
    const GltfImportResult missing = importGltfMesh("nao/existe.glb", settings);
    require(!missing.ok && !missing.error.empty(), "arquivo que nao existe e erro");
}

void testAutoSkin() {
    const Skeleton skeleton = chainSkeleton();
    Mesh arm = tubeMesh(0.5f, -0.6f, 0.06f, 23, 12, "braco");
    const BoneHeatResult result = boneHeatSkin(arm, skeleton);
    require(result.ok && arm.hasSkin(), "pesos criados");
    float elbowShare = 0.0f;
    for (std::size_t v = 0; v < arm.positions.size(); ++v) {
        require(std::abs(weightSum(arm.skin[v]) - 1.0f) < 1e-4f, "os pesos somam 1");
        const float z = arm.positions[v].z;
        if (z > 0.3f) require(weightFor(arm.skin[v], 0) > 0.9f, "o braco no ombro");
        if (z < -0.2f && z > -0.4f) require(weightFor(arm.skin[v], 1) > 0.9f, "o antebraco no cotovelo");
        if (std::abs(z) < 0.03f) elbowShare = std::max(elbowShare, std::min(weightFor(arm.skin[v], 0), weightFor(arm.skin[v], 1)));
    }
    require(elbowShare > 0.25f, "o cotovelo divide o peso");
    // O vertice de outra parte ja pesada (a borda costurada) fica como estava.
    Mesh joined = tubeMesh(0.5f, -0.6f, 0.06f, 12, 12, "braco");
    MeshBuilder builder(joined);
    builder.part("luva");
    std::vector<std::uint32_t> ring;
    float lowest = 1e9f;
    for (const Vec3& p : joined.positions) lowest = std::min(lowest, p.z);
    for (std::uint32_t v = 0; v < joined.positions.size(); ++v)
        if (std::abs(joined.positions[v].z - (-0.5f)) < 1e-4f) ring.push_back(v);
    require(ring.size() == 12, "o anel do pulso");
    // A luva: um leque do anel ate a ponta, pesada toda no pulso.
    const std::uint32_t tip = builder.vertex({ 0.0f, 0.0f, -0.7f });
    joined.skin.assign(joined.positions.size(), SkinWeights {});
    for (std::size_t i = 0; i < ring.size(); ++i) builder.triangle(ring[i], ring[(i + 1) % ring.size()], tip);
    for (const std::uint32_t v : ring) joined.skin[v] = { { 2, 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f } };
    joined.skin[tip] = { { 2, 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f } };
    const std::vector<int> armPart { joined.findPart("braco") };
    require(boneHeatSkin(joined, skeleton, armPart).ok, "pesos so do braco");
    for (const std::uint32_t v : ring) require(weightFor(joined.skin[v], 2) > 0.999f, "a borda costurada fica no pulso");
}

void testShapeTweaks() {
    // Os puxoes: o vertice no centro anda o puxao inteiro, o que esta fora do
    // raio nao anda, o do meio fica no meio e o filtro escolhe quem anda.
    Mesh tube = tubeMesh(-0.2f, 0.2f, 0.05f, 9, 16);
    const std::vector<Vec3> before = tube.positions;
    std::uint32_t front = 0, middle = 0;
    for (std::uint32_t v = 0; v < tube.positions.size(); ++v) {
        const Vec3 p = tube.positions[v];
        if (std::abs(p.z) < 1e-4f && p.x > tube.positions[front].x - 1e-6f) front = v;
        if (std::abs(p.z - 0.05f) < 1e-4f && std::abs(p.y) < 1e-6f && p.x > 0.0f) middle = v;
    }
    require(std::abs(before[front].z) < 1e-4f && std::abs(before[middle].y) < 1e-6f, "os vertices do teste");
    const std::vector<SculptPull> pulls { { before[front], 0.03f, { -0.004f, 0.0f, 0.002f } },
        { before[middle], 0.02f, { 0.0f, 0.003f, 0.001f } } };
    sculptMesh(tube, pulls);
    require(distance(tube.positions[front], before[front] + Vec3 { -0.004f, 0.0f, 0.002f }) < 1e-6f, "o centro anda o puxao inteiro");
    require(std::abs(tube.positions[middle].y) < 1e-7f && std::abs(tube.positions[middle].z - before[middle].z - 0.001f) < 1e-6f,
        "o do meio fica no meio");
    for (std::size_t v = 0; v < before.size(); ++v)
        if (distance(before[v], before[front]) >= 0.03f && distance(before[v], before[middle]) >= 0.02f)
            require(distance(tube.positions[v], before[v]) < 1e-7f, "fora do raio nada anda");
    Mesh kept = tubeMesh(-0.2f, 0.2f, 0.05f, 9, 16);
    sculptMesh(kept, pulls, [&](std::uint32_t v) { return v != front; });
    require(distance(kept.positions[front], before[front]) < 1e-7f, "o filtro segura o vertice");

    // O pescoco mais curto: o chao fica, a base do pescoco sobe "amount", a
    // linha de cima (mais alta atras) e o que esta acima dela ficam, a altura
    // nunca dobra sobre si (sempre crescendo) e so o z muda.
    NeckShortening neck;
    neck.amount = 0.018f;
    require(std::abs(neck.top(neck.topFront[0]) - neck.topFront[1]) < 1e-6f
            && std::abs(neck.top(neck.topBack[0]) - neck.topBack[1]) < 1e-6f,
        "a linha de cima passa pelos dois pontos");
    for (const float x : { -0.12f, -0.05f, 0.0f, 0.034f, 0.09f }) {
        require(std::abs(neck.lift(x, 0.0f)) < 1e-7f, "o chao fica");
        require(std::abs(neck.lift(x, neck.from) - neck.amount) < 1e-6f, "a base do pescoco sobe tudo");
        require(std::abs(neck.lift(x, neck.top(x))) < 1e-6f && std::abs(neck.lift(x, 1.8f)) < 1e-7f, "a cabeca fica");
        float previous = -1.0f;
        for (int i = 0; i <= 400; ++i) {
            const float height = 1.8f * static_cast<float>(i) / 400.0f;
            const float lifted = neck.liftedHeight(x, height);
            require(lifted > previous, "a altura sempre crescendo");
            previous = lifted;
        }
    }
    const Vec3 moved = neck.apply({ 0.02f, -0.03f, -1.0f + 1.5f }, -1.0f);
    require(std::abs(moved.x - 0.02f) < 1e-7f && std::abs(moved.y + 0.03f) < 1e-7f && moved.z > 0.5f + 1e-3f, "so o z muda");
    neck.amount = 0.0f;
    require(neck.lift(0.0f, 1.5f) == 0.0f, "sem encurtar nada muda");
}

void testNeckBand() {
    // A faixa do pescoco: 0 na base, 1 na linha da cabeca. Na frente a base
    // nunca desce ao fundo do V (a pele do V e peito); no lado ela e a gola.
    NeckBand band;
    band.lines.ground = 0.0f;
    band.lines.neckX = 0.0f;
    band.lines.collarDepth = 1.46f;
    band.lines.collarSide = 1.53f;
    band.lines.collarBack = 1.515f;
    band.lines.collarOpening = 75.0f * 3.14159265f / 180.0f;
    band.frontBase = 1.5f;
    const Vec3 front { 0.05f, 0.0f, 0.0f };
    const Vec3 side { 0.025f, -0.0433f, 0.0f }; // 60 graus da frente
    require(std::abs(band.base(front) - 1.5f) < 1e-5f, "na frente a base fica acima do fundo do V");
    require(std::abs(band.base(side) - (1.46f + 0.07f * 0.8f)) < 1e-4f, "no lado a base e a gola");
    const auto at = [&](Vec3 p, float u) {
        const float bottom = band.base(p);
        p.z = bottom + u * (band.headLine(p) - bottom);
        return p;
    };
    require(std::abs(band.along(at(front, 0.0f))) < 1e-4f && std::abs(band.along(at(front, 1.0f)) - 1.0f) < 1e-4f,
        "0 na base, 1 na linha da cabeca");
    // Os pesos: do alto do peito ao pescoco ate o meio, dele a cabeca depois;
    // somando 1; fora da faixa ou longe do meio, nada.
    const int chest = 3, neck = 4, head = 5;
    const auto weights = [&](Vec3 p) { return neckBandWeights(band, p, chest, neck, head); };
    for (const Vec3 p : { front, side }) {
        const std::optional<SkinWeights> low = weights(at(p, 0.25f)), middle = weights(at(p, 0.5f)), high = weights(at(p, 0.75f));
        require(low && middle && high, "os pesos dentro da faixa");
        require(std::abs(weightFor(*low, chest) - 0.5f) < 1e-3f && std::abs(weightFor(*low, neck) - 0.5f) < 1e-3f, "embaixo, peito e pescoco");
        require(std::abs(weightFor(*middle, neck) - 1.0f) < 1e-3f, "no meio, so o pescoco");
        require(std::abs(weightFor(*high, neck) - 0.5f) < 1e-3f && std::abs(weightFor(*high, head) - 0.5f) < 1e-3f, "em cima, pescoco e cabeca");
        for (const SkinWeights& w : { *low, *middle, *high }) require(std::abs(weightSum(w) - 1.0f) < 1e-5f, "os pesos somam 1");
        require(!weights(at(p, -0.1f)) && !weights(at(p, 1.0f)), "abaixo da base e na linha da cabeca, nada");
    }
    require(!weights(at({ 0.0f, -0.08f, 0.0f }, 0.5f)), "longe do meio (o ombro), nada");
}

void testBoneHeatLimits() {
    // Um quadril: a raiz vai na horizontal (em -y) ate a junta da coxa, que
    // desce. Em cima da junta, o vertice esta tao perto do comeco da coxa
    // quanto da ponta do osso da raiz; a coxa continua a raiz e nao pega o que
    // fica atras do comeco dela - a cintura fica com a raiz.
    Skeleton skeleton;
    skeleton.joints.push_back({ "Raiz", -1, { { 0, 0, 0 }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "Coxa", 0, { { 0, -0.2f, 0 }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "Joelho", 1, { { 0, -0.2f, -0.5f }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "Ponta", 2, { { 0, -0.2f, -0.6f }, {} }, JointKind::Tip, Side::Center, 0.1f });
    const auto waist = [&](const BoneHeatSettings& settings) {
        Mesh bar;
        MeshBuilder builder(bar);
        builder.part("corpo").material("pele", rgb(200, 150, 110));
        std::vector<std::vector<std::uint32_t>> loops;
        for (int i = 0; i < 13; ++i) {
            const float y = -0.3f * static_cast<float>(i) / 12.0f;
            loops.push_back(builder.ellipseRing({ { 0.0f, y, 0.0f }, frameAlong({ 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }) }, 0.06f,
                0.06f, 12));
        }
        builder.loft(loops, Cap::Flat, Cap::Flat);
        require(boneHeatSkin(bar, skeleton, {}, settings).ok, "pesos da cintura");
        float root = 0.0f;
        int count = 0;
        for (std::size_t v = 0; v < bar.positions.size(); ++v) {
            const Vec3 p = bar.positions[v];
            if (p.z > 0.05f && p.y < -0.21f && p.y > -0.29f) {
                root += weightFor(bar.skin[v], 0);
                ++count;
            }
        }
        require(count > 0, "os vertices de cima da junta");
        return root / static_cast<float>(count);
    };
    BoneHeatSettings open;
    open.behindStart = 1e9f;
    const float without = waist(open), with = waist({});
    require(with > 0.8f && with > without + 0.2f,
        "a cintura em cima da junta fica com a raiz: " + std::to_string(with) + " (sem o limite " + std::to_string(without) + ")");

    // O pino: o vertice pedido fica com os pesos pedidos (o resto se acomoda).
    Mesh arm = tubeMesh(0.5f, -0.6f, 0.06f, 23, 12, "braco");
    BoneHeatSettings pinned;
    pinned.pin = [](std::uint32_t v) -> std::optional<SkinWeights> {
        if (v != 0) return std::nullopt;
        return SkinWeights { { 2, 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f } };
    };
    require(boneHeatSkin(arm, chainSkeleton(), {}, pinned).ok, "pesos com o pino");
    require(weightFor(arm.skin[0], 2) > 0.999f, "o vertice preso no pulso");
}

void testSkinSmoothing() {
    Skeleton skeleton = chainSkeleton();
    skeleton.joints[1].side = Side::Right;
    skeleton.joints[1].name = "RightCotovelo";
    skeleton.joints.push_back({ "LeftCotovelo", 0, { { 0, 0.2f, 0.0f }, {} }, JointKind::Body, Side::Left, 0.1f });
    // O meio: metade de cada lado; longe do meio, como estava.
    Mesh mesh;
    MeshBuilder builder(mesh);
    builder.part("pano");
    const std::uint32_t a = builder.vertex({ 0.0f, 0.0f, 0.0f }), b = builder.vertex({ 0.0f, -0.1f, 0.0f }), c = builder.vertex({ 0.0f, -0.05f, 0.1f });
    builder.triangle(a, b, c);
    mesh.skin.assign(mesh.positions.size(), { { 1, 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f } });
    std::vector<int> mirror(skeleton.joints.size());
    for (std::size_t j = 0; j < mirror.size(); ++j) mirror[j] = static_cast<int>(j);
    mirror[1] = 4;
    mirror[4] = 1;
    blendMidlineWeights(mesh, mirror, 0.05f);
    require(std::abs(weightFor(mesh.skin[a], 1) - 0.5f) < 1e-4f && std::abs(weightFor(mesh.skin[a], 4) - 0.5f) < 1e-4f, "no meio, metade de cada lado");
    require(weightFor(mesh.skin[b], 1) > 0.999f, "longe do meio fica");
    // O espalhamento: os pesos vao para os vizinhos e somam 1.
    mesh.skin[c] = { { 0, 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f } };
    smoothSkinWeights(mesh, 3);
    for (const SkinWeights& skin : mesh.skin) require(std::abs(weightSum(skin) - 1.0f) < 1e-4f, "espalhados somam 1");
    require(weightFor(mesh.skin[b], 0) > 0.1f, "o peso do vizinho chegou");
    // Preso a uma junta perto do ponto.
    const Vec3 anchor[] { { 0.0f, -0.1f, 0.0f } };
    pinWeights(mesh, anchor, 0.05f, 2, 0.5f);
    require(std::abs(weightFor(mesh.skin[b], 2) - 0.5f) < 1e-3f && std::abs(weightSum(mesh.skin[b]) - 1.0f) < 1e-4f, "metade presa no ponto");
    // Uma fracao dos pesos de umas juntas passa para outra (a camisa atras do
    // quadril: das coxas para a pelve).
    mesh.skin[a] = { { 1, 0, 0, 0 }, { 0.6f, 0.4f, 0.0f, 0.0f } };
    const int from[] { 1 };
    moveWeights(mesh, a, from, 2, 0.5f);
    require(std::abs(weightFor(mesh.skin[a], 1) - 0.3f) < 1e-4f && std::abs(weightFor(mesh.skin[a], 0) - 0.4f) < 1e-4f
            && std::abs(weightFor(mesh.skin[a], 2) - 0.3f) < 1e-4f,
        "metade do peso da junta passa para a outra, o resto fica");
    // O espalhamento em volta de pontos medidos: so os vertices perto do
    // ponto andam e, com a parte, so os que sao so dela (a costura com a
    // outra parte e a outra parte ficam).
    Mesh strip;
    MeshBuilder pieces(strip);
    pieces.part("pele");
    std::vector<std::uint32_t> row;
    for (int k = 0; k <= 6; ++k) {
        row.push_back(pieces.vertex({ 0.01f * static_cast<float>(k), 0.0f, 0.0f }));
        row.push_back(pieces.vertex({ 0.01f * static_cast<float>(k), 0.0f, 0.01f }));
    }
    for (int k = 0; k < 6; ++k) {
        if (k == 3) pieces.part("bota");
        const std::size_t i = static_cast<std::size_t>(2 * k);
        pieces.quad(row[i], row[i + 2], row[i + 3], row[i + 1]);
    }
    strip.skin.assign(strip.positions.size(), {});
    for (std::size_t v = 0; v < strip.positions.size(); ++v)
        strip.skin[v] = { { static_cast<std::uint16_t>(strip.positions[v].x < 0.015f ? 0 : 1), 0, 0, 0 }, { 1.0f, 0.0f, 0.0f, 0.0f } };
    const Mesh before = strip;
    const WeightRelaxPoint skinOnly[] { { { 0.03f, 0.0f, 0.005f }, 0.025f, "pele" } };
    relaxWeights(strip, skinOnly, 2);
    bool skinMoved = false, otherStill = true;
    for (std::size_t v = 0; v < strip.positions.size(); ++v) {
        const bool changed = std::abs(weightFor(strip.skin[v], 0) - weightFor(before.skin[v], 0)) > 1e-5f;
        if (strip.positions[v].x < 0.025f) skinMoved |= changed;
        else otherStill &= !changed;
        require(std::abs(weightSum(strip.skin[v]) - 1.0f) < 1e-4f, "espalhados no lugar somam 1");
    }
    require(skinMoved && otherStill, "so a pele perto do ponto anda; a costura e a bota ficam");
    const WeightRelaxPoint anywhere[] { { { 0.03f, 0.0f, 0.005f }, 0.025f, {} } };
    relaxWeights(strip, anywhere, 2);
    require(std::abs(weightFor(strip.skin[row[6]], 0) - weightFor(before.skin[row[6]], 0)) > 1e-3f, "sem a parte, a costura anda tambem");
    require(std::abs(weightFor(strip.skin[row[12]], 1) - 1.0f) < 1e-6f, "longe do ponto, nada muda");
}

void testGarment() {
    // O "braco" direito (eixo em y = -0,3, z para cima e a mao; o pano e da
    // metade direita, longe do meio): a manga cobre de -0,2 a 0,2 com a barra
    // em 0,2; a pele entra 2 cm por baixo dela.
    const float side = -0.3f;
    Mesh body = tubeMesh(-0.5f, 0.5f, 0.1f, 21, 16, "corpo", side);
    Mesh sleeve = body;
    const RegionCut sleeveCuts[] {
        { planeField({ { 0.0f, side, 0.2f }, { 0.0f, 0.0f, -1.0f } }), { 0.0f, side, 0.2f }, 0.2f, { 0.0f, side, 0.45f } },
        { planeField({ { 0.0f, side, -0.2f }, { 0.0f, 0.0f, 1.0f } }), { 0.0f, side, -0.2f }, 0.2f, { 0.0f, side, -0.45f } } };
    const RegionResult kept = keepRegion(sleeve, sleeveCuts);
    require(kept.ok && kept.borders.size() == 2 && kept.borders[0].closed && kept.borders[1].closed, "a manga entre as barras");
    for (std::size_t t = 0; t < sleeve.triangles.size(); ++t) require(std::abs(sleeve.center(t).z) < 0.2f + 1e-4f, "so entre as barras");
    const Mesh bare = body;
    const RegionCut skinCuts[] {
        { planeField({ { 0.0f, side, 0.18f }, { 0.0f, 0.0f, 1.0f } }), { 0.0f, side, 0.18f }, 0.2f, {} },
        { planeField({ { 0.0f, side, -0.18f }, { 0.0f, 0.0f, -1.0f } }), { 0.0f, side, -0.18f }, 0.2f, {} } };
    const RegionResult skin = removeCovered(body, skinCuts, { 0.1f, side, 0.0f });
    require(skin.ok && skin.borders.size() == 2, "a pele coberta saiu");
    // O pano: a folga fora do corpo (a borda infla tambem: o degrau do recuo).
    inflateShell(sleeve, bare, [](std::uint32_t) { return 0.01f; });
    for (const Triangle& t : sleeve.triangles)
        for (const std::uint32_t v : t) {
            const Vec3 p = sleeve.positions[v];
            const float radius = std::hypot(p.x, p.y - side);
            require(radius > 0.1f + 0.007f && radius < 0.1f + 0.016f,
                "a folga do pano (raio " + std::to_string(radius) + " em z " + std::to_string(p.z) + ")");
        }
    // Os recuos: a superficie fecha numa so.
    const std::uint32_t first = appendMesh(body, sleeve);
    const auto shifted = [&](CutBorder border) {
        for (std::uint32_t& v : border.vertices) v += first;
        return border;
    };
    std::string error;
    const int part = body.findPart("corpo");
    require(stitchRecess(body, shifted(kept.borders[0]), skin.borders[0], static_cast<std::uint16_t>(part), 0, &error)
            && stitchRecess(body, shifted(kept.borders[1]), skin.borders[1], static_cast<std::uint16_t>(part), 0, &error),
        "os recuos costurados: " + error);
    MeshBuilder(body).compact();
    int components = 0;
    (void)edgeComponents(body, &components);
    require(openEdges(body) == 0 && components == 1, "uma superficie fechada");
    require(body.signedVolume() > 0.0f, "orientada para fora");
}

void testRepair() {
    // A diagonal comprida de um losango achatado troca pela curta.
    Mesh flat;
    MeshBuilder builder(flat);
    builder.part("plano");
    const std::uint32_t a = builder.vertex({ 0, 0, 0 }), b = builder.vertex({ 1, -0.1f, 0 }), c = builder.vertex({ 2, 0, 0 }),
                        d = builder.vertex({ 1, 0.1f, 0 });
    builder.triangle(a, b, c);
    builder.triangle(a, c, d);
    require(improveTriangles(flat) == 1, "a diagonal trocou");
    for (const Triangle& t : flat.triangles) {
        const std::set<std::uint32_t> corners(t.begin(), t.end());
        require(!(corners.count(a) && corners.count(c)), "a diagonal comprida saiu");
        require(cross(flat.positions[t[1]] - flat.positions[t[0]], flat.positions[t[2]] - flat.positions[t[0]]).z > 0.0f, "a frente continua");
    }
    // Uma aresta curta demais num tubo some (a borda fica).
    Mesh tube = tubeMesh(-0.5f, 0.5f, 0.1f, 9, 12);
    MeshBuilder(tube).compact();
    tube.positions[30] = tube.positions[31] + (tube.positions[30] - tube.positions[31]) * 0.05f;
    const std::size_t triangles = tube.triangles.size();
    require(collapseShortEdges(tube, 0.3f, [](std::uint32_t) { return false; }) >= 1, "a aresta curta juntou");
    require(tube.triangles.size() <= triangles - 2 && openEdges(tube) == 0, "sem furo");
}

// A malha mais densa sem mudar a forma (o calcao dobrando em curva): inteira
// (cada triangulo em quatro) e so numa regiao (os vizinhos em dois, sem
// rachadura), com a borda de um corte ganhando os meios e os pesos na media.
void testSubdivision() {
    Mesh tube = tubeMesh(-0.5f, 0.5f, 0.1f, 5, 8);
    MeshBuilder(tube).compact();
    tube.skin.assign(tube.positions.size(), SkinWeights {});
    for (std::size_t v = 0; v < tube.positions.size(); ++v) {
        const float lower = std::clamp(0.5f - tube.positions[v].z, 0.0f, 1.0f);
        tube.skin[v].joints = { 0, 1, 0, 0 };
        tube.skin[v].weights = { 1.0f - lower, lower, 0.0f, 0.0f };
    }
    const CutResult border = cutByPlane(tube, { { 0.0f, 0.0f, 0.3f }, { 0.0f, 0.0f, -1.0f } });
    require(border.ok && border.borders.size() == 1, "a borda do corte");
    const std::vector<std::uint32_t> loop = border.borders.front().vertices;

    // Inteira, duas vezes: 16 vezes os triangulos, a mesma area, a borda com
    // 4 vezes os vertices (todos no plano), os pesos somando 1.
    Mesh whole = tube;
    CutBorder wholeBorder = border.borders.front();
    CutBorder* wholeBorders[] { &wholeBorder };
    const std::size_t triangles = whole.triangles.size();
    const float area = whole.surfaceArea();
    const std::size_t open = openEdges(whole);
    require(subdivideTriangles(whole, 2, wholeBorders) > 0, "vertices novos");
    require(whole.triangles.size() == triangles * 16, "cada triangulo em 16");
    require(std::abs(whole.surfaceArea() - area) < 1e-5f, "a forma nao muda (a area)");
    require(openEdges(whole) == open * 4, "so a borda aberta, sem rachadura (" + std::to_string(openEdges(whole)) + ")");
    require(wholeBorder.vertices.size() == loop.size() * 4, "a borda ganhou os meios");
    for (const std::uint32_t v : wholeBorder.vertices) require(std::abs(whole.positions[v].z - 0.3f) < 1e-5f, "a borda no plano");
    for (const SkinWeights& weights : whole.skin) require(std::abs(weightSum(weights) - 1.0f) < 1e-4f, "os pesos somam 1");
    // O peso do meio de uma aresta e a media das pontas (aqui, linear em z).
    for (std::size_t v = 0; v < whole.positions.size(); ++v) {
        float lower = 0.0f;
        for (std::size_t i = 0; i < 4; ++i)
            if (whole.skin[v].joints[i] == 1) lower += whole.skin[v].weights[i];
        require(std::abs(lower - std::clamp(0.5f - whole.positions[v].z, 0.0f, 1.0f)) < 1e-4f, "o peso na media das pontas");
    }

    // So a metade de baixo: menos triangulos, a mesma area, nenhuma aresta
    // aberta a mais (os da passagem partem em dois).
    Mesh half = tube;
    CutBorder halfBorder = border.borders.front();
    CutBorder* halfBorders[] { &halfBorder };
    subdivideTriangles(half, 2, halfBorders, [&](std::size_t t) { return half.center(t).z < -0.05f; });
    require(half.triangles.size() > triangles && half.triangles.size() < triangles * 16, "so a regiao densa");
    require(std::abs(half.surfaceArea() - area) < 1e-5f, "a forma nao muda (a area, so a regiao)");
    require(openEdges(half) == openEdges(tube), "sem rachadura na passagem (" + std::to_string(openEdges(half)) + ")");
    require(halfBorder.vertices == loop, "a borda de cima, fora da regiao, fica");
    for (std::size_t t = 0; t < half.triangles.size(); ++t)
        require(half.area(t) > 1e-8f, "nenhum triangulo sem area");
    // Na passagem nenhuma agulha: a metade de uma metade nao parte de novo
    // (o menor angulo fica perto do da malha de antes).
    const auto smallestAngle = [](const Mesh& mesh) {
        float smallest = 180.0f;
        for (const Triangle& tri : mesh.triangles)
            for (int i = 0; i < 3; ++i) {
                const Vec3 a = mesh.positions[tri[static_cast<std::size_t>(i)]];
                const Vec3 b = mesh.positions[tri[static_cast<std::size_t>((i + 1) % 3)]] - a;
                const Vec3 c = mesh.positions[tri[static_cast<std::size_t>((i + 2) % 3)]] - a;
                smallest = std::min(smallest, std::acos(std::clamp(dot(b, c) / (b.length() * c.length()), -1.0f, 1.0f)) * 57.2958f);
            }
        return smallest;
    };
    require(smallestAngle(half) > 0.45f * smallestAngle(tube),
        "agulha na passagem (" + std::to_string(smallestAngle(half)) + " contra " + std::to_string(smallestAngle(tube)) + " graus)");
}

void testAtlasAndBake() {
    Mesh box;
    MeshBuilder builder(box);
    builder.part("caixa").material("cinza", rgb(128, 128, 128));
    builder.box({}, { 0.1f, 0.2f, 0.3f });
    UvAtlasSettings settings;
    settings.size = 256;
    const UvAtlasResult atlas = unwrapAtlas(box, settings);
    require(atlas.ok && box.hasUv() && atlas.charts == 6 && atlas.texelsPerMeter > 0.0f, "seis ilhas, uma por face");
    // As ilhas nao se sobrepoem (os texels de cada uma, pelo centro).
    std::vector<int> owner(256 * 256, -1);
    std::map<int, int> chartOfFace;
    for (std::size_t t = 0; t < box.triangles.size(); ++t) {
        const Vec3 n = box.normal(t);
        const int face = (std::abs(n.x) > 0.9f ? 0 : std::abs(n.y) > 0.9f ? 2 : 4) + ((n.x + n.y + n.z) > 0.0f ? 1 : 0);
        const std::array<Uv, 3>& uv = box.triangleUv[t];
        for (const Uv& corner : uv) require(corner.u >= 0.0f && corner.u <= 1.0f && corner.v >= 0.0f && corner.v <= 1.0f, "no atlas");
        // A escala: a aresta no atlas tem o comprimento dela vezes a densidade.
        const float edge3 = distance(box.positions[box.triangles[t][0]], box.positions[box.triangles[t][1]]);
        const float edge2 = std::hypot(uv[1].u - uv[0].u, uv[1].v - uv[0].v) * 256.0f;
        require(std::abs(edge2 - edge3 * atlas.texelsPerMeter) < 0.5f, "a mesma densidade em toda parte");
        for (int y = 0; y < 256; ++y)
            for (int x = 0; x < 256; ++x) {
                const float px = (static_cast<float>(x) + 0.5f) / 256.0f, py = (static_cast<float>(y) + 0.5f) / 256.0f;
                const auto side = [&](Uv p, Uv q) { return (q.u - p.u) * (py - p.v) - (q.v - p.v) * (px - p.u); };
                const float s0 = side(uv[0], uv[1]), s1 = side(uv[1], uv[2]), s2 = side(uv[2], uv[0]);
                if (!((s0 > 1e-6f && s1 > 1e-6f && s2 > 1e-6f) || (s0 < -1e-6f && s1 < -1e-6f && s2 < -1e-6f))) continue;
                int& texel = owner[static_cast<std::size_t>(y * 256 + x)];
                require(texel < 0 || texel == face, "as ilhas nao se cobrem");
                texel = face;
            }
    }
    // O bake: a cor da funcao no lugar de cada face.
    TextureBakeSettings bake;
    bake.size = 256;
    bake.samples = 2;
    const Image image = bakeTexture(box, [](const SurfaceSample& sample) {
        return sample.normal.x > 0.5f ? rgb(220, 30, 30) : sample.normal.z > 0.5f ? rgb(30, 30, 220) : rgb(128, 128, 128);
    }, bake);
    for (std::size_t t = 0; t < box.triangles.size(); ++t) {
        const std::array<Uv, 3>& uv = box.triangleUv[t];
        const Color c = image.sample({ (uv[0].u + uv[1].u + uv[2].u) / 3.0f, (uv[0].v + uv[1].v + uv[2].v) / 3.0f });
        const Vec3 n = box.normal(t);
        if (n.x > 0.5f) require(c.r > 0.8f && c.b < 0.2f, "a face +x vermelha");
        else if (n.z > 0.5f) require(c.b > 0.8f && c.r < 0.2f, "a face +z azul");
        else require(std::abs(c.r - 0.5f) < 0.03f, "as outras cinzas");
    }
    // A borda de um desenho (a cobertura pela distancia, como as camadas da
    // pintura) ocupa mais texels com a borda mais larga do bake.
    const auto softTexels = [&](float edgeTexels) {
        TextureBakeSettings soft;
        soft.size = 256;
        soft.samples = 3;
        soft.edgeTexels = edgeTexels;
        const Image edge = bakeTexture(box, [](const SurfaceSample& sample) {
            const float value = std::clamp(0.5f + sample.position.x / std::max(1e-6f, sample.footprint), 0.0f, 1.0f);
            return Color { value, value, value, 1.0f };
        }, soft);
        int count = 0;
        for (int y = 0; y < 256; ++y)
            for (int x = 0; x < 256; ++x) {
                const Color c = edge.pixel(x, y);
                if (c.a > 0.5f && c.r > 0.05f && c.r < 0.95f) ++count;
            }
        return count;
    };
    // O bake adaptativo (uma amostra no centro, todas so nas bordas) igual ao
    // completo: listras de borda suave em toda face; e uma linha mais fina que
    // um texel (que o centro pode nao ver) inteira na parte marcada como fina.
    const auto compare = [&](const SurfacePaint& paint, std::vector<int> fineParts) {
        TextureBakeSettings exact;
        exact.size = 256;
        exact.samples = 3;
        exact.fineParts = std::move(fineParts);
        TextureBakeSettings quick = exact;
        exact.adaptive = false;
        const Image full = bakeTexture(box, paint, exact), adaptive = bakeTexture(box, paint, quick);
        float worst = 0.0f;
        for (int y = 0; y < 256; ++y)
            for (int x = 0; x < 256; ++x) {
                const Color a = full.pixel(x, y), b = adaptive.pixel(x, y);
                require((a.a > 0.5f) == (b.a > 0.5f), "o adaptativo cobre os mesmos texels");
                worst = std::max({ worst, std::abs(a.r - b.r), std::abs(a.g - b.g), std::abs(a.b - b.b) });
            }
        return worst;
    };
    const auto stripes = [](const SurfaceSample& sample) {
        const float phase = std::sin(sample.position.x * 90.0f + sample.position.y * 40.0f + sample.position.z * 25.0f);
        const float value = std::clamp(0.5f + phase / std::max(1e-4f, sample.footprint * 60.0f), 0.0f, 1.0f);
        return Color { value, 0.4f, 1.0f - value, 1.0f };
    };
    const float soft = compare(stripes, {});
    require(soft <= 1.5f / 255.0f, "o adaptativo igual ao completo nas listras (pior " + std::to_string(soft * 255.0f) + "/255)");
    const auto hair = [](const SurfaceSample& sample) {
        return std::abs(sample.position.z - 0.013f) < 0.0004f ? Color { 0.0f, 0.0f, 0.0f, 1.0f } : Color { 1.0f, 1.0f, 1.0f, 1.0f };
    };
    const float fine = compare(hair, { box.findPart("caixa") });
    require(fine <= 1.5f / 255.0f, "a linha fina inteira na parte fina (pior " + std::to_string(fine * 255.0f) + "/255)");
    const int sharp = softTexels(0.0f), wide = softTexels(2.0f);
    require(sharp > 0 && 2 * wide > 3 * sharp, "a borda de 2 texels mais larga que a de um (" + std::to_string(sharp) + " e " + std::to_string(wide) + ")");
    // A amostra bilinear: o centro do texel e a cor dele; entre quatro, a media.
    Image four(2, 2, { 0, 0, 0, 1 });
    four.setPixel(0, 0, { 1, 0, 0, 1 });
    four.setPixel(1, 0, { 0, 1, 0, 1 });
    four.setPixel(0, 1, { 0, 0, 1, 1 });
    four.setPixel(1, 1, { 1, 1, 1, 1 });
    require(distance(Vec3 { four.sample({ 0.25f, 0.25f }).r, four.sample({ 0.25f, 0.25f }).g, 0.0f }, { 1.0f, 0.0f, 0.0f }) < 0.01f, "o centro do texel");
    const Color middle = four.sample({ 0.5f, 0.5f });
    require(std::abs(middle.r - 0.5f) < 0.01f && std::abs(middle.g - 0.5f) < 0.01f && std::abs(middle.b - 0.5f) < 0.01f, "a media no meio");
    // O mapa acompanha a malha: compactar e espelhar levam as coordenadas.
    MeshBuilder(box).mirrorParts(std::vector<std::string> { "caixa" });
    require(box.triangleUv.size() == box.triangles.size() && box.validateLayout().empty(), "o espelho leva o mapa");
}

// Um retangulo em dois triangulos (texels).
std::vector<std::array<PackPoint, 3>> rectangleIsland(float x0, float y0, float x1, float y1) {
    return { { PackPoint { x0, y0 }, PackPoint { x1, y0 }, PackPoint { x1, y1 } }, { PackPoint { x0, y0 }, PackPoint { x1, y1 }, PackPoint { x0, y1 } } };
}

// As ilhas arrumadas: dentro do atlas, so movidas e giradas, nenhum texel
// com duas e pelo menos 2 x padding texels entre duas delas.
void checkPacking(const std::vector<PackIsland>& islands, const std::vector<PackPlacement>& placed, int size, int padding) {
    std::vector<int> owner(static_cast<std::size_t>(size * size), -1);
    for (std::size_t i = 0; i < islands.size(); ++i)
        for (const auto& triangle : islands[i].triangles) {
            std::array<PackPoint, 3> t;
            for (std::size_t k = 0; k < 3; ++k) {
                t[k] = placed[i].at(triangle[k]);
                require(t[k].x >= 0.0f && t[k].x <= static_cast<float>(size) && t[k].y >= 0.0f && t[k].y <= static_cast<float>(size), "dentro do atlas");
            }
            for (std::size_t k = 0; k < 3; ++k) {
                const PackPoint a = triangle[k], b = triangle[(k + 1) % 3], p = t[k], q = t[(k + 1) % 3];
                require(std::abs(std::hypot(q.x - p.x, q.y - p.y) - std::hypot(b.x - a.x, b.y - a.y)) < 1e-3f, "o lugar so move e gira");
            }
            for (int y = 0; y < size; ++y)
                for (int x = 0; x < size; ++x) {
                    const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
                    const auto side = [&](PackPoint p, PackPoint q) { return (q.x - p.x) * (py - p.y) - (q.y - p.y) * (px - p.x); };
                    const float s0 = side(t[0], t[1]), s1 = side(t[1], t[2]), s2 = side(t[2], t[0]);
                    if (!((s0 >= 0.0f && s1 >= 0.0f && s2 >= 0.0f) || (s0 <= 0.0f && s1 <= 0.0f && s2 <= 0.0f))) continue;
                    int& texel = owner[static_cast<std::size_t>(y * size + x)];
                    require(texel < 0 || texel == static_cast<int>(i), "as ilhas nao se cobrem");
                    texel = static_cast<int>(i);
                }
        }
    int closest = 1000;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            const int a = owner[static_cast<std::size_t>(y * size + x)];
            if (a < 0) continue;
            for (int v = std::max(0, y - 2 * padding); v <= std::min(size - 1, y + 2 * padding); ++v)
                for (int u = std::max(0, x - 2 * padding); u <= std::min(size - 1, x + 2 * padding); ++u) {
                    const int b = owner[static_cast<std::size_t>(v * size + u)];
                    if (b >= 0 && b != a) closest = std::min(closest, std::max(std::abs(u - x), std::abs(v - y)));
                }
        }
    require(closest >= 2 * padding, "a folga entre as ilhas: " + std::to_string(closest) + " texels");
}

void testIslandPacking() {
    PackSettings settings;
    settings.size = 64;
    settings.padding = 2;
    settings.cell = 2;
    // Um L grande e um quadrado que so cabe no vao do L (pela caixa, os dois
    // nao cabem no quadrado de 64).
    PackIsland ell;
    ell.triangles = rectangleIsland(0.0f, 0.0f, 54.0f, 22.0f);
    for (const auto& t : rectangleIsland(0.0f, 22.0f, 22.0f, 54.0f)) ell.triangles.push_back(t);
    ell.width = ell.height = 54.0f;
    PackIsland square;
    square.triangles = rectangleIsland(0.0f, 0.0f, 22.0f, 22.0f);
    square.width = square.height = 22.0f;
    const std::vector<PackIsland> nested { square, ell };
    const auto placed = packIslands(nested, settings);
    require(placed.has_value() && placed->size() == 2, "o quadrado cabe no vao do L");
    checkPacking(nested, *placed, settings.size, settings.padding);
    // Um bloco e uma ilha comprida: uma das duas vai em pe.
    PackIsland block;
    block.triangles = rectangleIsland(0.0f, 0.0f, 50.0f, 56.0f);
    block.width = 50.0f;
    block.height = 56.0f;
    PackIsland stick;
    stick.triangles = rectangleIsland(0.0f, 0.0f, 56.0f, 4.0f);
    stick.width = 56.0f;
    stick.height = 4.0f;
    const std::vector<PackIsland> pair { block, stick };
    const auto both = packIslands(pair, settings);
    require(both && ((*both)[0].turned || (*both)[1].turned), "uma ilha em pe");
    checkPacking(pair, *both, settings.size, settings.padding);
    // Grande demais para o quadrado: nada.
    PackIsland huge;
    huge.triangles = rectangleIsland(0.0f, 0.0f, 80.0f, 10.0f);
    huge.width = 80.0f;
    huge.height = 10.0f;
    require(!packIslands({ huge }, settings), "a ilha maior que o atlas nao cabe");
}

void testFace() {
    FaceLayout face;
    face.ground = 0.0f;
    face.eyeSurfaceX = 0.08f;
    const FacePaintShape& s = face.shape;
    const Color skin = rgb(212, 152, 106);
    // O centro da pupila (um pouco para dentro e para cima do centro do olho).
    const float yaw = s.eyeYaw * 3.14159265f / 180.0f;
    const Vec3 inward { std::sin(yaw), std::cos(yaw), 0.0f };
    const Vec3 pupil = Vec3 { 0.08f, s.eyeY, s.eyeHeight + 0.0003f } + inward * 0.0006f;
    const std::optional<Color> right = paintFace(face, pupil, { 1, 0, 0 }, skin);
    const std::optional<Color> left = paintFace(face, { pupil.x, -pupil.y, pupil.z }, { 1, 0, 0 }, skin);
    require(right && left && right->r < 0.1f && left->r < 0.1f, "a pupila nos dois olhos");
    require(!paintFace(face, { 0.08f, 0.0f, 1.76f }, { 1, 0, 0 }, skin), "a testa fica pele");
    require(!paintFace(face, { -0.08f, s.eyeY, s.eyeHeight }, { -1, 0, 0 }, skin), "a nuca fica pele");
    const std::optional<Color> brow = paintFace(face, Vec3 { 0.08f, s.eyeY, s.eyeHeight + s.browLowInner + 0.004f }, { 1, 0, 0 }, skin);
    require(brow && brow->r < 0.3f, "a sobrancelha acima do olho");
    // A boca: a fresta escura e o labio de cima acima dela; abaixo dela, pele
    // (a face clara nao e labio).
    const float slit = s.mouthSlit.front()[1];
    const std::optional<Color> line = paintFace(face, { 0.11f, 0.0f, slit }, { 1, 0, 0 }, skin);
    require(line && line->r < skin.r - 0.2f, "a fresta da boca");
    const std::optional<Color> upper = paintFace(face, { 0.11f, -0.004f, slit + 0.0025f }, { 1, 0, 0 }, skin);
    require(upper && upper->g < skin.g && upper->r > 0.5f, "o labio de cima");
    require(!paintFace(face, { 0.11f, -0.004f, slit - 0.0015f }, { 1, 0, 0 }, skin), "logo abaixo da fresta, pele");
    require(!paintFace(face, { 0.11f, -0.004f, slit - 0.004f }, { 1, 0, 0 }, skin), "abaixo da fresta, pele");
    require(!paintFace(face, { 0.11f, -0.004f, s.upperLipTop.front()[1] + 0.006f }, { 1, 0, 0 }, skin), "acima do labio de cima, pele");
    // A placa da orelha medida na malha (earPlate): uma elipse inclinada
    // virada para fora (-y) alem de earOutside; uma peca menor ao lado, um
    // triangulo virado para a cabeca, um perto do meio e um de outra parte
    // ficam de fora.
    Mesh head;
    {
        MeshBuilder builder(head);
        builder.part("cabeca").material("pele", skin);
        const float tilt = 0.26f, side = -0.085f, a = 0.017f, b = 0.03f;
        const std::array<float, 2> centre { -0.01f, 1.68f }, major { -std::sin(tilt), std::cos(tilt) }, minor { std::cos(tilt), std::sin(tilt) };
        const std::uint32_t middle = builder.vertex({ centre[0], side, centre[1] });
        std::vector<std::uint32_t> rim;
        for (int k = 0; k < 48; ++k) {
            const float t = 2.0f * 3.14159265f * static_cast<float>(k) / 48.0f;
            rim.push_back(builder.vertex({ centre[0] + minor[0] * a * std::cos(t) + major[0] * b * std::sin(t), side,
                centre[1] + minor[1] * a * std::cos(t) + major[1] * b * std::sin(t) }));
        }
        for (std::size_t k = 0; k < rim.size(); ++k) builder.triangle(middle, rim[k], rim[(k + 1) % rim.size()]);
        const auto flat = [&](float x, float y, float h, float size, bool outward) {
            const std::uint32_t p = builder.vertex({ x, y, h }), q = builder.vertex({ x + size, y, h }), r = builder.vertex({ x, y, h + size });
            if (outward) builder.triangle(p, q, r);
            else builder.triangle(p, r, q);
        };
        flat(0.05f, side, 1.6f, 0.006f, true);
        flat(-0.05f, side, 1.6f, 0.04f, false);
        flat(-0.05f, -0.05f, 1.6f, 0.04f, true);
        builder.part("corpo");
        flat(-0.05f, side, 1.5f, 0.05f, true);
    }
    const EarPlate plate = earPlate(head, head.findPart("cabeca"), s.earOutside, 0.0f);
    require(plate.outline.size() == 48, "o contorno da placa: a borda da maior peca");
    require(std::abs(plate.center[0] + 0.01f) < 3e-4f && std::abs(plate.center[1] - 1.68f) < 3e-4f, "o centro da placa");
    require(std::abs(plate.major - 0.03f) < 0.0006f && std::abs(plate.minor - 0.017f) < 0.0004f, "os semieixos da elipse da placa");
    require(std::abs(plate.axis[0] * -std::sin(0.26f) + plate.axis[1] * std::cos(0.26f)) > 0.999f, "a inclinacao da placa");
    // A orelha nao e pintada aqui (e uma camada do desenho, PaintDesign).
    require(!paintFace(face, { plate.center[0], -0.085f, plate.center[1] }, { 0, -1, 0 }, skin), "a placa da orelha sem tinta do rosto");
}

// Os vertices levados a lugares exatos (a forma do rosto): o mais perto de
// cada ponto anda, o do meio fica no meio e um ponto sem vertice e erro.
void testVertexMoves() {
    Mesh tube = tubeMesh(-0.2f, 0.2f, 0.05f, 5, 8);
    std::uint32_t side = 0, middle = 0;
    for (std::uint32_t v = 0; v < tube.positions.size(); ++v) {
        const Vec3 p = tube.positions[v];
        if (std::abs(p.z) < 1e-4f && p.y < tube.positions[side].y - 1e-6f) side = v;
        if (std::abs(p.z) < 1e-4f && std::abs(p.y) < 1e-6f && p.x > 0.0f) middle = v;
    }
    const std::vector<Vec3> before = tube.positions;
    const std::vector<VertexMove> moves { { before[side] + Vec3 { 0.0005f, 0.0f, 0.0f }, before[side] + Vec3 { 0.0f, -0.01f, 0.003f } },
        { before[middle], before[middle] + Vec3 { 0.004f, -0.002f, 0.0f } } };
    std::string error;
    require(moveVertices(tube, moves, 0.002f, error), "os vertices andam (" + error + ")");
    require(distance(tube.positions[side], before[side] + Vec3 { 0.0f, -0.01f, 0.003f }) < 1e-6f, "o mais perto vai para o lugar");
    require(std::abs(tube.positions[middle].y) < 1e-7f && std::abs(tube.positions[middle].x - before[middle].x - 0.004f) < 1e-6f,
        "o do meio fica no meio");
    std::size_t moved = 0;
    for (std::size_t v = 0; v < before.size(); ++v) moved += distance(before[v], tube.positions[v]) > 1e-7f ? 1 : 0;
    require(moved == 2, "so os dois andam");
    const std::vector<VertexMove> lost { { { 0.3f, 0.3f, 0.3f }, { 0.3f, 0.3f, 0.31f } } };
    require(!moveVertices(tube, lost, 0.002f, error) && !error.empty(), "sem vertice perto: erro");
}

} // namespace

int main() {
    try {
        testCut();
        testImport();
        testShapeTweaks();
        testVertexMoves();
        testNeckBand();
        testAutoSkin();
        testBoneHeatLimits();
        testSkinSmoothing();
        testGarment();
        testRepair();
        testSubdivision();
        testAtlasAndBake();
        testIslandPacking();
        testFace();
    } catch (const std::exception& exception) {
        std::cerr << "MatterModelingAssembly test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterModelingAssembly tests passed (" << checks << " checagens)\n";
    return 0;
}
