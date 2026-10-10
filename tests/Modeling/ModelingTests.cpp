// Testes do nucleo da modelagem (MatterModeling): geometria, kit de
// modelagem, analise, cortes, camera, renderizador, regioes, skin e sessao.
// Linkado so contra MatterModeling: prova que o nucleo nao depende da engine.

#include "Engine/Modeling/Analysis/ConnectivityCheck.hpp"
#include "Engine/Modeling/Analysis/CrossSection.hpp"
#include "Engine/Modeling/Analysis/DeformationCheck.hpp"
#include "Engine/Modeling/Analysis/ModelValidator.hpp"
#include "Engine/Modeling/Analysis/SkinCheck.hpp"
#include "Engine/Modeling/Inspection/InspectionSession.hpp"
#include "Engine/Modeling/Inspection/ReferenceComparison.hpp"
#include "Engine/Modeling/Geometry/Triangulation2D.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshImprint.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Mesh/MeshTopology.hpp"
#include "Engine/Modeling/Mesh/ShadingNormals.hpp"
#include "Engine/Modeling/Mesh/SurfaceFit.hpp"
#include "Engine/Modeling/Mesh/MeshSkinning.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Raster/TextRenderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
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

bool hasCode(const Report& report, std::string_view code) {
    for (const Finding& finding : report.findings())
        if (finding.code == code) return true;
    return false;
}

void testGeometry() {
    const Vec3 a { 0, 0, 0 }, b { 1, 0, 0 }, c { 0, 1, 0 };
    require(distance(closestPointOnTriangle({ 0.2f, 0.2f, 1.0f }, a, b, c), { 0.2f, 0.2f, 0.0f }) < 1e-5f,
        "ponto mais proximo no triangulo");
    require(trianglesIntersect(a, b, c, { 0.2f, 0.2f, -1.0f }, { 0.2f, 0.2f, 1.0f }, { 0.4f, 0.1f, 0.0f }),
        "triangulos que se cruzam");
    require(!trianglesIntersect(a, b, c, { 0, 0, 1 }, { 1, 0, 1 }, { 0, 1, 1 }), "triangulos paralelos nao se cruzam");
    // Quase no mesmo plano (um quarto de grau), lado a lado com uma fresta
    // de 0,2 mm e um vertice encostado no plano do outro: vizinhos, nao se
    // cruzam (a reta de intersecao de planos assim e so ruido).
    {
        const Vec3 p0 { 0.0f, 0.0f, 0.0f }, p1 { 0.05f, 0.0f, 0.0f }, p2 { 0.0f, 0.05f, 0.0f };
        const float tilt = std::tan(0.25f * DegreesToRadians);
        const Vec3 q0 { 0.0252f, 0.0252f, 0.000001f }, q1 { 0.06f, 0.03f, 0.035f * tilt }, q2 { 0.03f, 0.06f, 0.035f * tilt };
        require(!trianglesIntersect(p0, p1, p2, q0, q1, q2), "quase coplanares lado a lado nao se cruzam");
        // O mesmo, um entrando no outro de verdade (cobrindo-se de frente).
        const Vec3 r0 { 0.01f, 0.01f, -0.0001f }, r1 { 0.04f, 0.005f, 0.0001f }, r2 { 0.005f, 0.04f, 0.0001f };
        require(trianglesIntersect(p0, p1, p2, r0, r1, r2), "quase coplanares que se cobrem e atravessam se cruzam");
    }
    const Quaternion q = frameAlong({ 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
    require(distance(axisX(q), { 0.0f, 1.0f, 0.0f }) < 1e-5f && distance(axisZ(q), { 0.0f, 0.0f, 1.0f }) < 1e-5f,
        "referencial ao longo de um eixo");
    const Transform t { { 1, 2, 3 }, Quaternion::fromAxisAngle({ 0, 0, 1 }, 0.7f) };
    require(distance((t * t.inverse()).position, {}) < 1e-5f, "transformacao vezes a inversa");
}

Mesh boxMesh(Vec3 center, Vec3 half, std::string_view part = "caixa") {
    Mesh mesh;
    MeshBuilder builder(mesh);
    builder.part(part, "tronco").material("cinza", rgb(128, 128, 128));
    builder.box({ center, {} }, half);
    return mesh;
}

void testBuilderAndTopology() {
    const Mesh box = boxMesh({}, { 0.1f, 0.2f, 0.3f });
    require(box.positions.size() == 8 && box.triangles.size() == 12, "caixa: 8 vertices e 12 triangulos");
    require(std::abs(box.signedVolume() - 0.2f * 0.4f * 0.6f) < 1e-5f, "caixa fechada e orientada para fora");
    Report clean;
    checkTopology(box, clean);
    require(clean.count(Severity::Error) == 0 && clean.count(Severity::Warning) == 0, "caixa sem problemas de topologia");

    Mesh open = box;
    open.triangles.pop_back();
    open.trianglePart.pop_back();
    open.triangleMaterial.pop_back();
    Report holes;
    checkConnectivity(open, holes);
    require(hasCode(holes, "borda_aberta"), "furo achado");
    open.parts.front().openings.push_back("tampa");
    Report declared;
    checkConnectivity(open, declared);
    require(!hasCode(declared, "borda_aberta") && hasCode(declared, "abertura_declarada"), "abertura declarada aceita");

    Mesh flipped = box;
    std::swap(flipped.triangles[0][1], flipped.triangles[0][2]);
    Report orientation;
    checkTopology(flipped, orientation);
    require(hasCode(orientation, "orientacao_trocada"), "triangulo virado achado");

    Mesh inside = box;
    MeshBuilder(inside).flipPart(0);
    Report inverted;
    checkTopology(inside, inverted);
    require(hasCode(inverted, "normais_para_dentro"), "malha do avesso achada");

    Mesh sphere;
    MeshBuilder builder(sphere);
    builder.part("bola");
    builder.ellipsoid({ {}, {} }, { 0.1f, 0.1f, 0.1f }, 24, 16);
    const float expected = 4.0f / 3.0f * Pi * 0.001f;
    require(std::abs(sphere.signedVolume() - expected) / expected < 0.06f, "elipsoide com o volume certo");
    Report sphereReport;
    checkTopology(sphere, sphereReport);
    checkConnectivity(sphere, sphereReport);
    require(sphereReport.count(Severity::Error) == 0 && !hasCode(sphereReport, "borda_aberta"), "elipsoide fechado");

    Mesh duplicated = box;
    duplicated.positions.push_back(duplicated.positions[0]);
    duplicated.triangles[0][0] = static_cast<std::uint32_t>(duplicated.positions.size() - 1);
    require(MeshBuilder(duplicated).weld(1e-6f) == 1 && duplicated.positions.size() == 8, "solda funde o repetido");
}

void testMirrorAndSymmetry() {
    Mesh mesh = boxMesh({ 0.0f, -0.3f, 0.0f }, { 0.05f, 0.05f, 0.2f }, "braco_direita");
    MeshBuilder builder(mesh);
    builder.mirrorParts(std::vector<std::string> { "braco_direita" });
    const int mirrored = mesh.findPart("braco_esquerda");
    require(mirrored >= 0, "parte espelhada criada");
    require(std::abs(mesh.partBounds(mirrored).center().y - 0.3f) < 1e-5f, "espelho no outro lado");
    require(mesh.signedVolume() > 0.0f, "espelho mantem a orientacao para fora");
    Report report;
    checkSymmetry(mesh, report);
    require(!hasCode(report, "assimetria"), "par espelhado e simetrico");
    MeshBuilder(mesh).transformPart(mirrored, { { 0.0f, 0.02f, 0.0f }, {} });
    Report shifted;
    checkSymmetry(mesh, shifted);
    require(hasCode(shifted, "assimetria"), "assimetria achada");
}

void testIntersections() {
    Mesh mesh = boxMesh({}, { 0.1f, 0.1f, 0.1f }, "a");
    MeshBuilder builder(mesh);
    builder.part("b");
    builder.box({ { 0.15f, 0.0f, 0.0f }, Quaternion::fromAxisAngle({ 0, 0, 1 }, 0.3f) }, { 0.1f, 0.1f, 0.1f });
    Report report;
    checkIntersections(mesh, mesh.positions, report);
    require(hasCode(report, "partes_se_cruzam"), "partes que se cruzam achadas");
    Mesh apart = boxMesh({}, { 0.1f, 0.1f, 0.1f }, "a");
    MeshBuilder(apart).part("b").box({ { 0.5f, 0.0f, 0.0f }, {} }, { 0.1f, 0.1f, 0.1f });
    Report none;
    checkIntersections(apart, apart.positions, none);
    require(none.findings().empty(), "caixas separadas nao se cruzam");

    // Pecas que se cruzam sao erro (ligacao e por costura, nunca sobreposicao).
    require(report.count(Severity::Error) > 0, "cruzar na ligacao e erro");
}

void testTriangulation() {
    // Quadrado com furo quadrado: o anel, so ele.
    Triangulation2DInput ring;
    ring.points = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 }, { 0.4f, 0.4f }, { 0.6f, 0.4f }, { 0.6f, 0.6f }, { 0.4f, 0.6f } };
    ring.loops = { { 0, 1, 2, 3 }, { 4, 5, 6, 7 } };
    const Triangulation2DResult result = triangulateDomain(ring);
    require(result.ok && result.triangles.size() == 8, "anel triangulado em 8 triangulos");
    double area = 0.0;
    for (const auto& tri : result.triangles) {
        const std::vector<Point2> polygon { ring.points[static_cast<std::size_t>(tri[0])], ring.points[static_cast<std::size_t>(tri[1])],
            ring.points[static_cast<std::size_t>(tri[2])] };
        const double a = polygonArea2D(polygon);
        require(a > 0.0, "triangulos anti-horarios");
        area += a;
    }
    require(std::abs(area - 0.96) < 1e-5, "area do anel");

    // Grade com uma aresta obrigatoria atravessando: ela aparece inteira.
    Triangulation2DInput grid;
    for (int j = 0; j <= 4; ++j)
        for (int i = 0; i <= 4; ++i) grid.points.push_back({ i * 0.25f, j * 0.25f });
    grid.loops = { { 0, 1, 2, 3, 4, 9, 14, 19, 24, 23, 22, 21, 20, 15, 10, 5 } };
    grid.points.push_back({ 0.1f, 0.15f });
    grid.points.push_back({ 0.9f, 0.8f });
    grid.segments = { { 25, 26 } };
    const Triangulation2DResult cut = triangulateDomain(grid);
    require(cut.ok, "grade com aresta obrigatoria: " + cut.error);
    bool found = false;
    for (const auto& tri : cut.triangles)
        for (int k = 0; k < 3; ++k) {
            const int a = tri[static_cast<std::size_t>(k)], b = tri[static_cast<std::size_t>((k + 1) % 3)];
            found = found || (a == 25 && b == 26) || (a == 26 && b == 25);
        }
    require(found, "a aresta obrigatoria existe na triangulacao");
    double gridArea = 0.0;
    for (const auto& tri : cut.triangles)
        gridArea += polygonArea2D({ grid.points[static_cast<std::size_t>(tri[0])], grid.points[static_cast<std::size_t>(tri[1])],
            grid.points[static_cast<std::size_t>(tri[2])] });
    require(std::abs(gridArea - 1.0) < 1e-5, "a grade inteira coberta");
}

void testStitchAndImprint() {
    // Aneis de 8 e 13 vertices costurados: tubo fechado, orientado para fora.
    Mesh tube;
    MeshBuilder builder(tube);
    builder.part("tubo").root();
    const Quaternion up = frameAlong({ 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f });
    const std::vector<std::uint32_t> lower = builder.ellipseRing({ { 0, 0, 0 }, up }, 0.05f, 0.05f, 8);
    const std::vector<std::uint32_t> upper = builder.ellipseRing({ { 0, 0, 0.1f }, up }, 0.04f, 0.03f, 13, 0.4f);
    stitchLoops(builder, lower, upper);
    require(tube.triangles.size() == 21, "costura de 8 com 13: 21 triangulos");
    require(boundaryLoops(tube).size() == 2, "a faixa tem as duas voltas de borda");
    builder.cap(lower, true, Cap::Flat);
    builder.cap(upper, false, Cap::Flat);
    Report tubeReport;
    checkTopology(tube, tubeReport);
    checkConnectivity(tube, tubeReport);
    require(tubeReport.count(Severity::Error) == 0 && tube.signedVolume() > 0.0f, "tubo costurado fechado e para fora");

    // Uma trava saindo da sola: o furo recortado na face de baixo e a trava
    // continuando dele, vertice com vertice.
    Mesh sole = boxMesh({ 0.0f, -0.1f, 0.01f }, { 0.05f, 0.03f, 0.01f }, "sola_direita");
    sole.parts.front().root = true;
    std::vector<Vec3> footprint;
    for (int i = 0; i < 5; ++i) {
        // Horario visto de cima: anti-horario visto de baixo (o eixo da trava).
        const float angle = -2.0f * Pi * static_cast<float>(i) / 5.0f;
        footprint.push_back({ 0.01f + 0.008f * std::cos(angle), -0.1f + 0.008f * std::sin(angle), 0.003f });
    }
    const std::vector<ImprintLoop> loops { { footprint } };
    ImprintSettings settings;
    settings.viewDirection = { 0.0f, 0.0f, -1.0f };
    const ImprintResult imprint = imprintLoops(sole, sole.partTriangles(0), loops, settings);
    require(imprint.ok && imprint.loops.size() == 1 && imprint.inputVertices[0].size() == 5
            && imprint.loops[0].size() >= 5, "furo recortado: " + imprint.error);
    for (const std::uint32_t v : imprint.loops[0]) require(std::abs(sole.positions[v].z) < 1e-6f, "o furo assenta na face");
    MeshBuilder soleBuilder(sole);
    soleBuilder.part("trava_direita").connectTo("sola_direita");
    std::vector<std::uint32_t> tip;
    for (int i = 0; i < 5; ++i) {
        const float angle = -2.0f * Pi * static_cast<float>(i) / 5.0f;
        tip.push_back(soleBuilder.vertex({ 0.01f + 0.005f * std::cos(angle), -0.1f + 0.005f * std::sin(angle), -0.012f }));
    }
    stitchLoops(soleBuilder, imprint.loops[0], tip);
    soleBuilder.cap(tip, false, Cap::Flat);
    soleBuilder.compact();
    Report studReport;
    checkTopology(sole, studReport);
    checkConnectivity(sole, studReport);
    checkIntersections(sole, sole.positions, studReport);
    // (A face da caixa tem 2 triangulos enormes: o corte deixa agulhas
    // planas, que nao aparecem; o resto tem que estar limpo.)
    require(studReport.count(Severity::Error) == 0 && !hasCode(studReport, "partes_se_cruzam")
            && !hasCode(studReport, "borda_aberta") && !hasCode(studReport, "costura_sem_solda"),
        "trava ligada de verdade:\n" + studReport.text());
    require(sole.signedVolume() > 0.0f, "sola com trava fechada e para fora");

    // O espelho leva a costura (os vertices divididos continuam divididos).
    soleBuilder.mirrorParts(std::vector<std::string> { "sola_direita", "trava_direita" });
    const int mirrored = sole.findPart("trava_esquerda");
    require(mirrored >= 0 && sole.parts[static_cast<std::size_t>(mirrored)].connectedTo == std::vector<std::string> { "sola_esquerda" },
        "o espelho leva a costura com o nome trocado");
    Report mirroredReport;
    checkConnectivity(sole, mirroredReport);
    require(mirroredReport.count(Severity::Error) == 0, "o lado espelhado continua costurado:\n" + mirroredReport.text());

    // Os defeitos: peca solta, costura que falta, costura sem solda.
    Mesh loose = boxMesh({}, { 0.1f, 0.1f, 0.1f }, "corpo");
    loose.parts.front().root = true;
    MeshBuilder(loose).part("botao").connectTo("corpo").box({ { 0.3f, 0.0f, 0.0f }, {} }, { 0.01f, 0.01f, 0.01f });
    Report looseReport;
    checkConnectivity(loose, looseReport);
    require(hasCode(looseReport, "peca_solta") && hasCode(looseReport, "costura_faltando"), "peca solta e costura faltando achadas");
    Mesh unwelded = boxMesh({}, { 0.1f, 0.1f, 0.1f }, "corpo");
    unwelded.parts.front().root = true;
    MeshBuilder(unwelded).part("vizinho").box({ { 0.2f, 0.0f, 0.0f }, {} }, { 0.1f, 0.1f, 0.1f });
    Report unweldedReport;
    checkConnectivity(unwelded, unweldedReport);
    require(hasCode(unweldedReport, "costura_sem_solda"), "costura sem solda achada");
}

void testCrossSection() {
    Mesh tube;
    MeshBuilder builder(tube);
    builder.part("tubo");
    builder.tube({ 0, 0, 0 }, { 0, 0, 1 }, 0.05f, 0.05f, 48, Cap::Flat);
    const MeshSection section = sliceMesh(tube, tube.positions, Plane::fromPointNormal({ 0, 0, 0.5f }, { 0, 0, 1 }));
    require(section.loops.size() == 1 && section.loops.front().closed, "corte do tubo: um contorno fechado");
    const float circle = 2.0f * Pi * 0.05f;
    require(std::abs(section.loops.front().length - circle) / circle < 0.01f, "circunferencia do tubo");
    require(loopContains(section.loops.front(), section.plane, { 0, 0, 0.5f }), "centro dentro do contorno");
    require(!loopContains(section.loops.front(), section.plane, { 0.2f, 0, 0.5f }), "ponto fora do contorno");
}

Skeleton chainSkeleton() {
    Skeleton skeleton;
    skeleton.groundHeight = -1.0f;
    skeleton.joints.push_back({ "Pelvis", -1, { { 0, 0, 0 }, {} }, JointKind::Body, Side::Center, 0.1f });
    skeleton.joints.push_back({ "LeftThigh", 0, { { 0, 0.1f, -0.1f }, {} }, JointKind::Body, Side::Left, 0.06f });
    skeleton.joints.push_back({ "LeftShin", 1, { { 0, 0.1f, -0.5f }, {} }, JointKind::Body, Side::Left, 0.05f });
    skeleton.joints.push_back({ "LeftFoot", 2, { { 0, 0.1f, -0.9f }, {} }, JointKind::Body, Side::Left, 0.04f });
    return skeleton;
}

void testRegionsAndSkin() {
    const Skeleton skeleton = chainSkeleton();
    const std::optional<BodyRegions> regions = BodyRegions::parse(R"({ "regions": [
        { "id": "geral", "label": "Geral", "overview": true },
        { "id": "quadril", "label": "Quadril", "roots": ["Pelvis"] },
        { "id": "coxa", "label": "Coxa", "side": "esquerda", "roots": ["LeftThigh"] },
        { "id": "canela", "label": "Canela", "roots": ["LeftShin"] } ] })");
    require(regions.has_value() && regions->regions.size() == 4, "regioes lidas");
    const std::vector<int> assignment = regions->assign(skeleton);
    require(assignment[0] == 1 && assignment[1] == 2 && assignment[2] == 3 && assignment[3] == 3,
        "cada junta na regiao da raiz mais proxima");
    require(regions->validate(skeleton).empty(), "regioes validas");
    BodyRegions broken = *regions;
    broken.regions[2].roots = { "NaoExiste" };
    require(!broken.validate(skeleton).empty(), "raiz inexistente achada");

    Mesh leg;
    MeshBuilder builder(leg);
    builder.part("perna", "coxa");
    std::vector<std::vector<std::uint32_t>> rings;
    for (int i = 0; i <= 8; ++i) {
        const float z = -0.1f - 0.8f * static_cast<float>(i) / 8.0f;
        rings.push_back(builder.ellipseRing({ { 0, 0.1f, z }, frameAlong({ 0, 0, -1 }, { 1, 0, 0 }) }, 0.05f, 0.05f, 12));
    }
    builder.loft(rings, Cap::Flat, Cap::Flat);
    computeAutoSkin(leg, skeleton, &*regions, assignment);
    require(leg.hasSkin(), "skin automatico criado");
    Report skinReport;
    checkSkin(leg, skeleton, &*regions, assignment, skinReport);
    require(skinReport.count(Severity::Error) == 0, "skin automatico sem erros");
    float kneeShare = 0.0f;
    for (std::size_t v = 0; v < leg.positions.size(); ++v)
        if (std::abs(leg.positions[v].z + 0.5f) < 0.06f) kneeShare = std::max(kneeShare, weightFor(leg.skin[v], 2));
    require(kneeShare > 0.2f, "o joelho divide o peso entre coxa e canela");

    SkeletonPose bent = bindPose(skeleton);
    const Quaternion bend = Quaternion::fromAxisAngle({ 0, 1, 0 }, 0.8f);
    const Vec3 knee = bent.joints[2].position;
    bent.joints[2].orientation = bend;
    bent.joints[3] = { knee + bend.rotate(bent.joints[3].position - knee), bend };
    const std::vector<Vec3> posed = leg.posedPositions(skinningTransforms(skeleton, bent));
    require(posed.size() == leg.positions.size() && distance(posed.back(), leg.positions.back()) > 0.01f,
        "a pose deforma a malha");
    Report deformation;
    const std::vector<PoseSample> poses { { "dobra", bent } };
    checkDeformation(leg, skeleton, poses, deformation);
    require(!hasCode(deformation, "virou_pose"), "joelho dobrado sem triangulo virado");

    const Report validation = validateModel(leg, &skeleton, &*regions, assignment);
    require(validation.count(Severity::Error) == 0, "perna valida (" + validation.text(5) + ")");
}

// As dobras de teste: o eixo preso ao pai (o joelho dobra no plano da perna
// mesmo com a coxa aberta), a dobradica de uma junta e a mistura de poses (o
// giro de cada junta no pai).
void testBendTools() {
    const Skeleton skeleton = chainSkeleton();
    const int thigh = 1, shin = 2, foot = 3;
    const float quarter = 1.5707963f;
    const auto position = [](const SkeletonPose& pose, int joint) { return pose.joints[static_cast<std::size_t>(joint)].position; };
    // A coxa aberta 90 graus para o lado (a perna vai para +y) e o joelho 90
    // em volta do y: no mundo, o y e o proprio eixo da canela (so torce);
    // preso ao pai, o eixo gira com a coxa e o joelho dobra.
    std::array<JointBend, 2> bends { JointBend { thigh, { 1, 0, 0 }, quarter }, JointBend { shin, { 0, 1, 0 }, quarter } };
    const SkeletonPose loose = bendPose(skeleton, bends);
    bends[1].followParent = true;
    const SkeletonPose held = bendPose(skeleton, bends);
    const Vec3 thighDirection = safeNormalized(position(held, shin) - position(held, thigh), {});
    require(dot(safeNormalized(position(loose, foot) - position(loose, shin), {}), thighDirection) > 0.99f, "no mundo o joelho so torce");
    require(std::abs(dot(safeNormalized(position(held, foot) - position(held, shin), {}), thighDirection)) < 0.01f,
        "preso ao pai, o joelho dobra 90 graus");
    // A dobradica: a perna reta dobra a canela para tras (-x); dobrada, o
    // plano dela.
    const SkeletonPose bind = bindPose(skeleton);
    const Vec3 hinge = jointHingeAxis(skeleton, bind, shin);
    require(std::abs(hinge.y) > 0.99f, "a dobradica do joelho reto e o lado");
    const std::array<JointBend, 1> knee { JointBend { shin, hinge, 0.6f } };
    const SkeletonPose bent = bendPose(skeleton, knee);
    require(position(bent, foot).x < position(bind, foot).x - 0.1f, "o joelho leva o pe para tras");
    const Vec3 again = jointHingeAxis(skeleton, bent, shin);
    require(dot(again, hinge) > 0.99f, "dobrado, a dobradica e a normal do plano da perna");
    // A mistura: inteira chega a pose alvo; pela metade, a metade do giro.
    const std::array<JointBend, 2> target { JointBend { thigh, { 1, 0, 0 }, 0.7f }, JointBend { shin, { 0, 1, 0 }, 1.2f, true } };
    const SkeletonPose goal = bendPose(skeleton, target);
    const std::array<int, 2> legs { thigh, shin };
    const SkeletonPose full = bendPose(skeleton, blendJointBends(skeleton, bind, goal, legs, 1.0f));
    for (const int joint : { thigh, shin, foot })
        require(distance(position(full, joint), position(goal, joint)) < 1e-4f, "a mistura inteira chega ao alvo");
    const SkeletonPose half = bendPose(skeleton, blendJointBends(skeleton, bind, goal, legs, 0.5f));
    const auto kneeAngle = [&](const SkeletonPose& pose) {
        const Vec3 a = safeNormalized(position(pose, shin) - position(pose, thigh), {}), b = safeNormalized(position(pose, foot) - position(pose, shin), {});
        return std::acos(std::clamp(dot(a, b), -1.0f, 1.0f));
    };
    require(std::abs(kneeAngle(half) - 0.5f * kneeAngle(goal)) < 0.01f, "pela metade, o joelho dobra a metade");
}

// O encaixe numa referencia: a semelhanca exata por Horn, o ICP achando de
// volta um alinhamento conhecido a partir de um chute errado e a projecao na
// superficie ao longo da normal.
void testSurfaceFit() {
    // Horn: pontos levados por uma semelhanca conhecida voltam a ela.
    Similarity truth;
    truth.scale = 1.7f;
    truth.rotation = Quaternion::fromAxisAngle(safeNormalized({ 0.3f, 0.8f, 0.5f }, {}), 0.6f);
    truth.translation = { 0.1f, -0.2f, 0.3f };
    std::vector<Vec3> source, target;
    for (int i = 0; i < 12; ++i) {
        const Vec3 p { std::sin(1.3f * static_cast<float>(i)), std::cos(0.7f * static_cast<float>(i)), 0.1f * static_cast<float>(i) - 0.5f };
        source.push_back(p);
        target.push_back(truth.apply(p));
    }
    const std::optional<Similarity> fitted = fitSimilarity(source, target);
    require(fitted.has_value(), "Horn achou a semelhanca");
    for (std::size_t i = 0; i < source.size(); ++i) require(distance(fitted->apply(source[i]), target[i]) < 1e-4f, "Horn: cada ponto no lugar");
    require(std::abs(fitted->scale - 1.7f) < 1e-4f, "Horn: a escala");
    const Similarity back = fitted->inverse();
    require(distance(back.apply(target[3]), source[3]) < 1e-4f, "a inversa desfaz");
    require(!fitSimilarity(std::span<const Vec3>(source.data(), 2), std::span<const Vec3>(target.data(), 2)), "com dois pontos, nada");

    // ICP: uma referencia simetrica (um ovo de aneis) e os pontos dela levados
    // por uma semelhanca simetrica (escala, giro em y, translacao em x e z);
    // a partir de um chute errado, o alinhamento acha a semelhanca.
    Mesh egg;
    MeshBuilder builder(egg);
    builder.part("ovo");
    std::vector<std::vector<std::uint32_t>> rings;
    for (int i = 1; i < 12; ++i) {
        const float a = 3.14159265f * static_cast<float>(i) / 12.0f;
        rings.push_back(builder.ellipseRing({ { 0.02f * std::sin(2.0f * a), 0.0f, -std::cos(a) * 0.12f }, frameAlong({ 0, 0, 1 }, { 1, 0, 0 }) },
            0.09f * std::sin(a) * (1.0f + 0.25f * std::cos(a)), 0.07f * std::sin(a), 16));
    }
    builder.loft(rings, Cap::Flat, Cap::Flat);
    Similarity placed;
    placed.scale = 0.9f;
    placed.rotation = Quaternion::fromAxisAngle({ 0, 1, 0 }, 0.12f);
    placed.translation = { 0.03f, 0.0f, 1.6f };
    std::vector<Vec3> points;
    for (std::size_t v = 0; v < egg.positions.size(); v += 2) points.push_back(placed.apply(egg.positions[v]));
    Similarity guess = placed;
    guess.scale = 0.95f;
    guess.rotation = Quaternion::fromAxisAngle({ 0, 1, 0 }, 0.06f);
    guess.translation = { 0.04f, 0.0f, 1.61f };
    AlignSettings settings;
    settings.trim = 0.0f;
    const AlignResult aligned = alignReference(egg.positions, egg.triangles, points, guess, settings);
    require(aligned.ok && aligned.rms < 5e-4f, "o ICP encaixa (rms " + std::to_string(aligned.rms) + ")");
    for (const std::size_t v : { std::size_t(0), std::size_t(37), std::size_t(90) })
        require(distance(aligned.transform.apply(egg.positions[v]), placed.apply(egg.positions[v])) < 2e-3f, "o ICP acha o alinhamento");

    // A projecao: na reta da normal (os dois sentidos) e, sem acerto, nada
    // alem da distancia.
    Mesh plate;
    MeshBuilder plateBuilder(plate);
    plateBuilder.part("placa");
    const std::uint32_t a = plateBuilder.vertex({ -1, -1, 0 }), b = plateBuilder.vertex({ 1, -1, 0 }), c = plateBuilder.vertex({ 1, 1, 0 }),
                        d = plateBuilder.vertex({ -1, 1, 0 });
    plateBuilder.triangle(a, b, c);
    plateBuilder.triangle(a, c, d);
    const TriangleBvh bvh(plate.positions, plate.triangles);
    const std::optional<Vec3> down = projectOntoSurface(bvh, { 0.1f, 0.2f, 0.05f }, { 0, 0, 1 }, 0.2f);
    require(down && distance(*down, { 0.1f, 0.2f, 0.0f }) < 1e-6f, "a projecao acha a placa atras do ponto");
    require(!projectOntoSurface(bvh, { 0.1f, 0.2f, 0.5f }, { 0, 0, 1 }, 0.2f), "longe demais, nada");
    const std::vector<Vec3> normals = vertexNormals(plate);
    require(normals[a].z > 0.999f && normals[c].z > 0.999f, "as normais dos vertices da placa");
}

// Triangulo virado a vista e erro; virado dentro de uma prega (fechado entre
// duas peles) e so observacao.
void testFoldVisibility() {
    Skeleton skeleton;
    skeleton.joints.push_back({ "Raiz", -1, { { 0, 0, 0 }, {} }, JointKind::Body, Side::Center, 0.05f });
    skeleton.joints.push_back({ "Ponta", 0, { { 0, 0, 0 }, {} }, JointKind::Body, Side::Center, 0.05f });
    const auto weigh = [](Mesh& mesh, std::uint32_t v, std::uint16_t joint) {
        if (mesh.skin.size() < mesh.positions.size()) mesh.skin.resize(mesh.positions.size());
        mesh.skin[v] = {};
        mesh.skin[v].joints[0] = joint;
        mesh.skin[v].weights[0] = 1.0f;
    };
    // Um quadrado aberto: o canto puxado pela "Ponta" passa por cima do outro
    // triangulo e o vira, a vista.
    Mesh patch;
    MeshBuilder open(patch);
    open.part("placa");
    const std::uint32_t a = open.vertex({ 0, 0, 0 }), b = open.vertex({ 0.1f, 0, 0 }), c = open.vertex({ 0.1f, 0.1f, 0 }),
                        d = open.vertex({ 0, 0.1f, 0 });
    open.triangle(a, b, c);
    open.triangle(a, c, d);
    for (const std::uint32_t v : { a, b, d }) weigh(patch, v, 0);
    weigh(patch, c, 1);
    SkeletonPose pulled = bindPose(skeleton);
    pulled.joints[1].position = { -0.15f, -0.15f, 0.0f };
    const std::vector<PoseSample> pull { { "puxado", pulled } };
    Report visible;
    checkDeformation(patch, skeleton, pull, visible);
    require(hasCode(visible, "virou_pose") && !hasCode(visible, "prega_pose"), "virado a vista e erro");
    // Uma caixa fechada com um triangulo virado la dentro (um anel interno
    // empurrado pela "Ponta"): nenhum raio de fora chega nele.
    Mesh box = boxMesh({}, { 0.2f, 0.2f, 0.2f });
    box.skin.assign(box.positions.size(), {});
    for (std::size_t v = 0; v < box.positions.size(); ++v) weigh(box, static_cast<std::uint32_t>(v), 0);
    MeshBuilder inner(box);
    inner.part("dentro");
    const std::uint32_t e = inner.vertex({ -0.05f, -0.05f, 0 }), f = inner.vertex({ 0.05f, -0.05f, 0 }),
                        g = inner.vertex({ 0.05f, 0.05f, 0 }), h = inner.vertex({ -0.05f, 0.05f, 0 });
    inner.triangle(e, f, g);
    inner.triangle(e, g, h);
    for (const std::uint32_t v : { e, f, h }) weigh(box, v, 0);
    weigh(box, g, 1);
    // O canto atravessa a aresta de baixo: um triangulo vira sem encolher.
    pulled.joints[1].position = { 0.0f, -0.2f, 0.0f };
    const std::vector<PoseSample> fold { { "prega", pulled } };
    DeformationSettings quiet;
    quiet.intersections = false;
    Report hidden;
    checkDeformation(box, skeleton, fold, hidden, quiet);
    require(!hasCode(hidden, "virou_pose") && hasCode(hidden, "prega_pose") && hidden.count(Severity::Error) == 0,
        "virado escondido dentro da peca e so observacao");
    // Uma caixa fechada com o meio da tampa puxado atraves dela ate embaixo:
    // a tampa vira uma piramide para fora, do avesso, sem girar osso nenhum
    // (a normal esperada continua para cima e a de cada triangulo quase de
    // lado) - a dobra contra as paredes e o verso a vista acusam.
    Mesh lid;
    MeshBuilder closed(lid);
    closed.part("caixa");
    std::array<std::uint32_t, 8> p {};
    for (int k = 0; k < 8; ++k)
        p[static_cast<std::size_t>(k)] = closed.vertex({ (k == 1 || k == 2 || k == 5 || k == 6) ? 0.1f : -0.1f,
            (k == 2 || k == 3 || k == 6 || k == 7) ? 0.1f : -0.1f, k < 4 ? -0.1f : 0.1f });
    const std::uint32_t middle = closed.vertex({ 0.0f, 0.0f, 0.1f });
    closed.quad(p[0], p[3], p[2], p[1]);
    closed.quad(p[1], p[2], p[6], p[5]);
    closed.quad(p[3], p[0], p[4], p[7]);
    closed.quad(p[2], p[3], p[7], p[6]);
    closed.quad(p[0], p[1], p[5], p[4]);
    for (int k = 0; k < 4; ++k) closed.triangle(p[static_cast<std::size_t>(4 + k)], p[static_cast<std::size_t>(4 + (k + 1) % 4)], middle);
    for (std::size_t v = 0; v < lid.positions.size(); ++v) weigh(lid, static_cast<std::uint32_t>(v), 0);
    weigh(lid, middle, 1);
    require(lid.validateLayout().empty(), "a caixa com a tampa em leque");
    pulled.joints[1].position = { 0.0f, 0.0f, -1.1f };
    const std::vector<PoseSample> through { { "atravessada", pulled } };
    Report inverted;
    checkDeformation(lid, skeleton, through, inverted, quiet);
    require(hasCode(inverted, "virou_pose"), "a tampa do avesso por fora da caixa e erro");
    pulled.joints[1].position = { 0.0f, 0.0f, 0.0f };
    const std::vector<PoseSample> still { { "parada", pulled } };
    Report rest;
    checkDeformation(lid, skeleton, still, rest, quiet);
    require(!hasCode(rest, "virou_pose") && !hasCode(rest, "prega_pose"), "parada, a caixa nao tem avesso");
}

void testRenderingAndCamera() {
    const Mesh box = boxMesh({}, { 0.2f, 0.2f, 0.2f });
    OrbitCamera camera;
    camera.yaw = 0.4f;
    camera.pitch = 0.3f;
    frameBox(camera, box.bounds(), 1.0f, 1.1f);
    const Projector projector(camera, 200, 200);
    for (int i = 0; i < 8; ++i) {
        const ScreenPoint p = projector.project({ i & 1 ? 0.2f : -0.2f, i & 2 ? 0.2f : -0.2f, i & 4 ? 0.2f : -0.2f });
        require(p.visible && p.x >= 0 && p.x <= 200 && p.y >= 0 && p.y <= 200, "enquadramento cabe na imagem");
    }
    InspectionScene scene;
    scene.mesh = &box;
    InspectionStyle style;
    style.hud = false;
    style.gizmo = false;
    style.scaleBar = false;
    style.supersample = 2;
    const Image image = renderInspection(scene, style, camera, 200, 200);
    require(image.width() == 200 && image.height() == 200, "tamanho da imagem");
    const Color center = image.pixel(100, 100), corner = image.pixel(2, 2);
    require(center.r > corner.r + 0.2f, "a caixa aparece no centro e o fundo no canto");

    const std::filesystem::path path = std::filesystem::temp_directory_path() / "matter_modeling_test.png";
    require(writePng(image, path.string()), "PNG gravado");
    const std::optional<Image> read = readImage(path.string());
    require(read && read->width() == 200 && std::abs(read->pixel(100, 100).r - center.r) < 0.01f, "PNG lido de volta");
    std::filesystem::remove(path);

    require(TextRenderer::standard().loaded(), "fonte dos rotulos carregada");
    require(TextRenderer::standard().measure("Mão direita", 16.0f).width > 40.0f, "texto com acento medido");

    // Silhueta de uma referencia sintetica.
    Image reference(120, 200, rgb(0, 0, 0));
    reference.fillRect(40, 20, 80, 190, rgb(220, 220, 220));
    const Silhouette figure = extractSilhouette(reference);
    require(figure.valid() && figure.top == 20 && figure.bottom == 189 && figure.left == 40 && figure.right == 79,
        "silhueta da referencia");
}

// O sombreamento liso: as normais dos cantos por grupo (a quina separa), a
// mistura com a face, a pose girando as normais e o render liso.
void testShadingNormals() {
    // A caixa: todas as arestas a 90 graus - cada canto fica com a face.
    const Mesh box = boxMesh({}, { 0.2f, 0.2f, 0.2f });
    const ShadingNormals boxShading(box, 45.0f);
    require(boxShading.groupCount() == 24, "a caixa: um grupo por canto de cada face (" + std::to_string(boxShading.groupCount()) + ")");
    std::vector<Vec3> corners;
    boxShading.compute(box.positions, box.triangles, 0.0f, corners);
    for (std::size_t t = 0; t < box.triangles.size(); ++t)
        for (std::size_t k = 0; k < 3; ++k) require(distance(corners[t * 3 + k], box.normal(t)) < 1e-4f, "na caixa, a normal da face");
    // O tubo de 16 lados com tampas: o lado liso (a normal radial do
    // vertice), as tampas chapadas (90 graus: quina).
    Mesh tube;
    MeshBuilder(tube).part("tubo").tube({ 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 0.2f, 0.2f, 16, Cap::Flat);
    const ShadingNormals tubeShading(tube, 45.0f);
    tubeShading.compute(tube.positions, tube.triangles, 0.0f, corners);
    int side = 0, cap = 0;
    for (std::size_t t = 0; t < tube.triangles.size(); ++t) {
        const Vec3 face = tube.normal(t);
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec3 p = tube.positions[tube.triangles[t][k]], c = corners[t * 3 + k];
            if (std::abs(face.z) > 0.9f) {
                require(distance(c, face) < 1e-4f, "a tampa chapada");
                ++cap;
            } else {
                const Vec3 radial = safeNormalized({ p.x, p.y, 0.0f }, { 1.0f, 0.0f, 0.0f });
                require(dot(c, radial) > 0.999f && std::abs(c.z) < 1e-3f, "o lado liso: a normal radial do vertice");
                ++side;
            }
        }
    }
    require(side > 0 && cap > 0, "o tubo tem lado e tampas");
    // A mistura: 1 volta a face; no meio, entre as duas.
    std::vector<Vec3> faceted, half;
    tubeShading.compute(tube.positions, tube.triangles, 1.0f, faceted);
    tubeShading.compute(tube.positions, tube.triangles, 0.5f, half);
    for (std::size_t t = 0; t < tube.triangles.size(); ++t)
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec3 face = tube.normal(t);
            require(distance(faceted[t * 3 + k], face) < 1e-4f, "facetado: a normal da face");
            require(dot(half[t * 3 + k], face) >= dot(corners[t * 3 + k], face) - 1e-5f, "o meio-termo mais perto da face que o liso");
        }
    // A pose: o tubo deitado (90 graus em volta de x); as normais giram junto.
    std::vector<Vec3> lying(tube.positions.size());
    for (std::size_t v = 0; v < lying.size(); ++v) lying[v] = { tube.positions[v].x, -tube.positions[v].z, tube.positions[v].y };
    std::vector<Vec3> turned;
    tubeShading.compute(lying, tube.triangles, 0.0f, turned);
    for (std::size_t i = 0; i < turned.size(); ++i)
        require(distance(turned[i], { corners[i].x, -corners[i].z, corners[i].y }) < 1e-4f, "a pose gira as normais junto");
    // O render: no lado do tubo a luz muda dentro de cada face (liso); no
    // facetado cada face e uma cor so.
    OrbitCamera camera;
    camera.yaw = 0.3f;
    camera.pitch = 0.1f;
    frameBox(camera, tube.bounds(), 1.0f, 1.2f);
    InspectionScene scene;
    scene.mesh = &tube;
    InspectionStyle style;
    style.hud = false;
    style.gizmo = false;
    style.scaleBar = false;
    style.grid = false;
    style.supersample = 1;
    const auto shades = [&](float facet) {
        style.facet = facet;
        const Image image = renderInspection(scene, style, camera, 200, 200);
        std::set<int> values;
        for (int x = 0; x < 200; ++x) {
            const Color c = image.pixel(x, 100);
            values.insert(static_cast<int>(std::lround(c.r * 255.0f)));
        }
        return values.size();
    };
    const std::size_t flat = shades(1.0f), smooth = shades(0.0f);
    require(smooth > 2 * flat, "o render liso tem a luz continua (" + std::to_string(smooth) + " tons contra " + std::to_string(flat) + ")");
}

void testSession() {
    auto model = std::make_shared<InspectionModel>();
    model->skeleton = chainSkeleton();
    model->regions = *BodyRegions::parse(R"({ "regions": [ { "id": "geral", "label": "Geral", "overview": true },
        { "id": "coxa", "label": "Coxa", "roots": ["LeftThigh"] } ] })");
    model->jointRegion = model->regions.assign(model->skeleton);
    InspectionSession session;
    session.setModel(model);
    require(session.render(320, 240), "primeiro render");
    require(!session.render(320, 240), "sem mudanca, sem render");
    const float yaw = session.camera().yaw;
    session.orbit(50.0f, 0.0f);
    require(std::abs(session.camera().yaw - yaw) > 0.1f, "arrastar gira a camera");
    require(session.render(320, 240), "girar pede render");
    session.selectRegion(1);
    for (int i = 0; i < 120; ++i) session.update(1.0f / 60.0f);
    require(!session.animating() && session.selectedRegion() == 1, "transicao ate a regiao termina");
    const OrbitCamera expected = frameRegion(session.scene(), 1, 320.0f / 240.0f);
    require(distance(session.camera().target, expected.target) < 1e-3f
            && std::abs(session.camera().distance - expected.distance) < 1e-3f,
        "camera no enquadramento da regiao");
    require(regionBounds(session.scene(), 1).contains(session.camera().target), "alvo dentro da regiao");
    session.editStyle().surface = SurfaceMode::XRay;
    require(session.render(320, 240), "mudar o estilo pede render");
}

} // namespace

int main() {
    try {
        testGeometry();
        testBuilderAndTopology();
        testMirrorAndSymmetry();
        testIntersections();
        testTriangulation();
        testStitchAndImprint();
        testCrossSection();
        testRegionsAndSkin();
        testBendTools();
        testSurfaceFit();
        testFoldVisibility();
        testRenderingAndCamera();
        testShadingNormals();
        testSession();
    } catch (const std::exception& exception) {
        std::cerr << "MatterModeling test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterModeling tests passed (" << checks << " checagens)\n";
    return 0;
}
