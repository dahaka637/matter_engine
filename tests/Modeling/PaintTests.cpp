// Testes do sistema de pintura (Paint/): as formas vetoriais e a cobertura
// pela distancia exata, as fontes (a de bloco medida na referencia e uma
// TrueType), as tintas e as camadas, os espacos na superficie (o plano e a
// volta desenrolada, com o corte da volta), os caminhos de SVG, as imagens e o
// desenho em JSON. Linkado so contra MatterModeling.

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Paint/PaintDesign.hpp"
#include "Engine/Modeling/Paint/PaintFont.hpp"
#include "Engine/Modeling/Paint/PaintLayer.hpp"
#include "Engine/Modeling/Paint/PaintSpace.hpp"
#include "Engine/Modeling/Paint/Shape2D.hpp"
#include "Engine/Modeling/Paint/SvgPath.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
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

bool near(float a, float b, float tolerance) { return std::abs(a - b) <= tolerance; }

void testShapes() {
    const Shape2D square = rectangleShape({ 0.0f, 0.0f }, { 1.0f, 1.0f });
    const PreparedShape box(square);
    require(box.contains({ 0.5f, 0.5f }) && !box.contains({ 1.5f, 0.5f }), "o quadrado: dentro e fora");
    require(near(box.signedDistance({ 0.5f, 0.5f }), -0.5f, 1e-5f) && near(box.signedDistance({ 2.0f, 0.5f }), 1.0f, 1e-5f),
        "a distancia com sinal");
    require(near(box.coverage({ 0.5f, 0.5f }, 0.1f), 1.0f, 1e-5f) && near(box.coverage({ 1.5f, 0.5f }, 0.1f), 0.0f, 1e-5f),
        "a cobertura longe da borda");
    require(near(box.coverage({ 1.0f, 0.5f }, 0.1f), 0.5f, 1e-3f) && near(box.coverage({ 1.025f, 0.5f }, 0.1f), 0.25f, 1e-3f)
            && near(box.coverage({ 0.975f, 0.5f }, 0.1f), 0.75f, 1e-3f),
        "a cobertura na borda, pelo tamanho do pixel");
    // Duas formas sobrepostas na mesma camada: sem costura por dentro.
    Shape2D both = square;
    both.add(rectangleShape({ 0.5f, 0.0f }, { 1.5f, 1.0f }));
    const PreparedShape joined(both);
    require(near(joined.coverage({ 1.0f, 0.5f }, 0.1f), 1.0f, 1e-5f) && near(joined.coverage({ 0.52f, 0.5f }, 0.1f), 1.0f, 1e-5f),
        "a uniao sem costura onde as formas se cruzam");
    require(near(joined.coverage({ 1.5f, 0.5f }, 0.1f), 0.5f, 1e-3f), "a borda da uniao");
    // O furo (contorno no sentido contrario) e o espelho que o mantem furo.
    Shape2D ring = square;
    std::vector<Point2> hole = rectangleShape({ 0.4f, 0.4f }, { 0.6f, 0.6f }).contours.front();
    std::reverse(hole.begin(), hole.end());
    ring.contours.push_back(hole);
    require(!ring.contains({ 0.5f, 0.5f }) && ring.contains({ 0.2f, 0.5f }), "o furo");
    const PreparedShape mirroredRing(ring.transformed(Affine2::scaling(-1.0f, 1.0f)));
    require(!mirroredRing.contains({ -0.5f, 0.5f }) && mirroredRing.contains({ -0.2f, 0.5f }), "o espelho mantem o furo");
    require(near(mirroredRing.coverage({ -0.4f, 0.5f }, 0.1f), 0.5f, 1e-3f), "a borda do furo espelhado");
    // Chanfro, traco e o contorno engordado.
    const PreparedShape chamfered(rectangleShape({ 0.0f, 0.0f }, { 1.0f, 1.0f }, 0.2f));
    require(!chamfered.contains({ 0.05f, 0.05f }) && chamfered.contains({ 0.5f, 0.05f }) && chamfered.contains({ 0.2f, 0.2f }),
        "o canto chanfrado");
    const PreparedShape rounded(rectangleShape({ 0.0f, 0.0f }, { 1.0f, 1.0f }, 0.0f, 0.3f));
    require(!rounded.contains({ 0.03f, 0.03f }) && rounded.contains({ 0.15f, 0.15f }), "o canto arredondado");
    const PreparedShape line(strokeShape({ { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f } }, 0.2f));
    require(line.contains({ 0.5f, 0.09f }) && !line.contains({ 0.5f, 0.11f }) && line.contains({ 0.95f, 0.5f })
            && line.contains({ 1.05f, -0.05f }),
        "a linha grossa com a quina");
    const PreparedShape frame(strokeShape({ { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f } }, 0.1f, true));
    require(!frame.contains({ 0.5f, 0.5f }) && frame.contains({ 0.5f, 0.02f }) && frame.contains({ 1.03f, 0.5f }), "o anel fechado");
    require(near(box.coverage({ 1.05f, 0.5f }, 0.02f, 0.1f), 1.0f, 1e-5f) && near(box.coverage({ 1.2f, 0.5f }, 0.02f, 0.1f), 0.0f, 1e-5f),
        "a forma engordada (o contorno)");
}

void testFonts() {
    const PaintFont& block = PaintFont::block();
    for (char digit = '0'; digit <= '9'; ++digit) {
        const PaintFont::Glyph* glyph = block.glyph(static_cast<char32_t>(digit));
        require(glyph != nullptr && !glyph->shape.empty(), std::string("o algarismo ") + digit);
        const Box2 bounds = glyph->shape.bounds();
        require(bounds.minimum.x >= -1e-5f && bounds.maximum.x <= glyph->advance + 1e-5f && near(bounds.minimum.y, 0.0f, 1e-5f)
                && near(bounds.maximum.y, 1.0f, 1e-5f),
            std::string("o algarismo na caixa dele: ") + digit);
    }
    // O "10" medido na referencia: 0,42 + 0,072 + 0,598 de largura.
    float width = 0.0f;
    const PreparedShape ten(block.layout("10", 0.072f, &width));
    require(near(width, 1.09f, 1e-4f), "a largura do 10 (" + std::to_string(width) + ")");
    require(ten.contains({ 0.2f, 0.5f }) && ten.contains({ 0.05f, 0.8f }) && !ten.contains({ 0.05f, 0.6f }) && ten.contains({ 0.05f, 0.1f }),
        "o 1: a haste, a bandeira, o vao embaixo dela e a base");
    require(!ten.contains({ 0.492f + 0.3f, 0.5f }) && ten.contains({ 0.492f + 0.1f, 0.5f }) && !ten.contains({ 0.492f + 0.01f, 0.99f }),
        "o 0: o furo, o traco e o canto chanfrado");
    // Uma TrueType: a altura das maiusculas 1 e o texto andando.
    std::string error;
    const std::optional<PaintFont> roboto = PaintFont::loadTrueType(MATTERPAINT_FONT_PATH, &error);
    require(roboto.has_value(), "a Roboto: " + error);
    const PaintFont::Glyph* h = roboto->glyph(U'H');
    require(h != nullptr && near(h->shape.bounds().maximum.y, 1.0f, 1e-3f), "o H com altura 1");
    float textWidth = 0.0f;
    const PreparedShape name(roboto->layout("SILVA", 0.05f, &textWidth));
    // A haste do I (depois do S e do espaco) e o furo do A.
    require(textWidth > 2.5f && textWidth < 5.0f && name.contains({ 1.1f, 0.5f }) && !name.contains({ 1.1f, 1.2f }), "o nome em TrueType");
    const PaintFont::Glyph* a = roboto->glyph(U'A');
    require(a != nullptr && a->shape.contours.size() == 2 && !PreparedShape(a->shape).contains({ 0.47f, 0.45f }), "o furo do A");
    require(!PaintFont::loadTrueType("/nao/existe.ttf", &error) && !error.empty(), "fonte que falta");
}

void testFillsAndLayers() {
    Fill linear;
    linear.kind = Fill::Kind::Linear;
    linear.from = { 0.0f, 0.0f };
    linear.to = { 1.0f, 0.0f };
    linear.stops = { { 0.0f, { 0.0f, 0.0f, 0.0f, 1.0f } }, { 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f } } };
    require(near(linear.at({ 0.5f, 3.0f }).r, 0.5f, 1e-5f) && near(linear.at({ -1.0f, 0.0f }).r, 0.0f, 1e-5f), "o degrade linear");
    Fill radial;
    radial.kind = Fill::Kind::Radial;
    radial.radius = { 2.0f, 1.0f };
    radial.stops = linear.stops;
    require(near(radial.at({ 2.0f, 0.0f }).r, 1.0f, 1e-5f) && near(radial.at({ 0.0f, 0.5f }).r, 0.5f, 1e-5f), "o degrade radial na elipse");
    radial.angle = 1.5707963f;
    require(near(radial.at({ 0.0f, 2.0f }).r, 1.0f, 1e-4f), "o degrade radial girado");

    PaintLayer layer;
    layer.shape = PreparedShape(rectangleShape({ 0.0f, 0.0f }, { 1.0f, 1.0f }));
    layer.fill.color = { 1.0f, 0.0f, 0.0f, 1.0f };
    layer.outline = 0.1f;
    layer.outlineColor = { 1.0f, 1.0f, 1.0f, 1.0f };
    const Color base { 0.2f, 0.2f, 0.2f, 1.0f };
    const Color inside = paintOver(base, layer, { 0.5f, 0.5f }, 0.01f), ring = paintOver(base, layer, { 1.05f, 0.5f }, 0.01f),
                outside = paintOver(base, layer, { 1.3f, 0.5f }, 0.01f);
    require(near(inside.r, 1.0f, 1e-4f) && near(inside.g, 0.0f, 1e-4f), "a tinta dentro");
    require(near(ring.g, 1.0f, 1e-4f), "o contorno em volta");
    require(near(outside.r, 0.2f, 1e-5f), "fora, o que estava");
    layer.outline = 0.0f;
    layer.blend = Blend::Multiply;
    layer.fill.color = { 0.5f, 1.0f, 1.0f, 1.0f };
    const Color multiplied = paintOver({ 0.5f, 0.5f, 0.5f, 1.0f }, layer, { 0.5f, 0.5f }, 0.01f);
    require(near(multiplied.r, 0.25f, 1e-5f) && near(multiplied.g, 0.5f, 1e-5f), "multiplicando (a sombra)");
    layer.blend = Blend::Normal;
    layer.opacity = 0.5f;
    require(near(paintOver({ 0.0f, 0.0f, 0.0f, 1.0f }, layer, { 0.5f, 0.5f }, 0.01f).r, 0.25f, 1e-5f), "a opacidade");
    layer.sides = Sides::Left;
    require(layer.appliesTo(1) && !layer.appliesTo(-1), "so o lado esquerdo");
}

// Um cilindro fechado em volta do eixo z (raio "radius"), parte "pano".
Mesh cylinder(float radius, int segments) {
    Mesh mesh;
    MeshBuilder builder(mesh);
    builder.part("pano");
    std::vector<std::vector<std::uint32_t>> rings;
    for (int k = 0; k <= 4; ++k)
        rings.push_back(builder.ellipseRing({ { 0.0f, 0.0f, 0.1f * static_cast<float>(k) }, frameAlong({ 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f }) },
            radius, radius, segments));
    builder.loft(rings, Cap::Flat, Cap::Flat);
    return mesh;
}

void testSpaces() {
    const Mesh mesh = cylinder(0.1f, 32);
    const int part = mesh.findPart("pano");
    // A amostra no centro de um triangulo do lado (sem as tampas).
    const auto sampleAt = [&](float angle, float height) {
        // O triangulo cujo centro fica mais perto do ponto do lado.
        const Vec3 target { 0.1f * std::sin(angle), -0.1f * std::cos(angle), height };
        std::size_t best = 0;
        float bestDistance = 1e9f;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            if (std::abs(mesh.normal(t).z) > 0.5f) continue;
            const float d = distance(mesh.center(t), target);
            if (d < bestDistance) {
                bestDistance = d;
                best = t;
            }
        }
        SurfaceSample sample;
        sample.triangle = best;
        sample.position = mesh.center(best);
        sample.normal = mesh.normal(best);
        sample.part = part;
        sample.footprint = 0.001f;
        return sample;
    };
    UnrolledSpaceSettings settings;
    settings.parts = { part };
    settings.origin = { 0.0f, 0.0f, 0.0f };
    const PaintSpace unrolled = unrolledSpace(mesh, settings);
    // (Cada metade mede da costura dela: os pontos da metade direita.)
    for (const float degrees : { 30.0f, 60.0f, 82.0f, -45.0f, -80.0f }) {
        const SurfaceSample sample = sampleAt(degrees * 0.0174533f, 0.23f);
        const std::optional<SpacePoint> located = unrolled.locate(sample);
        require(located.has_value(), "a volta cobre o lado");
        // O arco no poligono ate o centro do triangulo: o angulo dele vezes o
        // apotema medio (a corda e quase o arco).
        const float angle = std::atan2(sample.position.x, -sample.position.y);
        const float expected = angle * std::hypot(sample.position.x, sample.position.y);
        require(near(located->point.x, expected, 0.003f),
            "o arco a partir da costura em " + std::to_string(degrees) + " graus: " + std::to_string(located->point.x) + " (esperado "
                + std::to_string(expected) + ")");
        require(near(located->point.y, sample.position.z, 1e-4f) && located->side == -1, "a altura e o lado direito");
        // O espelho (o lado esquerdo) tem o mesmo arco.
        SurfaceSample mirror = sample;
        mirror.position.y = -sample.position.y;
        float bestDistance = 1e9f;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (distance(mesh.center(t), mirror.position) < bestDistance) {
                bestDistance = distance(mesh.center(t), mirror.position);
                mirror.triangle = t;
            }
        const std::optional<SpacePoint> other = unrolled.locate(mirror);
        require(other && other->side == 1 && near(other->point.x, located->point.x, 0.002f), "o lado esquerdo e o espelho");
    }
    const PaintSpace front = planarSpace({ part }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f }, 0.5f);
    const SurfaceSample facing = sampleAt(1.5707963f, 0.25f), away = sampleAt(-1.5707963f, 0.25f);
    const std::optional<SpacePoint> projected = front.locate(facing);
    require(projected && near(projected->point.x, facing.position.y, 1e-6f) && near(projected->point.y, 0.25f, 0.06f), "o plano de frente");
    require(!front.locate(away), "o plano so nas faces viradas para ele");
}

// Um tubo de pe so na metade direita (raio 8 cm, 24 lados, centro em
// y = -0,3), com os aneis inclinados e uma fenda (um lado aberto) no trecho de
// baixo, como a barra do calcao: abaixo do fim da fenda a fatia e aberta,
// acima e fechada - e o mesmo triangulo passa pelas duas.
Mesh slitTube() {
    Mesh mesh;
    MeshBuilder builder(mesh);
    builder.part("pano");
    constexpr int sides = 24;
    std::vector<std::vector<std::uint32_t>> rings;
    for (const float height : { 0.0f, 0.0507f, 0.1f, 0.15f, 0.2f }) {
        std::vector<std::uint32_t> ring;
        for (int k = 0; k < sides; ++k) {
            const float phi = (static_cast<float>(k) + 0.5f) * 6.2831853f / sides;
            ring.push_back(builder.vertex({ 0.08f * std::sin(phi), -0.3f - 0.08f * std::cos(phi), height + 0.03f * std::cos(phi) }));
        }
        rings.push_back(std::move(ring));
    }
    for (std::size_t j = 0; j + 1 < rings.size(); ++j)
        for (int k = 0; k < sides; ++k) {
            if (j == 0 && k == 8) continue;
            const auto next = static_cast<std::size_t>((k + 1) % sides), at = static_cast<std::size_t>(k);
            builder.quad(rings[j][at], rings[j][next], rings[j + 1][next], rings[j + 1][at]);
        }
    return mesh;
}

void testRing() {
    const Mesh tube = slitTube();
    const int part = tube.findPart("pano");
    UnrolledSpaceSettings settings;
    settings.parts = { part };
    settings.origin = { 0.0f, -0.3f, 0.0f };
    settings.seamFrom = 0.08f;
    const PaintSpace ring = unrolledSpace(tube, settings);
    // O arco no poligono a partir da costura (o meio do lado de fora, -y):
    // + para a frente, de meia volta para tras a meia volta para a frente.
    const float step = 6.2831853f / 24.0f, edge = 2.0f * 0.08f * std::sin(0.5f * step), perimeter = 24.0f * edge;
    const auto expectedArc = [&](Vec3 p) {
        const float x = p.x, y = p.y + 0.3f;
        float phi = std::atan2(x, -y);
        if (phi < 0.0f) phi += 6.2831853f;
        float arc = x;
        if (phi >= 0.5f * step && phi < 23.5f * step) {
            const int k = static_cast<int>(std::floor(phi / step - 0.5f));
            const float at = (static_cast<float>(k) + 0.5f) * step;
            arc = 0.5f * edge + static_cast<float>(k) * edge + std::hypot(x - 0.08f * std::sin(at), y + 0.08f * std::cos(at));
        }
        return arc > 0.5f * perimeter ? arc - perimeter : arc;
    };
    int located = 0, nearCut = 0;
    float worst = 0.0f;
    for (std::size_t t = 0; t < tube.triangles.size(); ++t) {
        const Vec3 a = tube.positions[tube.triangles[t][0]], b = tube.positions[tube.triangles[t][1]], c = tube.positions[tube.triangles[t][2]];
        for (int i = 0; i <= 10; ++i)
            for (int j = 0; i + j <= 10; ++j) {
                SurfaceSample sample;
                sample.triangle = t;
                sample.part = part;
                sample.normal = tube.normal(t);
                sample.footprint = 0.001f;
                sample.position = a + (b - a) * (static_cast<float>(i) / 10.0f) + (c - a) * (static_cast<float>(j) / 10.0f);
                const std::optional<SpacePoint> at = ring.locate(sample);
                if (!at) continue;
                ++located;
                const float expected = expectedArc(sample.position);
                // (Os dois lados do corte sao o mesmo lugar.)
                float error = std::abs(at->point.x - expected);
                error = std::min(error, std::abs(perimeter - error));
                worst = std::max(worst, error);
                if (std::abs(expected) > 0.4f * perimeter) ++nearCut;
            }
    }
    require(located > 2000 && nearCut > 200, "a volta inteira tem coordenada, perto do corte tambem");
    require(worst < 0.002f, "o arco em volta, sem borrar no corte nem na fenda (pior " + std::to_string(worst * 1000.0f) + " mm)");
}

void testSvg() {
    std::string error;
    const auto square = parseSvgPath("M0 0 L10 0 L10 10 L0 10 Z", &error);
    require(square && square->contours.size() == 1 && near(std::abs(contourArea(square->contours[0])), 100.0f, 1e-3f), "svg: o quadrado");
    const auto relative = parseSvgPath("m0,0 h10 v10 h-10 z", &error);
    require(relative && near(std::abs(contourArea(relative->contours[0])), 100.0f, 1e-3f), "svg: relativo, H e V");
    // Numeros colados, expoente e o L repetido sem letra.
    const auto compact = parseSvgPath("M0 0L1e1 0 10 10-0.0 1e1z", &error);
    require(compact && near(std::abs(contourArea(compact->contours[0])), 100.0f, 1e-3f), "svg: numeros compactos " + error);
    // O furo: o contorno de dentro no sentido contrario.
    const auto hole = parseSvgPath("M0 0 H10 V10 H0 Z M3 3 V7 H7 V3 Z", &error);
    require(hole && hole->contains({ 1.0f, 1.0f }) && !hole->contains({ 5.0f, 5.0f }), "svg: o furo");
    // O circulo em dois arcos (bandeiras coladas) e o quarto de circulo numa cubica.
    const auto circle = parseSvgPath("M0 5A5 5 0 1 0 10 5A5 5 0 10 0 5Z", &error);
    require(circle && near(std::abs(contourArea(circle->contours[0])), 78.5398f, 0.4f), "svg: o circulo em arcos " + error);
    const auto quarter = parseSvgPath("M0 0 L10 0 C10 5.523 5.523 10 0 10 Z", &error);
    require(quarter && near(std::abs(contourArea(quarter->contours[0])), 78.5398f, 0.4f), "svg: a cubica");
    // A quadratica e a continuacao refletida (T): o S fechado tem um lobo de
    // cada lado da reta.
    const auto wave = parseSvgPath("M0 0 Q5 10 10 0 T20 0", &error);
    require(wave && wave->contains({ 5.0f, 2.0f }) && wave->contains({ 15.0f, -2.0f }) && !wave->contains({ 5.0f, -2.0f }), "svg: Q e T");
    require(!parseSvgPath("L 1 2", &error) && !parseSvgPath("M 0 0 L 1 0 L 1 1 Z 4 4", &error) && !parseSvgPath("M 0 0 X 1", &error)
            && !parseSvgPath("M 0 0 L 5 0", &error),
        "svg: os erros (sem M, numero depois de Z, comando desconhecido, sem area)");
    const auto document = parseSvgDocument(R"(<?xml version="1.0"?>
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 50">
          <path id="fundo" style="fill:#000" d="M0 0 H100 V50 H0 Z"/>
          <path d='M10 10 H20 V20 H10 Z' id='estrela'/>
        </svg>)", &error);
    require(document && document->viewBox && near((*document->viewBox)[2], 100.0f, 1e-5f) && document->paths.size() == 2
            && document->paths[1].first == "estrela" && document->paths[0].second.find("H100") != std::string::npos,
        "svg: o arquivo (viewBox e os caminhos pelo id)");
}

void testImage() {
    // Uma imagem de 4 x 2: a metade da esquerda vermelha, a da direita
    // transparente (com cor de lixo, como sai dos editores).
    Image picture(4, 2, { 0.0f, 0.0f, 0.0f, 0.0f });
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 4; ++x) picture.setPixel(x, y, x < 2 ? Color { 1.0f, 0.0f, 0.0f, 1.0f } : Color { 0.0f, 0.0f, 1.0f, 0.0f });
    const PaintImage image(picture);
    require(image.width() == 4 && image.height() == 2, "a imagem: o tamanho");
    const Color left = image.sample(0.2f, 0.5f, 0.0f), right = image.sample(0.9f, 0.5f, 0.0f), edge = image.sample(0.5f, 0.5f, 0.0f);
    require(near(left.r, 1.0f, 1e-3f) && near(left.a, 1.0f, 1e-3f) && right.a < 1e-3f, "a imagem: a cor e o transparente");
    // Na beira, meio transparente mas vermelha (sem o azul do lixo).
    require(near(edge.a, 0.5f, 0.02f) && near(edge.r, 1.0f, 1e-3f) && edge.b < 1e-3f, "a imagem: a beira sem franja");
    require(image.sample(1.5f, 0.5f, 0.0f).a == 0.0f, "fora da imagem, transparente");
    // Uma amostra do tamanho da imagem le a media (o nivel mais baixo).
    require(near(image.sample(0.5f, 0.5f, 1.0f).a, 0.5f, 0.02f), "a amostra grande le a media");
    // No desenho: o arquivo, a caixa pela proporcao e a forma da caixa.
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "matter_paint_tests";
    std::filesystem::create_directories(directory);
    require(writePng(picture, (directory / "escudo.png").string()), "gravou a imagem de teste");
    std::map<std::string, PaintSpace> spaces;
    PaintSpace plane;
    plane.locate = [](const SurfaceSample& sample) -> std::optional<SpacePoint> {
        return SpacePoint { { sample.position.x, sample.position.y }, -1, sample.footprint };
    };
    spaces["plano"] = plane;
    std::string error;
    const auto design = parsePaintDesign(R"({ "layers": [
        { "name": "escudo", "space": "plano", "image": { "file": "escudo.png", "at": [1.0, 1.0], "height": 0.2 } },
        { "name": "logo", "space": "plano", "shape": { "type": "path", "d": "M0 0 L10 0 L5 5 Z", "viewBox": [0, 0, 10, 10], "at": [3.0, 2.0], "height": 0.5 },
          "color": [0, 0, 0] } ] })", spaces, directory, &error);
    require(design.has_value(), "o desenho com imagem e caminho: " + error);
    const PaintLayer& crest = design->layers[0];
    require(near(crest.shape.bounds().maximum.x - crest.shape.bounds().minimum.x, 0.4f, 1e-4f)
            && near(crest.shape.bounds().maximum.y - crest.shape.bounds().minimum.y, 0.2f, 1e-4f),
        "a caixa da imagem pela proporcao");
    const Color white { 1.0f, 1.0f, 1.0f, 1.0f };
    const Color onRed = paintOver(white, crest, { 0.85f, 1.0f }, 0.001f), onClear = paintOver(white, crest, { 1.15f, 1.0f }, 0.001f);
    require(near(onRed.r, 1.0f, 1e-3f) && onRed.g < 0.01f && near(onClear.g, 1.0f, 1e-3f), "a imagem pinta o vermelho e deixa o transparente");
    // O caminho: o quadro de 10 x 10 com 0,5 de altura em (3, 2); o
    // triangulo de cima do SVG (y desce) fica na metade de cima.
    const Box2 box = design->layers[1].shape.bounds();
    require(near(box.minimum.x, 2.75f, 1e-4f) && near(box.maximum.x, 3.25f, 1e-4f) && near(box.minimum.y, 2.0f, 1e-4f) && near(box.maximum.y, 2.25f, 1e-4f),
        "o caminho no quadro, com o y virado");
    std::filesystem::remove_all(directory);
    require(!parsePaintDesign(R"({ "layers": [ { "name": "x", "space": "plano", "image": { "file": "nenhuma.png", "at": [0, 0], "height": 1 } } ] })",
                spaces, directory, &error)
            && error.find("\"x\"") != std::string::npos,
        "a imagem que falta, com o nome da camada");
}

void testDesign() {
    std::map<std::string, PaintSpace> spaces;
    PaintSpace plane;
    plane.locate = [](const SurfaceSample& sample) -> std::optional<SpacePoint> {
        return SpacePoint { { sample.position.x, sample.position.y }, -1, sample.footprint };
    };
    plane.anchors["topo"] = 1.0f;
    spaces["plano"] = plane;
    const std::string text = R"({
        "colors": { "preto": [20, 20, 20], "branco": "#ffffff" },
        "base": { "camisa": "branco" },
        "layers": [
            { "name": "faixa", "space": "plano", "shape": { "type": "rectangle", "from": [0, "topo-0.25"], "to": [0.5, "topo"] }, "color": "preto" },
            { "name": "numero", "space": "plano", "shape": { "type": "text", "text": "10", "height": 0.2, "at": [0.5, 0.5], "spacing": 0.072 },
              "color": [10, 10, 10], "outline": { "width": 0.01, "color": "branco" }, "sides": "right" },
            { "name": "sombra", "space": "plano", "shape": { "type": "ellipse", "center": [0, 0], "radius": [0.1, 0.2] },
              "gradient": { "type": "radial", "center": [0, 0], "radius": [0.1, 0.2], "stops": [[0, [128, 128, 128]], [1, [255, 255, 255]]] },
              "blend": "multiply" }
        ] })";
    std::string error;
    const std::optional<PaintDesign> design = parsePaintDesign(text, spaces, ".", &error);
    require(design.has_value(), "o desenho: " + error);
    require(design->layers.size() == 3 && design->partBase.count("camisa") && near(design->partBase.at("camisa").r, 1.0f, 1e-5f),
        "as camadas e a base");
    const PaintLayer& band = design->layers[0];
    require(near(band.shape.bounds().minimum.y, 0.75f, 1e-5f) && near(band.shape.bounds().maximum.y, 1.0f, 1e-5f), "a ancora com deslocamento");
    const PaintLayer& number = design->layers[1];
    require(near(number.shape.bounds().maximum.y - number.shape.bounds().minimum.y, 0.2f, 1e-4f)
            && near(0.5f * (number.shape.bounds().minimum.x + number.shape.bounds().maximum.x), 0.5f, 1e-3f)
            && near(number.shape.bounds().maximum.x - number.shape.bounds().minimum.x, 0.2f * 1.09f, 1e-3f) && number.sides == Sides::Right
            && number.outline > 0.0f,
        "o texto centrado na altura dada, com o contorno, so a direita");
    require(design->layers[2].blend == Blend::Multiply && design->layers[2].fill.kind == Fill::Kind::Radial, "o degrade multiplicando");
    // Os erros dizem a camada.
    const std::string badSpace = R"({ "layers": [ { "name": "x", "space": "nenhum", "shape": { "type": "ellipse", "center": [0, 0], "radius": 1 } } ] })";
    require(!parsePaintDesign(badSpace, spaces, ".", &error) && error.find("nenhum") != std::string::npos, "espaco desconhecido");
    const std::string badAnchor = R"({ "layers": [ { "name": "y", "space": "plano", "shape": { "type": "rectangle", "from": [0, "fundo"], "to": [1, 1] } } ] })";
    require(!parsePaintDesign(badAnchor, spaces, ".", &error) && error.find("fundo") != std::string::npos && error.find("\"y\"") != std::string::npos,
        "ancora desconhecida, com o nome da camada");
    const std::string badColor = R"({ "layers": [ { "name": "z", "space": "plano", "shape": { "type": "ellipse", "center": [0, 0], "radius": 1 }, "color": "roxo" } ] })";
    require(!parsePaintDesign(badColor, spaces, ".", &error) && error.find("roxo") != std::string::npos, "cor desconhecida");
}

} // namespace

int main() {
    try {
        testShapes();
        testFonts();
        testFillsAndLayers();
        testSpaces();
        testRing();
        testSvg();
        testImage();
        testDesign();
    } catch (const std::exception& exception) {
        std::cerr << "MatterModelingPaint test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterModelingPaint tests passed (" << checks << " checagens)\n";
    return 0;
}
