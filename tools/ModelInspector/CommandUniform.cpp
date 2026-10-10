#include "Commands.hpp"

#include "Engine/Modeling/Analysis/CrossSection.hpp"
#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Assembly/PlayerAssembly.hpp"
#include "Engine/Modeling/Assembly/Uniform.hpp"
#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"
#include "Engine/Modeling/Raster/TextRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <tuple>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

float jointWeight(const Mesh& mesh, std::uint32_t v, int joint) {
    if (v >= mesh.skin.size() || joint < 0) return 0.0f;
    float weight = 0.0f;
    for (std::size_t i = 0; i < 4; ++i)
        if (mesh.skin[v].joints[i] == joint) weight += mesh.skin[v].weights[i];
    return weight;
}

Vec3 jointAt(const Skeleton& skeleton, const char* name) {
    const int joint = skeleton.find(name);
    return joint >= 0 ? skeleton.joints[static_cast<std::size_t>(joint)].bind.position : Vec3 {};
}

// Os vertices do pano: so os dos triangulos da camisa e do calcao (o da
// borda de dentro do recuo e da pele, fica de fora).
std::vector<int> clothVertices(const Mesh& mesh) {
    const int shirt = mesh.findPart("camisa"), shorts = mesh.findPart("calcao");
    std::vector<int> piece(mesh.positions.size(), -2);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const int part = mesh.partOf(t);
        const int kind = part == shirt ? 0 : part == shorts ? 1 : -1;
        for (const std::uint32_t v : mesh.triangles[t]) {
            if (piece[v] == -2) piece[v] = kind;
            else if (piece[v] != kind) piece[v] = -1;
        }
    }
    return piece;
}

struct Stats {
    std::vector<float> values;
    void add(float value) { values.push_back(value); }
    [[nodiscard]] float mean() const {
        float sum = 0.0f;
        for (const float value : values) sum += value;
        return values.empty() ? 0.0f : sum / static_cast<float>(values.size());
    }
    [[nodiscard]] float percentile(float fraction) const {
        if (values.empty()) return 0.0f;
        std::vector<float> sorted = values;
        std::sort(sorted.begin(), sorted.end());
        return sorted[std::min(sorted.size() - 1, static_cast<std::size_t>(fraction * static_cast<float>(sorted.size() - 1) + 0.5f))];
    }
};

// O alto do contorno de frente (o maior z) entre |y| em [y0, y1], acima de
// "floor" e abaixo de "ceiling".
float topAt(const Mesh& mesh, float y0, float y1, float floor, float ceiling) {
    float top = -1e9f;
    for (const Triangle& triangle : mesh.triangles)
        for (int i = 0; i < 3; ++i) {
            // As arestas cortadas na faixa (o contorno nao depende de cair
            // vertice nela).
            const Vec3 a = mesh.positions[triangle[static_cast<std::size_t>(i)]];
            const Vec3 b = mesh.positions[triangle[static_cast<std::size_t>((i + 1) % 3)]];
            for (int k = 0; k <= 8; ++k) {
                const Vec3 p = lerp(a, b, static_cast<float>(k) / 8.0f);
                const float ay = std::abs(p.y);
                if (ay < y0 || ay > y1 || p.z < floor || p.z > ceiling) continue;
                top = std::max(top, p.z);
            }
        }
    return top;
}

struct Extent {
    bool found = false;
    float half = 0.0f;
    float front = 0.0f;
    float back = 0.0f;
};

// O tronco numa altura: o corte horizontal da superficie, o contorno mais de
// fora em volta do meio (o que se ve: o pano, ou a pele sem ele).
Extent torsoExtent(const Mesh& mesh, float z, float centerX) {
    const Plane plane = Plane::fromPointNormal({ 0.0f, 0.0f, z }, { 0.0f, 0.0f, 1.0f });
    const MeshSection section = sliceMesh(mesh, mesh.positions, plane);
    Extent extent;
    float largest = -1.0f;
    for (const SectionLoop& loop : section.loops) {
        if (!loop.closed || !loopContains(loop, plane, { centerX, 0.0f, z })) continue;
        Extent candidate { true, 0.0f, -1e9f, 1e9f };
        for (const Vec3& p : loop.points) {
            candidate.half = std::max(candidate.half, std::abs(p.y));
            candidate.front = std::max(candidate.front, p.x);
            candidate.back = std::min(candidate.back, p.x);
        }
        const float size = candidate.half * (candidate.front - candidate.back);
        if (size > largest) {
            largest = size;
            extent = candidate;
        }
    }
    return extent;
}

// Um traco grosso na imagem.
void stroke(Image& image, float x0, float y0, float x1, float y1, Color color, float thickness) {
    const float length = std::hypot(x1 - x0, y1 - y0);
    const int steps = std::max(1, static_cast<int>(length * 2.0f));
    const int radius = static_cast<int>(std::ceil(thickness * 0.5f));
    for (int s = 0; s <= steps; ++s) {
        const float t = static_cast<float>(s) / static_cast<float>(steps);
        const float x = x0 + (x1 - x0) * t, y = y0 + (y1 - y0) * t;
        for (int dy = -radius; dy <= radius; ++dy)
            for (int dx = -radius; dx <= radius; ++dx) {
                const float d = std::hypot(static_cast<float>(dx), static_cast<float>(dy));
                if (d > thickness * 0.5f + 0.5f) continue;
                const int px = static_cast<int>(std::lround(x)) + dx, py = static_cast<int>(std::lround(y)) + dy;
                if (px >= 0 && py >= 0 && px < image.width() && py < image.height())
                    image.blendPixel(px, py, color, std::clamp(thickness * 0.5f + 0.5f - d, 0.0f, 1.0f));
            }
    }
}

// Um corte desenhado em 2D: os contornos do corpo (ciano) e com o uniforme
// (laranja) no plano, com a grade de 1 cm (5 cm mais forte).
struct SectionPlot {
    std::string title;
    Plane plane;
    // Os eixos da imagem (u para a direita, v para cima) e a janela (m).
    Vec3 u, v;
    float u0, u1, v0, v1;
};

Image plotSection(const SectionPlot& plot, const Mesh& bare, const Mesh& dressed, float pixelsPerMeter) {
    const int width = static_cast<int>((plot.u1 - plot.u0) * pixelsPerMeter);
    const int height = static_cast<int>((plot.v1 - plot.v0) * pixelsPerMeter);
    Image image(width, height, rgb(12, 15, 20));
    const auto toPixel = [&](Vec3 p) {
        return std::pair<float, float> { (dot(p, plot.u) - plot.u0) * pixelsPerMeter, (plot.v1 - dot(p, plot.v)) * pixelsPerMeter };
    };
    for (int k = static_cast<int>(std::floor(plot.u0 * 100.0f)); k <= static_cast<int>(std::ceil(plot.u1 * 100.0f)); ++k) {
        const float x = (static_cast<float>(k) * 0.01f - plot.u0) * pixelsPerMeter;
        stroke(image, x, 0.0f, x, static_cast<float>(height), k % 5 == 0 ? rgb(40, 48, 58) : rgb(24, 29, 36), 1.0f);
    }
    for (int k = static_cast<int>(std::floor(plot.v0 * 100.0f)); k <= static_cast<int>(std::ceil(plot.v1 * 100.0f)); ++k) {
        const float y = (plot.v1 - static_cast<float>(k) * 0.01f) * pixelsPerMeter;
        stroke(image, 0.0f, y, static_cast<float>(width), y, k % 5 == 0 ? rgb(40, 48, 58) : rgb(24, 29, 36), 1.0f);
    }
    const auto drawLoops = [&](const Mesh& mesh, Color color, float thickness) {
        const MeshSection section = sliceMesh(mesh, mesh.positions, plot.plane);
        for (const SectionLoop& loop : section.loops)
            for (std::size_t i = 0; i + 1 < loop.points.size() + (loop.closed ? 1 : 0); ++i) {
                const auto [ax, ay] = toPixel(loop.points[i]);
                const auto [bx, by] = toPixel(loop.points[(i + 1) % loop.points.size()]);
                stroke(image, ax, ay, bx, by, color, thickness);
            }
    };
    drawLoops(bare, rgb(70, 210, 255), 2.4f);
    drawLoops(dressed, rgb(255, 150, 60), 1.8f);
    TextRenderer::standard().drawLabel(image, 10.0f, 10.0f, plot.title, 18.0f, rgb(220, 228, 236), rgb(20, 26, 34), 5.0f);
    return image;
}

// A silhueta (o que nao e fundo) da malha na camera.
std::vector<bool> silhouette(const Mesh& mesh, const OrbitCamera& camera, int width, int height, const Image& background) {
    InspectionScene scene;
    scene.mesh = &mesh;
    InspectionStyle style;
    style.surface = SurfaceMode::Clay;
    style.skeleton = style.joints = style.grid = style.gizmo = style.scaleBar = style.hud = false;
    style.supersample = 1;
    const Image image = renderInspection(scene, style, camera, width, height);
    std::vector<bool> mask(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), false);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            const Color a = image.pixel(x, y), b = background.pixel(x, y);
            mask[static_cast<std::size_t>(y) * width + x] = std::abs(a.r - b.r) + std::abs(a.g - b.g) + std::abs(a.b - b.b) > 0.01f;
        }
    return mask;
}

// O uniforme por cima do corpo na vista: em vermelho o pano alem da
// silhueta do corpo, o contorno do corpo em ciano.
Image silhouetteOverlay(const Mesh& bare, const Mesh& dressed, const OrbitCamera& camera, int width, int height, const std::string& title) {
    InspectionStyle plain;
    plain.skeleton = plain.joints = plain.grid = plain.gizmo = plain.scaleBar = plain.hud = false;
    plain.supersample = 1;
    const Image background = renderInspection(InspectionScene {}, plain, camera, width, height);
    const std::vector<bool> body = silhouette(bare, camera, width, height, background);
    const std::vector<bool> worn = silhouette(dressed, camera, width, height, background);
    InspectionScene scene;
    scene.mesh = &dressed;
    InspectionStyle style;
    style.surface = SurfaceMode::Solid;
    style.skeleton = style.joints = style.grid = style.gizmo = style.hud = false;
    style.frontLight = true;
    Image image = renderInspection(scene, style, camera, width, height);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * width + x;
            if (worn[i] && !body[i]) image.blendPixel(x, y, rgb(255, 60, 50), 0.6f);
            // A borda do corpo.
            if (body[i]) {
                const bool edge = (x > 0 && !body[i - 1]) || (x + 1 < width && !body[i + 1]) || (y > 0 && !body[i - width])
                    || (y + 1 < height && !body[i + width]);
                if (edge) image.blendPixel(x, y, rgb(70, 210, 255), 1.0f);
            }
        }
    TextRenderer::standard().drawLabel(image, 10.0f, 10.0f, title, 18.0f, rgb(220, 228, 236), rgb(20, 26, 34), 5.0f);
    return image;
}

} // namespace

// O uniforme sobre o corpo (a mesma montagem sem o uniforme): a folga de
// verdade do pano em cada regiao (a distancia de cada vertice dele a pele),
// as larguras do tronco com e sem ele, a linha do ombro de frente e os cortes
// (ombro de frente e de lado, peito e cintura de cima) - o pano tem que seguir
// o corpo, que e o da referencia.
int runUniform(const ModelContext& context, const CommandLine& line) {
    (void)line;
    const PlayerModelManifest& manifest = context.loaded.manifest;
    if (manifest.assemblyPath.empty()) {
        std::cerr << "o modelo nao e montado (sem montagem.json)\n";
        return 1;
    }
    std::ifstream file(manifest.assemblyPath);
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    const std::optional<AssemblyRecipe> recipe = parseAssemblyRecipe(text.str(), &error);
    if (!recipe) {
        std::cerr << "receita: " << error << '\n';
        return 1;
    }
    if (!recipe->uniform) {
        std::cout << "a receita nao veste o uniforme\n";
        return 0;
    }
    const Skeleton& skeleton = context.loaded.rig->skeleton;
    const std::filesystem::path directory = manifest.assemblyPath.parent_path();
    // O corpo sem o uniforme e sem a pintura (os espacos dela sao os das
    // pecas) nem textura: so a forma conta.
    AssemblyRecipe bareRecipe = *recipe;
    bareRecipe.uniform = false;
    bareRecipe.paintFile.clear();
    bareRecipe.textured = false;
    const AssemblyResult dressed = assemblePlayer(*recipe, directory, skeleton);
    const AssemblyResult bare = assemblePlayer(bareRecipe, directory, skeleton);
    if (!dressed.ok || !bare.ok) {
        std::cerr << "montagem: " << (dressed.ok ? bare.error : dressed.error) << '\n';
        return 1;
    }
    const UniformLines lines = uniformLines(*recipe, skeleton);
    const Vec3 shoulder = jointAt(skeleton, "RightUpperArm"), neck = jointAt(skeleton, "Neck");
    const Vec3 upperChest = jointAt(skeleton, "UpperChest"), chest = jointAt(skeleton, "Chest");
    const Vec3 abdomen = jointAt(skeleton, "Abdomen"), pelvis = jointAt(skeleton, "Pelvis");
    const Vec3 hip = jointAt(skeleton, "RightThigh");
    const int rightArm = skeleton.find("RightUpperArm"), leftArm = skeleton.find("LeftUpperArm");

    // ---- A folga: a superficie do pano (pontos em cada triangulo) ate a pele ----
    const TriangleBvh body(bare.mesh.positions, bare.mesh.triangles);
    const std::vector<int> piece = clothVertices(dressed.mesh);
    std::map<std::string, Stats> regions;
    const std::vector<std::string> order { "ombro", "peito", "costas", "barriga", "lombar", "manga", "barra da camisa", "cintura",
        "gluteo", "coxa", "barra do calcao" };
    // Os pontos de um triangulo: os cantos, os meios das arestas e o centro.
    static constexpr float Samples[7][3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.5f, 0.5f, 0 }, { 0, 0.5f, 0.5f }, { 0.5f, 0, 0.5f },
        { 1.0f / 3, 1.0f / 3, 1.0f / 3 } };
    const int shirtPart = dressed.mesh.findPart("camisa"), shortsPart = dressed.mesh.findPart("calcao");
    for (std::size_t t = 0; t < dressed.mesh.triangles.size(); ++t) {
        const Triangle& triangle = dressed.mesh.triangles[t];
        const int part = dressed.mesh.partOf(t);
        if (part != shirtPart && part != shortsPart) continue;
        // So o pano (a faixa do recuo encosta na pele).
        if (piece[triangle[0]] < 0 || piece[triangle[1]] < 0 || piece[triangle[2]] < 0) continue;
        float arm = 0.0f;
        for (const std::uint32_t v : triangle) arm += (jointWeight(dressed.mesh, v, rightArm) + jointWeight(dressed.mesh, v, leftArm)) / 3.0f;
        const Vec3 a = dressed.mesh.positions[triangle[0]], b = dressed.mesh.positions[triangle[1]], c = dressed.mesh.positions[triangle[2]];
        for (const auto& weight : Samples) {
            const Vec3 p = a * weight[0] + b * weight[1] + c * weight[2];
            const std::optional<TriangleBvh::Nearest> nearest = body.nearest(p, 0.2f);
            if (!nearest) continue;
            const float gap = (p - nearest->point).length();
            std::string region;
            if (part == shirtPart) {
                if (arm > 0.5f && p.z < shoulder.z - 0.02f) region = "manga";
                else if (p.z > shoulder.z - 0.04f) region = "ombro";
                else if (p.z < lines.shirtHem + 0.06f) region = "barra da camisa";
                else if (p.z > chest.z) region = p.x > -0.02f ? "peito" : "costas";
                else region = p.x > -0.02f ? "barriga" : "lombar";
            } else {
                if (p.z > pelvis.z) region = "cintura";
                else if (p.z < lines.legHemPoint.z + 0.05f) region = "barra do calcao";
                else if (p.x < hip.x - 0.02f && p.z > hip.z - 0.15f) region = "gluteo";
                else region = "coxa";
            }
            regions[region].add(gap);
        }
    }
    std::cout << "Uniforme sobre o corpo (a montagem com e sem o uniforme)\n\n";
    std::cout << "Folga do pano ate a pele (mm)        pontos     media    p90    max\n";
    for (const std::string& region : order) {
        const auto found = regions.find(region);
        if (found == regions.end()) continue;
        const Stats& stats = found->second;
        std::printf("  %-32s %6zu  %6.1f %6.1f %6.1f\n", region.c_str(), stats.values.size(), stats.mean() * 1000.0f,
            stats.percentile(0.9f) * 1000.0f, stats.percentile(1.0f) * 1000.0f);
    }
    std::printf("  (receita: camisa %.1f / barra %.1f / manga %.1f / gola %.1f; calcao %.1f / alto %.1f / barra %.1f; queda %.2f, "
                "alisamento %d)\n",
        recipe->shirtLoose * 1000.0f, recipe->shirtLooseHem * 1000.0f, recipe->sleeveLoose * 1000.0f, recipe->collarLoose * 1000.0f,
        recipe->shortsLoose * 1000.0f, recipe->shortsLooseTop * 1000.0f, recipe->shortsLooseHem * 1000.0f, recipe->clothHang,
        recipe->clothSmoothing);

    // ---- As larguras do tronco ----
    std::cout << "\nTronco (cm; corpo -> com o uniforme)   meia largura          frente                costas\n";
    const std::pair<const char*, float> heights[] { { "ombro", shoulder.z }, { "axila", shoulder.z - 0.07f },
        { "peito", upperChest.z - 0.03f }, { "cintura", abdomen.z }, { "quadril", pelvis.z - 0.03f }, { "coxa", hip.z - 0.12f } };
    for (const auto& [name, z] : heights) {
        const Extent a = torsoExtent(bare.mesh, z, -0.02f), b = torsoExtent(dressed.mesh, z, -0.02f);
        if (!a.found || !b.found) continue;
        std::printf("  %-8s z %+.3f        %5.1f -> %5.1f (%+4.1f)   %5.1f -> %5.1f (%+4.1f)   %5.1f -> %5.1f (%+4.1f)\n", name, z,
            a.half * 100.0f, b.half * 100.0f, (b.half - a.half) * 100.0f, a.front * 100.0f, b.front * 100.0f, (b.front - a.front) * 100.0f,
            a.back * 100.0f, b.back * 100.0f, (a.back - b.back) * 100.0f);
    }
    // A largura dos ombros (com o deltoide).
    {
        float bareWidth = 0.0f, dressedWidth = 0.0f;
        for (const Vec3& p : bare.mesh.positions)
            if (p.z > shoulder.z - 0.06f && p.z < shoulder.z + 0.1f) bareWidth = std::max(bareWidth, std::abs(p.y));
        for (const Vec3& p : dressed.mesh.positions)
            if (p.z > shoulder.z - 0.06f && p.z < shoulder.z + 0.1f) dressedWidth = std::max(dressedWidth, std::abs(p.y));
        std::printf("  ombros (deltoide a deltoide)   %5.1f -> %5.1f cm (%+.1f)\n", bareWidth * 200.0f, dressedWidth * 200.0f,
            (dressedWidth - bareWidth) * 200.0f);
    }

    // ---- A linha do ombro de frente: o alto do contorno ----
    std::cout << "\nLinha do ombro de frente (o alto do contorno, cm acima da junta do ombro)\n   |y| cm   corpo   uniforme   diferenca (mm)\n";
    const float floor = shoulder.z - 0.12f, ceiling = neck.z + 0.03f;
    for (int k = 6; k <= 26; k += 2) {
        const float y = static_cast<float>(k) * 0.01f;
        const float a = topAt(bare.mesh, y - 0.005f, y + 0.005f, floor, ceiling);
        const float b = topAt(dressed.mesh, y - 0.005f, y + 0.005f, floor, ceiling);
        if (a < -1e8f || b < -1e8f) continue;
        std::printf("   %4d    %6.1f   %6.1f     %+6.1f\n", k, (a - shoulder.z) * 100.0f, (b - shoulder.z) * 100.0f, (b - a) * 1000.0f);
    }

    // ---- As imagens: as silhuetas e os cortes ----
    const float aspect = 0.75f;
    Box3 upper;
    upper.include({ -0.2f, -0.45f, hip.z - 0.25f });
    upper.include({ 0.2f, 0.45f, neck.z + 0.12f });
    std::vector<Image> images;
    std::vector<std::string> captions;
    for (const auto& [label, yaw] : { std::pair<const char*, float> { "frente", 0.0f }, { "lado direito", -90.0f }, { "costas", 180.0f } }) {
        const OrbitCamera camera = frameBounds(upper, yaw, 0.0f, aspect, 1.05f, true);
        images.push_back(silhouetteOverlay(bare.mesh, dressed.mesh, camera, 600, 800, std::string("Silhueta · ") + label));
        captions.push_back("vermelho: o pano alem do corpo · ciano: o contorno do corpo");
    }
    const float pixelsPerMeter = 1800.0f;
    const SectionPlot plots[] {
        { "Corte de frente pelo ombro (x " + formatLength(shoulder.x) + ")", Plane::fromPointNormal(shoulder, { 1.0f, 0.0f, 0.0f }),
            { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, -0.32f, 0.32f, shoulder.z - 0.22f, neck.z + 0.08f },
        { "Corte de lado pelo ombro (y " + formatLength(shoulder.y + 0.04f) + ")",
            Plane::fromPointNormal({ 0.0f, shoulder.y + 0.04f, 0.0f }, { 0.0f, 1.0f, 0.0f }), { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f },
            -0.22f, 0.2f, shoulder.z - 0.3f, neck.z + 0.08f },
        { "Corte de cima no peito (z " + formatLength(upperChest.z - 0.03f) + ")",
            Plane::fromPointNormal({ 0.0f, 0.0f, upperChest.z - 0.03f }, { 0.0f, 0.0f, 1.0f }), { 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f },
            -0.3f, 0.3f, -0.2f, 0.2f },
        { "Corte de cima na cintura (z " + formatLength(abdomen.z) + ")", Plane::fromPointNormal(abdomen, { 0.0f, 0.0f, 1.0f }),
            { 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, -0.3f, 0.3f, -0.2f, 0.2f },
        { "Corte de cima no quadril (z " + formatLength(pelvis.z - 0.03f) + ")",
            Plane::fromPointNormal({ 0.0f, 0.0f, pelvis.z - 0.03f }, { 0.0f, 0.0f, 1.0f }), { 0.0f, -1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f },
            -0.3f, 0.3f, -0.2f, 0.2f },
    };
    for (const SectionPlot& plot : plots) {
        images.push_back(plotSection(plot, bare.mesh, dressed.mesh, pixelsPerMeter));
        captions.push_back("ciano: o corpo · laranja: com o uniforme · grade de 1 cm");
    }
    const std::filesystem::path path = context.output(line, "uniforme.png");
    if (!saveImage(composeSheet(images, captions, 4, "Uniforme sobre o corpo"), path)) return 1;

    // ---- Os ombros de perto: o corpo e com o uniforme, nos mesmos angulos ----
    std::vector<Image> closeups;
    std::vector<std::string> closeupCaptions;
    Box3 around;
    around.include({ shoulder.x - 0.16f, shoulder.y - 0.12f, shoulder.z - 0.16f });
    around.include({ shoulder.x + 0.16f, shoulder.y + 0.2f, neck.z + 0.06f });
    InspectionStyle solid;
    solid.surface = SurfaceMode::Solid;
    solid.skeleton = solid.joints = solid.hud = false;
    solid.grid = false;
    solid.gizmoLegend = false;
    for (const auto& [label, yaw, pitch] : { std::tuple<const char*, float, float> { "3/4 de frente", -35.0f, 12.0f },
             { "de frente", 0.0f, 4.0f }, { "3/4 de costas", -145.0f, 12.0f }, { "de lado", -90.0f, 4.0f } }) {
        const OrbitCamera camera = frameBounds(around, yaw, pitch, 1.0f, 1.1f, false);
        for (const auto& [mesh, who] : { std::pair<const Mesh*, const char*> { &bare.mesh, "corpo" }, { &dressed.mesh, "uniforme" } }) {
            InspectionScene scene;
            scene.mesh = mesh;
            Image image = renderInspection(scene, solid, camera, 700, 700);
            TextRenderer::standard().drawLabel(image, 10.0f, 10.0f, std::string(who) + " · " + label, 18.0f, rgb(220, 228, 236),
                rgb(20, 26, 34), 5.0f);
            closeups.push_back(std::move(image));
            closeupCaptions.emplace_back();
        }
    }
    return saveImage(composeSheet(closeups, closeupCaptions, 4, "Ombro: corpo × uniforme"), path.parent_path() / "uniforme_ombro.png") ? 0 : 1;
}

} // namespace MatterEngine::ModelInspector
