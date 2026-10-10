#include "Engine/Modeling/Inspection/ReferenceComparison.hpp"

#include "Engine/Modeling/Raster/Rasterizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>

namespace MatterEngine::Modeling {

namespace {

// Mantem so o maior pedaco ligado e preenche os buracos de dentro.
void keepLargestAndFill(Silhouette& silhouette) {
    const int w = silhouette.width, h = silhouette.height;
    std::vector<int> label(silhouette.mask.size(), 0);
    int bestLabel = 0;
    std::size_t bestSize = 0;
    int next = 0;
    std::deque<int> queue;
    for (int start = 0; start < w * h; ++start) {
        if (silhouette.mask[static_cast<std::size_t>(start)] == 0 || label[static_cast<std::size_t>(start)] != 0) continue;
        ++next;
        std::size_t size = 0;
        queue.push_back(start);
        label[static_cast<std::size_t>(start)] = next;
        while (!queue.empty()) {
            const int index = queue.front();
            queue.pop_front();
            ++size;
            const int x = index % w, y = index / w;
            const std::array<std::array<int, 2>, 4> steps { { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } } };
            for (const auto& step : steps) {
                const int nx = x + step[0], ny = y + step[1];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                const int neighbor = ny * w + nx;
                if (silhouette.mask[static_cast<std::size_t>(neighbor)] == 0 || label[static_cast<std::size_t>(neighbor)] != 0)
                    continue;
                label[static_cast<std::size_t>(neighbor)] = next;
                queue.push_back(neighbor);
            }
        }
        if (size > bestSize) {
            bestSize = size;
            bestLabel = next;
        }
    }
    // Fundo de verdade: o que se liga a borda sem cruzar a figura.
    std::vector<std::uint8_t> outside(silhouette.mask.size(), 0);
    for (int x = 0; x < w; ++x) {
        queue.push_back(x);
        queue.push_back((h - 1) * w + x);
    }
    for (int y = 0; y < h; ++y) {
        queue.push_back(y * w);
        queue.push_back(y * w + w - 1);
    }
    while (!queue.empty()) {
        const int index = queue.front();
        queue.pop_front();
        if (outside[static_cast<std::size_t>(index)] != 0 || label[static_cast<std::size_t>(index)] == bestLabel) continue;
        outside[static_cast<std::size_t>(index)] = 1;
        const int x = index % w, y = index / w;
        if (x > 0) queue.push_back(index - 1);
        if (x + 1 < w) queue.push_back(index + 1);
        if (y > 0) queue.push_back(index - w);
        if (y + 1 < h) queue.push_back(index + w);
    }
    silhouette.top = h;
    silhouette.bottom = -1;
    silhouette.left = w;
    silhouette.right = -1;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) * w + x;
            const bool inside = outside[index] == 0;
            silhouette.mask[index] = inside ? 1 : 0;
            if (!inside) continue;
            silhouette.top = std::min(silhouette.top, y);
            silhouette.bottom = std::max(silhouette.bottom, y);
            silhouette.left = std::min(silhouette.left, x);
            silhouette.right = std::max(silhouette.right, x);
        }
}

} // namespace

float Silhouette::meanColumn(float fromFraction, float toFraction) const {
    const int from = top + static_cast<int>(fromFraction * static_cast<float>(bottom - top));
    const int to = top + static_cast<int>(toFraction * static_cast<float>(bottom - top));
    double sum = 0.0;
    std::size_t count = 0;
    for (int y = std::max(0, from); y <= std::min(height - 1, to); ++y)
        for (int x = 0; x < width; ++x)
            if (at(x, y)) {
                sum += x;
                ++count;
            }
    return count > 0 ? static_cast<float>(sum / static_cast<double>(count)) : static_cast<float>(width) * 0.5f;
}

int Silhouette::rowWidth(int y) const {
    if (y < 0 || y >= height) return 0;
    int filled = 0;
    for (int x = 0; x < width; ++x) filled += at(x, y) ? 1 : 0;
    return filled;
}

int Silhouette::rowSpan(int y) const {
    if (y < 0 || y >= height) return 0;
    int first = -1, last = -1;
    for (int x = 0; x < width; ++x)
        if (at(x, y)) {
            if (first < 0) first = x;
            last = x;
        }
    return first < 0 ? 0 : last - first + 1;
}

Silhouette extractSilhouette(const Image& image, float threshold) {
    Silhouette silhouette;
    silhouette.width = image.width();
    silhouette.height = image.height();
    silhouette.mask.assign(static_cast<std::size_t>(image.width()) * image.height(), 0);
    if (image.empty()) return silhouette;
    // O fundo: a media dos quatro cantos.
    Color background {};
    int samples = 0;
    const int patch = std::max(2, std::min(image.width(), image.height()) / 60);
    for (const auto& corner : std::array<std::array<int, 2>, 4> { { { 0, 0 }, { image.width() - patch, 0 },
             { 0, image.height() - patch }, { image.width() - patch, image.height() - patch } } })
        for (int y = 0; y < patch; ++y)
            for (int x = 0; x < patch; ++x) {
                const Color c = image.pixel(corner[0] + x, corner[1] + y);
                background.r += c.r;
                background.g += c.g;
                background.b += c.b;
                ++samples;
            }
    background.r /= static_cast<float>(samples);
    background.g /= static_cast<float>(samples);
    background.b /= static_cast<float>(samples);
    for (int y = 0; y < image.height(); ++y)
        for (int x = 0; x < image.width(); ++x) {
            const Color c = image.pixel(x, y);
            const float difference = std::max({ std::abs(c.r - background.r), std::abs(c.g - background.g),
                std::abs(c.b - background.b) });
            silhouette.mask[static_cast<std::size_t>(y) * image.width() + x] = difference > threshold ? 1 : 0;
        }
    keepLargestAndFill(silhouette);
    return silhouette;
}

Silhouette renderSilhouette(const InspectionScene& scene, const OrbitCamera& camera, int width, int height) {
    Silhouette silhouette;
    silhouette.width = width;
    silhouette.height = height;
    silhouette.mask.assign(static_cast<std::size_t>(width) * height, 0);
    if (scene.mesh == nullptr || scene.mesh->empty()) return silhouette;
    RasterTarget target(width, height);
    const Projector projector(camera, width, height);
    Rasterizer raster(target, projector);
    std::span<const Vec3> positions = scene.meshPositions;
    if (positions.empty()) positions = scene.mesh->positions;
    for (const Triangle& t : scene.mesh->triangles) raster.depthOnly(positions[t[0]], positions[t[1]], positions[t[2]]);
    silhouette.top = height;
    silhouette.left = width;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            if (target.depthAt(x, y) == -std::numeric_limits<float>::infinity()) continue;
            silhouette.mask[static_cast<std::size_t>(y) * width + x] = 1;
            silhouette.top = std::min(silhouette.top, y);
            silhouette.bottom = std::max(silhouette.bottom, y);
            silhouette.left = std::min(silhouette.left, x);
            silhouette.right = std::max(silhouette.right, x);
        }
    return silhouette;
}

ReferenceComparison compareWithReference(const InspectionScene& scene, const Image& reference, const ViewPreset& view,
    const std::string& title) {
    ReferenceComparison result;
    if (scene.skeleton == nullptr || reference.empty()) return result;
    const Silhouette figure = extractSilhouette(reference);
    if (!figure.valid()) {
        result.lines.push_back("a referencia nao tem uma figura separada do fundo");
        return result;
    }
    const int width = reference.width(), height = reference.height();
    const Skeleton& skeleton = *scene.skeleton;
    const std::vector<Vec3> joints = sceneJointPositions(scene);
    float top = skeleton.groundHeight;
    for (const Vec3& joint : joints) top = std::max(top, joint.z);
    result.hasMesh = scene.mesh != nullptr && !scene.mesh->empty();
    if (result.hasMesh) {
        std::span<const Vec3> positions = scene.meshPositions;
        if (positions.empty()) positions = scene.mesh->positions;
        for (const Vec3& p : positions) top = std::max(top, p.z);
    }
    const float modelHeight = std::max(0.1f, top - skeleton.groundHeight);
    const float figureHeight = static_cast<float>(figure.bottom - figure.top + 1);

    // Ortografica reta (pitch 0), a altura do modelo = a da figura.
    OrbitCamera camera;
    camera.orthographic = true;
    camera.yaw = view.yawDegrees * DegreesToRadians;
    camera.pitch = 0.0f;
    camera.distance = 6.0f;
    camera.orthoHalfHeight = modelHeight * static_cast<float>(height) / (2.0f * figureHeight);
    const float pixelsPerMeter = static_cast<float>(height) / (2.0f * camera.orthoHalfHeight);
    result.metersPerPixel = 1.0f / pixelsPerMeter;
    const float groundRow = static_cast<float>(figure.bottom + 1);
    const float targetZ = skeleton.groundHeight
        - (1.0f - 2.0f * groundRow / static_cast<float>(height)) * camera.orthoHalfHeight;
    // Horizontal: o eixo das pernas da figura no das pernas do modelo.
    Vec3 legs {};
    int legCount = 0;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const std::string& name = skeleton.joints[j].name;
        if (name.find("Thigh") != std::string::npos || name.find("Shin") != std::string::npos
            || name.find("Foot") != std::string::npos) {
            legs += joints[j];
            ++legCount;
        }
    }
    legs = legCount > 0 ? legs / static_cast<float>(legCount) : joints.front();
    const float figureAxis = figure.meanColumn(0.62f, 0.97f) + 0.5f;
    const Vec3 right = camera.right();
    const Vec3 shift = right * (-(figureAxis - static_cast<float>(width) * 0.5f) / pixelsPerMeter);
    camera.target = { legs.x + shift.x, legs.y + shift.y, targetZ };
    // Com malha: as pernas da silhueta do modelo no mesmo eixo das da figura
    // (as juntas ficam a frente da massa da perna, no pe).
    if (result.hasMesh) {
        const Silhouette provisional = renderSilhouette(scene, camera, width, height);
        if (provisional.valid()) {
            const float modelAxis = provisional.meanColumn(0.62f, 0.97f) + 0.5f;
            camera.target += right * ((modelAxis - figureAxis) / pixelsPerMeter);
        }
    }
    result.camera = camera;

    // Sobreposicao: a referencia ao fundo, o modelo em raio-x e os ossos.
    InspectionScene overlayScene = scene;
    overlayScene.backdrop = &reference;
    overlayScene.backdropOpacity = 0.85f;
    InspectionStyle style;
    style.surface = result.hasMesh ? SurfaceMode::XRay : SurfaceMode::Hidden;
    style.grid = false;
    style.gizmo = false;
    style.supersample = 2;
    style.title = title;
    result.overlay = renderInspection(overlayScene, style, camera, width, height);
    if (result.hasMesh) {
        InspectionScene solidScene = scene;
        solidScene.highlights = {};
        InspectionStyle solidStyle;
        solidStyle.surface = SurfaceMode::Solid;
        solidStyle.skeleton = false;
        solidStyle.joints = false;
        solidStyle.grid = false;
        solidStyle.gizmo = false;
        solidStyle.scaleBar = false;
        solidStyle.hud = false;
        solidStyle.supersample = 2;
        result.solid = renderInspection(solidScene, solidStyle, camera, width, height);
    }

    const Silhouette model = renderSilhouette(scene, camera, width, height);
    result.difference = Image(width, height, rgb(0, 0, 0));
    std::size_t both = 0, either = 0;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            const bool inReference = figure.at(x, y);
            const bool inModel = result.hasMesh && model.at(x, y);
            if (inReference || inModel) ++either;
            if (inReference && inModel) ++both;
            const Color color = inReference && inModel ? rgb(120, 120, 120)
                : inModel                             ? rgb(235, 60, 60)
                : inReference                         ? rgb(60, 110, 235)
                                                      : rgb(0, 0, 0);
            result.difference.setPixel(x, y, color);
            // Contorno do modelo na sobreposicao.
            if (inModel && (x == 0 || y == 0 || x == width - 1 || y == height - 1 || !model.at(x - 1, y)
                    || !model.at(x + 1, y) || !model.at(x, y - 1) || !model.at(x, y + 1)))
                result.overlay.setPixel(x, y, rgb(80, 255, 230));
        }
    result.intersectionOverUnion = result.hasMesh && either > 0 ? static_cast<float>(both) / static_cast<float>(either) : 0.0f;

    // Larguras por altura (fracao da altura a partir do chao).
    struct Level {
        float fraction;
        const char* name;
    };
    constexpr std::array<Level, 10> Levels { { { 0.04f, "tornozelo" }, { 0.16f, "panturrilha" }, { 0.27f, "joelho" },
        { 0.40f, "coxa" }, { 0.50f, "quadril" }, { 0.60f, "cintura" }, { 0.70f, "peito" }, { 0.79f, "ombros/bracos" },
        { 0.86f, "pescoco" }, { 0.93f, "cabeca" } } };
    const auto centimeters = [&](int pixels) {
        return static_cast<int>(std::lround(static_cast<float>(pixels) * result.metersPerPixel * 100.0f));
    };
    result.lines.push_back("larguras cheias (sem os vaos entre pernas e bracos), e o vao total entre parenteses:");
    for (const Level& level : Levels) {
        const int row = figure.bottom - static_cast<int>(level.fraction * figureHeight);
        const int referenceWidth = centimeters(figure.rowWidth(row));
        std::string line = std::string(level.name) + " (" + std::to_string(static_cast<int>(level.fraction * 100.0f))
            + "% da altura): referencia " + std::to_string(referenceWidth) + " cm ("
            + std::to_string(centimeters(figure.rowSpan(row))) + ")";
        if (result.hasMesh) {
            const int modelWidth = centimeters(model.rowWidth(row));
            line += " | modelo " + std::to_string(modelWidth) + " cm (" + std::to_string(centimeters(model.rowSpan(row)))
                + ") diferenca " + (modelWidth >= referenceWidth ? "+" : "") + std::to_string(modelWidth - referenceWidth);
        }
        result.lines.push_back(line);
    }
    result.lines.push_back("escala: 1 px = " + std::to_string(result.metersPerPixel * 1000.0f).substr(0, 5)
        + " mm (altura do modelo " + std::to_string(static_cast<int>(std::lround(modelHeight * 100.0f)))
        + " cm = altura da figura)");
    return result;
}

} // namespace MatterEngine::Modeling
