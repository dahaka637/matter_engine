#include "Workbench/ObjectInspector/ObjectInspectorScreen.hpp"

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/UI/FontAwesome.hpp"
#include "Workbench/UiKit.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <utility>

namespace MatterEngine::Workbench {

using namespace Modeling;

namespace {

constexpr ImVec4 MutedText { 0.47f, 0.54f, 0.60f, 1.0f };
constexpr ImVec4 InfoText { 0.62f, 0.72f, 0.80f, 1.0f };
constexpr ImVec4 ValueText { 0.84f, 0.89f, 0.94f, 1.0f };
constexpr ImVec4 AccentText { 0.38f, 0.69f, 1.0f, 1.0f };
constexpr ImVec4 WarningText { 1.0f, 0.62f, 0.42f, 1.0f };
// A imagem rapida (sem antisserrilhado) enquanto o mouse mexe a camera.
constexpr float InteractionHoldSeconds = 0.18f;
// Folga do enquadramento em volta da cena.
constexpr float FrameMargin = 1.3f;

struct ViewButton {
    const char* label;
    float yaw;
    float pitch;
};

constexpr std::array<ViewButton, 6> ViewButtons { { { "Frente", 0.0f, 4.0f }, { "Direita", -90.0f, 4.0f },
    { "Costas", 180.0f, 4.0f }, { "Esquerda", 90.0f, 4.0f }, { "Cima", 0.0f, 89.0f }, { "3/4", 35.0f, 22.0f } } };

struct ViewMode {
    ObjectInspectorScreen::View view;
    const char* label;
};

constexpr std::array<ViewMode, 6> ViewModes { { { ObjectInspectorScreen::View::Solid, "Sólido" },
    { ObjectInspectorScreen::View::SolidWire, "Sólido com arestas" }, { ObjectInspectorScreen::View::Wire, "Arestas" },
    { ObjectInspectorScreen::View::Normals, "Normais" }, { ObjectInspectorScreen::View::Clay, "Argila" },
    { ObjectInspectorScreen::View::Texture, "Textura (pixel art)" } } };

struct TestButton {
    ObjectTestKind3D kind;
    const char* label;
};

constexpr std::array<TestButton, 5> TestButtons { { { ObjectTestKind3D::Drop, "Queda" },
    { ObjectTestKind3D::Throw, "Arremesso" }, { ObjectTestKind3D::Ramp, "Rampa" }, { ObjectTestKind3D::Stack, "Pilha" },
    { ObjectTestKind3D::BallShot, "Bola" } } };

// Os materiais do catalogo com o nome na tela.
constexpr std::array<std::pair<const char*, const char*>, 12> MaterialNames { { { "default", "Padrão" },
    { "wood", "Madeira" }, { "steel", "Aço" }, { "concrete", "Concreto" }, { "plastic", "Plástico" },
    { "rubber", "Borracha" }, { "brick", "Tijolo" }, { "glass", "Vidro" }, { "grass", "Grama" }, { "body", "Corpo" },
    { "shoe_sole", "Sola" }, { "ocean", "Água" } } };

std::string materialName(const std::string& id) {
    for (const auto& [key, name] : MaterialNames)
        if (id == key) return name;
    return id;
}

SurfaceMode surfaceFor(ObjectInspectorScreen::View view) {
    switch (view) {
    case ObjectInspectorScreen::View::SolidWire: return SurfaceMode::SolidWire;
    case ObjectInspectorScreen::View::Wire: return SurfaceMode::Wire;
    case ObjectInspectorScreen::View::Normals: return SurfaceMode::Normals;
    case ObjectInspectorScreen::View::Clay: return SurfaceMode::Clay;
    default: return SurfaceMode::Solid;
    }
}

std::string timestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm local {};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local);
    return buffer;
}

float wrapAngle(float angle) {
    while (angle > Pi) angle -= 2.0f * Pi;
    while (angle < -Pi) angle += 2.0f * Pi;
    return angle;
}

// Os 8 cantos de uma caixa (centro, orientacao e meias-medidas).
std::array<Vec3, 8> boxCorners(Vec3 center, Quaternion orientation, Vec3 half) {
    std::array<Vec3, 8> corners;
    for (int k = 0; k < 8; ++k)
        corners[static_cast<std::size_t>(k)] = center
            + orientation.rotate({ (k & 1) ? half.x : -half.x, (k & 2) ? half.y : -half.y, (k & 4) ? half.z : -half.z });
    return corners;
}

// As 12 arestas de uma caixa.
void boxSegments(Vec3 center, Quaternion orientation, Vec3 half, std::vector<std::pair<Vec3, Vec3>>& segments) {
    const std::array<Vec3, 8> corners = boxCorners(center, orientation, half);
    for (int a = 0; a < 8; ++a)
        for (int bit = 1; bit < 8; bit <<= 1)
            if ((a & bit) == 0) segments.emplace_back(corners[static_cast<std::size_t>(a)], corners[static_cast<std::size_t>(a | bit)]);
}

// A sombra de um corpo no laboratorio: o contorno dele visto de cima,
// projetado no chao (um leque de ShadowRim lados logo acima do chao). Sem
// ela, no ar e no chao parecem iguais.
constexpr int ShadowRim = 8;
constexpr float ShadowLift = 0.002f;

// O contorno convexo (anti-horario) dos pontos no plano.
std::vector<Vec3> convexOutline(std::vector<Vec3> points) {
    std::sort(points.begin(), points.end(), [](Vec3 a, Vec3 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    const auto turn = [](Vec3 o, Vec3 a, Vec3 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); };
    std::vector<Vec3> hull(points.size() * 2);
    std::size_t count = 0;
    for (const Vec3& point : points) {
        while (count >= 2 && turn(hull[count - 2], hull[count - 1], point) <= 1e-9f) --count;
        hull[count++] = point;
    }
    for (std::size_t i = points.size() - 1, lower = count + 1; i-- > 0;) {
        while (count >= lower && turn(hull[count - 2], hull[count - 1], points[i]) <= 1e-9f) --count;
        hull[count++] = points[i];
    }
    hull.resize(count > 1 ? count - 1 : count);
    return hull;
}

// Uma linha da grade de propriedades: o nome a esquerda, o controle a
// direita (a largura toda da coluna).
void propertyRow(const char* label) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(MutedText, "%s", label);
    ImGui::TableSetColumnIndex(1);
    ImGui::SetNextItemWidth(-FLT_MIN);
}

bool beginPropertyTable(const char* id, float labelWidth) {
    if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_SizingFixedFit)) return false;
    ImGui::TableSetupColumn("nome", ImGuiTableColumnFlags_WidthFixed, labelWidth);
    ImGui::TableSetupColumn("valor", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

void metricRow(const char* label, const std::string& value) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextColored(MutedText, "%s", label);
    ImGui::TableSetColumnIndex(1);
    ImGui::TextColored(ValueText, "%s", value.c_str());
}

std::string secondsText(float value) {
    return value < 0.0f ? std::string("-") : formatNumber(value, 2) + " s";
}

std::string speedText(float value) {
    return value < 0.0f ? std::string("-") : formatNumber(value, 2) + " m/s";
}

} // namespace

ObjectInspectorScreen::ObjectInspectorScreen(Options options, std::function<void()> onBack)
    : m_options(std::move(options)), m_onBack(std::move(onBack)) {
    m_ground.groundHeight = 0.0f;
    reload();
}

ObjectInspectorScreen::~ObjectInspectorScreen() = default;

void ObjectInspectorScreen::reload() {
    stopTest();
    m_recipes.clear();
    m_savedJson.clear();
    m_error.clear();
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::path& file : objectRecipeFiles(m_options.objectsDirectory)) {
        std::string error;
        if (std::optional<ObjectRecipe> recipe = readObjectRecipe(file, &error)) {
            m_savedJson.push_back(objectRecipeJson(*recipe));
            m_recipes.push_back(std::move(*recipe));
            files.push_back(file);
        } else {
            m_error = file.filename().string() + ": " + error;
        }
    }
    m_files = std::move(files);
    m_selected = m_recipes.empty() ? 0 : std::min(m_selected, m_recipes.size() - 1);
    regenerate();
    frame(m_yawDegrees, m_pitchDegrees, false);
}

void ObjectInspectorScreen::select(std::size_t index) {
    if (index >= m_recipes.size() || index == m_selected) return;
    stopTest();
    m_selected = index;
    regenerate();
    frame(m_yawDegrees, m_pitchDegrees, true);
}

ObjectRecipe* ObjectInspectorScreen::recipe() {
    return m_selected < m_recipes.size() ? &m_recipes[m_selected] : nullptr;
}

bool ObjectInspectorScreen::modified() const {
    return m_selected < m_recipes.size() && objectRecipeJson(m_recipes[m_selected]) != m_savedJson[m_selected];
}

void ObjectInspectorScreen::regenerate() {
    m_object.reset();
    m_display = {};
    m_dirty = true;
    if (m_selected >= m_recipes.size()) return;
    std::string error;
    m_object = generateObject(m_recipes[m_selected], &error);
    if (!m_object) {
        m_error = error;
        stopTest();
        return;
    }
    m_error.clear();
    // De pe no chao: a base em z = 0.
    m_display = m_object->mesh;
    for (Vec3& position : m_display.positions) position.z += m_object->size.z * 0.5f;
    // Mudou com a fisica rodando: o roteiro recomeca com o objeto novo.
    if (m_lab && m_lab->active()) launchTest(false);
}

bool ObjectInspectorScreen::save(std::string* error) {
    if (m_selected >= m_recipes.size()) return false;
    std::string message;
    if (!writeObjectRecipe(m_recipes[m_selected], m_files[m_selected], &message)) {
        notify("Não foi possível salvar: " + message);
        if (error != nullptr) *error = message;
        return false;
    }
    m_savedJson[m_selected] = objectRecipeJson(m_recipes[m_selected]);
    notify("Receita salva em " + m_files[m_selected].filename().string());
    return true;
}

void ObjectInspectorScreen::revert() {
    if (m_selected >= m_recipes.size()) return;
    std::string error;
    if (std::optional<ObjectRecipe> recipe = readObjectRecipe(m_files[m_selected], &error)) {
        m_recipes[m_selected] = std::move(*recipe);
        regenerate();
    } else {
        notify("Não foi possível reler a receita: " + error);
    }
}

void ObjectInspectorScreen::setView(View view) {
    m_view = view;
    m_dirty = true;
}

void ObjectInspectorScreen::setShowCollider(bool show) {
    m_showCollider = show;
    m_dirty = true;
}

void ObjectInspectorScreen::notify(std::string text) {
    m_notice = std::move(text);
    m_noticeSeconds = 5.0f;
}

void ObjectInspectorScreen::startTest(ObjectTestKind3D kind) {
    m_test.kind = kind;
    launchTest(true);
}

void ObjectInspectorScreen::launchTest(bool reframe) {
    if (!m_object || m_options.physics == nullptr || m_options.materials == nullptr) return;
    if (!m_lab) m_lab = std::make_unique<ObjectPhysicsLab3D>(*m_options.physics, *m_options.materials);
    std::string error;
    if (!m_lab->start(*m_object, m_test, &error)) {
        notify("O teste não começou: " + error);
        stopTest();
        return;
    }
    m_paused = false;
    buildLabScene();
    if (reframe) frame(m_yawDegrees, m_pitchDegrees, true);
}

void ObjectInspectorScreen::stopTest() {
    if (m_lab) m_lab->stop();
    m_labMesh = {};
    m_labPieces.clear();
    m_labLocal.clear();
    m_labPositions.clear();
    m_dirty = true;
}

void ObjectInspectorScreen::buildLabScene() {
    m_labMesh = {};
    m_labPieces.clear();
    m_labShadows.clear();
    m_labLocal.clear();
    if (!m_lab || !m_lab->active() || !m_object) return;
    Mesh& scene = m_labMesh;
    scene.parts = { MeshPart { "cena", "", {}, {}, true } };
    scene.textures = m_object->mesh.textures;
    scene.materials = m_object->mesh.materials;
    const auto materialIndex = [&](const char* name, Color color) {
        const int found = scene.findMaterial(name);
        if (found >= 0) return static_cast<std::uint16_t>(found);
        scene.materials.push_back({ name, color, -1 });
        return static_cast<std::uint16_t>(scene.materials.size() - 1);
    };
    // Uma peca: as posicoes locais (as do corpo) e os triangulos dela; com
    // "flat", toda num material so.
    const auto append = [&](const Mesh& source, std::optional<std::uint16_t> flat, bool moving) {
        const auto offset = static_cast<std::uint32_t>(scene.positions.size());
        if (moving) m_labPieces.push_back({ scene.positions.size(), source.positions.size() });
        for (const Vec3& position : source.positions) {
            scene.positions.push_back(position);
            m_labLocal.push_back(position);
        }
        const bool uv = source.hasUv() && !flat;
        for (std::size_t t = 0; t < source.triangles.size(); ++t) {
            const Triangle& triangle = source.triangles[t];
            scene.triangles.push_back({ triangle[0] + offset, triangle[1] + offset, triangle[2] + offset });
            scene.trianglePart.push_back(0);
            scene.triangleMaterial.push_back(flat ? *flat : source.triangleMaterial[t]);
            scene.triangleUv.push_back(uv ? source.triangleUv[t] : std::array<Uv, 3> {});
        }
    };
    // Os corpos, na ordem do laboratorio (os objetos e a bola).
    for (const ObjectTestBody3D& body : m_lab->bodies()) {
        if (!body.ball) {
            append(m_object->mesh, std::nullopt, true);
            continue;
        }
        Mesh ball;
        MeshBuilder(ball).part("bola").material("bola", rgb(236, 236, 230)).ellipsoid({}, { body.radius, body.radius, body.radius }, 24, 16);
        append(ball, materialIndex("bola", rgb(236, 236, 230)), true);
    }
    // As sombras (as posicoes saem a cada quadro, em poseLabScene).
    const std::uint16_t shade = materialIndex("sombra", rgb(4, 6, 9));
    for (std::size_t b = 0; b < m_lab->bodies().size(); ++b) {
        const auto first = static_cast<std::uint32_t>(scene.positions.size());
        m_labShadows.push_back(first);
        for (int k = 0; k <= ShadowRim; ++k) {
            scene.positions.emplace_back();
            m_labLocal.emplace_back();
        }
        for (int k = 0; k < ShadowRim; ++k) {
            scene.triangles.push_back({ first, first + 1 + static_cast<std::uint32_t>(k),
                first + 1 + static_cast<std::uint32_t>((k + 1) % ShadowRim) });
            scene.trianglePart.push_back(0);
            scene.triangleMaterial.push_back(shade);
            scene.triangleUv.push_back({});
        }
    }
    // O cenario parado (a rampa; o chao e a grade).
    for (std::size_t s = 1; s < m_lab->statics().size(); ++s) {
        const ObjectTestStatic3D& piece = m_lab->statics()[s];
        Mesh box;
        MeshBuilder(box).part("rampa").material("concreto", rgb(112, 120, 130)).box({}, piece.halfExtents);
        for (Vec3& position : box.positions) position = piece.center + piece.orientation.rotate(position);
        append(box, materialIndex("concreto", rgb(112, 120, 130)), false);
    }
    m_labPositions = m_labLocal;
    poseLabScene();
}

void ObjectInspectorScreen::poseLabScene() {
    if (!m_lab) return;
    const std::vector<ObjectTestBody3D>& bodies = m_lab->bodies();
    for (std::size_t b = 0; b < bodies.size() && b < m_labPieces.size(); ++b) {
        const LabPiece& piece = m_labPieces[b];
        for (std::size_t i = piece.first; i < piece.first + piece.count; ++i)
            m_labPositions[i] = bodies[b].position + bodies[b].orientation.rotate(m_labLocal[i]);
    }
    // As sombras: o contorno de cima de cada corpo, no chao.
    for (std::size_t b = 0; b < bodies.size() && b < m_labShadows.size(); ++b) {
        const ObjectTestBody3D& body = bodies[b];
        std::vector<Vec3> rim;
        if (body.ball) {
            for (int k = 0; k < ShadowRim; ++k) {
                const float angle = 2.0f * Pi * static_cast<float>(k) / static_cast<float>(ShadowRim);
                rim.push_back({ body.position.x + body.radius * std::cos(angle), body.position.y + body.radius * std::sin(angle), 0.0f });
            }
        } else if (m_object) {
            std::vector<Vec3> corners;
            for (const ObjectCollider& shape : m_object->physics.colliders)
                if (shape.shape == ObjectColliderShape::Box)
                    for (const Vec3& corner : boxCorners(body.position + body.orientation.rotate(shape.center),
                             (body.orientation * shape.orientation).normalized(), shape.halfExtents))
                        corners.push_back({ corner.x, corner.y, 0.0f });
            rim = convexOutline(std::move(corners));
        }
        const std::size_t first = m_labShadows[b];
        if (rim.empty()) {
            for (int k = 0; k <= ShadowRim; ++k) m_labPositions[first + static_cast<std::size_t>(k)] = { 0.0f, 0.0f, -1.0f };
            continue;
        }
        // Mais lados que o leque: um sim, outro nao; menos: o ultimo repete.
        while (rim.size() > static_cast<std::size_t>(ShadowRim)) {
            std::vector<Vec3> fewer;
            for (std::size_t i = 0; i < rim.size(); i += 2) fewer.push_back(rim[i]);
            rim = std::move(fewer);
        }
        Vec3 center {};
        for (const Vec3& point : rim) center += point;
        center = center * (1.0f / static_cast<float>(rim.size()));
        m_labPositions[first] = { center.x, center.y, ShadowLift };
        for (int k = 0; k < ShadowRim; ++k) {
            const Vec3& point = rim[std::min(rim.size() - 1, static_cast<std::size_t>(k))];
            m_labPositions[first + 1 + static_cast<std::size_t>(k)] = { point.x, point.y, ShadowLift };
        }
    }
    m_dirty = true;
}

Box3 ObjectInspectorScreen::sceneBounds() const {
    Box3 box;
    if (!m_object) {
        box.include({ -0.5f, -0.5f, 0.0f });
        box.include({ 0.5f, 0.5f, 1.0f });
        return box;
    }
    const Vec3 half = m_object->size * 0.5f;
    box.include({ -half.x, -half.y, 0.0f });
    box.include({ half.x, half.y, m_object->size.z });
    if (!m_lab || !m_lab->active()) return box;
    // O roteiro inteiro: por onde o objeto vai passar.
    const float reach = std::max({ half.x, half.y, half.z });
    switch (m_test.kind) {
    case ObjectTestKind3D::Drop: box.include({ 0.0f, 0.0f, m_test.height + m_object->size.z + reach }); break;
    case ObjectTestKind3D::Throw:
        box.include({ -2.0f - reach, -1.0f, 0.0f });
        box.include({ 4.5f, 1.0f, 1.0f + m_object->size.z + 0.8f });
        break;
    case ObjectTestKind3D::Ramp:
        for (std::size_t s = 1; s < m_lab->statics().size(); ++s) {
            const ObjectTestStatic3D& piece = m_lab->statics()[s];
            for (const Vec3& corner : boxCorners(piece.center, piece.orientation, piece.halfExtents)) box.include(corner);
        }
        box.include({ 3.5f, 0.0f, 0.0f });
        break;
    case ObjectTestKind3D::Stack:
        box.include({ 0.0f, 0.0f, m_object->size.z * static_cast<float>(std::clamp(m_test.count, 1, 8)) });
        break;
    case ObjectTestKind3D::BallShot:
        box.include({ -3.2f, -0.4f, 0.0f });
        box.include({ 1.5f, 0.4f, 0.5f });
        break;
    }
    return box;
}

void ObjectInspectorScreen::frame(float yawDegrees, float pitchDegrees, bool animate) {
    m_yawDegrees = yawDegrees;
    m_pitchDegrees = pitchDegrees;
    m_goal = frameBounds(sceneBounds(), yawDegrees, pitchDegrees, m_aspect, FrameMargin, m_orthographic);
    if (!animate) m_camera = m_goal;
    m_animating = animate;
    m_userAdjusted = false;
    m_dirty = true;
}

void ObjectInspectorScreen::update(float deltaSeconds) {
    if (m_noticeSeconds > 0.0f) m_noticeSeconds -= deltaSeconds;
    if (m_interactionSeconds > 0.0f) {
        m_interactionSeconds -= deltaSeconds;
        if (m_interactionSeconds <= 0.0f) m_dirty = true;
    }
    if (m_animating) {
        // A camera vai ao alvo como na inspecao de jogador.
        const float alpha = 1.0f - std::exp(-std::max(0.0f, deltaSeconds) * 9.0f);
        m_camera.target = lerp(m_camera.target, m_goal.target, alpha);
        m_camera.yaw = wrapAngle(m_camera.yaw + wrapAngle(m_goal.yaw - m_camera.yaw) * alpha);
        m_camera.pitch += (m_goal.pitch - m_camera.pitch) * alpha;
        m_camera.distance = std::exp(std::log(m_camera.distance) + (std::log(m_goal.distance) - std::log(m_camera.distance)) * alpha);
        m_camera.orthoHalfHeight = std::exp(std::log(m_camera.orthoHalfHeight)
            + (std::log(m_goal.orthoHalfHeight) - std::log(m_camera.orthoHalfHeight)) * alpha);
        m_camera.orthographic = m_goal.orthographic;
        const bool settled = distance(m_camera.target, m_goal.target) < 1e-4f && std::abs(wrapAngle(m_goal.yaw - m_camera.yaw)) < 1e-3f
            && std::abs(m_goal.pitch - m_camera.pitch) < 1e-3f
            && std::abs(m_goal.distance - m_camera.distance) < 1e-4f * std::max(1.0f, m_goal.distance);
        if (settled) {
            m_camera = m_goal;
            m_animating = false;
        }
        m_interactionSeconds = std::max(m_interactionSeconds, m_animating ? InteractionHoldSeconds : 0.0f);
        m_dirty = true;
    }
    if (m_lab && m_lab->active() && !m_paused) {
        m_lab->advance(deltaSeconds * m_timeScale);
        poseLabScene();
    }
}

Image ObjectInspectorScreen::renderTexture(int width, int height) const {
    Image image(width, height, rgb(9, 12, 16));
    // Xadrez discreto atras (o transparente).
    const int cell = std::max(8, height / 48);
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (((x / cell) + (y / cell)) % 2 == 0) image.setPixel(x, y, rgb(13, 17, 22));
    if (!m_object || m_object->pixelArt.empty()) return image;
    const Image& art = m_object->pixelArt;
    // Na maior escala inteira que cabe: o pixel nitido.
    const int margin = std::max(16, height / 12);
    const int scale = std::max(1, std::min((width - 2 * margin) / art.width(), (height - 2 * margin) / art.height()));
    const int x0 = (width - art.width() * scale) / 2, y0 = (height - art.height() * scale) / 2;
    for (int y = 0; y < art.height() * scale; ++y)
        for (int x = 0; x < art.width() * scale; ++x) image.setPixel(x0 + x, y0 + y, art.pixel(x / scale, y / scale));
    return image;
}

bool ObjectInspectorScreen::render(int width, int height) {
    const bool running = m_lab && m_lab->active() && !m_paused;
    const bool fast = m_interactionSeconds > 0.0f || m_animating || running;
    if (!m_dirty && width == m_lastWidth && height == m_lastHeight && fast == m_lastFast && !m_image.empty()) return false;
    m_lastWidth = width;
    m_lastHeight = height;
    m_lastFast = fast;
    m_dirty = false;
    if (m_view == View::Texture) {
        m_image = renderTexture(width, height);
        return true;
    }
    const bool lab = m_lab && m_lab->active() && !m_labMesh.empty();
    InspectionStyle style;
    style.surface = surfaceFor(m_view);
    style.skeleton = false;
    style.joints = false;
    style.grid = m_grid;
    // A grade cobre o roteiro inteiro (o arremesso vai longe); as sombras
    // vistas de baixo nao sao avesso.
    if (lab) {
        style.backfaces = false;
        const Box3 bounds = sceneBounds();
        const Vec3 size = bounds.size();
        style.gridExtent = std::max(2.5f, 0.5f * std::max(size.x, size.y) + 1.0f);
    }
    style.hud = false;
    style.gizmoLegend = false;
    style.supersample = fast ? 1 : 2;
    InspectionScene scene;
    scene.skeleton = &m_ground;
    if (lab) {
        scene.mesh = &m_labMesh;
        scene.meshPositions = m_labPositions;
    } else if (m_object) {
        scene.mesh = &m_display;
    }
    std::vector<Highlight> highlights;
    if (m_showCollider && m_object) {
        Highlight collider;
        collider.color = rgb(64, 214, 255);
        const auto add = [&](Vec3 position, Quaternion orientation) {
            for (const ObjectCollider& shape : m_object->physics.colliders)
                if (shape.shape == ObjectColliderShape::Box)
                    boxSegments(position + orientation.rotate(shape.center), (orientation * shape.orientation).normalized(),
                        shape.halfExtents, collider.segments);
        };
        if (lab) {
            for (const ObjectTestBody3D& body : m_lab->bodies())
                if (!body.ball) add(body.position, body.orientation);
        } else {
            add({ 0.0f, 0.0f, m_object->size.z * 0.5f }, {});
        }
        highlights.push_back(std::move(collider));
    }
    scene.highlights = highlights;
    m_image = renderInspection(scene, style, m_camera, width, height);
    return true;
}

void ObjectInspectorScreen::draw(CanvasTarget& canvas, float uiScale) {
    m_uiScale = uiScale;
    beginFullscreenPanel("MatterEngineObjectInspector");
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    drawWorkbenchBackdrop(drawList, io.DisplaySize, uiScale);
    drawWorkbenchHeader(drawList, io.DisplaySize, uiScale, "OBJETOS");

    const float margin = ui(12.0f);
    const float top = headerHeight(uiScale) + margin;
    const float height = io.DisplaySize.y - top - margin;
    const float sidebar = std::max(sidebarWidth(uiScale), ui(300.0f));
    ImGui::SetCursorPos({ margin, top });
    ImGui::BeginChild("ObjectInspectorSidebar", { sidebar, height }, true, ImGuiWindowFlags_NoScrollbar);
    drawSidebar(height);
    ImGui::EndChild();

    const float left = margin * 2.0f + sidebar;
    ImGui::SetCursorPos({ left, top });
    ImGui::BeginChild("ObjectInspectorWorkspace", { io.DisplaySize.x - left - margin, height }, true, ImGuiWindowFlags_NoScrollbar);
    panelHeader("VISUALIZADOR");
    drawToolbar();
    drawViewport(canvas, ui(30.0f));
    drawFooter();
    ImGui::EndChild();
    ImGui::End();
}

void ObjectInspectorScreen::drawSidebar(float height) {
    panelHeader("CATÁLOGO");
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    for (std::size_t i = 0; i < m_recipes.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        const std::string label = UI::FontAwesome::label(UI::FontAwesome::Cube, m_recipes[i].displayName);
        if (navigationButton(label.c_str(), i == m_selected, { -1.0f, ui(34.0f) })) select(i);
        ImGui::PopID();
    }
    if (m_recipes.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(m_error.empty() ? MutedText : WarningText, "%s",
            m_error.empty() ? "Nenhuma receita em assets/objects." : m_error.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::Dummy({ 1.0f, ui(8.0f) });
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (tabButton("Propriedades", m_tab == 0, { half, ui(30.0f) })) m_tab = 0;
    ImGui::SameLine();
    if (tabButton("Teste físico", m_tab == 1, { half, ui(30.0f) })) m_tab = 1;
    ImGui::Dummy({ 1.0f, ui(2.0f) });
    // A aba rola; os botoes dela e a volta ficam embaixo.
    const float actions = m_tab == 0 ? ui(44.0f) : ui(84.0f);
    const float bottom = actions + ui(58.0f);
    ImGui::BeginChild("ObjectInspectorTab", { -1.0f, std::max(ui(120.0f), ImGui::GetContentRegionAvail().y - bottom) }, false);
    if (m_tab == 0) drawProperties();
    else drawPhysics();
    ImGui::EndChild();
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), height - bottom));
    drawActions();
    const std::string back = UI::FontAwesome::label(UI::FontAwesome::ArrowLeft, "Menu principal");
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), height - ui(58.0f)));
    if (navigationButton(back.c_str(), false, { -1.0f, ui(40.0f) }) && m_onBack) {
        stopTest();
        m_onBack();
    }
}

void ObjectInspectorScreen::drawProperties() {
    ObjectRecipe* current = recipe();
    const ObjectType* type = current != nullptr ? findObjectType(current->type) : nullptr;
    if (current == nullptr || type == nullptr) {
        if (!m_error.empty()) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextColored(WarningText, "%s", m_error.c_str());
            ImGui::PopTextWrapPos();
        }
        return;
    }
    bool changed = false;
    const float labelWidth = ui(104.0f);
    // Um grupo por vez: o titulo e a grade dele.
    std::size_t first = 0;
    while (first < type->parameters.size()) {
        const std::string& group = type->parameters[first].group;
        std::size_t last = first;
        while (last < type->parameters.size() && type->parameters[last].group == group) ++last;
        ImGui::Dummy({ 1.0f, ui(4.0f) });
        sectionLabel(group.c_str());
        ImGui::PushID(group.c_str());
        if (beginPropertyTable("##grupo", labelWidth)) {
            // O material vai junto da fisica.
            if (group == "Física" && m_options.materials != nullptr) {
                propertyRow("Material");
                if (ImGui::BeginCombo("##material", materialName(current->material).c_str())) {
                    std::vector<std::string> ids;
                    for (const auto& [id, material] : m_options.materials->all()) ids.push_back(id);
                    std::sort(ids.begin(), ids.end(),
                        [](const std::string& a, const std::string& b) { return materialName(a) < materialName(b); });
                    for (const std::string& id : ids)
                        if (ImGui::Selectable(materialName(id).c_str(), id == current->material)) {
                            current->material = id;
                            changed = true;
                        }
                    ImGui::EndCombo();
                }
            }
            for (std::size_t p = first; p < last; ++p) {
                const ObjectParameterSpec& spec = type->parameters[p];
                ImGui::PushID(spec.key.c_str());
                propertyRow(spec.label.c_str());
                switch (spec.kind) {
                case ObjectParameterKind::Number: {
                    float value = objectNumber(*current, *type, spec.key);
                    const std::string format = spec.unit.empty() ? "%.2f" : "%.2f " + spec.unit;
                    if (ImGui::SliderFloat("##valor", &value, spec.minimum, spec.maximum, format.c_str(), ImGuiSliderFlags_AlwaysClamp)) {
                        current->numbers[spec.key] = value;
                        changed = true;
                    }
                    break;
                }
                case ObjectParameterKind::Integer: {
                    int value = objectInteger(*current, *type, spec.key);
                    const std::string format = spec.unit.empty() ? "%d" : "%d " + spec.unit;
                    if (ImGui::SliderInt("##valor", &value, static_cast<int>(spec.minimum), static_cast<int>(spec.maximum),
                            format.c_str(), ImGuiSliderFlags_AlwaysClamp)) {
                        current->numbers[spec.key] = static_cast<float>(value);
                        changed = true;
                    }
                    break;
                }
                case ObjectParameterKind::Toggle: {
                    bool value = objectToggle(*current, *type, spec.key);
                    if (ImGui::Checkbox("##valor", &value)) {
                        current->numbers[spec.key] = value ? 1.0f : 0.0f;
                        changed = true;
                    }
                    break;
                }
                case ObjectParameterKind::Color: {
                    const Color color = objectColor(*current, *type, spec.key);
                    float values[3] { color.r, color.g, color.b };
                    if (ImGui::ColorEdit3("##valor", values, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel)) {
                        current->colors[spec.key] = { values[0], values[1], values[2], 1.0f };
                        changed = true;
                    }
                    ImGui::SameLine();
                    ImGui::TextColored(MutedText, "%s", hexColor(objectColor(*current, *type, spec.key)).c_str());
                    break;
                }
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::PopID();
        first = last;
    }
    if (changed) regenerate();
    if (!m_error.empty()) {
        ImGui::Dummy({ 1.0f, ui(6.0f) });
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(WarningText, "%s", m_error.c_str());
        ImGui::PopTextWrapPos();
    }
}

void ObjectInspectorScreen::drawPhysics() {
    const bool available = m_object && m_options.physics != nullptr && m_options.materials != nullptr;
    const bool running = m_lab && m_lab->active();
    ImGui::BeginDisabled(!available);
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    sectionLabel("Roteiro");
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float third = (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
    for (std::size_t i = 0; i < TestButtons.size(); ++i) {
        if (i % 3 != 0) ImGui::SameLine();
        if (tabButton(TestButtons[i].label, m_test.kind == TestButtons[i].kind, { third, ui(28.0f) })
            && m_test.kind != TestButtons[i].kind) {
            m_test.kind = TestButtons[i].kind;
            if (running) launchTest(true);
        }
    }
    ImGui::Dummy({ 1.0f, ui(4.0f) });
    sectionLabel("Ajustes");
    if (beginPropertyTable("##ajustes", ui(104.0f))) {
        const auto slider = [&](const char* label, float* value, float minimum, float maximum, const char* format) {
            ImGui::PushID(label);
            propertyRow(label);
            ImGui::SliderFloat("##valor", value, minimum, maximum, format, ImGuiSliderFlags_AlwaysClamp);
            ImGui::PopID();
        };
        switch (m_test.kind) {
        case ObjectTestKind3D::Drop:
            slider("Altura", &m_test.height, 0.2f, 6.0f, "%.2f m");
            slider("Inclinação", &m_test.tiltDegrees, 0.0f, 45.0f, "%.0f°");
            break;
        case ObjectTestKind3D::Throw:
            slider("Velocidade", &m_test.speed, 1.0f, 15.0f, "%.1f m/s");
            slider("Giro", &m_test.spin, 0.0f, 15.0f, "%.1f rad/s");
            break;
        case ObjectTestKind3D::Ramp: slider("Inclinação", &m_test.rampDegrees, 5.0f, 60.0f, "%.0f°"); break;
        case ObjectTestKind3D::Stack:
            propertyRow("Quantidade");
            ImGui::SliderInt("##quantidade", &m_test.count, 2, 8, "%d", ImGuiSliderFlags_AlwaysClamp);
            break;
        case ObjectTestKind3D::BallShot: slider("Velocidade", &m_test.speed, 2.0f, 35.0f, "%.1f m/s"); break;
        }
        propertyRow("Ritmo");
        const std::array<std::pair<const char*, float>, 3> scales { { { "1×", 1.0f }, { "½×", 0.5f }, { "¼×", 0.25f } } };
        const float width = (ImGui::GetContentRegionAvail().x - 2.0f * spacing) / 3.0f;
        for (std::size_t i = 0; i < scales.size(); ++i) {
            if (i > 0) ImGui::SameLine();
            if (tabButton(scales[i].first, std::abs(m_timeScale - scales[i].second) < 1e-3f, { width, ui(24.0f) }))
                m_timeScale = scales[i].second;
        }
        ImGui::EndTable();
    }
    ImGui::EndDisabled();
    if (!running) return;
    ImGui::Dummy({ 1.0f, ui(6.0f) });
    sectionLabel("Medidas");
    const ObjectTestMetrics3D& metrics = m_lab->metrics();
    if (beginPropertyTable("##medidas", ui(132.0f))) {
        metricRow("Tempo", secondsText(metrics.elapsed));
        switch (m_test.kind) {
        case ObjectTestKind3D::Drop:
            metricRow("Toque no chão", speedText(metrics.firstImpactSpeed));
            metricRow("Quique", metrics.firstImpactSpeed < 0.0f ? std::string("-") : formatLength(metrics.bounceHeight));
            break;
        case ObjectTestKind3D::Throw:
            metricRow("Percurso", formatLength(metrics.travel));
            metricRow("Maior velocidade", speedText(metrics.peakSpeed));
            break;
        case ObjectTestKind3D::Ramp:
            metricRow("Escorregou", formatLength(std::max(0.0f, metrics.slide)));
            metricRow("Maior velocidade", speedText(metrics.peakSpeed));
            break;
        case ObjectTestKind3D::Stack:
            metricRow("Desvio do topo", formatLength(metrics.stackDrift));
            metricRow("Pilha", metrics.toppled ? "caiu" : "de pé");
            break;
        case ObjectTestKind3D::BallShot:
            metricRow("Bola após o toque", speedText(metrics.ballSpeedAfter));
            metricRow("Objeto após o toque", speedText(metrics.objectSpeedAfter));
            break;
        }
        metricRow("Parou", metrics.settled ? "em " + secondsText(metrics.settleTime) : std::string("ainda não"));
        if (metrics.settled && !metrics.restFace.empty()) metricRow("Apoiado na", metrics.restFace);
        ImGui::EndTable();
    }
}

void ObjectInspectorScreen::drawActions() {
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    const float half = (ImGui::GetContentRegionAvail().x - spacing) * 0.5f;
    if (m_tab == 0) {
        ImGui::BeginDisabled(!modified());
        if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::FloppyDisk, "Salvar receita").c_str(), { half, ui(34.0f) })) save();
        ImGui::SameLine();
        if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::RotateLeft, "Desfazer").c_str(), { half, ui(34.0f) })) revert();
        ImGui::EndDisabled();
        return;
    }
    const bool running = m_lab && m_lab->active();
    ImGui::BeginDisabled(!m_object || m_options.physics == nullptr);
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Play, running ? "Reiniciar" : "Rodar").c_str(), { half, ui(34.0f) }))
        launchTest(!running);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    const std::string pause = m_paused ? UI::FontAwesome::label(UI::FontAwesome::Play, "Continuar")
                                       : UI::FontAwesome::label(UI::FontAwesome::Pause, "Pausar");
    if (ImGui::Button(pause.c_str(), { half, ui(34.0f) })) m_paused = !m_paused;
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Stop, "Parar").c_str(), { -1.0f, ui(34.0f) })) {
        stopTest();
        frame(m_yawDegrees, m_pitchDegrees, true);
    }
    ImGui::EndDisabled();
}

void ObjectInspectorScreen::drawToolbar() {
    ImGui::BeginDisabled(!m_object);
    const bool texture = m_view == View::Texture;
    ImGui::BeginDisabled(texture);
    for (const ViewButton& view : ViewButtons) {
        if (ImGui::Button(view.label, { ui(78.0f), ui(30.0f) })) frame(view.yaw, view.pitch, true);
        ImGui::SameLine();
    }
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Crosshairs, "Recentrar").c_str(), { ui(116.0f), ui(30.0f) }))
        frame(m_yawDegrees, m_pitchDegrees, true);
    ImGui::EndDisabled();

    const char* current = ViewModes.front().label;
    for (const ViewMode& mode : ViewModes)
        if (mode.view == m_view) current = mode.label;
    ImGui::SetNextItemWidth(ui(190.0f));
    if (ImGui::BeginCombo("##ObjectViewMode", current)) {
        for (const ViewMode& mode : ViewModes)
            if (ImGui::Selectable(mode.label, mode.view == m_view)) setView(mode.view);
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(texture);
    if (ImGui::Checkbox("Colisor", &m_showCollider)) m_dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Grade", &m_grid)) m_dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Ortográfica", &m_orthographic)) frame(m_yawDegrees, m_pitchDegrees, true);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(UI::FontAwesome::label(UI::FontAwesome::Camera, "Salvar imagem").c_str(), { ui(150.0f), ui(30.0f) })) saveCapture();
    ImGui::EndDisabled();
    ImGui::Dummy({ 1.0f, ui(2.0f) });
}

void ObjectInspectorScreen::drawViewport(CanvasTarget& canvas, float footerHeight) {
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 size { std::max(ui(160.0f), available.x), std::max(ui(160.0f), available.y - footerHeight) };
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##ObjectViewport", size,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 end { origin.x + size.x, origin.y + size.y };
    drawList->AddRectFilled(origin, end, IM_COL32(6, 8, 11, 255), ui(4.0f));
    const float scaleX = io.DisplayFramebufferScale.x > 0.0f ? io.DisplayFramebufferScale.x : 1.0f;
    const float scaleY = io.DisplayFramebufferScale.y > 0.0f ? io.DisplayFramebufferScale.y : 1.0f;
    const int width = std::max(16, static_cast<int>(std::lround(size.x * scaleX)));
    const int height = std::max(16, static_cast<int>(std::lround(size.y * scaleY)));
    const float aspect = static_cast<float>(width) / static_cast<float>(height);
    if (std::abs(aspect - m_aspect) > 1e-3f) {
        m_aspect = aspect;
        // A tela mudou de forma: reenquadra (se a camera nao foi mexida).
        if (!m_userAdjusted) {
            m_goal = frameBounds(sceneBounds(), m_yawDegrees, m_pitchDegrees, m_aspect, FrameMargin, m_orthographic);
            if (!m_animating) m_camera = m_goal;
        }
        m_dirty = true;
    }
    if (!m_object) {
        const char* text = m_recipes.empty() ? "NENHUM OBJETO" : "O OBJETO NÃO FOI GERADO";
        const ImVec2 textSize = ImGui::CalcTextSize(text);
        drawList->AddText({ origin.x + (size.x - textSize.x) * 0.5f, origin.y + (size.y - textSize.y) * 0.5f }, UiMuted, text);
        return;
    }
    // Arrastar gira (esquerdo) ou move (direito/meio), a roda aproxima e o
    // duplo clique recentra.
    const bool camera = m_view != View::Texture;
    if (active && camera && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f)) {
            m_goal.yaw = wrapAngle(m_goal.yaw - io.MouseDelta.x * 0.008f);
            m_goal.pitch = std::clamp(m_goal.pitch + io.MouseDelta.y * 0.006f, -1.55f, 1.55f);
            m_camera.yaw = m_goal.yaw;
            m_camera.pitch = m_goal.pitch;
            m_animating = false;
            m_userAdjusted = true;
            m_interactionSeconds = InteractionHoldSeconds;
            m_dirty = true;
        } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 3.0f) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 3.0f)) {
            const float metersPerPixel = m_goal.orthographic ? 2.0f * m_goal.orthoHalfHeight / size.y
                                                             : 2.0f * m_goal.distance * std::tan(m_goal.verticalFov * 0.5f) / size.y;
            m_goal.target += m_goal.right() * (-io.MouseDelta.x * metersPerPixel) + m_goal.up() * (io.MouseDelta.y * metersPerPixel);
            m_camera.target = m_goal.target;
            m_animating = false;
            m_userAdjusted = true;
            m_interactionSeconds = InteractionHoldSeconds;
            m_dirty = true;
        }
    }
    if (hovered && camera && std::abs(io.MouseWheel) > 0.0f) {
        const float factor = std::pow(0.87f, io.MouseWheel);
        m_goal.distance = std::clamp(m_goal.distance * factor, 0.05f, 40.0f);
        m_goal.orthoHalfHeight = std::clamp(m_goal.orthoHalfHeight * factor, 0.01f, 15.0f);
        m_camera.distance = m_goal.distance;
        m_camera.orthoHalfHeight = m_goal.orthoHalfHeight;
        m_animating = false;
        m_userAdjusted = true;
        m_interactionSeconds = InteractionHoldSeconds;
        m_dirty = true;
    }
    if (hovered && camera && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) frame(m_yawDegrees, m_pitchDegrees, true);
    const bool rendered = render(width, height);
    const std::uint64_t id = canvas.present(m_image, rendered);
    drawList->AddImage(static_cast<ImTextureID>(id), origin, end);
}

void ObjectInspectorScreen::drawFooter() {
    if (m_noticeSeconds > 0.0f && !m_notice.empty()) {
        ImGui::TextColored(AccentText, "%s", m_notice.c_str());
        return;
    }
    if (!m_object) return;
    const ProceduralObject& object = *m_object;
    ImGui::TextColored(InfoText, "%s × %s × %s m  |  %s kg  |  %s  |  %zu triângulos  |  textura %d × %d px",
        formatNumber(object.size.x, 2).c_str(), formatNumber(object.size.y, 2).c_str(), formatNumber(object.size.z, 2).c_str(),
        formatNumber(object.physics.massKg, 1).c_str(), materialName(object.physics.material).c_str(), object.mesh.triangles.size(),
        object.pixelArt.width(), object.pixelArt.height());
    if (modified()) {
        ImGui::SameLine(0.0f, ui(18.0f));
        ImGui::TextColored(WarningText, "receita não salva");
    }
}

void ObjectInspectorScreen::saveCapture() {
    if (m_image.empty() || !m_object) return;
    std::error_code code;
    std::filesystem::create_directories(m_options.capturesDirectory, code);
    const std::filesystem::path path = m_options.capturesDirectory / ("objeto_" + m_object->id + "_" + timestamp() + ".png");
    std::string error;
    if (writePng(m_image, path.string(), &error)) notify("Imagem salva: " + path.string());
    else notify("Falha ao salvar: " + error);
}

} // namespace MatterEngine::Workbench
