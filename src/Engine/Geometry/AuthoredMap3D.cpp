#include "Engine/Geometry/AuthoredMap3D.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {
namespace {
constexpr float Pi = 3.14159265359f;
constexpr float Line = 0.08f;

float number(const GltfExtras &extras, const char *key) {
    const std::string *text = extras.find(key);
    if (!text)
        throw std::invalid_argument(std::string("Map marker missing ") + key);
    float value = 0;
    const auto result = std::from_chars(text->data(), text->data() + text->size(), value);
    if (result.ec != std::errc{} || result.ptr != text->data() + text->size() || !std::isfinite(value))
        throw std::invalid_argument(std::string("Invalid map marker ") + key);
    return value;
}
} // namespace

AuthoredMap3D parseAuthoredMap3D(const LoadedGltfModel &model) {
    AuthoredMap3D result;
    for (const auto &entity : model.entities) {
        const auto *type = entity.extras.find("entity_type");
        if (!type || (*type != "info_player_start" && *type != "matter_field"))
            continue;
        if (!entity.hasOrientation)
            throw std::invalid_argument("Map marker has no orientation");
        const Vec3 direction = entity.orientation.rotate({0, 1, 0});
        const float yaw = std::atan2(direction.y, direction.x);
        if (*type == "info_player_start")
            result.spawns.push_back({entity.position, yaw});
        else {
            MapField3D field{entity.position, yaw, number(entity.extras, "length_m"), number(entity.extras, "width_m")};
            if (field.length < 1 || field.length > 200 || field.width < 1 || field.width > 200 ||
                field.width >= field.length)
                throw std::invalid_argument("Invalid matter_field dimensions");
            result.fields.push_back(field);
        }
    }
    if (result.spawns.empty())
        throw std::invalid_argument("Map has no info_player_start");
    return result;
}

MeshData3D buildFutsalMarkings3D(const MapField3D &field) {
    if (field.length < 25 || field.length > 42 || field.width < 16 || field.width > 25)
        throw std::invalid_argument("Futsal markings require 25..42 x 16..25 m");
    MeshData3D mesh;
    const auto rotation = Quaternion::fromAxisAngle({0, 0, 1}, field.yaw);
    const auto quad = [&](Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
        const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
        for (Vec2 p : {a, b, c, d}) {
            MeshVertex3D vertex;
            // A paint film, not a second collision surface or a coplanar overlay.
            vertex.position = field.center + rotation.rotate({p.x, p.y, 0.003f});
            vertex.normal = {0, 0, 1};
            vertex.uv = {p.x, p.y};
            vertex.color = {.92f, .94f, .89f};
            mesh.vertices.push_back(vertex);
        }
        mesh.indices.insert(mesh.indices.end(), {first, first + 1, first + 2, first, first + 2, first + 3});
    };
    const auto rect = [&](float x0, float y0, float x1, float y1) { quad({x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}); };
    const auto arc = [&](Vec2 center, float radius, float start, float end, float width) {
        const int segments = std::max(12, static_cast<int>(std::ceil((end - start) * radius * 24)));
        for (int i = 0; i < segments; ++i) {
            const float a = start + (end - start) * float(i) / segments;
            const float b = start + (end - start) * float(i + 1) / segments;
            const float r0 = std::max(0.0f, radius - width), r1 = radius;
            quad({center.x + r0 * std::cos(a), center.y + r0 * std::sin(a)},
                 {center.x + r1 * std::cos(a), center.y + r1 * std::sin(a)},
                 {center.x + r1 * std::cos(b), center.y + r1 * std::sin(b)},
                 {center.x + r0 * std::cos(b), center.y + r0 * std::sin(b)});
        }
    };
    const float x = field.length * .5f, y = field.width * .5f;
    rect(-x, -y, x, -y + Line);
    rect(-x, y - Line, x, y);
    rect(-x, -y + Line, -x + Line, y - Line);
    rect(x - Line, -y + Line, x, y - Line);
    rect(-Line * .5f, -y + Line, Line * .5f, y - Line);
    arc({0, 0}, 3, 0, 2 * Pi, Line);
    arc({0, 0}, .06f, 0, 2 * Pi, .06f);
    for (float side : {-1.0f, 1.0f}) {
        const float goal = side * x;
        // Radius 6 m from the OUTSIDE of the 8 cm posts: connector = 3.16 m.
        if (side < 0) {
            arc({goal, 1.58f}, 6, 0, Pi * .5f, Line);
            arc({goal, -1.58f}, 6, -Pi * .5f, 0, Line);
            rect(goal + 6 - Line, -1.58f, goal + 6, 1.58f);
        } else {
            arc({goal, 1.58f}, 6, Pi * .5f, Pi, Line);
            arc({goal, -1.58f}, 6, Pi, Pi * 1.5f, Line);
            rect(goal - 6, -1.58f, goal - 6 + Line, 1.58f);
        }
        for (float distance : {6.0f, 10.0f})
            arc({goal - side * distance, 0}, .06f, 0, 2 * Pi, .06f);
        for (float dy : {-5.0f, 5.0f})
            arc({goal - side * 10, dy}, .04f, 0, 2 * Pi, .04f);
        rect(goal - side * 5 - Line * .5f, -.2f, goal - side * 5 + Line * .5f, .2f);
        for (float sy : {-1.0f, 1.0f}) {
            const float start = side < 0 ? (sy < 0 ? 0 : -Pi * .5f) : (sy < 0 ? Pi * .5f : Pi);
            arc({goal, sy * y}, .25f, start, start + Pi * .5f, Line);
            const float ex = goal + (side < 0 ? -.45f : .05f);
            rect(ex, sy * (y - 5.25f) - Line * .5f, ex + .4f, sy * (y - 5.25f) + Line * .5f);
        }
    }
    // The authored benches are on local +Y. Each substitution zone is five
    // metres.
    for (float xMark : {-10.0f, -5.0f, 5.0f, 10.0f})
        rect(xMark - Line * .5f, y - .4f, xMark + Line * .5f, y + .4f);
    recomputeBounds(mesh);
    return mesh;
}
} // namespace MatterEngine
