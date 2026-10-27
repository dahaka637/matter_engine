#include "Engine/Appearance/Compose/VectorShape.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <limits>

namespace MatterEngine::Appearance {
namespace {

constexpr int CurveSegments = 16;

struct Reader {
    std::string_view text;
    std::size_t at = 0;

    void skip() {
        while (at < text.size() && (std::isspace(static_cast<unsigned char>(text[at])) || text[at] == ',')) ++at;
    }
    [[nodiscard]] bool done() {
        skip();
        return at >= text.size();
    }
    [[nodiscard]] bool command(char& out) {
        skip();
        if (at < text.size() && std::isalpha(static_cast<unsigned char>(text[at]))) {
            out = text[at++];
            return true;
        }
        return false;
    }
    [[nodiscard]] bool number(float& out) {
        skip();
        if (at >= text.size()) return false;
        const std::string piece(text.substr(at, std::min<std::size_t>(64, text.size() - at)));
        char* end = nullptr;
        out = std::strtof(piece.c_str(), &end);
        if (end == piece.c_str()) return false;
        at += static_cast<std::size_t>(end - piece.c_str());
        return true;
    }
    [[nodiscard]] bool point(Vec2& out) { return number(out.x) && number(out.y); }
};

} // namespace

bool VectorShape::contains(Vec2 p) const {
    if (p.x < min.x || p.y < min.y || p.x > max.x || p.y > max.y) return false;
    int winding = 0;
    for (const std::vector<Vec2>& contour : contours) {
        const std::size_t n = contour.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = contour[i], b = contour[(i + 1) % n];
            if ((a.y <= p.y) == (b.y <= p.y)) continue;
            const float x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) winding += b.y > a.y ? 1 : -1;
        }
    }
    return evenOdd ? (winding & 1) != 0 : winding != 0;
}

std::vector<std::pair<float, int>> VectorShape::crossings(float y) const {
    std::vector<std::pair<float, int>> out;
    if (y < min.y || y > max.y) return out;
    for (const std::vector<Vec2>& contour : contours) {
        const std::size_t n = contour.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = contour[i], b = contour[(i + 1) % n];
            if ((a.y <= y) == (b.y <= y)) continue;
            out.push_back({ a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y), b.y > a.y ? 1 : -1 });
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

VectorShape VectorShape::transformed(Vec2 a, Vec2 b, Vec2 c) const {
    VectorShape out;
    out.evenOdd = evenOdd;
    out.contours.reserve(contours.size());
    for (const std::vector<Vec2>& contour : contours) {
        std::vector<Vec2>& moved = out.contours.emplace_back();
        moved.reserve(contour.size());
        for (const Vec2 p : contour) moved.push_back({ a.x * p.x + b.x * p.y + c.x, a.y * p.x + b.y * p.y + c.y });
    }
    out.updateBounds();
    return out;
}

void VectorShape::append(const VectorShape& other) {
    contours.insert(contours.end(), other.contours.begin(), other.contours.end());
    updateBounds();
}

void VectorShape::updateBounds() {
    constexpr float big = std::numeric_limits<float>::max();
    min = { big, big };
    max = { -big, -big };
    for (const std::vector<Vec2>& contour : contours)
        for (const Vec2 p : contour) {
            min = { std::min(min.x, p.x), std::min(min.y, p.y) };
            max = { std::max(max.x, p.x), std::max(max.y, p.y) };
        }
    if (contours.empty()) min = max = { 0.0f, 0.0f };
}

std::optional<VectorShape> parsePathData(std::string_view data, bool evenOdd, std::string* error) {
    const auto fail = [&](const std::string& message) -> std::optional<VectorShape> {
        if (error != nullptr) *error = "caminho: " + message;
        return std::nullopt;
    };
    VectorShape shape;
    shape.evenOdd = evenOdd;
    Reader reader { data };
    std::vector<Vec2> current;
    Vec2 pen { 0.0f, 0.0f }, start { 0.0f, 0.0f };
    const auto close = [&] {
        if (current.size() >= 3) shape.contours.push_back(std::move(current));
        current.clear();
    };
    char command = 0;
    while (!reader.done()) {
        char next = 0;
        if (reader.command(next)) command = next;
        else if (command == 0) return fail("comeca sem comando");
        const bool relative = std::islower(static_cast<unsigned char>(command)) != 0;
        const Vec2 base = relative ? pen : Vec2 { 0.0f, 0.0f };
        switch (std::toupper(static_cast<unsigned char>(command))) {
        case 'M': {
            Vec2 p;
            if (!reader.point(p)) return fail("M sem ponto");
            close();
            pen = start = base + p;
            current.push_back(pen);
            // (Os pares seguintes de um M sao L.)
            command = relative ? 'l' : 'L';
            break;
        }
        case 'L': {
            Vec2 p;
            if (!reader.point(p)) return fail("L sem ponto");
            pen = base + p;
            current.push_back(pen);
            break;
        }
        case 'H': {
            float x = 0.0f;
            if (!reader.number(x)) return fail("H sem numero");
            pen.x = (relative ? pen.x : 0.0f) + x;
            current.push_back(pen);
            break;
        }
        case 'V': {
            float y = 0.0f;
            if (!reader.number(y)) return fail("V sem numero");
            pen.y = (relative ? pen.y : 0.0f) + y;
            current.push_back(pen);
            break;
        }
        case 'Q': {
            Vec2 c, p;
            if (!reader.point(c) || !reader.point(p)) return fail("Q incompleto");
            c = base + c;
            p = base + p;
            for (int i = 1; i <= CurveSegments; ++i) {
                const float t = static_cast<float>(i) / CurveSegments, u = 1.0f - t;
                current.push_back(pen * (u * u) + c * (2.0f * u * t) + p * (t * t));
            }
            pen = p;
            break;
        }
        case 'C': {
            Vec2 c1, c2, p;
            if (!reader.point(c1) || !reader.point(c2) || !reader.point(p)) return fail("C incompleto");
            c1 = base + c1;
            c2 = base + c2;
            p = base + p;
            for (int i = 1; i <= CurveSegments; ++i) {
                const float t = static_cast<float>(i) / CurveSegments, u = 1.0f - t;
                current.push_back(pen * (u * u * u) + c1 * (3.0f * u * u * t) + c2 * (3.0f * u * t * t) + p * (t * t * t));
            }
            pen = p;
            break;
        }
        case 'Z':
            close();
            pen = start;
            command = 0;
            break;
        default: return fail(std::string("comando desconhecido ") + command);
        }
    }
    close();
    if (shape.contours.empty()) return fail("vazio");
    shape.updateBounds();
    return shape;
}

} // namespace MatterEngine::Appearance
