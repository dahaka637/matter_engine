#include "Engine/Modeling/Paint/SvgPath.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace MatterEngine::Modeling {

namespace {

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

// Os pedacos do caminho em coordenadas absolutas (o ponto de partida de cada
// um e o fim do anterior).
struct Element {
    enum class Kind { Move, Line, Cubic, Arc, Close };
    Kind kind = Kind::Line;
    Point2 from;
    Point2 control1;
    Point2 control2;
    Point2 to;
    float rx = 0.0f;
    float ry = 0.0f;
    float rotation = 0.0f;
    bool large = false;
    bool sweep = false;
};

class Tokens {
public:
    explicit Tokens(std::string_view text) : m_text(text) {}

    bool atEnd() {
        skip();
        return m_at >= m_text.size();
    }

    // A letra de um comando (nao consome um numero).
    std::optional<char> command() {
        skip();
        if (m_at >= m_text.size()) return std::nullopt;
        const char c = m_text[m_at];
        if (!std::isalpha(static_cast<unsigned char>(c)) || c == 'e' || c == 'E') return std::nullopt;
        ++m_at;
        return c;
    }

    float number() {
        skip();
        const std::size_t start = m_at;
        if (m_at < m_text.size() && (m_text[m_at] == '+' || m_text[m_at] == '-')) ++m_at;
        bool digits = false;
        while (m_at < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_at]))) {
            ++m_at;
            digits = true;
        }
        if (m_at < m_text.size() && m_text[m_at] == '.') {
            ++m_at;
            while (m_at < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_at]))) {
                ++m_at;
                digits = true;
            }
        }
        if (!digits) fail("numero esperado na posicao " + std::to_string(start));
        if (m_at < m_text.size() && (m_text[m_at] == 'e' || m_text[m_at] == 'E')) {
            const std::size_t mark = m_at++;
            if (m_at < m_text.size() && (m_text[m_at] == '+' || m_text[m_at] == '-')) ++m_at;
            bool exponent = false;
            while (m_at < m_text.size() && std::isdigit(static_cast<unsigned char>(m_text[m_at]))) {
                ++m_at;
                exponent = true;
            }
            if (!exponent) m_at = mark;
        }
        // (O "+" na frente: from_chars nao aceita.)
        const std::size_t begin = m_text[start] == '+' ? start + 1 : start;
        float value = 0.0f;
        const auto [end, code] = std::from_chars(m_text.data() + begin, m_text.data() + m_at, value);
        if (code != std::errc() || end != m_text.data() + m_at) fail("numero invalido na posicao " + std::to_string(start));
        return value;
    }

    // As bandeiras do arco: um "0" ou "1" (podem vir coladas: "a5 5 0 01 9 9").
    bool flag() {
        skip();
        if (m_at >= m_text.size() || (m_text[m_at] != '0' && m_text[m_at] != '1')) fail("bandeira do arco (0 ou 1) na posicao " + std::to_string(m_at));
        return m_text[m_at++] == '1';
    }

    Point2 point() {
        const float x = number();
        return { x, number() };
    }

private:
    std::string_view m_text;
    std::size_t m_at = 0;

    void skip() {
        while (m_at < m_text.size() && (std::isspace(static_cast<unsigned char>(m_text[m_at])) || m_text[m_at] == ',')) ++m_at;
    }
};

Point2 plus(Point2 a, Point2 b) { return { a.x + b.x, a.y + b.y }; }
Point2 middle(Point2 a, Point2 b) { return { 0.5f * (a.x + b.x), 0.5f * (a.y + b.y) }; }
Point2 reflected(Point2 control, Point2 around) { return { 2.0f * around.x - control.x, 2.0f * around.y - control.y }; }

std::vector<Element> elementsOf(std::string_view d) {
    Tokens tokens(d);
    std::vector<Element> elements;
    Point2 current {}, start {}, lastCubic {}, lastQuad {};
    char command = 0, previous = 0;
    while (!tokens.atEnd()) {
        if (const std::optional<char> letter = tokens.command()) {
            command = *letter;
        } else if (command == 0) {
            fail("o caminho comeca com M");
        } else if (command == 'Z' || command == 'z') {
            fail("numero depois de Z");
        } else if (command == 'M') {
            command = 'L';
        } else if (command == 'm') {
            command = 'l';
        }
        const bool relative = std::islower(static_cast<unsigned char>(command)) != 0;
        const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));
        if (elements.empty() && upper != 'M') fail("o caminho comeca com M");
        const Point2 base = relative ? current : Point2 {};
        Element element;
        element.from = current;
        switch (upper) {
        case 'M':
            element.kind = Element::Kind::Move;
            element.to = plus(tokens.point(), base);
            start = element.to;
            break;
        case 'L':
            element.to = plus(tokens.point(), base);
            break;
        case 'H':
            element.to = { tokens.number() + base.x, current.y };
            break;
        case 'V':
            element.to = { current.x, tokens.number() + base.y };
            break;
        case 'C':
            element.kind = Element::Kind::Cubic;
            element.control1 = plus(tokens.point(), base);
            element.control2 = plus(tokens.point(), base);
            element.to = plus(tokens.point(), base);
            break;
        case 'S':
            element.kind = Element::Kind::Cubic;
            element.control1 = previous == 'C' || previous == 'S' ? reflected(lastCubic, current) : current;
            element.control2 = plus(tokens.point(), base);
            element.to = plus(tokens.point(), base);
            break;
        case 'Q':
        case 'T': {
            // A quadratica como cubica (os controles a dois tercos).
            const Point2 control = upper == 'Q' ? plus(tokens.point(), base)
                                                : (previous == 'Q' || previous == 'T' ? reflected(lastQuad, current) : current);
            element.kind = Element::Kind::Cubic;
            element.to = plus(tokens.point(), base);
            element.control1 = { current.x + 2.0f / 3.0f * (control.x - current.x), current.y + 2.0f / 3.0f * (control.y - current.y) };
            element.control2 = { element.to.x + 2.0f / 3.0f * (control.x - element.to.x), element.to.y + 2.0f / 3.0f * (control.y - element.to.y) };
            lastQuad = control;
            break;
        }
        case 'A':
            element.kind = Element::Kind::Arc;
            element.rx = tokens.number();
            element.ry = tokens.number();
            element.rotation = tokens.number();
            element.large = tokens.flag();
            element.sweep = tokens.flag();
            element.to = plus(tokens.point(), base);
            break;
        case 'Z':
            element.kind = Element::Kind::Close;
            element.to = start;
            break;
        default:
            fail(std::string("comando desconhecido: ") + command);
        }
        if (element.kind == Element::Kind::Cubic) lastCubic = element.control2;
        current = element.to;
        previous = upper;
        elements.push_back(element);
    }
    if (elements.empty()) fail("caminho vazio");
    return elements;
}

void flattenCubic(Point2 a, Point2 b, Point2 c, Point2 d, float tolerance, int depth, std::vector<Point2>& out) {
    const float dx = d.x - a.x, dy = d.y - a.y, length = std::hypot(dx, dy);
    float flat = 0.0f;
    if (length < 1e-12f) flat = std::max(std::hypot(b.x - a.x, b.y - a.y), std::hypot(c.x - a.x, c.y - a.y));
    else flat = std::max(std::abs(dx * (b.y - a.y) - dy * (b.x - a.x)), std::abs(dx * (c.y - a.y) - dy * (c.x - a.x))) / length;
    if (flat <= tolerance || depth >= 16) {
        out.push_back(d);
        return;
    }
    const Point2 ab = middle(a, b), bc = middle(b, c), cd = middle(c, d), abc = middle(ab, bc), bcd = middle(bc, cd), m = middle(abc, bcd);
    flattenCubic(a, ab, abc, m, tolerance, depth + 1, out);
    flattenCubic(m, bcd, cd, d, tolerance, depth + 1, out);
}

// O arco do SVG (pelas pontas) pelo centro (a conversao da especificacao,
// F.6.5), em segmentos com flecha abaixo da tolerancia.
void flattenArc(const Element& e, float tolerance, std::vector<Point2>& out) {
    float rx = std::abs(e.rx), ry = std::abs(e.ry);
    if (rx < 1e-12f || ry < 1e-12f || (std::abs(e.from.x - e.to.x) < 1e-12f && std::abs(e.from.y - e.to.y) < 1e-12f)) {
        out.push_back(e.to);
        return;
    }
    const float phi = e.rotation * 0.017453292f, cp = std::cos(phi), sp = std::sin(phi);
    const float hx = 0.5f * (e.from.x - e.to.x), hy = 0.5f * (e.from.y - e.to.y);
    const float x1 = cp * hx + sp * hy, y1 = -sp * hx + cp * hy;
    const float lambda = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry);
    if (lambda > 1.0f) {
        rx *= std::sqrt(lambda);
        ry *= std::sqrt(lambda);
    }
    const float numerator = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
    const float denominator = rx * rx * y1 * y1 + ry * ry * x1 * x1;
    const float coefficient = (e.large == e.sweep ? -1.0f : 1.0f) * std::sqrt(std::max(0.0f, numerator / std::max(1e-20f, denominator)));
    const float cx1 = coefficient * rx * y1 / ry, cy1 = -coefficient * ry * x1 / rx;
    const float cx = cp * cx1 - sp * cy1 + 0.5f * (e.from.x + e.to.x), cy = sp * cx1 + cp * cy1 + 0.5f * (e.from.y + e.to.y);
    const auto angle = [](float ux, float uy, float vx, float vy) { return std::atan2(ux * vy - uy * vx, ux * vx + uy * vy); };
    const float theta = angle(1.0f, 0.0f, (x1 - cx1) / rx, (y1 - cy1) / ry);
    float delta = angle((x1 - cx1) / rx, (y1 - cy1) / ry, (-x1 - cx1) / rx, (-y1 - cy1) / ry);
    if (!e.sweep && delta > 0.0f) delta -= 6.2831853f;
    if (e.sweep && delta < 0.0f) delta += 6.2831853f;
    const float radius = std::max(rx, ry);
    const float step = 2.0f * std::acos(std::clamp(1.0f - tolerance / radius, -1.0f, 1.0f));
    const int count = std::clamp(static_cast<int>(std::ceil(std::abs(delta) / std::max(step, 1e-3f))), 1, 512);
    for (int i = 1; i <= count; ++i) {
        if (i == count) {
            out.push_back(e.to);
            break;
        }
        const float t = theta + delta * static_cast<float>(i) / static_cast<float>(count);
        out.push_back({ cx + rx * cp * std::cos(t) - ry * sp * std::sin(t), cy + rx * sp * std::cos(t) + ry * cp * std::sin(t) });
    }
}

// O valor de um atributo dentro de uma etiqueta ("<path ... d='...'>").
std::optional<std::string> attribute(std::string_view tag, std::string_view name) {
    for (std::size_t at = tag.find(name); at != std::string_view::npos; at = tag.find(name, at + 1)) {
        if (at == 0 || !std::isspace(static_cast<unsigned char>(tag[at - 1]))) continue;
        std::size_t i = at + name.size();
        while (i < tag.size() && std::isspace(static_cast<unsigned char>(tag[i]))) ++i;
        if (i >= tag.size() || tag[i] != '=') continue;
        ++i;
        while (i < tag.size() && std::isspace(static_cast<unsigned char>(tag[i]))) ++i;
        if (i >= tag.size() || (tag[i] != '"' && tag[i] != '\'')) continue;
        const char quote = tag[i];
        const std::size_t end = tag.find(quote, i + 1);
        if (end == std::string_view::npos) return std::nullopt;
        return std::string(tag.substr(i + 1, end - i - 1));
    }
    return std::nullopt;
}

// As etiquetas que comecam com "<name" (seguido de espaco ou fim).
std::vector<std::string_view> tags(std::string_view text, std::string_view name) {
    std::vector<std::string_view> found;
    const std::string open = "<" + std::string(name);
    for (std::size_t at = text.find(open); at != std::string_view::npos; at = text.find(open, at + 1)) {
        const std::size_t after = at + open.size();
        if (after < text.size() && !std::isspace(static_cast<unsigned char>(text[after])) && text[after] != '>' && text[after] != '/') continue;
        const std::size_t end = text.find('>', after);
        if (end == std::string_view::npos) break;
        found.push_back(text.substr(at, end - at + 1));
    }
    return found;
}

} // namespace

std::optional<Shape2D> parseSvgPath(std::string_view d, std::string* error) {
    try {
        const std::vector<Element> elements = elementsOf(d);
        // A tolerancia: um milesimo do tamanho do desenho (os controles juntos).
        Box2 box;
        for (const Element& e : elements) {
            box.include(e.from);
            box.include(e.to);
            if (e.kind == Element::Kind::Cubic) {
                box.include(e.control1);
                box.include(e.control2);
            }
        }
        const float tolerance = std::max(1e-9f, 1e-3f * std::max(box.maximum.x - box.minimum.x, box.maximum.y - box.minimum.y));
        Shape2D shape;
        std::vector<Point2> contour;
        const auto finish = [&] {
            while (contour.size() > 1 && std::hypot(contour.back().x - contour.front().x, contour.back().y - contour.front().y) < 1e-3f * tolerance)
                contour.pop_back();
            if (contour.size() >= 3) shape.contours.push_back(contour);
            contour.clear();
        };
        for (const Element& e : elements) {
            if (e.kind == Element::Kind::Move) {
                finish();
                contour.push_back(e.to);
                continue;
            }
            if (e.kind == Element::Kind::Close) {
                finish();
                continue;
            }
            // (Depois de um Z sem M: o subcaminho novo sai do comeco.)
            if (contour.empty()) contour.push_back(e.from);
            if (e.kind == Element::Kind::Line) contour.push_back(e.to);
            else if (e.kind == Element::Kind::Cubic) flattenCubic(e.from, e.control1, e.control2, e.to, tolerance, 0, contour);
            else flattenArc(e, tolerance, contour);
        }
        finish();
        if (shape.empty()) fail("caminho sem area");
        return shape;
    } catch (const std::exception& exception) {
        if (error != nullptr) *error = std::string("svg: ") + exception.what();
        return std::nullopt;
    }
}

std::optional<SvgDocument> parseSvgDocument(std::string_view text, std::string* error) {
    SvgDocument document;
    const std::vector<std::string_view> roots = tags(text, "svg");
    if (roots.empty()) {
        if (error != nullptr) *error = "svg: sem <svg>";
        return std::nullopt;
    }
    if (const std::optional<std::string> box = attribute(roots.front(), "viewBox")) {
        std::array<float, 4> values {};
        try {
            Tokens tokens(*box);
            for (float& value : values) value = tokens.number();
        } catch (const std::exception&) {
            if (error != nullptr) *error = "svg: viewBox invalido: " + *box;
            return std::nullopt;
        }
        document.viewBox = values;
    }
    for (const std::string_view tag : tags(text, "path"))
        if (const std::optional<std::string> d = attribute(tag, "d")) document.paths.push_back({ attribute(tag, "id").value_or(""), *d });
    if (document.paths.empty()) {
        if (error != nullptr) *error = "svg: sem <path d=...>";
        return std::nullopt;
    }
    return document;
}

std::optional<SvgDocument> readSvgDocument(const std::filesystem::path& file, std::string* error) {
    std::ifstream stream(file);
    if (!stream) {
        if (error != nullptr) *error = "svg: nao abriu " + file.string();
        return std::nullopt;
    }
    std::stringstream text;
    text << stream.rdbuf();
    return parseSvgDocument(text.str(), error);
}

} // namespace MatterEngine::Modeling
