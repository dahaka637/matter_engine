#include "Engine/Modeling/Geometry/Triangulation2D.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace MatterEngine::Modeling {

namespace {

struct P {
    double x = 0.0;
    double y = 0.0;
};

// Positivo quando c fica a esquerda de a->b.
double orient(P a, P b, P c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Positivo quando d fica dentro do circulo do triangulo anti-horario abc.
double inCircle(P a, P b, P c, P d) {
    const double adx = a.x - d.x, ady = a.y - d.y;
    const double bdx = b.x - d.x, bdy = b.y - d.y;
    const double cdx = c.x - d.x, cdy = c.y - d.y;
    const double ad = adx * adx + ady * ady, bd = bdx * bdx + bdy * bdy, cd = cdx * cdx + cdy * cdy;
    return adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx);
}

std::uint64_t directedKey(int from, int to) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(from)) << 32) | static_cast<std::uint32_t>(to);
}

std::uint64_t undirectedKey(int a, int b) {
    return directedKey(std::min(a, b), std::max(a, b));
}

// Os segmentos ab e cd se cruzam no meio (sem contar pontas encostadas).
bool properlyCross(P a, P b, P c, P d, double eps) {
    const double d1 = orient(a, b, c), d2 = orient(a, b, d), d3 = orient(c, d, a), d4 = orient(c, d, b);
    return ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps));
}

struct Tri {
    std::array<int, 3> v {};
    bool alive = true;
};

// A triangulacao com as arestas orientadas: cada aresta anti-horaria de um
// triangulo vivo aponta para ele; o vizinho de (a,b) e o dono de (b,a).
class Mesh2D {
public:
    std::vector<P> points;
    std::vector<Tri> tris;
    std::unordered_map<std::uint64_t, int> owner;
    int last = -1;
    double eps = 1e-14;

    int add(int a, int b, int c) {
        const int t = static_cast<int>(tris.size());
        tris.push_back({ { a, b, c }, true });
        owner[directedKey(a, b)] = t;
        owner[directedKey(b, c)] = t;
        owner[directedKey(c, a)] = t;
        last = t;
        return t;
    }

    void kill(int t) {
        Tri& tri = tris[static_cast<std::size_t>(t)];
        if (!tri.alive) return;
        tri.alive = false;
        for (int i = 0; i < 3; ++i) {
            const auto found = owner.find(directedKey(tri.v[static_cast<std::size_t>(i)], tri.v[static_cast<std::size_t>((i + 1) % 3)]));
            if (found != owner.end() && found->second == t) owner.erase(found);
        }
    }

    // O vizinho pela aresta oposta ao vertice i.
    int neighbor(int t, int i) const {
        const Tri& tri = tris[static_cast<std::size_t>(t)];
        const int a = tri.v[static_cast<std::size_t>((i + 1) % 3)], b = tri.v[static_cast<std::size_t>((i + 2) % 3)];
        const auto found = owner.find(directedKey(b, a));
        return found == owner.end() ? -1 : found->second;
    }

    int ownerOf(int a, int b) const {
        const auto found = owner.find(directedKey(a, b));
        return found == owner.end() ? -1 : found->second;
    }

    P at(int index) const { return points[static_cast<std::size_t>(index)]; }

    bool contains(int t, P p) const {
        const Tri& tri = tris[static_cast<std::size_t>(t)];
        for (int i = 0; i < 3; ++i)
            if (orient(at(tri.v[static_cast<std::size_t>(i)]), at(tri.v[static_cast<std::size_t>((i + 1) % 3)]), p) < -eps)
                return false;
        return true;
    }

    int locate(P p) const {
        int t = last;
        if (t < 0 || !tris[static_cast<std::size_t>(t)].alive) {
            t = -1;
            for (std::size_t i = tris.size(); i-- > 0;)
                if (tris[i].alive) {
                    t = static_cast<int>(i);
                    break;
                }
        }
        const std::size_t limit = tris.size() * 3 + 64;
        for (std::size_t step = 0; t >= 0 && step < limit; ++step) {
            const Tri& tri = tris[static_cast<std::size_t>(t)];
            int next = -1;
            for (int k = 0; k < 3 && next < 0; ++k) {
                // A aresta inicial gira: o passeio nao entra em ciclo.
                const int i = (k + static_cast<int>(step)) % 3;
                const P a = at(tri.v[static_cast<std::size_t>((i + 1) % 3)]), b = at(tri.v[static_cast<std::size_t>((i + 2) % 3)]);
                if (orient(a, b, p) < -eps) next = neighbor(t, i);
            }
            if (next < 0) return t;
            t = next;
        }
        for (std::size_t i = 0; i < tris.size(); ++i)
            if (tris[i].alive && contains(static_cast<int>(i), p)) return static_cast<int>(i);
        return -1;
    }

    bool insert(int index) {
        const P p = at(index);
        const int start = locate(p);
        if (start < 0) return false;
        std::unordered_set<int> cavity { start };
        std::vector<int> stack { start };
        while (!stack.empty()) {
            const int t = stack.back();
            stack.pop_back();
            for (int i = 0; i < 3; ++i) {
                const int nb = neighbor(t, i);
                if (nb < 0 || cavity.count(nb) != 0) continue;
                const Tri& other = tris[static_cast<std::size_t>(nb)];
                if (inCircle(at(other.v[0]), at(other.v[1]), at(other.v[2]), p) > eps) {
                    cavity.insert(nb);
                    stack.push_back(nb);
                }
            }
        }
        // A cavidade precisa ver o ponto de dentro: toda aresta da borda com o
        // ponto a esquerda (estritamente). Senao entra o vizinho de fora.
        struct Edge {
            int a, b;
        };
        std::vector<Edge> boundary;
        bool valid = false;
        for (int round = 0; round < 64 && !valid; ++round) {
            boundary.clear();
            int grow = -1;
            for (const int t : cavity) {
                const Tri& tri = tris[static_cast<std::size_t>(t)];
                for (int i = 0; i < 3 && grow < 0; ++i) {
                    const int nb = neighbor(t, i);
                    if (nb >= 0 && cavity.count(nb) != 0) continue;
                    const int a = tri.v[static_cast<std::size_t>((i + 1) % 3)], b = tri.v[static_cast<std::size_t>((i + 2) % 3)];
                    if (orient(at(a), at(b), p) <= eps) {
                        if (nb < 0) return false;
                        grow = nb;
                        break;
                    }
                    boundary.push_back({ a, b });
                }
                if (grow >= 0) break;
            }
            if (grow < 0) valid = true;
            else cavity.insert(grow);
        }
        if (!valid) return false;
        for (const int t : cavity) kill(t);
        for (const Edge& edge : boundary) add(edge.a, edge.b, index);
        return true;
    }

    // Troca a diagonal da aresta u->v (dono t1) com o vizinho t2.
    void flip(int u, int v) {
        const int t1 = ownerOf(u, v), t2 = ownerOf(v, u);
        const int w1 = third(t1, u, v), w2 = third(t2, v, u);
        kill(t1);
        kill(t2);
        add(w1, u, w2);
        add(w2, v, w1);
    }

    int third(int t, int a, int b) const {
        for (const int v : tris[static_cast<std::size_t>(t)].v)
            if (v != a && v != b) return v;
        return -1;
    }

    // O quadrilatero da aresta u-v e convexo (a outra diagonal cabe).
    bool flippable(int u, int v) const {
        const int t1 = ownerOf(u, v), t2 = ownerOf(v, u);
        if (t1 < 0 || t2 < 0) return false;
        const int w1 = third(t1, u, v), w2 = third(t2, v, u);
        const P pu = at(u), pv = at(v), p1 = at(w1), p2 = at(w2);
        return orient(p1, pu, p2) > eps && orient(p2, pv, p1) > eps;
    }

    bool locallyDelaunay(int u, int v) const {
        const int t1 = ownerOf(u, v), t2 = ownerOf(v, u);
        if (t1 < 0 || t2 < 0) return true;
        const Tri& tri = tris[static_cast<std::size_t>(t1)];
        const int w2 = third(t2, v, u);
        return inCircle(at(tri.v[0]), at(tri.v[1]), at(tri.v[2]), at(w2)) <= eps;
    }
};

} // namespace

double polygonArea2D(const std::vector<Point2>& polygon) {
    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Point2 a = polygon[i], b = polygon[(i + 1) % polygon.size()];
        area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    return area * 0.5;
}

bool pointInPolygon2D(Point2 point, const std::vector<Point2>& polygon) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const Point2 a = polygon[i], b = polygon[j];
        if ((a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x)
            inside = !inside;
    }
    return inside;
}

Triangulation2DResult triangulateDomain(const Triangulation2DInput& input) {
    Triangulation2DResult result;
    const std::size_t count = input.points.size();
    if (count < 3 || input.loops.empty() || input.loops.front().size() < 3) {
        result.error = "triangulacao sem pontos ou sem a volta de fora";
        return result;
    }
    for (const auto& loop : input.loops)
        for (const int index : loop)
            if (index < 0 || static_cast<std::size_t>(index) >= count) {
                result.error = "volta com indice fora dos pontos";
                return result;
            }
    for (const auto& segment : input.segments)
        for (const int index : segment)
            if (index < 0 || static_cast<std::size_t>(index) >= count) {
                result.error = "aresta obrigatoria com indice fora dos pontos";
                return result;
            }

    // Espaco normalizado: o conjunto cabe em [-0,5; 0,5].
    double minX = std::numeric_limits<double>::max(), minY = minX, maxX = -minX, maxY = -minX;
    for (const Point2& p : input.points) {
        minX = std::min(minX, static_cast<double>(p.x));
        maxX = std::max(maxX, static_cast<double>(p.x));
        minY = std::min(minY, static_cast<double>(p.y));
        maxY = std::max(maxY, static_cast<double>(p.y));
    }
    const double scale = std::max(maxX - minX, maxY - minY);
    if (!(scale > 0.0)) {
        result.error = "pontos todos no mesmo lugar";
        return result;
    }
    const double cx = (minX + maxX) * 0.5, cy = (minY + maxY) * 0.5;
    Mesh2D mesh;
    mesh.points.reserve(count + 3);
    for (const Point2& p : input.points) mesh.points.push_back({ (p.x - cx) / scale, (p.y - cy) / scale });

    // Repetidos viram o primeiro.
    constexpr double Same = 1e-9;
    std::vector<int> representative(count);
    {
        std::map<std::pair<long long, long long>, std::vector<int>> grid;
        const auto cell = [](double value) { return static_cast<long long>(std::floor(value / (Same * 4.0))); };
        for (std::size_t i = 0; i < count; ++i) {
            const P p = mesh.points[i];
            const long long gx = cell(p.x), gy = cell(p.y);
            int found = static_cast<int>(i);
            for (long long dx = -1; dx <= 1 && found == static_cast<int>(i); ++dx)
                for (long long dy = -1; dy <= 1 && found == static_cast<int>(i); ++dy) {
                    const auto it = grid.find({ gx + dx, gy + dy });
                    if (it == grid.end()) continue;
                    for (const int other : it->second) {
                        const P q = mesh.points[static_cast<std::size_t>(other)];
                        if (std::hypot(p.x - q.x, p.y - q.y) <= Same) {
                            found = other;
                            break;
                        }
                    }
                }
            representative[i] = found;
            if (found == static_cast<int>(i)) grid[{ gx, gy }].push_back(static_cast<int>(i));
        }
    }

    // As arestas obrigatorias (as voltas e os segmentos), divididas nos
    // pontos que caem exatamente sobre elas.
    std::vector<std::array<int, 2>> constraints;
    const auto addConstraint = [&](int a, int b) {
        a = representative[static_cast<std::size_t>(a)];
        b = representative[static_cast<std::size_t>(b)];
        if (a != b) constraints.push_back({ a, b });
    };
    for (const auto& loop : input.loops)
        for (std::size_t i = 0; i < loop.size(); ++i) addConstraint(loop[i], loop[(i + 1) % loop.size()]);
    for (const auto& segment : input.segments) addConstraint(segment[0], segment[1]);
    {
        std::vector<std::array<int, 2>> split;
        for (const auto& [a, b] : constraints) {
            const P pa = mesh.at(a), pb = mesh.at(b);
            const double length = std::hypot(pb.x - pa.x, pb.y - pa.y);
            std::vector<std::pair<double, int>> on;
            for (std::size_t i = 0; i < count; ++i) {
                const int c = static_cast<int>(i);
                if (representative[i] != c || c == a || c == b) continue;
                const P pc = mesh.at(c);
                if (std::abs(orient(pa, pb, pc)) > Same * 10.0 * length) continue;
                const double t = ((pc.x - pa.x) * (pb.x - pa.x) + (pc.y - pa.y) * (pb.y - pa.y)) / (length * length);
                if (t > 1e-9 && t < 1.0 - 1e-9) on.push_back({ t, c });
            }
            std::sort(on.begin(), on.end());
            int from = a;
            for (const auto& [t, c] : on) {
                split.push_back({ from, c });
                from = c;
            }
            split.push_back({ from, b });
        }
        constraints = std::move(split);
        std::unordered_set<std::uint64_t> seen;
        std::vector<std::array<int, 2>> unique;
        for (const auto& edge : constraints)
            if (seen.insert(undirectedKey(edge[0], edge[1])).second) unique.push_back(edge);
        constraints = std::move(unique);
    }
    for (std::size_t i = 0; i < constraints.size(); ++i)
        for (std::size_t j = i + 1; j < constraints.size(); ++j)
            if (properlyCross(mesh.at(constraints[i][0]), mesh.at(constraints[i][1]), mesh.at(constraints[j][0]),
                    mesh.at(constraints[j][1]), 1e-15)) {
                result.error = "arestas obrigatorias se cruzam";
                result.crossing = { constraints[i][0], constraints[i][1], constraints[j][0], constraints[j][1] };
                return result;
            }

    // Delaunay incremental dentro de um triangulo grande.
    const int super = static_cast<int>(count);
    mesh.points.push_back({ -60.0, -60.0 });
    mesh.points.push_back({ 60.0, -60.0 });
    mesh.points.push_back({ 0.0, 60.0 });
    mesh.add(super, super + 1, super + 2);
    std::vector<int> order;
    for (std::size_t i = 0; i < count; ++i)
        if (representative[i] == static_cast<int>(i)) order.push_back(static_cast<int>(i));
    // Ordem embaralhada (semente fixa): o passeio do localizar fica curto.
    std::uint32_t state = 0x9E3779B9u;
    for (std::size_t i = order.size(); i > 1; --i) {
        state = state * 1664525u + 1013904223u;
        std::swap(order[i - 1], order[state % i]);
    }
    for (const int index : order)
        if (!mesh.insert(index)) {
            result.error = "falhou ao inserir um ponto na triangulacao";
            return result;
        }

    // Recupera as arestas obrigatorias trocando diagonais (Sloan).
    std::unordered_set<std::uint64_t> fixed;
    for (const auto& [a, b] : constraints) fixed.insert(undirectedKey(a, b));
    for (const auto& [a, b] : constraints) {
        if (mesh.ownerOf(a, b) >= 0 || mesh.ownerOf(b, a) >= 0) continue;
        const P pa = mesh.at(a), pb = mesh.at(b);
        std::deque<std::pair<int, int>> crossing;
        std::unordered_set<std::uint64_t> listed;
        for (std::size_t t = 0; t < mesh.tris.size(); ++t) {
            if (!mesh.tris[t].alive) continue;
            for (int i = 0; i < 3; ++i) {
                const int u = mesh.tris[t].v[static_cast<std::size_t>(i)], v = mesh.tris[t].v[static_cast<std::size_t>((i + 1) % 3)];
                if (u == a || u == b || v == a || v == b) continue;
                if (!properlyCross(pa, pb, mesh.at(u), mesh.at(v), 1e-15)) continue;
                if (listed.insert(undirectedKey(u, v)).second) crossing.push_back({ u, v });
            }
        }
        std::vector<std::pair<int, int>> created;
        std::size_t guard = 0;
        const std::size_t limit = crossing.size() * crossing.size() * 4 + 256;
        while (!crossing.empty()) {
            if (++guard > limit) {
                result.error = "nao conseguiu recuperar uma aresta obrigatoria";
                return result;
            }
            const auto [u, v] = crossing.front();
            crossing.pop_front();
            if (fixed.count(undirectedKey(u, v)) != 0) {
                result.error = "aresta obrigatoria atravessada por outra";
                return result;
            }
            if (!mesh.flippable(u, v)) {
                crossing.push_back({ u, v });
                continue;
            }
            const int t1 = mesh.ownerOf(u, v), t2 = mesh.ownerOf(v, u);
            const int w1 = mesh.third(t1, u, v), w2 = mesh.third(t2, v, u);
            mesh.flip(u, v);
            if (w1 != a && w1 != b && w2 != a && w2 != b && properlyCross(pa, pb, mesh.at(w1), mesh.at(w2), 1e-15))
                crossing.push_back({ w1, w2 });
            else
                created.push_back({ w1, w2 });
        }
        if (mesh.ownerOf(a, b) < 0 && mesh.ownerOf(b, a) < 0) {
            result.error = "aresta obrigatoria nao apareceu";
            return result;
        }
    }

    // Delaunay de novo (Lawson) sem tocar nas obrigatorias.
    {
        std::deque<std::pair<int, int>> queue;
        for (std::size_t t = 0; t < mesh.tris.size(); ++t) {
            if (!mesh.tris[t].alive) continue;
            for (int i = 0; i < 3; ++i) {
                const int u = mesh.tris[t].v[static_cast<std::size_t>(i)], v = mesh.tris[t].v[static_cast<std::size_t>((i + 1) % 3)];
                if (u < v) queue.push_back({ u, v });
            }
        }
        std::size_t guard = 0;
        const std::size_t limit = queue.size() * 64 + 1024;
        while (!queue.empty() && ++guard < limit) {
            const auto [u, v] = queue.front();
            queue.pop_front();
            if (fixed.count(undirectedKey(u, v)) != 0) continue;
            if (mesh.ownerOf(u, v) < 0 || mesh.ownerOf(v, u) < 0) continue;
            if (mesh.locallyDelaunay(u, v) || !mesh.flippable(u, v)) continue;
            const int w1 = mesh.third(mesh.ownerOf(u, v), u, v), w2 = mesh.third(mesh.ownerOf(v, u), v, u);
            mesh.flip(u, v);
            queue.push_back({ u, w1 });
            queue.push_back({ w1, v });
            queue.push_back({ v, w2 });
            queue.push_back({ w2, u });
        }
    }

    // O dominio: dentro da volta de fora e fora dos furos.
    std::vector<std::vector<Point2>> polygons;
    for (const auto& loop : input.loops) {
        std::vector<Point2> polygon;
        for (const int index : loop) polygon.push_back(input.points[static_cast<std::size_t>(index)]);
        polygons.push_back(std::move(polygon));
    }
    for (const Tri& tri : mesh.tris) {
        if (!tri.alive) continue;
        if (tri.v[0] >= super || tri.v[1] >= super || tri.v[2] >= super) continue;
        const Point2 a = input.points[static_cast<std::size_t>(tri.v[0])], b = input.points[static_cast<std::size_t>(tri.v[1])],
                     c = input.points[static_cast<std::size_t>(tri.v[2])];
        const Point2 centroid { (a.x + b.x + c.x) / 3.0f, (a.y + b.y + c.y) / 3.0f };
        if (!pointInPolygon2D(centroid, polygons.front())) continue;
        bool inHole = false;
        for (std::size_t h = 1; h < polygons.size() && !inHole; ++h) inHole = pointInPolygon2D(centroid, polygons[h]);
        if (inHole) continue;
        result.triangles.push_back(tri.v);
    }
    result.ok = !result.triangles.empty();
    if (!result.ok) result.error = "o dominio ficou vazio";
    return result;
}

} // namespace MatterEngine::Modeling
