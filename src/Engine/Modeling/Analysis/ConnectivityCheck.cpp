#include "Engine/Modeling/Analysis/ConnectivityCheck.hpp"

#include "Engine/Modeling/Mesh/MeshTopology.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>

namespace MatterEngine::Modeling {

namespace {

std::string partName(const Mesh& mesh, int part) {
    return part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size() ? mesh.parts[static_cast<std::size_t>(part)].name
                                                                          : std::string("?");
}

std::string partRegion(const Mesh& mesh, int part) {
    return part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size()
        ? mesh.parts[static_cast<std::size_t>(part)].region : std::string();
}

} // namespace

void checkConnectivity(const Mesh& mesh, Report& report, const ConnectivitySettings& settings) {
    if (mesh.triangles.empty()) return;

    // Pedacos ligados e as raizes.
    int componentCount = 0;
    const std::vector<int> component = edgeComponents(mesh, &componentCount);
    std::vector<std::set<int>> componentParts(static_cast<std::size_t>(componentCount));
    std::vector<std::vector<std::uint32_t>> componentTriangles(static_cast<std::size_t>(componentCount));
    for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t) {
        componentParts[static_cast<std::size_t>(component[t])].insert(mesh.partOf(t));
        componentTriangles[static_cast<std::size_t>(component[t])].push_back(t);
    }
    const bool anyRoot = std::any_of(mesh.parts.begin(), mesh.parts.end(), [](const MeshPart& p) { return p.root; });
    std::size_t largest = 0;
    for (std::size_t c = 0; c < componentTriangles.size(); ++c)
        if (componentTriangles[c].size() > componentTriangles[largest].size()) largest = c;
    if (!anyRoot) {
        Finding finding;
        finding.severity = Severity::Info;
        finding.code = "sem_raiz";
        finding.message = "nenhuma parte raiz declarada: o maior pedaco conta como o corpo";
        report.add(std::move(finding));
    }
    for (std::size_t c = 0; c < componentTriangles.size(); ++c) {
        bool rooted = false;
        for (const int part : componentParts[c])
            if (part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size() && mesh.parts[static_cast<std::size_t>(part)].root)
                rooted = true;
        if (!anyRoot) rooted = c == largest;
        if (rooted) continue;
        Finding finding;
        finding.severity = Severity::Error;
        finding.code = "peca_solta";
        std::string names;
        for (const int part : componentParts[c]) names += (names.empty() ? "" : ", ") + partName(mesh, part);
        finding.message = "pedaco sem ligacao com o corpo (" + std::to_string(componentTriangles[c].size())
            + " triangulos: " + names + ")";
        const int first = *componentParts[c].begin();
        finding.part = partName(mesh, first);
        finding.region = partRegion(mesh, first);
        finding.triangles = componentTriangles[c];
        finding.location = mesh.center(componentTriangles[c].front());
        finding.hasLocation = true;
        report.add(std::move(finding));
    }
    report.statistic("superficies (pedacos ligados)", std::to_string(componentCount));

    // As costuras: arestas com um triangulo de cada parte.
    const MeshEdges edges(mesh);
    std::map<std::pair<int, int>, std::size_t> seams;
    for (const auto& [key, use] : edges.uses()) {
        if (use.count != 2) continue;
        const int a = mesh.partOf(use.triangles[0]), b = mesh.partOf(use.triangles[1]);
        if (a != b) ++seams[{ std::min(a, b), std::max(a, b) }];
    }
    for (std::size_t p = 0; p < mesh.parts.size(); ++p)
        for (const std::string& otherName : mesh.parts[p].connectedTo) {
            const int other = mesh.findPart(otherName);
            Finding finding;
            finding.part = mesh.parts[p].name;
            finding.otherPart = otherName;
            finding.region = mesh.parts[p].region;
            if (other < 0) {
                finding.severity = Severity::Warning;
                finding.code = "costura_sem_parte";
                finding.message = mesh.parts[p].name + " declara costura com " + otherName + ", que nao existe";
                report.add(std::move(finding));
                continue;
            }
            const int self = static_cast<int>(p);
            const auto found = seams.find({ std::min(self, other), std::max(self, other) });
            if (found != seams.end() && found->second > 0) continue;
            finding.severity = Severity::Error;
            finding.code = "costura_faltando";
            finding.message = mesh.parts[p].name + " deveria estar costurada em " + otherName + " (nenhuma aresta em comum)";
            const std::vector<std::uint32_t> triangles = mesh.partTriangles(self);
            if (!triangles.empty()) {
                finding.location = mesh.center(triangles.front());
                finding.hasLocation = true;
            }
            report.add(std::move(finding));
        }
    std::size_t seamEdges = 0;
    for (const auto& [pair, count] : seams) seamEdges += count;
    report.statistic("arestas de costura entre partes", std::to_string(seamEdges));

    // Bordas abertas: so as declaradas (as maiores voltas de cada parte
    // levam os nomes das aberturas dela).
    const std::vector<std::vector<std::uint32_t>> loops = boundaryLoops(mesh);
    std::map<int, std::vector<std::size_t>> loopsByPart;
    std::vector<std::vector<std::uint32_t>> loopTriangles(loops.size());
    for (std::size_t l = 0; l < loops.size(); ++l) {
        std::map<int, int> votes;
        const std::vector<std::uint32_t>& loop = loops[l];
        for (std::size_t i = 0; i < loop.size(); ++i) {
            const MeshEdges::Use* use = edges.find(loop[i], loop[(i + 1) % loop.size()]);
            if (use == nullptr) continue;
            ++votes[mesh.partOf(use->triangles[0])];
            loopTriangles[l].push_back(use->triangles[0]);
        }
        int part = -1, best = -1;
        for (const auto& [candidate, count] : votes)
            if (count > best) {
                best = count;
                part = candidate;
            }
        loopsByPart[part].push_back(l);
    }
    std::size_t boundaryEdges = 0;
    for (auto& [part, list] : loopsByPart) {
        std::sort(list.begin(), list.end(), [&](std::size_t a, std::size_t b) { return loops[a].size() > loops[b].size(); });
        const std::vector<std::string> declared = part >= 0 && static_cast<std::size_t>(part) < mesh.parts.size()
            ? mesh.parts[static_cast<std::size_t>(part)].openings : std::vector<std::string> {};
        for (std::size_t i = 0; i < list.size(); ++i) {
            const std::vector<std::uint32_t>& loop = loops[list[i]];
            boundaryEdges += loop.size();
            Finding finding;
            finding.part = partName(mesh, part);
            finding.region = partRegion(mesh, part);
            for (std::size_t k = 0; k < loop.size(); ++k)
                finding.segments.push_back({ mesh.positions[loop[k]], mesh.positions[loop[(k + 1) % loop.size()]] });
            finding.triangles = loopTriangles[list[i]];
            Vec3 sum {};
            for (const std::uint32_t v : loop) sum += mesh.positions[v];
            finding.location = sum / static_cast<float>(loop.size());
            finding.hasLocation = true;
            if (i < declared.size()) {
                finding.severity = Severity::Info;
                finding.code = "abertura_declarada";
                finding.message = "abertura \"" + declared[i] + "\" (" + std::to_string(loop.size()) + " arestas)";
            } else {
                finding.severity = Severity::Error;
                finding.code = "borda_aberta";
                finding.message = "borda aberta nao declarada com " + std::to_string(loop.size())
                    + " aresta(s): furo ou costura que nao fechou";
            }
            report.add(std::move(finding));
        }
    }
    report.statistic("bordas abertas (arestas)", std::to_string(boundaryEdges));

    // Vertices no mesmo lugar e nao soldados (entre quaisquer partes).
    {
        std::vector<int> vertexPart(mesh.positions.size(), -2);
        for (std::uint32_t t = 0; t < mesh.triangles.size(); ++t)
            for (const std::uint32_t v : mesh.triangles[t])
                if (vertexPart[v] == -2) vertexPart[v] = mesh.partOf(t);
        const float cell = std::max(settings.duplicateTolerance * 4.0f, 1e-6f);
        std::map<std::array<long long, 3>, std::vector<std::uint32_t>> grid;
        std::map<std::pair<int, int>, Finding> duplicates;
        for (std::uint32_t v = 0; v < mesh.positions.size(); ++v) {
            if (vertexPart[v] == -2) continue;
            const Vec3 p = mesh.positions[v];
            const std::array<long long, 3> base { static_cast<long long>(std::floor(p.x / cell)),
                static_cast<long long>(std::floor(p.y / cell)), static_cast<long long>(std::floor(p.z / cell)) };
            int twin = -1;
            for (long long dx = -1; dx <= 1 && twin < 0; ++dx)
                for (long long dy = -1; dy <= 1 && twin < 0; ++dy)
                    for (long long dz = -1; dz <= 1 && twin < 0; ++dz) {
                        const auto found = grid.find({ base[0] + dx, base[1] + dy, base[2] + dz });
                        if (found == grid.end()) continue;
                        for (const std::uint32_t other : found->second)
                            if ((mesh.positions[other] - p).length() <= settings.duplicateTolerance) {
                                twin = static_cast<int>(other);
                                break;
                            }
                    }
            if (twin < 0) {
                grid[base].push_back(v);
                continue;
            }
            const int a = vertexPart[v], b = vertexPart[static_cast<std::size_t>(twin)];
            Finding& finding = duplicates[{ std::min(a, b), std::max(a, b) }];
            finding.points.push_back(p);
        }
        for (auto& [pair, finding] : duplicates) {
            finding.severity = Severity::Error;
            finding.code = "costura_sem_solda";
            finding.message = "vertices no mesmo lugar sem solda (" + std::to_string(finding.points.size()) + ")";
            finding.part = partName(mesh, pair.first);
            if (pair.second != pair.first) finding.otherPart = partName(mesh, pair.second);
            finding.region = partRegion(mesh, pair.first);
            finding.location = finding.points.front();
            finding.hasLocation = true;
            report.add(std::move(finding));
        }
    }

    const std::vector<std::uint32_t> pinched = pinchedVertices(mesh);
    if (!pinched.empty()) {
        Finding finding;
        finding.severity = Severity::Warning;
        finding.code = "vertice_pincado";
        finding.message = "a superficie se toca so por um vertice (" + std::to_string(pinched.size()) + ")";
        for (const std::uint32_t v : pinched) finding.points.push_back(mesh.positions[v]);
        finding.location = finding.points.front();
        finding.hasLocation = true;
        report.add(std::move(finding));
    }
}

} // namespace MatterEngine::Modeling
