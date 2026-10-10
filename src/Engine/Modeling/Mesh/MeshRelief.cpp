#include "Engine/Modeling/Mesh/MeshRelief.hpp"

#include "Engine/Modeling/Mesh/MeshStitch.hpp"

#include <algorithm>

namespace MatterEngine::Modeling {

void stripTop(ReliefPiece& piece, std::span<const Vec3> right, std::span<const Vec3> left) {
    const std::size_t count = std::min(right.size(), left.size());
    piece.topPositions.clear();
    piece.topTriangles.clear();
    piece.rim.clear();
    if (count < 2) return;
    for (std::size_t i = 0; i < count; ++i) piece.topPositions.push_back(right[i]);
    for (std::size_t i = 0; i < count; ++i) piece.topPositions.push_back(left[i]);
    const auto r = [](std::size_t i) { return static_cast<std::uint32_t>(i); };
    const auto l = [count](std::size_t i) { return static_cast<std::uint32_t>(count + i); };
    for (std::size_t i = 0; i + 1 < count; ++i) {
        // A diagonal mais curta (como MeshBuilder::quad).
        const Vec3 a = right[i], b = right[i + 1], c = left[i + 1], d = left[i];
        if ((a - c).lengthSquared() <= (b - d).lengthSquared()) {
            piece.topTriangles.push_back({ r(i), r(i + 1), l(i + 1) });
            piece.topTriangles.push_back({ r(i), l(i + 1), l(i) });
        } else {
            piece.topTriangles.push_back({ r(i), r(i + 1), l(i) });
            piece.topTriangles.push_back({ r(i + 1), l(i + 1), l(i) });
        }
    }
    for (std::size_t i = 0; i < count; ++i) piece.rim.push_back(r(i));
    for (std::size_t i = count; i-- > 0;) piece.rim.push_back(l(i));
}

bool addReliefs(MeshBuilder& builder, std::span<const std::uint32_t> host, std::span<const ReliefPiece> pieces,
    const ImprintSettings& settings, std::string* error) {
    if (pieces.empty()) return true;
    std::vector<ImprintLoop> loops;
    for (const ReliefPiece& piece : pieces) {
        ImprintLoop loop;
        loop.points = piece.footprint;
        loops.push_back(std::move(loop));
    }
    const ImprintResult cut = imprintLoops(builder.mesh(), host, loops, settings);
    if (!cut.ok) {
        if (error != nullptr) *error = cut.error;
        return false;
    }
    for (std::size_t p = 0; p < pieces.size(); ++p) {
        const ReliefPiece& piece = pieces[p];
        std::vector<std::uint32_t> top;
        top.reserve(piece.topPositions.size());
        for (const Vec3& position : piece.topPositions) top.push_back(builder.vertex(position));
        for (const Triangle& tri : piece.topTriangles) builder.triangle(top[tri[0]], top[tri[1]], top[tri[2]]);
        std::vector<std::uint32_t> rim;
        rim.reserve(piece.rim.size());
        for (const std::uint32_t index : piece.rim) rim.push_back(top[index]);
        if (piece.footprintToRim.empty() && piece.rim.size() != piece.footprint.size())
            stitchLoops(builder, cut.loops[p], rim);
        else
            stitchCorresponding(builder, cut.loops[p], cut.inputVertices[p], rim, piece.footprintToRim);
    }
    return true;
}

std::vector<std::vector<std::uint32_t>> loopSegments(std::span<const std::uint32_t> loop,
    std::span<const std::uint32_t> inputs) {
    std::vector<std::size_t> positions;
    for (const std::uint32_t input : inputs) {
        const auto found = std::find(loop.begin(), loop.end(), input);
        if (found == loop.end()) continue;
        const auto at = static_cast<std::size_t>(found - loop.begin());
        if (positions.empty() || positions.back() != at) positions.push_back(at);
    }
    std::vector<std::vector<std::uint32_t>> segments;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const std::size_t from = positions[i], to = positions[(i + 1) % positions.size()];
        std::vector<std::uint32_t> segment;
        for (std::size_t k = from;; k = (k + 1) % loop.size()) {
            segment.push_back(loop[k]);
            if (k == to && segment.size() > 1) break;
            if (segment.size() > loop.size() + 1) break;
        }
        segments.push_back(std::move(segment));
    }
    return segments;
}

} // namespace MatterEngine::Modeling
