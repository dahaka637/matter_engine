#include "Engine/Athlete/Body/SupportWrench3D.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace MatterEngine {
namespace {

Vec3 horizontal(Vec3 value) { return { value.x, value.y, 0.0f }; }

float cross2(float ax, float ay, float bx, float by) { return ax * by - ay * bx; }

// O fecho convexo (no plano) dos cantos das solas apoiadas.
std::vector<std::array<float, 2>> convexHull(std::vector<std::array<float, 2>> points) {
    std::sort(points.begin(), points.end());
    if (points.size() < 3) return points;
    std::vector<std::array<float, 2>> hull;
    for (int pass = 0; pass < 2; ++pass) {
        const std::size_t start = hull.size();
        for (const auto& p : points) {
            while (hull.size() >= start + 2) {
                const auto& a = hull[hull.size() - 2];
                const auto& b = hull[hull.size() - 1];
                if (cross2(b[0] - a[0], b[1] - a[1], p[0] - a[0], p[1] - a[1]) > 0.0f) break;
                hull.pop_back();
            }
            hull.push_back(p);
        }
        hull.pop_back();
        std::reverse(points.begin(), points.end());
    }
    return hull;
}

bool insideHull(const std::vector<std::array<float, 2>>& hull, float x, float y) {
    if (hull.size() < 3) return false;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto& a = hull[i];
        const auto& b = hull[(i + 1) % hull.size()];
        if (cross2(b[0] - a[0], b[1] - a[1], x - a[0], y - a[1]) < 0.0f) return false;
    }
    return true;
}

} // namespace

SupportWrenchResult3D routeSupportWrench3D(const SupportWrenchRequest3D& request,
    const BodyObservation3D& observation, const std::vector<Vec3>& linkCenters,
    const SupportWrenchSettings3D& settings) {
    SupportWrenchResult3D result;
    result.droppedForce = request.force;
    result.droppedTorque = request.torque;
    if (!observation.valid || observation.supportCount == 0 || request.bodyLink >= linkCenters.size()) return result;
    // A carga de cada pe apoiado (a medida pisca: um piso pela fracao do peso)
    // e a base: os cantos das solas apoiadas.
    std::array<float, 2> load {};
    float total = 0.0f;
    std::vector<std::array<float, 2>> corners;
    float ground = 1e9f;
    for (std::size_t side = 0; side < 2; ++side) {
        const FootObservation3D& foot = observation.feet[side];
        if (!foot.supporting) continue;
        load[side] = std::max(foot.loadNewtons, settings.minimumLoadShare * request.weightNewtons);
        total += load[side];
        ground = std::min(ground, foot.soleHeight);
        const Vec3 lateral = cross(Vec3 { 0.0f, 0.0f, 1.0f }, foot.forward);
        for (const float along : { -settings.soleHalfLength, settings.soleHalfLength })
            for (const float across : { -settings.soleHalfWidth, settings.soleHalfWidth }) {
                const Vec3 corner = foot.soleCenter + foot.forward * along + lateral * across;
                corners.push_back({ corner.x, corner.y });
            }
    }
    if (total <= 0.0f) return result;
    const std::vector<std::array<float, 2>> hull = convexHull(corners);
    const Vec3 bodyCenter = linkCenters[request.bodyLink];
    // O chao nao puxa: a forca para baixo vai no maximo ate tirar a carga.
    Vec3 force = request.force;
    if (force.z < -total) force.z = -total;
    const float normal = std::max(1.0f, total + force.z);
    // Atrito: a forca de lado ate mu vezes a normal.
    {
        const Vec3 lateral = horizontal(force);
        const float limit = settings.friction * normal;
        if (lateral.length() > limit && lateral.lengthSquared() > 1e-10f) {
            const Vec3 kept = lateral * (limit / lateral.length());
            force.x = kept.x;
            force.y = kept.y;
        }
    }
    Vec3 torque = request.torque;
    // A torcao (em volta da vertical) cabe no atrito das solas.
    const float twistLimit = normal * settings.friction * settings.torsionRadius * static_cast<float>(observation.supportCount);
    torque.z = std::clamp(torque.z, -twistLimit, twistLimit);
    // O centro de pressao que o pedido exige: com a reacao inteira do chao
    // (peso + forca) passando por ele, o momento em volta do centro de massa
    // do corpo inteiro e o pedido (pela pelve, numa base escalonada com a
    // pelve fora do poligono dos pes e o centro de massa dentro, nada
    // passava). Fora da base, a parte de lado da forca e o torque de
    // inclinacao encolhem juntos ate ele caber (sem o centro de massa sobre a
    // base, nada cabe: o corpo precisa de um passo).
    const Vec3 mass = observation.centerOfMass;
    const float height = std::max(0.2f, mass.z - ground);
    const auto pressureFor = [&](float scale, float& x, float& y) {
        const float fx = force.x * scale, fy = force.y * scale;
        const float tx = torque.x * scale, ty = torque.y * scale;
        x = mass.x + (-ty - height * fx) / normal;
        y = mass.y + (tx - height * fy) / normal;
    };
    float scale = 1.0f;
    float px = 0.0f, py = 0.0f;
    pressureFor(1.0f, px, py);
    if (!insideHull(hull, px, py)) {
        pressureFor(0.0f, px, py);
        if (!insideHull(hull, px, py)) {
            scale = 0.0f;
        } else {
            float low = 0.0f, high = 1.0f;
            for (int iteration = 0; iteration < 10; ++iteration) {
                const float middle = 0.5f * (low + high);
                pressureFor(middle, px, py);
                if (insideHull(hull, px, py)) low = middle;
                else high = middle;
            }
            scale = low;
        }
        force.x *= scale;
        force.y *= scale;
        torque.x *= scale;
        torque.y *= scale;
        pressureFor(1.0f, px, py);
    }
    result.droppedForce = request.force - force;
    result.droppedTorque = request.torque - torque;
    // A parte de cada pe: pela posicao do centro de pressao entre eles (com
    // um so, ele leva tudo).
    std::array<float, 2> share {};
    if (observation.supportCount == 1) {
        share[observation.feet[0].supporting ? 0 : 1] = 1.0f;
    } else {
        const Vec3 a = observation.feet[0].soleCenter;
        const Vec3 b = observation.feet[1].soleCenter;
        const Vec3 span { b.x - a.x, b.y - a.y, 0.0f };
        const float length = span.lengthSquared();
        const float towardB = length > 1e-6f
            ? std::clamp(((px - a.x) * span.x + (py - a.y) * span.y) / length, 0.0f, 1.0f) : 0.5f;
        share = { 1.0f - towardB, towardB };
    }
    Vec3 bodyForce, bodyTorque;
    for (std::size_t side = 0; side < 2; ++side) {
        const FootObservation3D& foot = observation.feet[side];
        if (!foot.supporting || foot.link >= linkCenters.size() || share[side] <= 0.0f) continue;
        const Vec3 footForce = force * share[side];
        const Vec3 footTorque = torque * share[side];
        bodyForce += footForce;
        bodyTorque += footTorque;
        // A reacao no pe: a forca contraria no centro de massa dele e o binario
        // que fecha o momento do par (a soma dos dois esforcos e zero).
        const Vec3 footCenter = linkCenters[foot.link];
        LinkWrench3D reaction;
        reaction.link = foot.link;
        reaction.force = footForce * -1.0f;
        reaction.torque = cross(bodyCenter - footCenter, footForce) * -1.0f - footTorque;
        result.wrenches.push_back(reaction);
    }
    LinkWrench3D action;
    action.link = request.bodyLink;
    action.force = bodyForce;
    action.torque = bodyTorque;
    result.wrenches.push_back(action);
    result.deliveredForce = bodyForce;
    result.deliveredTorque = bodyTorque;
    return result;
}

} // namespace MatterEngine
