#include "Engine/Tracking/TrackingBalance3D.hpp"

#include "Engine/RL/Body/RLBody3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace MatterEngine {
namespace {

float cross2(Vec3 o, Vec3 a, Vec3 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }

float segmentDistance2(Vec3 q, Vec3 a, Vec3 b) {
    const float ex = b.x - a.x, ey = b.y - a.y;
    const float length = ex * ex + ey * ey;
    const float t = length > 1.0e-12f ? std::clamp(((q.x - a.x) * ex + (q.y - a.y) * ey) / length, 0.0f, 1.0f) : 0.0f;
    return std::hypot(q.x - (a.x + t * ex), q.y - (a.y + t * ey));
}

} // namespace

void trackingSoleCorners3D(const RagdollLinkDefinition3D& link, const PhysicsBodyState3D& state, float groundHeight,
    std::span<Vec3, 4> out) {
    const bool box = ragdollColliderHasBox3D(link.collider);
    const Vec3 half = box ? link.collider.boxHalfExtents : Vec3 { 0.13f, 0.05f, 0.0f };
    const Vec3 center = box ? link.collider.localPosition : Vec3 {};
    const Quaternion local = box ? link.collider.localOrientation : Quaternion {};
    std::size_t n = 0;
    for (const float sx : { -1.0f, 1.0f })
        for (const float sy : { -1.0f, 1.0f }) {
            Vec3 p = state.position + state.orientation.rotate(center + local.rotate({ sx * half.x, sy * half.y, -half.z }));
            p.z = groundHeight;
            out[n++] = p;
        }
}

float trackingSignedHullDistance3D(std::span<const Vec3> points, Vec3 q) {
    if (points.empty()) return -1.0f;
    std::vector<Vec3> p(points.begin(), points.end());
    std::sort(p.begin(), p.end(), [](Vec3 a, Vec3 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    std::vector<Vec3> hull;
    hull.reserve(2 * p.size());
    // O fecho de Andrew (sentido anti-horario).
    for (const Vec3& v : p) {
        while (hull.size() >= 2 && cross2(hull[hull.size() - 2], hull.back(), v) <= 0.0f) hull.pop_back();
        hull.push_back(v);
    }
    const std::size_t lower = hull.size() + 1;
    for (std::size_t i = p.size(); i-- > 0;) {
        const Vec3& v = p[i];
        while (hull.size() >= lower && cross2(hull[hull.size() - 2], hull.back(), v) <= 0.0f) hull.pop_back();
        hull.push_back(v);
    }
    hull.pop_back();
    if (hull.size() < 3) {
        float best = std::numeric_limits<float>::max();
        if (hull.size() == 2) best = segmentDistance2(q, hull[0], hull[1]);
        else best = std::hypot(q.x - hull[0].x, q.y - hull[0].y);
        return -best;
    }
    float best = std::numeric_limits<float>::max();
    bool inside = true;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const Vec3& a = hull[i];
        const Vec3& b = hull[(i + 1) % hull.size()];
        best = std::min(best, segmentDistance2(q, a, b));
        inside = inside && cross2(a, b, q) >= 0.0f;
    }
    return inside ? best : -best;
}

TrackingBalance3D measureTrackingBalance3D(const RagdollProfile3D& profile, std::span<const PhysicsBodyState3D> links,
    std::span<const RagdollInteraction3D> contacts, std::array<std::uint32_t, 2> footLinks, float groundHeight) {
    TrackingBalance3D out;
    if (links.empty()) return out;
    out.centerOfMass = rlBodyCenterOfMass3D(profile, links);
    out.velocity = rlBodyCenterOfMassVelocity3D(profile, links);
    out.omega = std::sqrt(9.81f / std::max(0.4f, out.centerOfMass.z - groundHeight));
    out.capture = { out.centerOfMass.x + out.velocity.x / out.omega, out.centerOfMass.y + out.velocity.y / out.omega,
        groundHeight };
    std::array<std::array<Vec3, 4>, 2>& corners = out.corners;
    std::array<float, 2> impulse {};
    for (std::size_t side = 0; side < 2; ++side) {
        const std::uint32_t foot = footLinks[side];
        if (foot >= links.size() || foot >= profile.links.size()) continue;
        for (const RagdollInteraction3D& contact : contacts)
            if (contact.linkIndex == foot && contact.normalWorld.z > 0.5f) {
                out.footDown[side] = true;
                impulse[side] += std::max(0.0f, contact.normalImpulseNewtonSeconds);
            }
        trackingSoleCorners3D(profile.links[foot], links[foot], groundHeight, corners[side]);
        Vec3 sum {};
        for (const Vec3& c : corners[side]) sum += c;
        out.sole[side] = sum * 0.25f;
    }
    const bool any = out.footDown[0] || out.footDown[1];
    const float total = impulse[0] + impulse[1];
    if (total > 1.0e-4f) out.load = { impulse[0] / total, impulse[1] / total };
    else if (any) out.load = { out.footDown[0] ? (out.footDown[1] ? 0.5f : 1.0f) : 0.0f, out.footDown[1] ? (out.footDown[0] ? 0.5f : 1.0f) : 0.0f };
    std::vector<Vec3> base;
    Vec3 center {};
    int count = 0;
    for (std::size_t side = 0; side < 2; ++side) {
        if (any && !out.footDown[side]) continue;
        base.insert(base.end(), corners[side].begin(), corners[side].end());
        center += out.sole[side];
        ++count;
    }
    out.supportCenter = count > 0 ? center * (1.0f / static_cast<float>(count)) : Vec3 { out.capture.x, out.capture.y, groundHeight };
    out.margin = trackingSignedHullDistance3D(base, out.capture);
    Vec3 spin {};
    float mass = 0.0f;
    for (std::size_t i = 0; i < profile.links.size() && i < links.size(); ++i) {
        const float m = profile.links[i].massFraction;
        spin += cross(rlLinkCenterOfMass3D(profile.links[i], links[i]) - out.centerOfMass, links[i].linearVelocity - out.velocity) * m;
        mass += m;
    }
    out.spin = spin / std::max(1.0e-6f, mass);
    return out;
}

} // namespace MatterEngine
