#include "Engine/Control/BipedMath3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

float cross2(Vec3 a, Vec3 b, Vec3 c) {
    return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x);
}

}

bool finiteBiped(Vec3 value) {
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

bool finiteBiped(Quaternion value) {
    const float n = value.x*value.x + value.y*value.y
        + value.z*value.z + value.w*value.w;
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z) && std::isfinite(value.w)
        && n > 1.0e-8f;
}

float clamp01Biped(float value) {
    return std::clamp(value, 0.0f, 1.0f);
}

float smoothstepBiped(float value) {
    const float t = clamp01Biped(value);
    return t*t*(3.0f-2.0f*t);
}

Vec3 clampLengthBiped(Vec3 value, float maximumLength) {
    if (!(maximumLength > 0.0f) || !finiteBiped(value)) return {};
    const float length = value.length();
    return length > maximumLength && length > 1.0e-7f
        ? value * (maximumLength/length) : value;
}

float componentBiped(Vec3 value, std::size_t axis) {
    return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

void setComponentBiped(Vec3& value, std::size_t axis, float component) {
    if (axis == 0) value.x = component;
    else if (axis == 1) value.y = component;
    else value.z = component;
}

Quaternion yawQuaternionBiped(float radians) {
    return Quaternion::fromAxisAngle({0.0f, 0.0f, 1.0f}, radians);
}

float yawFromForwardBiped(Vec3 forwardWorld) {
    forwardWorld.z = 0.0f;
    if (forwardWorld.lengthSquared() < 1.0e-10f) return 0.0f;
    return std::atan2(forwardWorld.y, forwardWorld.x);
}

Vec3 orientationErrorBiped(Quaternion desired, Quaternion current) {
    if (!finiteBiped(desired) || !finiteBiped(current)) return {};
    Quaternion q = (desired.normalized()*current.normalized().conjugate())
        .normalized();
    if (q.w < 0.0f) q = {-q.x, -q.y, -q.z, -q.w};
    const Vec3 v{q.x, q.y, q.z};
    const float s = v.length();
    if (s < 1.0e-7f) return {};
    const float angle = 2.0f*std::atan2(s, std::clamp(q.w, -1.0f, 1.0f));
    return v*(angle/s);
}

Quaternion quaternionFromToBiped(Vec3 from, Vec3 to) {
    if (from.lengthSquared() < 1.0e-10f
        || to.lengthSquared() < 1.0e-10f) {
        return {};
    }
    from = from.normalized();
    to = to.normalized();
    const float d = std::clamp(dot(from, to), -1.0f, 1.0f);
    if (d > 0.999999f) return {};
    if (d < -0.999999f) {
        Vec3 axis = std::abs(from.x) < 0.7f
            ? cross(from, Vec3{1.0f, 0.0f, 0.0f})
            : cross(from, Vec3{0.0f, 1.0f, 0.0f});
        if (axis.lengthSquared() < 1.0e-10f)
            axis = cross(from, Vec3{0.0f, 0.0f, 1.0f});
        return Quaternion::fromAxisAngle(axis.normalized(),
            3.14159265358979323846f);
    }
    const Vec3 axis = cross(from, to).normalized();
    return Quaternion::fromAxisAngle(axis, std::acos(d));
}

Quaternion alignOrientationUpBiped(
    Quaternion baseOrientation, Vec3 localUp, Vec3 desiredWorldUp) {
    if (!finiteBiped(baseOrientation)
        || localUp.lengthSquared() < 1.0e-10f
        || desiredWorldUp.lengthSquared() < 1.0e-10f) {
        return baseOrientation;
    }
    const Vec3 currentUp =
        baseOrientation.rotate(localUp.normalized()).normalized();
    return (quaternionFromToBiped(
        currentUp, desiredWorldUp.normalized())*baseOrientation).normalized();
}

std::vector<Vec3> convexHullXYBiped(std::span<const Vec3> points) {
    std::vector<Vec3> p;
    p.reserve(points.size());
    for (Vec3 value : points) {
        if (!finiteBiped(value)) continue;
        value.z = 0.0f;
        p.push_back(value);
    }
    std::sort(p.begin(), p.end(), [](Vec3 a, Vec3 b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    p.erase(std::unique(p.begin(), p.end(), [](Vec3 a, Vec3 b) {
        return std::abs(a.x-b.x) < 1.0e-5f
            && std::abs(a.y-b.y) < 1.0e-5f;
    }), p.end());
    if (p.size() <= 2) return p;

    std::vector<Vec3> hull;
    hull.reserve(p.size()*2);
    for (Vec3 value : p) {
        while (hull.size() >= 2
            && cross2(hull[hull.size()-2], hull.back(), value) <= 0.0f) {
            hull.pop_back();
        }
        hull.push_back(value);
    }
    const std::size_t lower = hull.size();
    for (std::size_t i = p.size()-1; i-- > 0;) {
        const Vec3 value = p[i];
        while (hull.size() > lower
            && cross2(hull[hull.size()-2], hull.back(), value) <= 0.0f) {
            hull.pop_back();
        }
        hull.push_back(value);
    }
    if (!hull.empty()) hull.pop_back();
    return hull;
}

Vec3 closestPointOnSegmentXYBiped(Vec3 point, Vec3 a, Vec3 b) {
    point.z = a.z = b.z = 0.0f;
    const Vec3 ab = b-a;
    const float d = ab.x*ab.x + ab.y*ab.y;
    if (d < 1.0e-12f) return a;
    const float t = std::clamp(
        ((point.x-a.x)*ab.x + (point.y-a.y)*ab.y)/d, 0.0f, 1.0f);
    return a+ab*t;
}

float signedDistanceToConvexPolygonXYBiped(
    Vec3 point, std::span<const Vec3> polygon) {
    if (polygon.empty()) return -std::numeric_limits<float>::infinity();
    point.z = 0.0f;
    if (polygon.size() == 1) {
        Vec3 d = point-polygon.front();
        d.z = 0.0f;
        return -d.length();
    }
    if (polygon.size() == 2) {
        Vec3 q = closestPointOnSegmentXYBiped(
            point, polygon[0], polygon[1]);
        Vec3 d = point-q;
        d.z = 0.0f;
        return -d.length();
    }

    bool inside = true;
    float minimumDistance = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Vec3 a = polygon[i];
        const Vec3 b = polygon[(i+1)%polygon.size()];
        if (cross2(a, b, point) < -1.0e-6f) inside = false;
        const Vec3 q = closestPointOnSegmentXYBiped(point, a, b);
        Vec3 d = point-q;
        d.z = 0.0f;
        minimumDistance = std::min(minimumDistance, d.length());
    }
    return inside ? minimumDistance : -minimumDistance;
}

} // namespace MatterEngine
