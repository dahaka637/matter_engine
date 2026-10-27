#include "Engine/BodyControl/Planning/StepRecovery3D.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace MatterEngine {
namespace {

double cross2(const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    return (a - o).x() * (b - o).y() - (a - o).y() * (b - o).x();
}

std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> points) {
    std::sort(points.begin(), points.end(), [](const auto& a, const auto& b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
    if (points.size() < 3) return points;
    std::vector<Eigen::Vector2d> hull;
    for (int pass = 0; pass < 2; ++pass) {
        const std::size_t start = hull.size();
        for (std::size_t i = 0; i < points.size(); ++i) {
            const Eigen::Vector2d& p = pass == 0 ? points[i] : points[points.size() - 1 - i];
            while (hull.size() >= start + 2 && cross2(hull[hull.size() - 2], hull.back(), p) <= 0.0) hull.pop_back();
            hull.push_back(p);
        }
        hull.pop_back();
    }
    return hull;
}

} // namespace

double stepRecoveryOmega3D(const StepRecoverySettings3D& settings) {
    return std::sqrt(settings.gravity / std::max(0.3, settings.comHeight));
}

double polygonOutside3D(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point) {
    const std::vector<Eigen::Vector2d> hull = convexHull(polygon);
    if (hull.empty()) return std::numeric_limits<double>::infinity();
    if (hull.size() == 1) return (point - hull[0]).norm();
    if (hull.size() == 2) {
        const Eigen::Vector2d edge = hull[1] - hull[0];
        const double t = std::clamp((point - hull[0]).dot(edge) / edge.squaredNorm(), 0.0, 1.0);
        return (point - (hull[0] + t * edge)).norm();
    }
    // Contra-horario: a distancia com sinal ate cada aresta (positiva fora).
    double outside = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const Eigen::Vector2d a = hull[i], b = hull[(i + 1) % hull.size()];
        const Eigen::Vector2d edge = (b - a).normalized();
        outside = std::max(outside, -(edge.x() * (point - a).y() - edge.y() * (point - a).x()));
    }
    return outside;
}

Eigen::Vector2d replanLanding3D(const StepRecoverySettings3D& settings, const Eigen::Vector2d& com, const Eigen::Vector2d& comVelocity,
    const Eigen::Vector2d& stanceCenter, const Eigen::Vector2d& swingOutward, double remaining) {
    const double omega = stepRecoveryOmega3D(settings);
    const Eigen::Vector2d capture = com + comVelocity / omega;
    const Eigen::Vector2d predicted = stanceCenter + std::exp(omega * std::max(0.0, remaining)) * (capture - stanceCenter);
    Eigen::Vector2d offset = predicted - stanceCenter;
    if (offset.norm() > 1e-6) offset += settings.overshoot * offset.normalized();
    const double width = offset.dot(swingOutward);
    if (width < settings.minimumWidth) offset += (settings.minimumWidth - width) * swingOutward;
    if (offset.norm() > settings.maximumReach) offset *= settings.maximumReach / offset.norm();
    return stanceCenter + offset;
}

Eigen::Vector2d projectToPolygon3D(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point, double margin) {
    const std::vector<Eigen::Vector2d> hull = convexHull(polygon);
    if (hull.empty()) return point;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (const Eigen::Vector2d& p : hull) center += p / double(hull.size());
    if (hull.size() < 3) {
        // Um ponto ou um segmento (o pe numa linha, nota 29): o mais perto, recuado `margin` das pontas.
        const Eigen::Vector2d a = hull.front(), edge = hull.back() - a;
        const double length = edge.norm();
        if (length <= 2.0 * margin) return center;
        return a + std::clamp((point - a).dot(edge) / (length * length), margin / length, 1.0 - margin / length) * edge;
    }
    // O poligono recuado: cada vertice puxado `margin` para o centro (aproximacao do offset).
    std::vector<Eigen::Vector2d> inner;
    for (const Eigen::Vector2d& p : hull) {
        const Eigen::Vector2d toCenter = center - p;
        const double length = toCenter.norm();
        inner.push_back(length > margin ? Eigen::Vector2d(p + toCenter * (margin / length)) : center);
    }
    if (polygonOutside3D(inner, point) <= 0.0) return point;
    Eigen::Vector2d best = center;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < inner.size(); ++i) {
        const Eigen::Vector2d a = inner[i], b = inner[(i + 1) % inner.size()];
        const Eigen::Vector2d edge = b - a;
        const double t = std::clamp((point - a).dot(edge) / std::max(1e-12, edge.squaredNorm()), 0.0, 1.0);
        const Eigen::Vector2d candidate = a + t * edge;
        const double distance = (point - candidate).norm();
        if (distance < bestDistance) { bestDistance = distance; best = candidate; }
    }
    return best;
}

StepRecoveryPlan3D planRecoveryStep3D(const StepRecoverySettings3D& settings, const Eigen::Vector2d& com,
    const Eigen::Vector2d& comVelocity, const std::vector<Eigen::Vector2d>& support,
    const std::array<StepRecoveryFoot3D, 2>& feet) {
    StepRecoveryPlan3D plan;
    const double omega = stepRecoveryOmega3D(settings);
    plan.capturePoint = com + comVelocity / omega;
    plan.outside = polygonOutside3D(support, plan.capturePoint);
    if (plan.outside < settings.triggerMargin || !feet[0].supporting || !feet[1].supporting) return plan;
    // Pe e tempo escolhidos juntos (nota 26 do vault): o CoP util e a projecao do DCM
    // no pe que fica; a previsao usa esse CoP; alcance excedido rejeita (nao recorta).
    double best = std::numeric_limits<double>::infinity();
    for (int swing = 0; swing < 2; ++swing) {
        const StepRecoveryFoot3D& stance = feet[static_cast<std::size_t>(1 - swing)];
        const StepRecoveryFoot3D& moving = feet[static_cast<std::size_t>(swing)];
        const Eigen::Vector2d cop = !stance.support.empty() ? projectToPolygon3D(stance.support, plan.capturePoint, 0.02) : stance.center;
        for (const double duration : settings.durations) {
            const Eigen::Vector2d predicted = cop + std::exp(omega * duration) * (plan.capturePoint - cop);
            Eigen::Vector2d offset = predicted - stance.center;
            const Eigen::Vector2d direction = predicted - cop;
            if (direction.norm() > 1e-6) offset += settings.overshoot * direction.normalized();
            const double width = offset.dot(moving.outward);
            if (width < settings.minimumWidth) offset += (settings.minimumWidth - width) * moving.outward;
            if (offset.norm() > settings.maximumReach) continue;
            const Eigen::Vector2d landing = stance.center + offset;
            const double peak = 5.774 * (landing - moving.center).norm() / (duration * duration);
            if (peak > settings.maximumSwingAcceleration) continue;
            if (peak < best) {
                best = peak;
                plan.step = true;
                plan.swingFoot = swing;
                plan.landing = landing;
                plan.duration = duration;
                plan.stanceCop = cop;
            }
        }
    }
    return plan;
}

SwingSample3D sampleSwing3D(const Eigen::Vector3d& start, const Eigen::Vector3d& landing, double height, double duration,
    double time) {
    SwingSample3D sample;
    const double T = std::max(1e-3, duration);
    const double t = std::clamp(time / T, 0.0, 1.0);
    // Quintica: posicao, velocidade e aceleracao zero nas pontas.
    const double s = t * t * t * (10.0 - 15.0 * t + 6.0 * t * t);
    const double ds = 30.0 * t * t * (1.0 - t) * (1.0 - t) / T;
    const double dds = 60.0 * t * (1.0 - t) * (1.0 - 2.0 * t) / (T * T);
    // O arco da altura: 64 t^3 (1-t)^3 (pico 1 no meio).
    const double a = 64.0 * std::pow(t, 3) * std::pow(1.0 - t, 3);
    const double da = 64.0 * 3.0 * t * t * std::pow(1.0 - t, 2) * (1.0 - 2.0 * t) / T;
    const double dda = 64.0 * 6.0 * t * (1.0 - t) * (1.0 - 5.0 * t + 5.0 * t * t) / (T * T);
    const Eigen::Vector3d delta = landing - start;
    sample.position = start + s * delta + Eigen::Vector3d(0.0, 0.0, height * a);
    sample.velocity = ds * delta + Eigen::Vector3d(0.0, 0.0, height * da);
    sample.acceleration = dds * delta + Eigen::Vector3d(0.0, 0.0, height * dda);
    if (time >= T) {
        sample.velocity.setZero();
        sample.acceleration.setZero();
    }
    return sample;
}

} // namespace MatterEngine
