#include "Engine/Animation/MotionMatching/MotionMatchingSchema3D.hpp"

#include <algorithm>
#include <cstdio>

namespace MatterEngine {

namespace {

std::string seconds(float t) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.3f", static_cast<double>(t));
    return text;
}

using Feature = MotionMatchingBoneFeature3D;
using Bone = MotionMatchingBoneChannel3D;

} // namespace

MotionMatchingSchema3D MotionMatchingSchema3D::unrealDefault() {
    MotionMatchingSchema3D schema;
    schema.trajectoryWeight = 7.0f;
    schema.trajectory = {
        { -0.4f, true, false, false, false, 0.4f },
        { 0.0f, false, true, false, true, 2.0f },
        { 0.35f, true, false, false, true, 0.7f },
        { 0.7f, true, true, false, true, 0.5f },
    };
    schema.bones = {
        { Feature::Position, LeftFoot, Bone::Root, 1.0f, "", "pe_esquerdo" },
        { Feature::Velocity, LeftFoot, Bone::Root, 1.0f, "", "pe_esquerdo" },
        { Feature::Position, RightFoot, Bone::Root, 1.0f, "", "pe_direito" },
        { Feature::Velocity, RightFoot, Bone::Root, 1.0f, "", "pe_direito" },
    };
    return schema;
}

MotionMatchingSchema3D MotionMatchingSchema3D::gaspDefault() {
    MotionMatchingSchema3D schema;
    schema.trajectoryWeight = 1.0f;
    schema.trajectory = {
        { -0.05f, true, false, false, false, 0.3f },
        { 0.0f, false, true, false, true, 1.0f },
        { 0.35f, true, false, false, true, 1.0f },
        { 0.7f, true, true, false, true, 1.0f },
        { 1.0f, false, false, true, false, 1.5f },
    };
    schema.bones = {
        { Feature::Position, LeftFoot, RightFoot, 1.0f, "", "pe_esquerdo_do_direito" },
        { Feature::Velocity, LeftFoot, Bone::Root, 0.3f, "FeetVelZ", "pe_esquerdo" },
        { Feature::Velocity, RightFoot, Bone::Root, 0.3f, "FeetVelZ", "pe_direito" },
        { Feature::Heading, PelvisLateral, Bone::Root, 0.1f, "", "pelve_lateral" },
    };
    return schema;
}

MotionMatchingSchema3D MotionMatchingSchema3D::gaspStop() {
    MotionMatchingSchema3D schema = gaspDefault();
    schema.trajectoryWeight = 5.0f;
    schema.trajectory = {
        { -0.05f, true, false, false, false, 0.3f },
        { 0.0f, false, true, true, true, 2.0f },
        { 0.35f, true, false, false, true, 1.0f },
        { 0.7f, true, true, false, true, 1.0f },
    };
    schema.bones.back().weight = 0.3f;
    return schema;
}

MotionMatchingSchema3D MotionMatchingSchema3D::gaspIdle() {
    MotionMatchingSchema3D schema = gaspDefault();
    schema.trajectory.pop_back();
    // (O canal Pose do PSS_Idle: a pelve, posicao e as duas direcoes do giro dela.)
    schema.bones.pop_back();
    schema.bones.push_back({ Feature::Position, Pelvis, Bone::Root, 1.0f, "", "pelve" });
    schema.bones.push_back({ Feature::Heading, PelvisLateral, Bone::Root, 1.0f, "", "pelve_lateral_pose" });
    schema.bones.push_back({ Feature::Heading, PelvisForward, Bone::Root, 1.0f, "", "pelve_frente_pose" });
    return schema;
}

std::vector<MotionMatchingSchema3D::Leaf> MotionMatchingSchema3D::leaves() const {
    std::vector<Leaf> out;
    std::size_t at = 0;
    const auto add = [&](std::size_t size, float weight, std::string label) {
        out.push_back({ at, size, weight, std::move(label) });
        at += size;
    };
    for (const MotionMatchingTrajectorySample3D& s : trajectory) {
        const std::string when = seconds(s.offsetSeconds);
        if (s.velocity) add(2, s.weight * trajectoryWeight, "trajetoria_velocidade@" + when);
        if (s.velocityDirection) add(2, s.weight * trajectoryWeight, "trajetoria_direcao@" + when);
        if (s.position) add(2, s.weight * trajectoryWeight, "trajetoria_posicao@" + when);
        if (s.facing) add(2, s.weight * trajectoryWeight, "trajetoria_rumo@" + when);
    }
    for (const MotionMatchingBoneChannel3D& b : bones) {
        std::string label;
        switch (b.feature) {
        case Feature::Position:
            label = "posicao_" + std::to_string(b.index) + "_de_" + (b.origin == Bone::Root ? std::string("raiz") : std::to_string(b.origin));
            add(3, b.weight, b.normalizationGroup.empty() ? label : "grupo_" + b.normalizationGroup);
            break;
        case Feature::Velocity:
            label = "velocidade_" + std::to_string(b.index);
            add(3, b.weight, b.normalizationGroup.empty() ? label : "grupo_" + b.normalizationGroup);
            break;
        case Feature::Heading:
            label = "eixo_" + std::to_string(b.index);
            add(2, b.weight, b.normalizationGroup.empty() ? label : "grupo_" + b.normalizationGroup);
            break;
        }
    }
    return out;
}

std::size_t MotionMatchingSchema3D::featureSize() const {
    const std::vector<Leaf> all = leaves();
    return all.empty() ? 0 : all.back().begin + all.back().size;
}

float MotionMatchingSchema3D::pastHorizonSeconds() const {
    float past = 0.0f;
    for (const MotionMatchingTrajectorySample3D& s : trajectory) past = std::max(past, -s.offsetSeconds);
    return past;
}

float MotionMatchingSchema3D::futureHorizonSeconds() const {
    float future = 0.0f;
    for (const MotionMatchingTrajectorySample3D& s : trajectory) future = std::max(future, s.offsetSeconds);
    return future;
}

} // namespace MatterEngine
