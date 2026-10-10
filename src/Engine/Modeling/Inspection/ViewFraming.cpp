#include "Engine/Modeling/Inspection/ViewFraming.hpp"

namespace MatterEngine::Modeling {

Box3 regionBounds(const InspectionScene& scene, int region) {
    Box3 box;
    if (scene.skeleton == nullptr) return box;
    const bool overview = scene.regions == nullptr || region < 0
        || static_cast<std::size_t>(region) >= scene.regions->regions.size()
        || scene.regions->regions[static_cast<std::size_t>(region)].overview;
    const std::vector<Vec3> joints = sceneJointPositions(scene);
    if (overview) {
        for (std::size_t j = 0; j < joints.size(); ++j) box.include(joints[j], scene.skeleton->joints[j].radius);
        box.include(Vec3 { box.center().x, box.center().y, scene.skeleton->groundHeight });
    } else {
        SkeletonPose pose;
        for (const Vec3& position : joints) pose.joints.push_back({ position, {} });
        box = scene.regions->bounds(region, *scene.skeleton, pose, scene.jointRegion);
    }
    if (scene.mesh != nullptr && !scene.mesh->empty()) {
        std::span<const Vec3> positions = scene.meshPositions;
        if (positions.empty()) positions = scene.mesh->positions;
        for (std::size_t t = 0; t < scene.mesh->triangles.size(); ++t) {
            if (!overview && partRegion(scene, scene.mesh->partOf(t)) != region) continue;
            for (const std::uint32_t v : scene.mesh->triangles[t])
                if (v < positions.size()) box.include(positions[v]);
        }
    }
    return box;
}

std::vector<FramePoint> regionPoints(const InspectionScene& scene, int region) {
    std::vector<FramePoint> points;
    if (scene.skeleton == nullptr) return points;
    const bool overview = scene.regions == nullptr || region < 0
        || static_cast<std::size_t>(region) >= scene.regions->regions.size()
        || scene.regions->regions[static_cast<std::size_t>(region)].overview;
    const std::vector<Vec3> joints = sceneJointPositions(scene);
    const Skeleton& skeleton = *scene.skeleton;
    // Com malha na regiao, ela manda: os ossos entram sem o raio do corpo.
    bool regionHasMesh = false;
    if (scene.mesh != nullptr)
        for (std::size_t p = 0; p < scene.mesh->parts.size() && !regionHasMesh; ++p)
            regionHasMesh = overview || partRegion(scene, static_cast<int>(p)) == region;
    for (std::size_t j = 0; j < joints.size(); ++j) {
        const int owner = j < scene.jointRegion.size() ? scene.jointRegion[j] : -1;
        if (!overview && owner != region) continue;
        const float radius = regionHasMesh ? 0.0f : skeleton.joints[j].radius;
        points.push_back({ joints[j], radius });
        // O osso que sai daqui ate cada filha (o meio dele tambem).
        for (std::size_t child = j + 1; child < joints.size(); ++child)
            if (skeleton.joints[child].parent == static_cast<int>(j)) {
                points.push_back({ joints[child], radius });
                points.push_back({ (joints[j] + joints[child]) * 0.5f, radius });
            }
    }
    if (overview) points.push_back({ Vec3 { joints.front().x, joints.front().y, skeleton.groundHeight }, 0.0f });
    if (scene.mesh != nullptr && !scene.mesh->empty()) {
        std::span<const Vec3> positions = scene.meshPositions;
        if (positions.empty()) positions = scene.mesh->positions;
        // Um vertice por triangulo basta para enquadrar.
        for (std::size_t t = 0; t < scene.mesh->triangles.size(); ++t) {
            if (!overview && partRegion(scene, scene.mesh->partOf(t)) != region) continue;
            const std::uint32_t v = scene.mesh->triangles[t][t % 3];
            if (v < positions.size()) points.push_back({ positions[v], 0.0f });
        }
    }
    return points;
}

OrbitCamera frameBounds(const Box3& box, float yawDegrees, float pitchDegrees, float aspect, float margin,
    bool orthographic) {
    OrbitCamera camera;
    camera.yaw = yawDegrees * DegreesToRadians;
    camera.pitch = pitchDegrees * DegreesToRadians;
    camera.orthographic = orthographic;
    frameBox(camera, box, aspect, margin);
    return camera;
}

OrbitCamera frameRegion(const InspectionScene& scene, int region, float aspect, const ViewPreset* view,
    bool orthographic) {
    float yaw = 25.0f, pitch = 8.0f, margin = 1.2f;
    if (scene.regions != nullptr && region >= 0 && static_cast<std::size_t>(region) < scene.regions->regions.size()) {
        const BodyRegion& definition = scene.regions->regions[static_cast<std::size_t>(region)];
        yaw = definition.yawDegrees;
        pitch = definition.pitchDegrees;
        margin = definition.margin;
    }
    if (view != nullptr) {
        yaw = view->yawDegrees;
        pitch = view->pitchDegrees;
    }
    OrbitCamera camera;
    camera.yaw = yaw * DegreesToRadians;
    camera.pitch = pitch * DegreesToRadians;
    camera.orthographic = orthographic;
    const std::vector<FramePoint> points = regionPoints(scene, region);
    if (points.empty()) frameBox(camera, regionBounds(scene, region), aspect, margin);
    else framePoints(camera, points, aspect, margin);
    return camera;
}

} // namespace MatterEngine::Modeling
