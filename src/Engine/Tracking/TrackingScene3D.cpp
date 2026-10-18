#include "Engine/Tracking/TrackingScene3D.hpp"

#include <span>

namespace MatterEngine {

PhysicsSceneSettings3D trackingSceneSettings3D() {
    PhysicsSceneSettings3D scene;
    scene.workerThreadCount = 1;
    scene.runJobsInline = true;
    scene.maximumBodies = 128u;
    scene.maximumBodyPairs = 4096u;
    scene.maximumContactConstraints = 2048u;
    scene.scratchBufferSizeBytes = 4u * 1024u * 1024u;
    return scene;
}

void createTrackingGround3D(PhysicsScene3D& scene, float halfExtentMeters) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = { halfExtentMeters, halfExtentMeters, 0.1f };
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D ground;
    ground.motionType = PhysicsMotionType3D::Static;
    ground.position = { 0.0f, 0.0f, -0.1f };
    ground.materialId = "concrete";
    (void)scene.createBody(ground, std::span<const PhysicsShape3D>(&shape, 1));
}

} // namespace MatterEngine
