#pragma once

#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Core/TaskScheduler.hpp"

#include <PxPhysicsAPI.h>

namespace MatterEngine {

inline physx::PxVec3 toPhysX(Vec3 value) {
    return { value.x, value.y, value.z };
}

inline Vec3 fromPhysX(const physx::PxVec3& value) {
    return { value.x, value.y, value.z };
}

inline physx::PxQuat toPhysX(Quaternion value) {
    const Quaternion normalized = value.normalized();
    return { normalized.x, normalized.y, normalized.z, normalized.w };
}

inline Quaternion fromPhysX(const physx::PxQuat& value) {
    return { value.x, value.y, value.z, value.w };
}

inline physx::PxTransform toPhysX(Vec3 position, Quaternion orientation) {
    return { toPhysX(position), toPhysX(orientation) };
}

struct PhysicsEngine3D::Impl final
    : std::enable_shared_from_this<PhysicsEngine3D::Impl> {
    physx::PxDefaultAllocator allocator;
    physx::PxDefaultErrorCallback errorCallback;
    physx::PxFoundation* foundation = nullptr;
    physx::PxPhysics* physics = nullptr;
    physx::PxTolerancesScale scale;
    std::shared_ptr<TaskScheduler> scheduler;
    bool extensionsInitialized = false;

    ~Impl();
};

struct PhysicsMesh3D::Impl final {
    // Mantem Foundation/Physics vivos ate a ultima mesh nativa ser liberada.
    std::shared_ptr<PhysicsEngine3D::Impl> engineLifetime;
    physx::PxConvexMesh* convex = nullptr;
    physx::PxTriangleMesh* triangle = nullptr;

    ~Impl() {
        if (convex != nullptr) convex->release();
        if (triangle != nullptr) triangle->release();
    }
};

} // namespace MatterEngine
