#include "Engine/Physics/PhysicsEngine3D.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Physics/PhysX/PhysXInternals3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <extensions/PxExtensionsAPI.h>

#include <stdexcept>

namespace MatterEngine {

PhysicsEngine3D::Impl::~Impl() {
    if (extensionsInitialized) {
        PxCloseExtensions();
        extensionsInitialized = false;
    }
    if (physics != nullptr) {
        physics->release();
        physics = nullptr;
    }
    if (foundation != nullptr) {
        foundation->release();
        foundation = nullptr;
    }
}

PhysicsMesh3D::PhysicsMesh3D(std::shared_ptr<Impl> implementation,
    PhysicsMeshType3D type, std::uint32_t sourceTriangleCount)
    : m_impl(std::move(implementation)),
      m_type(type),
      m_sourceTriangleCount(sourceTriangleCount) {
}

PhysicsMesh3D::~PhysicsMesh3D() = default;

PhysicsEngine3D::PhysicsEngine3D()
    : PhysicsEngine3D(std::make_shared<TaskScheduler>()) {
}

PhysicsEngine3D::PhysicsEngine3D(
    std::shared_ptr<TaskScheduler> scheduler)
    : m_impl(std::make_shared<Impl>()) {
    if (!scheduler) {
        throw std::invalid_argument("PhysicsEngine3D exige um TaskScheduler");
    }
    m_impl->scheduler = std::move(scheduler);
    // A escala em metros define tolerancias internas coerentes para gravidade,
    // limiares de bounce, contato e estabilidade numerica.
    m_impl->scale.length = 1.0f;
    m_impl->scale.speed = 9.81f;
    m_impl->foundation = PxCreateFoundation(PX_PHYSICS_VERSION,
        m_impl->allocator, m_impl->errorCallback);
    if (m_impl->foundation == nullptr) {
        throw std::runtime_error("PhysX: falha ao criar PxFoundation");
    }

    m_impl->physics = PxCreatePhysics(PX_PHYSICS_VERSION,
        *m_impl->foundation, m_impl->scale, true, nullptr);
    if (m_impl->physics == nullptr) {
        m_impl->foundation->release();
        m_impl->foundation = nullptr;
        throw std::runtime_error("PhysX: falha ao criar PxPhysics");
    }
    if (!PxInitExtensions(*m_impl->physics, nullptr)) {
        m_impl->physics->release();
        m_impl->physics = nullptr;
        m_impl->foundation->release();
        m_impl->foundation = nullptr;
        throw std::runtime_error("PhysX: falha ao inicializar extensoes");
    }
    m_impl->extensionsInitialized = true;
    Log::info("Backend fisico NVIDIA PhysX 5.9.0 inicializado com "
        + std::to_string(m_impl->scheduler->workerCount())
        + " workers compartilhados.");
}

PhysicsEngine3D::~PhysicsEngine3D() = default;

std::unique_ptr<PhysicsScene3D> PhysicsEngine3D::createScene(
    const PhysicsSceneSettings3D& settings,
    const MaterialLibrary& materials) {
    return std::unique_ptr<PhysicsScene3D>(
        new PhysicsScene3D(*this, settings, materials));
}

} // namespace MatterEngine
