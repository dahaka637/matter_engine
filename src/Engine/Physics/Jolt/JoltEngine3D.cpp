#include "Engine/Core/Log.hpp"
#include "Engine/Physics/Jolt/JoltInternals3D.hpp"

#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

#include <cstdarg>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace MatterEngine {
namespace JoltDetail {
namespace {

// O Jolt roteia diagnostico por dois ponteiros de funcao globais. Sem
// instala-los, um aviso do SDK (shape degenerada, corpo fora de limites,
// orcamento de contatos estourado) desaparece silenciosamente.
void traceToEngineLog(const char* format, ...) {
    char message[1024];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);
    Log::warn(std::string("Jolt: ") + message);
}

#ifdef JPH_ENABLE_ASSERTS
// Um assert do Jolt indica uso indevido de API (corpo nao travado, camada
// invalida, constraint incoerente).
//
// Deliberadamente NAO lanca: este handler e chamado de dentro de um frame do
// SDK, e mesmo com tabelas de unwind disponiveis, atravessar o solver com uma
// excecao deixaria corpos e constraints em estado parcialmente atualizado. O
// registro em nivel de erro e ruidoso o suficiente para aparecer na saida de
// teste, e devolver false segue sem interromper (retornar true pediria um
// breakpoint, inutil numa execucao sem depurador).
bool assertFailedToLog(const char* expression, const char* message,
    const char* file, JPH::uint line) {
    Log::error(std::string("Jolt assert em ") + file + ":"
        + std::to_string(line) + " (" + expression + "): "
        + (message != nullptr ? message : ""));
    return false;
}
#endif

} // namespace

std::mutex GlobalRegistration::s_mutex;
std::size_t GlobalRegistration::s_useCount = 0;

void GlobalRegistration::acquire() {
    const std::lock_guard<std::mutex> lock(s_mutex);
    if (s_useCount++ > 0) return;

    // Ordem exigida pelo SDK: alocador, Factory, tipos. O alocador default do
    // Jolt encaminha para malloc/free; nao ha ganho medido em substituí-lo, e
    // um alocador proprio seria mais uma peca a manter correta.
    JPH::RegisterDefaultAllocator();
    JPH::Trace = &traceToEngineLog;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = &assertFailedToLog;)
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
}

void GlobalRegistration::release() {
    const std::lock_guard<std::mutex> lock(s_mutex);
    if (s_useCount == 0 || --s_useCount > 0) return;

    JPH::UnregisterTypes();
    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}

std::uint64_t packBodyIdentity(const BodyIdentity& identity) {
    return (static_cast<std::uint64_t>(identity.kind) << 56)
        | (static_cast<std::uint64_t>(identity.linkIndex) << 32)
        | static_cast<std::uint64_t>(identity.slotIndex);
}

BodyIdentity unpackBodyIdentity(std::uint64_t userData) {
    BodyIdentity identity;
    identity.kind = static_cast<BodyKind>((userData >> 56) & 0xFFull);
    identity.linkIndex =
        static_cast<std::uint32_t>((userData >> 32) & 0xFFFFFFull);
    identity.slotIndex = static_cast<std::uint32_t>(userData & 0xFFFFFFFFull);
    return identity;
}

} // namespace JoltDetail

PhysicsEngine3D::Impl::~Impl() {
    if (registrationHeld) {
        JoltDetail::GlobalRegistration::release();
        registrationHeld = false;
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
    JoltDetail::GlobalRegistration::acquire();
    m_impl->registrationHeld = true;
    Log::info("Backend fisico Jolt Physics 5.6.0 inicializado com "
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
