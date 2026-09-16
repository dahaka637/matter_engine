#include "Engine/Audio/DragAcoustics.hpp"

#include "Engine/Audio/WindAmbienceRamp.hpp"

namespace MatterEngine {
namespace {

// Mesmos limiares/referencia pra todo o catalogo de materiais (nao por
// material) - o pedido original distinguia por PESO do objeto arrastado,
// nao por tipo de superficie, entao um unico par de rampas globais basta e
// e mais facil de calibrar (ver DragAcoustics.hpp).
//
// Forca normal (N): abaixo de ~8N (acima do peso de uma bola de futebol,
// 0.435kg * 9.81 ~ 4.3N, mass_kg autorada em soccer_ball.glb) fica em
// silencio; 100N ja satura a rampa. Calibrado pelo objeto MODERADO (uma
// cadeira de madeira, 8.8kg * 9.81 ~ 86N, ja fica bem audivel), nao pelo
// extremo - ancorar a referencia no peso da bigorna (55kg * 9.81 ~ 540N)
// espremia qualquer objeto mais leve pra uma fatia minuscula da rampa; a
// bigorna so precisa ULTRAPASSAR a referencia pra soar no maximo, nao ficar
// exatamente nela.
constexpr float DragQuietForceNewtons = 8.0f;
constexpr float DragReferenceForceNewtons = 100.0f;
// Velocidade tangencial (m/s): filtra jitter de repouso (quase zero) e
// rolamento sem deslizar, que tem velocidade tangencial quase nula no ponto
// de contato por definicao fisica - e isso que deixa uma bola ROLANDO
// sutil mesmo sendo empurrada com forca, sem nenhum caso especial "isso e
// uma esfera".
constexpr float DragQuietTangentialSpeedMetersPerSecond = 0.05f;
constexpr float DragReferenceTangentialSpeedMetersPerSecond = 1.5f;
// Abaixo disto nem vale gerar o comando.
constexpr float MinimumDragIntensity = 0.01f;

std::uint64_t stableBodyKey(std::uint64_t bodyId, std::size_t bodyIndex) {
    if (bodyId != 0) return bodyId;
    if (bodyIndex == InvalidPhysicsBodyIndex) return 0;
    // O bit alto separa o fallback por indice dos IDs persistentes pequenos
    // (mesmo padrao de ImpactAcoustics.cpp).
    return (std::uint64_t { 1 } << 63)
        | static_cast<std::uint64_t>(bodyIndex + 1);
}

} // namespace

void DragAcousticResolver::resolve(
    std::span<const ContactSlideEvent3D> slides,
    const MaterialLibrary& materials, float deltaTime) {
    m_commands.clear();
    if (deltaTime <= 0.0f) return;

    const auto emitBody = [&](const ContactSlideEvent3D& slide,
            bool emitA) {
        const bool staticBody = emitA ? slide.staticA : slide.staticB;
        if (staticBody) return;   // uma superficie estatica sozinha nao "arrasta"

        const std::string& materialId = emitA
            ? slide.materialA : slide.materialB;
        const SurfaceMaterial* material = materials.find(materialId);
        if (material == nullptr) return;

        // Impulso normal vira forca dividindo pelo passo fixo - mesmo padrao
        // ja usado em ImpactAcoustics.cpp para approachSpeed =
        // normalImpulse/effectiveMass. A velocidade tangencial ja vem
        // pronta (calculada por cinematica em PhysXScene3D::onContact, nao
        // por impulso - ver comentario em ContactSlideEvent3D).
        const float normalForce =
            slide.normalImpulseNewtonSeconds / deltaTime;
        const float tangentialSpeed = slide.tangentialSpeedMetersPerSecond;

        const float forceRamp = windAmbienceVolumeRamp(normalForce,
            DragQuietForceNewtons, DragReferenceForceNewtons, 1.0f);
        const float speedRamp = windAmbienceVolumeRamp(tangentialSpeed,
            DragQuietTangentialSpeedMetersPerSecond,
            DragReferenceTangentialSpeedMetersPerSecond, 1.0f);
        const float intensity = forceRamp * speedRamp;
        if (intensity < MinimumDragIntensity) return;

        DragSoundCommand3D command;
        command.sourceBodyId = stableBodyKey(
            emitA ? slide.bodyIdA : slide.bodyIdB,
            emitA ? slide.bodyA : slide.bodyB);
        command.materialId = materialId;
        command.soundSet = material->acoustic.rollingSoundSet;
        command.position = slide.position;
        command.intensity = intensity;
        m_commands.push_back(std::move(command));
    };

    for (const ContactSlideEvent3D& slide : slides) {
        emitBody(slide, true);
        emitBody(slide, false);
    }
}

} // namespace MatterEngine
