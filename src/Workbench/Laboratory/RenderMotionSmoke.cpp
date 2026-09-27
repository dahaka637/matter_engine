#include "Workbench/WorkbenchApp.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Math/Mat4.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace MatterEngine::Workbench {

// Smokes de suavidade de movimento no render.
//
//   laboratory-camera-orbit-smoke  personagem controlado, camera em orbita a
//                                  velocidade constante, ponto PARADO ao lado
//                                  dele (a caixa do relato do usuario);
//   laboratory-prop-motion-smoke   camera parada, uma caixa caindo pela tela,
//                                  medida na queda e depois de dormir.
//
// Os dois medem a mesma coisa: quanto a posicao na tela se desvia de uma
// trajetoria suave. Em cada quadro, a distancia em pixels entre onde o ponto
// aparece e onde estaria mantendo a velocidade do quadro anterior. Uma
// trajetoria suave fica na ordem de 0,1 px; um quadro em que camera, corpo e
// passo fixo estao em instantes diferentes aparece como varios pixels.
//
// O sintoma e de render - fisica e locomocao ja rodam em lockstep nos testes
// headless -, entao so o aplicativo o alcanca.

namespace {

constexpr float WarmupSeconds = 1.5f;
constexpr float MeasureSeconds = 5.0f;
// Uma virada rapida de mouse: ~230 graus por segundo.
constexpr float OrbitRadiansPerSecond = 4.0f;
// A "caixa" do relato: ao lado do personagem, na altura do quadril.
const Vec3 OrbitProbeOffsetFromPelvis { 1.0f, 0.0f, 0.0f };
// Caixa solta a frente da camera, dentro do campo de visao.
constexpr float FallDistanceAheadMeters = 6.0f;
constexpr float FallHeightAboveEyeMeters = 3.0f;
// Desvio acima disto conta como salto visivel.
constexpr float VisibleJumpPixels = 1.0f;

bool projectToPixels(const Mat4& viewProjection, Vec3 point, float width,
    float height, float& outX, float& outY) {
    const auto clipRow = [&](int row) {
        return viewProjection.at(row, 0) * point.x
            + viewProjection.at(row, 1) * point.y
            + viewProjection.at(row, 2) * point.z + viewProjection.at(row, 3);
    };
    const float w = clipRow(3);
    if (w <= 0.001f) return false;
    outX = clipRow(0) / w * width * 0.5f;
    outY = clipRow(1) / w * height * 0.5f;
    return true;
}

void summarize(const char* label, const std::vector<float>& errors) {
    if (errors.empty()) {
        Log::error(std::string("Smoke movimento [") + label
            + "]: nenhuma amostra.");
        return;
    }
    std::vector<float> sorted = errors;
    std::sort(sorted.begin(), sorted.end());
    const auto visible = static_cast<std::size_t>(std::count_if(
        sorted.begin(), sorted.end(),
        [](float error) { return error > VisibleJumpPixels; }));
    Log::info(std::string("Smoke movimento [") + label + "]: "
        + std::to_string(sorted.size())
        + " quadros; desvio da trajetoria mediana "
        + std::to_string(sorted[sorted.size() / 2]) + " px, P95 "
        + std::to_string(sorted[sorted.size() * 95 / 100]) + " px, pior "
        + std::to_string(sorted.back()) + " px; quadros com salto > 1 px: "
        + std::to_string(visible) + " ("
        + std::to_string(100.0f * static_cast<float>(visible)
            / static_cast<float>(sorted.size()))
        + "%).");
}

} // namespace

void WorkbenchApp::sampleRenderMotionSmoke(float viewportWidth,
    float viewportHeight) {
    RenderMotionSmoke& smoke = m_renderMotionSmoke;
    if (smoke.mode == RenderMotionSmoke::Mode::Disabled || smoke.finished) {
        return;
    }
    if (smoke.mode == RenderMotionSmoke::Mode::CameraOrbit
        && m_controlledCharacterEntityId == 0) {
        return;
    }
    if (!m_laboratoryMapLoaded) return;

    const auto now = std::chrono::steady_clock::now();
    if (!smoke.started) {
        smoke.started = true;
        smoke.startTime = now;
    }
    const float elapsed =
        std::chrono::duration<float>(now - smoke.startTime).count();
    if (elapsed < WarmupSeconds) return;

    // O que e observado.
    Vec3 probe;
    bool moving = true;
    if (smoke.mode == RenderMotionSmoke::Mode::CameraOrbit) {
        if (!smoke.probePlaced) {
            const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
                m_spawnedRagdolls.end(),
                [&](const SpawnedRagdollInstance& item) {
                    return item.entityId == m_controlledCharacterEntityId;
                });
            if (controlled == m_spawnedRagdolls.end()
                || controlled->physicsState.links.empty()) {
                return;
            }
            smoke.probeWorld = controlled->physicsState.links.front().position
                + OrbitProbeOffsetFromPelvis;
            smoke.probePlaced = true;
        }
        probe = smoke.probeWorld;
    } else {
        if (!smoke.probePlaced) {
            const auto& definitions = m_propCatalog.definitions();
            const auto crate = std::find_if(definitions.begin(),
                definitions.end(), [](const PropDefinition3D& definition) {
                    return definition.id == "wood_crate";
                });
            if (crate == definitions.end()) {
                Log::error("Smoke movimento: prop wood_crate ausente.");
                smoke.finished = true;
                m_automaticQuitSeconds = 0.001f;
                return;
            }
            const float yaw = m_laboratoryCameraYaw;
            const Vec3 ahead { std::cos(yaw) * FallDistanceAheadMeters,
                std::sin(yaw) * FallDistanceAheadMeters, 0.0f };
            const Vec3 start = m_laboratoryCameraPosition + ahead
                + Vec3 { 0.0f, 0.0f, FallHeightAboveEyeMeters };
            if (!spawnPropAt(static_cast<std::size_t>(
                        std::distance(definitions.begin(), crate)),
                    start, Quaternion {}, false)) {
                Log::error("Smoke movimento: nao foi possivel soltar a caixa.");
                smoke.finished = true;
                m_automaticQuitSeconds = 0.001f;
                return;
            }
            smoke.probeEntityId = m_spawnedProps.back().entityId;
            smoke.probePlaced = true;
        }
        const auto prop = std::find_if(m_spawnedProps.begin(),
            m_spawnedProps.end(), [&](const SpawnedPropInstance& instance) {
                return instance.entityId == smoke.probeEntityId;
            });
        if (prop == m_spawnedProps.end()) return;
        // A pose DESENHADA, que e o que o jogador ve.
        probe = prop->renderedState.position;
        moving = !prop->physicsState.sleeping
            || prop->physicsState.linearVelocity.length() > 0.05f;
    }

    float screenX = 0.0f;
    float screenY = 0.0f;
    if (!projectToPixels(m_laboratoryViewProjection, probe, viewportWidth,
            viewportHeight, screenX, screenY)) {
        smoke.hasPreviousSample = false;
        smoke.hasPreviousVelocity = false;
        return;
    }
    if (smoke.hasPreviousSample) {
        const float dt = std::chrono::duration<float>(
            now - smoke.previousSampleTime).count();
        if (dt > 0.0001f) {
            if (smoke.hasPreviousVelocity) {
                const float errorX = screenX
                    - (smoke.previousScreenX + smoke.previousVelocityX * dt);
                const float errorY = screenY
                    - (smoke.previousScreenY + smoke.previousVelocityY * dt);
                const float error = std::sqrt(errorX * errorX + errorY * errorY);
                (moving ? smoke.movingErrors : smoke.restingErrors)
                    .push_back(error);
            }
            smoke.previousVelocityX = (screenX - smoke.previousScreenX) / dt;
            smoke.previousVelocityY = (screenY - smoke.previousScreenY) / dt;
            smoke.hasPreviousVelocity = true;
        }
    }
    smoke.previousScreenX = screenX;
    smoke.previousScreenY = screenY;
    smoke.previousSampleTime = now;
    smoke.hasPreviousSample = true;

    if (elapsed >= WarmupSeconds + MeasureSeconds) {
        smoke.finished = true;
        // Entrega ao caminho de saida comum dos smokes, que registra o
        // desempenho e chama reportRenderMotionSmoke.
        m_automaticQuitSeconds = 0.001f;
    }
}

void WorkbenchApp::advanceRenderMotionSmoke() {
    RenderMotionSmoke& smoke = m_renderMotionSmoke;
    if (smoke.mode != RenderMotionSmoke::Mode::CameraOrbit || !smoke.started
        || smoke.finished) {
        return;
    }
    // Chamado no FIM do render do quadro: o yaw muda entre este render e os
    // passos fixos do proximo quadro - a mesma posicao em que um evento de
    // mouse real entra no laco (antes dos passos fixos). Girar dentro do
    // render faria o erro aparecer em todo quadro e mediria outra coisa.
    const auto now = std::chrono::steady_clock::now();
    if (smoke.hasAdvanced) {
        const float dt = std::chrono::duration<float>(
            now - smoke.previousAdvanceTime).count();
        m_laboratoryCameraYaw += OrbitRadiansPerSecond * std::min(dt, 0.1f);
    }
    smoke.previousAdvanceTime = now;
    smoke.hasAdvanced = true;
}

void WorkbenchApp::reportRenderMotionSmoke() const {
    const RenderMotionSmoke& smoke = m_renderMotionSmoke;
    switch (smoke.mode) {
    case RenderMotionSmoke::Mode::Disabled:
        return;
    case RenderMotionSmoke::Mode::CameraOrbit:
        summarize("orbita, ponto parado", smoke.movingErrors);
        return;
    case RenderMotionSmoke::Mode::PropMotion:
        summarize("caixa em movimento", smoke.movingErrors);
        // Um prop dormindo tem de ficar imovel na tela: interpolar entre
        // duas poses velhas o faria tremer parado.
        summarize("caixa em repouso", smoke.restingErrors);
        return;
    }
}

} // namespace MatterEngine::Workbench
