#include "Workbench/WorkbenchApp.hpp"

#include "Workbench/Laboratory/StaticFootworkTerrainProbe.hpp"
#include "Workbench/UiKit.hpp"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace MatterEngine::Workbench {
namespace {

constexpr float Pi = 3.14159265358979323846f;

bool projectFootworkPoint(const Mat4& viewProjection, Vec3 point,
    ImVec2 displaySize, ImVec2& screenPoint) {
    const float clipX = viewProjection.at(0, 0) * point.x
        + viewProjection.at(0, 1) * point.y
        + viewProjection.at(0, 2) * point.z
        + viewProjection.at(0, 3);
    const float clipY = viewProjection.at(1, 0) * point.x
        + viewProjection.at(1, 1) * point.y
        + viewProjection.at(1, 2) * point.z
        + viewProjection.at(1, 3);
    const float clipW = viewProjection.at(3, 0) * point.x
        + viewProjection.at(3, 1) * point.y
        + viewProjection.at(3, 2) * point.z
        + viewProjection.at(3, 3);
    if (clipW <= 0.001f) return false;
    const float inverseW = 1.0f / clipW;
    screenPoint = {
        (clipX * inverseW * 0.5f + 0.5f) * displaySize.x,
        (0.5f - clipY * inverseW * 0.5f) * displaySize.y
    };
    return true;
}

const char* phaseName(FootPhase3D phase) {
    switch (phase) {
    case FootPhase3D::Planted: return "Plantado";
    case FootPhase3D::LiftOff: return "Saída";
    case FootPhase3D::Swing: return "Balanço";
    case FootPhase3D::TouchDown: return "Contato";
    case FootPhase3D::Aborting: return "Abortando";
    }
    return "?";
}

} // namespace

void WorkbenchApp::setFootworkTestEnabled(bool enabled) {
    if (enabled == m_footworkTestEnabled) {
        m_showFootworkPanel = enabled;
        return;
    }
    if (!enabled) {
        if (m_physicsScene && m_controlledFootworkEntityId != 0) {
            const auto controlled = std::find_if(
                m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
                [&](const SpawnedRagdollInstance& instance) {
                    return instance.entityId
                        == m_controlledFootworkEntityId;
                });
            if (controlled != m_spawnedRagdolls.end()) {
                if (m_physGunGrabbedEntityId == controlled->entityId) {
                    endPhysGunGrab();
                }
                m_physicsScene->destroyRagdoll(
                    controlled->physicsRagdoll);
                m_spawnedRagdolls.erase(controlled);
            }
        }
        m_controlledFootworkEntityId = 0;
        m_footworkTestEnabled = false;
        m_showFootworkPanel = false;
        m_footworkPlannerPaused = false;
        m_footworkAdvanceOneTick = false;
        m_footworkAutomaticForwardInput = false;
        m_laboratoryTaaHistoryValid = false;
        m_laboratoryCameraPitch = m_footworkSavedCameraPitch;
        m_laboratoryStatus.clear();
        return;
    }
    if (!m_physicsScene || !m_laboratoryMapLoaded
        || !m_physicsScene->hasCharacter()) {
        m_notification = {
            "Footwork aguarda o mapa e a fisica", 2.2f, 2.2f
        };
        return;
    }
    endPhysGunGrab();
    m_spawnMenuOpen = false;
    m_footworkSavedCameraPitch = m_laboratoryCameraPitch;
    m_laboratoryCameraPitch = -0.39f;
    m_footworkTestEnabled = true;
    m_showFootworkPanel = true;
    m_footworkPlannerPaused = false;
    m_footworkAdvanceOneTick = false;
    m_footworkAutomaticForwardInput = false;
    m_laboratoryTaaHistoryValid = false;
    m_footworkLastInput = {};
    resetFootworkTest();
    if (m_controlledFootworkEntityId == 0) {
        m_footworkTestEnabled = false;
        m_showFootworkPanel = false;
        m_laboratoryCameraPitch = m_footworkSavedCameraPitch;
    }
}

void WorkbenchApp::resetFootworkTest() {
    if (!m_physicsScene || !m_physicsScene->hasCharacter()
        || !m_footworkRagdollProfile) {
        return;
    }
    if (m_controlledFootworkEntityId != 0) {
        const auto previous = std::find_if(
            m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
            [&](const SpawnedRagdollInstance& instance) {
                return instance.entityId == m_controlledFootworkEntityId;
            });
        if (previous != m_spawnedRagdolls.end()) {
            m_physicsScene->destroyRagdoll(previous->physicsRagdoll);
            m_spawnedRagdolls.erase(previous);
        }
        m_controlledFootworkEntityId = 0;
    }
    const PhysicsCharacterState3D& character =
        m_physicsScene->characterState();
    const float bodyHeight = character.crouched
        ? m_characterSettings.crouchedHeight
        : m_characterSettings.standingHeight;
    const Vec3 feetPosition = character.position
        - Vec3 { 0.0f, 0.0f, bodyHeight * 0.5f };
    const Vec3 forward {
        std::cos(m_laboratoryCameraYaw),
        std::sin(m_laboratoryCameraYaw), 0.0f
    };
    StaticFootworkTerrainProbe terrain(*m_physicsScene);
    FootworkTerrainHit3D ground;
    const Vec3 requestedGround = feetPosition + forward * 1.90f;
    if (!terrain.raycastGround(
            requestedGround, 1.2f, 2.2f, ground)) {
        m_laboratoryStatus = "Footwork: terreno inicial invalido";
        m_notification = {
            "Mire para uma area plana e tente reposicionar", 2.5f, 2.5f
        };
        return;
    }
    const Quaternion orientation = Quaternion::fromAxisAngle(
        { 0.0f, 0.0f, 1.0f }, m_laboratoryCameraYaw);
    if (!spawnFootworkRagdollAt(
            ground.position + Vec3 { 0.0f, 0.0f,
                m_footworkRagdollProfile->standingRootHeightMeters
                    + 0.025f },
            orientation, true)) {
        m_laboratoryStatus = "Footwork: falha ao criar meio-esqueleto";
        return;
    }
    m_laboratoryStatus =
        "FOOTWORK FISICO: WASD move, Shift corre";
}

void WorkbenchApp::updateFootworkTest(float deltaTime) {
    static_cast<void>(deltaTime);
    if (!m_footworkTestEnabled || !m_physicsScene) {
        return;
    }
    const auto controlled = std::find_if(
        m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
        [&](const SpawnedRagdollInstance& instance) {
            return instance.entityId == m_controlledFootworkEntityId;
        });
    if (controlled == m_spawnedRagdolls.end()) return;
    const bool acceptsInput =
        !m_laboratoryDebugVisible && !m_spawnMenuOpen;
    if (acceptsInput) {
        FootworkInput3D input;
        if (m_footworkAutomaticForwardInput
            || this->input().keyDown(Key::W)) {
            input.movementLocal.y += 1.0f;
        }
        if (this->input().keyDown(Key::S)) input.movementLocal.y -= 1.0f;
        if (this->input().keyDown(Key::D)) input.movementLocal.x += 1.0f;
        if (this->input().keyDown(Key::A)) input.movementLocal.x -= 1.0f;
        input.lookYawRadians = m_laboratoryCameraYaw;
        input.fast = this->input().keyDown(Key::LeftShift)
            || this->input().keyDown(Key::RightShift);
        m_footworkLastInput = input;
    } else {
        // Abrir o painel pausa implicitamente a simulação. Assim inspecionar
        // um quadro não é interpretado como "soltei WASD" e não aborta a
        // passada que estava em andamento.
        m_footworkLastInput.lookYawRadians = m_laboratoryCameraYaw;
    }

    const bool advance = m_footworkAdvanceOneTick;
    const bool uiInspectionPause = m_laboratoryDebugVisible && !advance;
    controlled->footworkController.setInput(m_footworkLastInput);
    controlled->footworkController.setPaused(
        m_footworkPlannerPaused || uiInspectionPause);
    if (advance) {
        controlled->footworkController.setPaused(false);
        m_footworkAdvanceOneTick = false;
    }

    const Vec3 viewForward {
        std::cos(m_laboratoryCameraYaw),
        std::sin(m_laboratoryCameraYaw), 0.0f
    };
    if (!controlled->physicsState.links.empty()) {
        m_laboratoryCameraPosition =
            controlled->physicsState.links.front().position
            - viewForward * 2.75f + Vec3 { 0.0f, 0.0f, 0.72f };
    }
}

void WorkbenchApp::drawFootworkTestPanel() {
    if (!m_showFootworkPanel) return;
    const auto controlled = std::find_if(
        m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
        [&](const SpawnedRagdollInstance& instance) {
            return instance.entityId == m_controlledFootworkEntityId;
        });
    if (controlled == m_spawnedRagdolls.end()) return;
    PhysicalFootworkController3D& physical =
        controlled->footworkController;
    const ImGuiIO& io = ImGui::GetIO();
    const float panelWidth = ui(420.0f);
    ImGui::SetNextWindowPos(
        { ui(12.0f), headerHeight(m_uiScale) + ui(48.0f) },
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(
        { panelWidth, ui(650.0f) }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        { ui(340.0f), ui(360.0f) },
        { std::min(ui(680.0f), io.DisplaySize.x - ui(24.0f)),
            io.DisplaySize.y - headerHeight(m_uiScale) - ui(24.0f) });
    bool open = m_showFootworkPanel;
    if (ImGui::Begin("Footwork — laboratório isolado", &open,
            ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored({ 0.35f, 0.72f, 1.0f, 1.0f },
            "MEIO-ESQUELETO FISICO / 120 Hz");
        ImGui::TextWrapped(
            "WASD caminha; segure Shift para correr. A direção da câmera "
            "orienta a bacia. Passadas viram motores nas juntas e forças "
            "reais; nenhum osso recebe teleporte.");
        ImGui::Separator();

        const float gap = ui(7.0f);
        const float width =
            (ImGui::GetContentRegionAvail().x - gap * 2.0f) / 3.0f;
        if (ImGui::Button("REPOSICIONAR", { width, ui(36.0f) })) {
            resetFootworkTest();
        }
        ImGui::SameLine(0.0f, gap);
        if (ImGui::Button(
                m_footworkPlannerPaused ? "RETOMAR" : "PAUSAR",
                { width, ui(36.0f) })) {
            m_footworkPlannerPaused = !m_footworkPlannerPaused;
        }
        ImGui::SameLine(0.0f, gap);
        ImGui::BeginDisabled(!m_footworkPlannerPaused);
        if (ImGui::Button("1 TICK", { width, ui(36.0f) })) {
            m_footworkAdvanceOneTick = true;
        }
        ImGui::EndDisabled();

        FootworkConfig3D& config = physical.planner().config();
        ImGui::Dummy({ 1.0f, ui(8.0f) });
        ImGui::TextColored({ 0.55f, 0.68f, 0.78f, 1.0f },
            "DINÂMICA DA PASSADA");
        ImGui::SliderFloat("Velocidade (m/s)",
            &config.walkSpeedMetersPerSecond, 0.20f, 2.40f, "%.2f");
        ImGui::SliderFloat("Velocidade rápida (m/s)",
            &config.fastSpeedMetersPerSecond, 0.30f, 3.60f, "%.2f");
        ImGui::SliderFloat("Distância de disparo (m)",
            &config.stepTriggerDistanceMeters, 0.07f, 0.42f, "%.3f");
        ImGui::SliderFloat("Duração base (s)",
            &config.swingDurationSeconds, 0.18f, 0.72f, "%.3f");
        ImGui::SliderFloat("Altura do arco (m)",
            &config.swingClearanceMeters, 0.015f, 0.32f, "%.3f");
        ImGui::SliderFloat("Cadência da corrida (s)",
            &config.fastSwingDurationSeconds, 0.14f, 0.38f, "%.3f");
        ImGui::SliderFloat("Altura do passo correndo (m)",
            &config.fastSwingClearanceMeters, 0.05f, 0.28f, "%.3f");
        ImGui::SliderFloat("Largura da base (m)",
            &config.stanceWidthMeters, 0.13f, 0.52f, "%.3f");
        ImGui::SliderFloat("Alcance máximo (m)",
            &config.maximumStepReachMeters, 0.30f, 1.10f, "%.3f");
        float turnDegrees =
            config.stepTriggerYawRadians * 180.0f / Pi;
        if (ImGui::SliderFloat("Gatilho de giro",
                &turnDegrees, 4.0f, 38.0f, "%.1f°")) {
            config.stepTriggerYawRadians = turnDegrees * Pi / 180.0f;
        }

        ImGui::Dummy({ 1.0f, ui(8.0f) });
        ImGui::TextColored({ 0.55f, 0.68f, 0.78f, 1.0f },
            "ASSISTENCIA DA BACIA");
        PhysicalFootworkConfig3D& physicalConfig = physical.config();
        ImGui::SliderFloat("Sustentação da gravidade",
            &physicalConfig.gravitySupportFraction,
            0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Rigidez vertical",
            &physicalConfig.verticalStiffnessNewtonsPerMeter,
            250.0f, 2600.0f, "%.0f N/m");
        ImGui::SliderFloat("Controle horizontal",
            &physicalConfig.horizontalStiffnessNewtonsPerMeter,
            100.0f, 1600.0f, "%.0f N/m");
        ImGui::SliderFloat("Verticalidade",
            &physicalConfig.uprightStiffnessNewtonMeters,
            80.0f, 1200.0f, "%.0f Nm");

        ImGui::Dummy({ 1.0f, ui(8.0f) });
        ImGui::TextColored({ 0.55f, 0.68f, 0.78f, 1.0f },
            "DEPURAÇÃO");
        ImGui::Checkbox("Mostrar alvos planejados",
            &m_footworkShowHomes);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Contornos azuis: posição ideal de repouso de cada pé.\n"
                "Contorno laranja: contato escolhido para a passada ativa.");
        }
        ImGui::Checkbox("Trajetória da passada",
            &m_footworkShowTrajectory);
        ImGui::Checkbox("Polígono de suporte",
            &m_footworkShowSupportPolygon);
        ImGui::Checkbox("Métricas na tela",
            &m_footworkShowMetrics);

        const FootworkDebugState3D& state = physical.planner().state();
        const PhysicalFootworkTelemetry3D& physicalState =
            physical.telemetry();
        ImGui::Dummy({ 1.0f, ui(8.0f) });
        ImGui::TextColored({ 0.55f, 0.68f, 0.78f, 1.0f },
            "ESTADO");
        ImGui::Text("Passos concluídos: %llu",
            static_cast<unsigned long long>(state.completedStepCount));
        ImGui::Text("Raiz: %.2f m/s",
            state.rootVelocity.length());
        ImGui::Text("Esquerdo: %s  erro %.3f m",
            phaseName(state.feet[0].phase),
            state.feet[0].homeErrorMeters);
        ImGui::Text("Direito:  %s  erro %.3f m",
            phaseName(state.feet[1].phase),
            state.feet[1].homeErrorMeters);
        ImGui::Text("Apoio físico: %u pé(s), autoridade %.0f%%",
            physicalState.supportedFootCount,
            physicalState.supportConfidence * 100.0f);
        ImGui::Text("Bacia: força %.0f N | torque %.0f Nm",
            physicalState.rootForceNewtons,
            physicalState.rootTorqueNewtonMeters);
        if (physicalState.fallen) {
            ImGui::TextColored({ 1.0f, 0.52f, 0.24f, 1.0f },
                "QUEDA: assistência reduzida para não levitar");
        }
        if (state.terrainBlocked) {
            ImGui::TextColored({ 1.0f, 0.52f, 0.24f, 1.0f },
                "Terreno rejeitado pelo planejador");
        }

        ImGui::Dummy({ 1.0f, ui(10.0f) });
        if (ImGui::Button("SAIR DO MODO FOOTWORK",
                { -1.0f, ui(38.0f) })) {
            open = false;
        }
    }
    ImGui::End();
    if (!open) setFootworkTestEnabled(false);
}

void WorkbenchApp::drawFootworkTestOverlay() {
    if (!m_footworkTestEnabled) return;
    const auto controlled = std::find_if(
        m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
        [&](const SpawnedRagdollInstance& instance) {
            return instance.entityId == m_controlledFootworkEntityId;
        });
    if (controlled == m_spawnedRagdolls.end()) return;
    const PhysicalFootworkController3D& physical =
        controlled->footworkController;
    if (!physical.planner().state().initialized) return;
    const FootworkDebugState3D& state = physical.planner().state();
    const FootworkConfig3D& config = physical.planner().config();
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* draw = ImGui::GetForegroundDrawList();
    const auto drawWorldLine = [&](Vec3 a, Vec3 b,
            ImU32 color, float thickness) {
        ImVec2 screenA;
        ImVec2 screenB;
        if (projectFootworkPoint(m_laboratoryViewProjection,
                a, io.DisplaySize, screenA)
            && projectFootworkPoint(m_laboratoryViewProjection,
                b, io.DisplaySize, screenB)) {
            draw->AddLine(screenA, screenB, color, thickness);
        }
    };
    const auto drawFootOutline = [&](const FootPose3D& pose,
            ImU32 color) {
        const Vec3 forward =
            pose.orientation.rotate({ 1.0f, 0.0f, 0.0f });
        const Vec3 left =
            pose.orientation.rotate({ 0.0f, 1.0f, 0.0f });
        const float halfLength = config.footLengthMeters * 0.5f;
        const float halfWidth = config.footWidthMeters * 0.5f;
        const std::array<Vec3, 4> corners {
            pose.position + forward * halfLength + left * halfWidth,
            pose.position + forward * halfLength - left * halfWidth,
            pose.position - forward * halfLength - left * halfWidth,
            pose.position - forward * halfLength + left * halfWidth
        };
        for (std::size_t index = 0; index < corners.size(); ++index) {
            drawWorldLine(corners[index],
                corners[(index + 1) % corners.size()],
                color, ui(1.7f));
        }
    };

    if (m_footworkShowSupportPolygon
        && state.supportPolygonCount >= 2) {
        for (std::size_t index = 0;
                index < state.supportPolygonCount; ++index) {
            drawWorldLine(state.supportPolygon[index]
                    + Vec3 { 0.0f, 0.0f, 0.012f },
                state.supportPolygon[
                    (index + 1) % state.supportPolygonCount]
                    + Vec3 { 0.0f, 0.0f, 0.012f },
                IM_COL32(35, 229, 190, 220), ui(2.0f));
        }
    }
    if (m_footworkShowHomes) {
        drawFootOutline(state.feet[0].home,
            IM_COL32(66, 171, 255, 215));
        drawFootOutline(state.feet[1].home,
            IM_COL32(66, 171, 255, 215));
        if (state.hasActiveSwing) {
            drawFootOutline(state.feet[
                    footIndex3D(state.activeSwingSide)].target,
                IM_COL32(255, 173, 48, 235));
        }
    }
    if (m_footworkShowTrajectory
        && state.swingTrajectoryCount >= 2) {
        for (std::size_t index = 1;
                index < state.swingTrajectoryCount; ++index) {
            drawWorldLine(state.swingTrajectory[index - 1],
                state.swingTrajectory[index],
                IM_COL32(255, 198, 64, 230), ui(2.2f));
        }
    }

    ImVec2 rootScreen;
    if (projectFootworkPoint(m_laboratoryViewProjection,
            state.rootGroundPosition + Vec3 { 0.0f, 0.0f, 0.035f },
            io.DisplaySize, rootScreen)) {
        draw->AddCircle(rootScreen, ui(7.0f),
            IM_COL32(240, 247, 255, 235), 16, ui(1.5f));
        draw->AddLine({ rootScreen.x - ui(9.0f), rootScreen.y },
            { rootScreen.x + ui(9.0f), rootScreen.y },
            IM_COL32(240, 247, 255, 210), ui(1.3f));
        draw->AddLine({ rootScreen.x, rootScreen.y - ui(9.0f) },
            { rootScreen.x, rootScreen.y + ui(9.0f) },
            IM_COL32(240, 247, 255, 210), ui(1.3f));
    }

    if (m_footworkShowMetrics) {
        char text[256] {};
        const PhysicalFootworkTelemetry3D& physics =
            physical.telemetry();
        std::snprintf(text, sizeof(text),
            "FOOTWORK FISICO | %.2f m/s | %llu passos\n"
            "L %s %.2f m  R %s %.2f m | apoio %.0f%%",
            state.rootVelocity.length(),
            static_cast<unsigned long long>(state.completedStepCount),
            phaseName(state.feet[0].phase),
            state.feet[0].homeErrorMeters,
            phaseName(state.feet[1].phase),
            state.feet[1].homeErrorMeters,
            physics.supportConfidence * 100.0f);
        const ImVec2 position {
            ui(18.0f), headerHeight(m_uiScale) + ui(14.0f)
        };
        const ImVec2 size = ImGui::CalcTextSize(text);
        draw->AddRectFilled(
            { position.x - ui(9.0f), position.y - ui(7.0f) },
            { position.x + size.x + ui(9.0f),
                position.y + size.y + ui(7.0f) },
            IM_COL32(4, 14, 22, 196), ui(5.0f));
        draw->AddText(position, IM_COL32(193, 226, 246, 245), text);
    }
}

} // namespace MatterEngine::Workbench
