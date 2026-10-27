#include "Engine/Athlete/Planning/MotionPlanner3D.hpp"
#include "Workbench/WorkbenchApp.hpp"
#include "Workbench/UiKit.hpp"

#include "Engine/Core/Log.hpp"
#include "Engine/Environment/OceanSurface.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Math/Frustum3D.hpp"
#include "Engine/Math/JitterSequence.hpp"
#include "Engine/Math/ShadowCascade.hpp"
#include "Engine/RHI/RHITypes.hpp"
#include "Engine/Render/Scene3D.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Workbench {
namespace {

// Locomocao padrao (pedido do usuario, 05/10: as trocas de marcha estavam
// bruscas, sem deslocamento progressivo): o rumo da capsula gira rapido
// (rad/s) e a velocidade vai aos poucos (m/s2) - do parado ao trote em
// ~0,2 s, ao sprint em ~0,5 s, do sprint a zero em ~0,35 s; invertendo,
// freia antes.

constexpr float Pi = 3.14159265358979323846f;
constexpr float LaboratoryFarPlane = 2000.0f;

float smoothUnit(float edge0, float edge1, float value) {
    const float t = std::clamp((value - edge0) / (edge1 - edge0),
        0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

Vec3 mixColor(Vec3 a, Vec3 b, float amount) {
    return a * (1.0f - amount) + b * amount;
}

// Retained for ensureOceanClipmap()'s own geometry, even though the current
// dev-grid platform does not enable an ocean (no setOcean() call in
// ensureLaboratoryMapLoaded() below) — the sea level/extent constants that
// only that call used are gone with it.
constexpr float OceanNearExtentMeters = 160.0f;
constexpr float OceanNearCellMeters = 4.0f;
constexpr float OceanMidExtentMeters = 480.0f;
constexpr float OceanMidCellMeters = 16.0f;
constexpr float OceanFarExtentMeters = 1536.0f;
constexpr float OceanFarCellMeters = 64.0f;

// Órbita solar simples de latitude média: nasce a leste às 06h, culmina
// alto (sem ficar exatamente vertical) ao meio-dia e se põe a oeste às 18h.
// A Lua fica no lado oposto da mesma órbita nesta primeira iteração.
Vec3 solarDirectionForHour(float hour) {
    const float orbit = (hour - 6.0f) * (2.0f * Pi / 24.0f);
    return Vec3 {
        -std::cos(orbit) * 0.88f,
        -0.34f,
        std::sin(orbit)
    }.normalized();
}

// Aproxima uma nuvem atravessando especificamente o disco solar. A cobertura
// define a probabilidade/espessura global; ondas de escalas diferentes,
// transportadas pelo mesmo vento do céu, criam passagens graduais de sombra
// sem readback da GPU nem um segundo mapa de nuvens.
float cloudTransmittanceAtSun(float coverage, float animationTime,
    Vec2 windOffset, Vec3 sunDirection) {
    coverage = std::clamp(coverage, 0.0f, 1.0f);
    const float cloudSignal = std::clamp(
        0.50f
        + 0.22f * std::sin(sunDirection.x * 5.7f
            + windOffset.x * 2.1f + animationTime * 0.013f)
        + 0.16f * std::sin(sunDirection.y * 9.1f
            - windOffset.y * 1.7f + animationTime * 0.008f)
        + 0.09f * std::sin((sunDirection.x + sunDirection.y) * 17.0f
            + animationTime * 0.021f),
        0.0f, 1.0f);
    const float threshold = 0.82f - coverage * 0.55f;
    const float cloudOverSun = smoothUnit(
        threshold - 0.08f, threshold + 0.12f, cloudSignal);
    const float overcast = smoothUnit(0.72f, 0.98f, coverage);
    const float localTransmission = 1.0f
        - cloudOverSun * (0.30f + coverage * 0.58f);
    return std::clamp(localTransmission
        * (1.0f - overcast * 0.58f), 0.10f, 1.0f);
}

// Interpolacao de render entre os dois ultimos passos fixos. A fisica roda a
// 120 Hz e o quadro segue o monitor; desenhar o ultimo passo cru faz tudo que
// se move andar em degraus (cerca de um quadro em cada cinco a 143,8 Hz nao
// recebe passo nenhum).
Quaternion blendRotation(Quaternion from, Quaternion to, float amount) {
    if (from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w < 0.0f) {
        to = { -to.x, -to.y, -to.z, -to.w };
    }
    return Quaternion { from.x + (to.x - from.x) * amount,
        from.y + (to.y - from.y) * amount,
        from.z + (to.z - from.z) * amount,
        from.w + (to.w - from.w) * amount }.normalized();
}

PhysicsBodyState3D interpolatePose(const PhysicsBodyState3D& from,
    const PhysicsBodyState3D& to, float amount) {
    PhysicsBodyState3D pose = to;
    pose.position = from.position + (to.position - from.position) * amount;
    pose.orientation = blendRotation(from.orientation, to.orientation, amount);
    return pose;
}

Vec3 cameraForward(float yaw, float pitch) {
    const float planar = std::cos(pitch);
    return {
        planar * std::cos(yaw),
        planar * std::sin(yaw),
        std::sin(pitch)
    };
}

bool projectToScreen(const Mat4& viewProjection, Vec3 point,
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

// Abre um dos paineis flutuantes de depuracao (Physgun/Gráficos/Performance)
// com posicao/tamanho padrao e o mesmo teto de altura pra nenhum deles
// nascer maior que a tela - fatorado porque os 3 paineis precisam
// exatamente do mesmo comportamento, so o conteudo interno muda.
// `open` tambem vira o X de fechar da propria janela ImGui, entao fechar
// pelo X ou clicando o botao da barra de novo do exatamente no mesmo lugar.
bool beginDebugPanel(const char* name, bool* open, ImVec2 defaultPosition,
    ImVec2 defaultSize, ImVec2 displaySize, float uiScale) {
    ImGui::SetNextWindowPos(defaultPosition, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(defaultSize, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(
        { 300.0f * uiScale, 160.0f * uiScale },
        { std::min(760.0f * uiScale, displaySize.x - 32.0f * uiScale),
            displaySize.y - defaultPosition.y - 24.0f * uiScale });
    return ImGui::Begin(name, open, ImGuiWindowFlags_NoCollapse);
}

// Cresce a janela atual pra caber o conteudo que acabou de ser desenhado
// (mas nunca encolhe sozinha nem ultrapassa a tela) - so deve ser chamada
// enquanto o Begin correspondente retornou true (conteudo de verdade foi
// desenhado neste quadro).
void growPanelToFitContent(ImVec2 displaySize, float uiScale) {
    const float requiredHeight = ImGui::GetCursorPosY()
        + ImGui::GetStyle().WindowPadding.y;
    const ImVec2 currentSize = ImGui::GetWindowSize();
    const float maximumHeight = displaySize.y - ImGui::GetWindowPos().y
        - 12.0f * uiScale;
    if (requiredHeight > currentSize.y) {
        ImGui::SetWindowSize({ currentSize.x,
            std::min(requiredHeight, maximumHeight) });
    }
}

// A classic dev-grid checkerboard, generated in memory — no image asset.
// Baked at cellsPerSide x cellsPerSide so the sampler's own REPEAT
// addressing (the material sampler; see VulkanDevice's set=1 descriptor)
// tiles it across the platform, instead of needing one giant texture for a
// 500 m floor: the full mip chain createTexture2D already builds keeps each
// tile filtering correctly at any distance.
std::vector<std::byte> buildDevCheckerPixels(std::uint32_t textureSize,
    std::uint32_t cellsPerSide, Vec3 lightColor, Vec3 darkColor,
    Vec3 groutColor, float groutWidthTexels) {
    std::vector<std::byte> pixels(
        static_cast<std::size_t>(textureSize) * textureSize * 4);
    const float cellSize = static_cast<float>(textureSize)
        / static_cast<float>(cellsPerSide);
    const float halfGrout = groutWidthTexels * 0.5f;
    const auto channel = [](float value) {
        return static_cast<std::byte>(std::clamp(
            static_cast<int>(value * 255.0f + 0.5f), 0, 255));
    };
    for (std::uint32_t y = 0; y < textureSize; ++y) {
        for (std::uint32_t x = 0; x < textureSize; ++x) {
            const std::uint32_t cellX =
                static_cast<std::uint32_t>(static_cast<float>(x) / cellSize);
            const std::uint32_t cellY =
                static_cast<std::uint32_t>(static_cast<float>(y) / cellSize);
            // Distance (in texels) from this pixel to the nearest cell
            // boundary on each axis. The pattern repeats every cellSize
            // texels and the texture's own width is an exact multiple of
            // it, so this also draws the seam at the tile wrap-around
            // (x=textureSize meeting x=0 of the next REPEAT), with no gap
            // or double-width line at that edge.
            const float localX = std::fmod(static_cast<float>(x), cellSize);
            const float localY = std::fmod(static_cast<float>(y), cellSize);
            const float edgeDistanceX =
                std::min(localX, cellSize - localX);
            const float edgeDistanceY =
                std::min(localY, cellSize - localY);
            const bool onGrout = edgeDistanceX < halfGrout
                || edgeDistanceY < halfGrout;
            const Vec3& color = onGrout ? groutColor
                : ((cellX + cellY) % 2 == 0) ? lightColor : darkColor;
            const std::size_t index =
                (static_cast<std::size_t>(y) * textureSize + x) * 4;
            pixels[index + 0] = channel(color.x);
            pixels[index + 1] = channel(color.y);
            pixels[index + 2] = channel(color.z);
            pixels[index + 3] = std::byte { 255 };
        }
    }
    return pixels;
}

// One rectangular face of a dev-grid box, given in the box's own local
// space (center at the origin) plus the world transform to place it —
// UV comes from the LOCAL corner so a rotated box (the ramp) keeps square,
// undistorted checker cells instead of the pattern smearing along the
// slope. Winding does not matter: the scene pipeline runs with
// cullMode=NONE.
void appendDevBoxFace(MeshData3D& mesh, Vec3 center, Quaternion orientation,
    const std::array<Vec3, 4>& localCorners, Vec3 localNormal,
    std::size_t axisU, std::size_t axisV, float metersPerTextureTile) {
    const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
    const auto component = [](Vec3 value, std::size_t axis) {
        return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
    };
    const Vec3 worldNormal = orientation.rotate(localNormal);
    for (Vec3 localCorner : localCorners) {
        MeshVertex3D vertex;
        vertex.position = center + orientation.rotate(localCorner);
        vertex.normal = worldNormal;
        vertex.uv = {
            component(localCorner, axisU) / metersPerTextureTile,
            component(localCorner, axisV) / metersPerTextureTile
        };
        mesh.vertices.push_back(vertex);
    }
    mesh.indices.insert(mesh.indices.end(), {
        base, base + 1, base + 2, base, base + 2, base + 3
    });
}

// A real block, not a single quad: six faces with correct normals/UV, so
// every dev-grid prop — the floor, a wall, a ramp, a stair step — reads as
// an object with edges from any angle, not a cardboard cutout. center/
// orientation place it in world space; halfExtents are in the box's own
// local frame (so a rotated ramp's "length" stays halfExtents.x regardless
// of how it is tilted).
MeshData3D buildDevBoxMesh(Vec3 center, Vec3 halfExtents,
    Quaternion orientation, float metersPerTextureTile) {
    MeshData3D mesh;
    const float x0 = -halfExtents.x, x1 = halfExtents.x;
    const float y0 = -halfExtents.y, y1 = halfExtents.y;
    const float z0 = -halfExtents.z, z1 = halfExtents.z;
    appendDevBoxFace(mesh, center, orientation, { Vec3 { x0, y0, z1 },
        { x1, y0, z1 }, { x1, y1, z1 }, { x0, y1, z1 } },
        { 0.0f, 0.0f, 1.0f }, 0, 1, metersPerTextureTile);
    appendDevBoxFace(mesh, center, orientation, { Vec3 { x0, y0, z0 },
        { x0, y1, z0 }, { x1, y1, z0 }, { x1, y0, z0 } },
        { 0.0f, 0.0f, -1.0f }, 0, 1, metersPerTextureTile);
    appendDevBoxFace(mesh, center, orientation, { Vec3 { x1, y0, z0 },
        { x1, y1, z0 }, { x1, y1, z1 }, { x1, y0, z1 } },
        { 1.0f, 0.0f, 0.0f }, 1, 2, metersPerTextureTile);
    appendDevBoxFace(mesh, center, orientation, { Vec3 { x0, y0, z0 },
        { x0, y0, z1 }, { x0, y1, z1 }, { x0, y1, z0 } },
        { -1.0f, 0.0f, 0.0f }, 1, 2, metersPerTextureTile);
    appendDevBoxFace(mesh, center, orientation, { Vec3 { x0, y1, z0 },
        { x0, y1, z1 }, { x1, y1, z1 }, { x1, y1, z0 } },
        { 0.0f, 1.0f, 0.0f }, 0, 2, metersPerTextureTile);
    appendDevBoxFace(mesh, center, orientation, { Vec3 { x0, y0, z0 },
        { x1, y0, z0 }, { x1, y0, z1 }, { x0, y0, z1 } },
        { 0.0f, -1.0f, 0.0f }, 0, 2, metersPerTextureTile);
    recomputeBounds(mesh);
    return mesh;
}

} // namespace

void WorkbenchApp::enterLaboratory(bool resetCamera) {
    if (!m_laboratoryInitialized || resetCamera) {
        if (m_laboratoryMapLoaded) {
            resetLaboratoryCharacter();
        } else {
            m_laboratoryCameraPosition = { 0.0f, -90.0f, 4.6f };
            m_laboratoryCameraYaw = 1.57079632679f;
        }
        m_laboratoryCameraPitch = -0.08f;
        m_laboratoryInitialized = true;
    }

    m_screen = Screen::Laboratory;
    m_laboratoryPaused = false;
    m_laboratoryConfirmQuit = false;
    m_laboratoryDebugVisible = false;
    m_spawnMenuOpen = false;
    m_laboratoryJumpRequested = false;
    m_laboratoryFlightToggleRequested = false;
    m_laboratoryTaaHistoryValid = false;
    m_taaFrameIndex = 0;
    m_discardNextMouseDelta = true;
    // A entrada normal no laboratório já prepara o personagem padrão. O
    // spawn é executado quando o mapa e os assets terminarem de carregar;
    // assim o controle começa no primeiro ragdoll válido, sem exigir menu.
    m_takeCharacterControlWhenReady = m_controlledCharacterEntityId == 0;
    syncLaboratoryMouseCapture();
}

void WorkbenchApp::syncLaboratoryMouseCapture() {
    const bool capture = m_screen == Screen::Laboratory
        && m_laboratoryWindowFocused && !m_laboratoryPaused
        && !m_laboratoryDebugVisible && !m_spawnMenuOpen;
    renderer().setMouseCaptured(capture);
    if (capture) {
        m_discardNextMouseDelta = true;
    }
}

void WorkbenchApp::updateLaboratory(float deltaTime) {
    m_laboratoryElapsedTime += deltaTime;
    if (m_laboratoryClockRunning) {
        m_laboratoryEnvironment.solarTimeHours += deltaTime
            * m_laboratoryClockTimeScale / 3600.0f;
        m_laboratoryEnvironment.solarTimeHours = std::fmod(
            m_laboratoryEnvironment.solarTimeHours, 24.0f);
        if (m_laboratoryEnvironment.solarTimeHours < 0.0f) {
            m_laboratoryEnvironment.solarTimeHours += 24.0f;
        }
    }
    if (!m_laboratoryCharacterInitialized || !m_laboratoryMapLoaded
        || !m_physicsScene || !m_physicsScene->hasCharacter()) {
        return;
    }
    // O corpo pedido no painel (ao soltar): antes da capsula ser lida e dos
    // atletas andarem.
    if (m_bodyApplyRequested) applyLaboratoryBody();

    const PhysicsCharacterState3D stateBefore =
        m_physicsScene->characterState();

    bool controllingCharacter = m_controlledCharacterEntityId != 0;
    bool controlledCharacterFrozen = false;
    bool controlledCharacterDown = false;
    // Correndo (rapido o bastante para o carrinho), com o carrinho armado ou
    // em curso, o Ctrl e o carrinho: a capsula nao agacha (agachada ela
    // freava para 2,8 m/s e puxava o corpo para tras - ele reagia como num
    // tropeco antes do mergulho).
    bool controlRunningTackle = false;
    if (controllingCharacter) {
        const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
            m_spawnedRagdolls.end(), [&](const SpawnedRagdollInstance& item) {
                return item.entityId == m_controlledCharacterEntityId
                    && item.active;
            });
        if (controlled == m_spawnedRagdolls.end()) {
            releaseControlledCharacter();
            controllingCharacter = false;
        } else {
            controlledCharacterFrozen = controlled->frozen
                || controlled->physicsState.frozen;
            // One tick stale (this frame's locomotion update hasn't run
            // yet), which is fine at 120 Hz: the point is just to stop
            // feeding the capsule movement input while the ragdoll is down
            // or recovering, so it does not wander off and strand the
            // camera away from a character physics is puppeting on its own.
            const CharacterLocomotionState3D lastState =
                controlled->athlete.legacy.locomotion.telemetry().state;
            controlledCharacterDown =
                lastState == CharacterLocomotionState3D::Fallen
                || lastState == CharacterLocomotionState3D::GettingUp
                || lastState == CharacterLocomotionState3D::Tackling;
            const auto& telemetry = controlled->athlete.legacy.locomotion.telemetry();
            // (No ar o Ctrl e a voadora, tambem no fisico - 06/10.)
            controlRunningTackle = m_tackleEnabled
                && (telemetry.tacklePending || telemetry.tackling
                    || ((lastState == CharacterLocomotionState3D::Running
                            || lastState == CharacterLocomotionState3D::Walking)
                        && telemetry.speedMetersPerSecond > 2.0f)
                    || (!stateBefore.grounded && m_defaultAirTackle));
            // Locomocao padrao: o estado e o dela (o carrinho dela; correndo,
            // o Ctrl e o carrinho).
            if (m_controlledAnimated) {
                const bool tackling = controlled->athlete.legacy.defaultLocomotion.output().tackling;
                controlledCharacterDown = tackling;
                const Vec3 v = stateBefore.velocity;
                // (No ar o Ctrl e o carrinho aereo - a voadora - mesmo parado.)
                controlRunningTackle = m_tackleEnabled && (tackling || std::hypot(v.x, v.y) > 2.0f
                    || (!stateBefore.grounded && m_defaultAirTackle));
            }
        }
    }

    // Estresse da bola: o "jogador" automatico (rumo, sprint, olhar, bola
    // reposta perto e pedidos de toque e chute).
    if (m_ballStress && controllingCharacter && !m_spawnedRagdolls.empty()) {
        m_ballStressTime += deltaTime;
        const auto next = [&]() {
            m_ballStressState ^= m_ballStressState << 13;
            m_ballStressState ^= m_ballStressState >> 17;
            m_ballStressState ^= m_ballStressState << 5;
            return static_cast<float>(m_ballStressState % 10000u) / 10000.0f;
        };
        const auto controlled = std::find_if(m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
            [&](const SpawnedRagdollInstance& item) { return item.entityId == m_controlledCharacterEntityId; });
        if (m_ballStressTime >= m_ballStressSegmentEnd) {
            m_ballStressSegmentEnd = m_ballStressTime + 0.3f + 1.0f * next();
            const float yaw = next() * 6.2831853f;
            m_ballStressMove = next() < 0.2f ? Vec3 {} : Vec3 { std::cos(yaw), std::sin(yaw), 0.0f };
            m_ballStressSprint = next() < 0.3f;
            if (next() < 0.5f) m_laboratoryCameraYaw = next() * 6.2831853f;
            // (Pulos: o pouso passa ao modo fisico e ele volta ao padrao.
            // Ctrl curto correndo: o carrinho.)
            if (next() < 0.2f) m_laboratoryJumpRequested = true;
            else if (m_ballStressSprint && m_ballStressMove.lengthSquared() > 0.5f && next() < 0.3f) {
                // (O carrinho e para a frente do olhar: a camera no rumo, e o
                // Ctrl depois de embalar.)
                m_laboratoryCameraYaw = std::atan2(m_ballStressMove.y, m_ballStressMove.x);
                m_ballStressCrouchDelay = 0.6f;
            }
        }
        if (controlled != m_spawnedRagdolls.end() && !controlled->physicsState.links.empty()
            && m_ballStressTime >= m_ballStressNextPlace && !m_inflatedBalls.empty()) {
            m_ballStressNextPlace = m_ballStressTime + 0.8f + 1.2f * next();
            const Vec3 pelvis = controlled->physicsState.links.front().position;
            const float a = next() * 6.2831853f, d = 0.4f + 2.4f * next();
            const auto& ball = m_inflatedBalls.front();
            const float speed = next() < 0.4f ? 3.0f * next() : 0.0f;
            const float va = next() * 6.2831853f;
            const Vec3 velocity { speed * std::cos(va), speed * std::sin(va), 0.0f };
            // (No raio da bola do menu - o tamanho escolhido.)
            const float radius = m_soccerBallProfile.radiusMeters;
            const Vec3 spin = cross(Vec3 { 0.0f, 0.0f, 1.0f }, velocity) * (1.0f / radius);
            m_physicsScene->setInflatedBallMotion(ball.physicsBall,
                { pelvis.x + d * std::cos(a), pelvis.y + d * std::sin(a), ragdollGroundHeight(pelvis) + radius + 0.005f },
                velocity, spin);
        }
        if (m_ballStressTime >= m_ballStressNextRequest) {
            m_ballStressNextRequest = m_ballStressTime + 0.15f + 0.6f * next();
            const float roll = next();
            {
                m_ballRequest = roll < 0.55f ? BallActionKind3D::Shot : BallActionKind3D::Dribble;
                m_ballRequestPower = next();
                m_ballRequestAimYaw = m_laboratoryCameraYaw + (next() - 0.5f);
                m_ballRequestLift = next() * 2.0f - 1.0f;
                m_ballRequestCurve = next() < 0.3f ? next() * 2.0f - 1.0f : 0.0f;
            }
            ++m_ballStressRequests;
            if (m_ballStressRequests % 50 == 0)
                Log::info("estresse da bola: " + std::to_string(m_ballStressRequests) + " pedidos em "
                    + std::to_string(m_ballStressTime) + " s");
        }
    }

    CharacterMotorCommand3D command;
    // The capsule must never sweep against the controlled character's own
    // ragdoll avatar, in or out of a menu: leaving this at its default
    // false whenever a panel blocked new commands made the capsule collide
    // with its own overlapping body every tick a menu stayed open, and
    // PhysX's penetration recovery threw the character violently upward.
    command.ignoreRagdolls = controllingCharacter;
    const bool freeLook = controllingCharacter
        && (input().keyDown(Key::LeftAlt)
            || input().keyDown(Key::RightAlt));
    // Alt desacopla o olhar do corpo, mas nao suspende a locomocao. Enquanto
    // ele esta pressionado, WASD permanece no referencial do rumo que o
    // personagem ja tinha; assim orbitar a camera nao curva a trajetoria nem
    // zera o input no meio de uma passada.
    const bool cameraOnly = controlledCharacterFrozen || controlledCharacterDown;
    const float movementReferenceYaw = freeLook
        ? m_characterDesiredFacingYaw : m_laboratoryCameraYaw;
    const bool acceptsMovement = !m_laboratoryDebugVisible
        && !m_spawnMenuOpen && !cameraOnly;
    bool maxSprint = false;
    if (acceptsMovement) {
        const Vec3 planarForward = cameraForward(movementReferenceYaw, 0.0f);
        const Vec3 viewForward = cameraForward(movementReferenceYaw,
            m_laboratoryCameraPitch);
        // Em coordenadas Z-up, direita é forward × up. No voo, W/S usa
        // viewForward completo para acompanhar exatamente o ponto observado.
        const bool freeVerticalMovement =
            stateBefore.flying || stateBefore.swimming;
        const Vec3 forward = freeVerticalMovement
            ? viewForward : planarForward;
        const Vec3 right { planarForward.y, -planarForward.x, 0.0f };
        if (input().keyDown(Key::W)) command.moveDirection += forward;
        if (input().keyDown(Key::S)) command.moveDirection -= forward;
        if (input().keyDown(Key::D)) command.moveDirection += right;
        if (input().keyDown(Key::A)) command.moveDirection -= right;
        command.sprint = input().keyDown(Key::LeftShift)
            || input().keyDown(Key::RightShift);
        if (m_ballStress && controllingCharacter) {
            command.moveDirection = m_ballStressMove;
            command.sprint = m_ballStressSprint;
        }
        // O sprint maximo: armado pelo Shift duplo, so com o W (a frente e as diagonais da frente) e so no modo
        // padrao; soltar o Shift desarma.
        if (!command.sprint) m_maxSprintArmed = false;
        // Sair do cone (sem W, ou com S) com ele ja ativo o corta (25/10, pedido do usuario): voltando para a
        // frente, e a corrida normal ate o proximo Shift duplo.
        const bool forwardCone = input().keyDown(Key::W) && !input().keyDown(Key::S);
        if (m_maxSprintActive && !forwardCone) m_maxSprintArmed = false;
        maxSprint = controllingCharacter && m_controlledAnimated && command.sprint && m_maxSprintArmed && !m_ballStress
            && forwardCone;
        m_maxSprintActive = maxSprint;
        // O personagem olha para a câmera e anda em qualquer direção; para
        // trás ele recua, e recuar tem a velocidade do recuo, não a da
        // caminhada nem a do sprint (ver characterGaitSpeed3D).
        if (command.moveDirection.lengthSquared() <= 0.0001f) {
            m_v2RequestValid = false;
            m_v2RequestTurnRate = 0.0f;
        }
        if (controllingCharacter && !freeVerticalMovement
            && command.moveDirection.lengthSquared() > 0.0001f) {
            const float travelRelative = std::atan2(
                command.moveDirection.y, command.moveDirection.x)
                - movementReferenceYaw;
            const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
                m_spawnedRagdolls.end(),
                [&](const SpawnedRagdollInstance& item) {
                    return item.entityId == m_controlledCharacterEntityId;
                });
            // No modo padrao, a tabela dele (oito rumos; de costas na
            // diagonal perde menos).
            // (No fisico tambem, com a chave "Velocidades do padrao no fisico".)
            const bool defaultTable = (m_controlledAnimated || m_physicalDefaultSpeeds) && controlled != m_spawnedRagdolls.end();
            const float wanted = !defaultTable
                ? characterGaitSpeed3D(m_avatarGaitSpeeds, travelRelative, command.sprint)
                : maxSprint ? m_defaultMaxSprintSpeed
                : athleteDefaultTravelSpeed3D(athleteRuntimeSettings(), travelRelative, command.sprint);
            const float nominal = command.sprint
                ? m_avatarCharacterSettings.sprintSpeed
                : m_avatarCharacterSettings.walkSpeed;
            if (wanted > 0.0f && nominal > 0.0f) {
                command.speedScale = wanted / nominal;
            }
            if (m_controlledAnimated) command.speedScale *= m_controlledSpeedScale;
            // Escada, ladeira e curva fechada: o passo encurta e o corpo
            // desacelera junto (a locomocao diz quanto da velocidade cabe no
            // terreno a frente e na curva pedida).
            // (So no modo fisico: no padrao a locomocao fisica nao roda e o
            // limite ficava parado no ultimo valor - o freio da arrancada
            // dela deixava o jogador lento ate a proxima ida ao fisico.)
            if (controlled != m_spawnedRagdolls.end() && wanted > 0.0f && !m_controlledAnimated) {
                const float limit = controlled->athlete.legacy.locomotion.telemetry()
                    .terrainSpeedLimit;
                command.speedScale *= std::clamp(limit / wanted, 0.0f, 1.0f);
            }
            // Locomocao fisica v2: virando forte a velocidade cai como a de
            // quem corre cortando, e com pouca ajuda a capsula fica presa a
            // velocidade do corpo (no chao, so ganhando velocidade).
            {
                const Vec3 direction = Vec3 { command.moveDirection.x, command.moveDirection.y, 0.0f }.normalized();
                const float rate = m_v2RequestValid
                    ? std::acos(std::clamp(dot(m_v2RequestDirection, direction), -1.0f, 1.0f)) / std::max(deltaTime, 1e-4f)
                    : 0.0f;
                m_v2RequestTurnRate += (rate - m_v2RequestTurnRate) * (1.0f - std::exp(-deltaTime / 0.25f));
                m_v2RequestDirection = direction;
                m_v2RequestValid = true;
            }
            if (controlled != m_spawnedRagdolls.end() && wanted > 0.0f && !m_controlledAnimated
                && m_adaptivePhysics && m_physicalLocomotionV2 && stateBefore.grounded)
                command.speedScale *= physicalNavigationTurnScale3D(stateBefore.velocity, command.moveDirection,
                    m_v2RequestTurnRate, controlled->athlete.legacy.adaptivePhysics.settings().walkingAssistScale);
            if (controlled != m_spawnedRagdolls.end() && wanted > 0.0f && !m_controlledAnimated
                && m_adaptivePhysics && m_physicalLocomotionV2 && stateBefore.grounded) {
                command.speedScale = physicalNavigationSpeedScale3D(command.moveDirection, command.speedScale,
                    nominal, controlled->athlete.legacy.locomotion.physicalState().centerOfMassVelocityWorld,
                    physicalNavigationLead3D(controlled->athlete.legacy.adaptivePhysics.settings().walkingAssistScale));
            }
        }
        const bool control = input().keyDown(Key::LeftControl)
            || input().keyDown(Key::RightControl) || m_ballStressCrouchSeconds > 0.0f;
        if (freeVerticalMovement) {
            if (input().keyDown(Key::Space)) command.moveDirection.z += 1.0f;
            command.crouch = stateBefore.flying && control;
            if (stateBefore.swimming && control) {
                command.moveDirection.z -= 1.0f;
            }
            m_laboratoryJumpCharging = false;
            // (Voando ou nadando, o espaco sobe: o pulo instantaneo nao sai.)
            m_laboratoryJumpRequested = false;
        } else {
            command.crouch = control && !controlRunningTackle;
        }
        // Pulo ao soltar o espaco: o tempo segurado decide a altura (um
        // toque e o pulo baixo; ver characterJumpSpeed3D), parado ou em
        // movimento (pedido do usuario, 05/10 noite). O agachar de preparo
        // so parado (04/10: agachado correndo, as pernas ficavam travadas).
        if (m_laboratoryJumpCharging) {
            if (input().keyDown(Key::Space)) {
                m_laboratoryJumpChargeSeconds += deltaTime;
            } else {
                m_laboratoryJumpCharging = false;
                m_laboratoryJumpRequested = true;
            }
        }
        // O pulo instantaneo: sai ja, na altura media.
        if (m_instantJumpPressed) {
            m_instantJumpPressed = false;
            m_laboratoryJumpChargeSeconds = characterJumpHoldForScale3D(m_avatarCharacterSettings, 1.0f);
            m_laboratoryJumpRequested = true;
        }
        command.jumpPressed = m_laboratoryJumpRequested;
        command.jumpChargeSeconds = m_laboratoryJumpChargeSeconds;
        // Carga do manejo de bola: o chute enche em ~1 s, o drible em ~0,6 s (a
        // aba "Força na bola" muda os dois).
        if (m_ballShotHeld || m_ballDribbleHeld || m_ballRaiseHeld)
            m_ballCharge = std::min(1.0f, m_ballCharge
                + deltaTime / std::max(0.05f, m_ballShotHeld ? m_ballShotChargeSeconds
                    : m_ballRaiseHeld ? m_ballRaiseChargeSeconds : m_ballTouchChargeSeconds));
        // O gesto do chute esquece o puxao antigo (~0,15 s): vale o de logo
        // antes de soltar.
        m_ballFlick = m_ballShotHeld ? m_ballFlick * std::exp(-deltaTime / 0.15f) : Vec2 {};
        if (m_ballShotHeld) {
            m_shotLiftShown = ballFlickShape(-m_ballFlick.y * m_shotLiftSensitivity);
            m_shotCurveShown = ballFlickShape(m_ballFlick.x * m_shotCurveSensitivity);
        }
        // A camera mais lenta carregando o chute: entra em ~0,15 s e sai em
        // ~0,25 s depois de soltar.
        m_shotCameraSlow = std::clamp(m_shotCameraSlow + (m_ballShotHeld ? deltaTime / 0.15f : -deltaTime / 0.25f),
            0.0f, 1.0f);
        // E no sprint maximo: entra em ~0,4 s (com a arrancada) e sai em ~0,3 s.
        m_maxSprintCameraSlow = std::clamp(m_maxSprintCameraSlow + (maxSprint ? deltaTime / 0.4f : -deltaTime / 0.3f),
            0.0f, 1.0f);
        command.toggleFlight = !controllingCharacter
            && m_laboratoryFlightToggleRequested;
    } else {
        // Um painel aberto bloqueia novos comandos, mas não congela gravidade
        // nem força o personagem agachado a tentar levantar.
        command.crouch = stateBefore.crouched;
        m_laboratoryJumpCharging = false;
    }
    m_laboratoryJumpRequested = false;
    m_laboratoryFlightToggleRequested = false;
    // Intencao do jogador, pura: coletada tambem caido ou levantando (so
    // menus, o olhar livre e o congelado a bloqueiam). O corpo pode nao
    // conseguir executar, mas o pedido continua existindo - o proximo passo,
    // a protecao e o levantar vao usa-lo. A capsula segue parada nesses
    // estados (acceptsMovement acima).
    m_characterIntent = {};
    m_characterSpaceHeld = false;
    if (controllingCharacter) {
        m_characterIntent.requestedFacingYawRadians = freeLook
            ? m_characterDesiredFacingYaw : m_laboratoryCameraYaw;
        m_characterIntent.lookPitchRadians = m_laboratoryCameraPitch;
        const bool intentBlocked = m_laboratoryDebugVisible || m_spawnMenuOpen
            || controlledCharacterFrozen;
        if (!intentBlocked && !stateBefore.flying && !stateBefore.swimming) {
            const Vec3 planarForward = cameraForward(movementReferenceYaw, 0.0f);
            const Vec3 right { planarForward.y, -planarForward.x, 0.0f };
            Vec3 direction {};
            if (input().keyDown(Key::W)) direction += planarForward;
            if (input().keyDown(Key::S)) direction -= planarForward;
            if (input().keyDown(Key::D)) direction += right;
            if (input().keyDown(Key::A)) direction -= right;
            bool sprint = input().keyDown(Key::LeftShift)
                || input().keyDown(Key::RightShift);
            if (m_ballStress) {
                direction = m_ballStressMove;
                sprint = m_ballStressSprint;
            }
            m_characterIntent.effort = sprint ? 1.0f : 0.0f;
            m_characterSpaceHeld = input().keyDown(Key::Space);
            m_characterIntent.crouch = input().keyDown(Key::LeftControl)
                || input().keyDown(Key::RightControl) || m_ballStressCrouchSeconds > 0.0f;
            m_ballStressCrouchSeconds = std::max(0.0f, m_ballStressCrouchSeconds - deltaTime);
            if (m_ballStressCrouchDelay > 0.0f) {
                m_ballStressCrouchDelay -= deltaTime;
                if (m_ballStressCrouchDelay <= 0.0f) m_ballStressCrouchSeconds = 0.1f;
            }
            direction.z = 0.0f;
            if (direction.lengthSquared() > 0.0001f) {
                direction = direction.normalized();
                const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
                    m_spawnedRagdolls.end(), [&](const SpawnedRagdollInstance& item) {
                        return item.entityId == m_controlledCharacterEntityId;
                    });
                const float relative = std::atan2(direction.y, direction.x) - movementReferenceYaw;
                const bool defaultTable = (m_controlledAnimated || m_physicalDefaultSpeeds) && controlled != m_spawnedRagdolls.end();
                float speed = !defaultTable ? characterGaitSpeed3D(m_avatarGaitSpeeds, relative, sprint)
                    : maxSprint ? m_defaultMaxSprintSpeed
                    : athleteDefaultTravelSpeed3D(athleteRuntimeSettings(), relative, sprint);
                if (controlled != m_spawnedRagdolls.end() && !m_controlledAnimated) {
                    speed = std::min(speed, controlled->athlete.legacy.locomotion.telemetry().terrainSpeedLimit);
                }
                m_characterIntent.requestedDirectionWorld = direction;
                m_characterIntent.requestedVelocityWorld = direction * speed;
            }
        }
    }

    CharacterMotorSettings3D activeSettings = controllingCharacter
        ? m_avatarCharacterSettings : m_characterSettings;
    if (controllingCharacter && m_adaptivePhysics && m_controlledAnimated) {
        // Locomocao padrao: a capsula chega na velocidade definida quase na
        // hora e troca de direcao do mesmo jeito (como na Unreal) - o corpo e
        // so a animacao em cima dela.
        const AthleteCapsuleResponse3D response = athleteCapsuleResponse3D(m_capsuleResponsePercent * LabResponseScale, m_capsuleReversalPercent);
        activeSettings.groundAcceleration = response.acceleration;
        activeSettings.groundDeceleration = response.deceleration;
        activeSettings.groundTurnRate = response.turnRate;
        activeSettings.groundBrakeDeceleration = response.brake;
        activeSettings.groundReverseBrakeScale = response.reverseBrakeScale;
        activeSettings.groundReverseAccelerationScale = response.reverseAccelerationScale;
    } else if (controllingCharacter && m_adaptivePhysics) {
        activeSettings = adaptiveCharacterMotorSettings3D(activeSettings, stateBefore, command,
            m_physicalLocomotionV2 ? physicalNavigationAcceleration3D(m_e5WalkingAssistPercent * 0.01f) : 0.0f);
    }
    // Carga do pulo: o agachar de preparacao enquanto segura e, no pulo, a
    // carga que o corpo usa para o impulso.
    m_laboratoryJumpCrouch = m_laboratoryJumpCharging && stateBefore.grounded
        && command.moveDirection.lengthSquared() <= 0.0001f
        && std::hypot(stateBefore.velocity.x, stateBefore.velocity.y) <= 1.0f
        ? characterJumpCharge3D(activeSettings, m_laboratoryJumpChargeSeconds) : 0.0f;
    if (command.jumpPressed) {
        m_laboratoryJumpCharge = characterJumpCharge3D(activeSettings,
            command.jumpChargeSeconds);
    }
    if (controllingCharacter && m_biomechanicalExperiment) {
        Vec3 requested = command.moveDirection;
        requested.z = 0.0f;
        if (requested.lengthSquared() > 1.0f) {
            requested = requested.normalized();
        }
        m_biomechanicalRequestedVelocity = requested
            * (command.sprint ? 1.10f : 0.68f);
        m_characterSprinting = command.sprint
            && requested.lengthSquared() > 0.001f;
        if (!cameraOnly && !freeLook) {
            m_characterDesiredFacingYaw = m_laboratoryCameraYaw;
        }

        updateDynamicProps(deltaTime);
        // Depois da simulacao: camera e malha leem exatamente o mesmo par
        // de snapshots fisicos neste quadro.
        updateControlledPhysicalCameraFocus();
        return;
    }
    // Fisica adaptativa: a capsula vai atras do corpo (calculado no tick
    // anterior, em updateActiveRagdolls).
    if (controllingCharacter && m_adaptivePhysics) {
        command.followVelocity = m_characterFollowVelocity;
        command.navigationProxy = true;
    }
    // A voadora animada no ar (o padrao, ate encostar em algo): a capsula
    // segue no embalo do pulo. Com o carrinho em andamento o jogador conta como
    // caido e a capsula fica sem teclas - no ar ela freava ~2 m/s num voo de
    // meio segundo e o deslize saia lento.
    if (controllingCharacter && m_adaptivePhysics && m_controlledAnimated && !stateBefore.grounded) {
        const auto controlled = std::find_if(m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
            [&](const SpawnedRagdollInstance& item) { return item.entityId == m_controlledCharacterEntityId; });
        if (controlled != m_spawnedRagdolls.end() && controlled->athlete.legacy.defaultLocomotion.output().airTackle) {
            const Vec3 flat { stateBefore.velocity.x, stateBefore.velocity.y, 0.0f };
            const float speed = flat.length();
            if (speed > 0.05f && activeSettings.sprintSpeed > 0.1f) {
                command.moveDirection = flat * (1.0f / speed);
                command.sprint = true;
                command.speedScale = speed / activeSettings.sprintSpeed;
            }
        }
    }
    m_physicsScene->moveCharacter(command, activeSettings, deltaTime);
    const PhysicsCharacterState3D& character =
        m_physicsScene->characterState();
    // Batida da capsula: a velocidade no plano caiu, no rumo que tinha, mais
    // do que a frenagem dela tira num tick (parede no caminho). Pelo quanto
    // ela andou de fato: a velocidade dela e a pedida, e travada numa parede
    // seguia dizendo 3 m/s parada no lugar - a batida nunca contava e o corpo
    // do padrao corria parado ate um pe encostar (sem embalo nenhum).
    {
        const Vec3 moved { (character.position.x - stateBefore.position.x) / std::max(deltaTime, 1e-4f),
            (character.position.y - stateBefore.position.y) / std::max(deltaTime, 1e-4f), 0.0f };
        const float braking = std::max({ activeSettings.groundAcceleration, activeSettings.groundDeceleration,
            activeSettings.groundBrakeDeceleration }) * deltaTime;
        // (Subindo um degrau nao e parede.)
        const float rise = character.position.z - stateBefore.position.z;
        m_controlledWallImpact = controllingCharacter
            ? athleteCapsuleWallImpact3D(m_capsuleMovedVelocity, moved, braking, character.grounded,
                std::max(rise, m_capsuleRose))
            : 0.0f;
        m_capsuleRose = rise;
        m_capsuleMovedVelocity = moved;
    }
    if (character.flightExitBlocked) {
        m_laboratoryStatus = "Saia da geometria para desativar o voo";
    } else if (m_laboratoryStatus
            == "Saia da geometria para desativar o voo") {
        m_laboratoryStatus.clear();
    }

    // A altura dos olhos da primeira pessoa continua vindo da capsula. No
    // controle em terceira pessoa, o foco e substituido abaixo pelo torax
    // fisico do ragdoll.
    const float bodyHeight = character.crouched
        ? activeSettings.crouchedHeight
        : activeSettings.standingHeight;
    const float targetEyeHeight = character.crouched
        ? 1.02f : 1.68f;
    const float blend = 1.0f - std::exp(-14.0f * deltaTime);
    m_laboratoryEyeHeight +=
        (targetEyeHeight - m_laboratoryEyeHeight) * blend;
    const float feetZ = character.position.z - bodyHeight * 0.5f;
    if (controllingCharacter) {
        m_characterProxyVelocityWorld = {
            character.velocity.x, character.velocity.y, 0.0f
        };
        m_characterSprinting = command.sprint
            && m_characterProxyVelocityWorld.lengthSquared() > 0.01f;
        if (!cameraOnly && !freeLook && character.crouched
            && m_characterProxyVelocityWorld.lengthSquared() > 0.01f) {
            m_characterDesiredFacingYaw = std::atan2(
                m_characterProxyVelocityWorld.y,
                m_characterProxyVelocityWorld.x);
        } else if (!cameraOnly && !freeLook) {
            m_characterDesiredFacingYaw = m_laboratoryCameraYaw;
        }

    } else {
        m_characterProxyVelocityWorld = {};
        m_characterSprinting = false;
        setLaboratoryCameraFocus({ character.position.x,
                character.position.y,
                feetZ + m_laboratoryEyeHeight },
            LaboratoryCameraMode::FirstPerson);
    }
    updateDynamicProps(deltaTime);
    if (controllingCharacter) {
        // updateDynamicProps acabou de buscar o estado novo do ragdoll.
        updateControlledPhysicalCameraFocus();
    }
}

bool WorkbenchApp::updateControlledPhysicalCameraFocus() {
    if (!m_humanRagdollProfile || m_controlledCharacterEntityId == 0) {
        return false;
    }
    const auto controlled = std::find_if(m_spawnedRagdolls.begin(),
        m_spawnedRagdolls.end(), [&](const SpawnedRagdollInstance& item) {
            return item.entityId == m_controlledCharacterEntityId
                && item.active && !item.physicsState.links.empty();
        });
    if (controlled == m_spawnedRagdolls.end()) {
        return false;
    }

    // A capsula e apenas uma referencia de navegacao. O alvo e o centro do
    // corpo rigido UpperChest que acabou de sair do solver e que tambem sera
    // usado para desenhar a malha neste quadro.
    const auto linkPosition = [&](const char* id, Vec3& position) {
        for (std::size_t i = 0;
            i < m_humanRagdollProfile->links.size()
                && i < controlled->physicsState.links.size(); ++i) {
            if (m_humanRagdollProfile->links[i].id == id) {
                position = controlled->physicsState.links[i].position;
                return true;
            }
        }
        return false;
    };
    Vec3 physicalFocus;
    const bool hasUpperChest = linkPosition("UpperChest", physicalFocus);
    if (!hasUpperChest) {
        physicalFocus = controlled->physicsState.links.front().position;
    }
    setLaboratoryCameraFocus(physicalFocus,
        LaboratoryCameraMode::ThirdPerson);
    return true;
}

void WorkbenchApp::setLaboratoryCameraFocus(Vec3 focus,
    LaboratoryCameraMode mode) {
    // Troca de modo, ou um deslocamento que nenhum movimento fisico faz num
    // passo de 1/120 s (teleporte, spawn, assumir o personagem): nao se
    // interpola - o quadro intermediario desenharia a camera atravessando o
    // mapa.
    constexpr float MaximumStepTravelMeters = 1.0f;
    const bool continuous = mode == m_laboratoryCameraMode
        && (focus - m_laboratoryCameraFocus).length()
            <= MaximumStepTravelMeters;
    m_laboratoryCameraFocusPrevious =
        continuous ? m_laboratoryCameraFocus : focus;
    m_laboratoryCameraFocus = focus;
    m_laboratoryCameraMode = mode;
}

// A posicao da camera e montada por QUADRO, nao no passo fixo.
//
// O yaw muda a cada evento de mouse, ou seja, a cada quadro. A posicao em
// terceira pessoa (foco - frente(yaw) * distancia) era calculada so no passo
// fixo de 120 Hz. Num monitor de 143,8 Hz cerca de um quadro em cada cinco nao
// tem passo fixo, e nesses quadros a camera olhava na direcao do yaw novo a
// partir do ponto de orbita do yaw antigo: o centro da orbita escorregava
// ~distancia * delta-yaw para o lado e voltava no quadro seguinte. Tudo perto
// do personagem pulava na tela ao girar rapido - o sintoma que o usuario
// isolou com uma caixa parada ao lado dele. Medido pelo smoke
// laboratory-camera-orbit-smoke: desvio de trajetoria P95 de ~35 px.
//
// O foco tambem precisa estar no mesmo instante do boneco desenhado: o render
// interpola o corpo entre os dois ultimos passos com fixedStepAlpha, entao o
// foco usa o mesmo alfa. Com o estado cru do ultimo passo, camera e boneco
// ficavam em instantes diferentes mesmo nos quadros com passo.
void WorkbenchApp::updateLaboratoryCamera(float fixedStepAlpha) {
    if (m_laboratoryCameraMode == LaboratoryCameraMode::None) return;
    const float alpha = std::clamp(fixedStepAlpha, 0.0f, 1.0f);
    const Vec3 focus = m_laboratoryCameraFocusPrevious
        + (m_laboratoryCameraFocus - m_laboratoryCameraFocusPrevious) * alpha;
    if (m_laboratoryCameraMode == LaboratoryCameraMode::FirstPerson) {
        m_laboratoryCameraPosition = focus;
        return;
    }

    constexpr float OrbitDistanceMeters = 4.35f;
    // Encosta a camera numa parede em vez de atravessa-la, sem nunca colar
    // no foco.
    constexpr float MinimumOrbitDistanceMeters = 0.18f;
    constexpr float WallClearanceMeters = 0.12f;
    Vec3 cameraOffset = cameraForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized() * -OrbitDistanceMeters;
    PhysicsRayHit3D cameraHit;
    if (m_physicsScene && m_physicsScene->raycastStatic(
            { focus, cameraOffset / OrbitDistanceMeters },
            OrbitDistanceMeters, cameraHit)) {
        cameraOffset *= std::max(MinimumOrbitDistanceMeters,
            cameraHit.distance - WallClearanceMeters) / OrbitDistanceMeters;
    }
    m_laboratoryCameraPosition = focus + cameraOffset;
}

void WorkbenchApp::ensureLaboratoryMapLoaded(Renderer& activeRenderer) {
    if (m_laboratoryMapLoaded || m_laboratoryMapLoadFailed) {
        return;
    }

    // Dev-grid test platform: no map asset, no glTF, no ocean — a plain
    // checkered slab, generated procedurally (geometry and texture both),
    // replacing the old lab_map.glb the user asked to drop. A real block
    // with depth and side faces on purpose, not a single flat quad. A
    // scatter of static obstacles rides the same generator, for collision/
    // fall/trip/impact testing: walls of different sizes, a staircase, two
    // low trip curbs, a floating horizontal beam, a ramp and a platform.
    constexpr float PlatformHalfWidthMeters = 30.0f;
    constexpr float PlatformHalfDepthMeters = 30.0f;
    constexpr float PlatformThicknessMeters = 4.0f;
    constexpr float CheckerSquareMeters = 1.0f;
    constexpr std::uint32_t CheckerCellsPerTextureTile = 8;
    constexpr std::uint32_t CheckerTextureSize = 512;
    constexpr float MetersPerTextureTile =
        CheckerSquareMeters * static_cast<float>(CheckerCellsPerTextureTile);
    const Vec3 lightSquare { 0.80f, 0.80f, 0.82f };
    const Vec3 darkSquare { 0.58f, 0.58f, 0.60f };
    // "Rejunte": a thin, subtle seam between squares, in texels of the
    // 512x512 texture (64 texels/cell). Kept close to 1 texel and not
    // pure black on purpose — the first pass (3 texels, near-black) read
    // as a bold grid instead of a discreet line.
    const Vec3 groutColor { 0.24f, 0.24f, 0.25f };
    constexpr float GroutWidthTexels = 1.0f;

    const std::string materialId = "concrete";
    const SurfaceMaterial* material = m_materialLibrary.find(materialId);
    if (material == nullptr) {
        Log::error("Material fisico nao registrado no laboratorio: "
            + materialId);
        m_laboratoryMapLoadFailed = true;
        return;
    }

    // Shared by the floor and every obstacle below: one small tiling
    // texture, not one asset per prop.
    m_laboratoryMapTextures.push_back(activeRenderer.createTexture2D(
        { CheckerTextureSize, CheckerTextureSize },
        buildDevCheckerPixels(CheckerTextureSize, CheckerCellsPerTextureTile,
            lightSquare, darkSquare, groutColor, GroutWidthTexels)));
    const RHI::TextureHandle checkerTexture = m_laboratoryMapTextures.back();

    bool obstacleCreationFailed = false;
    const auto addDevBox = [&](std::string_view name, Vec3 center,
            Vec3 halfExtents, Quaternion orientation, bool useChecker) {
        if (obstacleCreationFailed) return;
        try {
            PhysicsShape3D collisionShape;
            collisionShape.type = PhysicsShapeType3D::Box;
            collisionShape.halfExtents = halfExtents;
            collisionShape.materialId = materialId;
            PhysicsBodyDefinition3D staticBody;
            staticBody.motionType = PhysicsMotionType3D::Static;
            staticBody.materialId = materialId;
            staticBody.position = center;
            staticBody.orientation = orientation;
            staticBody.entityId = m_nextEntityId++;
            m_laboratoryMapBodies.push_back(m_physicsScene->createBody(
                staticBody,
                std::span<const PhysicsShape3D>(&collisionShape, 1)));
        } catch (const std::exception& error) {
            Log::error("Falha ao criar colisao de " + std::string(name)
                + ": " + error.what());
            m_laboratoryMapLoadFailed = true;
            obstacleCreationFailed = true;
            return;
        }

        const MeshData3D boxMesh = buildDevBoxMesh(center, halfExtents,
            orientation, MetersPerTextureTile);
        LaboratoryMapPart gpuPart;
        const std::size_t vertexBytes =
            boxMesh.vertices.size() * sizeof(MeshVertex3D);
        const std::size_t indexBytes =
            boxMesh.indices.size() * sizeof(std::uint32_t);
        gpuPart.vertexBuffer = activeRenderer.createBuffer({ vertexBytes,
            RHI::BufferUsage::Vertex, true, "Laboratory map vertices" });
        gpuPart.indexBuffer = activeRenderer.createBuffer({ indexBytes,
            RHI::BufferUsage::Index, true, "Laboratory map indices" });
        activeRenderer.writeBuffer(gpuPart.vertexBuffer, 0,
            std::as_bytes(std::span(boxMesh.vertices)));
        activeRenderer.writeBuffer(gpuPart.indexBuffer, 0,
            std::as_bytes(std::span(boxMesh.indices)));
        gpuPart.indexCount =
            static_cast<std::uint32_t>(boxMesh.indices.size());
        gpuPart.boundsCenter =
            (boxMesh.boundsMin + boxMesh.boundsMax) * 0.5f;
        gpuPart.boundsRadius =
            (boxMesh.boundsMax - boxMesh.boundsMin).length() * 0.5f;
        gpuPart.name = name;
        gpuPart.materialId = materialId;
        gpuPart.metallic = 0.0f;
        gpuPart.roughness = 0.88f;
        gpuPart.matteSurface = true;
        // Only the floor carries the checker: every built structure is
        // plain white (no texture bound — the renderer's own 1x1 white
        // default material texture takes over, see defaultMaterialTexture
        // in VulkanDevice.cpp), as asked.
        if (useChecker) gpuPart.albedoTexture = checkerTexture;
        m_laboratoryMapParts.push_back(std::move(gpuPart));
    };

    addDevBox("DevGridPlatform",
        { 0.0f, 0.0f, -PlatformThicknessMeters * 0.5f },
        { PlatformHalfWidthMeters, PlatformHalfDepthMeters,
            PlatformThicknessMeters * 0.5f },
        {}, true);
    if (obstacleCreationFailed) return;

    // Five walls, deliberately varied in height/width/thickness/rotation.
    addDevBox("Wall_Tall", { 12.0f, 12.0f, 2.0f },
        { 1.5f, 0.15f, 2.0f }, {}, false);
    addDevBox("Wall_Medium", { -12.0f, 12.0f, 1.0f },
        { 2.0f, 0.15f, 1.0f },
        Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 0.52f), false);
    addDevBox("Wall_LowWide", { 12.0f, -12.0f, 0.5f },
        { 3.0f, 0.2f, 0.5f }, {}, false);
    addDevBox("Wall_NarrowTall", { -12.0f, -12.0f, 1.75f },
        { 0.6f, 0.15f, 1.75f },
        Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 1.05f), false);
    addDevBox("Wall_Angled", { 0.0f, 18.0f, 1.25f },
        { 1.5f, 0.25f, 1.25f },
        Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, 0.79f), false);

    // Staircase: each step is a solid block from the ground up to its own
    // height, so there is no gap underneath and no separate riser pieces
    // to align. Real proportions this time — 18 cm riser, 28 cm tread
    // (typical stair geometry; the first pass used 0.90 m of tread depth,
    // which is what made it read as a pile of deep ledges rather than a
    // normal staircase) — ten steps climbing along +Y.
    constexpr int StairStepCount = 10;
    constexpr float StairRiserMeters = 0.18f;
    constexpr float StairDepthMeters = 0.28f;
    constexpr float StairHalfWidthMeters = 1.2f;
    constexpr float StairStartX = -6.0f;
    constexpr float StairStartY = -3.0f;
    for (int step = 0; step < StairStepCount; ++step) {
        const float height = StairRiserMeters * static_cast<float>(step + 1);
        const float centerY = StairStartY
            + StairDepthMeters * (static_cast<float>(step) + 0.5f);
        addDevBox("StairStep_" + std::to_string(step),
            { StairStartX, centerY, height * 0.5f },
            { StairHalfWidthMeters, StairDepthMeters * 0.5f, height * 0.5f },
            {}, false);
    }

    // Two low trip curbs, exactly the heights asked for.
    addDevBox("TripCurb_20cm", { 6.0f, 6.0f, 0.10f },
        { 1.25f, 0.08f, 0.10f }, {}, false);
    addDevBox("TripCurb_30cm", { 9.5f, 6.0f, 0.15f },
        { 1.25f, 0.08f, 0.15f }, {}, false);

    // Floating horizontal beam: 2 m long, 30 cm square cross-section,
    // centered 1.2 m up — chest height, meant to be run/jumped into. Moved
    // well clear of the staircase (its first position at x=-6 sat right
    // inside the stairs' own footprint — that was the "wall inside the
    // staircase," not a separate object, and also why it was never
    // visible as its own thing).
    // (25/10: o arco e a viga sairam de x = 0, y = -18/-22 para o lado
    // leste, a pedido do usuario: no lugar deles fica a trave com rede.)
    addDevBox("FloatingBeam", { 22.0f, 4.0f, 1.2f },
        { 1.0f, 0.15f, 0.15f }, {}, false);

    // Ramp: a tilted slab whose low edge sits on the ground and high edge
    // reaches rampRiseMeters up, angle computed instead of hand-picked so
    // the geometry and the incline agree exactly.
    constexpr float RampRunMeters = 4.0f;
    constexpr float RampRiseMeters = 1.2f;
    constexpr float RampHalfWidthMeters = 1.25f;
    constexpr float RampThicknessMeters = 0.25f;
    const float rampLength =
        std::sqrt(RampRunMeters * RampRunMeters + RampRiseMeters * RampRiseMeters);
    const float rampAngle = std::atan2(RampRiseMeters, RampRunMeters);
    const Quaternion rampOrientation = Quaternion::fromAxisAngle(
        { 0.0f, 1.0f, 0.0f }, -rampAngle);
    const Vec3 rampHalfExtents {
        rampLength * 0.5f, RampHalfWidthMeters, RampThicknessMeters * 0.5f
    };
    const Vec3 rampLowGroundCorner { 6.0f, -6.0f, 0.0f };
    const Vec3 rampLowLocalCorner {
        -rampHalfExtents.x, 0.0f, -rampHalfExtents.z
    };
    const Vec3 rampCenter = rampLowGroundCorner
        - rampOrientation.rotate(rampLowLocalCorner);
    addDevBox("Ramp", rampCenter, rampHalfExtents, rampOrientation, false);

    // Elevated platform to land/jump onto.
    addDevBox("JumpPlatform", { -16.0f, -9.0f, 0.5f },
        { 1.5f, 1.5f, 0.5f }, {}, false);

    // More shapes, as asked: two free-standing pillars, a post-and-lintel
    // arch to walk through, three stacked crates, a walkable balance beam
    // and a low tunnel to duck under.
    addDevBox("Pillar_Tall", { 20.0f, 20.0f, 1.5f },
        { 0.4f, 0.4f, 1.5f }, {}, false);
    addDevBox("Pillar_Short", { -20.0f, -20.0f, 0.75f },
        { 0.5f, 0.5f, 0.75f }, {}, false);

    addDevBox("ArchPost_Left", { 20.8f, -4.0f, 1.25f },
        { 0.25f, 0.25f, 1.25f }, {}, false);
    addDevBox("ArchPost_Right", { 23.2f, -4.0f, 1.25f },
        { 0.25f, 0.25f, 1.25f }, {}, false);
    addDevBox("ArchLintel", { 22.0f, -4.0f, 2.65f },
        { 1.45f, 0.25f, 0.15f }, {}, false);

    addDevBox("Crate_Small", { 19.0f, -20.0f, 0.3f },
        { 0.3f, 0.3f, 0.3f }, {}, false);
    addDevBox("Crate_Medium", { 20.9f, -20.0f, 0.4f },
        { 0.4f, 0.4f, 0.4f }, {}, false);
    addDevBox("Crate_Large", { 19.5f, -21.8f, 0.5f },
        { 0.5f, 0.5f, 0.5f }, {}, false);

    // Wide enough on top to walk along, low enough on the side to trip on.
    addDevBox("BalanceBeam", { 0.0f, 24.0f, 0.25f },
        { 2.5f, 0.4f, 0.25f }, {}, false);

    addDevBox("TunnelWall_Left", { -21.2f, 20.0f, 1.0f },
        { 0.15f, 1.5f, 1.0f }, {}, false);
    addDevBox("TunnelWall_Right", { -18.8f, 20.0f, 1.0f },
        { 0.15f, 1.5f, 1.0f }, {}, false);
    addDevBox("TunnelRoof", { -20.0f, 20.0f, 2.1f },
        { 1.35f, 1.5f, 0.1f }, {}, false);
    if (obstacleCreationFailed) return;

    // A trave com rede (25/10, pedido do usuario: para treinar chutes e
    // colisoes): no lugar do arco e da viga, a linha do gol em y = -18 e a
    // boca virada para o centro do mapa (o nascimento do jogador), com ~11 m
    // livres na frente (o penalti).
    createLaboratoryGoal(activeRenderer, { 0.0f, -18.0f, 0.0f }, 1.5707963f);

    m_laboratorySpawnPosition = { 0.0f, 0.0f, 1.0f };
    m_laboratorySpawnYaw = 0.0f;
    m_worldAudio.setAcousticZones({});

    // O mapa só pode publicar o estado "carregado" depois de todos os
    // recursos terem sido criados. Assim, uma exceção nunca deixa uma cena
    // parcialmente utilizável sendo tratada como pronta.
    m_laboratoryMapLoaded = true;
    resetLaboratoryCharacter();
    Log::info("Plataforma xadrez de "
        + std::to_string(static_cast<int>(PlatformHalfWidthMeters * 2.0f))
        + "x" + std::to_string(static_cast<int>(PlatformHalfDepthMeters * 2.0f))
        + " m carregada (geometria e textura 100% procedurais).");
}

// Três anéis independentes formam um clipmap centrado na câmera. A densidade
// cai por fator quatro a cada anel, mantendo detalhe perto sem cobrir o
// horizonte inteiro com a mesma tesselação.
void WorkbenchApp::ensureOceanClipmap(Renderer& activeRenderer) {
    if (m_laboratoryOceanVertexBuffer) {
        return;
    }

    std::vector<Vec2> vertices;
    std::vector<std::uint32_t> indices;
    auto appendRing = [&vertices, &indices](float extent,
            float cellSize, float innerExtent) {
        const std::uint32_t cells = static_cast<std::uint32_t>(
            std::lround((extent * 2.0f) / cellSize));
        const std::uint32_t side = cells + 1;
        const std::uint32_t base =
            static_cast<std::uint32_t>(vertices.size());
        for (std::uint32_t row = 0; row < side; ++row) {
            for (std::uint32_t column = 0; column < side; ++column) {
                vertices.push_back({
                    -extent + static_cast<float>(column) * cellSize,
                    -extent + static_cast<float>(row) * cellSize
                });
            }
        }
        for (std::uint32_t row = 0; row < cells; ++row) {
            for (std::uint32_t column = 0; column < cells; ++column) {
                const float centerX = -extent
                    + (static_cast<float>(column) + 0.5f) * cellSize;
                const float centerY = -extent
                    + (static_cast<float>(row) + 0.5f) * cellSize;
                if (std::max(std::abs(centerX), std::abs(centerY))
                        < innerExtent) {
                    continue;
                }
                const std::uint32_t a = base + row * side + column;
                const std::uint32_t b = a + 1;
                const std::uint32_t c = a + side;
                const std::uint32_t d = c + 1;
                indices.insert(indices.end(), { a, c, b, b, c, d });
            }
        }
    };
    appendRing(OceanNearExtentMeters, OceanNearCellMeters, 0.0f);
    appendRing(OceanMidExtentMeters, OceanMidCellMeters,
        OceanNearExtentMeters);
    appendRing(OceanFarExtentMeters, OceanFarCellMeters,
        OceanMidExtentMeters);

    const std::size_t vertexBytes = vertices.size() * sizeof(Vec2);
    const std::size_t indexBytes = indices.size() * sizeof(std::uint32_t);
    m_laboratoryOceanVertexBuffer = activeRenderer.createBuffer(
        { vertexBytes, RHI::BufferUsage::Vertex, true,
            "Ocean clipmap vertices" });
    m_laboratoryOceanIndexBuffer = activeRenderer.createBuffer(
        { indexBytes, RHI::BufferUsage::Index, true,
            "Ocean clipmap indices" });
    activeRenderer.writeBuffer(m_laboratoryOceanVertexBuffer, 0,
        std::as_bytes(std::span(vertices)));
    activeRenderer.writeBuffer(m_laboratoryOceanIndexBuffer, 0,
        std::as_bytes(std::span(indices)));
    m_laboratoryOceanIndexCount =
        static_cast<std::uint32_t>(indices.size());
}

void WorkbenchApp::resetLaboratoryCharacter() {
    if (!m_physicsScene) return;
    m_controlledCharacterEntityId = 0;
    m_characterProxyVelocityWorld = {};
    m_characterSprinting = false;
    m_physicsScene->placeCharacter(m_laboratorySpawnPosition,
        m_characterSettings);
    m_laboratoryEyeHeight = 1.68f;
    m_laboratoryCameraPosition = m_laboratorySpawnPosition
        + Vec3 { 0.0f, 0.0f, m_laboratoryEyeHeight };
    m_laboratoryCameraMode = LaboratoryCameraMode::None;
    m_laboratoryCameraYaw = m_laboratorySpawnYaw;
    m_laboratoryCameraPitch = -0.08f;
    m_laboratoryCharacterInitialized = true;
    m_laboratoryJumpRequested = false;
    m_laboratoryFlightToggleRequested = false;
    // Teleporte/camera cut nao possui correspondencia valida no quadro
    // anterior. O proximo render semeia novamente o historico temporal.
    m_laboratoryTaaHistoryValid = false;
}

void WorkbenchApp::releaseLaboratoryAssets(Renderer& activeRenderer) {
    endPhysGunGrab();
    // A camera lenta e do laboratorio: fora dele, tempo real.
    setTimeScale(1.0f);
    if (m_physicsScene) {
        for (const SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
            m_physicsScene->destroyRagdoll(instance.physicsRagdoll);
        }
        for (const SpawnedPropInstance& instance : m_spawnedProps) {
            m_physicsScene->destroyBody(instance.physicsBody);
        }
        for (const SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
            m_physicsScene->destroyInflatedBall(ball.physicsBall);
        }
        for (const PhysicsBodyHandle3D body : m_laboratoryMapBodies) {
            m_physicsScene->destroyBody(body);
        }
        m_physicsScene->destroyCharacter();
    }
    releaseLaboratoryGoal(activeRenderer);
    m_spawnedRagdolls.clear();
    m_spawnedProps.clear();
    m_spawnedPropByBodyIndex.clear();
    m_inflatedBalls.clear();
    m_physicsBenchmarkPropEntityIds.clear();
    m_physicsBenchmarkRagdollEntityIds.clear();
    m_physicsBenchmarkRunning = false;
    m_physicsBenchmarkPaused = false;
    m_physicsBenchmarkSpawnedProps = 0;
    m_physicsBenchmarkSpawnedRagdolls = 0;
    m_laboratoryMapBodies.clear();
    m_propCatalog.release(activeRenderer);
    releaseGpuModel3D(activeRenderer, m_soccerBallVisual);
    m_soccerBallCage.clear();
    if (m_characterKit.valid()) activeRenderer.destroyCharacterMaterial(std::exchange(m_characterKit, {}));
    releaseGpuModel3D(activeRenderer, m_characterVisual);
    for (GpuModel3D& visual : m_ragdollLinkVisuals) {
        releaseGpuModel3D(activeRenderer, visual);
    }
    m_ragdollLinkVisuals.clear();
    releaseGpuModel3D(activeRenderer, m_ragdollJointVisual);
    m_ragdollJointLocalPositions.clear();
    m_humanRagdollProfile = nullptr;
    m_ragdollAssetsLoadFailed = false;
    releaseGpuModel3D(activeRenderer, m_physGunVisual);
    m_physGunVisualLoaded = false;
    releaseGpuModel3D(activeRenderer, m_viewerFloor);
    m_viewerFloorLoaded = false;
    m_propAssetsLoadFailed = false;
    for (LaboratoryMapPart& part : m_laboratoryMapParts) {
        if (part.vertexBuffer) {
            activeRenderer.destroyBuffer(part.vertexBuffer);
            part.vertexBuffer = {};
        }
        if (part.indexBuffer) {
            activeRenderer.destroyBuffer(part.indexBuffer);
            part.indexBuffer = {};
        }
        for (GpuMeshLod3D& lod : part.lods) {
            if (lod.indexBuffer) {
                activeRenderer.destroyBuffer(lod.indexBuffer);
                lod.indexBuffer = {};
            }
            lod.indexCount = 0;
        }
        part.lods.clear();
    }
    for (RHI::TextureHandle& texture : m_laboratoryMapTextures) {
        if (texture) {
            activeRenderer.destroyTexture(texture);
            texture = {};
        }
    }
    m_laboratoryMapParts.clear();
    m_laboratoryMapTextures.clear();
    m_laboratoryOceanEnabled = false;
    if (m_physicsScene) m_physicsScene->clearOcean();
    if (m_laboratoryOceanVertexBuffer) {
        activeRenderer.destroyBuffer(m_laboratoryOceanVertexBuffer);
        m_laboratoryOceanVertexBuffer = {};
    }
    if (m_laboratoryOceanIndexBuffer) {
        activeRenderer.destroyBuffer(m_laboratoryOceanIndexBuffer);
        m_laboratoryOceanIndexBuffer = {};
    }
    m_laboratoryOceanIndexCount = 0;
    m_laboratoryMapLoaded = false;
    m_laboratoryCharacterInitialized = false;
}

void WorkbenchApp::renderLaboratory3D(Renderer& activeRenderer) {
    ensureLaboratoryMapLoaded(activeRenderer);
    // O tamanho da bola mudou: a malha dela no raio novo (antes de montar o
    // quadro - nada desenhado ainda usa a antiga).
    if (m_soccerBallVisualDirty) {
        m_soccerBallVisualDirty = false;
        releaseGpuModel3D(activeRenderer, m_soccerBallVisual);
        m_soccerBallVisual = {};
        ensureSoccerBallVisual(activeRenderer);
        m_propPreviewAtlas = {};
    }
    updateLaboratoryCamera(applicationFrameMetrics().fixedStepAlpha);
    if (!m_spawnPropWhenReadyId.empty() && m_laboratoryMapLoaded
        && m_propCatalog.loaded()) {
        const auto& definitions = m_propCatalog.definitions();
        const auto found = std::find_if(definitions.begin(),
            definitions.end(), [&](const PropDefinition3D& definition) {
                return definition.id == m_spawnPropWhenReadyId;
            });
        const std::string requested = std::move(m_spawnPropWhenReadyId);
        m_spawnPropWhenReadyId.clear();
        if (found != definitions.end()) {
            spawnProp(static_cast<std::size_t>(
                std::distance(definitions.begin(), found)));
            Log::info("Smoke prop criado: " + requested);
        } else {
            Log::error("Smoke prop desconhecido: " + requested);
        }
    }
    if (m_spawnAllPropsWhenReady && m_laboratoryMapLoaded
        && m_propCatalog.loaded()) {
        m_spawnAllPropsWhenReady = false;
        const auto& suiteDefinitions = m_propCatalog.definitions();
        std::size_t spawned = 0;
        for (std::size_t index = 0;
                index < suiteDefinitions.size(); ++index) {
            // Linha espaçada e integralmente visível à câmera inicial.
            // Cada definição passa pelas mesmas consultas de overlap do
            // menu e do benchmark.
            const float offset = static_cast<float>(index)
                - (static_cast<float>(suiteDefinitions.size()) - 1.0f)
                    * 0.5f;
            const Vec3 requested {
                offset * 2.2f, -8.0f, 5.0f
            };
            const std::optional<Vec3> safe = findSafeSpawnPosition(
                requested,
                suiteDefinitions[index].dimensionsMeters * 0.5f,
                Quaternion {}, 3.0f);
            if (safe && spawnPropAt(
                    index, *safe, Quaternion {}, false)) {
                ++spawned;
            }
        }
        Log::info("Smoke suite de props: "
            + std::to_string(spawned) + "/"
            + std::to_string(suiteDefinitions.size())
            + " tipos criados.");
    }
    if (m_spawnRagdollWhenReady && m_laboratoryMapLoaded
        && m_humanRagdollProfile) {
        m_spawnRagdollWhenReady = false;
        spawnHumanRagdoll();
    }
    if((m_animationRunTest||m_manualRagdollInspection
            ||m_takeCharacterControlWhenReady)
        &&m_laboratoryMapLoaded&&m_humanRagdollProfile
        &&m_spawnedRagdolls.empty()) {
        // Ponto de spawn da plataforma de dev. A coordenada fixa anterior,
        // {-30,-45}, era do mapa glTF antigo e fica 15 m fora da plataforma
        // procedural de 60x60 m que o substituiu: sem chao embaixo,
        // ragdollGroundHeight devolvia a propria altura e o personagem caia no
        // vazio. Todos os modos de autostart que assumem o personagem
        // (character, animation, walk, pose) estavam quebrados por isso.
        Vec3 position{m_laboratorySpawnPosition.x, m_laboratorySpawnPosition.y,
            m_laboratorySpawnPosition.z + 50.0f};
        position.z=ragdollGroundHeight(position)+m_characterSpawnHalfExtents.z-m_characterSpawnCenter.z+0.055f;
        const auto safe=findSafeSpawnPosition(position+m_characterSpawnCenter,
            m_characterSpawnHalfExtents+Vec3{0.01f,0.01f,0.01f},{},5);
        if(!safe||!spawnHumanRagdollAt(*safe-m_characterSpawnCenter,{},false)) {
            m_animationRunTest=false;m_manualRagdollInspection=false;
            m_takeCharacterControlWhenReady=false;
            Log::error("Não foi possível criar o corpo na pista");
        }
    }
    if(m_takeCharacterControlWhenReady&&!m_spawnedRagdolls.empty()
        &&!m_spawnedRagdolls.front().physicsState.links.empty()) {
        m_takeCharacterControlWhenReady=false;
        takeControlOfLatestCharacter();
    }
    if((m_animationRunTest||m_manualRagdollInspection)&&!m_spawnedRagdolls.empty()&&!m_spawnedRagdolls.front().physicsState.links.empty()) {
        const Vec3 target=m_spawnedRagdolls.front().physicsState.links.front().position+Vec3{0,0,0.25f};
        const Vec3 offset{-2.0f,-5.0f,1.0f};
        m_laboratoryCameraPosition=target+offset;
        m_laboratoryCameraYaw=std::atan2(-offset.y,-offset.x);
        m_laboratoryCameraPitch=std::atan2(-offset.z,std::sqrt(offset.x*offset.x+offset.y*offset.y));
        // Manual inspection positions the camera once; no gait is requested.
        m_manualRagdollInspection=false;
    }
    const bool pixelArtMode =
        m_laboratoryRenderMode == SceneRenderMode3D::PixelArt;
    // O Mosaic conserva histórico somente no céu para estabilizar estrelas.
    // Geometria continua no caminho instantâneo antigo quando o TAA
    // experimental está desligado. No modo Normal, o perfil Desempenho
    // remove o passe fullscreen; Equilibrado/Alto conservam o TAA completo.
    const bool temporalResolveEnabled = pixelArtMode
        || m_laboratoryGraphicsPreset != 0;
    const bool temporalJitterEnabled = temporalResolveEnabled
        && (!pixelArtMode
            || m_laboratoryPixelArt.temporalAntiAliasingEnabled);
    const bool temporalHistoryValid =
        m_laboratoryTaaHistoryValid
        && temporalResolveEnabled;

    const float aspect = static_cast<float>(std::max(1, activeRenderer.width()))
        / static_cast<float>(std::max(1, activeRenderer.height()));
    const Mat4 view = Mat4::lookAt(m_laboratoryCameraPosition,
        m_laboratoryCameraPosition + cameraForward(
            m_laboratoryCameraYaw, m_laboratoryCameraPitch),
        { 0.0f, 0.0f, 1.0f });
    const Mat4 projection = Mat4::perspective(
        67.0f * Pi / 180.0f, aspect, 0.08f, LaboratoryFarPlane);
    m_laboratoryViewProjection = projection * view;

    // Fase 6 (TAA): amostra de Halton (base 2/3) convertida de [0,1) pra um
    // deslocamento sub-pixel em espaco NDC (2 unidades de NDC cobrem a tela
    // inteira em cada eixo, entao 1 pixel = 2/extent unidades de NDC) -
    // ver JitterSequence.hpp e Mat4::perspectiveJittered.
    Mat4 cameraViewProjectionJittered = m_laboratoryViewProjection;
    if (temporalJitterEnabled) {
        const float renderScale = pixelArtMode
            ? std::clamp(m_laboratoryPixelArt.renderScale,
                0.25f, 1.0f)
            : 1.0f;
        const float jitterWidth = static_cast<float>(std::max(1,
            static_cast<int>(std::lround(
                static_cast<float>(activeRenderer.width())
                    * renderScale))));
        const float jitterHeight = static_cast<float>(std::max(1,
            static_cast<int>(std::lround(
                static_cast<float>(activeRenderer.height())
                    * renderScale))));
        const Vec2 haltonSample = haltonJitter(m_taaFrameIndex);
        const Vec2 ndcJitter {
            (haltonSample.x - 0.5f) * 2.0f / jitterWidth,
            (haltonSample.y - 0.5f) * 2.0f / jitterHeight
        };
        const Mat4 jitteredProjection = Mat4::perspectiveJittered(
            67.0f * Pi / 180.0f, aspect, 0.08f,
            LaboratoryFarPlane, ndcJitter);
        cameraViewProjectionJittered = jitteredProjection * view;
    }

    // Distancia maxima que a sombra precisa cobrir - deliberadamente bem
    // menor que LaboratoryFarPlane (2000m): nevoa/distancia ja escondem
    // qualquer sombra la longe (ver scene.fogSettings em scene3d_mesh.frag),
    // entao gastar cascatas ate o plano distante da camera desperdicaria a
    // cascata mais proxima (a que mais importa) numa faixa gigante demais.
    // 450m cobre todo o espaco jogavel atual e ainda permite uma transicao
    // final gradual antes da neblina dominar, sem gastar cascatas nos 2000m
    // do plano distante usados apenas pelo ceu/horizonte.
    constexpr float ShadowMaxDistanceMeters = 450.0f;
    // Prioriza fortemente resolucao proxima. Lambda 0.5 produzia a primeira
    // fronteira perto de 38m (com far=300), tornando pernas/objetos pequenos
    // borrados mesmo ao lado do jogador.
    constexpr float CascadeSplitLambda = 0.85f;
    constexpr float CascadeBlendFraction = 0.12f;

    const Vec3 sunDirection = solarDirectionForHour(
        m_laboratoryEnvironment.solarTimeHours);
    const Vec3 moonDirection = -sunDirection;
    m_laboratorySunDirection = sunDirection;
    const float sunElevation = sunDirection.z;
    const float daylight = smoothUnit(-0.10f, 0.10f, sunElevation);
    const float directSun = smoothUnit(-0.015f, 0.16f, sunElevation);
    const float moonlight = smoothUnit(0.02f, 0.22f, moonDirection.z);
    const float cloudCoverage = std::clamp(
        m_laboratoryEnvironment.cloudCoverage, 0.0f, 1.0f);
    const float cloudTransmission = cloudTransmittanceAtSun(
        cloudCoverage, m_laboratoryElapsedTime, m_cloudWindOffset,
        sunDirection);
    m_laboratoryEnvironment.cloudSunTransmittance =
        cloudTransmission;

    // A neblina precisa desaparecer dentro do próprio horizonte, não formar
    // uma camada RGB independente. Estes valores são os mesmos horizontes
    //-base do shader do céu; crepúsculo e cobertura de nuvens fazem a
    // transição contínua durante todo o ciclo.
    const float nightFactor =
        1.0f - smoothUnit(-0.16f, 0.025f, sunElevation);
    const float twilight = (1.0f
        - smoothUnit(0.015f, 0.30f, std::abs(sunElevation)))
        * (1.0f - nightFactor * 0.55f);
    const Vec3 nightFog { 0.018f, 0.026f, 0.060f };
    const Vec3 dayFog { 0.610f, 0.750f, 0.880f };
    Vec3 dynamicFog = mixColor(nightFog, dayFog, daylight);
    dynamicFog += Vec3 { 0.34f, 0.075f, 0.012f } * twilight;
    const Vec3 overcastFog = mixColor(
        { 0.025f, 0.031f, 0.050f },
        { 0.660f, 0.690f, 0.720f }, daylight);
    dynamicFog = mixColor(dynamicFog, overcastFog,
        cloudCoverage * 0.48f);
    m_laboratoryFog.color = dynamicFog;
    m_laboratoryFog.endDistanceMeters = std::max(
        m_laboratoryFog.endDistanceMeters,
        m_laboratoryFog.startDistanceMeters + 1.0f);
    m_laboratoryFog.maxOpacity = 1.0f;

    // O céu nublado perde menos energia que o feixe direto: as nuvens
    // espalham a luz pelo hemisfério em vez de simplesmente apagá-la.
    const float daytimeSky = 0.31f
        * (1.0f - 0.26f * cloudCoverage);
    // Mesmo sem luz artificial, a Lua e o espalhamento atmosférico deixam
    // informação suficiente para leitura de silhuetas, sem transformar a
    // noite em dia nem depender de adaptação automática de exposição.
    const float nighttimeSky = 0.024f + moonlight
        * (0.020f * (1.0f - 0.55f * cloudCoverage));
    m_laboratoryEnvironment.skyIrradiance = nighttimeSky
        + daylight * (daytimeSky - nighttimeSky);
    m_laboratoryEnvironment.sunDiffuseBounce =
        0.058f * directSun * (0.72f + cloudCoverage * 0.18f);

    const Vec3 cameraForwardVector =
        cameraForward(m_laboratoryCameraYaw, m_laboratoryCameraPitch)
            .normalized();
    const CameraFrustumParameters3D cameraFrustumParameters {
        m_laboratoryCameraPosition, cameraForwardVector,
        { 0.0f, 0.0f, 1.0f }, 67.0f * Pi / 180.0f, aspect
    };
    const std::vector<float> cascadeFarDistances = computeCascadeSplits(
        0.08f, ShadowMaxDistanceMeters, ShadowCascadeCount, CascadeSplitLambda);

    std::array<Mat4, ShadowCascadeCount> cascadeViewProjections {};
    std::array<float, ShadowCascadeCount> cascadeSplits {};
    std::array<float, ShadowCascadeCount> cascadeTexelWorldSizes {};
    std::array<float, ShadowCascadeCount> cascadeDepthRanges {};
    std::array<Frustum3D, ShadowCascadeCount> lightFrustums {};
    float previousCascadeNear = 0.08f;
    for (std::uint32_t cascade = 0; cascade < ShadowCascadeCount; ++cascade) {
        const float cascadeFar = cascadeFarDistances.size() > cascade
            ? cascadeFarDistances[cascade] : ShadowMaxDistanceMeters;
        // Cascatas adjacentes se sobrepoem na faixa em que o shader mistura
        // as duas. Sem essa geometria extra, a segunda cascata seria lida
        // antes do seu near real e a transicao mostraria regioes vazias.
        const float nominalLength = cascadeFar - previousCascadeNear;
        const float fittedNear = cascade == 0 ? previousCascadeNear
            : std::max(0.08f, previousCascadeNear
                - nominalLength * CascadeBlendFraction);
        const FittedShadowCascade fitted = fitCascadeFrustumToCamera(
            cameraFrustumParameters, fittedNear, cascadeFar,
            sunDirection,
            static_cast<float>(ShadowCascadeMapSizes[cascade]));
        cascadeViewProjections[cascade] = fitted.viewProjection;
        cascadeSplits[cascade] = cascadeFar;
        cascadeTexelWorldSizes[cascade] = fitted.texelWorldSizeMeters;
        cascadeDepthRanges[cascade] = fitted.depthRangeMeters;
        lightFrustums[cascade] = Frustum3D::fromViewProjection(
            cascadeViewProjections[cascade], /*reversedDepth=*/false);
        previousCascadeNear = cascadeFar;
    }

    const Frustum3D cameraFrustum =
        Frustum3D::fromViewProjection(m_laboratoryViewProjection,
            /*reversedDepth=*/true);

    // Pose desenhada de cada prop neste quadro, interpolada no mesmo instante
    // do boneco e da camera. Culling e malha usam esta, nao a do passo cru.
    const float propAlpha = std::clamp(
        applicationFrameMetrics().fixedStepAlpha, 0.0f, 1.0f);
    for (SpawnedPropInstance& instance : m_spawnedProps) {
        instance.renderedState =
            instance.updatedAtPhysicsStep == m_laboratoryPhysicsStep
            ? interpolatePose(instance.simulationPreviousState,
                instance.physicsState, propAlpha)
            : instance.physicsState;
        // O feixe da Physgun termina no ponto agarrado do objeto DESENHADO.
        // O passo fixo o calcula a partir do estado cru (e antes de simular),
        // o que descolava o feixe do prop ao girar a camera segurando algo.
        if (m_physGunGrabbedEntityId != 0
            && instance.entityId == m_physGunGrabbedEntityId) {
            m_physGunBeamTargetWorldPosition = instance.renderedState.position
                + instance.renderedState.orientation.rotate(
                    m_physGunLocalGrabPoint);
        }
    }

    std::vector<MeshRender3D> meshes;
    const auto& definitions = m_propCatalog.definitions();
    std::vector<std::uint8_t> propVisibility(m_spawnedProps.size(), 0);
    struct VisibilityContext {
        const std::vector<SpawnedPropInstance>* instances = nullptr;
        const std::vector<PropDefinition3D>* definitions = nullptr;
        const Frustum3D* camera = nullptr;
        // Uma por cascata - um prop conta como "projeta sombra" se estiver
        // dentro de QUALQUER uma delas (as fatias sao adjacentes ao longo da
        // distancia de camera, nao aninhadas, entao a uniao das 4 e o teste
        // correto, nao so a mais distante).
        const std::array<Frustum3D, ShadowCascadeCount>* lightCascades = nullptr;
        std::vector<std::uint8_t>* output = nullptr;
    } visibilityContext {
        &m_spawnedProps, &definitions, &cameraFrustum, &lightFrustums,
        &propVisibility
    };
    taskScheduler()->parallelFor(m_spawnedProps.size(), 64,
        [](std::size_t begin, std::size_t end, void* rawContext) noexcept {
            auto& context = *static_cast<VisibilityContext*>(rawContext);
            for (std::size_t index = begin; index < end; ++index) {
                const SpawnedPropInstance& instance =
                    (*context.instances)[index];
                if (instance.definitionIndex
                    >= context.definitions->size()) {
                    continue;
                }
                const Vec3 dimensions = (*context.definitions)[
                    instance.definitionIndex].dimensionsMeters;
                const float radius = dimensions.length() * 0.5f;
                const Vec3 center = instance.renderedState.position;
                std::uint8_t visibility = 0;
                if (context.camera->intersectsSphere(center, radius)) {
                    visibility |= 1u;
                }
                for (std::size_t cascade = 0;
                        cascade < context.lightCascades->size(); ++cascade) {
                    if ((*context.lightCascades)[cascade].intersectsSphere(
                            center, radius)) {
                        // Bit 0 pertence a camera; bits 1..4 representam as
                        // quatro cascatas individualmente.
                        visibility |= static_cast<std::uint8_t>(
                            1u << (cascade + 1u));
                    }
                }
                (*context.output)[index] = visibility;
            }
        }, &visibilityContext);
    std::vector<std::size_t> dynamicOffsets(m_spawnedProps.size() + 1, 0);
    for (std::size_t index = 0; index < m_spawnedProps.size(); ++index) {
        const SpawnedPropInstance& instance = m_spawnedProps[index];
        dynamicOffsets[index + 1] = dynamicOffsets[index];
        if (propVisibility[index] != 0
            && instance.definitionIndex < definitions.size()) {
            dynamicOffsets[index + 1] += definitions[instance.definitionIndex]
                .visual.parts.size();
        }
    }
    meshes.reserve(m_laboratoryMapParts.size() + dynamicOffsets.back()
        + m_physGunVisual.parts.size()
        + m_spawnedRagdolls.size() * 38u + 64u);
    for (const LaboratoryMapPart& part : m_laboratoryMapParts) {
        const bool visibleInCamera = cameraFrustum.intersectsSphere(
            part.boundsCenter, part.boundsRadius);
        std::uint8_t shadowCascadeMask = 0;
        for (std::size_t cascade = 0;
                cascade < lightFrustums.size(); ++cascade) {
            if (lightFrustums[cascade].intersectsSphere(
                    part.boundsCenter, part.boundsRadius)) {
                shadowCascadeMask |= static_cast<std::uint8_t>(
                    1u << cascade);
            }
        }
        if (!visibleInCamera && shadowCascadeMask == 0) continue;
        MeshRender3D mesh;
        mesh.vertexBuffer = part.vertexBuffer;
        mesh.indexBuffer = part.indexBuffer;
        mesh.indexCount = part.indexCount;
        if (!part.lods.empty()) {
            const float relativeDistance =
                (part.boundsCenter - m_laboratoryCameraPosition).length()
                / std::max(part.boundsRadius, 0.001f);
            // Tamanho projetado, não distância absoluta: uma arquibancada
            // grande conserva detalhe muito além de uma pequena lixeira.
            if (relativeDistance > 42.0f) {
                mesh.indexBuffer = part.lods.back().indexBuffer;
                mesh.indexCount = part.lods.back().indexCount;
            } else if (relativeDistance > 16.0f) {
                mesh.indexBuffer = part.lods.front().indexBuffer;
                mesh.indexCount = part.lods.front().indexCount;
            }
            mesh.shadowIndexBuffer = part.lods.back().indexBuffer;
            mesh.shadowIndexCount = part.lods.back().indexCount;
        }
        mesh.albedoTexture = part.albedoTexture;
        mesh.metallic = part.metallic;
        mesh.roughness = part.roughness;
        mesh.matteSurface = part.matteSurface;
        mesh.visibleInCamera = visibleInCamera;
        mesh.castsShadow = shadowCascadeMask != 0;
        mesh.shadowCascadeMask = shadowCascadeMask;
        meshes.push_back(mesh);
    }

    const std::size_t dynamicBase = meshes.size();
    meshes.resize(dynamicBase + dynamicOffsets.back());
    struct RenderPreparationContext {
        const std::vector<SpawnedPropInstance>* instances = nullptr;
        const std::vector<PropDefinition3D>* definitions = nullptr;
        const std::vector<std::size_t>* offsets = nullptr;
        const std::vector<std::uint8_t>* visibility = nullptr;
        std::vector<MeshRender3D>* output = nullptr;
        std::size_t outputBase = 0;
        std::uint64_t heldEntityId = 0;
        Vec3 cameraPosition;
        bool temporalHistoryValid = false;
    } preparation {
        &m_spawnedProps, &definitions, &dynamicOffsets, &propVisibility,
        &meshes,
        dynamicBase, m_physGunGrabbedEntityId, m_laboratoryCameraPosition,
        temporalHistoryValid
    };
    taskScheduler()->parallelFor(m_spawnedProps.size(), 64,
        [](std::size_t begin, std::size_t end, void* rawContext) noexcept {
            auto& context = *static_cast<RenderPreparationContext*>(
                rawContext);
            for (std::size_t index = begin; index < end; ++index) {
                const SpawnedPropInstance& instance =
                    (*context.instances)[index];
                if (instance.definitionIndex
                        >= context.definitions->size()
                    || (*context.visibility)[index] == 0) {
                    continue;
                }
                const PropDefinition3D& definition =
                    (*context.definitions)[instance.definitionIndex];
                const bool highlighted = instance.entityId
                        == context.heldEntityId
                    || instance.freezeFlashSeconds > 0.0f;
                const std::size_t first = context.outputBase
                    + (*context.offsets)[index];
                const bool visibleInCamera =
                    ((*context.visibility)[index] & 1u) != 0;
                const bool castsShadow =
                    ((*context.visibility)[index] & 0x1Eu) != 0;
                const std::uint8_t shadowCascadeMask =
                    static_cast<std::uint8_t>(
                        (*context.visibility)[index] >> 1u);
                std::span<MeshRender3D> destination =
                    std::span<MeshRender3D>(*context.output).subspan(
                        first, definition.visual.parts.size());
                writeGpuModelRenderables(definition.visual,
                    instance.renderedState.position,
                    instance.renderedState.orientation,
                    context.temporalHistoryValid
                        ? instance.previousPhysicsState.position
                        : instance.renderedState.position,
                    context.temporalHistoryValid
                        ? instance.previousPhysicsState.orientation
                        : instance.renderedState.orientation,
                    1.0f,
                    highlighted, highlighted, castsShadow, destination,
                    (instance.renderedState.position
                        - context.cameraPosition).length());
                for (MeshRender3D& mesh : destination) {
                    mesh.visibleInCamera = visibleInCamera;
                    mesh.shadowCascadeMask = shadowCascadeMask;
                    mesh.pixelArtHighDetail = true;
                }
            }
        }, &preparation);

    // Characters share a weighted surface and submit only bone palettes.
    for (SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
        const bool highlighted =
            instance.entityId == m_physGunGrabbedEntityId;
        const auto& linkVisuals = m_ragdollLinkVisuals;
        const auto& jointLocalPositions = m_ragdollJointLocalPositions;
        const std::size_t linkCount = std::min(
            instance.physicsState.links.size(),
            linkVisuals.size());
        if (linkCount == 0) continue;

        // Culling grosso uma única vez por articulação. Antes cada segmento
        // era enviado à câmera e às QUATRO cascatas mesmo quando o ragdoll
        // inteiro estava fora delas. Além do trabalho desperdiçado, 30
        // ragdolls multiplicavam mais de mil pequenas instâncias por quatro
        // passes de sombra. A esfera é conservadora: nunca corta um membro
        // isolado que ainda esteja visível.
        Vec3 ragdollCenter {};
        for (std::size_t linkIndex = 0; linkIndex < linkCount; ++linkIndex) {
            ragdollCenter +=
                instance.physicsState.links[linkIndex].position;
        }
        ragdollCenter *= 1.0f / static_cast<float>(linkCount);
        float ragdollRadius = 0.35f;
        for (std::size_t linkIndex = 0; linkIndex < linkCount; ++linkIndex) {
            ragdollRadius = std::max(ragdollRadius,
                (instance.physicsState.links[linkIndex].position
                    - ragdollCenter).length() + 0.22f);
        }
        const bool ragdollVisible =
            cameraFrustum.intersectsSphere(ragdollCenter, ragdollRadius);
        std::uint8_t ragdollShadowMask = 0;
        for (std::size_t cascade = 0;
                cascade < lightFrustums.size(); ++cascade) {
            if (lightFrustums[cascade].intersectsSphere(
                    ragdollCenter, ragdollRadius)) {
                ragdollShadowMask |= static_cast<std::uint8_t>(
                    1u << cascade);
            }
        }
        if (!ragdollVisible && ragdollShadowMask == 0) continue;
        const bool ragdollCastsShadow = ragdollShadowMask != 0;

        const bool previousStateValid = temporalHistoryValid
            && instance.previousPhysicsState.links.size() >= linkCount;
        if (instance.kind == ArticulatedRigKind::Human && m_ragdollCharacter) {
            // O personagem do corpo do laboratorio (a paleta e os ossos visuais).
            const RagdollCharacter3D& character = laboratoryCharacter();
            std::vector<Vec3> positions, previousPositions;
            std::vector<Quaternion> orientations, previousOrientations;
            // Always the simulated body: there is exactly one character here,
            // and it is the physical one. Drawing an animated copy over it
            // (the Unreal PhysicsBlendWeight split) meant limbs went through
            // walls, because nothing the player saw was ever in the solver.
            //
            // Interpolated across the fixed step this frame is sitting in.
            // Physics runs at 120 Hz and frames follow the display, so a
            // frame that lands between two steps used to redraw the previous
            // one unchanged: the world and the camera moved smoothly while
            // the character advanced in jerks of its own. That reads as the
            // body glitching whenever the camera turns, and no amount of
            // work on the simulation side can remove it.
            const float alpha = std::clamp(
                applicationFrameMetrics().fixedStepAlpha, 0.0f, 1.0f);
            const bool interpolate =
                instance.simulationPreviousState.links.size() >= linkCount;
            for (std::size_t i=0; i<linkCount; ++i) {
                const auto& state=instance.physicsState.links[i];
                if (interpolate) {
                    const auto& from =
                        instance.simulationPreviousState.links[i];
                    positions.push_back(from.position
                        + (state.position - from.position) * alpha);
                    orientations.push_back(blendRotation(from.orientation,
                        state.orientation, alpha));
                } else {
                    positions.push_back(state.position);
                    orientations.push_back(state.orientation);
                }
            }
            // Mesmo motivo do prop: o feixe termina no link desenhado.
            if (m_physGunGrabbedEntityId != 0
                && instance.entityId == m_physGunGrabbedEntityId
                && m_physicsScene && m_physicsScene->grabbing()) {
                const std::uint32_t grabbedLink =
                    m_physicsScene->grabbedRagdollLink();
                if (grabbedLink < positions.size()) {
                    m_physGunBeamTargetWorldPosition = positions[grabbedLink]
                        + orientations[grabbedLink].rotate(
                            m_physGunLocalGrabPoint);
                }
            }
            // Motion vectors compare against what was actually drawn last
            // frame, not against a simulation step nobody saw.
            const bool renderedHistoryValid = previousStateValid
                && instance.renderedPositions.size() == linkCount
                && instance.renderedOrientations.size() == linkCount;
            for (std::size_t i=0; i<linkCount; ++i) {
                previousPositions.push_back(renderedHistoryValid
                    ? instance.renderedPositions[i] : positions[i]);
                previousOrientations.push_back(renderedHistoryValid
                    ? instance.renderedOrientations[i] : orientations[i]);
            }
            instance.renderedPositions = positions;
            instance.renderedOrientations = orientations;
            // Os ossos visuais (07/10): os dedos do pe no chao embaixo de cada
            // pe e a pose das maos (com transicao no tempo do quadro).
            const double now = ImGui::GetTime();
            const float poseDt = instance.visualPoseTime >= 0.0
                ? static_cast<float>(std::clamp(now - instance.visualPoseTime, 0.0, 0.1)) : 0.0f;
            instance.visualPoseTime = now;
            {
                RagdollVisualPoseInput3D visualInput;
                visualInput.linkPositions = positions;
                visualInput.linkOrientations = orientations;
                for (std::size_t side = 0; side < 2; ++side) {
                    const char* id = side == 0 ? "LeftFoot" : "RightFoot";
                    for (std::size_t link = 0; link < character.profile.links.size(); ++link)
                        if (link < positions.size() && character.profile.links[link].id == id)
                            visualInput.groundHeight[side] = ragdollGroundHeight(positions[link]);
                }
                visualInput.handPose = { m_handPose, m_handPose };
                visualInput.toesFollowGround = m_toeRoll;
                visualInput.deltaTime = poseDt;
                instance.previousVisualRotations = instance.visualRotations;
                instance.visualRotations = instance.visualPose.update(character, visualInput);
                if (instance.previousVisualRotations.size() != instance.visualRotations.size())
                    instance.previousVisualRotations = instance.visualRotations;
            }
            instance.skinMatrices=buildRagdollSkinMatrices3D(character,positions,orientations,
                instance.visualRotations);
            instance.previousSkinMatrices=buildRagdollSkinMatrices3D(character,previousPositions,
                previousOrientations,instance.previousVisualRotations);
            const auto first=meshes.size();
            appendGpuModelRenderables(m_characterVisual, {}, {}, {}, {},
                1.0f, highlighted, highlighted, ragdollCastsShadow, meshes);
            for (std::size_t i=first; i<meshes.size(); ++i) {
                meshes[i].skinMatrices=instance.skinMatrices;
                meshes[i].previousSkinMatrices=instance.previousSkinMatrices;
                meshes[i].flatShaded=character.flatShaded;
                meshes[i].facet=character.facet;
                meshes[i].pixelArtHighDetail=true;
                meshes[i].visibleInCamera=ragdollVisible;
                meshes[i].shadowCascadeMask=ragdollShadowMask;
                meshes[i].tintColor=instance.tintColor;
                // O uniforme do sistema de aparencia (camada 0: um uniforme e
                // um numero para todos, por enquanto).
                meshes[i].characterMaterial=m_characterKit;
            }
            continue;
        }
        for (std::size_t linkIndex = 0;
                linkIndex < linkCount; ++linkIndex) {
            const PhysicsBodyState3D& state =
                instance.physicsState.links[linkIndex];
            const PhysicsBodyState3D& previous = previousStateValid
                ? instance.previousPhysicsState.links[linkIndex] : state;
            const std::size_t first = meshes.size();
            appendGpuModelRenderables(linkVisuals[linkIndex],
                state.position, state.orientation,
                previous.position, previous.orientation,
                1.0f, highlighted, highlighted,
                ragdollCastsShadow, meshes);
            for (std::size_t part = first; part < meshes.size(); ++part) {
                meshes[part].pixelArtHighDetail = true;
                meshes[part].visibleInCamera = ragdollVisible;
                meshes[part].shadowCascadeMask = ragdollShadowMask;
            }
            if (linkIndex > 0
                && linkIndex < jointLocalPositions.size()
                && !m_ragdollJointVisual.parts.empty()) {
                const Vec3 localJoint =
                    jointLocalPositions[linkIndex];
                const Vec3 jointPosition = state.position
                    + state.orientation.rotate(localJoint);
                const Vec3 previousJointPosition = previous.position
                    + previous.orientation.rotate(localJoint);
                const std::size_t jointFirst = meshes.size();
                appendGpuModelRenderables(m_ragdollJointVisual,
                    jointPosition, state.orientation,
                    previousJointPosition, previous.orientation,
                    0.063f, highlighted, highlighted,
                    ragdollCastsShadow, meshes);
                for (std::size_t part = jointFirst;
                        part < meshes.size(); ++part) {
                    meshes[part].pixelArtHighDetail = true;
                    meshes[part].visibleInCamera = ragdollVisible;
                    meshes[part].shadowCascadeMask =
                        ragdollShadowMask;
                }
            }
        }
    }

    appendInflatedBallRenderables(meshes, cameraFrustum, lightFrustums,
        temporalHistoryValid);
    appendLaboratoryGoalRenderables(meshes, cameraFrustum, lightFrustums,
        temporalHistoryValid);

    Vec3 renderedWeaponPosition;
    Quaternion renderedWeaponOrientation;
    bool renderedWeapon = false;
    if (m_physGunVisualLoaded && !m_physGunVisual.parts.empty()) {
        const Vec3 forward = cameraForward(m_laboratoryCameraYaw,
            m_laboratoryCameraPitch).normalized();
        const Vec3 planarForward = cameraForward(
            m_laboratoryCameraYaw, 0.0f);
        const Vec3 right { planarForward.y, -planarForward.x, 0.0f };
        const Vec3 cameraUp = cross(right, forward).normalized();
        const Vec3 dimensions = m_physGunVisual.dimensionsMeters;
        const float largest = std::max({ dimensions.x,
            dimensions.y, dimensions.z, 0.001f });
        const Quaternion viewOrientation =
            Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                m_laboratoryCameraYaw)
            * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f },
                -m_laboratoryCameraPitch);
        const Vec3 rotationRadians =
            m_physGunViewCalibration.rotationDegrees * (Pi / 180.0f);
        const Quaternion localCalibration =
            Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f },
                rotationRadians.x)
            * Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f },
                rotationRadians.y)
            * Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f },
                rotationRadians.z);
        Vec3 localOffset = m_physGunViewCalibration.offsetCameraLocal;
        localOffset.y -= m_physGunRecoilOffset;
        const Vec3 weaponPosition = m_laboratoryCameraPosition
            + right * localOffset.x + forward * localOffset.y
            + cameraUp * localOffset.z;
        const Quaternion weaponOrientation = viewOrientation * localCalibration;
        if (!m_hidePhysGunPresentation) {
            appendGpuModelRenderables(m_physGunVisual, weaponPosition,
                weaponOrientation,
                temporalHistoryValid ? m_previousPhysGunWeaponPosition
                    : weaponPosition,
                temporalHistoryValid ? m_previousPhysGunWeaponOrientation
                    : weaponOrientation,
                m_physGunViewCalibration.sizeMeters / largest,
                false, false, false,
                meshes);
        }
        // Mantém a pose anterior sincronizada mesmo no modo foto, evitando
        // um vetor de movimento gigante no quadro em que o modelo reaparece.
        renderedWeaponPosition = weaponPosition;
        renderedWeaponOrientation = weaponOrientation;
        renderedWeapon = true;

        const Vec3 beamStart = m_physGunBeamStartCameraLocal;
        m_physGunBeamStartWorldPosition = m_laboratoryCameraPosition
            + right * beamStart.x
            + forward * (beamStart.y - m_physGunRecoilOffset)
            + cameraUp * beamStart.z;

    }

    const float warmSun = 1.0f
        - smoothUnit(0.02f, 0.42f, sunElevation);
    std::array<LightRender3D, 3> lights;
    lights[0].type = LightType3D::Directional;
    lights[0].direction = sunDirection;
    lights[0].color = mixColor(
        { 1.0f, 0.50f, 0.22f }, { 1.0f, 0.96f, 0.88f }, 1.0f - warmSun);
    lights[0].intensity = 1.08f * directSun * cloudTransmission;
    lights[0].castsShadow = directSun > 0.001f;
    lights[1].type = LightType3D::Directional;
    lights[1].direction = moonDirection;
    lights[1].color = { 0.35f, 0.48f, 0.72f };
    lights[1].intensity = 0.080f * moonlight
        * (1.0f - 0.68f * cloudCoverage);
    lights[1].castsShadow = false;
    lights[2].type = LightType3D::Spot;
    lights[2].position = m_physGunBeamStartWorldPosition;
    lights[2].direction = cameraForward(m_laboratoryCameraYaw,
        m_laboratoryCameraPitch).normalized();
    lights[2].color = m_physGunFlashlight.color;
    lights[2].range = m_physGunFlashlight.enabled
        ? m_physGunFlashlight.rangeMeters : 0.0f;
    lights[2].intensity = m_physGunFlashlight.enabled
        ? m_physGunFlashlight.intensity : 0.0f;
    lights[2].coneOuterDegrees = m_physGunFlashlight.coneDegrees;

    Scene3DFrame scene;
    scene.renderMode = m_laboratoryRenderMode;
    scene.pixelArt = m_laboratoryPixelArt;
    scene.cameraViewProjection = m_laboratoryViewProjection;
    scene.cameraViewProjectionJittered = cameraViewProjectionJittered;
    scene.previousCameraViewProjection = temporalHistoryValid
        ? m_previousLaboratoryViewProjection
        : m_laboratoryViewProjection;
    scene.taaFrameIndex = m_taaFrameIndex;
    scene.temporalAntiAliasingEnabled =
        temporalResolveEnabled;
    // Mesmo sem TAA, o modo Pixel Art ainda conserva o histórico 1x1 da
    // exposição automática. Reiniciar o estado todo quadro deixaria a
    // paleta presa à exposição instantânea e faria a troca claro/escuro
    // divergir do modo Normal.
    scene.resetTemporalHistory = !m_laboratoryTaaHistoryValid;
    scene.cascadeViewProjections = cascadeViewProjections;
    scene.cascadeSplits = cascadeSplits;
    scene.cascadeTexelWorldSizes = cascadeTexelWorldSizes;
    scene.cascadeDepthRanges = cascadeDepthRanges;
    scene.cameraPosition = m_laboratoryCameraPosition;
    scene.lights = lights;
    scene.meshes = meshes;
    scene.environment = m_laboratoryEnvironment;
    scene.environment.skyAnimationTime = m_laboratoryElapsedTime;
    scene.environment.cloudWindOffset = m_cloudWindOffset;
    scene.showShadows = directSun > 0.001f;
    scene.shadowFilterSampleCount =
        m_laboratoryGraphicsPreset == 0 ? 4u
        : m_laboratoryGraphicsPreset == 1 ? 8u : 12u;
    scene.showSky = true;
    scene.toneMapping = m_laboratoryToneMapping;
    scene.ambientOcclusion = m_laboratoryAmbientOcclusion;
    scene.opticalEffects = m_laboratoryOpticalEffects;
    if (pixelArtMode) {
        // No Matter Mosaic, a grade já estabiliza bordas. O TAA permanece
        // desligado por padrão, mas pode ser habilitado experimentalmente
        // no painel Render. Glare e flare passam pela mesma grade artística
        // no tonemap, conservando o Sol ardido sem virar uma camada em alta
        // resolução sobre o Mosaic. Sombras usam o kernel mínimo.
        scene.ambientOcclusion.enabled = false;
        scene.shadowFilterSampleCount = 4u;
    }
    scene.fog = m_laboratoryFog;
    if (m_laboratoryOceanEnabled) {
        scene.oceanEnabled = true;
        scene.ocean.center = m_laboratoryOcean.center;
        scene.ocean.meanSeaLevelMeters =
            m_laboratoryOcean.meanSeaLevelMeters;
        scene.ocean.halfExtentMeters =
            m_laboratoryOcean.halfExtents.x;
        scene.ocean.vertexBuffer = m_laboratoryOceanVertexBuffer;
        scene.ocean.indexBuffer = m_laboratoryOceanIndexBuffer;
        scene.ocean.indexCount = m_laboratoryOceanIndexCount;

        const Vec2 relative {
            m_laboratoryCameraPosition.x - m_laboratoryOcean.center.x,
            m_laboratoryCameraPosition.y - m_laboratoryOcean.center.y
        };
        const bool insideDomain =
            std::abs(relative.x) <= m_laboratoryOcean.halfExtents.x
            && std::abs(relative.y) <= m_laboratoryOcean.halfExtents.y;
        const float surface = evaluateOceanSurface({
                m_laboratoryCameraPosition.x,
                m_laboratoryCameraPosition.y
            }, m_laboratoryOcean.meanSeaLevelMeters,
            m_laboratoryElapsedTime).heightMeters;
        scene.oceanSubmersion = insideDomain
            ? 1.0f - smoothUnit(surface - 0.12f, surface + 0.12f,
                m_laboratoryCameraPosition.z)
            : 0.0f;
    }
    // Os dois modos terminam diretamente no swapchain. O Mosaic agora
    // pixeliza a amostragem HDR numa grade artística dentro do tonemap, em
    // vez de reduzir todo o render 3D a uma textura minúscula. Além de
    // conservar detalhe seletivo, isto deixa o alvo offscreen exclusivamente
    // para previews persistentes (o atlas do menu Q), evitando que um
    // descriptor fosse destruído enquanto a UI ainda o utilizava.
    const bool sceneRendered = activeRenderer.renderScene3DToScreen(scene);
    if (sceneRendered) {
        // Estado temporal avanca na cadencia de RENDER, nao na cadencia fixa
        // da fisica. Em FPS maior que 120, repetir o mesmo delta de transform
        // em varios quadros geraria vetores de movimento falsos.
        for (SpawnedPropInstance& instance : m_spawnedProps) {
            // O que foi desenhado, nao o passo simulado: senao a propria
            // interpolacao viraria erro de reprojecao no TAA.
            instance.previousPhysicsState = instance.renderedState;
        }
        for (SpawnedRagdollInstance& instance : m_spawnedRagdolls) {
            instance.previousPhysicsState = instance.physicsState;
        }
        finishInflatedBallFrame();
        finishLaboratoryGoalFrame();
        if (renderedWeapon) {
            m_previousPhysGunWeaponPosition = renderedWeaponPosition;
            m_previousPhysGunWeaponOrientation = renderedWeaponOrientation;
        }
        m_previousLaboratoryViewProjection = m_laboratoryViewProjection;
        // Este sinal também marca que os históricos auxiliares (sobretudo
        // exposição automática) já receberam uma semente. O backend mantém
        // o histórico de TAA propriamente dito inválido quando TAA está
        // desligado, portanto não existe acumulação temporal no Mosaic.
        m_laboratoryTaaHistoryValid = true;
        if (temporalJitterEnabled) ++m_taaFrameIndex;
    }
    // No fim do quadro: a pose desenhada ja existe, e um spawn feito pelo
    // smoke nao altera vetores que este render ainda estivesse percorrendo.
    sampleRenderMotionSmoke(static_cast<float>(activeRenderer.width()),
        static_cast<float>(activeRenderer.height()));
    advanceRenderMotionSmoke();
}

void WorkbenchApp::drawSpawnMenu() {
    const ImGuiIO& io = ImGui::GetIO();
    // O escurecimento pertence ao fundo da interface: a cena fica atenuada,
    // enquanto a janela do menu permanece nítida e totalmente legível.
    ImDrawList* background = ImGui::GetBackgroundDrawList();
    background->AddRectFilled({ 0.0f, 0.0f }, io.DisplaySize,
        IM_COL32(2, 7, 13, 160));

    const float width = std::min(ui(870.0f), io.DisplaySize.x - ui(36.0f));
    const float height = std::min(ui(760.0f),
        io.DisplaySize.y - ui(44.0f));
    ImGui::SetNextWindowPos({ (io.DisplaySize.x - width) * 0.5f,
        (io.DisplaySize.y - height) * 0.5f }, ImGuiCond_Always);
    ImGui::SetNextWindowSize({ width, height }, ImGuiCond_Always);
    ImGui::Begin("SpawnMenu", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings);
    ImGui::SetWindowFontScale(1.18f);
    ImGui::TextUnformatted("OBJETOS");
    ImGui::SetWindowFontScale(1.0f);
    subtleSeparator();
    const auto& definitions = m_propCatalog.definitions();
    if (definitions.empty()) {
        ImGui::Dummy({ 1.0f, ui(28.0f) });
        centeredTextColored({ 0.55f, 0.68f, 0.78f, 1.0f },
            m_propAssetsLoadFailed
                ? "Falha ao carregar objetos" : "Carregando objetos");
        ImGui::End();
        return;
    }

    ImGui::Dummy({ 1.0f, ui(10.0f) });
    constexpr int Columns = 3;
    const float gap = ui(10.0f);
    const float contentWidth = ImGui::GetContentRegionAvail().x;
    const float cardWidth = (contentWidth
        - gap * static_cast<float>(Columns - 1))
        / static_cast<float>(Columns);
    const float cardHeight = ui(238.0f);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    for (std::size_t index = 0; index < definitions.size(); ++index) {
        if (index % Columns != 0) ImGui::SameLine(0.0f, gap);
        const PropDefinition3D& definition = definitions[index];
        ImGui::PushID(static_cast<int>(index));
        const ImVec2 cardMin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("PropCard", { cardWidth, cardHeight });
        const bool hovered = ImGui::IsItemHovered();
        const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
        const ImVec2 cardMax { cardMin.x + cardWidth,
            cardMin.y + cardHeight };
        drawList->AddRectFilled(cardMin, cardMax,
            hovered ? IM_COL32(15, 34, 49, 255)
                    : IM_COL32(10, 23, 34, 255), ui(7.0f));
        drawList->AddRect(cardMin, cardMax,
            hovered ? UiAccent : UiBorder, ui(7.0f), 0,
            hovered ? ui(1.5f) : ui(1.0f));

        const ImVec2 previewMin { cardMin.x + ui(5.0f),
            cardMin.y + ui(5.0f) };
        const ImVec2 previewMax { cardMax.x - ui(5.0f),
            cardMax.y - ui(39.0f) };
        if (m_propPreviewAtlas) {
            const float column = static_cast<float>(index % 3);
            const float row = static_cast<float>(index / 3);
            drawList->AddImage(
                static_cast<ImTextureID>(m_propPreviewAtlas.id),
                previewMin, previewMax,
                { column / 3.0f, row / 2.0f },
                { (column + 1.0f) / 3.0f,
                    (row + 1.0f) / 2.0f });
        }
        const ImVec2 nameSize = ImGui::CalcTextSize(
            definition.displayName.c_str());
        drawList->AddText({ cardMin.x
                + (cardWidth - nameSize.x) * 0.5f,
                cardMax.y - ui(27.0f) }, UiText,
            definition.displayName.c_str());

        if (hovered) {
            ImGui::BeginTooltip();
            ImGui::TextUnformatted(definition.displayName.c_str());
            subtleSeparator();
            ImGui::Text("Material: %s", definition.materialId.c_str());
            ImGui::Text("Massa: %.2f kg", definition.massKg);
            ImGui::Text("Dimensões: %.2f x %.2f x %.2f m",
                definition.dimensionsMeters.x,
                definition.dimensionsMeters.y,
                definition.dimensionsMeters.z);
            ImGui::EndTooltip();
        }
        if (clicked) spawnProp(index);
        ImGui::PopID();
    }

    const std::size_t ragdollCardIndex = definitions.size();
    if (ragdollCardIndex % Columns != 0) ImGui::SameLine(0.0f, gap);
    ImGui::PushID("HumanRagdollCard");
    const ImVec2 cardMin = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("RagdollCard", { cardWidth, cardHeight });
    const bool hovered = ImGui::IsItemHovered();
    const bool clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
    const bool available = m_humanRagdollProfile != nullptr
        && !m_ragdollAssetsLoadFailed;
    const ImVec2 cardMax {
        cardMin.x + cardWidth, cardMin.y + cardHeight
    };
    drawList->AddRectFilled(cardMin, cardMax,
        hovered ? IM_COL32(15, 34, 49, 255)
                : IM_COL32(10, 23, 34, 255), ui(7.0f));
    drawList->AddRect(cardMin, cardMax,
        hovered ? UiAccent : UiBorder, ui(7.0f), 0,
        hovered ? ui(1.5f) : ui(1.0f));

    const ImVec2 center {
        cardMin.x + cardWidth * 0.5f,
        cardMin.y + cardHeight * 0.43f
    };
    const ImU32 boneColor = available
        ? IM_COL32(115, 202, 235, 255)
        : IM_COL32(78, 99, 111, 255);
    if (m_characterThumbnail) {
        const float size=std::min(cardWidth-ui(12.0f),cardHeight-ui(36.0f));
        const ImVec2 corner{cardMin.x+(cardWidth-size)*0.5f,cardMin.y+ui(4.0f)};
        drawList->AddImage(static_cast<ImTextureID>(m_characterThumbnail.id),corner,
            {corner.x+size,corner.y+size});
    } else {
    const float stroke = ui(5.0f);
    const auto bone = [&](ImVec2 from, ImVec2 to) {
        drawList->AddLine(from, to, boneColor, stroke);
    };
    drawList->AddCircleFilled(
        { center.x, center.y - ui(69.0f) }, ui(13.0f), boneColor, 18);
    bone({ center.x, center.y - ui(52.0f) },
        { center.x, center.y + ui(23.0f) });
    bone({ center.x - ui(53.0f), center.y - ui(31.0f) },
        { center.x + ui(53.0f), center.y - ui(31.0f) });
    bone({ center.x - ui(53.0f), center.y - ui(31.0f) },
        { center.x - ui(78.0f), center.y + ui(7.0f) });
    bone({ center.x + ui(53.0f), center.y - ui(31.0f) },
        { center.x + ui(78.0f), center.y + ui(7.0f) });
    bone({ center.x, center.y + ui(23.0f) },
        { center.x - ui(28.0f), center.y + ui(88.0f) });
    bone({ center.x, center.y + ui(23.0f) },
        { center.x + ui(28.0f), center.y + ui(88.0f) });
    drawList->AddCircleFilled(
        { center.x, center.y + ui(23.0f) }, ui(7.0f), boneColor, 12);
    }

    const char* RagdollName = m_ragdollCharacter
        ? m_ragdollCharacter->displayName.c_str() : "Ragdoll humano";
    const ImVec2 nameSize = ImGui::CalcTextSize(RagdollName);
    drawList->AddText(
        { cardMin.x + (cardWidth - nameSize.x) * 0.5f,
            cardMax.y - ui(27.0f) },
        available ? UiText : IM_COL32(112, 130, 141, 255),
        RagdollName);
    if (hovered) {
        ImGui::BeginTooltip();
        ImGui::TextUnformatted(RagdollName);
        subtleSeparator();
        ImGui::TextUnformatted("PhysX Reduced Coordinate Articulation");
        ImGui::TextUnformatted("18 segmentos | 41 DOFs | 75 kg");
        ImGui::Text("Rigidez inicial: %.0f%%", m_ragdollRigidityPercent);
        if (!available) {
            ImGui::TextColored({ 1.0f, 0.55f, 0.48f, 1.0f },
                "Perfil indisponivel");
        }
        ImGui::EndTooltip();
    }
    if (clicked && available) spawnHumanRagdoll();
    ImGui::PopID();

    ImGui::End();
}

void WorkbenchApp::drawLaboratoryPauseMenu() {
    const ImGuiIO& io = ImGui::GetIO();
    // Assim como no menu de objetos, o overlay não pode cobrir o próprio
    // diálogo. A camada de fundo preserva a hierarquia visual correta.
    ImDrawList* background = ImGui::GetBackgroundDrawList();
    background->AddRectFilled({ 0.0f, 0.0f }, io.DisplaySize,
        IM_COL32(1, 5, 10, 205));

    const float width = ui(340.0f);
    ImGui::SetNextWindowPos({ (io.DisplaySize.x - width) * 0.5f,
        io.DisplaySize.y * 0.22f }, ImGuiCond_Always);
    ImGui::SetNextWindowSize({ width, 0.0f }, ImGuiCond_Always);
    ImGui::Begin("LaboratoryPause", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SetWindowFontScale(1.30f);
    ImGui::TextUnformatted("LABORATÓRIO");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Dummy({ 1.0f, ui(12.0f) });
    const ImVec2 buttonSize { -1.0f, ui(42.0f) };
    if (ImGui::Button("Continuar", buttonSize)) {
        m_laboratoryPaused = false;
        m_laboratoryConfirmQuit = false;
        syncLaboratoryMouseCapture();
    }
    if (ImGui::Button("Configurações", buttonSize)) {
        m_settingsReturnScreen = Screen::Laboratory;
        m_screen = Screen::Settings;
    }
    if (ImGui::Button("Menu principal", buttonSize)) {
        m_laboratoryPaused = false;
        m_laboratoryConfirmQuit = false;
        m_screen = Screen::MainMenu;
        renderer().setMouseCaptured(false);
    }
    if (!m_laboratoryConfirmQuit) {
        if (ImGui::Button("Sair", buttonSize)) {
            m_laboratoryConfirmQuit = true;
        }
    } else {
        ImGui::TextColored({ 1.0f, 0.55f, 0.48f, 1.0f },
            "Encerrar o MatterEngine?");
        if (ImGui::Button("Confirmar", { ui(145.0f), ui(38.0f) })) {
            requestQuit();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancelar", { ui(145.0f), ui(38.0f) })) {
            m_laboratoryConfirmQuit = false;
        }
    }
    ImGui::End();
}

void WorkbenchApp::drawLaboratoryUi() {
    const ImGuiIO& io = ImGui::GetIO();
    ImDrawList* foreground = ImGui::GetForegroundDrawList();
    ImDrawList* background = ImGui::GetBackgroundDrawList();

    char headerStatus[32] {};
    if (m_showPerformanceOverlay) {
        std::snprintf(headerStatus, sizeof(headerStatus), "%.0f FPS",
            io.Framerate);
    }
    // Desenhado no background draw list (nao no foreground usado pelo resto
    // desta funcao) de proposito: o foreground sempre renderiza por cima de
    // TODAS as janelas ImGui, entao o retangulo opaco do cabecalho cobriria
    // os botoes da barra de depuracao (ver mais abaixo) mesmo que eles
    // sejam desenhados depois - o unico jeito da barra ficar por cima do
    // cabecalho e o cabecalho nao estar no foreground. Isso nao muda nada
    // visualmente pro resto da cena 3D (ja renderizada via Vulkan antes de
    // qualquer coisa do ImGui, background inclusive).
    drawWorkbenchHeader(background, io.DisplaySize, m_uiScale,
        "LABORATÓRIO", headerStatus);


    // A bola escondida atras do proprio corpo (25/10, pedidos do usuario): o
    // contorno branco quando o corpo do jogador controlado fica entre a camera e
    // ela (os raios da camera batem nele) - o circulo inteiro com o centro dela
    // coberto, ou so os trechos cobertos do contorno; e o ponto no centro,
    // sempre (opcional). As opacidades do painel; entram e saem suaves.
    if (m_physicsScene) {
        const auto controlled = std::find_if(m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
            [&](const SpawnedRagdollInstance& item) { return item.entityId == m_controlledCharacterEntityId; });
        const Vec3 eye = m_laboratoryCameraPosition;
        const auto covered = [&](Vec3 point, float shorten) {
            const Vec3 ray = point - eye;
            const float length = ray.length();
            PhysicsRagdollRayHit3D hit;
            return controlled != m_spawnedRagdolls.end() && length > shorten + 1e-3f
                && m_physicsScene->raycastRagdoll({ eye, ray * (1.0f / length) }, length - shorten, hit)
                && hit.ragdoll == controlled->physicsRagdoll;
        };
        constexpr int Points = 32;
        for (SpawnedInflatedBallInstance& ball : m_inflatedBalls) {
            const Vec3 toBall = ball.renderedPosition - eye;
            const float distance = toBall.length();
            const float radius = m_physicsScene->contains(ball.physicsBall)
                ? m_physicsScene->inflatedBallProfile(ball.physicsBall).radiusMeters : 0.11f;
            const bool near = controlled != m_spawnedRagdolls.end() && distance > radius && distance < 25.0f;
            const bool hidden = near && covered(ball.renderedPosition, radius);
            // O contorno da esfera na tela: o circulo perpendicular a visada.
            Vec3 across = cross(toBall, Vec3 { 0.0f, 0.0f, 1.0f });
            if (across.lengthSquared() < 1e-8f) across = Vec3 { 1.0f, 0.0f, 0.0f };
            across = across.normalized();
            const Vec3 upward = cross(across, toBall).normalized();
            std::array<Vec3, Points> rim {};
            std::array<bool, Points> rimCovered {};
            bool anyCovered = false;
            for (int i = 0; i < Points; ++i) {
                const float angle = 2.0f * Pi * static_cast<float>(i) / Points;
                rim[i] = ball.renderedPosition + (across * std::cos(angle) + upward * std::sin(angle)) * radius;
                rimCovered[i] = near && m_hiddenBallCoveredOnly && covered(rim[i], 0.005f);
                anyCovered = anyCovered || rimCovered[i];
            }
            const bool showOutline = m_hiddenBallCoveredOnly ? anyCovered : hidden;
            ball.hiddenOutline = std::clamp(ball.hiddenOutline + (showOutline ? io.DeltaTime / 0.12f : -io.DeltaTime / 0.2f),
                0.0f, 1.0f);
            ball.centerDot = std::clamp(ball.centerDot + (m_ballDot ? io.DeltaTime / 0.12f : -io.DeltaTime / 0.2f),
                0.0f, 1.0f);
            ImVec2 center, edge;
            if (!projectToScreen(m_laboratoryViewProjection, ball.renderedPosition, io.DisplaySize, center)
                || !projectToScreen(m_laboratoryViewProjection, ball.renderedPosition + across * radius, io.DisplaySize, edge))
                continue;
            const float screenRadius = std::hypot(edge.x - center.x, edge.y - center.y);
            const float opacity = std::clamp(m_hiddenBallOpacity, 0.0f, 1.0f);
            if (ball.hiddenOutline > 0.01f) {
                const ImU32 color = IM_COL32(255, 255, 255, static_cast<int>(255.0f * opacity * ball.hiddenOutline));
                if (!m_hiddenBallCoveredOnly) {
                    foreground->AddCircle(center, screenRadius + ui(1.0f), color, 48, ui(1.5f));
                } else {
                    // So os trechos com as duas pontas cobertas.
                    std::array<ImVec2, Points> screen {};
                    std::array<bool, Points> visible {};
                    for (int i = 0; i < Points; ++i)
                        visible[i] = projectToScreen(m_laboratoryViewProjection, rim[i], io.DisplaySize, screen[i]);
                    for (int i = 0; i < Points; ++i) {
                        const int j = (i + 1) % Points;
                        if (rimCovered[i] && rimCovered[j] && visible[i] && visible[j])
                            foreground->AddLine(screen[i], screen[j], color, ui(1.5f));
                    }
                }
            }
            if (ball.centerDot > 0.01f) {
                // Bem pequeno de perto, maior de longe; nunca maior que a bola na tela.
                const float dot = std::clamp(ui(0.35f) * distance, ui(1.0f), std::max(ui(1.0f), screenRadius));
                foreground->AddCircleFilled(center, dot,
                    IM_COL32(255, 255, 255, static_cast<int>(255.0f * std::clamp(m_ballDotOpacity, 0.0f, 1.0f)
                        * ball.centerDot)), 16);
            }
        }
    }

    // As areas de alcance (menu do jogador, 24/10): as caixas em que o centro
    // da bola precisa estar, no referencial da escolha do gesto (a pelve no
    // chao, o rumo do corpo) - os pes (e, acima de highBall, o gesto alto), o
    // peito e a cabeca (pela altura da pelve: sobem com o pulo).
    if (m_showReachAreas && m_reachValid && m_controlledCharacterEntityId != 0) {
        // O alcance do corpo (o mesmo do runtime: os pes x s, o peito pelo
        // tronco, a cabeca so na posicao).
        const PlayerBody3D body = laboratoryBody();
        const GestureReach3D reach = gestureReachForBody3D(athleteRuntimeSettings().gestureReach, bodyScale3D(body),
            bodyGirth3D(PhysiqueRegion3D::Chest, bodyTone3D(body)) - 1.0f);
        const Vec3 forward { std::cos(m_reachYaw), std::sin(m_reachYaw), 0.0f };
        const Vec3 left { -forward.y, forward.x, 0.0f };
        const auto box = [&](float x0, float x1, float y0, float y1, float z0, float z1, ImU32 color) {
            ImVec2 corner[8];
            bool visible[8];
            for (int i = 0; i < 8; ++i) {
                const Vec3 point = m_reachOrigin + forward * ((i & 1) ? x1 : x0) + left * ((i & 2) ? y1 : y0)
                    + Vec3 { 0.0f, 0.0f, (i & 4) ? z1 : z0 };
                visible[i] = projectToScreen(m_laboratoryViewProjection, point, io.DisplaySize, corner[i]);
            }
            for (int i = 0; i < 8; ++i)
                for (const int bit : { 1, 2, 4 })
                    if ((i & bit) == 0 && visible[i] && visible[i | bit])
                        foreground->AddLine(corner[i], corner[i | bit], color, ui(1.5f));
        };
        box(-reach.back, reach.front, -reach.side, reach.side, -reach.below, reach.highBall, IM_COL32(90, 220, 120, 200));
        box(-reach.back, reach.front, -reach.side, reach.side, reach.highBall, reach.height, IM_COL32(230, 220, 110, 200));
        const float hip = m_reachPelvisHeight;
        box(-reach.bodyBack, reach.bodyFront, -reach.bodySide, reach.bodySide, hip + reach.chestBottom, hip + reach.chestTop,
            IM_COL32(90, 160, 240, 200));
        box(-reach.headBack, reach.headFront, -reach.headSide, reach.headSide, hip + reach.chestTop, hip + reach.headTop,
            IM_COL32(245, 150, 60, 200));
    }

    // Barra de forca do manejo de bola: centro, embaixo; vermelha no chute,
    // azul no drible, amarela no levantar. Aparece carregando e um instante
    // depois de soltar.
    if (m_controlledCharacterEntityId != 0) {
        m_ballBarSeconds = std::max(0.0f, m_ballBarSeconds - io.DeltaTime);
        const bool charging = m_ballShotHeld || m_ballDribbleHeld || m_ballRaiseHeld;
        const bool shotBar = m_ballBarKind == BallActionKind3D::Shot;
        if (charging || m_ballBarSeconds > 0.0f) {
            const float width = ui(360.0f), height = ui(14.0f);
            const ImVec2 from { io.DisplaySize.x * 0.5f - width * 0.5f, io.DisplaySize.y - ui(58.0f) };
            const ImVec2 to { from.x + width, from.y + height };
            const float alpha = charging ? 1.0f : std::clamp(m_ballBarSeconds / 0.3f, 0.0f, 1.0f);
            const auto a = [&](int value) { return static_cast<int>(value * alpha); };
            foreground->AddRectFilled({ from.x - ui(3.0f), from.y - ui(3.0f) }, { to.x + ui(3.0f), to.y + ui(3.0f) },
                IM_COL32(10, 14, 20, a(190)), ui(4.0f));
            const float level = charging ? m_ballCharge : m_ballRequestPower;
            const ImU32 fill = shotBar ? IM_COL32(222, 52, 46, a(240))
                : m_ballBarKind == BallActionKind3D::Raise ? IM_COL32(240, 200, 40, a(240)) : IM_COL32(52, 128, 232, a(240));
            if (level > 0.001f)
                foreground->AddRectFilled(from, { from.x + width * level, to.y }, fill, ui(3.0f));
            foreground->AddRect(from, to, IM_COL32(220, 230, 240, a(150)), ui(3.0f));
            const char* label = shotBar ? "CHUTE" : m_ballBarKind == BallActionKind3D::Raise ? "LEVANTAR" : "DRIBLE";
            foreground->AddText({ from.x, from.y - ui(20.0f) }, IM_COL32(230, 236, 244, a(220)), label);
            // Chute: o gesto (quadradinho a direita) - o ponto sobe para bola
            // alta, desce para rasteira, vai ao lado do efeito.
            if (shotBar) {
                const float lift = charging ? m_shotLiftShown : m_ballRequestLift;
                const float curve = charging ? m_shotCurveShown : m_ballRequestCurve;
                const float half = ui(17.0f);
                const ImVec2 middle { to.x + ui(14.0f) + half, (from.y + to.y) * 0.5f - ui(6.0f) };
                foreground->AddRectFilled({ middle.x - half, middle.y - half }, { middle.x + half, middle.y + half },
                    IM_COL32(10, 14, 20, a(190)), ui(4.0f));
                foreground->AddLine({ middle.x - half, middle.y }, { middle.x + half, middle.y }, IM_COL32(220, 230, 240, a(60)));
                foreground->AddLine({ middle.x, middle.y - half }, { middle.x, middle.y + half }, IM_COL32(220, 230, 240, a(60)));
                foreground->AddCircleFilled({ middle.x + curve * (half - ui(4.0f)), middle.y - lift * (half - ui(4.0f)) },
                    ui(3.5f), IM_COL32(250, 210, 80, a(240)));
            }
        }
    }

    const ImVec2 center { io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f };
    if (!m_hidePhysGunPresentation) {
        foreground->AddLine({ center.x - ui(5.0f), center.y },
            { center.x + ui(5.0f), center.y },
            IM_COL32(190, 222, 245, 205));
        foreground->AddLine({ center.x, center.y - ui(5.0f) },
            { center.x, center.y + ui(5.0f) },
            IM_COL32(190, 222, 245, 205));
    }

    if (m_physGunTriggerHeld) {
        ImVec2 beamStartScreen;
        ImVec2 beamEndScreen = center;
        if (projectToScreen(m_laboratoryViewProjection,
                m_physGunBeamStartWorldPosition, io.DisplaySize,
                beamStartScreen)) {
            if (m_physGunGrabbedEntityId != 0) {
                ImVec2 heldPointScreen;
                if (projectToScreen(m_laboratoryViewProjection,
                        m_physGunBeamTargetWorldPosition,
                        io.DisplaySize, heldPointScreen)) {
                    beamEndScreen = heldPointScreen;
                }
            }
            // Overlay deliberado: o feixe pertence à apresentação da arma e
            // permanece legível independentemente da profundidade da cena.
            foreground->AddLine(beamStartScreen, beamEndScreen,
                IM_COL32(0, 150, 255, 36), ui(8.5f));
            foreground->AddLine(beamStartScreen, beamEndScreen,
                IM_COL32(0, 206, 255, 112), ui(3.8f));
            foreground->AddLine(beamStartScreen, beamEndScreen,
                IM_COL32(184, 249, 255, 250), ui(1.3f));
            foreground->AddCircleFilled(beamStartScreen, ui(4.0f),
                IM_COL32(38, 211, 255, 190));
            foreground->AddCircleFilled(beamEndScreen, ui(2.8f),
                IM_COL32(195, 250, 255, 225));
        }
    }

    if (!m_laboratoryStatus.empty()) {
        const ImVec2 size = ImGui::CalcTextSize(m_laboratoryStatus.c_str());
        foreground->AddText({ (io.DisplaySize.x - size.x) * 0.5f,
            io.DisplaySize.y - ui(44.0f) }, UiMuted,
            m_laboratoryStatus.c_str());
    }

    if (m_notification.secondsRemaining > 0.0f
        && !m_notification.text.empty()) {
        const float elapsed = m_notification.durationSeconds
            - m_notification.secondsRemaining;
        const float alpha = std::clamp(std::min(
            elapsed / 0.16f,
            m_notification.secondsRemaining / 0.42f), 0.0f, 1.0f);
        const ImVec2 textSize = ImGui::CalcTextSize(
            m_notification.text.c_str());
        const ImVec2 minimum { ui(18.0f),
            io.DisplaySize.y - ui(36.0f) - textSize.y };
        const ImVec2 maximum { minimum.x + textSize.x + ui(38.0f),
            minimum.y + textSize.y + ui(20.0f) };
        foreground->AddRectFilled(minimum, maximum,
            IM_COL32(5, 16, 25, static_cast<int>(218.0f * alpha)),
            ui(6.0f));
        foreground->AddRectFilled(minimum,
            { minimum.x + ui(3.0f), maximum.y },
            IM_COL32(17, 166, 255, static_cast<int>(240.0f * alpha)),
            ui(6.0f));
        foreground->AddText({ minimum.x + ui(18.0f),
            minimum.y + ui(10.0f) },
            IM_COL32(205, 229, 244, static_cast<int>(255.0f * alpha)),
            m_notification.text.c_str());
    }

    if (m_physicsBenchmarkShowAnalytics
        && (!m_physicsBenchmarkPropEntityIds.empty()
            || !m_physicsBenchmarkRagdollEntityIds.empty()
            || m_physicsBenchmarkRunning)) {
        const float analyticsWidth =
            std::min(ui(405.0f), io.DisplaySize.x - ui(24.0f));
        ImGui::SetNextWindowPos({
            io.DisplaySize.x - analyticsWidth - ui(12.0f),
            headerHeight(m_uiScale) + ui(10.0f)
        }, ImGuiCond_Always);
        ImGui::SetNextWindowSize(
            { analyticsWidth, ui(342.0f) }, ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.91f);
        ImGui::Begin("PhysicsBenchmarkAnalytics", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
                | ImGuiWindowFlags_NoSavedSettings
                | ImGuiWindowFlags_NoInputs
                | ImGuiWindowFlags_NoFocusOnAppearing);
        panelHeader("BENCHMARK FÍSICO — ANALÍTICOS");
        ImGui::Text("%.0f FPS  |  %zu props  |  %zu ragdolls",
            io.Framerate,
            m_physicsBenchmarkPropEntityIds.size(),
            m_physicsBenchmarkRagdollEntityIds.size());
        const float overallAverage =
            m_physicsBenchmarkOverallFrameSamples > 0
            ? m_physicsBenchmarkOverallFpsSum
                / static_cast<float>(
                    m_physicsBenchmarkOverallFrameSamples)
            : 0.0f;
        ImGui::Text(
            "Antes %.0f  |  média %.0f  |  mínimo %.0f FPS",
            m_physicsBenchmarkBaselineFps, overallAverage,
            m_physicsBenchmarkOverallMinimumFps);
        if (m_physicsScene) {
            const PhysicsStepDiagnostics3D& physics =
                m_physicsScene->diagnostics();
            ImGui::Text("PhysX %.3f ms  |  %zu ativos  |  %zu contatos",
                physics.totalStepMilliseconds,
                physics.activeDynamicBodyCount,
                physics.discreteContactPairs);
            ImGui::Text("Dormindo %zu  |  CCD %zu  |  workers %u",
                physics.sleepingDynamicBodyCount, physics.ccdPairs,
                physics.physicsWorkerCount);
        }
        ImGui::Dummy({ 1.0f, ui(5.0f) });
        std::vector<float> fpsHistory;
        fpsHistory.reserve(m_physicsBenchmarkSamples.size());
        for (const PhysicsBenchmarkSample& sample :
                m_physicsBenchmarkSamples) {
            fpsHistory.push_back(sample.averageFps);
        }
        if (fpsHistory.empty()) {
            ImGui::TextColored(
                { 0.55f, 0.68f, 0.78f, 1.0f },
                "Primeiro ponto em %.0f s",
                std::max(0.0f,
                    5.0f - m_physicsBenchmarkSampleSeconds));
            const float liveFps = io.Framerate;
            ImGui::PlotLines("##BenchmarkFps", &liveFps, 1, 0,
                "FPS médio / janela de 5 s", 0.0f,
                std::max(120.0f, liveFps * 1.15f),
                { -1.0f, ui(92.0f) });
        } else {
            const float maximum = *std::max_element(
                fpsHistory.begin(), fpsHistory.end());
            ImGui::PlotLines("##BenchmarkFps", fpsHistory.data(),
                static_cast<int>(fpsHistory.size()), 0,
                "FPS médio / janela de 5 s", 0.0f,
                std::max(120.0f, maximum * 1.15f),
                { -1.0f, ui(92.0f) });
            const PhysicsBenchmarkSample& last =
                m_physicsBenchmarkSamples.back();
            ImGui::Text(
                "Última janela: %.0f médio / %.0f mínimo",
                last.averageFps, last.minimumFps);
            ImGui::Text(
                "%u props + %u ragdolls  |  PhysX %.3f ms (pico %.3f)",
                last.propCount, last.ragdollCount,
                last.averagePhysicsMilliseconds,
                last.maximumPhysicsMilliseconds);
        }
        ImGui::End();
    }

    if (m_laboratoryDebugVisible) {
        // Botoes na MESMA linha do cabecalho "MATTERENGINE | LABORATÓRIO"
        // (nao uma segunda barra abaixo dele) - cada um abre/fecha seu
        // proprio painel flutuante e independente (ver beginDebugPanel), em
        // vez da unica janela com abas de antes. A posicao X replica a
        // mesma matematica que drawWorkbenchHeader (UiKit.cpp) usa pra
        // desenhar "LABORATÓRIO", entao os botoes sempre nascem logo depois
        // desse texto, nunca por cima dele - sem precisar alterar aquele
        // componente, que tambem e usado por outras telas.
        const ImVec2 brandSize = ImGui::CalcTextSize("MATTERENGINE");
        const ImVec2 contextSize = ImGui::CalcTextSize("LABORATÓRIO");
        const float toolbarStartX = ui(76.0f) + brandSize.x + contextSize.x
            + ui(28.0f);
        const float toolbarButtonHeight = ui(30.0f);
        const float toolbarY =
            (headerHeight(m_uiScale) - toolbarButtonHeight) * 0.5f;
        // ImGuiWindowFlags_NoDecoration NAO remove o WindowPadding padrao
        // do ImGui (8,8 nao escalado) - sem empurrar isso pra zero, a
        // janela reservava altura so pro botao (toolbarButtonHeight+4) e o
        // padding de cima/baixo cortava a base de cada botao. Mesmo
        // mecanismo ja usado em SettingsScreen.cpp (drawVideoConfirmPopup).
        const ImVec2 toolbarPadding { ui(4.0f), ui(2.0f) };
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, toolbarPadding);

        ImGui::SetNextWindowPos({ toolbarStartX, toolbarY }, ImGuiCond_Always);
        ImGui::SetNextWindowSize(
            { io.DisplaySize.x - toolbarStartX - ui(90.0f),
                toolbarButtonHeight + toolbarPadding.y * 2.0f },
            ImGuiCond_Always);
        ImGui::Begin("LaboratoryDebugToolbar", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove
                | ImGuiWindowFlags_NoSavedSettings
                | ImGuiWindowFlags_NoFocusOnAppearing
                | ImGuiWindowFlags_NoBackground);
        // A barra precisa acomodar também o benchmark em telas de 1366 px.
        // 130 px por aba fazia o último botão existir, mas ficar recortado
        // fora da janela transparente da toolbar.
        const float toolbarGap = ImGui::GetStyle().ItemSpacing.x;
        const float benchmarkButtonWidth = ui(140.0f);
        const float regularButtonWidth = std::clamp(
            (ImGui::GetContentRegionAvail().x - benchmarkButtonWidth
                - toolbarGap * 8.0f) / 8.0f,
            ui(82.0f), ui(110.0f));
        const ImVec2 toolbarButtonSize {
            regularButtonWidth, toolbarButtonHeight
        };
        if (tabButton("RENDER", m_showRenderPanel, toolbarButtonSize)) {
            m_showRenderPanel = !m_showRenderPanel;
        }
        ImGui::SameLine();
        if (tabButton("Physgun", m_showPhysgunPanel, toolbarButtonSize)) {
            m_showPhysgunPanel = !m_showPhysgunPanel;
        }
        ImGui::SameLine();
        if (tabButton("RAGDOLL", m_showRagdollPanel, toolbarButtonSize)) {
            m_showRagdollPanel = !m_showRagdollPanel;
        }
        ImGui::SameLine();
        if (tabButton("PERSONAGEM", m_showCharacterPanel,
                toolbarButtonSize)) {
            m_showCharacterPanel = !m_showCharacterPanel;
        }
        ImGui::SameLine();
        if (tabButton("BENCHMARK FÍSICO",
                m_showPhysicsBenchmarkPanel,
                { benchmarkButtonWidth, toolbarButtonHeight })) {
            m_showPhysicsBenchmarkPanel =
                !m_showPhysicsBenchmarkPanel;
        }
        ImGui::SameLine();
        if (tabButton("Gráficos", m_showGraphicsPanel, toolbarButtonSize)) {
            m_showGraphicsPanel = !m_showGraphicsPanel;
        }
        ImGui::SameLine();
        if (tabButton("TEMPO", m_showTimePanel, toolbarButtonSize)) {
            m_showTimePanel = !m_showTimePanel;
        }
        ImGui::SameLine();
        if (tabButton("Performance", m_showPerformancePanel,
                toolbarButtonSize)) {
            m_showPerformancePanel = !m_showPerformancePanel;
        }
        ImGui::SameLine();
        if (tabButton("Vento", m_showWindPanel, toolbarButtonSize)) {
            m_showWindPanel = !m_showWindPanel;
        }
        ImGui::End();
        ImGui::PopStyleVar();


        const float panelWidth = ui(390.0f);
        const float panelTop = headerHeight(m_uiScale) + ui(12.0f);
        if (m_showCharacterPanel) {
            const bool drawCharacterPanel = beginDebugPanel("Personagem",
                &m_showCharacterPanel,
                { ui(12.0f), panelTop },
                { panelWidth, ui(560.0f) }, io.DisplaySize, m_uiScale);
            if (drawCharacterPanel) {
                panelHeader("CONTROLE EM TERCEIRA PESSOA");
                const auto controlled = std::find_if(
                    m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
                    [&](const SpawnedRagdollInstance& instance) {
                        return instance.entityId
                            == m_controlledCharacterEntityId;
                    });
                const bool isControlling = controlled
                    != m_spawnedRagdolls.end();
                const bool hasCandidate = std::any_of(
                    m_spawnedRagdolls.begin(), m_spawnedRagdolls.end(),
                    [](const SpawnedRagdollInstance& instance) {
                        return instance.kind == ArticulatedRigKind::Human
                            && instance.active
                            && !instance.physicsState.links.empty();
                    });
                ImGui::TextColored(isControlling
                        ? ImVec4 { 0.38f, 0.82f, 0.62f, 1.0f }
                        : ImVec4 { 0.72f, 0.76f, 0.82f, 1.0f },
                    "%s", isControlling
                        ? "CONTROLE ATIVO" : "CONTROLE LIVRE");
                ImGui::Dummy({ 1.0f, ui(8.0f) });
                if (!isControlling && !hasCandidate) ImGui::BeginDisabled();
                if (ImGui::Button(isControlling
                        ? "LIBERAR CONTROLE" : "ASSUMIR ÚLTIMO PERSONAGEM",
                        { -1.0f, ui(42.0f) })) {
                    if (isControlling) releaseControlledCharacter();
                    else takeControlOfLatestCharacter();
                }
                if (!isControlling && !hasCandidate) ImGui::EndDisabled();
                if (!hasCandidate) {
                    ImGui::TextWrapped(
                        "Crie um Crash Test Dummy ativo pelo menu de spawn.");
                }
                // So o que ainda se ajusta (pedido do usuario, 05/10 noite: o
                // resto ja esta consolidado e fica nos padroes); explicacoes
                // so ao passar o mouse.
                const auto hint = [](const char* text) {
                    if (!ImGui::IsItemHovered()) return;
                    ImGui::BeginTooltip();
                    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);
                    ImGui::TextUnformatted(text);
                    ImGui::PopTextWrapPos();
                    ImGui::EndTooltip();
                };
                const auto percent = [&](const char* label, const char* id, float* value,
                        float minimum, float maximum, const char* text) {
                    ImGui::TextUnformatted(label);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat(id, value, minimum, maximum, "%.0f%%", ImGuiSliderFlags_AlwaysClamp);
                    hint(text);
                };
                // Abas (06/10): o geral, a forca na bola (07/10) e as velocidades do
                // modo padrao.
                if (ImGui::BeginTabBar("##CharacterTabs")) {
                    if (ImGui::BeginTabItem("Geral")) {
                        panelHeader("LOCOMOÇÃO");
                        {
                            // Os tres tipos (17/10, pedido do usuario): as caixas de
                            // trocas de modo, velocidades do padrao no fisico e rede
                            // sairam - as duas primeiras ficam ligadas, a rede e o
                            // terceiro tipo.
                            const char* modes[] = { "Padrão", "Físico total", "Físico com RL" };
                            int mode = m_defaultLocomotion ? 0 : m_locomotionPolicyOn ? 2 : 1;
                            ImGui::SetNextItemWidth(-1.0f);
                            if (ImGui::Combo("##LocomotionMode", &mode, modes, 3)) {
                                m_defaultLocomotion = mode == 0;
                                m_locomotionPolicyOn = mode == 2;
                                if (m_locomotionPolicyOn && m_athleteRuntime.locomotionPolicy() == nullptr)
                                    reloadLocomotionPolicy();
                            }
                            hint("Padrão: o corpo segue a animação (com massa) e vai ao físico numa pancada, parede ou queda, voltando quando assenta. Físico total: sempre pelos músculos e pelas ajudas. Físico com RL: o físico com a rede treinada corrigindo os pés, o equilíbrio, a pose e a rigidez.");
                            // (A locomocao fisica v2 e o "tentar se manter em pe no
                            // impacto" viraram o padrao - as caixas sairam, pedido do
                            // usuario, 06/10.)
                            // (As pegadas sairam de vez: pedido do usuario, 11/10.)
                            // A guiagem de ossos (experimento do usuario, 11/10): so no
                            // Fisico total (com a rede ela fica em zero, como no treino).
                            if (mode == 1)
                                percent("Guiagem de ossos", "##BoneGuide", &m_physicalBoneGuidePercent, 0.0f, 100.0f,
                                    "Os ossos vão até a pose do modo padrão nesta fração: 0% é a locomoção física de sempre, 100% é como o padrão.");
                            if (mode == 1) {
                                ImGui::Checkbox("Virada brusca (experimento)", &m_dynamicTurnMove);
                                hint("Ao mudar bruscamente de direção em movimento, o corpo faz uma manobra: inclina para dentro da curva, abaixa, encurta a passada, planta o pé de fora e corta. Só no Físico total; sem força de fora.");
                                if (m_dynamicTurnMove)
                                    percent("Intensidade da virada", "##TurnMoveAmount", &m_dynamicTurnMovePercent, 0.0f, 150.0f,
                                        "Quanto da manobra entra (100% é o calculado).");
                            }
                            ImGui::Checkbox("Pulo instantâneo", &m_instantJump);
                            hint("Ligado: pula ao apertar o espaço, na altura média. Desligado: segurar carrega e pula ao soltar (mais alto quanto mais segura).");
                            ImGui::Checkbox("Áreas de alcance", &m_showReachAreas);
                            hint("Desenha onde a bola precisa estar para cada parte: verde os pés (o amarelo mais claro é a bola alta, pé na altura do joelho), azul o peito, laranja a cabeça. É o centro da bola que conta.");
                            if (mode == 2) {
                                if (ImGui::Button("Recarregar a rede", { -1.0f, 0.0f })) reloadLocomotionPolicy();
                                hint("Carrega a última rede que o treino exportou.");
                                if (!m_locomotionPolicyLabel.empty()) ImGui::TextDisabled("%s", m_locomotionPolicyLabel.c_str());
                            }
                        }
                        panelHeader("AJUDAS");
                        {
                            // A ajuda auxiliar (17/10): uma regua para as quatro; no
                            // Avancado, cada uma. 0% e nenhuma forca de fora.
                            ImGui::Checkbox("Avançado", &m_assistAdvanced);
                            hint("Ajustar cada ajuda separada (parado, andando, nos pés, ao levantar).");
                            if (!m_assistAdvanced) {
                                float all = m_e5WalkingAssistPercent;
                                ImGui::TextUnformatted("Ajuda auxiliar");
                                ImGui::SetNextItemWidth(-1.0f);
                                if (ImGui::SliderFloat("##AssistAll", &all, 0.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp))
                                    m_e5StandingAssistPercent = m_e5WalkingAssistPercent = m_physicalFeetAssistPercent
                                        = m_getUpAssistPercent = all;
                                hint("Forças de fora no corpo físico (parado, andando, nos pés e ao levantar). 0%: nenhuma, só os músculos; 100%: a ajuda cheia.");
                            } else {
                                percent("Ajuda parado", "##AssistStanding", &m_e5StandingAssistPercent, 0.0f, 100.0f,
                                    "Força de fora na pelve parado (no físico). 0%: nenhuma.");
                                percent("Ajuda andando", "##AssistWalking", &m_e5WalkingAssistPercent, 0.0f, 100.0f,
                                    "Força de fora andando e correndo (no físico). 0%: nenhuma.");
                                percent("Ajuda nos pés", "##AssistFeet", &m_physicalFeetAssistPercent, 0.0f, 100.0f,
                                    "No físico, a força que leva o corpo ao movimento passa pelos pés apoiados. 0%: nenhuma.");
                                percent("Ajuda ao levantar", "##AssistGetUp", &m_getUpAssistPercent, 0.0f, 100.0f,
                                    "Forças de fora no levantar do chão. 0%: só os músculos.");
                            }
                        }
                        panelHeader("MÚSCULOS");
                        percent("Rigidez muscular geral", "##MuscleStiffness", &m_characterMusclePercent, 0.0f, 100.0f,
                            "Rigidez e força de todos os músculos no físico (100% é a calibrada).");
                        percent("Rigidez ao levantar", "##GetUpStiffness", &m_getUpStiffnessPercent, 20.0f, 150.0f,
                            "Rigidez dos músculos enquanto ele levanta; menos, mais relaxado.");
                        // (24/10, pedido do usuario: a forca sai no clique; so o
                        // tempo do gesto se configura, o do manejo e o do chute.)
                        panelHeader("BOLA");
                        const auto milliseconds = [&](const char* label, const char* id, float* value, float minimum,
                                                      float maximum, const char* text) {
                            ImGui::TextUnformatted(label);
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::SliderFloat(id, value, minimum, maximum, "%.0f ms", ImGuiSliderFlags_AlwaysClamp);
                            hint(text);
                        };
                        milliseconds("Tempo do manejo", "##TouchGestureTime", &m_touchGestureMilliseconds, 150.0f, 900.0f,
                            "A duração do gesto do manejo inteiro, do toque à volta. A força sai no clique.");
                        milliseconds("Tempo do chute", "##ShotGestureTime", &m_shotGestureMilliseconds, 250.0f, 1200.0f,
                            "A duração do gesto do chute inteiro, da batida à volta. A força sai no clique.");
                        ImGui::EndTabItem();
                    }
                    // O corpo (26/10, pedido do usuario): um so para todos os bonecos
                    // humanos do laboratorio; vale ao soltar.
                    if (ImGui::BeginTabItem("Corpo")) {
                        ImGui::TextUnformatted("Altura");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##BodyHeight", &m_bodyHeightCm, 170.0f, 200.0f, "%.0f cm",
                            ImGuiSliderFlags_AlwaysClamp);
                        if (ImGui::IsItemDeactivatedAfterEdit()) m_bodyApplyRequested = true;
                        hint("A altura do jogador. Vale ao soltar.");
                        ImGui::TextUnformatted("Peso");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##BodyMass", &m_bodyMassKg, MinimumBodyMassKg, MaximumBodyMassKg, "%.0f kg",
                            ImGuiSliderFlags_AlwaysClamp);
                        if (ImGui::IsItemDeactivatedAfterEdit()) m_bodyApplyRequested = true;
                        hint("O peso do jogador: com a altura, decide se ele é franzino ou encorpado. Vale ao soltar.");
                        if (ImGui::Button("RESTAURAR PADRÕES##Body", { -1.0f, ui(30.0f) })) {
                            m_bodyHeightCm = 180.0f;
                            m_bodyMassKg = 80.0f;
                            m_bodyApplyRequested = true;
                        }
                        ImGui::EndTabItem();
                    }
                    // A forca na bola (07/10, pedido do usuario: "uma tab no menu do
                    // personagem para configuracoes da forca na bola, a de manejo e de
                    // chute ... e um botao de copiar parametros").
                    if (ImGui::BeginTabItem("Força na bola")) {
                        BallOutcomeSettings3D& f = m_ballForce;
                        const auto value = [&](const char* label, const char* id, float* v, float minimum,
                                float maximum, const char* format, const char* text) {
                            ImGui::TextUnformatted(label);
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::SliderFloat(id, v, minimum, maximum, format, ImGuiSliderFlags_AlwaysClamp);
                            hint(text);
                        };
                        // A velocidade (m/s) que a barra da, de vazia a cheia (parado).
                        const auto curve = [&](const char* id, float minimum, float power, float exponent) {
                            float samples[33];
                            for (int k = 0; k <= 32; ++k)
                                samples[k] = minimum + power * std::pow(static_cast<float>(k) / 32.0f,
                                    std::max(0.05f, exponent));
                            char overlay[96];
                            std::snprintf(overlay, sizeof(overlay), "25%%: %.1f  50%%: %.1f  100%%: %.1f m/s", samples[8],
                                samples[16], samples[32]);
                            ImGui::PlotLines(id, samples, 33, 0, overlay, 0.0f, std::max(1.0f, samples[32]) * 1.1f,
                                { -1.0f, ui(52.0f) });
                            hint("A velocidade que a barra dá à bola, de vazia (esquerda) a cheia (direita), parado. Correndo, a bola ainda leva a velocidade do jogador no rumo do toque e o empurrão da corrida.");
                        };
                        // A barra: o minimo (vazia) e a cheia (o total); guardada como minimo + forca.
                        const auto bar = [&](const char* tag, float& minimum, float& power, float& exponent,
                                             float maximumFull, float* chargeSeconds, float maximumCharge) {
                            const std::string id = std::string("##") + tag;
                            value("Barra mínima", (id + "Min").c_str(), &minimum, 0.0f, std::min(20.0f, maximumFull), "%.2f m/s",
                                "A velocidade que a bola sai com a barra vazia (o clique rápido), parado.");
                            float full = minimum + power;
                            value("Barra cheia", (id + "Full").c_str(), &full, 0.0f, maximumFull, "%.1f m/s",
                                "A velocidade que a bola sai com a barra cheia, parado.");
                            power = std::max(0.0f, full - minimum);
                            value("Inclinação da curva", (id + "Curve").c_str(), &exponent, 0.5f, 5.0f, "%.2f",
                                "O formato da barra: acima de 1, fraca embaixo e forte em cima; 1, reta; abaixo de 1, forte já no começo.");
                            curve((id + "Plot").c_str(), minimum, power, exponent);
                            value("Tempo para encher a barra", (id + "Charge").c_str(), chargeSeconds, 0.2f, maximumCharge,
                                "%.2f s", "Quanto tempo segurando o botão leva para a barra encher.");
                        };
                        panelHeader("CHUTE");
                        bar("ForceShot", f.shotMinimum, f.shotPower, f.shotCurve, 45.0f, &m_ballShotChargeSeconds, 3.0f);
                        value("Sensibilidade da câmera carregando o chute", "##ShotCameraSensitivity",
                            &m_shotCameraSensitivity, 0.3f, 1.0f, "%.2f",
                            "Segurando o chute, a câmera gira esta fração da sensibilidade normal (1: igual), para mirar com calma enquanto o gesto do mouse ajusta a altura e o efeito.");
                        value("Sensibilidade vertical do efeito", "##ShotLiftSensitivity", &m_shotLiftSensitivity, 0.25f, 3.0f,
                            "%.2f", "Quanto o puxão do mouse para cima ou para baixo vale na altura e no giro para trás ou por cima (1: como antes; maior: o mesmo puxão dá mais).");
                        value("Sensibilidade horizontal do efeito", "##ShotCurveSensitivity", &m_shotCurveSensitivity, 0.25f,
                            3.0f, "%.2f", "Quanto o puxão do mouse para os lados vale no efeito de lado (1: como antes; maior: o mesmo puxão curva mais).");
                        value("Altura com a barra vazia", "##ForceShotElevationMinimum", &f.shotElevationMinimum, 0.0f,
                            30.0f, "%.1f°", "O ângulo de saída do chute fraco, sem gesto do mouse.");
                        value("Altura a mais com a barra cheia", "##ForceShotElevationPower", &f.shotElevationPower, 0.0f,
                            30.0f, "%.1f°", "Quanto o ângulo sobe do chute fraco ao cheio, sem gesto do mouse.");
                        value("Bola alta (gesto para cima)", "##ForceShotHigh", &f.shotHighElevation, 10.0f, 70.0f, "%.1f°",
                            "O ângulo com o gesto do mouse todo para cima.");
                        value("Rasteira (gesto para baixo)", "##ForceShotLow", &f.shotLowElevation, 0.0f, 10.0f, "%.1f°",
                            "O ângulo com o gesto do mouse todo para baixo.");
                        value("Giro de lado (efeito)", "##ForceShotCurveSpin", &f.shotCurveSpin, 0.0f, 200.0f, "%.0f rad/s",
                            "O giro com o gesto do mouse todo para o lado: quanto a bola curva.");
                        value("Saída para fora da mira", "##ForceShotCurveLaunch", &f.shotCurveLaunch, 0.0f, 15.0f, "%.1f°",
                            "Com efeito, a bola sai este tanto para o lado contrário da curva, para voltar na mira.");
                        value("Giro para trás na bola alta", "##ForceShotBackSpin", &f.shotBackSpin, 0.0f, 60.0f,
                            "%.0f rad/s", "O giro para trás com o gesto todo para cima (a bola alta flutua).");
                        value("Giro por cima na rasteira", "##ForceShotTopSpin", &f.shotTopSpin, 0.0f, 60.0f, "%.0f rad/s",
                            "O giro por cima com o gesto todo para baixo (a rasteira desce e corre).");
                        panelHeader("MANEJO");
                        bar("ForceTouch", f.touchMinimum, f.touchPower, f.touchCurve, 30.0f, &m_ballTouchChargeSeconds, 2.0f);
                        value("Toque mais forte correndo", "##ForceSpeedPush", &f.speedPush, 0.0f, 1.0f, "%.2f",
                            "Quanto o toque empurra a bola a mais por m/s da velocidade do jogador: correndo, o mesmo clique leva a bola mais à frente.");
                        value("Poder de parada", "##ForceStop", &f.stopPower, 0.0f, 1.0f, "%.2f",
                            "A bola vindo contra o manejo: o toque mais fraco que ela corta esta fração do que sobraria dela seguindo (1: ela para no pé; 0: segue como antes, mais devagar). O toque mais forte que ela inverte o rumo com o que sobra da força.");
                        panelHeader("RECEPÇÃO");
                        value("Bola vindo a partir de", "##ForceReceiveClosing", &f.receiveClosing, 0.5f, 10.0f,
                            "%.1f m/s", "O manejo ou o chute fraco com a bola vindo para o jogador mais rápido que isto vira recepção: a bola para junto dele.");
                        value("Resto com a barra vazia", "##ForceReceiveMinimum", &f.receiveMinimum, 0.0f, 3.0f,
                            "%.2f m/s", "Na recepção, a bola para e sai com isto no rumo das teclas (sem tecla, ela fica).");
                        value("Resto a mais com a barra cheia", "##ForceReceivePower", &f.receivePower, 0.0f, 10.0f,
                            "%.2f m/s", "Quanto o resto da recepção cresce da barra vazia à cheia.");
                        value("Herdar a velocidade do jogador", "##ForceReceiveCarry", &f.receiveCarry, 0.0f, 1.0f, "%.2f",
                            "Quanto da velocidade do jogador a bola recebida leva junto (0: ela para no chão e sai só o resto; 1: ela corre com ele).");
                        panelHeader("LEVANTAR");
                        bar("ForceRaise", f.raiseMinimum, f.raisePower, f.raiseCurve, 12.0f, &m_ballRaiseChargeSeconds, 2.0f);
                        value("Para a frente com as teclas", "##ForceRaiseAhead", &f.raiseAhead, 0.0f, 1.5f, "%.2f",
                            "Com uma tecla de direção, a bola sai também nesse rumo: esta fração da velocidade de subida (o chapéu). Sem tecla, sobe reto.");
                        panelHeader("PEITO E CABEÇA");
                        value("Força do peito", "##ForceChestShare", &f.chestShare, 0.0f, 1.0f, "%.2f",
                            "O peito vai até esta fração da barra cheia do manejo e do chute (os valores de cima, lidos na hora).");
                        value("Parada no peito", "##ForceChestStop", &f.chestStop, 0.0f, 1.0f, "%.2f",
                            "Quanto da velocidade que a bola trazia contra o corpo o peito tira: 1 é o domínio perfeito, ela cai a frente dele.");
                        value("Força da cabeça", "##ForceHeadShare", &f.headShare, 0.0f, 1.0f, "%.2f",
                            "O cabeceio vai até esta fração da barra cheia do manejo e do chute (os valores de cima, lidos na hora); a barra vazia fica igual.");
                        // (A bola presa e a hitbox do pe eram do manejo V3: sairam com
                        // ele.)
                        ImGui::Dummy({ 1.0f, ui(6.0f) });
                        if (ImGui::Button("COPIAR PARÂMETROS##BallForce", { -1.0f, ui(36.0f) })) {
                            std::string text = "Força na bola (laboratório, MatterEngine)\n";
                            const auto line = [&](const char* name, const char* label, float v, const char* unit) {
                                char buffer[160];
                                std::snprintf(buffer, sizeof(buffer), "%s = %.3f   # %s%s%s\n", name, v, label,
                                    unit[0] != '\0' ? " " : "", unit);
                                text += buffer;
                            };
                            text += "[chute]\n";
                            line("barra_minima", "barra mínima", f.shotMinimum, "m/s");
                            line("barra_cheia", "barra cheia", f.shotMinimum + f.shotPower, "m/s");
                            line("curva", "inclinação da curva", f.shotCurve, "");
                            line("barra_s", "tempo para encher a barra", m_ballShotChargeSeconds, "s");
                            line("camera", "sensibilidade da câmera carregando o chute", m_shotCameraSensitivity, "");
                            line("sens_vertical", "sensibilidade vertical do efeito", m_shotLiftSensitivity, "");
                            line("sens_horizontal", "sensibilidade horizontal do efeito", m_shotCurveSensitivity, "");
                            line("altura_vazia", "altura com a barra vazia", f.shotElevationMinimum, "graus");
                            line("altura_cheia", "altura a mais com a barra cheia", f.shotElevationPower, "graus");
                            line("alta", "bola alta (gesto para cima)", f.shotHighElevation, "graus");
                            line("rasteira", "rasteira (gesto para baixo)", f.shotLowElevation, "graus");
                            line("efeito", "giro de lado (efeito)", f.shotCurveSpin, "rad/s");
                            line("saida_efeito", "saída para fora da mira", f.shotCurveLaunch, "graus");
                            line("giro_tras", "giro para trás na bola alta", f.shotBackSpin, "rad/s");
                            line("giro_cima", "giro por cima na rasteira", f.shotTopSpin, "rad/s");
                            text += "[manejo]\n";
                            line("barra_minima", "barra mínima", f.touchMinimum, "m/s");
                            line("barra_cheia", "barra cheia", f.touchMinimum + f.touchPower, "m/s");
                            line("curva", "inclinação da curva", f.touchCurve, "");
                            line("barra_s", "tempo para encher a barra", m_ballTouchChargeSeconds, "s");
                            line("correndo", "toque mais forte correndo", f.speedPush, "");
                            line("parada", "poder de parada", f.stopPower, "");
                            text += "[recepcao]\n";
                            line("vindo", "bola vindo a partir de", f.receiveClosing, "m/s");
                            line("resto_minimo", "resto com a barra vazia", f.receiveMinimum, "m/s");
                            line("resto_cheia", "resto a mais com a barra cheia", f.receivePower, "m/s");
                            line("herdar", "herdar a velocidade do jogador", f.receiveCarry, "");
                            text += "[levantar]\n";
                            line("barra_minima", "barra mínima (subida)", f.raiseMinimum, "m/s");
                            line("barra_cheia", "barra cheia (subida)", f.raiseMinimum + f.raisePower, "m/s");
                            line("curva", "inclinação da curva", f.raiseCurve, "");
                            line("barra_s", "tempo para encher a barra", m_ballRaiseChargeSeconds, "s");
                            line("frente", "para a frente com as teclas", f.raiseAhead, "");
                            text += "[peito_cabeca]\n";
                            line("peito", "força do peito", f.chestShare, "da barra cheia");
                            line("parada_peito", "parada no peito", f.chestStop, "");
                            line("cabeca", "força da cabeça", f.headShare, "da barra cheia");
                            ImGui::SetClipboardText(text.c_str());
                            m_ballForceCopiedTime = ImGui::GetTime();
                        }
                        if (ImGui::GetTime() - m_ballForceCopiedTime < 2.0)
                            ImGui::TextColored({ 0.38f, 0.82f, 0.62f, 1.0f }, "Copiado para a área de transferência.");
                        if (ImGui::Button("RESTAURAR PADRÕES##BallForce", { -1.0f, ui(30.0f) })) {
                            m_ballForce = defaultBallOutcomeSettings3D();
                            m_ballTouchChargeSeconds = 0.6f;
                            m_ballShotChargeSeconds = 1.0f;
                            m_ballRaiseChargeSeconds = 0.6f;
                            m_shotCameraSensitivity = 0.6f;
                            m_shotLiftSensitivity = m_shotCurveSensitivity = 1.0f;
                        }
                        ImGui::EndTabItem();
                    }
                    if (ImGui::BeginTabItem("Velocidades")) {
                        panelHeader("VELOCIDADES DO MODO PADRÃO");
                        const auto speed = [&](const char* label, const char* id, float* value,
                                float minimum, float maximum, const char* text) {
                            ImGui::TextUnformatted(label);
                            ImGui::SetNextItemWidth(-1.0f);
                            ImGui::SliderFloat(id, value, minimum, maximum, "%.2f m/s", ImGuiSliderFlags_AlwaysClamp);
                            hint(text);
                        };
                        speed("Trote", "##SpeedTrot", &m_defaultTrotSpeed, 1.5f, 6.0f,
                            "O trote em todos os rumos (frente, lados, costas e diagonais).");
                        speed("Corrida para a frente", "##SpeedSprintForward", &m_defaultSprintForwardSpeed, 3.0f, 10.0f,
                            "A corrida (Shift) para a frente; vale também para as diagonais da frente.");
                        speed("Corrida de lado", "##SpeedSprintSide", &m_defaultSprintSideSpeed, 2.0f, 9.0f,
                            "A corrida para a esquerda e para a direita.");
                        speed("Corrida de costas", "##SpeedSprintBackward", &m_defaultSprintBackwardSpeed, 2.0f, 9.0f,
                            "A corrida para trás; vale também para as diagonais de costas.");
                        speed("Sprint máximo", "##SpeedMaxSprint", &m_defaultMaxSprintSpeed, 6.0f, 12.0f,
                            "Shift duas vezes rápido e segurado: só para a frente e nas diagonais da frente.");
                        ImGui::TextUnformatted("Câmera no sprint máximo");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##MaxSprintCamera", &m_maxSprintCameraSensitivity, 0.1f, 1.0f, "%.2f",
                            ImGuiSliderFlags_AlwaysClamp);
                        hint("A fração da sensibilidade da câmera que fica no sprint máximo (1: igual).");
                        ImGui::TextUnformatted("Resposta");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##CapsuleResponse", &m_capsuleResponsePercent, 20.0f, 150.0f, "%.0f%%",
                            ImGuiSliderFlags_AlwaysClamp);
                        hint("O quanto o jogador acelera, freia e vira no modo padrão. 100%: 24 m/s² de aceleração, 33 de desaceleração, 45 de freada e 15 rad/s de giro. Menos, mais pesado e realista (perto de 27%); mais, mais arcade.");
                        ImGui::TextUnformatted("Troca de direção");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##CapsuleReversal", &m_capsuleReversalPercent, 0.0f, 100.0f, "%.0f%%",
                            ImGuiSliderFlags_AlwaysClamp);
                        hint("Ao pedir o sentido contrário (ir para a esquerda e logo para a direita), o jogador freia e arranca para o outro lado até 1,5 vez mais forte (a 100%), ainda parando antes de sair, para os pés trocarem a passada. 0%: a troca de antes.");
                        ImGui::TextUnformatted("Arrancada");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##StartGesture", &m_defaultStartGesturePercent, 0.0f, 100.0f, "%.0f%%",
                            ImGuiSliderFlags_AlwaysClamp);
                        hint("Ao sair do parado, o corpo se joga para a frente (o gesto do Idle To Sprint no tronco, nos braços e na cabeça) e se inclina com a aceleração, até 2,5 vezes mais que antes (e para trás na freada). 0%: sem o gesto e a inclinação de antes.");
                        ImGui::Dummy({ 1.0f, ui(6.0f) });
                        if (ImGui::Button("COPIAR PARÂMETROS", { -1.0f, ui(36.0f) })) {
                            char text[512];
                            std::snprintf(text, sizeof(text),
                                "Velocidades do modo padrão (m/s)\n"
                                "Trote (todos os rumos): %.2f\n"
                                "Corrida para a frente (e diagonais da frente): %.2f\n"
                                "Corrida de lado: %.2f\n"
                                "Corrida de costas (e diagonais de costas): %.2f\n"
                                "Sprint máximo: %.2f\n"
                                "Câmera no sprint máximo: %.2f\n"
                                "Resposta: %.0f%%\n"
                                "Troca de direção: %.0f%%\n"
                                "Arrancada: %.0f%%\n",
                                m_defaultTrotSpeed, m_defaultSprintForwardSpeed, m_defaultSprintSideSpeed,
                                m_defaultSprintBackwardSpeed, m_defaultMaxSprintSpeed, m_maxSprintCameraSensitivity,
                                m_capsuleResponsePercent, m_capsuleReversalPercent, m_defaultStartGesturePercent);
                            ImGui::SetClipboardText(text);
                            m_defaultSpeedsCopiedTime = ImGui::GetTime();
                        }
                        if (ImGui::GetTime() - m_defaultSpeedsCopiedTime < 2.0)
                            ImGui::TextColored({ 0.38f, 0.82f, 0.62f, 1.0f }, "Copiado para a área de transferência.");
                        if (ImGui::Button("RESTAURAR PADRÕES", { -1.0f, ui(30.0f) })) {
                            // (Os padrões calibrados do laboratório, 17/10.)
                            m_capsuleResponsePercent = LabDefaultCapsuleResponsePercent;
                            m_capsuleReversalPercent = LabDefaultCapsuleReversalPercent;
                            m_defaultStartGesturePercent = LabDefaultStartGesturePercent;
                            m_defaultTrotSpeed = LabDefaultTrotSpeed;
                            m_defaultSprintForwardSpeed = LabDefaultSprintForwardSpeed;
                            m_defaultSprintSideSpeed = LabDefaultSprintSideSpeed;
                            m_defaultSprintBackwardSpeed = LabDefaultSprintBackwardSpeed;
                            m_defaultMaxSprintSpeed = LabDefaultMaxSprintSpeed;
                            m_maxSprintCameraSensitivity = LabDefaultMaxSprintCameraSensitivity;
                        }
                        ImGui::EndTabItem();
                    }
                    ImGui::EndTabBar();
                }
                growPanelToFitContent(io.DisplaySize, m_uiScale);
            }
            ImGui::End();
        }
        if (m_showPhysicsBenchmarkPanel
            && beginDebugPanel("Benchmark físico",
                &m_showPhysicsBenchmarkPanel,
                { ui(12.0f), panelTop + ui(36.0f) },
                { ui(430.0f), ui(510.0f) },
                io.DisplaySize, m_uiScale)) {
            panelHeader("CARGA CONTROLADA");
            const bool configurationLocked =
                m_physicsBenchmarkRunning
                || m_physicsBenchmarkPaused
                || !m_physicsBenchmarkPropEntityIds.empty()
                || !m_physicsBenchmarkRagdollEntityIds.empty();
            ImGui::BeginDisabled(configurationLocked);
            ImGui::SliderInt("Props aleatórios",
                &m_physicsBenchmarkRequestedProps, 10, 500);
            ImGui::SliderInt("Ragdolls",
                &m_physicsBenchmarkRequestedRagdolls, 1, 100);
            ImGui::SliderFloat("Spawn por segundo",
                &m_physicsBenchmarkSpawnRatePerSecond,
                1.0f, 30.0f, "%.0f/s");
            ImGui::EndDisabled();

            ImGui::Dummy({ 1.0f, ui(8.0f) });
            const float buttonGap = ui(7.0f);
            const float buttonWidth =
                (ImGui::GetContentRegionAvail().x
                    - buttonGap * 2.0f) / 3.0f;
            ImGui::BeginDisabled(m_physicsBenchmarkRunning);
            if (ImGui::Button(
                    m_physicsBenchmarkPaused ? "RETOMAR" : "COMEÇAR",
                    { buttonWidth, ui(38.0f) })) {
                beginPhysicsBenchmark();
            }
            ImGui::EndDisabled();
            ImGui::SameLine(0.0f, buttonGap);
            ImGui::BeginDisabled(
                !m_physicsBenchmarkRunning
                && !m_physicsBenchmarkPaused);
            if (ImGui::Button("PAUSAR",
                    { buttonWidth, ui(38.0f) })) {
                pausePhysicsBenchmark();
            }
            ImGui::EndDisabled();
            ImGui::SameLine(0.0f, buttonGap);
            ImGui::BeginDisabled(
                m_physicsBenchmarkPropEntityIds.empty()
                && m_physicsBenchmarkRagdollEntityIds.empty());
            if (ImGui::Button("DELETAR TUDO",
                    { buttonWidth, ui(38.0f) })) {
                clearPhysicsBenchmark();
            }
            ImGui::EndDisabled();

            ImGui::Dummy({ 1.0f, ui(12.0f) });
            panelHeader("PROGRESSO");
            const float propProgress =
                static_cast<float>(m_physicsBenchmarkSpawnedProps)
                / static_cast<float>(std::max(
                    1, m_physicsBenchmarkRequestedProps));
            const float ragdollProgress =
                static_cast<float>(m_physicsBenchmarkSpawnedRagdolls)
                / static_cast<float>(std::max(
                    1, m_physicsBenchmarkRequestedRagdolls));
            char progressText[96] {};
            std::snprintf(progressText, sizeof(progressText),
                "Props %u / %d",
                m_physicsBenchmarkSpawnedProps,
                m_physicsBenchmarkRequestedProps);
            ImGui::ProgressBar(propProgress,
                { -1.0f, ui(22.0f) }, progressText);
            std::snprintf(progressText, sizeof(progressText),
                "Ragdolls %u / %d",
                m_physicsBenchmarkSpawnedRagdolls,
                m_physicsBenchmarkRequestedRagdolls);
            ImGui::ProgressBar(ragdollProgress,
                { -1.0f, ui(22.0f) }, progressText);
            ImGui::TextColored(
                m_physicsBenchmarkPaused
                    ? ImVec4 { 1.0f, 0.72f, 0.25f, 1.0f }
                    : ImVec4 { 0.39f, 0.78f, 1.0f, 1.0f },
                "%s",
                m_physicsBenchmarkPaused ? "PAUSADO"
                : m_physicsBenchmarkRunning ? "SPAWNANDO"
                : configurationLocked ? "CARGA COMPLETA"
                : "PRONTO");

            ImGui::Dummy({ 1.0f, ui(12.0f) });
            panelHeader("TELEMETRIA");
            ImGui::Checkbox("Mostrar analíticos",
                &m_physicsBenchmarkShowAnalytics);
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showPhysicsBenchmarkPanel) ImGui::End();

        if (m_showRenderPanel && beginDebugPanel("Render",
                &m_showRenderPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f), panelTop },
                { panelWidth, ui(360.0f) }, io.DisplaySize, m_uiScale)) {
            panelHeader("MODO DE APRESENTAÇÃO");
            const float modeGap = ui(7.0f);
            const float modeWidth =
                (ImGui::GetContentRegionAvail().x - modeGap) * 0.5f;
            const bool standardMode =
                m_laboratoryRenderMode == SceneRenderMode3D::Standard;
            if (tabButton("NORMAL", standardMode,
                    { modeWidth, ui(42.0f) })
                && !standardMode) {
                m_laboratoryRenderMode = SceneRenderMode3D::Standard;
                m_laboratoryTaaHistoryValid = false;
            }
            ImGui::SameLine(0.0f, modeGap);
            const bool pixelArtMode =
                m_laboratoryRenderMode == SceneRenderMode3D::PixelArt;
            if (tabButton("PIXEL ART", pixelArtMode,
                    { modeWidth, ui(42.0f) })
                && !pixelArtMode) {
                m_laboratoryRenderMode = SceneRenderMode3D::PixelArt;
                m_laboratoryTaaHistoryValid = false;
                m_taaFrameIndex = 0;
            }

            if (m_laboratoryRenderMode != SceneRenderMode3D::Standard) {
                ImGui::Dummy({ 1.0f, ui(13.0f) });
                panelHeader("MATTER MOSAIC");
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Resolução interna do mundo");
                int renderScalePercent = static_cast<int>(std::lround(
                    m_laboratoryPixelArt.renderScale * 100.0f));
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::SliderInt("##PixelArtRenderScale",
                        &renderScalePercent, 25, 100, "%d%%",
                        ImGuiSliderFlags_AlwaysClamp)) {
                    m_laboratoryPixelArt.renderScale =
                        static_cast<float>(renderScalePercent) * 0.01f;
                    m_laboratoryTaaHistoryValid = false;
                    m_taaFrameIndex = 0;
                }
                const int internalWidth = std::max(1,
                    static_cast<int>(std::lround(
                        static_cast<float>(renderer().width())
                            * m_laboratoryPixelArt.renderScale)));
                const int internalHeight = std::max(1,
                    static_cast<int>(std::lround(
                        static_cast<float>(renderer().height())
                            * m_laboratoryPixelArt.renderScale)));
                ImGui::TextColored({ 0.38f, 0.82f, 0.62f, 1.0f },
                    "Mundo %dx%d  |  UI %dx%d",
                    internalWidth, internalHeight,
                    renderer().width(), renderer().height());

                ImGui::Dummy({ 1.0f, ui(11.0f) });
                panelHeader("PIXELIZAÇÃO");
                int resolutionIndex =
                    m_laboratoryPixelArt.pixelGridHeight <= 270.0f ? 0
                    : m_laboratoryPixelArt.pixelGridHeight <= 360.0f ? 1
                    : m_laboratoryPixelArt.pixelGridHeight <= 540.0f ? 2 : 3;
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Tamanho-base dos pixels");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##PixelArtResolution",
                        &resolutionIndex,
                        "270p — Grande\0"
                        "360p — Marcado\0"
                        "540p — Fino\0"
                        "1080p — Somente paleta\0")) {
                    constexpr std::array<float, 4> GridHeights {
                        270.0f, 360.0f, 540.0f, 1080.0f
                    };
                    m_laboratoryPixelArt.pixelGridHeight =
                        GridHeights[static_cast<std::size_t>(
                            std::clamp(resolutionIndex, 0, 3))];
                    m_laboratoryTaaHistoryValid = false;
                }

                int worldPixelationPercent =
                    static_cast<int>(std::lround(
                        m_laboratoryPixelArt.worldPixelation * 100.0f));
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Pixelização do mundo");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::SliderInt("##WorldPixelation",
                        &worldPixelationPercent, 0, 100, "%d%%",
                        ImGuiSliderFlags_AlwaysClamp)) {
                    m_laboratoryPixelArt.worldPixelation =
                        static_cast<float>(worldPixelationPercent) * 0.01f;
                }

                int propPixelationPercent =
                    static_cast<int>(std::lround(
                        m_laboratoryPixelArt.physicalPropPixelation
                            * 100.0f));
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Pixelização dos objetos físicos");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::SliderInt("##PropPixelation",
                        &propPixelationPercent, 0, 100, "%d%%",
                        ImGuiSliderFlags_AlwaysClamp)) {
                    m_laboratoryPixelArt.physicalPropPixelation =
                        static_cast<float>(propPixelationPercent) * 0.01f;
                }

                ImGui::Dummy({ 1.0f, ui(11.0f) });
                panelHeader("LOD ARTÍSTICO");
                int lodStrengthPercent = static_cast<int>(std::lround(
                    m_laboratoryPixelArt.distanceLodStrength * 100.0f));
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Pixelização adicional à distância");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::SliderInt("##DistancePixelLodStrength",
                        &lodStrengthPercent, 0, 100, "%d%%",
                        ImGuiSliderFlags_AlwaysClamp)) {
                    m_laboratoryPixelArt.distanceLodStrength =
                        static_cast<float>(lodStrengthPercent) * 0.01f;
                }
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Início / alcance total do LOD");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::SliderFloat("##DistancePixelLodStart",
                    &m_laboratoryPixelArt.distanceLodStartMeters,
                    0.0f, 120.0f, "%.0f m",
                    ImGuiSliderFlags_AlwaysClamp);
                m_laboratoryPixelArt.distanceLodEndMeters = std::max(
                    m_laboratoryPixelArt.distanceLodEndMeters,
                    m_laboratoryPixelArt.distanceLodStartMeters + 1.0f);
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::SliderFloat("##DistancePixelLodEnd",
                    &m_laboratoryPixelArt.distanceLodEndMeters,
                    m_laboratoryPixelArt.distanceLodStartMeters + 1.0f,
                    500.0f, "%.0f m",
                    ImGuiSliderFlags_AlwaysClamp);
                ImGui::SetItemTooltip("Os objetos recebem só uma fração do LOD (bolas e objetos continuam legíveis).");

                ImGui::Dummy({ 1.0f, ui(11.0f) });
                panelHeader("ACABAMENTO 2D");
                int flatteningPercent = static_cast<int>(std::lround(
                    m_laboratoryPixelArt.flatteningStrength * 100.0f));
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Achatamento de luz e paleta");
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::SliderInt("##MosaicFlattening",
                        &flatteningPercent, 0, 100, "%d%%",
                        ImGuiSliderFlags_AlwaysClamp)) {
                    m_laboratoryPixelArt.flatteningStrength =
                        static_cast<float>(flatteningPercent) * 0.01f;
                }
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Níveis de luz");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::SliderFloat("##MosaicLuminanceLevels",
                    &m_laboratoryPixelArt.luminanceLevelCount,
                    4.0f, 24.0f, "%.0f",
                    ImGuiSliderFlags_AlwaysClamp);
                ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                    "Dithering estável");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::SliderFloat("##MosaicDitherStrength",
                    &m_laboratoryPixelArt.ditherStrength,
                    0.0f, 0.30f, "%.3f",
                    ImGuiSliderFlags_AlwaysClamp);
                ImGui::Dummy({ 1.0f, ui(11.0f) });
                panelHeader("ANTIALIASING EXPERIMENTAL");
                if (ImGui::Checkbox("Ativar TAA no Pixel Art",
                        &m_laboratoryPixelArt
                            .temporalAntiAliasingEnabled)) {
                    m_laboratoryTaaHistoryValid = false;
                    m_taaFrameIndex = 0;
                }
                ImGui::SetItemTooltip("Suaviza o movimento, mas arredonda um pouco os pixels.");

                ImGui::Dummy({ 1.0f, ui(11.0f) });
                if (ImGui::Button("RESTAURAR IDENTIDADE PADRÃO",
                        { -1.0f, ui(34.0f) })) {
                    m_laboratoryPixelArt = PixelArtSettings3D {};
                    m_laboratoryTaaHistoryValid = false;
                    m_taaFrameIndex = 0;
                }
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showRenderPanel) ImGui::End();

        if (m_showPhysgunPanel && beginDebugPanel("Physgun",
                &m_showPhysgunPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f), panelTop },
                { panelWidth, ui(430.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    panelHeader("APRESENTAÇÃO");
                    if (ImGui::Checkbox(
                            "Ocultar modelo e mira (modo foto)",
                            &m_hidePhysGunPresentation)) {
                        m_laboratoryTaaHistoryValid = false;
                        m_taaFrameIndex = 0;
                    }
                    ImGui::SetItemTooltip("A Physgun continua funcionando; só o modelo e a mira somem.");
                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    panelHeader("FORÇA DE MANIPULAÇÃO");
                    ImGui::Dummy({ 1.0f, ui(8.0f) });
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##PhysGunMaximumForce",
                        &m_physGunSettings.maximumForce,
                        250.0f, 500000.0f, "%.0f N",
                        ImGuiSliderFlags_Logarithmic
                            | ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    panelHeader("COMPORTAMENTO");
                    int holdMode = m_physGunHoldMode
                            == PhysGunHoldMode::GrabPoint
                        ? 0 : 1;
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##PhysGunHoldMode", &holdMode,
                            "Ponto de grab\0Pose fixa\0")) {
                        m_physGunHoldMode = holdMode == 0
                            ? PhysGunHoldMode::GrabPoint
                            : PhysGunHoldMode::FixedPose;
                    }
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Suavidade da rotação");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##PhysGunRotationSmoothness",
                        &m_physGunRotationSmoothness,
                        0.0f, 1.0f, "%.2f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Passo angular do Shift + E");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##PhysGunRotationSnap",
                        &m_physGunRotationSnapDegrees,
                        1.0f, 90.0f, "%.0f°",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    if (ImGui::CollapsingHeader("Ajuste visual")) {
                        ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                            "Eixos locais da câmera");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat3("Posição X/Y/Z",
                            &m_physGunViewCalibration.offsetCameraLocal.x,
                            0.005f, -2.0f, 2.0f, "%.3f m");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat3("Rotação X/Y/Z",
                            &m_physGunViewCalibration.rotationDegrees.x,
                            0.25f, -180.0f, 180.0f, "%.2f°");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat("Tamanho",
                            &m_physGunViewCalibration.sizeMeters,
                            0.005f, 0.05f, 2.5f, "%.3f m");
                        if (ImGui::Button("Copiar valores",
                                { -1.0f, ui(36.0f) })) {
                            char calibration[384] {};
                            std::snprintf(calibration, sizeof(calibration),
                                "Physgun: posição={%.3ff, %.3ff, %.3ff}; "
                                "rotação={%.2ff, %.2ff, %.2ff} graus; "
                                "tamanho=%.3ff",
                                m_physGunViewCalibration.offsetCameraLocal.x,
                                m_physGunViewCalibration.offsetCameraLocal.y,
                                m_physGunViewCalibration.offsetCameraLocal.z,
                                m_physGunViewCalibration.rotationDegrees.x,
                                m_physGunViewCalibration.rotationDegrees.y,
                                m_physGunViewCalibration.rotationDegrees.z,
                                m_physGunViewCalibration.sizeMeters);
                            ImGui::SetClipboardText(calibration);
                            m_physGunCalibrationCopyStatus =
                                "Valores copiados";
                        }
                        if (!m_physGunCalibrationCopyStatus.empty()) {
                            ImGui::TextColored(
                                { 0.38f, 0.82f, 0.62f, 1.0f },
                                "%s",
                                m_physGunCalibrationCopyStatus.c_str());
                        }
                    }

                    if (ImGui::CollapsingHeader("Origem do feixe")) {
                        ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                            "Eixos locais da câmera");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::DragFloat3("Origem X/Y/Z",
                            &m_physGunBeamStartCameraLocal.x,
                            0.0025f, -2.0f, 3.0f, "%.3f m");
                        if (ImGui::Button("Copiar origem do feixe",
                                { -1.0f, ui(36.0f) })) {
                            char calibration[256] {};
                            std::snprintf(calibration, sizeof(calibration),
                                "Feixe da Physgun: origem={%.3ff, %.3ff, %.3ff}",
                                m_physGunBeamStartCameraLocal.x,
                                m_physGunBeamStartCameraLocal.y,
                                m_physGunBeamStartCameraLocal.z);
                            ImGui::SetClipboardText(calibration);
                            m_physGunBeamCalibrationCopyStatus =
                                "Origem copiada";
                        }
                        if (!m_physGunBeamCalibrationCopyStatus.empty()) {
                            ImGui::TextColored(
                                { 0.38f, 0.82f, 0.62f, 1.0f },
                                "%s",
                                m_physGunBeamCalibrationCopyStatus.c_str());
                        }
                    }

                    if (ImGui::CollapsingHeader("Lanterna")) {
                        ImGui::Checkbox("Ativa (F)",
                            &m_physGunFlashlight.enabled);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::ColorEdit3("Cor",
                            &m_physGunFlashlight.color.x,
                            ImGuiColorEditFlags_NoInputs);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("Abertura do cone",
                            &m_physGunFlashlight.coneDegrees,
                            6.0f, 120.0f, "%.0f°");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("Distância",
                            &m_physGunFlashlight.rangeMeters,
                            2.0f, 100.0f, "%.1f m",
                            ImGuiSliderFlags_Logarithmic);
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("Brilho",
                            &m_physGunFlashlight.intensity,
                            0.1f, 20.0f, "%.2f",
                            ImGuiSliderFlags_Logarithmic);
                    }
                    ImGui::Dummy({ 1.0f, ui(10.0f) });
                    drawInflatedBallPanel();
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showPhysgunPanel) ImGui::End();

        if (m_showRagdollPanel && beginDebugPanel("Ragdoll",
                &m_showRagdollPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f), panelTop },
                { panelWidth, ui(420.0f) }, io.DisplaySize, m_uiScale)) {
            panelHeader("ARTICULATION PHYSX");
            const std::size_t humanRagdollCount = static_cast<std::size_t>(
                std::count_if(m_spawnedRagdolls.begin(),
                    m_spawnedRagdolls.end(),
                    [](const SpawnedRagdollInstance& instance) {
                        return instance.kind == ArticulatedRigKind::Human;
                    }));
            ImGui::TextColored({ 0.38f, 0.82f, 0.62f, 1.0f },
                "%zu ragdoll%s ativo%s",
                humanRagdollCount,
                humanRagdollCount == 1 ? "" : "s",
                humanRagdollCount == 1 ? "" : "s");
            ImGui::Dummy({ 1.0f, ui(12.0f) });

            if (ImGui::BeginTabBar("##RagdollPanelTabs")) {
            if (ImGui::BeginTabItem("CONTROLE")) {
            panelHeader("MODO");
            if (ImGui::Checkbox("Ragdoll ativo", &m_ragdollActive)
                && m_physicsScene) {
                for (SpawnedRagdollInstance& instance :
                        m_spawnedRagdolls) {
                    if (instance.kind != ArticulatedRigKind::Human) continue;
                    instance.active = m_ragdollActive;
                    if (instance.active && m_humanRagdollProfile) {
                        instance.athlete.legacy.locomotion.reset(*m_humanRagdollProfile,
                            instance.physicsState);
                    }
                    m_physicsScene->setRagdollActive(
                        instance.physicsRagdoll, instance.active);
                }
            }
            ImGui::SetItemTooltip("Ativo: músculos, torque corporal e contatos. Passivo: só juntas, limites e colisões.");
            panelHeader("FORÇA AUXILIAR E MUSCULAR");
            ImGui::Checkbox("Forçar alteração dos valores", &m_ragdollForceControlOverride);
            ImGui::BeginDisabled(!m_ragdollForceControlOverride);
            ImGui::TextUnformatted("Força auxiliar");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::SliderFloat("##RagdollAuxiliaryAuthority", &m_ragdollAuxiliaryPercent,
                0.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp);
            ImGui::TextUnformatted("Força muscular");
            ImGui::SetNextItemWidth(-1.0f);
            ImGui::SliderFloat("##RagdollMuscleAuthority", &m_ragdollMusclePercent,
                0.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp);
            ImGui::EndDisabled();
            ImGui::Dummy({1.0f,ui(12.0f)});

            panelHeader("ALVOS");
            if (m_ragdollActive) ImGui::BeginDisabled();
            const ImVec2 actionSize {
                (ImGui::GetContentRegionAvail().x - ui(7.0f)) * 0.5f,
                ui(38.0f)
            };
            if (ImGui::Button("CAPTURAR POSE", actionSize)
                && m_physicsScene) {
                for (const SpawnedRagdollInstance& instance :
                        m_spawnedRagdolls) {
                    if (instance.kind != ArticulatedRigKind::Human) continue;
                    m_physicsScene->captureRagdollPose(
                        instance.physicsRagdoll);
                }
            }
            ImGui::SameLine(0.0f, ui(7.0f));
            if (ImGui::Button("POSE NEUTRA", actionSize)
                && m_physicsScene) {
                for (const SpawnedRagdollInstance& instance :
                        m_spawnedRagdolls) {
                    if (instance.kind != ArticulatedRigKind::Human) continue;
                    m_physicsScene->setRagdollNeutralPose(
                        instance.physicsRagdoll);
                }
            }
            if (ImGui::Button("SOLTAR DRIVES",
                    { -1.0f, ui(38.0f) }) && m_physicsScene) {
                m_ragdollRigidityPercent = 0.0f;
                for (const SpawnedRagdollInstance& instance :
                        m_spawnedRagdolls) {
                    if (instance.kind != ArticulatedRigKind::Human) continue;
                    m_physicsScene->releaseRagdollDrives(
                        instance.physicsRagdoll);
                }
            }
            if (m_ragdollActive) ImGui::EndDisabled();
            const auto latestHuman = std::find_if(
                m_spawnedRagdolls.rbegin(), m_spawnedRagdolls.rend(),
                [](const SpawnedRagdollInstance& instance) {
                    return instance.kind == ArticulatedRigKind::Human
                        && instance.active;
                });
            if (latestHuman != m_spawnedRagdolls.rend()) {
                const auto& telemetry=latestHuman->athlete.legacy.locomotion.telemetry();
                const char* phase="IDLE";
                switch(telemetry.state) {
                case CharacterLocomotionState3D::Idle: phase="IDLE";break;
                case CharacterLocomotionState3D::Turning: phase="GIRANDO";break;
                case CharacterLocomotionState3D::Walking: phase="CAMINHANDO";break;
                case CharacterLocomotionState3D::Running: phase="CORRENDO";break;
                case CharacterLocomotionState3D::CrouchIdle: phase="AGACHADO";break;
                case CharacterLocomotionState3D::CrouchWalking: phase="AGACHADO ANDANDO";break;
                case CharacterLocomotionState3D::JumpStarting: phase="SALTO";break;
                case CharacterLocomotionState3D::Airborne: phase="NO AR";break;
                case CharacterLocomotionState3D::Landing: phase="ATERRISSANDO";break;
                case CharacterLocomotionState3D::Fallen: phase="CAÍDO";break;
                case CharacterLocomotionState3D::GettingUp: phase="LEVANTANDO";break;
                case CharacterLocomotionState3D::Tackling: phase="CARRINHO";break;
                }
                panelHeader("ANIMAÇÃO FÍSICA");
                ImGui::Text("Estado: %s",phase);
                ImGui::Text("Velocidade: %.2f m/s | fase: %.2f",
                    telemetry.speedMetersPerSecond,telemetry.cyclePhase);
                ImGui::Text("Ritmo da animação: %.2fx",
                    telemetry.playbackRate);
                ImGui::Text("Erro articular RMS: %.1f graus",
                    telemetry.jointErrorRmsDegrees);
                ImGui::Text("Pés travados E/D: %s / %s",
                    telemetry.footPlanted[0]?"SIM":"NÃO",
                    telemetry.footPlanted[1]?"SIM":"NÃO");
                ImGui::Text("Folga dos pés E/D: %.1f / %.1f cm",
                    100*telemetry.footClearanceMeters[0],
                    100*telemetry.footClearanceMeters[1]);
                ImGui::Text("Músculos: %.0f%% | guia articular: %.0f%%",
                    telemetry.muscleAuthority*100,
                    telemetry.poseAuthority*100);
                ImGui::Text("Translação: %.0f%% | guia angular: %.0f%%",
                    telemetry.rootAuthority*100,
                    telemetry.rootRotationAuthority*100);
                ImGui::Text("Reação: %.0f%% | giro físico: %.1f°/s",
                    telemetry.reactionStrength*100,
                    telemetry.turningRateRadiansPerSecond*180.0f/Pi);
            }
            ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("IMPULSOS")) {
                RagdollImpactTestConfig3D& impactConfig =
                    m_ragdollImpactLab.config();
                const RagdollImpactTestOutput3D& impactOutput =
                    m_ragdollImpactLab.output();
                panelHeader("LABORATÓRIO DE EQUILÍBRIO");
                ImGui::Dummy({ 1.0f, ui(8.0f) });

                int impactMode = static_cast<int>(impactConfig.mode);
                const char* impactModes[] = {
                    "IMPACTO DIRETO", "EMPURRÃO CONTÍNUO", "ALEATÓRIO"
                };
                ImGui::SetNextItemWidth(-1.0f);
                if (ImGui::Combo("##RagdollImpactMode", &impactMode,
                        impactModes, 3)) {
                    impactConfig.mode = static_cast<RagdollImpactMode3D>(
                        impactMode);
                }

                if (impactConfig.mode != RagdollImpactMode3D::Random) {
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Força",
                        &impactConfig.forceNewtons,
                        0.0f, 2'000.0f, "%.0f N",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Direção relativa",
                        &impactConfig.directionDegrees,
                        0.0f, 360.0f, "%.0f°",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetItemTooltip("0° frente, 90° esquerda, 180° costas, 270° direita (de cada boneco).");
                }

                if (impactConfig.mode
                        == RagdollImpactMode3D::DirectPulse) {
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Intervalo",
                        &impactConfig.intervalSeconds,
                        0.25f, 15.0f, "%.2f s",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Duração da rajada",
                        &impactConfig.pulseDurationSeconds,
                        0.03f, 2.0f, "%.2f s",
                        ImGuiSliderFlags_AlwaysClamp);
                } else if (impactConfig.mode
                        == RagdollImpactMode3D::Random) {
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Força mínima",
                        &impactConfig.randomMinimumForceNewtons,
                        0.0f, 2'000.0f, "%.0f N",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Força máxima",
                        &impactConfig.randomMaximumForceNewtons,
                        0.0f, 2'000.0f, "%.0f N",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Intervalo mínimo",
                        &impactConfig.randomMinimumIntervalSeconds,
                        0.25f, 15.0f, "%.2f s",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Intervalo máximo",
                        &impactConfig.randomMaximumIntervalSeconds,
                        0.25f, 15.0f, "%.2f s",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Duração mínima",
                        &impactConfig.randomMinimumDurationSeconds,
                        0.03f, 4.0f, "%.2f s",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("Duração máxima",
                        &impactConfig.randomMaximumDurationSeconds,
                        0.03f, 4.0f, "%.2f s",
                        ImGuiSliderFlags_AlwaysClamp);
                }

                ImGui::Dummy({ 1.0f, ui(8.0f) });
                const ImVec2 impactActionSize {
                    (ImGui::GetContentRegionAvail().x - ui(7.0f)) * 0.5f,
                    ui(38.0f)
                };
                if (!m_ragdollImpactLab.running()) {
                    if (ImGui::Button("INICIAR", impactActionSize)) {
                        m_ragdollImpactLab.setRunning(true);
                    }
                } else if (ImGui::Button("PAUSAR", impactActionSize)) {
                    m_ragdollImpactLab.setRunning(false);
                }
                ImGui::SameLine(0.0f, ui(7.0f));
                if (ImGui::Button("APLICAR AGORA", impactActionSize)) {
                    m_ragdollImpactLab.triggerNow();
                }
                if (ImGui::Button("PARAR E ZERAR",
                        { -1.0f, ui(36.0f) })) {
                    m_ragdollImpactLab.reset();
                }

                ImGui::Dummy({ 1.0f, ui(8.0f) });
                ImGui::Text("Estado: %s",
                    impactOutput.applying ? "APLICANDO FORÇA"
                    : (m_ragdollImpactLab.running()
                        ? "AGUARDANDO" : "PARADO"));
                ImGui::Text("Evento: %llu",
                    static_cast<unsigned long long>(
                        impactOutput.eventCount));
                if (impactOutput.applying) {
                    ImGui::Text("Força atual: %.0f N a %.0f°",
                        impactOutput.forceNewtons,
                        impactOutput.directionDegrees);
                    if (impactOutput.secondsRemaining >= 0.0f) {
                        ImGui::Text("Tempo restante: %.2f s",
                            impactOutput.secondsRemaining);
                    }
                } else if (m_ragdollImpactLab.running()
                        && impactConfig.mode
                            != RagdollImpactMode3D::Continuous) {
                    ImGui::Text("Próximo em: %.2f s",
                        impactOutput.secondsUntilNext);
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showRagdollPanel) ImGui::End();

        if (m_showGraphicsPanel && beginDebugPanel("Gráficos",
                &m_showGraphicsPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f),
                    panelTop + ui(24.0f) },
                { panelWidth, ui(430.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    panelHeader("PERFIL DE QUALIDADE");
                    int graphicsPreset =
                        m_laboratoryGraphicsPreset;
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::Combo("##GraphicsQualityPreset",
                            &graphicsPreset,
                            "Desempenho\0Equilibrado\0Alto\0"
                            "Personalizado\0")) {
                        if (graphicsPreset < 3) {
                            applyLaboratoryGraphicsPreset(
                                graphicsPreset);
                        } else {
                            m_laboratoryGraphicsPreset = 3;
                        }
                    }
                    const std::uint32_t shadowSamples =
                        m_laboratoryGraphicsPreset == 0 ? 4u
                        : m_laboratoryGraphicsPreset == 1 ? 8u : 12u;
                    ImGui::TextColored(
                        { 0.50f, 0.62f, 0.72f, 1.0f },
                        "PCSS: %u + %u amostras | cascatas 2048 px",
                        shadowSamples, shadowSamples);
                    ImGui::Dummy({ 1.0f, ui(12.0f) });

                    panelHeader("ILUMINAÇÃO AMBIENTAL");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Céu %.3f  |  Rebote solar %.3f",
                        m_laboratoryEnvironment.skyIrradiance,
                        m_laboratoryEnvironment.sunDiffuseBounce);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Luz mínima em oclusão total");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat(
                        "##EnvironmentMinimumIndirectVisibility",
                        &m_laboratoryEnvironment.minimumIndirectVisibility,
                        0.0f, 0.25f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(12.0f) });

                    panelHeader("ÓPTICA SOLAR HDR");
                    if (ImGui::Checkbox("Ativar glare e flare",
                            &m_laboratoryOpticalEffects.enabled)) {
                        m_laboratoryGraphicsPreset = 3;
                    }
                    if (m_laboratoryOpticalEffects.enabled) {
                        ImGui::Dummy({ 1.0f, ui(7.0f) });
                        ImGui::TextColored(
                            { 0.50f, 0.62f, 0.72f, 1.0f },
                            "Glare do Sol");
                        ImGui::SetNextItemWidth(-1.0f);
                        if (ImGui::SliderFloat("##SunGlareStrength",
                            &m_laboratoryOpticalEffects
                                .sunGlareStrength,
                            0.0f, 0.65f, "%.2f",
                            ImGuiSliderFlags_AlwaysClamp)) {
                            m_laboratoryGraphicsPreset = 3;
                        }
                        ImGui::Dummy({ 1.0f, ui(7.0f) });
                        ImGui::TextColored(
                            { 0.50f, 0.62f, 0.72f, 1.0f },
                            "Ghosts da lente");
                        ImGui::SetNextItemWidth(-1.0f);
                        if (ImGui::SliderFloat("##LensFlareStrength",
                            &m_laboratoryOpticalEffects
                                .lensFlareStrength,
                            0.0f, 0.35f, "%.2f",
                            ImGuiSliderFlags_AlwaysClamp)) {
                            m_laboratoryGraphicsPreset = 3;
                        }
                    }
                    ImGui::Dummy({ 1.0f, ui(12.0f) });

                    panelHeader("TONEMAP (HDR → LDR)");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Exposição fixa");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingExposure",
                        &m_laboratoryToneMapping.exposure,
                        0.05f, 2.0f, "%.3f",
                        ImGuiSliderFlags_Logarithmic
                            | ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Brilho");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingBrightness",
                        &m_laboratoryToneMapping.brightness,
                        -0.5f, 0.5f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Contraste");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingContrast",
                        &m_laboratoryToneMapping.contrast,
                        0.0f, 2.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Saturação");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##ToneMappingSaturation",
                        &m_laboratoryToneMapping.saturation,
                        0.0f, 2.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    panelHeader("NEBLINA");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Distância de início (m)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogStartDistance",
                        &m_laboratoryFog.startDistanceMeters,
                        0.0f, 1500.0f, "%.0f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Ocultação total do cenário (m)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogEndDistance",
                        &m_laboratoryFog.endDistanceMeters,
                        m_laboratoryFog.startDistanceMeters + 1.0f,
                        2000.0f, "%.0f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Densidade (queda por metro após o início)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogDensity",
                        &m_laboratoryFog.density,
                        0.0f, 0.05f, "%.4f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Acoplamento altura-distância");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##FogHeightFalloff",
                        &m_laboratoryFog.heightFalloff,
                        0.0f, 0.10f, "%.4f",
                        ImGuiSliderFlags_AlwaysClamp);
                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    if (ImGui::Button("Copiar valores calibrados",
                            { -1.0f, ui(36.0f) })) {
                        char calibration[640] {};
                        std::snprintf(calibration, sizeof(calibration),
                            "ToneMapping: exposure=%.4ff; brightness=%.4ff; "
                            "contrast=%.4ff; saturation=%.4ff; "
                            "automatic=%s; autoMin=%.3ff; autoMax=%.3ff; "
                            "meteringKey=%.3ff; brightSpeed=%.3ff; "
                            "darkSpeed=%.3ff\n"
                            "Fog: startDistanceMeters=%.1ff; density=%.4ff; "
                            "heightFalloff=%.4ff; maxOpacity=%.4ff; "
                            "color={%.3ff, %.3ff, %.3ff}",
                            m_laboratoryToneMapping.exposure,
                            m_laboratoryToneMapping.brightness,
                            m_laboratoryToneMapping.contrast,
                            m_laboratoryToneMapping.saturation,
                            m_laboratoryToneMapping
                                .automaticExposureEnabled ? "true" : "false",
                            m_laboratoryToneMapping
                                .automaticExposureMinimum,
                            m_laboratoryToneMapping
                                .automaticExposureMaximum,
                            m_laboratoryToneMapping
                                .automaticExposureMeteringKey,
                            m_laboratoryToneMapping.brightAdaptationSpeed,
                            m_laboratoryToneMapping.darkAdaptationSpeed,
                            m_laboratoryFog.startDistanceMeters,
                            m_laboratoryFog.density,
                            m_laboratoryFog.heightFalloff,
                            m_laboratoryFog.maxOpacity,
                            m_laboratoryFog.color.x,
                            m_laboratoryFog.color.y,
                            m_laboratoryFog.color.z);
                        ImGui::SetClipboardText(calibration);
                        m_toneMappingCopyStatus = "Valores copiados";
                    }
                    if (!m_toneMappingCopyStatus.empty()) {
                        ImGui::TextColored({ 0.38f, 0.82f, 0.62f, 1.0f },
                            "%s", m_toneMappingCopyStatus.c_str());
                    }
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showGraphicsPanel) ImGui::End();

        if (m_showTimePanel && beginDebugPanel("Tempo e atmosfera",
                &m_showTimePanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f),
                    panelTop + ui(48.0f) },
                { panelWidth, ui(420.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    float hour = std::clamp(
                        m_laboratoryEnvironment.solarTimeHours,
                        0.0f, 24.0f);
                    int totalMinutes = static_cast<int>(
                        std::round(hour * 60.0f)) % (24 * 60);
                    const int displayHour = totalMinutes / 60;
                    const int displayMinute = totalMinutes % 60;
                    const char* phase = hour < 5.0f ? "Madrugada"
                        : hour < 7.0f ? "Amanhecer"
                        : hour < 12.0f ? "Manhã"
                        : hour < 17.0f ? "Tarde"
                        : hour < 19.0f ? "Entardecer"
                        : "Noite";

                    panelHeader("RELÓGIO CELESTE");
                    ImGui::TextColored({ 0.72f, 0.86f, 0.96f, 1.0f },
                        "%02d:%02d  —  %s",
                        displayHour, displayMinute, phase);
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::SliderFloat("##TimeOfDay", &hour,
                            0.0f, 24.0f, "%.2f h",
                            ImGuiSliderFlags_AlwaysClamp)) {
                        m_laboratoryEnvironment.solarTimeHours =
                            hour >= 24.0f ? 0.0f : hour;
                        m_laboratoryTaaHistoryValid = false;
                    }
                    ImGui::Checkbox("Avançar o tempo",
                        &m_laboratoryClockRunning);
                    if (m_laboratoryClockRunning) {
                        ImGui::TextColored(
                            { 0.50f, 0.62f, 0.72f, 1.0f },
                            "Escala temporal");
                        ImGui::SetNextItemWidth(-1.0f);
                        ImGui::SliderFloat("##ClockTimeScale",
                            &m_laboratoryClockTimeScale,
                            1.0f, 3600.0f, "%.0fx",
                            ImGuiSliderFlags_Logarithmic
                                | ImGuiSliderFlags_AlwaysClamp);
                    }

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    panelHeader("NUVENS E LUZ");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Cobertura do céu");
                    float cloudPercentage =
                        m_laboratoryEnvironment.cloudCoverage * 100.0f;
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WeatherCloudCoverage",
                        &cloudPercentage, 0.0f, 100.0f, "%.0f%%",
                        ImGuiSliderFlags_AlwaysClamp);
                    m_laboratoryEnvironment.cloudCoverage =
                        cloudPercentage * 0.01f;
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Sol atravessando nuvens: %.0f%%",
                        m_laboratoryEnvironment.cloudSunTransmittance
                            * 100.0f);

                    const float presetGap = ui(5.0f);
                    const float presetWidth =
                        (ImGui::GetContentRegionAvail().x
                            - presetGap * 3.0f) * 0.25f;
                    if (ImGui::Button("Limpo",
                            { presetWidth, ui(34.0f) })) {
                        m_laboratoryEnvironment.cloudCoverage = 0.05f;
                    }
                    ImGui::SameLine(0.0f, presetGap);
                    if (ImGui::Button("Parcial",
                            { presetWidth, ui(34.0f) })) {
                        m_laboratoryEnvironment.cloudCoverage = 0.38f;
                    }
                    ImGui::SameLine(0.0f, presetGap);
                    if (ImGui::Button("Nublado",
                            { presetWidth, ui(34.0f) })) {
                        m_laboratoryEnvironment.cloudCoverage = 0.72f;
                    }
                    ImGui::SameLine(0.0f, presetGap);
                    if (ImGui::Button("Fechado",
                            { presetWidth, ui(34.0f) })) {
                        m_laboratoryEnvironment.cloudCoverage = 0.96f;
                    }

            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showTimePanel) ImGui::End();

        if (m_showPerformancePanel && beginDebugPanel("Performance",
                &m_showPerformancePanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f),
                    panelTop + ui(48.0f) },
                { panelWidth, ui(300.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    const ApplicationFrameMetrics& application =
                        applicationFrameMetrics();
                    const RHI::FramePerformanceMetrics graphics =
                        renderer().framePerformanceMetrics();
                    panelHeader("FRAME");
                    ImGui::Text("Total     %.3f ms",
                        application.totalMilliseconds);
                    ImGui::Text("Update    %.3f ms",
                        application.fixedUpdateMilliseconds);
                    ImGui::Text("Render    %.3f ms",
                        application.renderMilliseconds);
                    ImGui::Text("UI        %.3f ms",
                        application.guiMilliseconds);
                    ImGui::Text("GPU       %.3f ms",
                        graphics.gpuTimingValid
                            ? graphics.gpuFrameMilliseconds : 0.0f);
                    ImGui::Text("Fence     %.3f ms",
                        graphics.cpuFenceWaitMilliseconds);
                    ImGui::Text("Acquire   %.3f ms",
                        graphics.cpuAcquireMilliseconds);
                    ImGui::Text("Present   %.3f ms",
                        graphics.cpuPresentMilliseconds);
                    if (m_physicsScene) {
                        const PhysicsStepDiagnostics3D& physics =
                            m_physicsScene->diagnostics();
                        ImGui::Dummy({ 1.0f, ui(10.0f) });
                        panelHeader("PHYSX 120 HZ");
                        ImGui::Text("Passo     %.3f ms",
                            physics.totalStepMilliseconds);
                        ImGui::Text("Pre       %.3f ms",
                            physics.preSimulationMilliseconds);
                        ImGui::Text("Dispatch  %.3f ms",
                            physics.simulationDispatchMilliseconds);
                        ImGui::Text("Espera    %.3f ms",
                            physics.simulationWaitMilliseconds);
                        ImGui::Text("Callback  %.3f ms",
                            physics.contactCallbackMilliseconds);
                        ImGui::Text("Sync      %.3f ms",
                            physics.stateSyncMilliseconds);
                        ImGui::Text("Workers   %u",
                            physics.physicsWorkerCount);
                        ImGui::Text("Tarefas   %zu",
                            physics.submittedPhysicsTasks);
                        ImGui::Text("Ativos    %zu",
                            physics.activeDynamicBodyCount);
                        ImGui::Text("Dormindo  %zu",
                            physics.sleepingDynamicBodyCount);
                        ImGui::Text("Contatos  %zu",
                            physics.discreteContactPairs);
                        ImGui::Text("Reportados %zu (%zu pontos)",
                            physics.reportedContactPairs,
                            physics.reportedContactPoints);
                        ImGui::Text("CCD       %zu", physics.ccdPairs);
                    }
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showPerformancePanel) ImGui::End();

        // Painel TEMPORARIO de calibracao (ver WindSettings3D::
        // mutableSettings, criado exatamente pra isto e nunca ligado ate
        // pouco atras) - acompanha e ajusta o vento durante os testes de
        // calmaria/rajada/abrigo por parede. Nao faz parte da experiencia
        // final: remover este painel inteiro quando os valores padrao em
        // WindSystem.hpp estiverem calibrados.
        if (m_showWindPanel && beginDebugPanel("Vento",
                &m_showWindPanel,
                { io.DisplaySize.x - panelWidth - ui(12.0f),
                    panelTop + ui(72.0f) },
                { panelWidth, ui(430.0f) }, io.DisplaySize, m_uiScale)) {
            {
                    WindSettings3D& wind = m_windSystem.mutableSettings();
                    const Vec3 currentWind = m_windSystem.velocityAtHeight(
                        wind.referenceHeightMeters);
                    const float currentSpeed = currentWind.length();
                    const float currentHeadingDegrees =
                        currentSpeed > 1.0e-4f
                            ? std::atan2(currentWind.y, currentWind.x)
                                * (180.0f / Pi)
                            : 0.0f;
                    panelHeader("VENTO (debug, remover depois dos testes)");
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Direção atual: %.0f°  |  Velocidade: %.2f m/s",
                        currentHeadingDegrees, currentSpeed);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Velocidade base (m/s)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindBaseSpeed",
                        &wind.baseSpeedMetersPerSecond,
                        0.0f, 15.0f, "%.2f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Amplitude de rajada (m/s)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindGustAmplitude",
                        &wind.gustAmplitudeMetersPerSecond,
                        0.0f, 10.0f, "%.2f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Frequência de rajada (Hz)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindGustFrequency",
                        &wind.gustFrequencyHz,
                        0.0f, 0.5f, "%.4f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Frequência de vagueio de direção (Hz)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindDirectionWander",
                        &wind.directionWanderFrequencyHz,
                        0.0f, 0.2f, "%.4f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    ImGui::TextColored({ 0.45f, 0.55f, 0.65f, 1.0f },
                        "Envelope de calmaria");
                    ImGui::Dummy({ 1.0f, ui(4.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Frequência do envelope (Hz)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindCalmEnvelopeFrequency",
                        &wind.calmEnvelopeFrequencyHz,
                        0.0001f, 0.02f, "%.5f",
                        ImGuiSliderFlags_Logarithmic
                            | ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Piso de calmaria (fração do pico)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindCalmEnvelopeFloor",
                        &wind.calmEnvelopeFloor,
                        0.0f, 1.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    ImGui::TextColored({ 0.45f, 0.55f, 0.65f, 1.0f },
                        "Resquício de rajada no chão");
                    ImGui::Dummy({ 1.0f, ui(4.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Frequência do evento (Hz)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindGroundGustEventFrequency",
                        &wind.groundGustEventFrequencyHz,
                        0.0001f, 0.02f, "%.5f",
                        ImGuiSliderFlags_Logarithmic
                            | ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Nitidez do evento (expoente)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindGroundGustSharpness",
                        &wind.groundGustEventSharpness,
                        1.0f, 20.0f, "%.1f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(7.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Resquício máximo (fração do pico)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindGroundGustResidueMax",
                        &wind.groundGustResidueMaxFraction,
                        0.0f, 1.0f, "%.3f",
                        ImGuiSliderFlags_AlwaysClamp);

                    ImGui::Dummy({ 1.0f, ui(12.0f) });
                    ImGui::TextColored({ 0.45f, 0.55f, 0.65f, 1.0f },
                        "Abrigo por parede (áudio)");
                    ImGui::Dummy({ 1.0f, ui(4.0f) });
                    ImGui::TextColored({ 0.50f, 0.62f, 0.72f, 1.0f },
                        "Distância do raycast (m)");
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::SliderFloat("##WindAudioShelterDistance",
                        &wind.audioShelterDistanceMeters,
                        0.0f, 20.0f, "%.2f",
                        ImGuiSliderFlags_AlwaysClamp);
            }
            growPanelToFitContent(io.DisplaySize, m_uiScale);
        }
        if (m_showWindPanel) ImGui::End();
    }

    if (m_spawnMenuOpen) {
        drawSpawnMenu();
    }
    if (m_laboratoryPaused) {
        drawLaboratoryPauseMenu();
    }
}

} // namespace MatterEngine::Workbench
