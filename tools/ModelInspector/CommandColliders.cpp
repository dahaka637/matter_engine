#include "Commands.hpp"

#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/RL/Calibration/RagdollColliderWireframe3D.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <optional>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

namespace {

void appendCross(Highlight& highlight, Vec3 point, float radius) {
    highlight.segments.emplace_back(point - Vec3 { radius, 0.0f, 0.0f }, point + Vec3 { radius, 0.0f, 0.0f });
    highlight.segments.emplace_back(point - Vec3 { 0.0f, radius, 0.0f }, point + Vec3 { 0.0f, radius, 0.0f });
    highlight.segments.emplace_back(point - Vec3 { 0.0f, 0.0f, radius }, point + Vec3 { 0.0f, 0.0f, radius });
}

Highlight& colliderHighlightFor(std::vector<Highlight>& highlights, RagdollColliderWireframeSide3D side) {
    switch (side) {
        case RagdollColliderWireframeSide3D::Center: return highlights[0];
        case RagdollColliderWireframeSide3D::Left: return highlights[1];
        case RagdollColliderWireframeSide3D::Right: return highlights[2];
    }
    return highlights[0];
}

struct PhysicalPoseRequest {
    std::optional<AnimationClip3D> clip;
    float timeSeconds = 0.0f;
};

std::optional<PhysicalPoseRequest> physicalPoseFromSpec(const std::string& fullSpec, std::string* error) {
    std::string spec = fullSpec;
    if (const std::size_t hand = spec.find('#'); hand != std::string::npos) spec = spec.substr(0, hand);
    if (spec.rfind("dobra:", 0) == 0) {
        if (error)
            *error = "colisores: dobra: altera apenas o esqueleto visual; use um "
                     "clipe fisico para auditar os colliders";
        return std::nullopt;
    }

    const std::size_t at = spec.find('@');
    const std::string clipId = spec.substr(0, at);
    if (clipId == "ligacao" || clipId == "bind") {
        if (error)
            *error = "colisores: ligacao e a pose A de autoria; use repouso para a "
                     "pose fisica T";
        return std::nullopt;
    }

    PhysicalPoseRequest request;
    if (at != std::string::npos) {
        std::string value = spec.substr(at + 1);
        for (char& character : value)
            if (character == ',') character = '.';
        char* end = nullptr;
        request.timeSeconds = std::strtof(value.c_str(), &end);
        if (end == value.c_str() || *end != '\0') {
            if (error) *error = "colisores: instante invalido em --pose " + fullSpec;
            return std::nullopt;
        }
    }
    if (clipId.empty() || clipId == "repouso") return request;
    request.clip = loadCharacterClip(clipId, error);
    if (!request.clip) return std::nullopt;
    return request;
}

std::vector<Highlight> colliderHighlights(const RagdollColliderWireframe3D& wireframe) {
    std::vector<Highlight> highlights(6);
    highlights[0].color = Palette::Center;
    highlights[1].color = Palette::Left;
    highlights[2].color = Palette::Right;
    highlights[3].color = rgb(225, 70, 210); // centros articulares
    highlights[4].color = rgb(145, 155, 168); // origens dos links
    highlights[5].color = Palette::Warning; // centros de massa
    for (const RagdollColliderWireframeLink3D& link : wireframe.links) {
        Highlight& collider = colliderHighlightFor(highlights, link.side);
        collider.segments.insert(collider.segments.end(), link.segments.begin(), link.segments.end());
        if (link.hasInboundJoint) highlights[3].points.push_back(link.inboundJointModel);
        appendCross(highlights[4], link.linkOriginModel, 0.012f);
        highlights[5].points.push_back(link.centerOfMassModel);
    }
    return highlights;
}

Box3 boundsIncludingHighlights(
    const InspectionScene& scene, std::span<const Highlight> highlights, int overviewRegion) {
    Box3 bounds = regionBounds(scene, overviewRegion);
    for (const Highlight& highlight : highlights) {
        for (const auto& [a, b] : highlight.segments) {
            bounds.include(a);
            bounds.include(b);
        }
        for (const Vec3 point : highlight.points) bounds.include(point);
    }
    return bounds;
}

// Com --link: o colisor daquele osso, com uma folga (as malhas em volta
// entram no quadro so ate ela).
std::optional<Box3> linkBounds(const RagdollColliderWireframe3D& wireframe, const std::string& link, float margin) {
    if (link.empty()) return std::nullopt;
    for (const RagdollColliderWireframeLink3D& entry : wireframe.links) {
        if (entry.linkId != link) continue;
        Box3 bounds;
        for (const auto& [a, b] : entry.segments) {
            bounds.include(a);
            bounds.include(b);
        }
        bounds.include(bounds.minimum - Vec3 { margin, margin, margin });
        bounds.include(bounds.maximum + Vec3 { margin, margin, margin });
        return bounds;
    }
    return std::nullopt;
}

// A malha do rig fica no espaco da modelagem (o chao fixo dela) e o perfil no
// do jogo (a pelve na origem): o wireframe vai para o da malha pela pelve
// (sem isso, depois de uma exportacao que muda a altura de pe, os colisores
// apareciam deslocados da malha).
RagdollColliderWireframe3D alignedToRig(const RagdollColliderWireframe3D& wireframe, const GameRig& rig,
    const SkeletonPose& pose, Vec3 rootLink) {
    for (std::size_t j = 0; j < rig.link.size() && j < pose.joints.size(); ++j) {
        if (rig.link[j] != 0) continue;
        const Vec3 delta = pose.joints[j].position - rootLink;
        RagdollColliderWireframe3D moved = wireframe;
        for (RagdollColliderWireframeLink3D& link : moved.links) {
            for (auto& [a, b] : link.segments) {
                a = a + delta;
                b = b + delta;
            }
            link.linkOriginModel = link.linkOriginModel + delta;
            link.colliderCenterModel = link.colliderCenterModel + delta;
            link.centerOfMassModel = link.centerOfMassModel + delta;
            link.inboundJointModel = link.inboundJointModel + delta;
        }
        return moved;
    }
    return wireframe;
}

} // namespace

int runColliders(const ModelContext& context, const CommandLine& line) {
    // Sem --perfil, o do personagem carregado (o do corpo, com --corpo).
    const RagdollProfile3D profile =
        line.has("perfil") ? loadRagdollProfile3D(line.text("perfil")) : context.loaded.character->profile;
    RagdollColliderWireframe3D wireframe = buildRagdollColliderWireframe3D(
        profile, std::clamp(line.integer("radiais", 16), 8, 128), std::clamp(line.integer("tampas", 4), 2, 32));

    const std::vector<std::string> poseSpecs = line.list("poses");
    if (!poseSpecs.empty()) {
        if (line.has("pose")) {
            std::cerr << "colisores: use --pose para uma pose ou --poses para a "
                         "matriz, nao ambos\n";
            return 2;
        }
        std::vector<std::string> views = line.list("vistas");
        if (views.empty()) views = { "frente", "direita", "cima" };
        const int panelWidth = line.integer("largura", 520);
        const int panelHeight = line.integer("altura", 640);
        std::vector<Image> images;
        std::vector<std::string> captions;
        images.reserve(poseSpecs.size() * views.size());
        captions.reserve(poseSpecs.size() * views.size());
        for (const std::string& poseSpec : poseSpecs) {
            std::string error;
            const std::optional<PhysicalPoseRequest> poseRequest = physicalPoseFromSpec(poseSpec, &error);
            if (!poseRequest) {
                std::cerr << error << '\n';
                return 2;
            }
            const RagdollAnimationPose3D physicalPose = sampleRagdollAnimationPose3D(
                profile, poseRequest->clip ? &*poseRequest->clip : nullptr, poseRequest->timeSeconds, true);
            const SkeletonPose visualPose = sampleGameRigPose(*context.loaded.character, *context.loaded.rig,
                poseRequest->clip ? &*poseRequest->clip : nullptr, poseRequest->timeSeconds, RigHandPose::Relaxed,
                true);
            const RagdollColliderWireframe3D posed = poseRagdollColliderWireframe3D(
                profile, wireframe, physicalPose.linkPositions, physicalPose.linkOrientations, true);
            const RagdollColliderWireframe3D posedWireframe = alignedToRig(posed, *context.loaded.rig, visualPose,
                posed.links.front().linkOriginModel);
            std::vector<Vec3> posedMesh;
            if (context.model().mesh.hasSkin())
                posedMesh =
                    context.model().mesh.posedPositions(skinningTransforms(context.loaded.rig->skeleton, visualPose));
            const std::vector<Highlight> highlights = colliderHighlights(posedWireframe);
            InspectionScene scene = context.scene();
            scene.pose = &visualPose;
            scene.meshPositions = posedMesh;
            scene.highlights = highlights;
            const Box3 completeBounds = linkBounds(posedWireframe, line.text("link", ""), 0.04f)
                                            .value_or(boundsIncludingHighlights(scene, highlights, context.region("visao_geral")));

            for (const std::string& view : views) {
                CommandLine panelLine = line;
                panelLine.options["regiao"] = "visao_geral";
                panelLine.options["vista"] = view;
                panelLine.options["modo"] = line.text("modo", "raiox");
                panelLine.options["orto"] = "1";
                panelLine.options["sem-hud"] = "1";
                panelLine.options["sem-ossos"] = "1";
                panelLine.options["largura"] = std::to_string(panelWidth);
                panelLine.options["altura"] = std::to_string(panelHeight);
                const std::optional<ViewRequest> viewRequest =
                    viewFromOptions(context, panelLine, panelWidth, panelHeight);
                if (!viewRequest) return 2;
                InspectionStyle style = viewRequest->style;
                style.skeleton = false;
                style.joints = false;
                OrbitCamera camera = viewRequest->camera;
                frameBox(
                    camera, completeBounds, static_cast<float>(panelWidth) / static_cast<float>(panelHeight), 1.12f);
                images.push_back(renderInspection(scene, style, camera, panelWidth, panelHeight));
                captions.push_back(poseSpec + " · " + view);
            }
        }
        const int columns =
            std::clamp(line.integer("colunas", static_cast<int>(views.size())), 1, static_cast<int>(images.size()));
        const std::string title = "Matriz de colliders " + profile.id
            + " · esquerda azul · direita "
              "laranja · centro branco · COM amarelo · juntas magenta";
        return saveImage(composeSheet(images, captions, columns, title), context.output(line, "colliders_matrix.png"))
            ? 0
            : 1;
    }

    if (context.pose) {
        std::string error;
        const std::optional<PhysicalPoseRequest> request = physicalPoseFromSpec(context.poseLabel, &error);
        if (!request) {
            std::cerr << error << '\n';
            return 2;
        }
        const RagdollAnimationPose3D physicalPose = sampleRagdollAnimationPose3D(
            profile, request->clip ? &*request->clip : nullptr, request->timeSeconds, true);
        wireframe = poseRagdollColliderWireframe3D(
            profile, wireframe, physicalPose.linkPositions, physicalPose.linkOrientations, true);
    }

    // Sem --pose, converte a pose A de autoria para a pose T fisica. Com um
    // clipe, o ModelContext ja fornece exatamente a mesma amostra visual usada
    // acima para os links do ragdoll.
    std::optional<SkeletonPose> rest;
    std::vector<Vec3> restMesh;
    InspectionScene scene = context.scene();
    if (!context.pose) {
        rest =
            sampleGameRigPose(*context.loaded.character, *context.loaded.rig, nullptr, 0.0f, RigHandPose::Bind, true);
        restMesh = context.model().mesh.posedPositions(skinningTransforms(context.loaded.rig->skeleton, *rest));
        scene.pose = &*rest;
        scene.meshPositions = restMesh;
    }
    if (scene.pose) wireframe = alignedToRig(wireframe, *context.loaded.rig, *scene.pose, wireframe.links.front().linkOriginModel);
    const std::vector<Highlight> highlights = colliderHighlights(wireframe);
    scene.highlights = highlights;
    const Box3 completeBounds = linkBounds(wireframe, line.text("link", ""), 0.04f)
                                    .value_or(boundsIncludingHighlights(scene, highlights, context.region("visao_geral")));
    std::vector<std::string> views = line.list("vistas");
    if (views.empty()) views = { "frente", "direita", "cima" };
    const int panelWidth = line.integer("largura", 620);
    const int panelHeight = line.integer("altura", 760);
    std::vector<SheetPanel> panels;
    panels.reserve(views.size());
    for (const std::string& view : views) {
        CommandLine panelLine = line;
        panelLine.options["regiao"] = "visao_geral";
        panelLine.options["vista"] = view;
        panelLine.options["modo"] = line.text("modo", "raiox");
        panelLine.options["orto"] = "1";
        panelLine.options["sem-hud"] = "1";
        panelLine.options["sem-ossos"] = "1";
        panelLine.options["largura"] = std::to_string(panelWidth);
        panelLine.options["altura"] = std::to_string(panelHeight);
        const std::optional<ViewRequest> request = viewFromOptions(context, panelLine, panelWidth, panelHeight);
        if (!request) return 2;
        InspectionStyle style = request->style;
        style.skeleton = false;
        style.joints = false;
        OrbitCamera camera = request->camera;
        frameBox(camera, completeBounds, static_cast<float>(panelWidth) / static_cast<float>(panelHeight), 1.12f);
        panels.push_back({ camera, style, view });
    }

    const int columns =
        std::clamp(line.integer("colunas", static_cast<int>(panels.size())), 1, static_cast<int>(panels.size()));
    const std::string title = "Colliders " + profile.id
        + (context.poseLabel.empty() ? " · repouso" : " · " + context.poseLabel)
        + " · esquerda azul · direita laranja · centro branco · COM amarelo · "
          "juntas magenta · origem cinza";
    const Image sheet = renderSheet(scene, panels, columns, panelWidth, panelHeight, title);
    return saveImage(sheet, context.output(line, "colliders.png")) ? 0 : 1;
}

} // namespace MatterEngine::ModelInspector
