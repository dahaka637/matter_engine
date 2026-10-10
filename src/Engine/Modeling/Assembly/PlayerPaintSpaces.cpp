#include "Engine/Modeling/Assembly/PlayerPaintSpaces.hpp"

#include "Engine/Modeling/Assembly/FacePaint.hpp"
#include "Engine/Modeling/Assembly/Uniform.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

Vec3 jointAt(const Skeleton& skeleton, const char* name) {
    const int joint = skeleton.find(name);
    return joint >= 0 ? skeleton.joints[static_cast<std::size_t>(joint)].bind.position : Vec3 {};
}

float distanceToSegment(Vec3 p, Vec3 a, Vec3 b) {
    const Vec3 ab = b - a;
    const float t = std::clamp(dot(p - a, ab) / std::max(1e-9f, dot(ab, ab)), 0.0f, 1.0f);
    return (p - (a + ab * t)).length();
}

// O ponto e a normal levados ao lado direito (y < 0) e o lado de onde vieram.
struct RightSide {
    Vec3 point;
    Vec3 normal;
    int side = -1;
};

RightSide toRight(const SurfaceSample& sample) {
    const bool left = sample.position.y > 0.0f;
    return { { sample.position.x, -std::abs(sample.position.y), sample.position.z },
        { sample.normal.x, left ? -sample.normal.y : sample.normal.y, sample.normal.z }, left ? 1 : -1 };
}

// O angulo em volta de um eixo (0 em "zero", positivo para "toward"), em metros
// num raio tipico.
float around(Vec3 offset, Vec3 axis, Vec3 zero, Vec3 toward, float radius) {
    const Vec3 a = safeNormalized(zero - axis * dot(zero, axis), { 0.0f, 0.0f, 1.0f });
    const Vec3 b = safeNormalized(toward - axis * dot(toward, axis) - a * dot(toward, a), { 1.0f, 0.0f, 0.0f });
    return std::atan2(dot(offset, b), dot(offset, a)) * radius;
}

} // namespace

std::map<std::string, PaintSpace> playerPaintSpaces(const Mesh& mesh, const AssemblyRecipe& recipe, const Skeleton& skeleton) {
    std::map<std::string, PaintSpace> spaces;
    const UniformLines lines = uniformLines(recipe, skeleton);
    const float ground = skeleton.groundHeight;
    const int shirt = mesh.findPart("camisa"), shorts = mesh.findPart("calcao"), body = mesh.findPart("corpo"), head = mesh.findPart("cabeca");
    const Vec3 shoulder = jointAt(skeleton, "RightUpperArm"), elbow = jointAt(skeleton, "RightForearm");
    const Vec3 hip = jointAt(skeleton, "RightThigh"), knee = jointAt(skeleton, "RightShin"), ankle = jointAt(skeleton, "RightFoot");
    const auto sleeveAround = [=](Vec3 q) {
        return distanceToSegment(q, shoulder, elbow) < 0.1f && dot(q - lines.sleevePoint, lines.armAlong) > -0.12f;
    };
    // As linhas do tronco: a barra, a axila (a linha da receita do corpo
    // base, 1,27 m) e a borda da gola na frente (o fundo do V) e atras.
    std::map<std::string, float> torsoAnchors;
    torsoAnchors["barra"] = lines.shirtHem - ground;
    torsoAnchors["axila"] = lines.lifted(1.27f);
    torsoAnchors["gola_frente"] = lines.collarHeight({ lines.neckX + 0.1f, 0.0f, 0.0f }) - ground;
    torsoAnchors["gola_costas"] = lines.collarHeight({ lines.neckX - 0.1f, 0.0f, 0.0f }) - ground;

    if (shirt >= 0) {
        UnrolledSpaceSettings torso;
        torso.parts = { shirt };
        torso.origin = { 0.0f, 0.0f, ground };
        torso.include = [=](Vec3 center) { return !sleeveAround(center); };
        // So a camisa de fora: a dobra recuada por dentro da barra emenda a
        // volta de fora com a de dentro na fatia (o arco salta na barra).
        torso.outerOnly = true;
        // A costura ajustada longe da barra (o recuo) e da axila.
        torso.seamFrom = torsoAnchors["barra"] + 0.04f;
        torso.seamTo = torsoAnchors["axila"] - 0.04f;
        PaintSpace unrolled = unrolledSpace(mesh, torso);
        unrolled.anchors.insert(torsoAnchors.begin(), torsoAnchors.end());
        spaces["tronco"] = std::move(unrolled);

        PaintSpace front = planarSpace({ shirt }, { 0.0f, 0.0f, ground }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f }, 0.25f);
        front.anchors = torsoAnchors;
        spaces["frente"] = std::move(front);
        PaintSpace back = planarSpace({ shirt }, { 0.0f, 0.0f, ground }, { 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { -1.0f, 0.0f, 0.0f }, 0.25f);
        back.anchors = torsoAnchors;
        spaces["costas"] = std::move(back);

        PaintSpace sleeve;
        sleeve.locate = [=](const SurfaceSample& sample) -> std::optional<SpacePoint> {
            if (sample.part != shirt) return std::nullopt;
            const RightSide r = toRight(sample);
            if (!sleeveAround(r.point)) return std::nullopt;
            const Vec3 d = r.point - lines.sleevePoint;
            return SpacePoint { { around(d, lines.armAlong, { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f }, 0.06f), -dot(d, lines.armAlong) }, r.side,
                sample.footprint };
        };
        sleeve.anchors["barra"] = 0.0f;
        spaces["manga"] = std::move(sleeve);

        // A gola: a distancia ate a borda medida de lado a ela (o gradiente da
        // altura acerta a largura no V inclinado).
        PaintSpace collar;
        collar.locate = [=](const SurfaceSample& sample) -> std::optional<SpacePoint> {
            if (sample.part != shirt) return std::nullopt;
            const Vec3 q = toRight(sample).point;
            const float h = 1e-3f;
            const float f = lines.collarHeight(q) - q.z;
            const float gx = (lines.collarHeight(q + Vec3 { h, 0.0f, 0.0f }) - lines.collarHeight(q - Vec3 { h, 0.0f, 0.0f })) / (2.0f * h);
            const float gy = (lines.collarHeight(q + Vec3 { 0.0f, h, 0.0f }) - lines.collarHeight(q - Vec3 { 0.0f, h, 0.0f })) / (2.0f * h);
            const float below = f / std::sqrt(1.0f + gx * gx + gy * gy);
            if (below > 0.15f) return std::nullopt;
            return SpacePoint { { std::atan2(-q.y, q.x - lines.neckX) * 0.07f, below }, toRight(sample).side, sample.footprint };
        };
        collar.anchors["borda"] = 0.0f;
        spaces["gola"] = std::move(collar);
    }
    if (shorts >= 0) {
        UnrolledSpaceSettings leg;
        leg.parts = { shorts };
        leg.origin = hip;
        leg.axis = lines.legAlong;
        const float hem = dot(lines.legHemPoint - hip, lines.legAlong);
        leg.seamFrom = 0.06f;
        leg.seamTo = hem - 0.02f;
        leg.outerOnly = true;
        PaintSpace unrolled = unrolledSpace(mesh, leg);
        unrolled.anchors["barra"] = hem;
        spaces["calcao"] = std::move(unrolled);
        PaintSpace front = planarSpace({ shorts }, { 0.0f, 0.0f, ground }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f }, 0.25f);
        front.anchors["barra"] = lines.lifted(recipe.shortsHem);
        spaces["calcao_frente"] = std::move(front);
    }
    if (body >= 0) {
        // A pele das pernas (abaixo do joelho e um pouco acima).
        const float top = knee.z - ground + 0.12f;
        PaintSpace leg;
        leg.locate = [=](const SurfaceSample& sample) -> std::optional<SpacePoint> {
            if (sample.part != body) return std::nullopt;
            const RightSide r = toRight(sample);
            const float height = r.point.z - ground;
            if (height > top) return std::nullopt;
            const Vec3 axis = safeNormalized(knee - ankle, { 0.0f, 0.0f, 1.0f });
            const Vec3 offset = r.point - ankle;
            return SpacePoint { { around(offset - axis * dot(offset, axis), axis, { 1.0f, 0.0f, 0.0f }, { 0.0f, -1.0f, 0.0f }, 0.05f), height }, r.side,
                sample.footprint };
        };
        leg.anchors["joelho"] = knee.z - ground;
        spaces["perna"] = std::move(leg);
    }
    if (head >= 0) {
        // A orelha: a caixa da placa de fora medida na malha (earPlate).
        const EarPlate plate = earPlate(mesh, head, recipe.face.earOutside, ground);
        if (plate.outline.size() >= 3) {
            float minX = plate.outline.front()[0], maxX = minX, minH = plate.outline.front()[1], maxH = minH;
            for (const auto& p : plate.outline) {
                minX = std::min(minX, p[0]);
                maxX = std::max(maxX, p[0]);
                minH = std::min(minH, p[1]);
                maxH = std::max(maxH, p[1]);
            }
            const float width = std::max(1e-4f, maxX - minX), height = std::max(1e-4f, maxH - minH);
            const float outside = recipe.face.earOutside;
            PaintSpace ear;
            ear.locate = [=](const SurfaceSample& sample) -> std::optional<SpacePoint> {
                if (sample.part != head) return std::nullopt;
                const RightSide r = toRight(sample);
                if (-r.point.y < outside || r.normal.y > -0.35f) return std::nullopt;
                return SpacePoint { { (r.point.x - minX) / width, (r.point.z - ground - minH) / height }, r.side,
                    sample.footprint / std::sqrt(width * height) };
            };
            spaces["orelha"] = std::move(ear);
        }
    }
    return spaces;
}

} // namespace MatterEngine::Modeling
