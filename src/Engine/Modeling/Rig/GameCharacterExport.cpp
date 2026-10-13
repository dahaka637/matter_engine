#include "Engine/Modeling/Rig/GameCharacterExport.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Geometry/ConvexHull3D.hpp"
#include "Engine/Modeling/Mesh/ShadingNormals.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Rig/GameRig.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>

namespace MatterEngine::Modeling {

namespace {

using Json = nlohmann::json;

Json readJson(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("nao abriu " + path.string());
    return Json::parse(stream);
}

void writeJson(const std::filesystem::path& path, const Json& value) {
    std::ofstream stream(path);
    stream << value.dump(1) << '\n';
    if (!stream) throw std::runtime_error("nao gravou " + path.string());
}

float rounded(float value, float step = 1e-6f) { return std::round(value / step) * step; }

Json vector3(Vec3 v) { return Json::array({ rounded(v.x), rounded(v.y), rounded(v.z) }); }

Json quaternion4(Quaternion q) {
    // O sinal fixo (w >= 0): exportar de novo da os mesmos numeros.
    q = q.normalized();
    if (q.w < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
    return Json::array({ rounded(q.x, 1e-7f), rounded(q.y, 1e-7f), rounded(q.z, 1e-7f), rounded(q.w, 1e-7f) });
}

// O percentil como o do numpy (interpolado entre as amostras vizinhas).
float percentile(std::vector<float> values, float p) {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    const float at = std::clamp(p, 0.0f, 100.0f) / 100.0f * static_cast<float>(values.size() - 1);
    const auto low = static_cast<std::size_t>(std::floor(at));
    const std::size_t high = std::min(values.size() - 1, low + 1);
    return values[low] + (values[high] - values[low]) * (at - static_cast<float>(low));
}

// O giro que leva +x a "direction".
Quaternion rotationFromX(Vec3 direction) {
    const Vec3 x { 1.0f, 0.0f, 0.0f };
    const Vec3 d = safeNormalized(direction, x);
    const Vec3 axis = cross(x, d);
    const float s = axis.length(), c = dot(x, d);
    if (s < 1e-9f) return c > 0.0f ? Quaternion {} : Quaternion { 0.0f, 0.0f, 1.0f, 0.0f };
    return Quaternion::fromAxisAngle(axis / s, std::atan2(s, c));
}

// Um colisor medido, no espaco do modelo (o centro e o giro). A casca
// convexa (a chuteira) tambem tem a caixa (a justa dela, centrada).
struct Collider {
    bool box = false;
    bool hull = false;
    Vec3 center;
    Quaternion orientation;
    float length = 0.0f;
    float radius = 0.0f;
    float radiusPositive = 0.0f;
    bool tapered = false;
    Vec3 halfExtents;
    std::vector<Vec3> hullPoints;
    std::vector<std::array<std::uint16_t, 3>> hullFaces;
    float convexRadius = 0.005f;
};

// Membro (braco, antebraco, coxa, canela; a mao, o pescoco e a cabeca sem
// afinar): o eixo do osso deslocado pelo centro da malha no meio dele; os
// raios perto de cima e de baixo; as tampas ate as pontas da malha e pelo
// menos de uma junta a outra (a pele da dobra do joelho, do cotovelo e do
// pulso fica fora da medida: sem isso sobrava um vao nelas).
Collider fitLimb(Vec3 start, Vec3 end, const std::vector<Vec3>& points, float radiusPercentile, bool tapered,
    std::pair<float, float> distalBand) {
    Collider out;
    const Vec3 bone = end - start;
    const float length = std::max(1e-4f, bone.length());
    const Vec3 axis = bone / length;
    std::vector<float> along(points.size());
    std::vector<Vec3> radial(points.size());
    Vec3 offset {};
    int middle = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        along[i] = dot(points[i] - start, axis);
        radial[i] = (points[i] - start) - axis * along[i];
        if (along[i] > 0.25f * length && along[i] < 0.75f * length) {
            offset += radial[i];
            ++middle;
        }
    }
    if (middle > 0) offset = offset / static_cast<float>(middle);
    std::vector<float> distance(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) distance[i] = (radial[i] - offset).length();
    const auto band = [&](float low, float high) {
        std::vector<float> selected;
        for (std::size_t i = 0; i < points.size(); ++i)
            if (along[i] > low * length && along[i] < high * length) selected.push_back(distance[i]);
        return percentile(selected.empty() ? distance : selected, radiusPercentile);
    };
    const float low = std::min(percentile(along, 1.0f), 0.0f), high = std::max(percentile(along, 99.0f), length);
    const float proximal = tapered ? band(0.15f, 0.35f) : percentile(distance, radiusPercentile);
    const float distal = tapered ? band(distalBand.first, distalBand.second) : proximal;
    float first = low + proximal, last = high - distal;
    if (last <= first) first = last = 0.5f * (low + high);
    // Pelo menos um cilindro curto: o osso do pescoco e mais curto que o
    // diametro dele, e so as duas esferas o perfil recusa (o comprimento tem
    // de passar a soma dos raios; o arredondamento do arquivo decidia).
    constexpr float MinimumCylinder = 0.002f;
    out.center = start + offset + axis * (0.5f * (first + last));
    out.orientation = rotationFromX(axis);
    out.length = std::max(last - first, MinimumCylinder) + proximal + distal;
    out.radius = proximal;
    out.radiusPositive = distal;
    out.tapered = tapered;
    return out;
}

// O tronco (pelve, abdomen, peito): a capsula deitada do perfil (o x local
// e o lado do corpo), o raio a media da meia profundidade e da meia altura
// mas nunca mais que a meia profundidade (a faixa da pelve e alta: a media
// passava 3 cm do gluteo e da barriga), o comprimento a largura.
Collider fitTorso(Quaternion orientation, Vec3 center, const std::vector<Vec3>& points) {
    std::vector<float> axes[3];
    const Vec3 frame[3] = { orientation.rotate({ 1.0f, 0.0f, 0.0f }), orientation.rotate({ 0.0f, 1.0f, 0.0f }),
        orientation.rotate({ 0.0f, 0.0f, 1.0f }) };
    for (const Vec3& p : points)
        for (int a = 0; a < 3; ++a) axes[a].push_back(dot(p - center, frame[a]));
    float half[3], mid[3];
    for (int a = 0; a < 3; ++a) {
        const float low = percentile(axes[a], 3.0f), high = percentile(axes[a], 97.0f);
        half[a] = 0.5f * (high - low);
        mid[a] = 0.5f * (high + low);
    }
    // No meio do corpo de um lado ao outro.
    mid[0] = 0.0f;
    // A profundidade: o eixo (y ou z local) mais perto da frente do corpo.
    const float depth = std::abs(frame[1].x) >= std::abs(frame[2].x) ? half[1] : half[2];
    Collider out;
    out.radius = std::min(0.5f * (half[1] + half[2]), depth);
    out.length = std::max(2.0f * out.radius + 0.012f, 2.0f * half[0]);
    out.center = center + frame[0] * mid[0] + frame[1] * mid[1] + frame[2] * mid[2];
    out.orientation = orientation;
    return out;
}

// O pe: a caixa da chuteira (o comprimento e a largura no rumo dela, a
// altura a do perfil, o fundo no chao do repouso).
Collider fitFoot(const std::vector<Vec3>& points, float halfHeight, float ground) {
    // O rumo: o eixo maior dos pontos vistos de cima.
    Vec3 mean {};
    for (const Vec3& p : points) mean += p;
    mean = mean / static_cast<float>(std::max<std::size_t>(1, points.size()));
    float xx = 0.0f, xy = 0.0f, yy = 0.0f;
    for (const Vec3& p : points) {
        xx += (p.x - mean.x) * (p.x - mean.x);
        xy += (p.x - mean.x) * (p.y - mean.y);
        yy += (p.y - mean.y) * (p.y - mean.y);
    }
    float yaw = 0.5f * std::atan2(2.0f * xy, xx - yy);
    if (std::cos(yaw) < 0.0f) yaw += 3.14159265f;
    const Quaternion orientation = Quaternion::fromAxisAngle({ 0.0f, 0.0f, 1.0f }, yaw);
    const Vec3 forward = orientation.rotate({ 1.0f, 0.0f, 0.0f }), side = orientation.rotate({ 0.0f, 1.0f, 0.0f });
    std::vector<float> a, b;
    for (const Vec3& p : points) {
        a.push_back(dot(p - mean, forward));
        b.push_back(dot(p - mean, side));
    }
    const float a0 = percentile(a, 1.0f), a1 = percentile(a, 99.0f), b0 = percentile(b, 1.0f), b1 = percentile(b, 99.0f);
    Collider out;
    out.box = true;
    out.orientation = orientation;
    out.halfExtents = { 0.5f * (a1 - a0), 0.5f * (b1 - b0), halfHeight };
    out.center = mean + forward * (0.5f * (a0 + a1)) + side * (0.5f * (b0 + b1));
    out.center.z = ground + halfHeight;
    return out;
}

// O pe: a casca convexa da chuteira (pedido do usuario, 12/10: "cada cantinho
// do pe em contato com a bola importa"). O rumo e o da caixa de antes (o eixo
// maior visto de cima); a meia acima de BootTop fica de fora; a sola e plana
// no chao, no contorno da chuteira (o que fica abaixo de SoleBand desce ao
// chao: as travas e a placa; a ponta, que sobe, fica como e); a casca para a
// HullTolerance da exata. O centro e o da caixa justa dela.
constexpr float BootTop = 0.13f, SoleBand = 0.025f, HullTolerance = 0.0015f, BootConvexRadius = 0.005f;
Collider fitBoot(const std::vector<Vec3>& points, float ground, std::vector<std::string>& lines) {
    const Collider guide = fitFoot(points, 0.05f, ground);
    const Quaternion inverse = guide.orientation.conjugate();
    std::vector<Vec3> local;
    local.reserve(points.size());
    for (const Vec3& p : points) {
        if (p.z > ground + BootTop) continue;
        Vec3 q = inverse.rotate(Vec3 { p.x - guide.center.x, p.y - guide.center.y, p.z - ground });
        if (q.z < SoleBand) q.z = 0.0f;
        local.push_back(q);
    }
    if (local.size() < 64) throw std::runtime_error("a chuteira com poucos pontos para medir");
    ConvexHull3D shape = buildConvexHull3D(local, HullTolerance, 128);
    Vec3 low = shape.points.front(), high = low;
    for (const Vec3& q : shape.points) {
        low = { std::min(low.x, q.x), std::min(low.y, q.y), std::min(low.z, q.z) };
        high = { std::max(high.x, q.x), std::max(high.y, q.y), std::max(high.z, q.z) };
    }
    const Vec3 middle = (low + high) * 0.5f;
    for (Vec3& q : shape.points) q = q - middle;
    // A medida: quanto da chuteira fica fora da casca (a tolerancia, no maximo).
    float worst = 0.0f;
    for (const Vec3& q : local) worst = std::max(worst, convexHullOutside3D(shape, q - middle));
    Collider out;
    out.box = true;
    out.hull = true;
    out.orientation = guide.orientation;
    out.center = Vec3 { guide.center.x, guide.center.y, ground } + guide.orientation.rotate(middle);
    out.halfExtents = (high - low) * 0.5f;
    out.hullPoints = std::move(shape.points);
    out.hullFaces = std::move(shape.faces);
    out.convexRadius = BootConvexRadius;
    char line[256];
    std::snprintf(line, sizeof line,
        "chuteira: %zu pontos medidos, casca de %zu vertices e %zu faces, %.1f x %.1f x %.1f cm; a malha fica no maximo "
        "%.1f mm fora", local.size(), out.hullPoints.size(), out.hullFaces.size(), out.halfExtents.x * 200.0f,
        out.halfExtents.y * 200.0f, out.halfExtents.z * 200.0f, worst * 1000.0f);
    lines.push_back(line);
    return out;
}

// O eixo de uma capsula: os centros das esferas das pontas e os raios delas.
struct Segment {
    Vec3 a, b;
    float ra, rb;
};

Segment capsuleSegment(const Collider& c) {
    const Vec3 axis = c.orientation.rotate({ 1.0f, 0.0f, 0.0f });
    const float rb = c.tapered ? c.radiusPositive : c.radius;
    const float half = std::max(0.0f, 0.5f * (c.length - c.radius - rb));
    return { c.center - axis * half, c.center + axis * half, c.radius, rb };
}

// A folga entre duas capsulas (a menor distancia entre as superficies,
// amostrada ao longo dos dois eixos) e onde ela fica em cada eixo (0 a
// ponta -x, 1 a +x).
struct Contact {
    float gap = std::numeric_limits<float>::max();
    float u = 0.0f;
    float v = 0.0f;
};

Contact capsuleContact(const Collider& first, const Collider& second) {
    const Segment s = capsuleSegment(first), t = capsuleSegment(second);
    Contact best;
    for (int i = 0; i <= 40; ++i)
        for (int j = 0; j <= 40; ++j) {
            const float u = static_cast<float>(i) / 40.0f, v = static_cast<float>(j) / 40.0f;
            const Vec3 p = s.a + (s.b - s.a) * u, q = t.a + (t.b - t.a) * v;
            const float gap = (p - q).length() - (s.ra + (s.rb - s.ra) * u) - (t.ra + (t.rb - t.ra) * v);
            if (gap < best.gap) best = { gap, u, v };
        }
    return best;
}

float capsuleGap(const Collider& first, const Collider& second) { return capsuleContact(first, second).gap; }

// O colisor (medido no repouso) com o link numa pose.
Collider posedCollider(const Collider& rest, const Transform& linkRest, const Transform& linkNow) {
    const Transform move = linkNow * linkRest.inverse();
    Collider out = rest;
    out.center = move.apply(rest.center);
    out.orientation = (move.orientation * rest.orientation).normalized();
    return out;
}

// Encolhe a ponta de um membro (o raio dela; os centros das esferas das
// pontas ficam: o centro da capsula e o meio deles) ate "floor"; a mao
// (uniforme), as duas. Devolve quanto encolheu.
float shrinkLimbEnd(Collider& c, bool proximal, float by, float floor) {
    if (!c.tapered) {
        const float delta = std::clamp(c.radius - floor, 0.0f, by);
        c.radius -= delta;
        c.radiusPositive = c.radius;
        c.length -= 2.0f * delta;
        return delta;
    }
    float& radius = proximal ? c.radius : c.radiusPositive;
    const float delta = std::clamp(radius - floor, 0.0f, by);
    radius -= delta;
    c.length -= delta;
    return delta;
}

// Estreita o tronco (a capsula deitada: as duas pontas de lado) ate "floor"
// de comprimento. Devolve quanto cada ponta entrou.
float shrinkTorsoWidth(Collider& c, float by, float floor) {
    const float length = std::max(floor, c.length - 2.0f * by);
    const float delta = 0.5f * std::max(0.0f, c.length - length);
    c.length = std::max(c.length - 2.0f * delta, length);
    return delta;
}

// O colisor de um link do perfil, no espaco do perfil.
Collider profileCollider(const RagdollLinkDefinition3D& link) {
    const RagdollCapsuleDefinition3D& c = link.collider;
    const Quaternion frame = link.modelOrientation.normalized();
    Collider out;
    out.box = ragdollColliderHasBox3D(c);
    out.hull = c.shape == RagdollColliderShape3D::ConvexHull;
    out.hullPoints = c.hullPoints;
    out.hullFaces = c.hullFaces;
    out.convexRadius = c.convexRadiusMeters;
    out.center = link.modelPosition + frame.rotate(c.localPosition);
    out.orientation = (frame * c.localOrientation).normalized();
    out.length = c.lengthMeters;
    out.radius = c.radiusMeters;
    out.tapered = c.radiusAtPositiveXMeters > 0.0f;
    out.radiusPositive = out.tapered ? c.radiusAtPositiveXMeters : c.radiusMeters;
    out.halfExtents = c.boxHalfExtents;
    return out;
}

// Uma linha do colisor (o de antes e o novo, lado a lado).
std::string describe(const std::string& id, const RagdollCapsuleDefinition3D& before, const Collider& now) {
    char line[256];
    if (now.hull) {
        std::snprintf(line, sizeof line, "%-14s casca %zu vertices, %4.1f x %4.1f x %4.1f cm (antes %4.1f x %4.1f x %4.1f), rumo %+.1f graus",
            id.c_str(), now.hullPoints.size(), now.halfExtents.x * 200.0f, now.halfExtents.y * 200.0f, now.halfExtents.z * 200.0f,
            before.boxHalfExtents.x * 200.0f, before.boxHalfExtents.y * 200.0f, before.boxHalfExtents.z * 200.0f,
            2.0f * std::atan2(now.orientation.z, now.orientation.w) * 57.29578f);
    } else if (now.box) {
        std::snprintf(line, sizeof line, "%-14s caixa %4.1f x %4.1f x %4.1f cm (antes %4.1f x %4.1f x %4.1f), rumo %+.1f graus", id.c_str(),
            now.halfExtents.x * 200.0f, now.halfExtents.y * 200.0f, now.halfExtents.z * 200.0f, before.boxHalfExtents.x * 200.0f,
            before.boxHalfExtents.y * 200.0f, before.boxHalfExtents.z * 200.0f,
            2.0f * std::atan2(now.orientation.z, now.orientation.w) * 57.29578f);
    } else {
        const float beforeEnd = before.radiusAtPositiveXMeters > 0.0f ? before.radiusAtPositiveXMeters : before.radiusMeters;
        std::snprintf(line, sizeof line, "%-14s r %4.1f -> %4.1f cm, %5.1f cm (antes r %4.1f -> %4.1f, %5.1f)", id.c_str(), now.radius * 100.0f,
            (now.tapered ? now.radiusPositive : now.radius) * 100.0f, now.length * 100.0f, before.radiusMeters * 100.0f, beforeEnd * 100.0f,
            before.lengthMeters * 100.0f);
    }
    return line;
}

} // namespace

bool exportGameCharacter(const LoadedPlayerModel& loaded, const std::filesystem::path& outputDirectory,
    const GameCharacterExportSettings& settings, GameCharacterExportReport& report, std::string& error) {
    try {
        if (!loaded.character || !loaded.rig || !loaded.model || loaded.model->mesh.empty() || !loaded.model->mesh.hasSkin())
            throw std::runtime_error("precisa do modelo montado (malha com skin) e do personagem do jogo");
        if (!loaded.rig->fitted) throw std::runtime_error("o esqueleto precisa ser o medido (model.json \"skeleton\".\"fit\")");
        const RagdollCharacter3D& character = *loaded.character;
        const GameRig& rig = *loaded.rig;
        const Skeleton& skeleton = rig.skeleton;
        const Mesh& mesh = loaded.model->mesh;
        const RagdollProfile3D& profile = character.profile;
        if (mesh.textures.empty() || !mesh.hasUv()) throw std::runtime_error("o modelo precisa do atlas (montagem com textura)");
        if (profile.links.empty() || profile.links.front().parentIndex >= 0) throw std::runtime_error("o perfil sem a raiz primeiro");
        std::filesystem::create_directories(outputDirectory);
        char line[256];

        // ---- O repouso: a pose T do esqueleto medido e a malha nela ----
        const SkeletonPose rest = sampleGameRigPose(character, rig, nullptr, 0.0f, RigHandPose::Bind, true);
        const std::vector<Vec3> restMesh = mesh.posedPositions(skinningTransforms(skeleton, rest));
        const auto restOf = [&](const std::string& name) -> std::optional<Vec3> {
            const int joint = skeleton.find(name);
            if (joint < 0) return std::nullopt;
            return rest.joints[static_cast<std::size_t>(joint)].position;
        };

        // A junta de cada link e o osso da paleta nova de cada junta: os
        // fisicos na ordem dos links, depois os visuais, depois os auxiliares.
        const std::size_t links = profile.links.size();
        std::vector<int> linkJoint(links, -1);
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
            if (rig.link[j] >= 0) linkJoint[static_cast<std::size_t>(rig.link[j])] = static_cast<int>(j);
        for (std::size_t i = 0; i < links; ++i)
            if (linkJoint[i] < 0) throw std::runtime_error("o link " + profile.links[i].id + " sem junta no esqueleto");
        std::vector<int> helpers;
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
            if (skeleton.joints[j].kind == JointKind::Helper) helpers.push_back(static_cast<int>(j));
        const std::size_t visuals = character.visualBones.size();
        const std::size_t physicalPalette = character.boneLinkIndices.size();
        std::vector<int> palette(skeleton.joints.size(), -1);
        for (std::size_t i = 0; i < links; ++i) palette[static_cast<std::size_t>(linkJoint[i])] = static_cast<int>(i);
        for (std::size_t v = 0; v < visuals; ++v) {
            const int joint = rig.paletteJoint[physicalPalette + v];
            if (joint < 0) throw std::runtime_error("o osso visual " + character.visualBones[v].name + " sem junta");
            palette[static_cast<std::size_t>(joint)] = static_cast<int>(links + v);
        }
        for (std::size_t h = 0; h < helpers.size(); ++h) palette[static_cast<std::size_t>(helpers[h])] = static_cast<int>(links + visuals + h);
        // As pontas: o osso do pai.
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
            for (int at = static_cast<int>(j); palette[j] < 0 && at >= 0;) {
                at = skeleton.joints[static_cast<std::size_t>(at)].parent;
                if (at >= 0) palette[j] = palette[static_cast<std::size_t>(at)];
            }

        // ---- Os links: no mesmo lugar relativo do perfil atual, na escala
        // do osso (da junta ate a junta do filho; na mao e no pe, ate o dedo
        // medio e os dedos do pe; a cabeca, na do pescoco). Exportar de novo
        // no mesmo corpo deixa tudo onde esta ----
        // O repouso de um osso visual do perfil atual (o caminho ate o link).
        const auto oldVisual = [&](const std::string& name) -> std::optional<Vec3> {
            for (std::size_t v = 0; v < visuals; ++v) {
                const RagdollVisualBone3D& bone = character.visualBones[v];
                if (bone.name != name) continue;
                Vec3 position = bone.localBindPosition;
                std::size_t parent = bone.parentBone;
                while (parent >= physicalPalette) {
                    const RagdollVisualBone3D& up = character.visualBones[parent - physicalPalette];
                    position = up.localBindPosition + up.localBindOrientation.rotate(position);
                    parent = up.parentBone;
                }
                const auto& link = profile.links[character.boneLinkIndices[parent]];
                return link.modelPosition + link.modelOrientation.rotate(position);
            }
            return std::nullopt;
        };
        const auto oldJoint = [&](std::size_t i) {
            const auto& link = profile.links[i];
            return link.parentIndex >= 0 ? link.inboundJoint.anchorModelPosition : link.modelPosition;
        };
        const auto firstChild = [&](std::size_t i) -> int {
            for (std::size_t c = 0; c < links; ++c)
                if (profile.links[c].parentIndex == static_cast<int>(i)) return static_cast<int>(c);
            return -1;
        };
        const auto restJoint = [&](std::size_t link) { return rest.joints[static_cast<std::size_t>(linkJoint[link])].position; };
        std::vector<Vec3> linkPosition(links), anchor(links);
        std::vector<float> scale(links, 1.0f);
        for (std::size_t i = 0; i < links; ++i) {
            const auto& link = profile.links[i];
            const Vec3 nowJoint = restJoint(i);
            std::optional<Vec3> oldEnd, nowEnd;
            const std::string side = link.id.rfind("Left", 0) == 0 ? "Left" : link.id.rfind("Right", 0) == 0 ? "Right" : "";
            if (const int child = firstChild(i); child >= 0) {
                oldEnd = oldJoint(static_cast<std::size_t>(child));
                nowEnd = restJoint(static_cast<std::size_t>(child));
            } else if (link.parentIndex >= 0 && link.id == "Head") {
                // A cabeca nao tem ponta no perfil: a escala do pescoco.
                const auto neck = static_cast<std::size_t>(link.parentIndex);
                const float before = (oldJoint(i) - oldJoint(neck)).length(), now = (nowJoint - restJoint(neck)).length();
                if (before > 1e-4f) scale[i] = now / before;
            } else if (link.id.ends_with("Hand")) {
                oldEnd = oldVisual(side + "Middle1");
                nowEnd = restOf(side + "Middle1");
            } else if (link.id.ends_with("Foot")) {
                oldEnd = oldVisual(side + "Toe");
                nowEnd = restOf(side + "Toe");
            }
            if (oldEnd && nowEnd) {
                const float before = (*oldEnd - oldJoint(i)).length(), now = (*nowEnd - nowJoint).length();
                if (before > 1e-4f) scale[i] = now / before;
            }
            linkPosition[i] = nowJoint + (link.modelPosition - oldJoint(i)) * scale[i];
            anchor[i] = nowJoint;
        }

        // ---- Os colisores medidos na malha em repouso ----
        // A parte de cada link em cada vertice (os dedos contam para a mao e
        // para o pe; os auxiliares, metade para cada lado).
        std::vector<int> jointLink(skeleton.joints.size(), -1);
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
            int at = static_cast<int>(j);
            while (at >= 0 && rig.link[static_cast<std::size_t>(at)] < 0) at = skeleton.joints[static_cast<std::size_t>(at)].parent;
            if (at >= 0) jointLink[j] = rig.link[static_cast<std::size_t>(at)];
        }
        std::vector<float> vertexShare(mesh.positions.size() * links, 0.0f);
        for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
            float* share = &vertexShare[v * links];
            const SkinWeights& weights = mesh.skin[v];
            for (std::size_t k = 0; k < weights.weights.size(); ++k) {
                if (weights.weights[k] <= 0.0f || weights.joints[k] >= skeleton.joints.size()) continue;
                const Joint& joint = skeleton.joints[weights.joints[k]];
                if (joint.kind == JointKind::Helper && joint.derivedFrom >= 0 && joint.derivedTo >= 0) {
                    const int upper = jointLink[static_cast<std::size_t>(joint.derivedFrom)];
                    const int lower = jointLink[static_cast<std::size_t>(joint.derivedTo)];
                    if (upper >= 0) share[upper] += 0.5f * weights.weights[k];
                    if (lower >= 0) share[lower] += 0.5f * weights.weights[k];
                } else if (const int link = jointLink[weights.joints[k]]; link >= 0) {
                    share[link] += weights.weights[k];
                }
            }
        }
        // Os pontos de cada link: espalhados por igual na superficie (a malha
        // low poly tem poucos vertices - o tronco, uma volta na cintura e
        // outra no peito). Cada um e do link que mais pesa nele quando esse
        // peso passa de 0,6 (a pele de transicao - a axila, a virilha, a
        // ponta do ombro - fica com os dois vizinhos e com nenhum colisor);
        // o tronco junta os quatro links dele (o peso deles passando de 0,7)
        // e corta em faixas de altura entre as juntas: o skin dele espalha o
        // peso (nenhum ponto e do peito).
        const auto isTorso = [&](std::size_t i) {
            const std::string& id = profile.links[i].id;
            return id == "Pelvis" || id == "Abdomen" || id == "Chest" || id == "UpperChest";
        };
        std::vector<std::vector<Vec3>> linkPoints(links);
        std::vector<Vec3> torso;
        {
            constexpr float Spacing = 0.004f, OwnShare = 0.6f, TorsoShare = 0.7f;
            std::vector<float> share(links);
            for (const Triangle& triangle : mesh.triangles) {
                const Vec3 a = restMesh[triangle[0]], b = restMesh[triangle[1]], c = restMesh[triangle[2]];
                const float area = 0.5f * cross(b - a, c - a).length();
                const int k = std::clamp(static_cast<int>(std::ceil(std::sqrt(2.0f * area) / Spacing)), 1, 64);
                const float* sa = &vertexShare[triangle[0] * links];
                const float* sb = &vertexShare[triangle[1] * links];
                const float* sc = &vertexShare[triangle[2] * links];
                // O centro de cada um dos k^2 triangulos da grade baricentrica.
                const auto sample = [&](float u, float v) {
                    const float w = 1.0f - u - v;
                    float total = 0.0f, torsoTotal = 0.0f;
                    for (std::size_t i = 0; i < links; ++i) {
                        share[i] = w * sa[i] + u * sb[i] + v * sc[i];
                        total += share[i];
                        if (isTorso(i)) torsoTotal += share[i];
                    }
                    if (total <= 0.0f) return;
                    const Vec3 point = a * w + b * u + c * v;
                    const auto best = static_cast<std::size_t>(std::max_element(share.begin(), share.end()) - share.begin());
                    if (isTorso(best)) {
                        if (torsoTotal >= TorsoShare * total) torso.push_back(point);
                    } else if (share[best] >= OwnShare * total) {
                        linkPoints[best].push_back(point);
                    }
                };
                const float step = 1.0f / static_cast<float>(k);
                for (int i = 0; i < k; ++i)
                    for (int j = 0; i + j < k; ++j) {
                        sample((static_cast<float>(i) + 1.0f / 3.0f) * step, (static_cast<float>(j) + 1.0f / 3.0f) * step);
                        if (i + j + 1 < k) sample((static_cast<float>(i) + 2.0f / 3.0f) * step, (static_cast<float>(j) + 2.0f / 3.0f) * step);
                    }
            }
        }
        for (std::size_t i = 0; i < links; ++i) {
            if (!isTorso(i)) continue;
            const int child = firstChild(i);
            const bool bottom = profile.links[i].parentIndex < 0;
            const bool top = child < 0 || !isTorso(static_cast<std::size_t>(child));
            const float low = bottom ? -std::numeric_limits<float>::max() : restJoint(i).z;
            const float high = top ? std::numeric_limits<float>::max() : restJoint(static_cast<std::size_t>(child)).z;
            for (const Vec3& point : torso)
                if (point.z >= low && point.z < high) linkPoints[i].push_back(point);
        }
        std::vector<Collider> colliders(links);
        {
            std::string counts = "pontos por link:";
            for (std::size_t i = 0; i < links; ++i) counts += " " + profile.links[i].id + " " + std::to_string(linkPoints[i].size());
            report.lines.push_back(counts);
        }
        for (std::size_t i = 0; i < links; ++i) {
            const auto& link = profile.links[i];
            const std::string part = link.id.rfind("Left", 0) == 0 ? link.id.substr(4)
                : link.id.rfind("Right", 0) == 0                    ? link.id.substr(5)
                                                                    : link.id;
            const std::string side = link.id.substr(0, link.id.size() - part.size());
            const std::vector<Vec3>& points = linkPoints[i];
            if (points.size() < 64) throw std::runtime_error("o link " + link.id + " com pouca malha para medir (" + report.lines.back() + ")");
            const Vec3 start = anchor[i];
            const int child = firstChild(i);
            if (part == "UpperArm" || part == "Forearm" || part == "Thigh" || part == "Shin") {
                const bool leg = part == "Thigh" || part == "Shin";
                colliders[i] = fitLimb(start, restJoint(static_cast<std::size_t>(child)), points, leg ? 50.0f : 70.0f, true,
                    part == "Thigh" ? std::pair { 0.90f, 0.99f } : std::pair { 0.75f, 0.92f });
            } else if (part == "Hand") {
                const Vec3 end = restOf(side + "MiddleTip").value_or(start + Vec3 { 0.0f, side == "Left" ? 0.18f : -0.18f, 0.0f });
                colliders[i] = fitLimb(start, end, points, 70.0f, false, {});
            } else if (part == "Neck") {
                colliders[i] = fitLimb(start, restJoint(static_cast<std::size_t>(child)), points, 70.0f, false, {});
            } else if (part == "Head") {
                colliders[i] = fitLimb(start, restOf("HeadTop").value_or(start + Vec3 { 0.0f, 0.0f, 0.2f }), points, 70.0f, false, {});
            } else if (part == "Foot") {
                if (!ragdollColliderHasBox3D(link.collider)) throw std::runtime_error("o pe do perfil sem caixa nem casca");
                colliders[i] = fitBoot(points, skeleton.groundHeight, report.lines);
            } else {
                // O tronco: a capsula deitada do perfil, no lugar novo.
                const Quaternion orientation = (link.modelOrientation * link.collider.localOrientation).normalized();
                const Vec3 center = linkPosition[i] + link.modelOrientation.rotate(link.collider.localPosition * scale[i]);
                colliders[i] = fitTorso(orientation, center, points);
            }
            if (colliders[i].box != ragdollColliderHasBox3D(link.collider))
                throw std::runtime_error("o colisor de " + link.id + " mudou de forma");
        }
        // As coxas afastadas no repouso: sobrepostas, a autocolisao
        // desligaria o par. So o raio de cima encolhe (a coxa mais fina
        // perto da virilha).
        int leftThigh = -1, rightThigh = -1;
        for (std::size_t i = 0; i < links; ++i) {
            if (profile.links[i].id == "LeftThigh") leftThigh = static_cast<int>(i);
            if (profile.links[i].id == "RightThigh") rightThigh = static_cast<int>(i);
        }
        float thighGap = 0.0f;
        if (leftThigh >= 0 && rightThigh >= 0) {
            constexpr float Clearance = 0.024f;
            Collider& left = colliders[static_cast<std::size_t>(leftThigh)];
            Collider& right = colliders[static_cast<std::size_t>(rightThigh)];
            thighGap = capsuleGap(left, right);
            for (int round = 0; round < 20 && thighGap < Clearance - 1e-4f; ++round) {
                const float shrink = 0.5f * (Clearance - thighGap) + 0.0005f;
                for (Collider* c : { &left, &right }) shrinkLimbEnd(*c, true, shrink, 0.0f);
                thighGap = capsuleGap(left, right);
            }
        }
        // A folga nas animacoes de locomocao: o braco solto ao lado do corpo
        // (parado, correndo, de lado) nao encosta no tronco e as coxas nao
        // se cruzam na passada de lado - o perfil anterior passava a 0,7 cm
        // e 0,8 cm no mais apertado. Na malha o braco encosta no tronco
        // (a axila, o latissimo): o colisor e o nucleo. Estreita primeiro o
        // tronco (ate 70% da largura medida: o peito fica na largura das
        // costelas) e depois o membro na ponta do contato (ate 70% do raio),
        // igual dos dois lados. O braco fica a 0,4 cm: o ombro do corpo novo
        // e mais baixo, o braco solto pende ao lado do peito.
        {
            constexpr float ArmTorsoClearance = 0.004f, ThighClearance = 0.008f, Floor = 0.7f;
            const auto partName = [&](std::size_t i) {
                const std::string& id = profile.links[i].id;
                return id.rfind("Left", 0) == 0 ? id.substr(4) : id.rfind("Right", 0) == 0 ? id.substr(5) : id;
            };
            const auto isArm = [&](std::size_t i) {
                const std::string part = partName(i);
                return part == "UpperArm" || part == "Forearm" || part == "Hand";
            };
            const auto mirror = [&](std::size_t i) {
                const std::string& id = profile.links[i].id;
                const std::string other = id.rfind("Left", 0) == 0 ? "Right" + id.substr(4)
                    : id.rfind("Right", 0) == 0                    ? "Left" + id.substr(5)
                                                                   : id;
                for (std::size_t k = 0; k < links; ++k)
                    if (profile.links[k].id == other) return k;
                return i;
            };
            struct Pair {
                std::size_t a, b;
                float target;
            };
            std::vector<Pair> pairs;
            for (std::size_t a = 0; a < links; ++a)
                for (std::size_t b = a + 1; b < links; ++b) {
                    if (profile.links[a].parentIndex == static_cast<int>(b) || profile.links[b].parentIndex == static_cast<int>(a)) continue;
                    if ((isArm(a) && isTorso(b)) || (isArm(b) && isTorso(a))) pairs.push_back({ a, b, ArmTorsoClearance });
                    else if (partName(a) == "Thigh" && partName(b) == "Thigh") pairs.push_back({ a, b, ThighClearance });
                }
            // As poses: as animacoes de locomocao do manifesto (fora os
            // levantar, os carrinhos e a arrancada agachada: no chao a mao
            // encosta na perna de proposito).
            std::vector<std::string> clipIds;
            const Json characterManifest = readJson(loaded.manifest.characterPath);
            const auto addClip = [&](const std::string& role, const Json& id) {
                if (!id.is_string() || role.rfind("standUp", 0) == 0 || role == "crouchToSprint") return;
                if (std::find(clipIds.begin(), clipIds.end(), id.get<std::string>()) == clipIds.end()) clipIds.push_back(id.get<std::string>());
            };
            for (const char* section : { "locomotion", "defaultLocomotion" })
                if (characterManifest.contains(section))
                    for (const auto& [role, id] : characterManifest.at(section).items()) addClip(role, id);
            RagdollProfile3D fitted = profile;
            std::vector<Transform> linkRest(links);
            for (std::size_t i = 0; i < links; ++i) {
                fitted.links[i].modelPosition = linkPosition[i];
                if (fitted.links[i].parentIndex >= 0) fitted.links[i].inboundJoint.anchorModelPosition = anchor[i];
                linkRest[i] = { linkPosition[i], profile.links[i].modelOrientation.normalized() };
            }
            std::vector<std::vector<Transform>> frames;
            for (const std::string& id : clipIds) {
                std::string clipError;
                const std::optional<AnimationClip3D> clip = loadCharacterClip(id, &clipError);
                if (!clip) {
                    report.lines.push_back("folga: sem a animacao " + id + " (" + clipError + ")");
                    continue;
                }
                for (int k = 0; k <= 40; ++k) {
                    const RagdollAnimationPose3D pose =
                        sampleRagdollAnimationPose3D(fitted, &*clip, clip->durationSeconds * static_cast<float>(k) / 40.0f, true);
                    std::vector<Transform> frame(links);
                    for (std::size_t i = 0; i < links; ++i) frame[i] = { pose.linkPositions[i], pose.linkOrientations[i] };
                    frames.push_back(std::move(frame));
                }
            }
            const std::vector<Collider> measured = colliders;
            const auto worstContact = [&](const Pair& pair) {
                Contact worst;
                for (const auto& frame : frames) {
                    const Contact c = capsuleContact(posedCollider(colliders[pair.a], linkRest[pair.a], frame[pair.a]),
                        posedCollider(colliders[pair.b], linkRest[pair.b], frame[pair.b]));
                    if (c.gap < worst.gap) worst = c;
                }
                return worst;
            };
            const auto shrinkArm = [&](std::size_t i, bool proximal, float by) {
                const float done = shrinkLimbEnd(colliders[i], proximal, by, Floor * (proximal ? measured[i].radius : measured[i].radiusPositive));
                if (const std::size_t other = mirror(i); other != i) shrinkLimbEnd(colliders[other], proximal, done, 0.0f);
                return done;
            };
            std::vector<float> before(pairs.size());
            for (std::size_t k = 0; k < pairs.size(); ++k) before[k] = worstContact(pairs[k]).gap;
            for (int round = 0; round < 40 && !frames.empty(); ++round) {
                bool changed = false;
                for (const Pair& pair : pairs) {
                    const Contact contact = worstContact(pair);
                    if (contact.gap >= pair.target - 1e-4f) continue;
                    float need = pair.target - contact.gap + 0.0005f;
                    if (isTorso(pair.a) || isTorso(pair.b)) {
                        const bool torsoFirst = isTorso(pair.a);
                        const std::size_t torsoLink = torsoFirst ? pair.a : pair.b, arm = torsoFirst ? pair.b : pair.a;
                        Collider& torsoCollider = colliders[torsoLink];
                        need -= shrinkTorsoWidth(torsoCollider, need,
                            std::max(2.0f * torsoCollider.radius + 0.012f, Floor * measured[torsoLink].length));
                        if (need > 1e-5f) need -= shrinkArm(arm, (torsoFirst ? contact.v : contact.u) < 0.5f, need);
                    } else {
                        // As duas coxas, metade cada uma, na ponta do contato.
                        for (const auto& [link, along] : { std::pair { pair.a, contact.u }, std::pair { pair.b, contact.v } })
                            need -= shrinkLimbEnd(colliders[link], along < 0.5f, 0.5f * (pair.target - contact.gap + 0.0005f),
                                Floor * (along < 0.5f ? measured[link].radius : measured[link].radiusPositive));
                    }
                    changed = true;
                }
                if (!changed) break;
            }
            for (std::size_t k = 0; k < pairs.size(); ++k) {
                const float after = worstContact(pairs[k]).gap;
                if (std::abs(after - before[k]) < 1e-4f && after > 0.05f) continue;
                std::snprintf(line, sizeof line, "folga nas animacoes %s x %s: %+.1f cm (medido %+.1f)%s", profile.links[pairs[k].a].id.c_str(),
                    profile.links[pairs[k].b].id.c_str(), after * 100.0f, before[k] * 100.0f, after < pairs[k].target - 1e-4f ? "  AVISO: abaixo do alvo" : "");
                report.lines.push_back(line);
            }
            std::snprintf(line, sizeof line, "folga medida em %zu poses de %zu animacoes", frames.size(), clipIds.size());
            report.lines.push_back(line);
        }
        for (std::size_t i = 0; i < links; ++i) report.lines.push_back(describe(profile.links[i].id, profile.links[i].collider, colliders[i]));
        std::snprintf(line, sizeof line, "coxas em repouso: folga %.1f mm", thighGap * 1000.0f);
        report.lines.push_back(line);

        // A autocolisao como era: o backend desliga o par sobreposto no
        // repouso (e o da mesma cadeia a menos de 1 cm). O par que era
        // desligado e passaria a colidir fica declarado no perfil (o queixo
        // no alto do peito, olhando a bola); o que colidia e passaria a
        // desligar vira aviso. Os pes (caixas) ficam longe de tudo em repouso.
        const auto isAncestor = [&](std::size_t ancestor, std::size_t link) {
            for (int at = profile.links[link].parentIndex; at >= 0; at = profile.links[static_cast<std::size_t>(at)].parentIndex)
                if (static_cast<std::size_t>(at) == ancestor) return true;
            return false;
        };
        const auto declared = [&](const std::string& a, const std::string& b) {
            for (const RagdollIgnoredCollisionPair3D& pair : profile.selfCollisionIgnoredPairs)
                if ((pair.firstLinkId == a && pair.secondLinkId == b) || (pair.firstLinkId == b && pair.secondLinkId == a)) return true;
            return false;
        };
        std::vector<std::pair<std::string, std::string>> keptOff;
        for (std::size_t a = 0; a < links; ++a)
            for (std::size_t b = a + 1; b < links; ++b) {
                const auto& first = profile.links[a];
                const auto& second = profile.links[b];
                if (first.parentIndex == static_cast<int>(b) || second.parentIndex == static_cast<int>(a)) continue;
                if (colliders[a].box || colliders[b].box || declared(first.id, second.id)) continue;
                const float limit = isAncestor(a, b) || isAncestor(b, a) ? 0.01f : 0.0f;
                const bool wasOff = capsuleGap(profileCollider(first), profileCollider(second)) < limit;
                const float gap = capsuleGap(colliders[a], colliders[b]);
                if (wasOff && gap >= limit) {
                    keptOff.push_back({ first.id, second.id });
                    std::snprintf(line, sizeof line, "%s x %s: desligado como antes (folga em repouso %.1f cm)", first.id.c_str(),
                        second.id.c_str(), gap * 100.0f);
                    report.lines.push_back(line);
                } else if (!wasOff && gap < limit) {
                    std::snprintf(line, sizeof line, "AVISO %s x %s deixa de colidir (folga em repouso %.1f cm)", first.id.c_str(),
                        second.id.c_str(), gap * 100.0f);
                    report.lines.push_back(line);
                }
            }

        // ---- O espaco do jogo: a pelve na origem; o chao, o do repouso em pe
        // (o rig sobe a pelve ate a sola tocar o chao do modelo) ----
        const Vec3 lift { 0.0f, 0.0f, -linkPosition.front().z };
        const float standing = linkPosition.front().z - skeleton.groundHeight;
        float lowest = std::numeric_limits<float>::max();
        for (const Vec3& p : restMesh) lowest = std::min(lowest, p.z);
        report.pelvisHeightBefore = profile.links.front().modelPosition.z + profile.standingRootHeightMeters;
        report.pelvisHeightAfter = standing;
        std::snprintf(line, sizeof line, "pelve do chao no repouso: %.4f m (antes %.4f m: %+.1f%%)", standing, report.pelvisHeightBefore,
            100.0f * (standing / report.pelvisHeightBefore - 1.0f));
        report.lines.push_back(line);
        std::snprintf(line, sizeof line, "o mais baixo da malha no repouso: %+.1f mm do chao (as travas)", (lowest - skeleton.groundHeight) * 1000.0f);
        report.lines.push_back(line);
        const Transform toGame { lift, Quaternion {} };

        // ---- O perfil: o atual com as posicoes e os colisores novos ----
        const std::filesystem::path characterDirectory = loaded.manifest.characterPath.parent_path();
        Json manifest = readJson(loaded.manifest.characterPath);
        const std::string profileFile = manifest.at("physicsProfile").get<std::string>();
        Json profileJson = readJson(characterDirectory / profileFile);
        profileJson["standingRootHeightMeters"] = rounded(standing);
        for (const auto& [a, b] : keptOff)
            profileJson["selfCollisionIgnoredPairs"].push_back({ { "links", Json::array({ a, b }) },
                { "reason", "desligado no perfil anterior (sobreposto em repouso); no corpo novo os colisores se afastam, e colidindo o par "
                            "seria um batente (o queixo no alto do peito ao olhar a bola)" } });
        for (std::size_t i = 0; i < links; ++i) {
            Json& entry = profileJson.at("links").at(i);
            if (entry.at("id").get<std::string>() != profile.links[i].id) throw std::runtime_error("o perfil fora da ordem dos links");
            entry["position"] = vector3(linkPosition[i] + lift);
            if (profile.links[i].parentIndex >= 0) entry.at("joint")["anchor"] = vector3(anchor[i] + lift);
            const Collider& c = colliders[i];
            const Quaternion linkInverse = profile.links[i].modelOrientation.normalized().conjugate();
            if (c.hull && !entry.contains("hull")) {
                // A caixa vira casca (o material e o sensor continuam).
                Json previous = entry.at("box");
                previous.erase("halfExtents");
                entry.erase("box");
                entry["hull"] = previous;
            }
            Json& shape = entry.at(c.hull ? "hull" : c.box ? "box" : "capsule");
            shape["position"] = vector3(linkInverse.rotate(c.center - linkPosition[i]));
            shape["orientation"] = quaternion4(linkInverse * c.orientation);
            if (c.hull) {
                Json hullPoints = Json::array(), hullFaces = Json::array();
                for (const Vec3& q : c.hullPoints) hullPoints.push_back(vector3(q));
                for (const auto& face : c.hullFaces) hullFaces.push_back(Json::array({ face[0], face[1], face[2] }));
                shape["convexRadius"] = rounded(c.convexRadius);
                shape["points"] = hullPoints;
                shape["faces"] = hullFaces;
            } else if (c.box) {
                shape["halfExtents"] = vector3(c.halfExtents);
            } else {
                shape["length"] = rounded(c.length);
                shape["radius"] = rounded(c.radius);
                if (c.tapered) shape["radiusAtPositiveX"] = rounded(c.radiusPositive);
                else shape.erase("radiusAtPositiveX");
            }
        }

        // ---- A pele (versao 2) ----
        Json skin;
        skin["schema"] = "matter-ragdoll-skin-2";
        skin["rigId"] = profile.id;
        Json bones = Json::array();
        for (std::size_t i = 0; i < links; ++i) {
            // O repouso L (o link), a junta em repouso T e na ligacao da
            // malha A: a ligacao do osso e A T^-1 L (a malha deforma da pose
            // dela; o link, nas poses do jogo, leva a junta junto).
            const auto joint = static_cast<std::size_t>(linkJoint[i]);
            const Transform link { linkPosition[i] + lift, profile.links[i].modelOrientation.normalized() };
            const Transform bind = toGame * skeleton.joints[joint].bind * (toGame * rest.joints[joint]).inverse() * link;
            bones.push_back({ { "link", profile.links[i].id }, { "restPosition", vector3(link.position) },
                { "restOrientation", quaternion4(link.orientation) }, { "bindPosition", vector3(bind.position) },
                { "bindOrientation", quaternion4(bind.orientation) } });
        }
        skin["bones"] = bones;
        Json visualJson = Json::array();
        static const char* const FingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
        for (std::size_t v = 0; v < visuals; ++v) {
            const RagdollVisualBone3D& bone = character.visualBones[v];
            const auto joint = static_cast<std::size_t>(rig.paletteJoint[physicalPalette + v]);
            const std::string parentName = bone.parentBone < physicalPalette
                ? profile.links[character.boneLinkIndices[bone.parentBone]].id
                : character.visualBones[bone.parentBone - physicalPalette].name;
            const Transform restFrame = toGame * rest.joints[joint];
            const Transform bindFrame = toGame * skeleton.joints[joint].bind;
            Json entry { { "name", bone.name }, { "parent", parentName }, { "restPosition", vector3(restFrame.position) },
                { "restOrientation", quaternion4(restFrame.orientation) }, { "bindPosition", vector3(bindFrame.position) },
                { "bindOrientation", quaternion4(bindFrame.orientation) }, { "restRotation", quaternion4(bone.restRotation) } };
            if (bone.kind == RagdollVisualBone3D::Kind::Finger) {
                entry["kind"] = "finger";
                entry["finger"] = FingerNames[std::clamp(bone.finger, 0, 4)];
                entry["joint"] = bone.joint;
                entry["flexion"] = bone.flexion;
                entry["close"] = bone.close;
                entry["closeAngle"] = bone.closeAngle;
            } else if (bone.kind == RagdollVisualBone3D::Kind::Toe) {
                // Da base dos dedos a ponta (a medida no corpo novo).
                const std::optional<Vec3> tip = restOf(bone.name + "Tip");
                entry["kind"] = "toe";
                entry["length"] = rounded(tip ? (*tip - rest.joints[joint].position).length() : bone.length);
            }
            visualJson.push_back(entry);
        }
        skin["visualBones"] = visualJson;
        Json helperJson = Json::array();
        for (const int h : helpers) {
            const Joint& helper = skeleton.joints[static_cast<std::size_t>(h)];
            const auto name = [&](int joint) -> std::string {
                const auto at = static_cast<std::size_t>(palette[static_cast<std::size_t>(joint)]);
                if (at < links) return profile.links[at].id;
                if (at < links + visuals) return character.visualBones[at - links].name;
                throw std::runtime_error("o auxiliar " + helper.name + " preso a outro auxiliar");
            };
            Json entry { { "name", helper.name }, { "upper", name(helper.derivedFrom) }, { "lower", name(helper.derivedTo) },
                { "fraction", helper.derivedFraction },
                { "pivot", vector3(skeleton.joints[static_cast<std::size_t>(helper.derivedTo)].bind.position + lift) } };
            // So a parte do giro em volta do eixo (o espaco da malha e o do
            // modelo girado nada: so a origem muda).
            if (helper.derivedAxis.lengthSquared() > 0.0f) entry["axis"] = vector3(helper.derivedAxis);
            helperJson.push_back(std::move(entry));
        }
        skin["helperBones"] = helperJson;

        // Os vertices: um por (vertice, grupo do sombreamento, coordenada do
        // atlas) - a quina e a costura do atlas separam, o resto e liso.
        const ShadingNormals shading(mesh, settings.creaseDegrees);
        std::vector<Vec3> corners;
        shading.compute(mesh.positions, mesh.triangles, 0.0f, corners);
        std::map<std::tuple<std::uint32_t, std::uint32_t, std::int32_t, std::int32_t>, std::uint32_t> unique;
        Json vertices = Json::array(), indices = Json::array();
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            for (int k = 0; k < 3; ++k) {
                const std::uint32_t source = mesh.triangles[t][static_cast<std::size_t>(k)];
                const Uv uv = mesh.triangleUv[t][static_cast<std::size_t>(k)];
                const auto key = std::make_tuple(source, shading.group(t, k), static_cast<std::int32_t>(std::lround(uv.u * 1048576.0f)),
                    static_cast<std::int32_t>(std::lround(uv.v * 1048576.0f)));
                auto found = unique.find(key);
                if (found == unique.end()) {
                    // Os pesos na paleta (somados quando duas juntas caem no
                    // mesmo osso), os quatro maiores, somando 1.
                    std::map<int, float> influence;
                    const SkinWeights& weights = mesh.skin[source];
                    for (std::size_t i = 0; i < weights.weights.size(); ++i)
                        if (weights.weights[i] > 0.0f && weights.joints[i] < palette.size() && palette[weights.joints[i]] >= 0)
                            influence[palette[weights.joints[i]]] += weights.weights[i];
                    std::vector<std::pair<float, int>> ranked;
                    for (const auto& [bone, weight] : influence) ranked.push_back({ weight, bone });
                    std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
                    if (ranked.size() > 4) ranked.resize(4);
                    float total = 0.0f;
                    for (const auto& entry : ranked) total += entry.first;
                    if (total <= 0.0f) throw std::runtime_error("vertice sem peso");
                    std::array<float, 4> share {};
                    Json joints = Json::array();
                    float written = 0.0f;
                    for (std::size_t i = 0; i < 4; ++i) {
                        joints.push_back(i < ranked.size() ? ranked[i].second : 0);
                        share[i] = i < ranked.size() ? rounded(ranked[i].first / total) : 0.0f;
                        written += share[i];
                    }
                    // A soma exata (o arredondamento sobra no maior).
                    share[0] = rounded(share[0] + (1.0f - written));
                    Vec3 normal = corners[t * 3 + static_cast<std::size_t>(k)];
                    if (normal.length() < 0.5f) normal = { 0.0f, 0.0f, 1.0f };
                    vertices.push_back({ { "position", vector3(mesh.positions[source] + lift) }, { "normal", vector3(normal) },
                        { "color", Json::array({ 1.0, 1.0, 1.0 }) }, { "uv", Json::array({ rounded(uv.u, 1e-7f), rounded(uv.v, 1e-7f) }) },
                        { "joints", joints }, { "weights", Json::array({ share[0], share[1], share[2], share[3] }) } });
                    found = unique.emplace(key, static_cast<std::uint32_t>(vertices.size() - 1)).first;
                }
                indices.push_back(found->second);
            }
        skin["vertices"] = vertices;
        skin["indices"] = indices;
        report.vertices = vertices.size();
        report.triangles = mesh.triangles.size();
        report.bones = links + visuals + helpers.size();
        std::snprintf(line, sizeof line, "pele: %zu vertices (%zu na modelagem), %zu triangulos, %zu ossos (%zu fisicos, %zu dedos, %zu auxiliares)",
            report.vertices, mesh.positions.size(), report.triangles, report.bones, links, visuals, helpers.size());
        report.lines.push_back(line);

        // ---- O atlas e o manifesto ----
        std::string imageError;
        if (!writePng(mesh.textures.front().image, (outputDirectory / settings.albedoFile).string(), &imageError))
            throw std::runtime_error(imageError);
        manifest["displayName"] = settings.displayName;
        manifest["skin"] = settings.skinFile;
        manifest["albedo"] = settings.albedoFile;
        manifest["flatShaded"] = false;
        manifest["facet"] = rounded(settings.facet, 1e-3f);
        manifest["creaseDegrees"] = rounded(settings.creaseDegrees, 1e-3f);
        manifest["attribution"] = { { "title", "Jogador montado pela engine" }, { "sourceFile", "../player_model/model.json" },
            { "note",
                "Exportado de assets/characters/player_model (MatterModelInspector exportar-jogo). Corpo \"Male Lowpoly Base Mesh\" "
                "(CarvMad) e cabeca \"lowpoly man's head\" (Jelveh 3D), CC BY 4.0, com o molde \"Zbrush head base\" (hexmeff, CC BY 4.0) "
                "so na forma do queixo; creditos e alteracoes em player_model/base/CREDITOS.md." } };
        manifest["rigCompatibility"] = { { "note",
            "O rig e o do perfil calibrado (as juntas, os eixos, os limites, os motores e as massas); as ancoras, os links e os "
            "colisores foram medidos no corpo novo na pose T dele, com a pelve na origem. As animacoes mantem os mesmos tracks (o "
            "movimento da raiz como esta: a pelve do repouso fica 1,5% mais baixa que a do corpo anterior)." } };
        writeJson(outputDirectory / profileFile, profileJson);
        writeJson(outputDirectory / settings.skinFile, skin);
        writeJson(outputDirectory / "character.json", manifest);
        return true;
    } catch (const std::exception& exception) {
        error = std::string("exportar o jogador: ") + exception.what();
        return false;
    }
}

} // namespace MatterEngine::Modeling
