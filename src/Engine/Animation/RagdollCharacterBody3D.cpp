#include "Engine/Animation/RagdollCharacterBody3D.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace MatterEngine {

namespace {

// Os links fisicos que um osso da paleta move, com a parte de cada um.
using Shares = std::vector<std::pair<std::size_t, float>>;

std::vector<Shares> paletteShares(const RagdollCharacter3D& character) {
    const std::size_t physical = character.boneLinkIndices.size();
    const std::size_t skinned = physical + character.visualBones.size();
    const auto physicalOf = [&](std::size_t bone) {
        for (int guard = 0; bone >= physical && bone < skinned && guard < 64; ++guard)
            bone = character.visualBones[bone - physical].parentBone;
        if (bone >= physical) throw std::runtime_error("Visual bone without a physical ancestor");
        return character.boneLinkIndices[bone];
    };
    std::vector<Shares> shares(skinned + character.helperBones.size());
    for (std::size_t bone = 0; bone < skinned; ++bone) shares[bone] = { { physicalOf(bone), 1.0f } };
    for (std::size_t h = 0; h < character.helperBones.size(); ++h) {
        const RagdollHelperBone3D& helper = character.helperBones[h];
        const float lower = helper.fraction;
        Shares& out = shares[skinned + h];
        if (lower < 1.0f) out.push_back({ physicalOf(helper.upperBone), 1.0f - lower });
        if (lower > 0.0f) out.push_back({ physicalOf(helper.lowerBone), lower });
    }
    return shares;
}

std::vector<Vec3> smoothNormals(const MeshData3D& mesh, const std::vector<Vec3>& positions) {
    std::vector<Vec3> normals(positions.size());
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const std::uint32_t a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
        const Vec3 face = cross(positions[b] - positions[a], positions[c] - positions[a]);
        normals[a] += face;
        normals[b] += face;
        normals[c] += face;
    }
    for (Vec3& normal : normals)
        normal = normal.lengthSquared() > 1e-20f ? normal.normalized() : Vec3 {};
    return normals;
}

} // namespace

RagdollToneDeltas3D computeRagdollToneDeltas3D(const RagdollCharacter3D& character) {
    const RagdollProfile3D& profile = character.profile;
    const std::vector<BodyGirthAxis3D> restAxes = bodyGirthAxes3D(profile);
    // Os eixos levados do repouso a ligacao da malha (o osso de cada link).
    std::vector<BodyGirthAxis3D> axes = restAxes;
    std::vector<float> heavy(profile.links.size(), 1.0f), light(profile.links.size(), 1.0f);
    std::vector<float> heavyDepth(profile.links.size(), 1.0f), lightDepth(profile.links.size(), 1.0f);
    std::vector<float> heavyBack(profile.links.size(), 1.0f), lightBack(profile.links.size(), 1.0f);
    std::vector<bool> inward(profile.links.size(), false), upperArm(profile.links.size(), false);
    for (std::size_t bone = 0; bone < character.boneLinkIndices.size(); ++bone) {
        const std::size_t link = character.boneLinkIndices[bone];
        const RagdollLinkDefinition3D& rest = profile.links[link];
        const Quaternion turn = (character.bindOrientations[bone] * rest.modelOrientation.conjugate()).normalized();
        axes[link].point = character.bindPositions[bone] + turn.rotate(restAxes[link].point - rest.modelPosition);
        axes[link].direction = turn.rotate(restAxes[link].direction).normalized();
        if (const std::optional<PhysiqueRegion3D> region = physiqueRegionOfLink3D(rest.id)) {
            heavy[link] = bodyGirth3D(*region, 1.0f);
            light[link] = bodyGirth3D(*region, -1.0f);
            heavyDepth[link] = bodyGirthDepth3D(*region, 1.0f);
            lightDepth[link] = bodyGirthDepth3D(*region, -1.0f);
            heavyBack[link] = bodyGirthBack3D(*region, 1.0f);
            lightBack[link] = bodyGirthBack3D(*region, -1.0f);
            // A coxa e o braco quase nao crescem para dentro (para a outra
            // coxa, para o tronco): o rego, o gancho e a dobra de tras da
            // axila se cruzavam.
            inward[link] = *region == PhysiqueRegion3D::Thigh || *region == PhysiqueRegion3D::UpperArm;
            upperArm[link] = *region == PhysiqueRegion3D::UpperArm;
        }
    }
    // A malha e o espelho exato de um lado no outro (y): os eixos tambem (os
    // links do perfil diferem uns milimetros entre os lados; sem isto o tonus
    // tirava a malha da simetria). O meio fica no plano do meio.
    const auto mirrored = [](Vec3 v) { return Vec3 { v.x, -v.y, v.z }; };
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const std::string& id = profile.links[i].id;
        if (id.rfind("Left", 0) == 0) {
            for (std::size_t k = 0; k < profile.links.size(); ++k) {
                if (profile.links[k].id != "Right" + id.substr(4)) continue;
                const Vec3 point = (axes[k].point + mirrored(axes[i].point)) * 0.5f;
                const Vec3 direction = (axes[k].direction + mirrored(axes[i].direction)).normalized();
                axes[k].point = point;
                axes[k].direction = direction;
                axes[i].point = mirrored(point);
                axes[i].direction = mirrored(direction);
            }
        } else if (id.rfind("Right", 0) != 0) {
            axes[i].point.y = 0.0f;
            axes[i].direction.y = 0.0f;
            axes[i].direction = axes[i].direction.normalized();
        }
    }
    const std::vector<Shares> shares = paletteShares(character);
    const std::size_t count = character.mesh.vertices.size();
    RagdollToneDeltas3D deltas;
    deltas.heavyPosition.resize(count);
    deltas.lightPosition.resize(count);
    for (std::size_t v = 0; v < count; ++v) {
        const MeshVertex3D& vertex = character.mesh.vertices[v];
        for (std::size_t k = 0; k < 4; ++k) {
            if (vertex.weights[k] <= 0.0f) continue;
            for (const auto& [link, share] : shares.at(vertex.joints[k])) {
                const BodyGirthAxis3D& axis = axes[link];
                const Vec3 offset = vertex.position - axis.point;
                Vec3 radial = offset - axis.direction * dot(offset, axis.direction);
                // (So no alto do osso - a virilha, o rego, a axila -, sumindo
                // ate o meio dele: o resto do membro engorda por igual.)
                if (inward[link] && axis.length > 1e-6f) {
                    const Vec3 middle { 0.0f, axis.point.y > 0.0f ? -1.0f : 1.0f, 0.0f };
                    const float toward = dot(radial, middle);
                    const float t = dot(offset, axis.direction) / axis.length;
                    const float near = 1.0f - std::clamp((t - 0.45f) / 0.30f, 0.0f, 1.0f);
                    if (toward > 0.0f) radial = radial - middle * (0.9f * near * toward);
                }
                // O ombro do encorpado (o alto do braco e a escapula) quase nao
                // cresce para tras (x e a frente) nem para cima: a manga de tras e
                // a escapula estufavam e a linha do trapezio subia ate a orelha
                // (corcunda, sem pescoco). O triceps cresce.
                Vec3 heavyRadial = radial;
                if (upperArm[link] && axis.length > 1e-6f) {
                    const float t = dot(offset, axis.direction) / axis.length;
                    const float top = 1.0f - std::clamp((t - 0.15f) / 0.25f, 0.0f, 1.0f);
                    if (heavyRadial.x < 0.0f) heavyRadial.x *= 1.0f - 0.8f * top;
                    if (heavyRadial.z > 0.0f) heavyRadial.z *= 1.0f - 0.7f * top;
                }
                const float weight = vertex.weights[k] * share;
                // A parte de frente para tras (x) pela profundidade (a de tras, as
                // costas, pela dela), o resto pela largura.
                const auto moved = [&](Vec3 around, const std::vector<float>& wide, const std::vector<float>& deep,
                                       const std::vector<float>& back) {
                    const Vec3 depth { around.x, 0.0f, 0.0f }, width = around - depth;
                    return width * (bodyGirthAlong3D(axes, wide, link, vertex.position) - 1.0f)
                        + depth * (bodyGirthAlong3D(axes, around.x < 0.0f ? back : deep, link, vertex.position) - 1.0f);
                };
                deltas.heavyPosition[v] += moved(heavyRadial, heavy, heavyDepth, heavyBack) * weight;
                deltas.lightPosition[v] += moved(radial, light, lightDepth, lightBack) * weight;
            }
        }
    }
    // O campo encorpado alisado na malha (os vertices da mesma origem sao um so; os
    // vizinhos pelas arestas): perto de uma costura de duas camadas (a faixa do
    // colarinho, a dobra da manga) cada camada seguia o eixo do seu osso e elas
    // se cruzavam a 95 kg. A chuteira (deslocamento zero) fica parada.
    if (!character.surfaceVertices.empty() && character.surfaceVertices.size() == count) {
        std::uint32_t sources = 0;
        for (const auto& extra : character.surfaceVertices) sources = std::max(sources, extra.source + 1);
        std::vector<std::vector<std::uint32_t>> neighbors(sources);
        for (std::size_t i = 0; i + 2 < character.mesh.indices.size(); i += 3)
            for (std::size_t a = 0; a < 3; ++a) {
                const std::uint32_t p = character.surfaceVertices[character.mesh.indices[i + a]].source;
                const std::uint32_t q = character.surfaceVertices[character.mesh.indices[i + (a + 1) % 3]].source;
                if (p == q) continue;
                if (std::find(neighbors[p].begin(), neighbors[p].end(), q) == neighbors[p].end()) neighbors[p].push_back(q);
                if (std::find(neighbors[q].begin(), neighbors[q].end(), p) == neighbors[q].end()) neighbors[q].push_back(p);
            }
        // (So o encorpado: o franzino ja fechava sem cruzar, e alisado ele
        // cruzava a barra do calcao com a coxa.)
        for (std::vector<Vec3>* field : { &deltas.heavyPosition }) {
            std::vector<Vec3> bySource(sources);
            std::vector<bool> pinned(sources, false);
            for (std::size_t v = 0; v < count; ++v) {
                const std::uint32_t source = character.surfaceVertices[v].source;
                bySource[source] = (*field)[v];
                pinned[source] = (*field)[v].lengthSquared() == 0.0f;
            }
            for (int pass = 0; pass < 8; ++pass) {
                std::vector<Vec3> next = bySource;
                for (std::uint32_t p = 0; p < sources; ++p) {
                    if (pinned[p] || neighbors[p].empty()) continue;
                    Vec3 mean;
                    for (const std::uint32_t q : neighbors[p]) mean += bySource[q];
                    next[p] = bySource[p] * 0.5f + mean * (0.5f / static_cast<float>(neighbors[p].size()));
                }
                bySource.swap(next);
            }
            for (std::size_t v = 0; v < count; ++v) (*field)[v] = bySource[character.surfaceVertices[v].source];
        }
    }
    // As normais: a da malha deformada menos a da malha de hoje, pela mesma
    // conta (o metodo da exportacao nao importa: so a diferenca entra).
    std::vector<Vec3> positions(count);
    for (std::size_t v = 0; v < count; ++v) positions[v] = character.mesh.vertices[v].position;
    const std::vector<Vec3> base = smoothNormals(character.mesh, positions);
    for (auto [delta, normal] : { std::pair { &deltas.heavyPosition, &deltas.heavyNormal },
             std::pair { &deltas.lightPosition, &deltas.lightNormal } }) {
        std::vector<Vec3> moved(count);
        for (std::size_t v = 0; v < count; ++v) moved[v] = positions[v] + (*delta)[v];
        const std::vector<Vec3> deformed = smoothNormals(character.mesh, moved);
        normal->resize(count);
        for (std::size_t v = 0; v < count; ++v)
            (*normal)[v] = base[v].lengthSquared() > 0.0f && deformed[v].lengthSquared() > 0.0f ? deformed[v] - base[v] : Vec3 {};
    }
    return deltas;
}

Vec3 ragdollTonePosition3D(const RagdollToneDeltas3D& deltas, std::size_t vertex, Vec3 position, float tone) {
    if (tone == 0.0f || vertex >= deltas.heavyPosition.size()) return position;
    return tone > 0.0f ? position + deltas.heavyPosition[vertex] * tone : position + deltas.lightPosition[vertex] * -tone;
}

Vec3 ragdollToneNormal3D(const RagdollToneDeltas3D& deltas, std::size_t vertex, Vec3 normal, float tone) {
    if (tone == 0.0f || vertex >= deltas.heavyNormal.size()) return normal;
    const Vec3 moved = tone > 0.0f ? normal + deltas.heavyNormal[vertex] * tone : normal + deltas.lightNormal[vertex] * -tone;
    return moved.lengthSquared() > 1e-12f ? moved.normalized() : normal;
}

RagdollCharacter3D bodyRagdollCharacter3D(const RagdollCharacter3D& base, const PlayerBody3D& body,
    const RagdollToneDeltas3D* tone, std::span<const AnimationClip3D* const> clearanceClips) {
    RagdollCharacter3D result = base;
    if (isBaseBody3D(body)) return result;
    const float s = bodyScale3D(body), w = bodyTone3D(body);
    result.profile = bodyRagdollProfileForClips3D(base.profile, body, clearanceClips);
    for (std::size_t bone = 0; bone < result.bindPositions.size(); ++bone) {
        result.bindPositions[bone] = base.bindPositions[bone] * s;
        result.inverseBindMatrices[bone] = Mat4::rotation(result.bindOrientations[bone].conjugate())
            * Mat4::translation(result.bindPositions[bone] * -1.0f);
    }
    for (RagdollVisualBone3D& visual : result.visualBones) {
        visual.localBindPosition = visual.localBindPosition * s;
        visual.length *= s;
    }
    for (RagdollHelperBone3D& helper : result.helperBones) {
        helper.pivot = helper.pivot * s;
        if (helper.root) helper.root = *helper.root * s;
        helper.rampOffset = helper.rampOffset * s;
    }
    const bool shaped = tone && !tone->empty() && tone->heavyPosition.size() == base.mesh.vertices.size();
    for (std::size_t v = 0; v < result.mesh.vertices.size(); ++v) {
        MeshVertex3D& vertex = result.mesh.vertices[v];
        if (shaped) {
            vertex.position = ragdollTonePosition3D(*tone, v, vertex.position, w);
            vertex.normal = ragdollToneNormal3D(*tone, v, vertex.normal, w);
        }
        vertex.position = vertex.position * s;
    }
    recomputeBounds(result.mesh);
    return result;
}

std::vector<const AnimationClip3D*> ragdollClearanceClips3D(const RagdollCharacter3D& character,
    const std::vector<AnimationClip3D>& library) {
    std::vector<std::string> ids { character.idleClipId, character.walkClipId, character.walkBackwardClipId, character.sprintClipId,
        character.idleToSprintClipId, character.runForwardArcLeftClipId, character.runForwardArcRightClipId,
        character.runBackwardArcRightClipId, character.sprintBackwardClipId, character.strafeLeftClipId, character.strafeRightClipId,
        character.sprintStrafeLeftClipId, character.sprintStrafeRightClipId, character.jumpStandingClipId, character.jumpForwardClipId,
        character.jumpBackwardClipId, character.jumpLeftClipId, character.jumpRightClipId, character.jumpPrepClipId,
        character.turnLeftClipId, character.turnRightClipId };
    for (const auto& [role, id] : character.defaultLocomotionClipIds) ids.push_back(id);
    std::vector<const AnimationClip3D*> clips;
    for (const std::string& id : ids) {
        if (id.empty()) continue;
        for (const AnimationClip3D& clip : library)
            if (clip.id == id && std::find(clips.begin(), clips.end(), &clip) == clips.end()) clips.push_back(&clip);
    }
    return clips;
}

AnimationClip3D bodyRagdollClip3D(const AnimationClip3D& clip, const RagdollProfile3D& profile, const PlayerBody3D& body) {
    if (isBaseBody3D(body)) return clip;
    return bodyAnimationClip3D(clip, profile, bodyScale3D(body), bodyArmOpening3D(bodyTone3D(body)), 0.0f);
}

RagdollProfile3D bodyRagdollProfileForClips3D(const RagdollProfile3D& base, const PlayerBody3D& body,
    std::span<const AnimationClip3D* const> clips) {
    RagdollProfile3D profile = bodyRagdollProfile3D(base, body);
    if (isBaseBody3D(body) || clips.empty()) return profile;
    const float s = bodyScale3D(body);
    const std::size_t count = profile.links.size();
    const auto part = [](const std::string& id) {
        return id.rfind("Left", 0) == 0 ? id.substr(4) : id.rfind("Right", 0) == 0 ? id.substr(5) : id;
    };
    std::vector<std::string> parts(count);
    for (std::size_t i = 0; i < count; ++i) parts[i] = part(profile.links[i].id);
    const auto isArm = [&](std::size_t i) { return parts[i] == "UpperArm" || parts[i] == "Forearm" || parts[i] == "Hand"; };
    const auto isTorso = [&](std::size_t i) {
        return parts[i] == "Pelvis" || parts[i] == "Abdomen" || parts[i] == "Chest" || parts[i] == "UpperChest";
    };
    const auto mirror = [&](std::size_t i) {
        const std::string& id = profile.links[i].id;
        const std::string other = id.rfind("Left", 0) == 0 ? "Right" + id.substr(4) : id.rfind("Right", 0) == 0 ? "Left" + id.substr(5) : id;
        for (std::size_t k = 0; k < count; ++k)
            if (profile.links[k].id == other) return k;
        return i;
    };
    struct Pair {
        std::size_t a, b;
        float target;
    };
    std::vector<Pair> pairs;
    for (std::size_t a = 0; a < count; ++a)
        for (std::size_t b = a + 1; b < count; ++b) {
            if (profile.links[a].parentIndex == static_cast<int>(b) || profile.links[b].parentIndex == static_cast<int>(a)) continue;
            if (profile.links[a].collider.shape != RagdollColliderShape3D::Capsule
                || profile.links[b].collider.shape != RagdollColliderShape3D::Capsule)
                continue;
            if ((isArm(a) && isTorso(b)) || (isArm(b) && isTorso(a))) pairs.push_back({ a, b, 0.004f * s });
            else if (parts[a] == "Thigh" && parts[b] == "Thigh") pairs.push_back({ a, b, 0.008f * s });
        }
    // As poses (os colisores nao mexem nelas): 41 instantes de cada clipe do corpo.
    std::vector<RagdollAnimationPose3D> poses;
    for (const AnimationClip3D* source : clips) {
        if (source == nullptr) continue;
        const AnimationClip3D clip = bodyRagdollClip3D(*source, profile, body);
        for (int k = 0; k <= 40; ++k)
            poses.push_back(sampleRagdollAnimationPose3D(profile, &clip, clip.durationSeconds * static_cast<float>(k) / 40.0f, true));
    }
    std::vector<float> floorNegative(count), floorPositive(count);
    for (std::size_t i = 0; i < count; ++i) {
        floorNegative[i] = 0.7f * profile.links[i].collider.radiusMeters;
        floorPositive[i] = 0.7f * ragdollCapsuleRadiusAtPositiveX3D(profile.links[i].collider);
    }
    // Encolhe a ponta (os centros das esferas ficam); a uniforme, as duas.
    const auto shrink = [&](std::size_t i, bool negativeEnd, float by) {
        RagdollCapsuleDefinition3D& capsule = profile.links[i].collider;
        const float half = ragdollCapsuleHalfSegment3D(capsule);
        float done = 0.0f;
        if (capsule.radiusAtPositiveXMeters <= 0.0f) {
            done = std::clamp(capsule.radiusMeters - floorNegative[i], 0.0f, by);
            capsule.radiusMeters -= done;
        } else if (negativeEnd) {
            done = std::clamp(capsule.radiusMeters - floorNegative[i], 0.0f, by);
            capsule.radiusMeters -= done;
        } else {
            done = std::clamp(capsule.radiusAtPositiveXMeters - floorPositive[i], 0.0f, by);
            capsule.radiusAtPositiveXMeters -= done;
        }
        capsule.lengthMeters = 2.0f * half + capsule.radiusMeters + ragdollCapsuleRadiusAtPositiveX3D(capsule);
        return done;
    };
    const auto worst = [&](const Pair& pair) {
        RagdollCapsuleContact3D out { std::numeric_limits<float>::max(), 0.0f, 0.0f };
        for (const RagdollAnimationPose3D& pose : poses) {
            const RagdollCapsuleContact3D c = ragdollPosedCapsuleContact3D(profile.links[pair.a], pose.linkPositions[pair.a],
                pose.linkOrientations[pair.a], profile.links[pair.b], pose.linkPositions[pair.b], pose.linkOrientations[pair.b]);
            if (c.gap < out.gap) out = c;
        }
        return out;
    };
    for (int round = 0; round < 40 && !poses.empty(); ++round) {
        bool changed = false;
        for (const Pair& pair : pairs) {
            const RagdollCapsuleContact3D contact = worst(pair);
            if (contact.gap >= pair.target - 1e-4f) continue;
            const float need = pair.target - contact.gap + 0.0005f;
            float done = 0.0f;
            if (isArm(pair.a) || isArm(pair.b)) {
                const bool first = isArm(pair.a);
                const std::size_t arm = first ? pair.a : pair.b;
                const bool negativeEnd = (first ? contact.alongA : contact.alongB) < 0.5f;
                done = shrink(arm, negativeEnd, need);
                if (const std::size_t other = mirror(arm); other != arm) shrink(other, negativeEnd, done);
            } else {
                done = shrink(pair.a, contact.alongA < 0.5f, 0.5f * need) + shrink(pair.b, contact.alongB < 0.5f, 0.5f * need);
            }
            changed = changed || done > 1e-6f;
        }
        if (!changed) break;
    }
    return profile;
}

void scaleRagdollSkinPalette3D(std::span<Mat4> palette, float scale) {
    if (scale == 1.0f) return;
    for (Mat4& matrix : palette)
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 3; ++column) matrix.at(row, column) *= scale;
}

} // namespace MatterEngine
