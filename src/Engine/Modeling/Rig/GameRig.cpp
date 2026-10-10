#include "Engine/Modeling/Rig/GameRig.hpp"

#include "Engine/Animation/RagdollVisualPose3D.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <optional>
#include <tuple>

namespace MatterEngine::Modeling {

namespace {

Side sideOf(const std::string& name) {
    if (name.rfind("Left", 0) == 0) return Side::Left;
    if (name.rfind("Right", 0) == 0) return Side::Right;
    return Side::Center;
}

float linkRadius(const RagdollLinkDefinition3D& link) {
    const RagdollCapsuleDefinition3D& collider = link.collider;
    if (collider.shape == RagdollColliderShape3D::Box) {
        // A meia medida do meio (a largura do pe, nao o comprimento).
        float extents[3] = { collider.boxHalfExtents.x, collider.boxHalfExtents.y, collider.boxHalfExtents.z };
        std::sort(extents, extents + 3);
        return extents[1];
    }
    return std::max(collider.radiusMeters, collider.radiusAtPositiveXMeters);
}

// O fim do colisor mais longe do pai (o alto da cabeca, sem malha).
Vec3 colliderFarEnd(const RagdollLinkDefinition3D& link, Vec3 parent) {
    const Transform frame { link.modelPosition, link.modelOrientation };
    const RagdollCapsuleDefinition3D& collider = link.collider;
    const Vec3 center = frame.apply(collider.localPosition);
    const Vec3 axis = axisX((link.modelOrientation * collider.localOrientation).normalized());
    const float half = collider.lengthMeters * 0.5f;
    const Vec3 a = center + axis * half, b = center - axis * half;
    return distance(a, parent) > distance(b, parent) ? a : b;
}

// Quanto os vertices dominados por um osso da paleta vao alem da junta no
// rumo dado (a ponta de verdade de um dedo, o alto da cabeca); -1 sem malha.
float meshReach(const RagdollCharacter3D& character, std::size_t paletteBone, Vec3 origin, Vec3 direction) {
    float reach = -1.0f;
    for (const MeshVertex3D& vertex : character.mesh.vertices) {
        const std::size_t strongest = static_cast<std::size_t>(
            std::max_element(vertex.weights.begin(), vertex.weights.end()) - vertex.weights.begin());
        if (vertex.weights[strongest] <= 0.0f || vertex.joints[strongest] != paletteBone) continue;
        reach = std::max(reach, dot(vertex.position - origin, direction));
    }
    return reach;
}

// O espelho no plano do meio (y = 0): a posicao e o giro (o referencial
// espelhado continua de mao direita: M R M).
Transform mirrored(const Transform& transform) {
    const Quaternion q = transform.orientation;
    return { { transform.position.x, -transform.position.y, transform.position.z }, Quaternion { -q.x, q.y, -q.z, q.w } };
}

bool isHandJoint(const std::string& name) {
    for (const char* part : { "Hand", "Thumb", "Index", "Middle", "Ring", "Pinky" })
        if (name.find(part) != std::string::npos) return true;
    return false;
}

// O referencial de um osso: x ao longo dele, y o secundario sem a parte ao
// longo, z o que sobra (de mao direita).
Quaternion boneFrame(Vec3 along, Vec3 secondary) {
    const Vec3 x = safeNormalized(along, { 1.0f, 0.0f, 0.0f });
    Vec3 y = secondary - x * dot(secondary, x);
    y = safeNormalized(y, std::abs(x.z) < 0.9f ? Vec3 { 0.0f, 0.0f, 1.0f } : Vec3 { 1.0f, 0.0f, 0.0f });
    const Vec3 z = cross(x, y);
    return quaternionFromBasis(x, y, z);
}

// A filha que segue o osso (a pelve vai ao abdomen, o alto do peito ao
// pescoco, a mao ao dedo medio; nos outros, a primeira).
int boneChild(const Skeleton& skeleton, int joint) {
    const std::vector<int> children = skeleton.children(joint);
    if (children.empty()) return -1;
    const std::string& name = skeleton.joints[static_cast<std::size_t>(joint)].name;
    const auto named = [&](std::string_view suffix) {
        for (const int child : children)
            if (skeleton.joints[static_cast<std::size_t>(child)].name.ends_with(suffix)) return child;
        return -1;
    };
    int child = -1;
    if (name == "Pelvis") child = named("Abdomen");
    else if (name == "UpperChest") child = named("Neck");
    else if (name.ends_with("Hand")) child = named("Middle1");
    for (const int candidate : children)
        if (child < 0 && skeleton.joints[static_cast<std::size_t>(candidate)].kind != JointKind::Helper) child = candidate;
    return child;
}

// Leva o esqueleto do jogo ao medido: posicoes do arquivo (o lado
// esquerdo em espelho do direito), cada osso girado da direcao do jogo para
// a medida (o giro em volta dele pelo lado da mao nos dedos, pela frente no
// resto). As pontas giram com o pai.
void applyJointFit(GameRig& rig, const std::vector<std::pair<std::string, Vec3>>& fit, float gameGround) {
    Skeleton& skeleton = rig.skeleton;
    const std::size_t count = skeleton.joints.size();
    std::vector<Transform> game(count);
    for (std::size_t j = 0; j < count; ++j) game[j] = skeleton.joints[j].bind;
    std::vector<Vec3> position(count);
    std::vector<bool> known(count, false);
    for (std::size_t j = 0; j < count; ++j) {
        const std::string& name = skeleton.joints[j].name;
        if (name.rfind("Left", 0) == 0) continue;
        const auto found = std::find_if(fit.begin(), fit.end(), [&](const auto& entry) { return entry.first == name; });
        if (found == fit.end()) {
            rig.fitMissing.push_back(name);
            continue;
        }
        position[j] = { found->second.x, found->second.y, found->second.z + skeleton.groundHeight };
        known[j] = true;
    }
    if (!rig.fitMissing.empty()) return;
    // O eixo de flexao dos dedos (de um lado ao outro da mao) no jogo e no
    // medido: o secundario dos ossos da mao.
    const auto lateral = [&](const std::string& side, bool fitted) {
        const int index = skeleton.find(side + "Index1"), pinky = skeleton.find(side + "Pinky1");
        if (index < 0 || pinky < 0) return Vec3 { 1.0f, 0.0f, 0.0f };
        return fitted ? position[static_cast<std::size_t>(index)] - position[static_cast<std::size_t>(pinky)]
                      : game[static_cast<std::size_t>(index)].position - game[static_cast<std::size_t>(pinky)].position;
    };
    // A normal da palma (o lado de um lado ao outro vezes o comprimento da
    // mao): o secundario do polegar, que corre quase ao longo do lado (com o
    // lado o referencial dele ficava solto e o punho o levava para fora).
    const auto palm = [&](const std::string& side, bool fitted) {
        const int hand = skeleton.find(side + "Hand"), middle = skeleton.find(side + "Middle1");
        if (hand < 0 || middle < 0) return Vec3 { 0.0f, 1.0f, 0.0f };
        const Vec3 length = fitted ? position[static_cast<std::size_t>(middle)] - position[static_cast<std::size_t>(hand)]
                                   : game[static_cast<std::size_t>(middle)].position - game[static_cast<std::size_t>(hand)].position;
        return cross(lateral(side, fitted), length);
    };
    // O plano em que um dedo da mao nova curva (as falanges e a ponta
    // medidas): a dobradica das falanges dele, do lado do "lado da mao" (o
    // -y continua fechando para a palma). Quase reto, nada. O indicador da
    // malha base curva 26 graus fora do lado da mao: com o lado, ele fechava
    // torto.
    const auto fingerPlane = [&](const std::string& name) -> std::optional<Vec3> {
        if (name.find("Hand") != std::string::npos || name.find("Thumb") != std::string::npos || name.empty()) return std::nullopt;
        const std::string base = name.substr(0, name.size() - 1);
        const int a = skeleton.find(base + "1"), b = skeleton.find(base + "2"), c = skeleton.find(base + "3"), d = skeleton.find(base + "Tip");
        if (a < 0 || b < 0 || c < 0 || d < 0) return std::nullopt;
        const Vec3 p1 = position[static_cast<std::size_t>(a)], p2 = position[static_cast<std::size_t>(b)];
        const Vec3 p3 = position[static_cast<std::size_t>(c)], tip = position[static_cast<std::size_t>(d)];
        const Vec3 normal = cross(p2 - p1, p3 - p2) + cross(p3 - p2, tip - p3);
        const float scale = (p2 - p1).length() * (p3 - p2).length() + (p3 - p2).length() * (tip - p3).length();
        if (normal.length() < 0.14f * scale) return std::nullopt;
        const Vec3 side = lateral("Right", true);
        return dot(normal, side) < 0.0f ? -normal : normal;
    };
    std::vector<Transform> bind(count);
    for (std::size_t j = 0; j < count; ++j) {
        if (!known[j]) continue;
        const Joint& joint = skeleton.joints[j];
        const int child = boneChild(skeleton, static_cast<int>(j));
        if (joint.kind == JointKind::Tip || child < 0 || !known[static_cast<std::size_t>(child)]) {
            bind[j] = { position[j], joint.parent >= 0 ? bind[static_cast<std::size_t>(joint.parent)].orientation : game[j].orientation };
            continue;
        }
        const Vec3 gameAlong = game[static_cast<std::size_t>(child)].position - game[j].position;
        const Vec3 fitAlong = position[static_cast<std::size_t>(child)] - position[j];
        Vec3 gameSecondary { 1.0f, 0.0f, 0.0f }, fitSecondary { 1.0f, 0.0f, 0.0f };
        if (joint.name.find("Thumb") != std::string::npos) {
            gameSecondary = palm("Right", false);
            fitSecondary = palm("Right", true);
        } else if (isHandJoint(joint.name)) {
            gameSecondary = lateral("Right", false);
            fitSecondary = fingerPlane(joint.name).value_or(lateral("Right", true));
        } else if (std::abs(safeNormalized(fitAlong, {}).x) > 0.8f || std::abs(safeNormalized(gameAlong, {}).x) > 0.8f) {
            gameSecondary = fitSecondary = { 0.0f, 0.0f, 1.0f };
        }
        const Quaternion offset = (boneFrame(fitAlong, fitSecondary) * boneFrame(gameAlong, gameSecondary).conjugate()).normalized();
        bind[j] = { position[j], (offset * game[j].orientation).normalized() };
    }
    for (std::size_t j = 0; j < count; ++j) {
        const std::string& name = skeleton.joints[j].name;
        if (name.rfind("Left", 0) != 0) continue;
        const int source = skeleton.find("Right" + name.substr(4));
        if (source < 0 || !known[static_cast<std::size_t>(source)]) {
            rig.fitMissing.push_back("Right" + name.substr(4));
            continue;
        }
        bind[j] = mirrored(bind[static_cast<std::size_t>(source)]);
    }
    if (!rig.fitMissing.empty()) return;
    for (std::size_t j = 0; j < count; ++j) skeleton.joints[j].bind = bind[j];
    // As pontas presas ao pai na ligacao nova; a raiz e quanto o movimento
    // dela escala.
    for (std::size_t j = 0; j < count; ++j) {
        const int parent = skeleton.joints[j].parent;
        if (skeleton.joints[j].kind == JointKind::Tip && parent >= 0)
            rig.tipLocal[j] = bind[static_cast<std::size_t>(parent)].inverse() * bind[j];
    }
    for (std::size_t j = 0; j < count; ++j)
        if (skeleton.joints[j].parent < 0) {
            rig.rootGameBind = game[j];
            const float gameHeight = game[j].position.z - gameGround;
            rig.rootScale = gameHeight > 1e-3f ? (bind[j].position.z - skeleton.groundHeight) / gameHeight : 1.0f;
            break;
        }
    rig.fitted = true;
}

// O repouso do esqueleto medido em pe no chao: a sola (os pontos do chao sob
// o tornozelo, atras dele, sob os dedos e a ponta deles na pose A, levados
// pelo pe e pelos dedos ate o repouso) toca o chao, e o movimento da raiz
// escala pela altura da pelve nesse repouso sobre a do jogo.
void standRestOnGround(const RagdollCharacter3D& character, GameRig& rig, float gameGround) {
    const Skeleton& skeleton = rig.skeleton;
    rig.rootRestLift = 0.0f;
    const SkeletonPose rest = sampleGameRigPose(character, rig, nullptr, 0.0f, RigHandPose::Bind, true);
    const std::vector<Transform> skin = skinningTransforms(skeleton, rest);
    const float ground = skeleton.groundHeight;
    float sole = std::numeric_limits<float>::max();
    for (const std::string side : { "Left", "Right" }) {
        const int foot = skeleton.find(side + "Foot"), toe = skeleton.find(side + "Toe"), tip = skeleton.find(side + "ToeTip");
        if (foot < 0 || toe < 0 || tip < 0) continue;
        const Vec3 ankle = skeleton.joints[static_cast<std::size_t>(foot)].bind.position;
        const Vec3 ball = skeleton.joints[static_cast<std::size_t>(toe)].bind.position;
        const Vec3 end = skeleton.joints[static_cast<std::size_t>(tip)].bind.position;
        Vec3 back { ankle.x - ball.x, ankle.y - ball.y, 0.0f };
        back = safeNormalized(back, { -1.0f, 0.0f, 0.0f }) * 0.06f;
        const Transform& footSkin = skin[static_cast<std::size_t>(foot)];
        const Transform& toeSkin = skin[static_cast<std::size_t>(toe)];
        for (const Vec3 point : { Vec3 { ankle.x, ankle.y, ground }, Vec3 { ankle.x + back.x, ankle.y + back.y, ground } })
            sole = std::min(sole, footSkin.apply(point).z);
        for (const Vec3 point : { Vec3 { ball.x, ball.y, ground }, Vec3 { end.x, end.y, ground } })
            sole = std::min(sole, toeSkin.apply(point).z);
    }
    if (sole == std::numeric_limits<float>::max()) return;
    rig.rootRestLift = ground - sole;
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
        if (skeleton.joints[j].parent < 0) {
            const float gameHeight = rig.rootGameBind.position.z - gameGround;
            const float restHeight = skeleton.joints[j].bind.position.z + rig.rootRestLift - ground;
            rig.rootScale = gameHeight > 1e-3f ? restHeight / gameHeight : 1.0f;
            break;
        }
}

} // namespace

GameRig buildGameRig(const RagdollCharacter3D& character, const GameRigAdjustments& adjustments) {
    const std::span<const LinkShift> shifts = adjustments.linkShifts;
    GameRig rig;
    Skeleton& skeleton = rig.skeleton;
    const RagdollProfile3D& profile = character.profile;
    // Os deslocamentos de link (espaco do modelo) e no referencial de cada um.
    std::vector<Vec3> shiftOf(profile.links.size());
    rig.linkOffset.assign(profile.links.size(), {});
    for (const LinkShift& shift : shifts)
        for (std::size_t i = 0; i < profile.links.size(); ++i)
            if (profile.links[i].id == shift.link) {
                shiftOf[i] = shift.offset;
                rig.linkOffset[i] = profile.links[i].modelOrientation.normalized().conjugate().rotate(shift.offset);
            }
    skeleton.groundHeight = adjustments.ground.value_or(-profile.standingRootHeightMeters);
    // O deslocamento de cada junta (o do link dela ou do ancestral): a malha
    // do jogo continua no lugar antigo (as pontas medem nela sem ele).
    std::vector<Vec3> jointShift;
    const auto addJoint = [&](Joint joint, int link, int visual, Transform tip, Transform frame) {
        const Vec3 moved = link >= 0 ? shiftOf[static_cast<std::size_t>(link)]
            : joint.parent >= 0 ? jointShift[static_cast<std::size_t>(joint.parent)] : Vec3 {};
        jointShift.push_back(moved);
        rig.jointInFrame.push_back(frame.inverse() * joint.bind);
        rig.frame.push_back(frame);
        skeleton.joints.push_back(std::move(joint));
        rig.link.push_back(link);
        rig.visualBone.push_back(visual);
        rig.tipLocal.push_back(tip);
        return static_cast<int>(skeleton.joints.size() - 1);
    };

    // Links fisicos (o perfil ja vem com pais antes dos filhos): a junta na
    // ancora (a raiz, sem ancora, na posicao dela).
    std::vector<int> linkJoint(profile.links.size(), -1);
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& link = profile.links[i];
        const Transform linkFrame { link.modelPosition + shiftOf[i], link.modelOrientation.normalized() };
        Joint joint;
        joint.name = link.id;
        joint.parent = link.parentIndex >= 0 ? linkJoint[static_cast<std::size_t>(link.parentIndex)] : -1;
        joint.bind = { (link.parentIndex >= 0 ? link.inboundJoint.anchorModelPosition : link.modelPosition) + shiftOf[i],
            linkFrame.orientation };
        joint.kind = JointKind::Body;
        joint.side = sideOf(link.id);
        joint.radius = std::max(0.02f, linkRadius(link));
        linkJoint[i] = addJoint(std::move(joint), static_cast<int>(i), -1, {}, linkFrame);
    }

    // Paleta: fisicos (boneLinkIndices) e depois os visuais.
    const std::size_t physicalBones = character.boneLinkIndices.size();
    rig.paletteJoint.assign(physicalBones + character.visualBones.size(), -1);
    for (std::size_t k = 0; k < physicalBones; ++k)
        if (character.boneLinkIndices[k] < linkJoint.size()) rig.paletteJoint[k] = linkJoint[character.boneLinkIndices[k]];

    // A ligacao de cada osso visual no pai; com o espelho, os dedos de um
    // lado copiam os do outro (os pais, as maos, ja sao o espelho um do
    // outro).
    const std::string mirrorFrom = adjustments.mirrorFingersFrom;
    const std::string mirrorTo = mirrorFrom == "Right" ? "Left" : mirrorFrom == "Left" ? "Right" : "";
    const auto counterpart = [&](const std::string& name) {
        return !mirrorTo.empty() && name.rfind(mirrorTo, 0) == 0 ? mirrorFrom + name.substr(mirrorTo.size()) : std::string();
    };
    rig.visualLocal.resize(character.visualBones.size());
    for (std::size_t v = 0; v < character.visualBones.size(); ++v)
        rig.visualLocal[v] = { character.visualBones[v].localBindPosition, character.visualBones[v].localBindOrientation };
    for (std::size_t v = 0; v < character.visualBones.size(); ++v) {
        const RagdollVisualBone3D& bone = character.visualBones[v];
        const std::string source = counterpart(bone.name);
        if (bone.kind != RagdollVisualBone3D::Kind::Finger || source.empty()) continue;
        for (std::size_t u = 0; u < character.visualBones.size(); ++u)
            if (character.visualBones[u].name == source) rig.visualLocal[v] = mirrored(rig.visualLocal[u]);
    }

    // Ossos visuais (dedos e dedos do pe): o pai vem antes na paleta.
    for (std::size_t v = 0; v < character.visualBones.size(); ++v) {
        const RagdollVisualBone3D& bone = character.visualBones[v];
        const int parent = bone.parentBone < rig.paletteJoint.size() ? rig.paletteJoint[bone.parentBone] : -1;
        if (parent < 0) continue;
        Joint joint;
        joint.name = bone.name;
        joint.parent = parent;
        joint.bind = rig.frame[static_cast<std::size_t>(parent)] * rig.visualLocal[v];
        joint.kind = bone.kind == RagdollVisualBone3D::Kind::Toe ? JointKind::Toe : JointKind::Finger;
        joint.side = sideOf(bone.name);
        joint.radius = joint.kind == JointKind::Toe ? 0.022f : 0.009f;
        const Transform own = joint.bind;
        rig.paletteJoint[physicalBones + v] = addJoint(std::move(joint), -1, static_cast<int>(v), {}, own);
    }

    // Pontas: o alto da cabeca, a ponta de cada dedo e dos dedos do pe.
    const auto addTip = [&](int parent, const std::string& name, Vec3 at, float radius) {
        const Transform& frame = skeleton.joints[static_cast<std::size_t>(parent)].bind;
        Joint joint;
        joint.name = name;
        joint.parent = parent;
        joint.bind = { at, frame.orientation };
        joint.kind = JointKind::Tip;
        joint.side = skeleton.joints[static_cast<std::size_t>(parent)].side;
        joint.radius = radius;
        const Transform tip = frame.inverse() * Transform { at, frame.orientation };
        const Transform own = joint.bind;
        addJoint(std::move(joint), -1, -1, tip, own);
    };
    // As pontas saem da malha do jogo quando ela existe (o esqueleto veio
    // dela): o vertice mais alto da cabeca, a ponta de cada dedo.
    const auto paletteOf = [&](std::size_t joint) {
        for (std::size_t k = 0; k < rig.paletteJoint.size(); ++k)
            if (rig.paletteJoint[k] == static_cast<int>(joint)) return k;
        return rig.paletteJoint.size();
    };
    const std::size_t declared = skeleton.joints.size();
    for (std::size_t j = 0; j < declared; ++j) {
        const Joint joint = skeleton.joints[j];
        if (!skeleton.children(static_cast<int>(j)).empty() || joint.parent < 0) continue;
        const Vec3 parent = skeleton.joints[static_cast<std::size_t>(joint.parent)].bind.position;
        if (rig.link[j] >= 0 && joint.name == "Head") {
            Vec3 top = colliderFarEnd(profile.links[static_cast<std::size_t>(rig.link[j])], parent);
            const float reach = meshReach(character, paletteOf(j), joint.bind.position - jointShift[j], { 0.0f, 0.0f, 1.0f });
            if (reach > 0.0f) top = joint.bind.position + Vec3 { 0.0f, 0.0f, reach };
            addTip(static_cast<int>(j), "HeadTop", top, joint.radius);
        } else if (joint.kind == JointKind::Toe) {
            const float length = character.visualBones[static_cast<std::size_t>(rig.visualBone[j])].length;
            addTip(static_cast<int>(j), joint.name + "Tip",
                joint.bind.position + axisX(joint.bind.orientation) * std::max(0.02f, length), 0.012f);
        } else if (joint.kind == JointKind::Finger) {
            // A ultima falange segue o rumo da anterior; o tamanho vem da
            // malha (ou 80% da anterior).
            const Vec3 step = joint.bind.position - parent;
            const Vec3 direction = safeNormalized(step, axisX(joint.bind.orientation));
            float length = step.length() * 0.8f;
            const float reach = meshReach(character, paletteOf(j), joint.bind.position - jointShift[j], direction);
            if (reach > 0.005f) length = reach;
            std::string name = joint.name;
            if (!name.empty() && name.back() >= '0' && name.back() <= '9') name.pop_back();
            addTip(static_cast<int>(j), name + "Tip", joint.bind.position + direction * length, 0.008f);
        }
    }
    // As pontas dos dedos espelhadas como os ossos (a malha do jogo as poe
    // um pouco diferentes dos dois lados).
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        Joint& joint = skeleton.joints[j];
        if (joint.kind != JointKind::Tip || joint.parent < 0
            || skeleton.joints[static_cast<std::size_t>(joint.parent)].kind != JointKind::Finger)
            continue;
        const std::string source = counterpart(joint.name);
        const int from = source.empty() ? -1 : skeleton.find(source);
        if (from < 0) continue;
        joint.bind = mirrored(skeleton.joints[static_cast<std::size_t>(from)].bind);
        rig.frame[j] = joint.bind;
        rig.tipLocal[j] = skeleton.joints[static_cast<std::size_t>(joint.parent)].bind.inverse() * joint.bind;
    }
    // O esqueleto medido num corpo novo (as auxiliares saem dele).
    if (!adjustments.joints.empty()) applyJointFit(rig, adjustments.joints, -profile.standingRootHeightMeters);
    // As juntas auxiliares do skin: giram a metade (a quina nao perde
    // volume) no joelho, na virilha, no tornozelo, no ombro, no cotovelo e
    // no pulso. Na virilha tambem 1/4 e 3/4 do giro: o calcao passa da pelve
    // para a coxa em aneis que giram inteiros em volta da junta (como uma
    // junta de bola), sem a media dos pesos puxar o pano para dentro. E a da
    // dobra da frente (HipLift): so a flexao (o giro em volta do eixo de
    // lado), 45% dela - a coxa que sobe dobra a cintura do calcao e a barra
    // da camisa para tras e a barra fica em cima da perna levantada a 90
    // graus (menos que ~43%, a perna atravessava a barra); abrir de lado ou
    // torcer nao mexe nelas (girando junto, o pano virava no trote de lado).
    constexpr float LiftFraction = 0.45f;
    for (const char* side : { "Left", "Right" }) {
        const std::string prefix(side);
        const Vec3 none {}, across { 0.0f, 1.0f, 0.0f };
        const std::array<std::tuple<std::string, std::string, std::string, float, Vec3>, 9> helpers { {
            { prefix + "KneeHelper", prefix + "Thigh", prefix + "Shin", 0.5f, none },
            { prefix + "HipHelper", "Pelvis", prefix + "Thigh", 0.5f, none },
            { prefix + "HipHelper25", "Pelvis", prefix + "Thigh", 0.25f, none },
            { prefix + "HipHelper75", "Pelvis", prefix + "Thigh", 0.75f, none },
            { prefix + "HipLift", "Pelvis", prefix + "Thigh", LiftFraction, across },
            { prefix + "AnkleHelper", prefix + "Shin", prefix + "Foot", 0.5f, none },
            { prefix + "ShoulderHelper", "UpperChest", prefix + "UpperArm", 0.5f, none },
            { prefix + "ElbowHelper", prefix + "UpperArm", prefix + "Forearm", 0.5f, none },
            { prefix + "WristHelper", prefix + "Forearm", prefix + "Hand", 0.5f, none } } };
        for (const auto& [name, from, to, fraction, axis] : helpers) {
            const int upper = skeleton.find(from), lower = skeleton.find(to);
            if (upper < 0 || lower < 0) continue;
            const int helper = addHelperJoint(skeleton, name, upper, lower, fraction, axis);
            if (helper < 0) continue;
            rig.link.push_back(-1);
            rig.visualBone.push_back(-1);
            rig.tipLocal.push_back({});
            rig.frame.push_back(skeleton.joints[static_cast<std::size_t>(helper)].bind);
            rig.jointInFrame.push_back({});
        }
    }
    if (rig.fitted) standRestOnGround(character, rig, -profile.standingRootHeightMeters);
    return rig;
}

SkeletonPose withGameHands(const RagdollCharacter3D& character, const GameRig& rig, const SkeletonPose& base, RigHandPose hand) {
    const Skeleton& skeleton = rig.skeleton;
    if (hand == RigHandPose::Bind || base.joints.size() != skeleton.joints.size()) return base;
    const SkeletonPose game = sampleGameRigPose(character, rig, nullptr, 0.0f, hand, true);
    std::vector<int> joints = fingerJoints(skeleton, Side::Right, true);
    const std::vector<int> left = fingerJoints(skeleton, Side::Left, true);
    joints.insert(joints.end(), left.begin(), left.end());
    // Cada falange toma o giro do jogo no pai (presas ao pai: a ponta vai
    // junto com a base).
    return bendPose(skeleton, base, blendJointBends(skeleton, base, game, joints, 1.0f));
}

SkeletonPose sampleGameRigPose(const RagdollCharacter3D& character, const GameRig& rig, const AnimationClip3D* clip,
    float timeSeconds, bool relaxedFingers, bool inPlace) {
    return sampleGameRigPose(character, rig, clip, timeSeconds, relaxedFingers ? RigHandPose::Relaxed : RigHandPose::Bind,
        inPlace);
}

SkeletonPose sampleGameRigPose(const RagdollCharacter3D& character, const GameRig& rig, const AnimationClip3D* clip,
    float timeSeconds, RigHandPose hand, bool inPlace) {
    const RagdollAnimationPose3D links = sampleRagdollAnimationPose3D(character.profile, clip, timeSeconds, true);
    // O giro de cada osso visual (dedos e dedos do pe) desde a ligacao.
    std::vector<Quaternion> visualRotation(character.visualBones.size());
    if (hand == RigHandPose::Relaxed) {
        for (std::size_t v = 0; v < visualRotation.size(); ++v) visualRotation[v] = character.visualBones[v].restRotation;
    } else if (hand == RigHandPose::Open || hand == RigHandPose::Fist) {
        const RagdollHandPose3D pose = hand == RigHandPose::Open ? RagdollHandPose3D::Open : RagdollHandPose3D::Fist;
        RagdollVisualPoseInput3D input;
        input.linkPositions = links.linkPositions;
        input.linkOrientations = links.linkOrientations;
        input.handPose = { pose, pose };
        input.toesFollowGround = false;
        RagdollVisualPose3D visual;
        visualRotation = visual.update(character, input);
        // Os dedos do pe ficam como no repouso.
        for (std::size_t v = 0; v < visualRotation.size(); ++v)
            if (character.visualBones[v].kind != RagdollVisualBone3D::Kind::Finger)
                visualRotation[v] = character.visualBones[v].restRotation;
    }
    return gameRigPoseFromLinks(character, rig, links, visualRotation, inPlace);
}

SkeletonPose gameRigPoseFromLinks(const RagdollCharacter3D& character, const GameRig& rig,
    const RagdollAnimationPose3D& links, std::span<const Quaternion> visualRotationIn, bool inPlace) {
    std::vector<Quaternion> visualRotation(visualRotationIn.begin(), visualRotationIn.end());
    if (visualRotation.size() != character.visualBones.size()) {
        visualRotation.resize(character.visualBones.size());
        for (std::size_t v = 0; v < visualRotation.size(); ++v) visualRotation[v] = character.visualBones[v].restRotation;
    }
    const Skeleton& skeleton = rig.skeleton;
    SkeletonPose pose;
    pose.joints.resize(skeleton.joints.size());
    Vec3 shift {};
    if (inPlace && !links.linkPositions.empty() && !character.profile.links.empty()) {
        const Vec3 moved = links.linkPositions.front() - character.profile.links.front().modelPosition;
        shift = { -moved.x, -moved.y, 0.0f };
    }
    // O referencial de cada junta na pose (o do link, ou o do osso visual).
    std::vector<Transform> frames(skeleton.joints.size());
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const int parent = skeleton.joints[j].parent;
        if (rig.link[j] >= 0) {
            const std::size_t link = static_cast<std::size_t>(rig.link[j]);
            const Vec3 offset = link < rig.linkOffset.size() ? links.linkOrientations[link].rotate(rig.linkOffset[link]) : Vec3 {};
            frames[j] = { links.linkPositions[link] + offset + shift, links.linkOrientations[link] };
            pose.joints[j] = frames[j] * rig.jointInFrame[j];
            // A junta fica onde o pai a poe (a ligacao relativa do esqueleto
            // do modelo): o jogo da os giros dos links; com um link deslocado
            // (o braco a frente), cada um girando o proprio deslocamento
            // separaria as juntas (o cotovelo dobrado saia do lugar). O link
            // anda junto (os dedos se prendem nele).
            if (parent >= 0) {
                const Transform& parentBind = skeleton.joints[static_cast<std::size_t>(parent)].bind;
                const Vec3 expected = (pose.joints[static_cast<std::size_t>(parent)]
                                          * (parentBind.inverse() * skeleton.joints[j].bind))
                                          .position;
                const Vec3 correction = expected - pose.joints[j].position;
                pose.joints[j].position += correction;
                frames[j].position += correction;
            } else if (rig.fitted) {
                // A raiz do esqueleto medido: em pe no repouso, o movimento da
                // do jogo na escala da altura nova.
                const Vec3 expected = skeleton.joints[j].bind.position + Vec3 { 0.0f, 0.0f, rig.rootRestLift }
                    + (pose.joints[j].position - rig.rootGameBind.position) * rig.rootScale;
                const Vec3 correction = expected - pose.joints[j].position;
                pose.joints[j].position += correction;
                frames[j].position += correction;
            }
        } else if (rig.visualBone[j] >= 0 && parent >= 0) {
            const std::size_t visual = static_cast<std::size_t>(rig.visualBone[j]);
            const Transform& bind = rig.visualLocal[visual];
            const Quaternion local = (bind.orientation * visualRotation[visual]).normalized();
            frames[j] = frames[static_cast<std::size_t>(parent)] * Transform { bind.position, local };
            // No esqueleto medido, o dedo fica onde o pai o poe (as medidas
            // da mao nova, nao as do jogo); o giro e o do jogo.
            if (rig.fitted) {
                const Transform& parentBind = skeleton.joints[static_cast<std::size_t>(parent)].bind;
                frames[j].position = (pose.joints[static_cast<std::size_t>(parent)]
                                         * (parentBind.inverse() * skeleton.joints[j].bind))
                                         .position;
            }
            pose.joints[j] = frames[j];
        } else if (skeleton.joints[j].kind == JointKind::Helper) {
            pose.joints[j] = skeleton.joints[j].bind;
        } else if (parent >= 0) {
            pose.joints[j] = pose.joints[static_cast<std::size_t>(parent)] * rig.tipLocal[j];
        } else {
            pose.joints[j] = skeleton.joints[j].bind;
        }
    }
    updateDerivedJoints(skeleton, pose);
    return pose;
}

} // namespace MatterEngine::Modeling
