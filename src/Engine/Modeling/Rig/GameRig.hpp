#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

// A ponte entre o personagem do jogo (RagdollCharacter3D: os 18 links
// fisicos e os ossos visuais dos dedos) e o esqueleto da modelagem. E a
// unica parte da modelagem que conhece a engine.
namespace MatterEngine::Modeling {

// As juntas ficam nos pivos de verdade (a ancora da junta de cada link: o
// quadril, o joelho, o tornozelo...), nao na posicao do link, que no perfil e
// o meio do pedaco do corpo. Os dedos continuam presos ao referencial do link
// (e assim que o jogo os monta).
// Um link fisico deslocado no esqueleto do modelo (metros, no espaco do
// modelo na ligacao): o modelo novo pode por o osso onde ele fica melhor (o
// braco no meio do braco) antes do perfil fisico do jogo ser refeito para
// ele. O link leva junto a junta dele, os ossos visuais e as pontas.
struct LinkShift {
    std::string link;
    Vec3 offset;
};

// Os ajustes do modelo novo sobre o esqueleto do jogo (model.json,
// "skeleton"): os links deslocados e os dedos de um lado ("Right" ou "Left")
// copiados em espelho no outro - a malha e espelhada, entao os ossos dos dois
// lados precisam ser o espelho exato (no jogo eles diferem uns milimetros).
struct GameRigAdjustments {
    std::vector<LinkShift> linkShifts;
    std::string mirrorFingersFrom;
    // O esqueleto medido num corpo novo (esqueleto.json): a posicao de cada
    // junta do lado direito e do meio ([x, y, altura do chao]); o esquerdo e
    // o espelho. Cada osso fica na direcao medida e gira, nas poses, como o
    // do jogo (a orientacao da ligacao e a do jogo levada ate ela).
    std::vector<std::pair<std::string, Vec3>> joints;
    // O chao do espaco do modelo (model.json "skeleton"."ground"): fixo,
    // porque as pecas, a pintura e os ajustes do rosto foram postos nele.
    // Sem ele, o do perfil do jogo (-standingRootHeightMeters). O perfil
    // exportado tem a pelve na origem: o chao dele e outro.
    std::optional<float> ground;
};

struct GameRig {
    Skeleton skeleton;
    // Por link fisico: o deslocamento no referencial do link (as poses o
    // levam junto, girando com o link).
    std::vector<Vec3> linkOffset;
    // Por junta do esqueleto: o link fisico (ou -1) e o osso visual (ou -1);
    // pontas tem os dois -1 e um deslocamento fixo no pai.
    std::vector<int> link;
    std::vector<int> visualBone;
    std::vector<Transform> tipLocal;
    // Por junta: o referencial onde os filhos visuais se prendem (o do link,
    // ou o do proprio osso visual) e a junta escrita nele.
    std::vector<Transform> frame;
    std::vector<Transform> jointInFrame;
    // Por osso da paleta do jogo (fisicos, depois visuais): a junta.
    std::vector<int> paletteJoint;
    // Por osso visual (na ordem do personagem): a ligacao no pai (a do jogo,
    // ou o espelho do outro lado).
    std::vector<Transform> visualLocal;
    // Com o esqueleto medido: a ligacao da raiz no jogo; quanto a raiz sobe
    // no repouso (a pose T do jogo tem as pernas retas e verticais - na pose
    // A da malha elas abrem e inclinam -, entao a pelve sobe ate a sola tocar
    // o chao); quanto o movimento dela escala (a altura da pelve nesse
    // repouso sobre a do jogo: 1 com o jogador exportado); as juntas que
    // faltaram no arquivo (vazio = todas la).
    bool fitted = false;
    Transform rootGameBind;
    float rootRestLift = 0.0f;
    float rootScale = 1.0f;
    std::vector<std::string> fitMissing;
};

// O esqueleto a partir do personagem do jogo, na pose de ligacao (T, mao
// aberta como no FBX). Pontas: HeadTop, <Dedo>Tip e <Lado>ToeTip.
[[nodiscard]] GameRig buildGameRig(const RagdollCharacter3D& character, const GameRigAdjustments& adjustments = {});

// A mao numa pose: a da ligacao (dedos retos), a de repouso do jogo, ou a
// aberta e a fechada do jogo (as tabelas de RagdollVisualPose3D).
enum class RigHandPose : std::uint8_t { Bind, Relaxed, Open, Fist };

// A pose numa animacao (clip nulo = ligacao). relaxedFingers usa a mao de
// repouso do jogo; inPlace tira o deslocamento horizontal da pelve.
[[nodiscard]] SkeletonPose sampleGameRigPose(const RagdollCharacter3D& character, const GameRig& rig,
    const AnimationClip3D* clip, float timeSeconds, bool relaxedFingers = true, bool inPlace = true);
// O mesmo com a mao pedida (as duas maos).
[[nodiscard]] SkeletonPose sampleGameRigPose(const RagdollCharacter3D& character, const GameRig& rig,
    const AnimationClip3D* clip, float timeSeconds, RigHandPose hand, bool inPlace = true);
// A pose a partir dos links fisicos ja postos (no espaco do modelo do jogo: a
// pelve do repouso na origem, o chao em -standingRootHeightMeters) e do giro
// de cada osso visual desde a ligacao (vazio = o repouso do jogo). E o que
// as folhas de quadros usam para desenhar uma gravacao do jogo.
[[nodiscard]] SkeletonPose gameRigPoseFromLinks(const RagdollCharacter3D& character, const GameRig& rig,
    const RagdollAnimationPose3D& links, std::span<const Quaternion> visualRotation, bool inPlace = false);

// Uma pose com os dedos (polegar incluido) da mao do jogo: o giro de cada um
// na mao, como no jogo; o resto fica como esta (os testes de dobra partem da
// ligacao com a mao solta, aberta ou fechada do jogo).
[[nodiscard]] SkeletonPose withGameHands(const RagdollCharacter3D& character, const GameRig& rig, const SkeletonPose& base,
    RigHandPose hand);

} // namespace MatterEngine::Modeling
