#pragma once

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/Animation/RagdollCharacter3D.hpp"
#include "Engine/Physics/RagdollProfileBody3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"

#include <span>
#include <vector>

// O corpo do jogador no personagem (26/10): a altura (x s em tudo) e o tonus
// (a forma da malha em volta dos ossos). A lei esta em Engine/Physique; o
// perfil fisico, em RagdollProfileBody3D.
namespace MatterEngine {

// O tonus na pele, por vertice e na ligacao da malha (antes da altura): o
// deslocamento e a mudanca da normal no tonus +1 (a forma encorpada, a folha
// de 90 kg) e -1 (a franzina, a de 70). No tonus w o vertice anda w vezes o
// deles (|w| vezes o franzino abaixo de 0). O campo: cada osso da pele leva o
// vertice em volta do eixo do link dele (bodyGirthAxes3D, na ligacao) pela
// grossura da regiao, nos pesos da pele (os auxiliares repartem entre o osso
// de cima e o de baixo pela fracao; os dedos seguem a mao e o pe). A normal sai
// da malha deformada (as faces em volta de cada vertice, no grupo de
// suavizacao dele) menos a da malha de hoje pela mesma conta.
struct RagdollToneDeltas3D {
    std::vector<Vec3> heavyPosition, lightPosition;
    std::vector<Vec3> heavyNormal, lightNormal;
    [[nodiscard]] bool empty() const { return heavyPosition.empty(); }
};
[[nodiscard]] RagdollToneDeltas3D computeRagdollToneDeltas3D(const RagdollCharacter3D& character);
// O deslocamento e a normal de um vertice no tonus w.
[[nodiscard]] Vec3 ragdollTonePosition3D(const RagdollToneDeltas3D& deltas, std::size_t vertex, Vec3 position, float tone);
[[nodiscard]] Vec3 ragdollToneNormal3D(const RagdollToneDeltas3D& deltas, std::size_t vertex, Vec3 normal, float tone);

// O personagem de um corpo: o perfil do corpo; as ligacoes, os ossos visuais
// e os auxiliares x s; e a malha x s, no tonus quando tone vem (sem ele, a
// forma de hoje em escala). E o caminho da CPU (a modelagem, as ferramentas,
// os testes): quem desenha na GPU usa a malha de hoje com os deltas e a
// paleta x S(s), nunca esta malha. No corpo de hoje, a copia do original.
// Com os clipes de locomocao (ragdollClearanceClips3D), o perfil leva a folga
// deles (bodyRagdollProfileForClips3D).
[[nodiscard]] RagdollCharacter3D bodyRagdollCharacter3D(const RagdollCharacter3D& base, const PlayerBody3D& body,
    const RagdollToneDeltas3D* tone, std::span<const AnimationClip3D* const> clearanceClips = {});

// Os clipes de locomocao que a folga do corpo confere (os do manifesto do
// personagem, fora os levantar, o carrinho e a arrancada agachada: no chao a
// mao encosta na perna de proposito) - a lista do exportador.
[[nodiscard]] std::vector<const AnimationClip3D*> ragdollClearanceClips3D(const RagdollCharacter3D& character,
    const std::vector<AnimationClip3D>& library);
// O clipe no corpo: a raiz x s e o braco aberto pelo tonus (bodyArmOpening3D).
[[nodiscard]] AnimationClip3D bodyRagdollClip3D(const AnimationClip3D& clip, const RagdollProfile3D& profile,
    const PlayerBody3D& body);
// O perfil do corpo com a folga do exportador nos clipes de locomocao do corpo
// (o braco a 4 mm do tronco e as coxas a 8 mm, x s): onde o corpo grosso
// encosta, a ponta do colisor do braco (ou das duas coxas) encolhe, ate 70%
// dela, igual dos dois lados. Sem clipes, o bodyRagdollProfile3D. No corpo de
// hoje, o original.
[[nodiscard]] RagdollProfile3D bodyRagdollProfileForClips3D(const RagdollProfile3D& base, const PlayerBody3D& body,
    std::span<const AnimationClip3D* const> clips);

// A altura sem copiar a malha: a paleta do personagem do corpo vezes S(s) a
// direita (cada matriz leva o vertice da malha de hoje, ja no tonus, como o
// vertice x s - os auxiliares e a clavicula inclusive; a normal sai certa
// porque o shader a normaliza).
void scaleRagdollSkinPalette3D(std::span<Mat4> palette, float scale);

} // namespace MatterEngine
