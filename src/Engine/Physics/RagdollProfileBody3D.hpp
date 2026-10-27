#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"

#include <vector>

namespace MatterEngine {

// O eixo em volta do qual a grossura de um link muda (o tonus), no espaco de
// modelo do perfil (o repouso): passa pela junta de entrada (a pelve, pela
// posicao dela) e segue o osso ate a junta do filho que continua a cadeia (a
// coluna ate o pescoco; a coxa ate o joelho...). A mao segue o colisor; a
// cabeca, o alto. A malha (o personagem) usa o mesmo eixo levado a ligacao.
struct BodyGirthAxis3D {
    Vec3 point;
    Vec3 direction { 0.0f, 0.0f, 1.0f };
    // Ate a junta do filho que continua a cadeia (m; 0 e -1 sem ele).
    float length = 0.0f;
    int child = -1;
};
[[nodiscard]] std::vector<BodyGirthAxis3D> bodyGirthAxes3D(const RagdollProfile3D& profile);
// A grossura num ponto do link (girth: a da regiao de cada link): a dele e,
// perto da junta do filho que continua a cadeia, indo ate a do filho - sem
// degrau na junta (a canela chega a chuteira rigida, o pescoco a cabeca).
[[nodiscard]] float bodyGirthAlong3D(const std::vector<BodyGirthAxis3D>& axes, const std::vector<float>& girth,
    std::size_t link, Vec3 point);

// O perfil fisico de um corpo (altura e peso) a partir do de hoje:
// - os comprimentos x s (as posicoes, as ancoras, os colisores, a chuteira, a
//   altura de pe);
// - a grossura de cada colisor em volta do eixo do link, a da regiao no tonus
//   (o mesmo campo da malha); as coxas mantem a folga do exportador em
//   repouso (sobrepostas, o backend desligaria o par);
// - a massa total e a do jogador: cada link fica com a dele mais a parte do
//   ganho (ou da perda) pelo volume que a forma ganhou (a cabeca e a chuteira
//   quase nada);
// - os motores pela carga (bodyMotorScale3D).
// Os limites, os eixos, os ids e os pares ignorados nao mudam. No corpo de
// hoje devolve o perfil intacto.
[[nodiscard]] RagdollProfile3D bodyRagdollProfile3D(const RagdollProfile3D& base, const PlayerBody3D& body);

// A menor distancia entre as superficies de duas capsulas (conicas
// inclusive), no espaco de modelo, com os links em repouso (negativa:
// sobrepostas).
[[nodiscard]] float ragdollRestCapsuleGap3D(const RagdollLinkDefinition3D& a, const RagdollLinkDefinition3D& b);
// O mesmo com os links numa pose (a posicao e o giro de cada um no mundo); o
// contato diz tambem onde fica em cada capsula (0 a ponta -x, 1 a +x).
struct RagdollCapsuleContact3D {
    float gap = 0.0f;
    float alongA = 0.0f;
    float alongB = 0.0f;
};
[[nodiscard]] RagdollCapsuleContact3D ragdollPosedCapsuleContact3D(const RagdollLinkDefinition3D& a, Vec3 positionA,
    Quaternion orientationA, const RagdollLinkDefinition3D& b, Vec3 positionB, Quaternion orientationB);
[[nodiscard]] float ragdollPosedCapsuleGap3D(const RagdollLinkDefinition3D& a, Vec3 positionA, Quaternion orientationA,
    const RagdollLinkDefinition3D& b, Vec3 positionB, Quaternion orientationB);

} // namespace MatterEngine
