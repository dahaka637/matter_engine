#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

// O corpo do jogador (26/10, pedido do usuario): a altura de 1,70 a 2,00 m e o
// peso de 60 a 100 kg, continuos. So a lei - altura e peso dao a escala, o
// tonus de cada regiao, a massa e os motores; quem aplica e a fisica (o
// perfil), o personagem (a malha) e o atleta (os clipes, a capsula, o
// alcance). So matematica: nada da engine. O plano e as medidas estao no
// vault (03_entidades/plano_corpo_variavel.md).
namespace MatterEngine {

struct PlayerBody3D {
    float heightMeters = 1.80f;
    float massKg = 80.0f;
    bool operator==(const PlayerBody3D&) const = default;
};

// O corpo de hoje (a malha mede 1,7995 m; o perfil soma 80 kg): nele nada e
// recalculado - quem aplica devolve o original, bit a bit.
inline constexpr float BaseBodyHeightMeters = 1.80f;
inline constexpr float BaseBodyMassKg = 80.0f;
inline constexpr float MinimumBodyHeightMeters = 1.70f;
inline constexpr float MaximumBodyHeightMeters = 2.00f;
inline constexpr float MinimumBodyMassKg = 60.0f;
inline constexpr float MaximumBodyMassKg = 100.0f;

[[nodiscard]] constexpr bool isBaseBody3D(const PlayerBody3D& body) {
    return body.heightMeters == BaseBodyHeightMeters && body.massKg == BaseBodyMassKg;
}
// Dentro das faixas (o que nao for finito vira o corpo de hoje).
[[nodiscard]] PlayerBody3D clampedBody3D(PlayerBody3D body);
// "182,77" ou "182,77.5" (cm e kg, separados por virgula, x ou /): o corpo
// das ferramentas de linha de comando (--corpo). Nada fora das faixas.
[[nodiscard]] std::optional<PlayerBody3D> parseBody3D(std::string_view text);

// A escala dos comprimentos (o mesmo fisico em escala, como a folha de
// alturas dele): altura / 1,80.
[[nodiscard]] float bodyScale3D(const PlayerBody3D& body);

// O tonus (w): a forma do corpo, -1 na franzina da folha de pesos dele e +1 na
// encorpada, 0 no corpo de hoje. Vem do peso em relacao a altura (ordem dele,
// 26/10 a noite: "um jogador de 2.00m com 90 kg vai ter menos massa corporal do
// que se ele tivesse 1.70"): o peso equivalente a 1,80 m e peso / s^3 nos
// baixos (a malha e o mesmo fisico em escala; a ANSUR II da o expoente ~2,9) e
// peso / s^2 nos altos (o ajuste dele, 26/10: os altos pesados ficavam franzinos
// demais), e o tonus e o da forma cuja malha tem esse volume
// (bodyToneVolume3D): a franzina pesa 70 kg; a encorpada, ~99 kg. Abaixo de
// -2,4 (os altos e leves) um terco do ritmo e acima de +0,9 (os pesados) a
// metade: a malha nao dobra e o pesado nao fica rechonchudo (o pedido dele).
// Nos altos, entao, a malha tem mais volume que o peso: a forma e a do olho
// dele, a fisica continua com o peso de verdade.
[[nodiscard]] float bodyTone3D(const PlayerBody3D& body);
// O volume da malha no tonus w sobre o de hoje, na mesma altura.
[[nodiscard]] float bodyToneVolume3D(float tone);

// As regioes do corpo, uma por link fisico (os dois lados iguais).
enum class PhysiqueRegion3D : std::uint8_t {
    Pelvis, Abdomen, Chest, UpperChest, Neck, Head,
    UpperArm, Forearm, Hand, Thigh, Shin, Foot,
    Count
};
// A regiao do link pelo nome do perfil (sem o Left/Right na frente).
[[nodiscard]] std::optional<PhysiqueRegion3D> physiqueRegionOfLink3D(std::string_view linkId);
// A grossura da regiao (as larguras em volta do eixo do osso; 1 no corpo de
// hoje) no tonus w: as razoes medidas na folha de pesos dele (70 e 90 kg sobre
// o de 80) em -1 e +1, em linha reta de cada lado (como a malha).
[[nodiscard]] float bodyGirth3D(PhysiqueRegion3D region, float tone);
// A grossura de frente para tras (a profundidade, x do corpo): a mesma, menos a
// cintura do franzino, que so afina dos lados.
[[nodiscard]] float bodyGirthDepth3D(PhysiqueRegion3D region, float tone);
// A profundidade para tras (as costas): a mesma, menos o peito, o peito alto e
// o pescoco do encorpado, que crescem pouco para tras (sem corcunda, ordem dele:
// o musculo vai para os lados e para a frente).
[[nodiscard]] float bodyGirthBack3D(PhysiqueRegion3D region, float tone);

// O braco do encorpado um pouco aberto nos clipes (radianos, para fora; 0 no
// franzino): o braco grosso encostava no tronco grosso. 7 graus por unidade
// de tonus (~7 a 1,80 m e 100 kg, ~10 no de 1,70 m); o resto e o colisor que
// encolhe na ponta.
[[nodiscard]] float bodyArmOpening3D(float tone);

// Os motores (a carga da gravidade, M g L): o torque maximo x (M/80) s; a
// rigidez e o amortecimento x (M/80) s^2 (a mesma frequencia e o mesmo
// amortecimento relativo num corpo de inercia M L^2).
struct BodyMotorScale3D {
    float torque = 1.0f;
    float stiffness = 1.0f;
    float damping = 1.0f;
};
[[nodiscard]] BodyMotorScale3D bodyMotorScale3D(const PlayerBody3D& body);

} // namespace MatterEngine
