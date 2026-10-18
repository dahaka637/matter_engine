#pragma once

#include <array>

namespace MatterEngine {

// O movimento pedido (rework de 10/10/2026, Etapa 3): a velocidade da capsula
// pela tabela de rumos e a resposta dela no modo padrao. Antes isto morava no
// laboratorio (e a velocidade era lida de dentro do estado do modo padrao);
// agora o laboratorio e a sonda pedem aqui.

// A velocidade (m/s) para um movimento que aponta travelRelativeToLookRadians
// em relacao ao olhar: a tabela tem 0, 45, 90, 135 e 180 graus (os dois lados
// iguais), interpolada entre eles; trote ou corrida.
[[nodiscard]] float athleteTravelSpeed3D(const std::array<float, 5>& trot, const std::array<float, 5>& sprint,
    float travelRelativeToLookRadians, bool sprinting);

// A resposta da capsula no modo padrao: aceleracao, desaceleracao, freada
// (m/s2) e giro (rad/s). Em 100% os valores aprovados (16 / 22 / 30, 10
// rad/s); a regua "Resposta" escala os quatro (menos e mais pesado - perto
// dos limites humanos, ~5-8 m/s2 -, mais e mais arcade).
struct AthleteCapsuleResponse3D {
    float acceleration = 16.0f;
    float deceleration = 22.0f;
    float brake = 30.0f;
    float turnRate = 10.0f;
    // A troca de direcao um pouco mais viva (1 e 1: a de antes): quantas vezes mais
    // forte freia ao inverter o rumo e arranca para o outro lado. Ainda para antes
    // de sair (os pes precisam desse instante).
    float reverseBrakeScale = 1.0f;
    float reverseAccelerationScale = 1.0f;
};
// reversalPercent (0 a 100, 0 = a troca de antes): 100 freia e arranca 1,5 vez mais
// forte na inversao do rumo (ajuste fino; a resposta escala o resto).
[[nodiscard]] AthleteCapsuleResponse3D athleteCapsuleResponse3D(float responsePercent, float reversalPercent = 0.0f);

} // namespace MatterEngine
