#pragma once

namespace MatterEngine {

// A camada de reacoes por molas (28/10, item 10, pedido do usuario: mais
// "sensacao de fisica" na locomocao padrao, a fluidez do GASP da Unreal; ele
// pediu esta e a fisica de verdade sem exagero, e escolhe na pratica). O
// tronco e os bracos como pendulos no corpo, empurrados pela aceleracao dele
// no referencial dele: acelerando, o tronco fica um pouco para tras e passa do
// ponto quando ela acaba; numa troca brusca, o tronco e os bracos vao para
// fora da curva e voltam. A inclinacao ativa (a dos clipes e a da curva) fica;
// esta e so a inercia. Graus pequenos e com teto.
struct BodyReactionSettings3D {
    // O tronco: a frequencia (Hz), o amortecimento (1: critico), o giro por
    // m/s2 de aceleracao constante (graus) e o teto (graus).
    float trunkHertz = 3.0f;
    float trunkDamping = 0.5f;
    float trunkDegreesPerAcceleration = 0.6f;
    float trunkMaximumDegrees = 12.0f;
    // A aceleracao entra saturada suave (tanh) nisto (m/s2): a capsula vira em
    // 0,1 s com 40 m/s2 e, no teto duro, o tronco batia no limite e voltava seco.
    float accelerationSoftLimit = 12.0f;
    // Os bracos (no ombro), mais soltos.
    float armHertz = 1.8f;
    float armDamping = 0.3f;
    float armDegreesPerAcceleration = 1.3f;
    float armMaximumDegrees = 25.0f;
};

// O estado das molas (rad, rad/s). Tronco: pitch + para a frente, roll + para
// a esquerda. Bracos: swing + para a frente, side + para a esquerda.
struct BodyReaction3D {
    float trunkPitch = 0.0f, trunkPitchRate = 0.0f;
    float trunkRoll = 0.0f, trunkRollRate = 0.0f;
    float armSwing = 0.0f, armSwingRate = 0.0f;
    float armSide = 0.0f, armSideRate = 0.0f;

    // A aceleracao do corpo no referencial dele (m/s2: frente, esquerda).
    void step(float forwardAcceleration, float leftAcceleration, float deltaTime, const BodyReactionSettings3D& settings);
};

} // namespace MatterEngine
