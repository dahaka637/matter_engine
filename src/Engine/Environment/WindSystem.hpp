#pragma once

#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

// Parametros calibraveis do vento. Pensados para virar sliders no painel de
// depuracao (mesmo padrao de ToneMappingSettings3D/FogSettings3D em
// Scene3D.hpp) - os valores padrao aqui sao so um ponto de partida
// fisicamente razoavel, nao um resultado calibrado a olho.
struct WindSettings3D {
    // Velocidade media do vento (m/s) na altura de referencia abaixo. Uma
    // brisa moderada real fica em torno de 3-5 m/s (escala Beaufort 2-3).
    float baseSpeedMetersPerSecond = 3.2f;
    // Quanto uma rajada pode somar/subtrair da velocidade media (m/s). Nunca
    // inverte o vento sozinha - isso e papel da direcao, nao da rajada.
    float gustAmplitudeMetersPerSecond = 1.6f;
    // Quao rapido a intensidade da rajada varia, em Hz (ciclos completos por
    // segundo). Vento real varia devagar - bem menor que 1 Hz e esperado.
    float gustFrequencyHz = 0.070f;
    // Quao rapido a direcao do vento vagueia, em Hz. Mais lento que a
    // rajada: a direcao muda de forma bem mais gradual que a intensidade.
    float directionWanderFrequencyHz = 0.015f;
    // Dois marcos de altura definem a curva de influencia do vento (ver
    // velocityAtHeight): abaixo de groundHeightMeters o vento tem influencia
    // ZERO (fisica e audio) - "linha do chao", como se o ar ali estivesse
    // parado, abrigado pelo terreno/estruturas ao redor. A partir de
    // referenceHeightMeters o vento vale sua forca cheia
    // (baseSpeedMetersPerSecond +- rajada). Entre os dois marcos a
    // transicao e suave (smoothstep), nunca um corte abrupto. Isto e um
    // efeito de jogo deliberadamente mais dramatico que um perfil
    // atmosferico real (que nunca chega a exatamente zero) - o objetivo e
    // uma sensacao clara de "no chao nao se sente vento nenhum, la em cima
    // sim", nao rigor meteorologico.
    float groundHeightMeters = 0.0f;
    // Altura em que o vento atinge forca plena. 10 m e o padrao usado por
    // estacoes meteorologicas reais para medir baseSpeedMetersPerSecond,
    // reaproveitado aqui tambem como o marco superior da rampa chao->ceu.
    float referenceHeightMeters = 10.0f;
};

// Fonte unica de verdade do vento do mundo: uma direcao que vagueia
// lentamente e uma velocidade que varia por rajadas, ambas deterministicas a
// partir do tempo decorrido (o mesmo tempo sempre produz o mesmo vento -
// reproduzivel para testes, e mantem render/fisica/audio concordando sobre
// "qual e o vento agora" sem precisar sincronizar estado entre si). Hoje
// consumida por tres lugares independentes: a rolagem de nuvens do ceu
// (LaboratoryScreen), o arrasto aerodinamico da fisica
// (PhysicsScene3D::setAirVelocity) e o assobio de vento no ouvido do jogador
// (WorldAudioController).
class WindSystem final {
public:
    explicit WindSystem(WindSettings3D settings = {});

    // Avanca o relogio interno do vento. Chamar uma vez por passo fixo de
    // simulacao (mesmo padrao de PhysicsScene3D::simulate) - a direcao e a
    // rajada sao funcoes puras do tempo acumulado, entao chamar mais de uma
    // vez por quadro sem necessidade dessincronizaria fisica/audio/render
    // entre si.
    void advance(float deltaTimeSeconds);

    // Velocidade do vento (m/s, mundo, Z-up, componente vertical sempre
    // zero) numa altura especifica acima do solo. Funcao pura em relacao ao
    // estado atual - pode ser chamada quantas vezes forem necessarias no
    // mesmo quadro (ex.: uma vez na altura do jogador, outra na altitude de
    // referencia das nuvens) sem custo de recalcular rajada/direcao.
    [[nodiscard]] Vec3 velocityAtHeight(float heightMeters) const;

    [[nodiscard]] const WindSettings3D& settings() const { return m_settings; }
    void setSettings(const WindSettings3D& settings) { m_settings = settings; }

    // Referencia mutavel direta - existe para o painel de depuracao poder
    // ligar sliders ImGui aos campos sem copiar a struct inteira a cada
    // quadro (mesmo padrao usado por ToneMappingSettings3D/FogSettings3D,
    // que sao membros diretos e mutaveis de WorkbenchApp).
    [[nodiscard]] WindSettings3D& mutableSettings() { return m_settings; }

private:
    WindSettings3D m_settings;
    float m_elapsedSeconds = 0.0f;
};

} // namespace MatterEngine
