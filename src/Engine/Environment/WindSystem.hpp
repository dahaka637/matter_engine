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

    // --- Envelope de calmaria/vento forte ---
    // Ruido MUITO mais lento que rajada/direcao, que multiplica a velocidade
    // inteira (ver velocityAtHeight). E o que separa "textura de rajada" (que
    // ja existia) de "as vezes nao ha vento nenhum, as vezes ha" (que nao
    // existia): a rajada sozinha nunca chega perto de zero, so oscila em
    // torno da media. Periodo bem mais longo que os outros dois canais de
    // ruido de proposito - deve ler como "o clima mudou", nao como "mais uma
    // rajada".
    float calmEnvelopeFrequencyHz = 0.0020f;
    // Piso do envelope (nunca exatamente 0): evita um vetor de vento
    // degenerado durante calmaria total, e 5% do pico (base+rajada) ja fica
    // abaixo do limiar de silencio do uivo ambiente (ver
    // WindAmbientQuietThresholdMetersPerSecond em WorldAudioController.cpp),
    // entao continua soando como silencio total.
    float calmEnvelopeFloor = 0.05f;

    // --- Resquicio de rajada forte perto do chao ---
    // Abaixo de ~30cm a rampa chao->ceu ja entrega uma fracao praticamente
    // nula sozinha (smoothstep(0.3/10) ~ 0.27%), entao "praticamente nada"
    // ali ja acontece sem nenhum campo novo. O que falta e a excecao: um
    // resquicio RARO e ALEATORIO de rajada forte que ocasionalmente vaza ate
    // o chao, proporcional a intensidade real da rajada acontecendo em
    // altura (ver velocityAtHeight) - nao um piso constante, que soaria
    // artificial/previsivel.
    //
    // Frequencia do evento de rajada-no-chao: bem mais raro que os outros
    // canais (periodo de minutos, nao segundos) - de proposito, para nao
    // virar "mais uma rajada" perceptivel.
    float groundGustEventFrequencyHz = 0.0009f;
    // Expoente aplicado ao ruido do evento (ver velocityAtHeight): quanto
    // maior, mais tempo o valor passa perto de zero e mais breve/pontudo fica
    // o pico quando acontece - e isso que da a sensacao de evento raro e
    // aleatorio em vez de uma onda lenta e previsivel.
    float groundGustEventSharpness = 10.0f;
    // Fracao maxima (do vento de referencia) que o resquicio pode alcancar
    // QUANDO o evento raro coincide com uma rajada forte em altura. 0.65 e
    // deliberadamente alto (bem acima de uma brisa de fundo) - o pedido era
    // um sopro de verdade ocasional, nao uma brisa constante de baixo volume.
    float groundGustResidueMaxFraction = 0.65f;

    // --- Abrigo do vento (audio) por geometria ---
    // Distancia (m) do raycast a barlavento usado para saber se ha uma
    // parede/obstaculo estatico entre o ouvinte e de onde o vento vem (ver
    // Engine/Physics/WindShelter3D.hpp). Deliberadamente um campo proprio, e
    // nao um reaproveitamento de
    // PhysicsSceneSettings3D::windShelterDistanceMeters (que abriga o
    // arrasto fisico dos props): PhysicsScene3D nao expoe essa struct
    // publicamente, e "quao perto de uma parede um prop se sente abrigado"
    // e "quao grande precisa ser um comodo pra abafar o som" sao tunaveis
    // conceitualmente diferentes que nao deveriam ficar presos ao mesmo
    // slider. Mesmo valor padrao (4.0f) para comportamento dia-um
    // consistente entre os dois sistemas, mesmo sendo tunaveis
    // independentemente.
    float audioShelterDistanceMeters = 4.0f;
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
