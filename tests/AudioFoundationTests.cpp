#include "Engine/Audio/AcousticEnvironmentSystem3D.hpp"
#include "Engine/Audio/AcousticOcclusion3D.hpp"
#include "Engine/Audio/AudioDevice3D.hpp"
#include "Engine/Audio/BiquadFilter.hpp"
#include "Engine/Audio/Envelope.hpp"
#include "Engine/Audio/ImpactAcoustics.hpp"
#include "Engine/Audio/ProceduralNoise.hpp"
#include "Engine/Audio/Waveform.hpp"
#include "Engine/Audio/WindSound.hpp"
#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>

namespace {

using namespace MatterEngine;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// Forca o OpenAL Soft a usar o backend "null" (sem hardware de audio real).
// HRTF e o mixer inteiro rodam independente do backend de saida, entao os
// testes abaixo continuam validando comportamento real, nao um placebo -
// isso e o que permite rodar esta suite em CI/headless.
void selectNullAudioBackend() {
#if defined(_MSC_VER)
    _putenv_s("ALSOFT_DRIVERS", "null");
#else
    setenv("ALSOFT_DRIVERS", "null", 1);
#endif
}

ContactImpactEvent3D makeWoodImpact(float approachSpeed, float energyJoules) {
    ContactImpactEvent3D impact;
    impact.bodyIdA = 1;
    impact.bodyIdB = 2;
    impact.materialA = "wood";
    impact.materialB = "wood";
    impact.approachSpeedMetersPerSecond = approachSpeed;
    impact.transferredEnergyJoules = energyJoules;
    impact.effectiveMassKg = 2.0f;
    impact.massA = 2.0f;
    impact.massB = 2.0f;
    impact.characteristicSizeA = 0.5f;
    impact.characteristicSizeB = 0.5f;
    impact.staticA = false;
    impact.staticB = false;
    return impact;
}

void testRestingContactsStaySilent() {
    // Contatos de acomodacao (baixa velocidade de aproximacao) nao podem
    // gerar som, mesmo carregando alguma energia acumulada — e a mesma
    // porta de velocidade minima documentada em ImpactAcoustics.cpp.
    MaterialLibrary materials;
    ImpactAcousticResolver resolver;
    const ContactImpactEvent3D resting = makeWoodImpact(0.05f, 4.0f);
    resolver.resolve(std::span<const ContactImpactEvent3D>(&resting, 1),
        materials, 1.0f / 120.0f);
    require(resolver.commands().empty(),
        "Contato de repouso gerou som indevidamente");
}

void testRealImpactProducesCommand() {
    MaterialLibrary materials;
    ImpactAcousticResolver resolver;
    const ContactImpactEvent3D hit = makeWoodImpact(3.5f, 6.0f);
    resolver.resolve(std::span<const ContactImpactEvent3D>(&hit, 1),
        materials, 1.0f / 120.0f);

    // Os dois corpos sao dinamicos e identicos: ambos devem soar.
    require(resolver.commands().size() == 2,
        "Impacto real entre dois corpos dinamicos nao emitiu dos dois lados");
    for (const ImpactSoundCommand3D& command : resolver.commands()) {
        require(command.materialId == "wood",
            "Comando de impacto perdeu o material de origem");
        require(command.soundSet == "material.wood.impact",
            "Comando de impacto usou soundSet fora do catalogo esperado");
        require(command.volume > 0.0f && command.volume <= 1.0f,
            "Volume do comando fora do intervalo valido");
        require(command.pitch >= 0.45f && command.pitch <= 2.25f,
            "Pitch do comando fora do intervalo clampado por design");
    }
}

void testCooldownSuppressesImmediateRepeat() {
    // Sem o cooldown por par de corpos, uma vibracao residual entre dois
    // passos fixos consecutivos spammaria dezenas de comandos por segundo.
    MaterialLibrary materials;
    ImpactAcousticResolver resolver;
    const ContactImpactEvent3D hit = makeWoodImpact(3.5f, 6.0f);
    resolver.resolve(std::span<const ContactImpactEvent3D>(&hit, 1),
        materials, 1.0f / 120.0f);
    require(!resolver.commands().empty(),
        "Impacto inicial nao produziu comando para testar o cooldown");
    resolver.resolve(std::span<const ContactImpactEvent3D>(&hit, 1),
        materials, 1.0f / 120.0f);
    require(resolver.commands().empty(),
        "Cooldown nao suprimiu repeticao imediata do mesmo par de corpos");
}

PhysicsBodyHandle3D createStaticWallBox(PhysicsScene3D& scene, Vec3 position,
    Vec3 halfExtents) {
    PhysicsShape3D shape;
    shape.type = PhysicsShapeType3D::Box;
    shape.halfExtents = halfExtents;
    shape.materialId = "concrete";
    PhysicsBodyDefinition3D body;
    body.motionType = PhysicsMotionType3D::Static;
    body.position = position;
    body.materialId = "concrete";
    return scene.createBody(body, std::span<const PhysicsShape3D>(&shape, 1));
}

void testOcclusionSamplingAroundWall() {
    MaterialLibrary materials;
    PhysicsEngine3D engine;
    auto scene = engine.createScene({}, materials);
    // Parede fina cobrindo y em [-2,2] e z em [0,3].
    createStaticWallBox(*scene, { 0.0f, 0.0f, 1.5f },
        { 0.1f, 2.0f, 1.5f });
    const AcousticOcclusionSettings3D settings;

    // Atras da parede: ouvinte e fonte dead-center, a amostra central e
    // todas as amostras em cruz atravessam a parede.
    const float blockedFactor = computeOcclusionFactor3D(*scene,
        { -3.0f, 0.0f, 1.5f }, { 3.0f, 0.0f, 1.5f }, settings);
    require(blockedFactor > 0.99f,
        "Linha totalmente bloqueada nao produziu oclusao proxima de 1");

    // Longe da parede em Y: nenhuma amostra encontra geometria.
    const float freeFactor = computeOcclusionFactor3D(*scene,
        { -3.0f, 10.0f, 1.5f }, { 3.0f, 10.0f, 1.5f }, settings);
    require(freeFactor < 0.01f,
        "Linha totalmente livre produziu oclusao indevida");

    // Na borda da parede (y=1.85, raio de amostragem 0.35 cobre
    // y em [1.5, 2.2], que cruza o limite da parede em y=2): algumas
    // amostras atravessam a parede, outras passam ao largo dela.
    const float edgeFactor = computeOcclusionFactor3D(*scene,
        { -3.0f, 1.85f, 1.5f }, { 3.0f, 1.85f, 1.5f }, settings);
    require(edgeFactor > 0.01f && edgeFactor < 0.99f,
        "Amostragem na borda da parede nao produziu valor continuo entre "
        "livre e bloqueado");
}

void testWaveform() {
    // Seno: valores conhecidos em fases marcantes do ciclo.
    require(std::abs(oscillatorSample(Waveform::Sine, 0.0f)) < 1e-5f,
        "Seno na fase 0 deveria ser 0");
    require(std::abs(oscillatorSample(Waveform::Sine, 0.25f) - 1.0f) < 1e-4f,
        "Seno na fase 0.25 deveria ser 1 (pico)");
    require(std::abs(oscillatorSample(Waveform::Sine, 0.75f) - (-1.0f))
            < 1e-4f,
        "Seno na fase 0.75 deveria ser -1 (vale)");
    // Fase fora de [0,1) precisa ser normalizada de volta pro mesmo
    // resultado (permite acumular fase livremente sem envolver a mao).
    require(std::abs(oscillatorSample(Waveform::Sine, 1.25f)
            - oscillatorSample(Waveform::Sine, 0.25f)) < 1e-4f,
        "Fase fora de [0,1) deveria normalizar pro mesmo resultado");

    // Quadrada: alterna entre +1 e -1, sem valores intermediarios.
    require(oscillatorSample(Waveform::Square, 0.1f) == 1.0f,
        "Onda quadrada antes de 0.5 deveria ser exatamente 1");
    require(oscillatorSample(Waveform::Square, 0.6f) == -1.0f,
        "Onda quadrada depois de 0.5 deveria ser exatamente -1");

    // Triangular e dente-de-serra: limites e ponto medio conhecidos.
    require(std::abs(oscillatorSample(Waveform::Triangle, 0.0f)
            - (-1.0f)) < 1e-5f,
        "Triangular na fase 0 deveria comecar em -1");
    require(std::abs(oscillatorSample(Waveform::Triangle, 0.5f) - 1.0f)
            < 1e-5f,
        "Triangular na fase 0.5 deveria atingir o pico em 1");
    require(std::abs(oscillatorSample(Waveform::Sawtooth, 0.0f)
            - (-1.0f)) < 1e-5f,
        "Dente-de-serra na fase 0 deveria comecar em -1");
    require(std::abs(oscillatorSample(Waveform::Sawtooth, 0.999f) - 1.0f)
            < 1e-2f,
        "Dente-de-serra perto do fim do ciclo deveria estar perto de 1");
}

void testBiquadFilter() {
    // Um passa-baixa precisa atenuar uma frequencia bem acima do corte
    // muito mais do que uma frequencia bem abaixo dele - a verificacao
    // funcional basica de que o filtro realmente filtra, nao so processa.
    constexpr float SampleRate = 48000.0f;
    constexpr float CutoffHz = 500.0f;
    const BiquadCoefficients coefficients = biquadCoefficients(
        BiquadFilterType::LowPass, SampleRate, CutoffHz, 0.707f);

    auto measurePeakAmplitude = [&](float toneHz) {
        BiquadState state;
        float peak = 0.0f;
        // Descarta o primeiro trecho (regime transitorio do filtro) e so
        // mede a amplitude em regime permanente.
        constexpr int TotalSamples = 4800;
        constexpr int WarmupSamples = 2400;
        for (int index = 0; index < TotalSamples; ++index) {
            const float t = static_cast<float>(index) / SampleRate;
            const float input = oscillatorSample(Waveform::Sine, t * toneHz);
            const float output = state.process(input, coefficients);
            if (index >= WarmupSamples) {
                peak = std::max(peak, std::abs(output));
            }
        }
        return peak;
    };

    const float lowToneAmplitude = measurePeakAmplitude(80.0f);
    const float highToneAmplitude = measurePeakAmplitude(8000.0f);
    require(lowToneAmplitude > 0.7f,
        "Passa-baixa deveria deixar passar um tom bem abaixo do corte "
        "quase sem atenuar");
    require(highToneAmplitude < 0.1f,
        "Passa-baixa deveria atenuar fortemente um tom bem acima do corte");
    require(lowToneAmplitude > highToneAmplitude,
        "Passa-baixa deveria atenuar agudos mais que graves");
}

void testEnvelopeAdsr() {
    EnvelopeAdsr envelope;
    envelope.attackSeconds = 0.1f;
    envelope.decaySeconds = 0.1f;
    envelope.sustainLevel = 0.6f;
    envelope.releaseSeconds = 0.2f;
    constexpr float TotalDuration = 1.0f;

    require(envelope.sample(0.0f, TotalDuration) < 1e-5f,
        "Envelope deveria comecar em 0 (inicio do ataque)");
    require(std::abs(envelope.sample(0.1f, TotalDuration) - 1.0f) < 1e-4f,
        "Envelope deveria atingir o pico ao final do ataque");
    require(std::abs(envelope.sample(0.2f, TotalDuration)
            - envelope.sustainLevel) < 1e-4f,
        "Envelope deveria atingir o sustain ao final do decaimento");
    require(std::abs(envelope.sample(0.5f, TotalDuration)
            - envelope.sustainLevel) < 1e-5f,
        "Envelope deveria permanecer no sustain entre decay e release");
    require(envelope.sample(TotalDuration, TotalDuration) < 1e-4f,
        "Envelope deveria terminar em (perto de) 0 ao final do release");

    // Duracao mais curta que attack+decay: o release (que sempre comeca a
    // releaseSeconds do fim) precisa ter prioridade, terminando suave em
    // vez de cortar abrupto.
    require(envelope.sample(0.05f, 0.05f) < 1e-4f,
        "Som mais curto que o ataque deveria terminar em 0, nao cortar "
        "abrupto");
}

void testWindSoundGeneration() {
    // Deterministico, mesma garantia de generateWindNoiseSamples.
    const auto samplesA = generateWindSoundSamples(0.5f, 48000, 42u);
    const auto samplesB = generateWindSoundSamples(0.5f, 48000, 42u);
    require(samplesA == samplesB,
        "generateWindSoundSamples deveria ser deterministico");
    require(samplesA.size() == 24000,
        "Contagem de amostras nao bateu com duracao * taxa de amostragem");

    // O filtro precisa realmente mudar o sinal - moldado nao pode ser
    // identico ao ruido bruto de entrada.
    const auto rawNoise = generateWindNoiseSamples(0.5f, 48000, 42u);
    require(samplesA != rawNoise,
        "Ruido moldado pelo filtro nao deveria ser identico ao ruido bruto");

    // Nao pode ser silencio constante nem saturar em amplitude maxima.
    bool sawNonZero = false;
    bool allClipped = true;
    for (std::int16_t sample : samplesA) {
        if (sample != 0) sawNonZero = true;
        if (std::abs(static_cast<int>(sample)) < 32000) allClipped = false;
    }
    require(sawNonZero, "Vento gerado ficou em silencio constante");
    require(!allClipped, "Vento gerado saturou em amplitude maxima constante");

    require(generateWindSoundSamples(0.0f, 48000, 1u).empty(),
        "Duracao zero deveria produzir vetor vazio");
}

void testProceduralNoiseGeneration() {
    // Deterministico: a mesma (duracao, taxa, semente) sempre produz as
    // mesmas amostras - e o que permite o vento procedural nunca precisar
    // versionar um arquivo de audio (ver ProceduralNoise.hpp).
    const auto samplesA = generateWindNoiseSamples(0.5f, 48000, 12345u);
    const auto samplesB = generateWindNoiseSamples(0.5f, 48000, 12345u);
    require(samplesA == samplesB,
        "generateWindNoiseSamples deveria ser deterministico para a "
        "mesma semente");

    // Sementes diferentes produzem ruido diferente, nao um padrao fixo
    // reaproveitado.
    const auto samplesC = generateWindNoiseSamples(0.5f, 48000, 99999u);
    require(samplesA != samplesC,
        "Sementes diferentes deveriam produzir ruido diferente");

    // Contagem de amostras bate com duracao * taxa de amostragem.
    require(samplesA.size() == 24000,
        "Contagem de amostras nao bateu com duracao * taxa de amostragem");

    // Duracao invalida (zero ou negativa) produz vetor vazio, sem travar.
    require(generateWindNoiseSamples(0.0f, 48000, 1u).empty(),
        "Duracao zero deveria produzir vetor vazio");
    require(generateWindNoiseSamples(-1.0f, 48000, 1u).empty(),
        "Duracao negativa deveria produzir vetor vazio");

    // O resultado nao pode ser nem silencio constante nem saturacao
    // constante - sinal de que o filtro de Kellet e a normalizacao estao
    // de fato moldando o ruido, nao so preenchendo um valor fixo.
    bool sawNonZero = false;
    bool allClipped = true;
    for (std::int16_t sample : samplesA) {
        if (sample != 0) sawNonZero = true;
        if (std::abs(static_cast<int>(sample)) < 32000) allClipped = false;
    }
    require(sawNonZero, "Ruido gerado ficou em silencio constante");
    require(!allClipped, "Ruido gerado saturou em amplitude maxima constante");
}

void testLoopingVoiceLifecycle() {
    AudioDevice3D device;
    AudioDevice3D::Settings settings;
    // Pool pequeno de proposito - forca os sons de um tiro so a competirem
    // entre si por voz, pra provar que a voz em loop nunca entra nessa
    // disputa (ver o comentario de protecao em findFreeVoice()).
    settings.maxSources = 4;
    require(device.initialize(settings), "AudioDevice3D nao inicializou (backend null)");

    const auto noise = generateWindNoiseSamples(0.2f, 48000, 7u);
    const int buffer = device.loadProceduralBuffer(noise);
    require(buffer >= 0, "Falha ao carregar buffer procedural de ruido");
    require(device.bufferDurationSeconds(buffer) > 0.0f,
        "Buffer procedural carregado com duracao zero");

    const int loopHandle = device.playLooping(buffer, { 0.0f, 0.0f, 0.0f },
        0.4f, 1.0f, /*listenerRelative=*/true);
    require(loopHandle >= 0, "playLooping deveria retornar um handle valido");

    // Estoura o pool inteiro (so 4 vozes, 1 ja usada pelo loop) com sons de
    // um tiro so - se a protecao contra roubo falhasse, o loop seria uma
    // das vitimas candidatas aqui.
    for (int i = 0; i < 6; ++i) {
        device.play(buffer, { 0.0f, 0.0f, 0.0f }, 1.0f, 1.0f);
    }
    for (int step = 0; step < 3; ++step) {
        device.update(0.05f);
    }

    // Moduladores de loop nao podem travar nem gerar erro do AL sob o
    // backend null.
    device.setLoopingVolume(loopHandle, 0.7f);
    device.setLoopingPitch(loopHandle, 1.1f);
    device.setLoopingMuffle(loopHandle, 0.3f);

    device.stopLooping(loopHandle);
    // Depois de parada, a mesma voz precisa poder ser reciclada por um som
    // comum sem herdar AL_LOOPING nem AL_SOURCE_RELATIVE dela.
    device.play(buffer, { 2.0f, 0.0f, 0.0f }, 1.0f, 1.0f);

    device.shutdown();
}

void testMuffleDoesNotLeakBetweenRecycledVoices() {
    AudioDevice3D device;
    require(device.initialize(), "AudioDevice3D nao inicializou (backend null)");

    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/audio/materials/wood/impact/wood.wav";
    const int buffer = device.loadBuffer(path);
    require(buffer >= 0,
        "Falha ao carregar WAV para o teste de reciclagem de voz");

    // Um som curto e bem abafado.
    constexpr float ShortDurationSeconds = 0.05f;
    device.playTimed(buffer, { 0.0f, 0.0f, 0.0f }, ShortDurationSeconds,
        0.01f, 1.0f, 1.0f, 0.9f);
    require(device.debugLastVoiceHasActiveFilter(),
        "Som abafado nao registrou o filtro esperado");

    // Avanca alem da duracao para a voz ser reciclada.
    for (int step = 0; step < 5; ++step) {
        device.update(ShortDurationSeconds);
    }

    // Um som limpo na mesma voz (a unica ociosa neste dispositivo recem-
    // criado) nao pode herdar o abafamento do som anterior sem relacao.
    device.play(buffer, { 1.0f, 0.0f, 0.0f }, 1.0f, 1.0f, 0.0f);
    require(!device.debugLastVoiceHasActiveFilter(),
        "Voz reciclada herdou o abafamento de um som anterior sem relacao");

    device.shutdown();
}

void testAcousticEnvironmentBlend() {
    AcousticEnvironmentSystem3D environment;

    // Fora de qualquer zona: nenhum slot pode ficar ativo nem carregar
    // ganho - e o bug de maior visibilidade possivel (reverb vazando em
    // tudo), entao merece verificacao dedicada mesmo sem nenhuma zona
    // configurada ainda.
    const AcousticEnvironmentBlend3D emptyBlend =
        environment.evaluate({ 0.0f, 0.0f, 0.0f });
    for (const AcousticEnvironmentBlend3D::Slot& slot : emptyBlend.slots) {
        require(!slot.active, "Slot ativou sem nenhuma zona configurada");
        require(slot.wetSendGain <= 0.0f,
            "Slot com ganho positivo sem nenhuma zona configurada");
    }

    AcousticZoneDefinition3D cave;
    cave.name = "cave";
    cave.center = { 0.0f, 0.0f, 0.0f };
    cave.radiusMeters = 5.0f;
    cave.blendDistanceMeters = 5.0f;
    cave.preset = AcousticReverbPreset3D::Cave;
    cave.wetSendGain = 1.0f;
    environment.setZones({ cave });

    // Fora do alcance total (raio + blend): ainda silencio.
    const AcousticEnvironmentBlend3D farBlend =
        environment.evaluate({ 20.0f, 0.0f, 0.0f });
    require(!farBlend.slots[0].active,
        "Zona ativou fora do seu alcance total (raio + blend_distance)");

    // Dentro do raio central: peso maximo.
    const AcousticEnvironmentBlend3D insideBlend =
        environment.evaluate({ 2.0f, 0.0f, 0.0f });
    require(insideBlend.slots[0].active, "Zona nao ativou dentro do raio");
    require(insideBlend.slots[0].preset == AcousticReverbPreset3D::Cave,
        "Preset incorreto no slot ativo");
    require(insideBlend.slots[0].wetSendGain > 0.99f,
        "Ganho dentro do raio central deveria ser proximo do maximo");

    // Dentro da faixa de transicao: o peso cai conforme a distancia
    // aumenta (comportamento continuo, nao liga/desliga).
    const AcousticEnvironmentBlend3D nearEdgeBlend =
        environment.evaluate({ 7.0f, 0.0f, 0.0f });
    const AcousticEnvironmentBlend3D fartherBlend =
        environment.evaluate({ 9.0f, 0.0f, 0.0f });
    require(nearEdgeBlend.slots[0].active && fartherBlend.slots[0].active,
        "Faixa de transicao nao produziu slots ativos");
    require(nearEdgeBlend.slots[0].wetSendGain
            > fartherBlend.slots[0].wetSendGain,
        "Ganho nao caiu monotonicamente com a distancia na faixa de "
        "transicao");

    // Duas zonas sobrepostas, ambas com peso maximo: a soma final nunca
    // pode ultrapassar 1 (senao o reverb ficaria mais forte que qualquer
    // zona individual jamais autorou).
    AcousticZoneDefinition3D hallway;
    hallway.name = "hallway";
    hallway.center = { 0.0f, 0.0f, 0.0f };
    hallway.radiusMeters = 5.0f;
    hallway.blendDistanceMeters = 5.0f;
    hallway.preset = AcousticReverbPreset3D::Hallway;
    hallway.wetSendGain = 1.0f;
    environment.setZones({ cave, hallway });
    const AcousticEnvironmentBlend3D overlapBlend =
        environment.evaluate({ 0.0f, 0.0f, 0.0f });
    require(overlapBlend.slots[0].active && overlapBlend.slots[1].active,
        "Duas zonas sobrepostas nao ocuparam os 2 slots disponiveis");
    const float overlapTotal = overlapBlend.slots[0].wetSendGain
        + overlapBlend.slots[1].wetSendGain;
    require(overlapTotal <= 1.001f,
        "Soma do ganho de zonas sobrepostas ultrapassou 1");
}

void testAudioDeviceHrtfAndPlayback() {
    AudioDevice3D device;
    AudioDevice3D::Settings settings;
    settings.hrtfEnabled = true;
    require(device.initialize(settings),
        "AudioDevice3D nao inicializou sob o backend null");
    require(device.hrtfActive(),
        "HRTF nao ativou (dataset embutido deveria funcionar sem hardware)");

    const std::string path = std::string(MATTERENGINE_TEST_ASSETS_DIR)
        + "/audio/materials/wood/impact/wood.wav";
    const int buffer = device.loadBuffer(path);
    require(buffer >= 0, "Falha ao carregar um WAV real de impacto");
    require(device.bufferDurationSeconds(buffer) > 0.0f,
        "Buffer carregado com duracao zero");

    // ImpactAcousticResolver produz pitch em [0.45, 2.25]; os dois extremos
    // nao podem travar nem quebrar o dispositivo.
    device.play(buffer, { 0.0f, 0.0f, 0.0f }, 1.0f, 0.45f);
    device.play(buffer, { 3.0f, 0.0f, 0.0f }, 0.6f, 2.25f);
    device.playTimed(buffer, { -2.0f, 1.0f, 0.5f }, 0.2f, 0.05f, 0.8f, 1.0f);

    // Aplicar uma mistura de ambiente (com um slot ativo e outro nao) nao
    // pode travar nem gerar erro do AL, mesmo sob o backend null.
    AcousticEnvironmentBlend3D blend;
    blend.slots[0].active = true;
    blend.slots[0].preset = AcousticReverbPreset3D::Cave;
    blend.slots[0].wetSendGain = 0.7f;
    device.setEnvironment(blend);

    for (int step = 0; step < 30; ++step) {
        device.setListenerPose({ { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } });
        device.update(1.0f / 120.0f);
    }

    // Voltar para "fora de qualquer zona" tambem nao pode travar.
    device.setEnvironment(AcousticEnvironmentBlend3D {});

    device.shutdown();
    require(!device.hrtfActive(),
        "hrtfActive() deveria refletir o dispositivo desligado");
}

} // namespace

int main() {
    try {
        selectNullAudioBackend();
        testRestingContactsStaySilent();
        testRealImpactProducesCommand();
        testCooldownSuppressesImmediateRepeat();
        testOcclusionSamplingAroundWall();
        testAcousticEnvironmentBlend();
        testAudioDeviceHrtfAndPlayback();
        testMuffleDoesNotLeakBetweenRecycledVoices();
        testWaveform();
        testBiquadFilter();
        testEnvelopeAdsr();
        testProceduralNoiseGeneration();
        testWindSoundGeneration();
        testLoopingVoiceLifecycle();
        std::cout << "MatterEngine audio foundation tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MatterEngine audio test failure: "
            << error.what() << '\n';
        return 1;
    }
}
