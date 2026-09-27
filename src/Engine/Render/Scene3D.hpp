#pragma once

#include "Engine/Math/Mat4.hpp"
#include "Engine/Math/Vec2.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/RHI/RHIHandles.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace MatterEngine {

// Quantidade de cascatas do mapa de sombra direcional (ver
// Engine/Math/ShadowCascade.hpp) - 4 e o padrao comum em engines AAA
// (Unreal/id Tech), bom equilibrio entre nitidez perto da camera e custo de
// preencher/amostrar N mapas. Uma constante so, nao espalhada: o layout do
// UBO da GPU (VulkanDevice.cpp) e o array aqui embaixo precisam concordar
// sempre - assim como o valor espelhado nos shaders (sem include
// compartilhado entre C++ e GLSL neste projeto, ver comentario de
// SceneUniformGpu).
constexpr std::uint32_t ShadowCascadeCount = 4;
// A cascata mais próxima conserva resolução total; as distantes cobrem áreas
// progressivamente maiores e já são suavizadas por perspectiva/neblina.
// Quatro mapas de 2048² desperdiçavam fill-rate sem detalhe perceptível.
constexpr std::array<std::uint32_t, ShadowCascadeCount>
    ShadowCascadeMapSizes { 2048u, 1536u, 1024u, 512u };

// Backend-neutral submission consumed immediately by Renderer. The same
// scene can target an editor preview texture or the real full-screen 3D pass.
//
// A real triangle mesh uploaded once to GPU vertex/index buffers (see
// MeshData3D and Renderer::createBuffer). The fragment shader always
// samples albedoTexture (a 1x1 white default when the mesh has none) and
// multiplies it by vertex color, so a procedural, solid-color mesh and an
// imported, textured one share the exact same draw path.
struct MeshRender3D {
    RHI::BufferHandle vertexBuffer;
    RHI::BufferHandle indexBuffer;
    std::uint32_t indexCount = 0;
    // Proxy opcional para depth/shadow. Ele compartilha os mesmos vértices,
    // mas usa menos índices; vazio reutiliza a geometria principal.
    RHI::BufferHandle shadowIndexBuffer;
    std::uint32_t shadowIndexCount = 0;
    Vec3 position;
    Quaternion orientation;
    // Posicao/orientacao deste MESMO objeto no quadro anterior (nao da
    // camera - essa vem de Scene3DFrame::previousCameraViewProjection)
    // - usadas para calcular o vetor de movimento por pixel que o resolve de
    // TAA consome (ver assets/shaders/scene3d_mesh.frag e
    // taa_resolve.frag). Objetos parados (geometria estatica do mapa,
    // pre-visualizacoes com camera fixa) simplesmente deixam estes campos
    // no mesmo default de position/orientation acima - motion vector zero,
    // sem nada especial a fazer. Objetos que se movem (props, a arma da
    // physgun) precisam trazer o valor de verdade do quadro passado; ver
    // SpawnedPropInstance::previousPhysicsState e
    // WorkbenchApp::m_previousPhysGunWeapon* para quem rastreia isso.
    Vec3 previousPosition;
    Quaternion previousOrientation;
    float scale = 1.0f;
    // Multiplies the sampled albedo/vertex color per instance (see
    // scene3d_mesh.frag) - default of {1,1,1} changes nothing, so only
    // callers that actually want a per-instance variation (a spawned
    // ragdoll's random pale color, see SpawnedRagdollInstance::tintColor)
    // need to touch it.
    Vec3 tintColor { 1.0f, 1.0f, 1.0f };
    RHI::TextureHandle albedoTexture;
    RHI::TextureHandle metallicRoughnessTexture;
    // Fatores multiplicam o mapa metallicRoughness por pixel, como exige o
    // contrato glTF. Sem mapa, uma textura branca neutra preserva os fatores.
    float metallic = 0.0f;
    float roughness = 1.0f;
    // Espelhos planos recebem uma renderizacao da camera refletida. SSR
    // continua responsavel por metais comuns, mas nao consegue enxergar as
    // faces de objetos viradas para o espelho e ocultas da camera principal.
    bool planarReflection = false;
    bool selected = false;
    bool outlineGlow = false;
    // Props físicos podem conservar uma grade mais fina a médias/longas
    // distâncias no modo Matter Mosaic. O mapa e o viewmodel permanecem na
    // grade artística principal; esta flag apenas grava uma classe por
    // pixel no attachment já existente de motion vectors.
    bool pixelArtHighDetail = false;
    // Vegetacao/terreno poroso nao deve produzir o reflexo concentrado do
    // disco solar que faz plastico, verniz e metal parecerem brilhantes.
    // A flag preserva toda a iluminacao difusa e reduz somente o lobulo
    // especular no shader.
    bool matteSurface = false;
    bool visibleInCamera = true;
    bool castsShadow = true;
    // Um bit por Cascaded Shadow Map. O Workbench calcula em quais volumes
    // o objeto realmente entra; o backend evita renderiza-lo quatro vezes
    // quando ele so pode afetar uma cascata. Geometria sem culling explicito
    // (como o mapa estatico) conserva o default de todas as cascatas.
    std::uint8_t shadowCascadeMask = 0x0Fu;
    // World-space deformation matrices (current * inverse bind). Consumed
    // immediately, copied into fence-protected GPU storage by the backend.
    std::span<const Mat4> skinMatrices;
    std::span<const Mat4> previousSkinMatrices;
    bool flatShaded = false;
};

enum class LightType3D {
    Directional,
    Point,
    Spot
};

// Uma fonte de luz generica e backend-neutra (ver Scene3DFrame::lights). Os
// campos que nao se aplicam ao tipo escolhido sao ignorados por
// packSceneLights/accumulateLighting (ver SceneLightPacking.hpp e
// scene3d_mesh.frag) - ex.: range/coneOuterDegrees nao tem efeito numa luz
// Directional.
struct LightRender3D {
    LightType3D type = LightType3D::Directional;
    Vec3 position;
    // Direcao da luz (Directional/Spot) - tambem serve, por convencao, como a
    // "direcao do sol" usada pelo ceu procedural e pelo reflexo especular de
    // metais (ver scene3d_sky.frag/scene3d_mesh.frag), mesmo quando a luz em
    // lights[0] e do tipo Point (mesma convencao que o antigo campo unico
    // Scene3DFrame::lightDirection ja seguia, agora so realocada).
    Vec3 direction { -0.45f, -0.35f, 0.82f };
    Vec3 color { 1.0f, 1.0f, 1.0f };
    float intensity = 1.0f;
    float range = 0.0f;
    float coneOuterDegrees = 35.0f;
    bool castsShadow = false;
};

// Controles de correcao de cor pos-HDR, todos aplicados no passe de tonemap
// (ver VulkanDevice::renderScene3DInternal e tonemap.frag) - nenhum afeta a
// intensidade de nenhuma luz, so a apresentacao final da cena inteira.
// exposure multiplica a cor HDR *antes* da curva ACES (em espaco linear);
// brightness/contrast/saturation agem *depois* da curva e da codificacao
// sRGB (em espaco de tela, 0..1), do mesmo jeito que um controle de imagem
// de monitor ou editor de fotos - por isso contrast pivota em 0.5 (o cinza
// medio da tela), nao em 0.0.
//
// Defaults calibrados a olho (ver painel de debug do Laboratorio, aba
// "Gráficos" - o botao "Copiar valores calibrados" gera exatamente esta
// lista) contra o conjunto de cores/intensidades desta engine
// (environment.skyIrradiance ~0.3-0.4, ceu ~0.6-0.8, sol ~1.0), que foram
// ajustadas a olho para um
// pipeline sem tonemap e sem correcao de gama (a Fase 2 introduziu os dois).
struct ToneMappingSettings3D {
    float exposure = 0.470f;
    float brightness = 0.007f;
    float contrast = 1.233f;
    float saturation = 1.341f;
    // A exposição manual acima continua sendo a calibração artística-base.
    // Quando ativado, este multiplicador automático só compensa mudanças
    // grandes de luminância (entrar/sair de interiores, futuro dia/noite).
    // A medição e a adaptação temporal rodam num alvo 1x1, com custo fixo.
    bool automaticExposureEnabled = false;
    float automaticExposureMinimum = 0.75f;
    float automaticExposureMaximum = 2.50f;
    float automaticExposureMeteringKey = 0.35f;
    // Mudanças grandes recebem ainda um reforço proporcional no shader.
    // Estes valores-base rápidos evitam a sensação de a imagem chegar
    // atrasada quando a câmera alterna entre interior, chão e céu.
    float brightAdaptationSpeed = 7.00f;
    float darkAdaptationSpeed = 2.40f;
};

// Oclusão de ambiente em espaço de tela. O backend calcula em meia
// resolução e deixa o TAA estabilizar/reamostrar, portanto o custo fica
// limitado. maxDarkening impede que GTAO vire uma segunda sombra global:
// mesmo totalmente ocluído, o pixel conserva a fração restante da luz.
struct AmbientOcclusionSettings3D {
    bool enabled = false;
    float radiusMeters = 1.15f;
    float strength = 0.72f;
    float maxDarkening = 0.24f;
    float normalBias = 0.055f;
    // Escalabilidade real do kernel: direções * 2 lados * passos.
    std::uint32_t sampleDirectionCount = 6;
    std::uint32_t sampleStepCount = 2;
};

// Efeitos solares HDR de custo fixo. O backend reúne glare e ghosts de lente
// num passe a 1/8 da largura e altura (1/64 dos pixels). Não há bloom geral:
// só o Sol gera o efeito, e o flare consulta profundidade para desaparecer
// atrás de paredes/objetos.
struct OpticalEffectsSettings3D {
    bool enabled = true;
    float sunGlareStrength = 0.65f;
    float lensFlareStrength = 0.35f;
};

enum class SceneRenderMode3D {
    Standard,
    PixelArt
};

// Identidade visual do modo Matter Mosaic. O mundo continua sendo
// renderizado em HDR e em 3D; estes controles só entram no último passe,
// depois da iluminação e antes da apresentação no swapchain.
struct PixelArtSettings3D {
    // Escala do HDR/geometry pass. A UI continua sempre na resolução nativa;
    // 1.0 é o padrão e preserva toda a resolução real.
    float renderScale = 1.0f;
    // Altura virtual da grade artística. É independente de renderScale:
    // reduzir a escala melhora desempenho, enquanto esta grade decide o
    // tamanho aparente dos pixels desenhados.
    float pixelGridHeight = 540.0f;
    // Intensidades independentes. 1.0 representa o máximo artístico e é o
    // default pedido; zero conserva pixels individuais disponíveis.
    float worldPixelation = 1.0f;
    float physicalPropPixelation = 1.0f;
    // O cenário perde detalhe mais depressa que props físicos, preservando
    // leitura de bolas/objetos sem parecerem recortes HD de perto.
    float distanceLodStrength = 0.65f;
    float distanceLodStartMeters = 12.0f;
    float distanceLodEndMeters = 110.0f;
    // Mistura entre a imagem filmic normal e a paleta/iluminação em bandas.
    float flatteningStrength = 1.0f;
    float luminanceLevelCount = 12.0f;
    float chromaLevelCount = 18.0f;
    float ditherStrength = 0.025f;
    bool temporalAntiAliasingEnabled = false;
};

// Neblina atmosferica por distancia (ver scene3d_mesh.frag), so aplicada
// quando showSky=true. density controla a taxa de queda exponencial por
// metro de distancia da camera; heightFalloff faz o chao a distancia ficar
// mais coberto que objetos no ar, multiplicando a distancia antes de somar
// a altura no mundo (mais alto = precisa de mais distancia pra enevoar
// igual). A transição termina em endDistanceMeters, onde o cenário fica
// totalmente oculto pela cor atmosférica e o fragment shader pode ignorar
// toda a iluminação/material distante.
struct FogSettings3D {
    // Distancia (em metros) antes da qual NENHUMA neblina e aplicada -
    // sem isso, a curva exponencial de "density" comeca a esmaecer desde
    // distancia zero (visivel mesmo em objetos proximos com qualquer
    // density>0). Ver o uso em scene3d_mesh.frag/ocean_surface.frag:
    // subtrai esta distancia ANTES de aplicar a curva exponencial, entao
    // tudo dentro dela fica 100% limpo e a neblina só nasce a partir daqui.
    float startDistanceMeters = 394.0f;
    // Ponto em que a neblina se torna uma barreira visual totalmente opaca.
    float endDistanceMeters = 1250.0f;
    // Taxa de crescimento por metro APOS startDistanceMeters (nao desde a
    // camera) - ver comentario acima.
    float density = 0.0018f;
    float heightFalloff = 0.0223f;
    float maxOpacity = 1.0f;
    Vec3 color { 0.570f, 0.700f, 0.820f };
};

// Contrato ambiental contínuo consumido pelo renderer. O futuro sistema de
// horário/clima altera estes valores; nenhum passe precisa conhecer estados
// discretos como "dia", "noite" ou "chuva". skyIrradiance é a energia
// indireta vinda do hemisfério do céu. minimumIndirectVisibility impede
// preto matemático no preenchimento global barato.
// Os campos de precipitação/umidade já fazem parte do contrato para que a
// integração climática não exija quebrar a API de cena depois.
struct EnvironmentLightingState3D {
    float solarTimeHours = 12.0f;
    float skyAnimationTime = 0.0f;
    float skyIrradiance = 0.31f;
    float minimumIndirectVisibility = 0.18f;
    // Fração aproximada da energia solar direta que retorna como primeiro
    // rebote difuso em regiões sombreadas. Mantido discreto para não lavar
    // materiais enquanto usamos a aproximação global de baixo custo.
    float sunDiffuseBounce = 0.035f;
    float cloudCoverage = 0.48f;
    // Fração da luz solar direta que atravessa a camada de nuvens no ponto
    // ocupado pelo Sol. É separada de cloudCoverage porque uma camada
    // parcialmente nublada alterna entre Sol aberto e encoberto conforme o
    // vento move as nuvens, mesmo sem mudar a cobertura média.
    float cloudSunTransmittance = 1.0f;
    float precipitation = 0.0f;
    float surfaceWetness = 0.0f;
    Vec2 cloudWindOffset;
};

struct OceanRender3D {
    Vec2 center;
    float meanSeaLevelMeters = 0.0f;
    float halfExtentMeters = 1500.0f;
    RHI::BufferHandle vertexBuffer;
    RHI::BufferHandle indexBuffer;
    std::uint32_t indexCount = 0;
};

struct Scene3DFrame {
    SceneRenderMode3D renderMode = SceneRenderMode3D::Standard;
    PixelArtSettings3D pixelArt;
    // Sem jitter - usada pra cull em CPU (Frustum3D) e picking de UI em
    // espaco de tela (ver LaboratoryScreen.cpp, projectToScreen do feixe da
    // physgun). Jitterar essa deixaria o picking tremendo visivelmente.
    Mat4 cameraViewProjection;
    // A GPU sempre desenha com esta - identica a cameraViewProjection quando
    // a cena nao usa TAA (ex.: previews do Object Viewer/catalogo de props),
    // ou com um deslocamento sub-pixel em espaco NDC quando usa (ver
    // Mat4::perspectiveJittered, JitterSequence.hpp e o passe de resolve de
    // TAA em VulkanDevice). Nao ha default valido - todo chamador precisa
    // definir explicitamente (mesmo que so copiando cameraViewProjection).
    Mat4 cameraViewProjectionJittered;
    // VP SEM jitter do quadro anterior, usada para os vetores de movimento.
    // O historico resolvido vive na grade estavel de pixels; incluir a
    // diferenca entre os jitters atual e anterior na reprojecao faria essa
    // grade deslizar a cada amostra e produziria tremor. Cenas estaticas
    // podem repetir cameraViewProjection, gerando movimento zero.
    Mat4 previousCameraViewProjection;
    // Indice monotonico de quadro usado pra gerar o jitter (ver
    // WorkbenchApp - nao e o currentFrame do backend Vulkan, que so alterna
    // 0/1 pro duplo buffer). Cenas sem TAA de verdade podem deixar em 0.
    std::uint32_t taaFrameIndex = 0;
    // O TAA e opt-in. Previews e ferramentas que nao mantem historico entre
    // quadros devem deixa-lo desativado; assim o backend apenas copia a cor
    // atual para a etapa de tonemap, sem misturar imagens de cenas distintas.
    bool temporalAntiAliasingEnabled = false;
    // Invalida explicitamente o historico em cortes de camera, troca de tela
    // ou retomada de uma cena. Resize tambem invalida internamente no backend.
    bool resetTemporalHistory = false;
    // Cascaded Shadow Maps da luz que projeta sombra - por convencao,
    // sempre a de lights[0] (que deve ser Directional quando sombras estao
    // ligadas). Limitacao deliberada, ainda vale mesmo com N cascatas: o
    // backend so mantem cascatas pra UMA luz (ver
    // VulkanDevice::sceneShadowImages) - luzes em outros indices nunca
    // projetam sombra, mesmo com castsShadow=true. cascadeViewProjections[i]
    // e a matriz view-projection ajustada (ver
    // ShadowCascade::fitCascadeFrustumToCamera) da cascata i; cascadeSplits[i]
    // e a distancia (metros, espaco de camera) onde a cascata i termina -
    // ambos vem de computeCascadeSplits/fitCascadeFrustumToCamera, calculados
    // uma vez por quadro em LaboratoryScreen.cpp.
    std::array<Mat4, ShadowCascadeCount> cascadeViewProjections;
    std::array<float, ShadowCascadeCount> cascadeSplits {};
    // Tamanho (metros) de um texel do mapa de sombra em CADA cascata (ver
    // ShadowCascade::FittedShadowCascade::texelWorldSizeMeters) - usado no
    // shader pra um bias de normal-offset proporcional a resolucao real de
    // amostragem daquela cascata (ver shadowVisibility em
    // scene3d_mesh.frag). Um bias fixo em espaco de profundidade nao
    // escala com o tamanho de cada cascata e "come" a sombra de geometria
    // fina (pernas de cadeira etc.) nas cascatas mais distantes/largas.
    std::array<float, ShadowCascadeCount> cascadeTexelWorldSizes {};
    // Extensao near/far em metros de cada projecao ortografica. Necessaria
    // para o PCSS calcular distancia receptor-bloqueador em unidades fisicas.
    std::array<float, ShadowCascadeCount> cascadeDepthRanges {};
    Vec3 cameraPosition;
    std::span<const LightRender3D> lights;
    std::span<const MeshRender3D> meshes;
    EnvironmentLightingState3D environment;
    // Um plano refletor por cena mantém o custo previsível: um passe extra
    // mesmo que existam muitos metais, que continuam usando SSR.
    bool planarReflectionEnabled = false;
    Vec3 planarReflectionPoint;
    Vec3 planarReflectionNormal { 0.0f, 1.0f, 0.0f };
    bool showSky = false;
    bool showShadows = true;
    // Amostras para cada uma das duas fases do PCSS (busca de bloqueador e
    // filtro de penumbra). A resolução das quatro cascatas permanece 2048;
    // este controle reduz apenas o kernel por pixel.
    std::uint32_t shadowFilterSampleCount = 12;
    ToneMappingSettings3D toneMapping;
    AmbientOcclusionSettings3D ambientOcclusion;
    OpticalEffectsSettings3D opticalEffects;
    FogSettings3D fog;
    bool oceanEnabled = false;
    OceanRender3D ocean;
    float oceanSubmersion = 0.0f;
};

} // namespace MatterEngine
