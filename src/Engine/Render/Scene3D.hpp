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
    RHI::TextureHandle albedoTexture;
    RHI::TextureHandle metallicRoughnessTexture;
    // Fatores multiplicam o mapa metallicRoughness por pixel, como exige o
    // contrato glTF. Sem mapa, uma textura branca neutra preserva os fatores.
    float metallic = 0.0f;
    float roughness = 1.0f;
    bool selected = false;
    bool outlineGlow = false;
    bool visibleInCamera = true;
    bool castsShadow = true;
    // Um bit por Cascaded Shadow Map. O Workbench calcula em quais volumes
    // o objeto realmente entra; o backend evita renderiza-lo quatro vezes
    // quando ele so pode afetar uma cascata. Geometria sem culling explicito
    // (como o mapa estatico) conserva o default de todas as cascatas.
    std::uint8_t shadowCascadeMask = 0x0Fu;
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
// lista) contra o conjunto de cores/intensidades desta engine (ambientLight
// ~0.3-0.4, ceu ~0.6-0.8, sol ~1.0), que foram ajustadas a olho para um
// pipeline sem tonemap e sem correcao de gama (a Fase 2 introduziu os dois).
struct ToneMappingSettings3D {
    float exposure = 0.470f;
    float brightness = 0.007f;
    float contrast = 1.233f;
    float saturation = 1.341f;
};

// Neblina atmosferica por distancia (ver scene3d_mesh.frag), so aplicada
// quando showSky=true. density controla a taxa de queda exponencial por
// metro de distancia da camera; heightFalloff faz o chao a distancia ficar
// mais coberto que objetos no ar, multiplicando a distancia antes de somar
// a altura no mundo (mais alto = precisa de mais distancia pra enevoar
// igual); maxOpacity limita o quanto da cor real da superficie pode ser
// substituida pela cor da neblina, mesmo a distancias enormes (1.0 deixaria
// o horizonte inteiro virar uma parede solida da cor da neblina).
struct FogSettings3D {
    float density = 0.000f;
    float heightFalloff = 0.0223f;
    float maxOpacity = 0.720f;
    Vec3 color { 0.570f, 0.700f, 0.820f };
};

struct Scene3DFrame {
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
    float ambientLight = 0.30f;
    float skyTime = 0.0f;
    float cloudCoverage = 0.46f;
    // Deslocamento acumulado do vento nas nuvens, em unidades de ruido do
    // shader do ceu (nao metros - ver WindSystem/WorkbenchApp para a
    // conversao). E um deslocamento ja integrado ao longo do tempo, nao a
    // velocidade instantanea do vento: como a velocidade do vento varia
    // (rajadas), multiplicar a velocidade atual pelo tempo total produziria
    // um salto toda vez que ela mudasse - integrar quadro a quadro (
    // += velocidade * deltaTime) e a unica forma correta de mover as nuvens
    // de forma continua com um vento que nao e constante. Default zero é
    // seguro (nuvens paradas) para cenas que nao alimentam vento algum,
    // como as pre-visualizacoes estaticas do Object Viewer.
    Vec2 cloudWindOffset;
    bool showSky = false;
    bool showShadows = true;
    ToneMappingSettings3D toneMapping;
    FogSettings3D fog;
};

} // namespace MatterEngine
