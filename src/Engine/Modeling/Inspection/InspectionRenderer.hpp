#pragma once

#include "Engine/Modeling/Analysis/Report.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/ShadingNormals.hpp"
#include "Engine/Modeling/Raster/Camera.hpp"
#include "Engine/Modeling/Raster/Image.hpp"
#include "Engine/Modeling/Skeleton/BodyRegions.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

enum class SurfaceMode : std::uint8_t {
    // So o esqueleto.
    Hidden,
    // Facetado com as cores dos materiais (o visual do jogo).
    Solid,
    // Facetado com as arestas por cima.
    SolidWire,
    // So as arestas (as de tras escondidas).
    Wire,
    // Transparente, mais forte nas bordas, com os ossos dentro.
    XRay,
    // Cor pela direcao da normal (avesso em magenta).
    Normals,
    // Uma cor por parte.
    Parts,
    // Peso de uma junta (calor).
    Weights,
    // Cinza neutro: so a forma.
    Clay,
};

[[nodiscard]] std::string_view surfaceModeId(SurfaceMode mode);
[[nodiscard]] std::string_view surfaceModeLabel(SurfaceMode mode);
[[nodiscard]] std::optional<SurfaceMode> parseSurfaceMode(std::string_view id);
[[nodiscard]] std::span<const SurfaceMode> allSurfaceModes();

struct InspectionStyle {
    SurfaceMode surface = SurfaceMode::Solid;
    bool skeleton = true;
    bool joints = true;
    bool jointLabels = false;
    bool grid = true;
    // O meio lado da grade no chao (m); 0 = pela distancia da camera (ate
    // 2,5 m). As cenas grandes (o laboratorio de objetos) pedem mais.
    float gridExtent = 0.0f;
    // O passo da grade (m); 0 = pela distancia da camera (1, 5 ou 10 cm). Um
    // chao grande pede passo largo (a linha forte fica a cada 5).
    float gridStep = 0.0f;
    bool gizmo = true;
    // A legenda dos eixos ao lado do gizmo ("X frente · Y esquerda · Z cima").
    bool gizmoLegend = true;
    bool scaleBar = true;
    bool hud = true;
    // O avesso (triangulo visto por tras) em magenta: normal invertida.
    bool backfaces = true;
    // Regiao em foco: os ossos dela destacados e os outros apagados.
    int focusRegion = -1;
    // Mostra so a regiao em foco (malha e ossos).
    bool isolate = false;
    // A junta do modo Weights.
    int weightJoint = -1;
    // Supersampling (antisserrilhado): 1 a 4.
    int supersample = 2;
    // A luz principal de frente (como nas referencias em render) em vez da de
    // estudio (de cima e de um lado).
    bool frontLight = false;
    // O sombreamento do modo Solido (e com arame): liso, com as faces que
    // dobram mais que "creaseAngle" graus marcadas (ShadingNormals), e o
    // quanto volta ao facetado ("facet": 0 liso, 1 as faces de verdade). So a
    // luz muda: a malha e o contorno sao os mesmos. A argila e os modos de
    // analise continuam mostrando as faces. O padrao (0,5 e 45 graus) e o do
    // jogador no jogo (GameCharacterExportSettings).
    float creaseAngle = 45.0f;
    float facet = 0.5f;
    std::string title;
    std::vector<std::string> notes;
};

// O que destacar por cima (achados do relatorio, cortes, medidas).
struct Highlight {
    std::vector<std::uint32_t> triangles;
    std::vector<std::pair<Vec3, Vec3>> segments;
    std::vector<Vec3> points;
    Color color = Palette::Error;
    std::string label;
    Vec3 labelAt;
    bool hasLabel = false;
};

[[nodiscard]] Highlight highlightFor(const Finding& finding);

// Um marcador no chao (o alvo do RL): o disco, o aro e um brilho que sobe das
// bordas e some no alto. O centro fica no chao. strength apaga o marcador
// inteiro (os proximos alvos de uma sequencia vem mais fracos).
struct InspectionMarker {
    Vec3 center;
    float radius = 0.5f;
    Color color = rgb(232, 44, 44);
    float glowHeight = 0.9f;
    float strength = 1.0f;
};

// Uma seta na horizontal (o movimento do RL): chapada, com a haste de width de
// largura e a ponta; ou, com lineWidthPixels > 0, so o traco (a haste e as duas
// pernas da ponta), fino como um desenho por cima do chao.
struct InspectionArrow {
    Vec3 from;
    Vec3 to;
    Color color = rgb(80, 230, 90);
    float width = 0.06f;
    float lineWidthPixels = 0.0f;
};

// Os alvos de uma sequencia (o de agora e os proximos, em ordem): o de agora
// cheio, os seguintes cada vez mais fracos e baixos, ligados por um traco fino
// de aro a aro com a ponta no que vem depois.
void appendTargetSequence(std::span<const Vec3> targets, float radius, std::vector<InspectionMarker>& markers,
    std::vector<InspectionArrow>& arrows);

// Um olho desenhado de frente para a camera (o rumo mandado olhar, no
// horizonte): as palpebras, a iris e a pupila, com size de largura (m).
struct InspectionEye {
    Vec3 position;
    float size = 1.2f;
    Color color = rgb(120, 200, 255);
};

struct InspectionScene {
    const Skeleton* skeleton = nullptr;
    // Nulo = pose de ligacao.
    const SkeletonPose* pose = nullptr;
    const Mesh* mesh = nullptr;
    // As posicoes da malha a desenhar (a pose ja aplicada); vazio = as da malha.
    std::span<const Vec3> meshPositions;
    const BodyRegions* regions = nullptr;
    std::span<const int> jointRegion;
    std::span<const Highlight> highlights;
    std::span<const InspectionMarker> markers;
    std::span<const InspectionArrow> arrows;
    std::span<const InspectionEye> eyes;
    // Os grupos do sombreamento liso da malha, se ja feitos (na quina do
    // estilo); nulo = feitos a cada imagem.
    const ShadingNormals* shading = nullptr;
    // Imagem ao fundo (a referencia, ja no enquadramento da camera).
    const Image* backdrop = nullptr;
    float backdropOpacity = 0.55f;
};

[[nodiscard]] Image renderInspection(const InspectionScene& scene, const InspectionStyle& style,
    const OrbitCamera& camera, int width, int height);

// A posicao de cada junta na pose da cena.
[[nodiscard]] std::vector<Vec3> sceneJointPositions(const InspectionScene& scene);
// A regiao de uma parte da malha (pela regiao declarada nela).
[[nodiscard]] int partRegion(const InspectionScene& scene, int part);
// A junta mais perto do pixel (ou -1), entre as visiveis.
[[nodiscard]] int pickJoint(const InspectionScene& scene, const InspectionStyle& style, const OrbitCamera& camera,
    int width, int height, float x, float y, float radiusPixels);

} // namespace MatterEngine::Modeling
