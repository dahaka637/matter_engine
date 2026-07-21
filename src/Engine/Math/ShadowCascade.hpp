#pragma once

#include "Engine/Math/Mat4.hpp"
#include "Engine/Math/Vec3.hpp"

#include <cstdint>
#include <vector>

namespace MatterEngine {

// Geometria minima da camera principal necessaria pra reconstruir os 8
// cantos de um sub-frustum (entre duas distancias) - deliberadamente nao
// depende de nenhuma classe de camera do Workbench (Engine/Math fica numa
// camada abaixo dele, ver Mat4/Frustum3D pelo mesmo motivo).
struct CameraFrustumParameters3D {
    Vec3 position;
    Vec3 forward { 1.0f, 0.0f, 0.0f };
    Vec3 up { 0.0f, 0.0f, 1.0f };
    float verticalFovRadians = 1.0f;
    float aspectRatio = 16.0f / 9.0f;
};

// Calcula as N distancias de fronteira entre cascatas (a distancia "far" de
// cada uma, da mais perto pra mais longe - a cascata i cobre o intervalo
// [i==0 ? nearPlane : splits[i-1], splits[i])), misturando um split
// logaritmico e um uniforme via lambda (0=so uniforme, 1=so logaritmico) -
// formula pratica padrao da industria (Zhang et al., "Parallel-Split Shadow
// Maps", tambem usada por Unreal/id Tech). Log sozinho deixa a cascata mais
// proxima grande demais (pouca prioridade pro que esta perto da camera, que
// e onde a nitidez mais importa); uniforme sozinho desperdica resolucao em
// cascatas distantes que cobrem pouca coisa relevante da cena - a mistura
// pega o meio termo. Retorna vetor vazio se os parametros forem invalidos
// (nearPlane<=0, farPlane<=nearPlane ou cascadeCount==0).
[[nodiscard]] std::vector<float> computeCascadeSplits(float nearPlane,
    float farPlane, std::uint32_t cascadeCount, float lambda);

// Resultado do ajuste de uma cascata: a matriz view-projection e o tamanho
// (em metros) de um texel do mapa de sombra NESSA cascata especificamente -
// cascatas mais proximas da camera cobrem uma area menor, entao seu texel
// e mais fino. Exposto pro chamador porque o shader precisa dele pra
// calcular um bias de normal-offset proporcional a resolucao real de
// amostragem (ver shadowVisibility em scene3d_mesh.frag) - um bias fixo em
// espaco de profundidade nao escala com o tamanho da cascata e acaba
// "comendo" a sombra de geometria fina (pernas de cadeira etc.).
struct FittedShadowCascade {
    Mat4 viewProjection;
    float texelWorldSizeMeters = 0.0f;
    // Extensao linear do volume da luz entre near/far. O PCSS converte a
    // diferenca de profundidade normalizada de volta para metros usando este
    // valor; sem isso a penumbra mudaria quando o fitting da cascata mudasse.
    float depthRangeMeters = 1.0f;
};

// Ajusta uma matriz view-projection ortografica que cobre exatamente o
// sub-frustum da camera principal entre cascadeNear e cascadeFar (uma
// "fatia" do frustum inteiro, ver computeCascadeSplits), vista a partir de
// uma luz direcional. lightDirection segue a MESMA convencao do resto do
// motor (LightRender3D::direction, sunDirection em LaboratoryScreen.cpp):
// aponta PRA luz (de um ponto da cena, a direcao em que se olharia pra ver
// o sol), nao a direcao que a luz viaja - a funcao inverte internamente
// pra posicionar a camera de sombra do lado certo (ver comentario no .cpp).
// O resultado ja sai alinhado ao texel do mapa de
// sombra (shadowMapResolution, em texels por lado - o mapa e sempre
// quadrado): sem isso, cada sub-pixel de movimento da camera desliza a
// projecao em relacao a textura por baixo, produzindo cintilacao visivel
// (o mesmo problema que o texel-snapping de hoje ja evita pro unico mapa
// antigo, aqui generalizado por cascata).
[[nodiscard]] FittedShadowCascade fitCascadeFrustumToCamera(
    const CameraFrustumParameters3D& camera, float cascadeNear,
    float cascadeFar, Vec3 lightDirection, float shadowMapResolution);

} // namespace MatterEngine
