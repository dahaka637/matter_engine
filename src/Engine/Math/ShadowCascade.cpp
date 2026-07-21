#include "Engine/Math/ShadowCascade.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace MatterEngine {

std::vector<float> computeCascadeSplits(float nearPlane, float farPlane,
    std::uint32_t cascadeCount, float lambda) {
    std::vector<float> splits;
    if (cascadeCount == 0 || nearPlane <= 0.0f || farPlane <= nearPlane) {
        return splits;
    }
    splits.reserve(cascadeCount);
    const float clampedLambda = std::clamp(lambda, 0.0f, 1.0f);
    const float ratio = farPlane / nearPlane;
    for (std::uint32_t index = 1; index <= cascadeCount; ++index) {
        const float fraction = static_cast<float>(index)
            / static_cast<float>(cascadeCount);
        const float logSplit = nearPlane * std::pow(ratio, fraction);
        const float uniformSplit = nearPlane
            + (farPlane - nearPlane) * fraction;
        splits.push_back(clampedLambda * logSplit
            + (1.0f - clampedLambda) * uniformSplit);
    }
    return splits;
}

FittedShadowCascade fitCascadeFrustumToCamera(const CameraFrustumParameters3D& camera,
        float cascadeNear, float cascadeFar, Vec3 lightDirection,
        float shadowMapResolution) {
    const Vec3 forward = camera.forward.normalized();
    const Vec3 right = cross(forward, camera.up).normalized();
    const Vec3 up = cross(right, forward).normalized();
    const float tanHalfFov = std::tan(camera.verticalFovRadians * 0.5f);

    // Os 8 cantos do sub-frustum da camera entre cascadeNear e cascadeFar -
    // a "fatia" do frustum inteiro que esta cascata precisa cobrir.
    const auto corner = [&](float distance, float rightSign, float upSign) {
        const float halfHeight = tanHalfFov * distance;
        const float halfWidth = halfHeight * camera.aspectRatio;
        return camera.position + forward * distance
            + right * (halfWidth * rightSign) + up * (halfHeight * upSign);
    };
    const std::array<Vec3, 8> corners {
        corner(cascadeNear, -1.0f, -1.0f), corner(cascadeNear, 1.0f, -1.0f),
        corner(cascadeNear, 1.0f, 1.0f), corner(cascadeNear, -1.0f, 1.0f),
        corner(cascadeFar, -1.0f, -1.0f), corner(cascadeFar, 1.0f, -1.0f),
        corner(cascadeFar, 1.0f, 1.0f), corner(cascadeFar, -1.0f, 1.0f),
    };

    Vec3 center {};
    for (const Vec3& point : corners) center += point;
    center = center / 8.0f;

    // lightDirection segue a convencao do resto do motor (LightRender3D::
    // direction, sunDirection em LaboratoryScreen.cpp): aponta PRA luz (de
    // um ponto da cena, a direcao em que se olharia pra ver o sol - e por
    // isso dot(normal, lightDirection) da positivo num chao iluminado por
    // cima). A CAMERA da sombra precisa do oposto disso: ela fica do lado
    // da luz e olha PRA BAIXO em direcao a cena, entao sua direcao de visao
    // e -lightDirection, nao lightDirection. Passar lightDirection sem
    // inverter aqui colocava a camera de sombra do lado ERRADO da cena
    // (embaixo olhando pra cima) - a causa raiz de uma sombra sem sentido
    // no mapa e de objetos proximos ficando fora do frustum ajustado.
    const Vec3 lightForward = (-lightDirection).normalized();
    // "Up" de mundo (mesma escolha de eixo do resto do motor), a nao ser
    // que a luz esteja quase alinhada com ele - caso degenerado onde
    // cross(lightForward, worldUp) perderia precisao/zeraria.
    Vec3 worldUp { 0.0f, 0.0f, 1.0f };
    if (std::abs(dot(lightForward, worldUp)) > 0.999f) {
        worldUp = { 0.0f, 1.0f, 0.0f };
    }
    const Vec3 lightRight = cross(lightForward, worldUp).normalized();
    const Vec3 lightUp = cross(lightRight, lightForward);

    // Uma esfera envolve a fatia inteira. Diferente de um AABB justo no
    // espaco da luz, seu raio nao muda quando a camera gira: a densidade de
    // texels da cascata permanece constante e nao produz "zoom" da sombra.
    float boundingRadius = 0.0f;
    for (const Vec3& point : corners) {
        boundingRadius = std::max(boundingRadius, (point - center).length());
    }

    // Quantizar o raio evita que ruido de ponto flutuante altere a escala da
    // projecao. Um texel adicional absorve o deslocamento maximo introduzido
    // pelo snapping do centro sem cortar os cantos da esfera.
    const float quantizedRadius = std::ceil(boundingRadius * 16.0f) / 16.0f;
    const float preliminaryTexel = (quantizedRadius * 2.0f)
        / std::max(1.0f, shadowMapResolution);
    const float halfSize = quantizedRadius + preliminaryTexel;
    const float texelWorldSize = (halfSize * 2.0f)
        / std::max(1.0f, shadowMapResolution);

    // Alinhamento ao texel precisa quantizar a coordenada ABSOLUTA do centro
    // na base da luz. A implementacao anterior projetava `point-center` e
    // quantizava apenas um offset relativo; mover a camera movia `center`
    // continuamente e anulava por completo o snapping.
    const float absoluteCenterX = dot(center, lightRight);
    const float absoluteCenterY = dot(center, lightUp);
    const float snappedCenterX = std::round(absoluteCenterX / texelWorldSize)
        * texelWorldSize;
    const float snappedCenterY = std::round(absoluteCenterY / texelWorldSize)
        * texelWorldSize;
    const Vec3 snappedWorldCenter = center
        + lightRight * (snappedCenterX - absoluteCenterX)
        + lightUp * (snappedCenterY - absoluteCenterY);

    float minimumDepth = std::numeric_limits<float>::max();
    float maximumDepth = std::numeric_limits<float>::lowest();
    for (const Vec3& point : corners) {
        const float depth = dot(point - snappedWorldCenter, lightForward);
        minimumDepth = std::min(minimumDepth, depth);
        maximumDepth = std::max(maximumDepth, depth);
    }

    // A fatia descreve RECEPTORES visiveis, mas um bloqueador pode estar fora
    // dela, na direcao da luz, e ainda projetar sombra para dentro. Extrudar
    // o volume nessa direcao evita sumicos quando o caster sai ligeiramente
    // do frustum da camera. D32F mantem precisao suficiente para esta folga.
    constexpr float DepthMarginMeters = 5.0f;
    const float sliceLength = std::max(0.0f, cascadeFar - cascadeNear);
    const float casterExtensionMeters = std::max(35.0f, sliceLength * 0.35f);
    minimumDepth -= casterExtensionMeters;
    const float pushbackDistance = -minimumDepth + DepthMarginMeters;
    const Vec3 eye = snappedWorldCenter - lightForward * pushbackDistance;
    const float nearPlane = DepthMarginMeters;
    const float depthRangeMeters = std::max(1.0f,
        maximumDepth - minimumDepth);
    const float farPlane = depthRangeMeters + DepthMarginMeters;

    const Mat4 view = Mat4::lookAt(eye, snappedWorldCenter, worldUp);
    const Mat4 projection = Mat4::orthographic(-halfSize, halfSize,
        -halfSize, halfSize, nearPlane, farPlane);
    return FittedShadowCascade {
        projection * view, texelWorldSize, depthRangeMeters
    };
}

} // namespace MatterEngine
