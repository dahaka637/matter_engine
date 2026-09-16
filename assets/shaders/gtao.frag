#version 450

// O passe roda em meia resolução depois do opaco. O UBO fornece a inversa
// estável da câmera; depth+normal vêm dos attachments em resolução cheia.
layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;
    mat4 cameraViewProjectionUnjittered;
    mat4 inverseCameraViewProjection;
    mat4 cascadeViewProjections[4];
    vec4 cascadeSplits;
    vec4 cascadeTexelWorldSizes;
    vec4 cascadeDepthRanges;
    mat4 previousCameraViewProjection;
    vec4 cameraPosition;
    vec4 settings;
    vec4 skySettings;
    vec4 fogSettings;
    vec4 fogColor;
    vec4 windOffset;
    vec4 environmentSettings;
} scene;

layout(set = 1, binding = 0) uniform sampler2D depthMap;
layout(set = 1, binding = 1) uniform sampler2D normalRoughnessMap;

layout(push_constant) uniform GtaoPushConstants {
    float radiusMeters;
    float strength;
    float maxDarkening;
    float normalBias;
    uint sampleDirectionCount;
    uint sampleStepCount;
    uvec2 reserved;
} pushConstants;

layout(location = 0) in vec2 texCoord;
// R = visibilidade AO; G = distância da superfície à câmera, usada pelo
// upsample bilateral no TAA para não vazar sombra através de silhuetas.
layout(location = 0) out vec2 outAmbientOcclusion;

vec3 octDecode(vec2 encoded) {
    encoded = encoded * 2.0 - 1.0;
    vec3 normal = vec3(encoded.x, encoded.y,
        1.0 - abs(encoded.x) - abs(encoded.y));
    float folded = clamp(-normal.z, 0.0, 1.0);
    normal.x += normal.x >= 0.0 ? -folded : folded;
    normal.y += normal.y >= 0.0 ? -folded : folded;
    return normalize(normal);
}

vec3 reconstructWorldPosition(vec2 uv, float ndcDepth) {
    vec2 ndc = uv * 2.0 - 1.0;
    vec4 clip = vec4(ndc.x, -ndc.y, ndcDepth, 1.0);
    vec4 world = scene.inverseCameraViewProjection * clip;
    return world.xyz / world.w;
}

// Hash inteiro sem a correlação diagonal do interleaved-gradient-noise. O
// padrão anterior era excelente para dithering, mas seus riscos diagonais
// ficavam visíveis em planos enormes vistos de raspão (o horizonte).
float pixelRotationNoise(ivec2 pixel) {
    uint value = uint(pixel.x) * 1597334677u
        ^ uint(pixel.y) * 3812015801u;
    value = (value ^ (value >> 16u)) * 2246822519u;
    value = (value ^ (value >> 13u)) * 3266489917u;
    value ^= value >> 16u;
    return float(value & 0x00ffffffu) / 16777216.0;
}

void main() {
    float centerDepth = texture(depthMap, texCoord).r;
    if (centerDepth <= 0.0001) {
        outAmbientOcclusion = vec2(1.0, 65504.0);
        return;
    }

    vec3 centerPosition =
        reconstructWorldPosition(texCoord, centerDepth);
    float centerDistance =
        length(centerPosition - scene.cameraPosition.xyz);
    if (pushConstants.strength <= 0.0001) {
        outAmbientOcclusion = vec2(1.0, centerDistance);
        return;
    }

    vec3 normal = octDecode(
        texture(normalRoughnessMap, texCoord).rg);
    vec2 fullResolution = vec2(textureSize(depthMap, 0));
    float projectedRadiusPixels = clamp(
        pushConstants.radiusMeters
            * fullResolution.y * 0.68 / max(centerDistance, 0.20),
        2.0, 36.0);
    vec2 texel = 1.0 / fullResolution;
    float rotation = pixelRotationNoise(ivec2(gl_FragCoord.xy))
        * 6.28318530718;

    float occlusion = 0.0;
    float weightSum = 0.0;
    // O limite fixo permite ao compilador otimizar o kernel, mas os perfis
    // encerram cedo: 3x2x1=6, 4x2x2=16 ou 6x2x2=24 amostras por pixel de
    // meia resolução.
    int directionCount = clamp(
        int(pushConstants.sampleDirectionCount), 3, 6);
    int stepCount = clamp(int(pushConstants.sampleStepCount), 1, 2);
    for (int directionIndex = 0; directionIndex < 6;
        ++directionIndex) {
        if (directionIndex >= directionCount) break;
        float angle = rotation
            + float(directionIndex) * 6.28318530718
                / float(directionCount);
        vec2 direction = vec2(cos(angle), sin(angle));
        for (int side = -1; side <= 1; side += 2) {
            float horizon = 0.0;
            float horizonWeight = 0.0;
            for (int stepIndex = 1; stepIndex <= 2; ++stepIndex) {
                if (stepIndex > stepCount) break;
                float stepFraction = stepCount == 1
                    ? 0.62
                    : float(stepIndex) / float(stepCount);
                vec2 sampleUv = texCoord + direction * float(side)
                    * texel * projectedRadiusPixels * stepFraction;
                if (any(lessThan(sampleUv, vec2(0.0)))
                    || any(greaterThan(sampleUv, vec2(1.0)))) {
                    continue;
                }
                float sampleDepth = texture(depthMap, sampleUv).r;
                if (sampleDepth <= 0.0001) continue;
                vec3 samplePosition =
                    reconstructWorldPosition(sampleUv, sampleDepth);
                vec3 offset = samplePosition - centerPosition;
                float distanceMeters = length(offset);
                if (distanceMeters <= 0.001
                    || distanceMeters > pushConstants.radiusMeters) {
                    continue;
                }
                vec3 sampleNormal = octDecode(
                    texture(normalRoughnessMap, sampleUv).rg);
                float heightAboveSurface = dot(normal, offset);
                // Um chão/paredão contínuo não pode ocluir a si mesmo.
                // Rejeitar amostras quase coplanares remove a amplificação
                // dos erros de precisão do depth em ângulos rasantes.
                float coplanarTolerance = max(0.018,
                    distanceMeters * 0.075);
                if (dot(normal, sampleNormal) > 0.965
                    && abs(heightAboveSurface) < coplanarTolerance) {
                    continue;
                }
                float geometricBias = max(
                    pushConstants.normalBias,
                    coplanarTolerance / distanceMeters);
                float tangentOcclusion = max(
                    heightAboveSurface / distanceMeters
                        - geometricBias, 0.0);
                float distanceWeight = 1.0 - smoothstep(
                    pushConstants.radiusMeters * 0.15,
                    pushConstants.radiusMeters, distanceMeters);
                // A contribuição distante cai quadraticamente. Isso mantém
                // contato forte nas quinas sem formar uma faixa grossa ao
                // redor de paredes quando o raio está alto.
                distanceWeight *= distanceWeight;
                horizon = max(horizon,
                    tangentOcclusion * distanceWeight);
                horizonWeight = max(horizonWeight, distanceWeight);
            }
            occlusion += horizon;
            weightSum += max(horizonWeight, 0.25);
        }
    }

    float normalizedOcclusion = occlusion / max(weightSum, 0.001);
    // AO é um efeito local. Em dezenas de metros a projeção subpixel do
    // depth é menos precisa e a contribuição visual deveria ser nula de
    // qualquer forma; o fade impede ruído no horizonte.
    float distanceFade = 1.0 - smoothstep(
        22.0, 48.0, centerDistance);
    float darkening = min(pushConstants.maxDarkening,
        normalizedOcclusion * pushConstants.strength)
        * distanceFade;
    outAmbientOcclusion = vec2(1.0 - darkening, centerDistance);
}
