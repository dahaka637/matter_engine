#version 450

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, usada nos motion vectors
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - ver push constant abaixo
    vec4 cascadeSplits;                // distancia (espaco de camera) onde cada cascata termina
    vec4 cascadeTexelWorldSizes;       // metros por texel em cada cascata (bias de normal-offset)
    vec4 cascadeDepthRanges;           // extensao near/far em metros de cada projecao de sombra
    mat4 previousCameraViewProjection; // VP anterior estavel
    vec4 cameraPosition;
    vec4 settings;    // x=sombras, y=luzes, z=historico TAA valido, w=ambiente
    vec4 skySettings; // x=mostrar ceu, y=tempo do ceu, z=cobertura de nuvens, w reservado
    vec4 fogSettings; // x=densidade (por metro apos w), y=acoplamento altura-distancia, z=opacidade maxima, w=distancia de inicio (m)
    vec4 fogColor;    // rgb=cor da neblina, a reservado
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
} scene;

// Qual das 4 cascatas este passe esta desenhando agora (ver o laco de
// VulkanDevice::renderScene3DInternal, que empurra um indice diferente e
// desenha num mapa de sombra diferente a cada iteracao).
layout(push_constant) uniform ShadowCascadePush {
    uint cascadeIndex;
} cascadePush;

// Position only - the vertex buffer bound here is the same one the main
// mesh pipeline uses (see scene3d_mesh.vert), this pipeline just declares
// fewer attributes over the same stride.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 13) in uvec4 inJoints;
layout(location = 14) in vec4 inWeights;
layout(set = 0, binding = 4, std430) readonly buffer SkinPalette {
    mat4 matrices[];
} skin;

mat4 skinTransform(uint base) {
    return skin.matrices[base + inJoints.x] * inWeights.x
        + skin.matrices[base + inJoints.y] * inWeights.y
        + skin.matrices[base + inJoints.z] * inWeights.z
        + skin.matrices[base + inJoints.w] * inWeights.w;
}
layout(location = 4) in vec4 instancePositionScale;
layout(location = 5) in vec4 instanceOrientationX;
layout(location = 6) in vec4 instanceOrientationY;
layout(location = 7) in vec4 instanceOrientationZ;
// Para o pipeline da geometria fina (scene3d_mesh_shadow_thin.frag): a posicao
// no mundo e a fracao do fio engrossado que bloqueia a luz. O pipeline normal
// de sombra nao tem fragment shader e ignora as duas.
layout(location = 0) out vec3 outWorldPosition;
layout(location = 1) flat out float outCoverage;

void main() {
    mat3 orientation = mat3(instanceOrientationX.xyz,
        instanceOrientationY.xyz, instanceOrientationZ.xyz);
    float scale = instancePositionScale.w;
    vec3 worldPosition = instancePositionScale.xyz
        + orientation * (inPosition * scale);
    vec3 worldNormal = orientation * inNormal;
    if (instanceOrientationX.w > 0.5) {
        mat4 deform = skinTransform(uint(instanceOrientationX.w));
        worldPosition = (deform * vec4(inPosition, 1.0)).xyz;
        worldNormal = mat3(deform) * inNormal;
    }
    // Geometria fina (os fios da rede, w = o raio de verdade): engrossa ate
    // ~0,7 texel desta cascata. Mais fina que o texel, ela so cobria o centro
    // de um texel aqui e ali, e a sombra virava pontilhado piscando.
    float thinRadius = instanceOrientationZ.w;
    outCoverage = 1.0;
    if (thinRadius > 0.0 && dot(worldNormal, worldNormal) > 1e-8) {
        float target = 0.7 * scene.cascadeTexelWorldSizes[cascadePush.cascadeIndex];
        worldPosition += normalize(worldNormal) * max(0.0, target - thinRadius);
        // So a fracao do fio de verdade bloqueia a luz (e a metade dela: a
        // sombra da rede e sutil; as camadas do teto, do fundo e dos lados se
        // somam no chao).
        outCoverage = 0.5 * thinRadius / max(thinRadius, target);
    }
    outWorldPosition = worldPosition;
    gl_Position = scene.cascadeViewProjections[cascadePush.cascadeIndex]
        * vec4(worldPosition, 1.0);
    gl_Position.y = -gl_Position.y;
}
