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
    vec4 fogSettings; // x=densidade, y=acoplamento altura-distancia, z=opacidade maxima, w reservado
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
layout(location = 4) in vec4 instancePositionScale;
layout(location = 5) in vec4 instanceOrientationX;
layout(location = 6) in vec4 instanceOrientationY;
layout(location = 7) in vec4 instanceOrientationZ;

void main() {
    mat3 orientation = mat3(instanceOrientationX.xyz,
        instanceOrientationY.xyz, instanceOrientationZ.xyz);
    float scale = instancePositionScale.w;
    vec3 worldPosition = instancePositionScale.xyz
        + orientation * (inPosition * scale);
    gl_Position = scene.cascadeViewProjections[cascadePush.cascadeIndex]
        * vec4(worldPosition, 1.0);
    gl_Position.y = -gl_Position.y;
}
