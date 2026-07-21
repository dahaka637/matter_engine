#version 450

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, usada nos motion vectors
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter (usada pelo ceu)
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - usadas so no fragment shader agora
    vec4 cascadeSplits;                // distancia (espaco de camera) onde cada cascata termina
    vec4 cascadeTexelWorldSizes;       // metros por texel em cada cascata (bias de normal-offset)
    vec4 cascadeDepthRanges;           // extensao near/far em metros de cada projecao de sombra
    mat4 previousCameraViewProjection; // VP anterior estavel, usada nos motion vectors
    vec4 cameraPosition;
    vec4 settings;    // x=sombras, y=luzes, z=historico TAA valido, w=ambiente
    vec4 skySettings; // x=mostrar ceu, y=tempo do ceu, z=cobertura de nuvens, w reservado
    vec4 fogSettings; // x=densidade, y=acoplamento altura-distancia, z=opacidade maxima, w reservado
    vec4 fogColor;    // rgb=cor da neblina, a reservado
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
} scene;

// Reuses the exact push constant layout the analytic box/sphere pipeline
// uses (see scene3d.vert): center is the mesh's world position,
// halfExtents.x is a uniform scale, orientation is the rotation basis,
// parameters.z is the "selected" outline flag. materialParameters is unused
// here - color/albedo travel via the vertex buffer and the set=1 material
// texture instead, but albedo.xy is repurposed to carry this mesh's
// metallic/roughness through to the fragment shader's analytic sky
// reflection (see MeshRender3D / VulkanDevice's pushMesh).
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in vec3 inColor;
layout(location = 4) in vec4 instancePositionScale;
layout(location = 5) in vec4 instanceOrientationX;
layout(location = 6) in vec4 instanceOrientationY;
layout(location = 7) in vec4 instanceOrientationZ;
layout(location = 8) in vec4 instanceMaterialAndFlags;
// Transformacao do MESMO objeto no quadro anterior - so para o vetor de
// movimento por pixel (ver outMotionClipPosition abaixo e
// SceneMeshInstanceGpu no lado CPU).
layout(location = 9) in vec4 instancePreviousPositionScale;
layout(location = 10) in vec4 instancePreviousOrientationX;
layout(location = 11) in vec4 instancePreviousOrientationY;
layout(location = 12) in vec4 instancePreviousOrientationZ;

layout(location = 0) out vec3 worldPosition;
layout(location = 1) out vec3 worldNormal;
layout(location = 2) out vec2 vertexUv;
layout(location = 3) out vec3 vertexColor;
// Local 4 nao e mais usado (era lightClipPosition - a selecao de cascata e o
// calculo da posicao em clip space da luz agora vivem inteiramente no
// fragment shader, que ja recebe worldPosition; ver shadowVisibility em
// scene3d_mesh.frag). Deixado vago de proposito em vez de renumerar 5-8 pra
// baixo - GLSL/SPIR-V nao exige locations contiguas, e isso evita mexer em
// mais linhas do que o necessario.
layout(location = 5) flat out ivec4 objectParameters;
layout(location = 6) flat out vec2 objectMetallicRoughness;
// Posicao em clip space ESTAVEL deste quadro e do anterior (camera E objeto,
// nao so a camera) - o fragment shader divide cada uma pelo seu proprio w e
// subtrai pra obter o vetor de movimento (ver scene3d_mesh.frag). As duas VPs
// sao deliberadamente sem jitter: o jitter pertence apenas a gl_Position e
// nao ao deslocamento da grade na qual o historico resolvido e armazenado. Usar a
// transformacao do PROPRIO objeto aqui (nao so a matriz da camera) e o que
// torna isto correto tambem para objetos que se moveram entre os dois
// quadros (um prop lancado pela physgun, por exemplo), diferente da
// reconstrucao por profundidade que o TAA usava antes (Fase 6 original),
// que so sabia reprojetar a camera - qualquer objeto em movimento virava
// rastro/ghosting visivel nela.
layout(location = 7) out vec4 currentClipPosition;
layout(location = 8) out vec4 previousMotionClipPosition;

void main() {
    mat3 orientation = mat3(instanceOrientationX.xyz,
        instanceOrientationY.xyz, instanceOrientationZ.xyz);
    float scale = instancePositionScale.w;
    worldPosition = instancePositionScale.xyz
        + orientation * (inPosition * scale);
    worldNormal = normalize(orientation * inNormal);
    vertexUv = inUv;
    vertexColor = inColor;
    objectParameters = ivec4(3, 0,
        instanceMaterialAndFlags.z > 0.5 ? 1 : 0,
        instanceMaterialAndFlags.w > 0.5 ? 1 : 0);
    objectMetallicRoughness = instanceMaterialAndFlags.xy;
    gl_Position = scene.cameraViewProjection * vec4(worldPosition, 1.0);
    gl_Position.y = -gl_Position.y;

    mat3 previousOrientation = mat3(instancePreviousOrientationX.xyz,
        instancePreviousOrientationY.xyz, instancePreviousOrientationZ.xyz);
    vec3 previousWorldPosition = instancePreviousPositionScale.xyz
        + previousOrientation * (inPosition * instancePreviousPositionScale.w);
    currentClipPosition = scene.cameraViewProjectionUnjittered
        * vec4(worldPosition, 1.0);
    currentClipPosition.y = -currentClipPosition.y;
    previousMotionClipPosition = scene.previousCameraViewProjection
        * vec4(previousWorldPosition, 1.0);
    previousMotionClipPosition.y = -previousMotionClipPosition.y;
}
