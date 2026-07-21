#version 450

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, usada nos motion vectors
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - nao usadas por este pre-pass
    vec4 cascadeSplits;                // idem - so aqui pra bater o layout do UBO
    vec4 cascadeTexelWorldSizes;       // idem - so aqui pra bater o layout do UBO
    vec4 cascadeDepthRanges;           // idem - so aqui pra bater o layout do UBO
    mat4 previousCameraViewProjection; // VP anterior estavel
    vec4 cameraPosition;
    vec4 settings;    // x=sombras, y=luzes, z=historico TAA valido, w=ambiente
    vec4 skySettings; // x=mostrar ceu, y=tempo do ceu, z=cobertura de nuvens, w reservado
    vec4 fogSettings; // x=densidade, y=acoplamento altura-distancia, z=opacidade maxima, w reservado
    vec4 fogColor;    // rgb=cor da neblina, a reservado
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
} scene;

// Pre-pass de profundidade da cena principal (nao da sombra - ver
// scene3d_mesh_shadow.vert para essa) - so posicao, mesmo vertex buffer que
// scene3d_mesh.vert usa, essa pipeline so declara menos atributos sobre o
// mesmo stride.
layout(location = 0) in vec3 inPosition;
layout(location = 4) in vec4 instancePositionScale;
layout(location = 5) in vec4 instanceOrientationX;
layout(location = 6) in vec4 instanceOrientationY;
layout(location = 7) in vec4 instanceOrientationZ;

void main() {
    // Calculo identico ao de scene3d_mesh.vert, de proposito: o passe de cor
    // (sceneMeshPipeline) testa profundidade com EQUAL contra o que este
    // pre-pass escreve, entao qualquer divergencia aqui - mesmo so na ordem
    // das operacoes - viraria z-fighting visivel (buracos piscando na
    // geometria). Nao fatorar num include/funcao compartilhada e
    // deliberado: os dois arquivos precisam continuar sendo comparados
    // lado a lado se algum dia um dos dois mudar.
    mat3 orientation = mat3(instanceOrientationX.xyz,
        instanceOrientationY.xyz, instanceOrientationZ.xyz);
    float scale = instancePositionScale.w;
    vec3 worldPosition = instancePositionScale.xyz
        + orientation * (inPosition * scale);
    gl_Position = scene.cameraViewProjection * vec4(worldPosition, 1.0);
    gl_Position.y = -gl_Position.y;
}
