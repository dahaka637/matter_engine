#version 450

// (17/11, pedido dele: a camera nao e mais empurrada) O pre-pass das
// superficies que podem virar vidro (MeshRender3D::seeThrough; os bonecos e a
// bola ficam de fora). Onde a superficie fica entre a camera e o jogador ela
// sai da profundidade aqui: o passe de cor (EQUAL) nao a desenha e o passe de
// vidro (scene3d_mesh.frag com MATTER_SEE_THROUGH_GHOST, teste GREATER) a
// desenha por cima, translucida. So este pre-pass tem fragment shader; o
// normal (scene3d_mesh_depth.vert sozinho) segue igual.

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, usada nos motion vectors
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter (usada pelo ceu)
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - ver shadowVisibility abaixo
    vec4 cascadeSplits;                // distancia (espaco de camera) onde cada cascata termina
    vec4 cascadeTexelWorldSizes;       // metros por texel em cada cascata (bias de normal-offset)
    vec4 cascadeDepthRanges;           // extensao near/far em metros de cada projecao de sombra
    mat4 previousCameraViewProjection; // VP anterior estavel, usada nos motion vectors
    vec4 cameraPosition;
    vec4 settings;    // x=amostras PCSS, y=luzes, z=historico TAA, w=ambiente
    vec4 skySettings; // x=mostrar ceu, y=tempo, z=nuvens, w=transmitancia solar
    vec4 fogSettings; // x=densidade (por metro apos w), y=acoplamento altura-distancia, z=opacidade maxima, w=distancia de inicio (m)
    vec4 fogColor;    // rgb=cor da neblina, a=distancia de ocultacao total
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
    vec4 environmentSettings; // x=visibilidade indireta minima, y=rebote solar, z=chuva, w=umidade
    vec4 renderSettings;      // x=Pixel Art, y=tamanho base do pixel, z=TAA completo
    // (17/11) A camera sem empurrao - ver seeThroughMask: xyz o pe e a cabeca
    // do jogador; w do primeiro = forca do efeito (0 desligado), do segundo =
    // altura do chao sob ele. Shape: x,y raio da janela (vidro inteiro ate x,
    // opaco a partir de y), z,w o vidro perto da camera (inteiro ate z, nada
    // a partir de w).
    vec4 seeThroughBottom;
    vec4 seeThroughTop;
    vec4 seeThroughShape;
} scene;

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 worldNormal;
layout(location = 5) flat in int materialFlags;

// A MESMA conta de scene3d_mesh.frag - as duas copias precisam dar o mesmo
// numero (sem include compartilhado neste projeto).
float seeThroughMask(vec3 position, vec3 geometricNormal) {
    float strength = scene.seeThroughBottom.w;
    if (strength <= 0.0) return 0.0;
    // O piso na altura do jogador nunca vira vidro.
    if (geometricNormal.z > 0.7 && position.z < scene.seeThroughTop.w + 0.25) return 0.0;
    vec3 origin = scene.cameraPosition.xyz;
    vec3 toSurface = position - origin;
    float surfaceDistance = length(toSurface);
    vec3 direction = toSurface / max(surfaceDistance, 1e-4);
    // O raio da camera por esta superficie, prolongado: onde ele passa mais
    // perto do corpo (o segmento pe-cabeca) e a que distancia da camera.
    vec3 bodyBottom = scene.seeThroughBottom.xyz;
    vec3 bodyAxis = scene.seeThroughTop.xyz - bodyBottom;
    vec3 fromBody = origin - bodyBottom;
    float axisDot = dot(direction, bodyAxis);
    float axisLength2 = dot(bodyAxis, bodyAxis);
    float denominator = axisLength2 - axisDot * axisDot;
    float along = denominator > 1e-6
        ? clamp((dot(bodyAxis, fromBody) - axisDot * dot(direction, fromBody)) / denominator, 0.0, 1.0)
        : 0.0;
    vec3 bodyPoint = bodyBottom + bodyAxis * along;
    float bodyDepth = max(dot(bodyPoint - origin, direction), 0.0);
    float missDistance = length(origin + direction * bodyDepth - bodyPoint);
    // (17/11, pedidos dele: "tudo que esta entre a camera e o jogador fica
    // transparente... nao apenas ao redor do corpo"; "tudo... ao gradativo a
    // opacidade, nao so ao redor do jogador") Tudo na frente da profundidade
    // dele vira vidro na tela inteira, gradativo pela profundidade: inteiro a
    // 1,2 m na frente dele e voltando aos poucos a opaco ate ela. Junto da
    // linha da camera ao corpo (ate x, sumindo ate y) a volta e curta (0,3 m):
    // o que esta colado na frente dele tambem nao o esconde.
    float depthGap = bodyDepth - surfaceDistance;
    float everywhere = smoothstep(0.0, 1.2, depthGap);
    float closeness = 1.0 - smoothstep(scene.seeThroughShape.x, scene.seeThroughShape.y, missDistance);
    float nearBody = closeness * smoothstep(0.0, 0.3, depthGap);
    float nearCamera = 1.0 - smoothstep(scene.seeThroughShape.z, scene.seeThroughShape.w, surfaceDistance);
    return strength * max(max(everywhere, nearBody), nearCamera);
}

void main() {
    // Bit 8192 de materialFlags: pode virar vidro (VulkanDevice, materialFlags).
    if ((materialFlags & 8192) != 0
            && seeThroughMask(worldPosition, normalize(worldNormal)) > 0.002) {
        discard;
    }
}
