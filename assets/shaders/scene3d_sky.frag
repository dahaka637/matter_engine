#version 450

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, usada nos motion vectors
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - nao usadas pelo ceu
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

// Mesmo layout de luz generica usado por scene3d_mesh.frag (ver
// GpuLightData3D/packSceneLights) - o ceu so le a direcao de lights[0], por
// convencao a luz direcional/sol da cena (ver
// Scene3DFrame::cascadeViewProjections).
struct GpuLight {
    vec4 positionRange;
    vec4 directionOuterCosine;
    vec4 colorIntensity;
    vec4 parameters;
};

layout(std430, set = 0, binding = 2) readonly buffer SceneLights {
    GpuLight lights[];
} sceneLights;

layout(location = 0) in vec2 clipPosition;
layout(location = 0) out vec4 outColor;
// O ceu compartilha o mesmo passe MRT que a mesh (ver scene3d_mesh.frag) -
// toda pipeline que desenha nesse passe precisa escrever os mesmos 2
// attachments, mesmo sem usar o segundo de verdade. Vetor zero: o ceu nunca
// e reprojetado pelo resolve de TAA (que sai cedo com base na profundidade
// de clear, ver taa_resolve.frag), entao este valor nunca chega a ser lido.
layout(location = 1) out vec2 outMotionVector;

// Hash 3D -> escalar em [0,1), baseado em pcg3d (Jarzynski & Olano, "Hash
// Functions for GPU Rendering", 2020) - mistura por multiplicacao, deslocamento
// de bits e XOR sobre inteiros, em vez das constantes float "magicas"
// 0.1031/0.1030/0.0973 do hash anterior. Essas tres constantes eram quase
// identicas entre si, o que correlacionava os tres eixos da grade: blocos
// inteiros e alinhados aos eixos acabavam recebendo valores parecidos (quase
// sempre baixos), visiveis como um "quadrado" de nuvem apagada escorregando
// pelo ceu conforme o vento desloca a amostragem (bug antigo, anterior a
// qualquer mudanca desta sessao). O vies de 65536 antes da conversao pra
// inteiro sem sinal garante um valor sempre nao-negativo para a faixa de
// coordenadas que os oitavos do fbm alcancam.
float hash31(vec3 p) {
    uvec3 v = uvec3(p + 65536.0);
    v = v * 1664525u + 1013904223u;
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    v ^= v >> 16u;
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    return float(v.x) * (1.0 / 4294967295.0);
}

float valueNoise3D(vec3 p) {
    vec3 cell = floor(p);
    vec3 f = fract(p);
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float x00 = mix(hash31(cell), hash31(cell + vec3(1, 0, 0)), f.x);
    float x10 = mix(hash31(cell + vec3(0, 1, 0)),
        hash31(cell + vec3(1, 1, 0)), f.x);
    float x01 = mix(hash31(cell + vec3(0, 0, 1)),
        hash31(cell + vec3(1, 0, 1)), f.x);
    float x11 = mix(hash31(cell + vec3(0, 1, 1)),
        hash31(cell + vec3(1, 1, 1)), f.x);
    return mix(mix(x00, x10, f.y), mix(x01, x11, f.y), f.z);
}

float fbm3D(vec3 p) {
    float result = 0.0;
    float amplitude = 0.54;
    mat3 rotation = mat3(
        0.00, 0.80, 0.60,
       -0.80, 0.36, -0.48,
       -0.60, -0.48, 0.64
    );
    for (int octave = 0; octave < 5; ++octave) {
        result += valueNoise3D(p) * amplitude;
        p = rotation * p * 2.02 + vec3(7.1, 11.7, 5.3);
        amplitude *= 0.48;
    }
    return result;
}

void main() {
    vec4 farPoint = scene.inverseCameraViewProjection
        * vec4(clipPosition.x, -clipPosition.y, 1.0, 1.0);
    vec3 worldFar = farPoint.xyz / max(abs(farPoint.w), 0.00001);
    vec3 viewDirection = normalize(worldFar - scene.cameraPosition.xyz);
    // lights[0] e, por convencao, a luz direcional/sol da cena - sem luz
    // nenhuma (nao deveria acontecer com showSky=true, mas o SSBO nunca fica
    // vazio de verdade, ver ensureSceneLightCapacity), cai pra uma direcao
    // vertical arbitraria so pra nao ler fora dos limites do array.
    int lightCount = int(scene.settings.y);
    vec3 sunDirection = lightCount > 0
        ? normalize(sceneLights.lights[0].directionOuterCosine.xyz)
        : vec3(0.0, 0.0, 1.0);

    float elevation = clamp(viewDirection.z * 0.5 + 0.5, 0.0, 1.0);
    vec3 horizon = vec3(0.61, 0.75, 0.88);
    vec3 zenith = vec3(0.12, 0.37, 0.68);
    vec3 sky = mix(horizon, zenith, pow(elevation, 0.72));
    sky += vec3(0.11, 0.075, 0.035)
        * pow(max(dot(viewDirection, sunDirection), 0.0), 7.0);

    if (viewDirection.z > -0.04) {
        // Noise sampled directly on the unit sky sphere has no planar
        // projection, poles or longitude seam, so it stays continuous while
        // the free camera turns through a full 360 degrees.
        // windOffset ja vem integrado no tempo pelo lado CPU (WindSystem +
        // WorkbenchApp) - direcao e intensidade reais do vento, nao mais um
        // scroll de direcao fixa proporcional so ao tempo decorrido.
        vec3 wind = vec3(scene.windOffset.xy, 0.0);
        vec3 cloudPoint = viewDirection * 3.8 + wind
            + vec3(scene.cameraPosition.xy * 0.0015, 0.0);
        float broadShape = fbm3D(cloudPoint);
        float fineShape = fbm3D(viewDirection * 8.2 - wind * 0.37 + 14.6);
        float density = broadShape * 0.78 + fineShape * 0.22;
        float threshold = 0.76 - clamp(scene.skySettings.z, 0.0, 1.0) * 0.42;
        float edgeFeather = max(fwidth(density) * 1.35, 0.035);
        float cloud = smoothstep(threshold - edgeFeather,
            threshold + 0.18 + edgeFeather, density);
        cloud = smoothstep(0.0, 1.0, cloud);
        cloud *= smoothstep(-0.03, 0.20, viewDirection.z);
        float cloudLight = 0.72 + 0.28 * max(dot(sunDirection,
            normalize(vec3(-viewDirection.xy, 0.55))), 0.0);
        vec3 cloudColor = mix(vec3(0.61, 0.67, 0.73), vec3(1.0, 0.98, 0.92), cloudLight);
        sky = mix(sky, cloudColor, cloud * 0.88);
    }

    float sunCore = smoothstep(0.99945, 0.99982, dot(viewDirection, sunDirection));
    float sunGlow = pow(max(dot(viewDirection, sunDirection), 0.0), 320.0);
    sky += vec3(1.0, 0.72, 0.31) * sunGlow * 0.65;
    sky = mix(sky, vec3(1.0, 0.94, 0.72), sunCore);
    outColor = vec4(sky, 1.0);
    outMotionVector = vec2(0.0);
}
