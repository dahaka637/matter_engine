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
    vec4 skySettings; // x=mostrar ceu, y=tempo, z=nuvens, w=transmitancia solar
    vec4 fogSettings; // x=densidade (por metro apos w), y=acoplamento altura-distancia, z=opacidade maxima, w=distancia de inicio (m)
    vec4 fogColor;    // rgb=cor da neblina, a reservado
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
    vec4 environmentSettings; // layout compartilhado; não usado pelo céu
    vec4 renderSettings; // x=Pixel Art, y=tamanho base do pixel, zw reservados
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

// As nuvens so existem no hemisferio superior do ceu. Nesse dominio a
// projecao octaedrica e continua e nos permite avaliar um noise 2D com quatro
// cantos em vez dos oito cantos do antigo volume 3D. Alem de reduzir pela
// metade os hashes por oitava, este hash escalar evita toda a mistura uvec3
// do campo estelar (que continua usando hash31 e nao muda visualmente).
float hash21(vec2 p) {
    uvec2 v = uvec2(p + 65536.0);
    uint h = v.x * 1597334677u ^ v.y * 3812015801u;
    h ^= h >> 16u;
    h *= 2246822519u;
    h ^= h >> 13u;
    h *= 3266489917u;
    h ^= h >> 16u;
    return float(h) * (1.0 / 4294967295.0);
}

float valueNoise2D(vec2 p) {
    vec2 cell = floor(p);
    vec2 f = fract(p);
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float x0 = mix(hash21(cell), hash21(cell + vec2(1, 0)), f.x);
    float x1 = mix(hash21(cell + vec2(0, 1)),
        hash21(cell + vec2(1, 1)), f.x);
    return mix(x0, x1, f.y);
}

float fbm3D(vec3 p) {
    float result = 0.0;
    float amplitude = 0.54;
    mat3 rotation = mat3(
        0.00, 0.80, 0.60,
       -0.80, 0.36, -0.48,
       -0.60, -0.48, 0.64
    );
    // Três oitavas constroem a silhueta ampla. O detalhe fino logo abaixo já
    // fornece a quarta escala visual; repetir outra valueNoise3D aqui fazia
    // oito hashes extras em cada pixel do céu sem mudar o contorno da nuvem.
    for (int octave = 0; octave < 3; ++octave) {
        result += valueNoise3D(p) * amplitude;
        p = rotation * p * 2.02 + vec3(7.1, 11.7, 5.3);
        amplitude *= 0.48;
    }
    return result;
}

float cloudFbm(vec2 p) {
    float result = 0.0;
    float amplitude = 0.54;
    mat2 rotation = mat2(0.80, 0.60, -0.60, 0.80);
    for (int octave = 0; octave < 3; ++octave) {
        result += valueNoise2D(p) * amplitude;
        p = rotation * p * 2.02 + vec2(7.1, 11.7);
        amplitude *= 0.48;
    }
    return result;
}

vec2 octahedralDirection(vec3 direction) {
    direction /= abs(direction.x) + abs(direction.y)
        + abs(direction.z);
    vec2 encoded = direction.xy;
    if (direction.z < 0.0) {
        vec2 folded = vec2(1.0) - abs(encoded.yx);
        encoded = vec2(
            encoded.x >= 0.0 ? folded.x : -folded.x,
            encoded.y >= 0.0 ? folded.y : -folded.y);
    }
    return encoded * 0.5 + 0.5;
}

// Campo estelar ancorado numa projeção octaédrica do mundo. A versão
// anterior usava células 3D atravessadas pelo raio da câmera: estrelas muito
// pequenas trocavam de célula no meio de um pixel e pareciam piscar ao girar.
// Aqui cada estrela tem uma posição 2D permanente, consulta também as oito
// células vizinhas e usa fwidth para cobertura subpixel estável.
vec3 stableStarField(vec3 direction) {
    const float StarGridScale = 285.0;
    vec2 lattice = octahedralDirection(direction) * StarGridScale;
    vec2 baseCell = floor(lattice);
    vec2 local = fract(lattice);
    vec3 result = vec3(0.0);
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            vec2 cellOffset = vec2(x, y);
            vec2 cell = baseCell + cellOffset;
            float seed = hash31(vec3(cell, 23.0));
            float exists = step(0.9870, seed);
            vec2 starPosition = cellOffset + vec2(
                fract(seed * 37.17 + 0.13),
                fract(seed * 91.73 + 0.47));
            float starKind = fract(seed * 173.31);
            // Pontos pequenos; apenas as estrelas raras chegam perto de um
            // pixel inteiro. A variacao de tamanho sugere distancia sem
            // transformar o ponto numa flare ou numa cruz.
            float radius = mix(0.040, 0.078,
                starKind * starKind);
            float distanceToStar = length(local - starPosition);
            // Garante cobertura suficiente para o centro do bloco artístico
            // sem criar halo ou animação de brilho.
            float latticeFootprint = max(
                length(dFdx(lattice)), length(dFdy(lattice)));
            if (scene.renderSettings.x > 0.5) {
                radius = max(radius, latticeFootprint
                    * max(scene.renderSettings.y, 1.0) * 0.56);
            }
            // fwidth cresce muito nas dobras da projecao octaedrica. Sem o
            // teto, a cobertura se alongava por varios pixels e criava as
            // "faíscas" em X vistas no céu.
            float antialiasWidth = clamp(
                fwidth(distanceToStar) * 0.55, 0.008, 0.024);
            float coverage = 1.0 - smoothstep(
                radius - antialiasWidth,
                radius + antialiasWidth, distanceToStar);
            // Brilho falso somente por intensidade e temperatura da cor:
            // nenhum halo, bloom ou variacao temporal.
            float intensity = mix(0.22, 0.92,
                starKind * starKind);
            float temperature = fract(seed * 57.31);
            vec3 warm = vec3(1.00, 0.84, 0.70);
            vec3 cool = vec3(0.70, 0.84, 1.00);
            vec3 starColor = mix(warm, cool, temperature);
            result = max(result,
                starColor * exists * coverage * intensity);
        }
    }
    return result;
}

void main() {
    vec4 farPoint = scene.inverseCameraViewProjection
        * vec4(clipPosition.x, -clipPosition.y, 1.0, 1.0);
    vec3 worldFar = farPoint.xyz / max(abs(farPoint.w), 0.00001);
    vec3 viewDirection = normalize(worldFar - scene.cameraPosition.xyz);
    // Ordem do contrato do laboratório: Sol em lights[0], Lua em lights[1].
    // A intensidade pode chegar a zero, mas as direções continuam válidas e
    // dirigem a atmosfera, estrelas e discos celestes.
    int lightCount = int(scene.settings.y);
    vec3 sunDirection = lightCount > 0
        ? normalize(sceneLights.lights[0].directionOuterCosine.xyz)
        : vec3(0.0, 0.0, 1.0);
    vec3 moonDirection = lightCount > 1
        ? normalize(sceneLights.lights[1].directionOuterCosine.xyz)
        : -sunDirection;

    float sunElevation = sunDirection.z;
    float dayFactor = smoothstep(-0.10, 0.10, sunElevation);
    float nightFactor = 1.0 - smoothstep(-0.16, 0.025, sunElevation);
    float twilight = (1.0 - smoothstep(0.015, 0.30,
        abs(sunElevation))) * (1.0 - nightFactor * 0.55);
    float skyHeight = clamp((viewDirection.z + 0.035) / 1.035,
        0.0, 1.0);
    float horizonWeight = 1.0 - smoothstep(0.02, 0.58, skyHeight);
    float towardSun = pow(max(dot(viewDirection, sunDirection), 0.0), 3.0);

    vec3 dayHorizon = vec3(0.61, 0.75, 0.88);
    vec3 dayZenith = vec3(0.10, 0.34, 0.67);
    vec3 nightHorizon = vec3(0.018, 0.026, 0.060);
    vec3 nightZenith = vec3(0.0015, 0.0045, 0.019);
    vec3 daySky = mix(dayHorizon, dayZenith, pow(skyHeight, 0.62));
    vec3 nightSky = mix(nightHorizon, nightZenith,
        pow(skyHeight, 0.48));
    vec3 sky = mix(nightSky, daySky, dayFactor);

    // A faixa quente só nasce perto do horizonte e na direção do Sol. O
    // lado oposto preserva tons frios, evitando o céu inteiro laranja.
    vec3 sunsetColor = mix(vec3(1.02, 0.20, 0.035),
        vec3(0.75, 0.24, 0.10), skyHeight);
    sky += sunsetColor * twilight * horizonWeight
        * (0.18 + towardSun * 0.82) * 0.78;
    sky += vec3(0.11, 0.070, 0.030)
        * pow(max(dot(viewDirection, sunDirection), 0.0), 7.0)
        * dayFactor;

    // O ramo depende apenas do horario (uniforme para o draw inteiro).
    // Durante o dia evita executar o campo estelar de nove celulas para
    // depois multiplica-lo por zero.
    vec3 stars = vec3(0.0);
    if (nightFactor > 0.001) {
        stars = stableStarField(viewDirection)
            * smoothstep(-0.08, 0.20, viewDirection.z)
            * nightFactor;
    }

    float cloudOpacity = 0.0;
    if (viewDirection.z > -0.04) {
        // Noise sampled directly on the unit sky sphere has no planar
        // projection, poles or longitude seam, so it stays continuous while
        // the free camera turns through a full 360 degrees.
        // windOffset ja vem integrado no tempo pelo lado CPU (WindSystem +
        // WorkbenchApp) - direcao e intensidade reais do vento, nao mais um
        // scroll de direcao fixa proporcional so ao tempo decorrido.
        vec2 wind = scene.windOffset.xy;
        vec2 cloudDirection = octahedralDirection(viewDirection);
        vec2 cloudPoint = cloudDirection * 12.0 + wind
            + scene.cameraPosition.xy * 0.0015;
        float broadShape = cloudFbm(cloudPoint);
        // O detalhe participa com apenas 22% da densidade. Uma oitava de
        // noise já quebra a borda sem repetir outro fBm completo.
        float fineShape = valueNoise2D(
            cloudDirection * 27.0 - wind * 0.37 + 14.6);
        float density = broadShape * 0.78 + fineShape * 0.22;
        float coverage = clamp(scene.skySettings.z, 0.0, 1.0);
        float threshold = 0.80 - coverage * 0.48;
        float edgeFeather = max(fwidth(density) * 1.35, 0.035);
        float cloud = smoothstep(threshold - edgeFeather,
            threshold + 0.18 + edgeFeather, density);
        cloud = smoothstep(0.0, 1.0, cloud);
        cloud *= smoothstep(-0.03, 0.20, viewDirection.z);
        cloud *= smoothstep(0.015, 0.13, coverage);
        float cloudLight = 0.72 + 0.28 * max(dot(sunDirection,
            normalize(vec3(-viewDirection.xy, 0.55))), 0.0);
        vec3 daytimeCloud = mix(vec3(0.46, 0.51, 0.58),
            vec3(1.0, 0.98, 0.92), cloudLight);
        vec3 nighttimeCloud = mix(vec3(0.012, 0.018, 0.038),
            vec3(0.055, 0.070, 0.12), cloudLight);
        vec3 cloudColor = mix(nighttimeCloud, daytimeCloud, dayFactor);
        cloudColor += vec3(0.42, 0.10, 0.025)
            * twilight * towardSun * 0.55;
        cloudOpacity = cloud * 0.90;
        sky = mix(sky, cloudColor, cloudOpacity);
    }

    sky += stars * (1.0 - cloudOpacity);

    float sunAlignment = max(dot(viewDirection, sunDirection), 0.0);
    float sunCore = smoothstep(0.99945, 0.99982, sunAlignment);
    float sunGlow = pow(sunAlignment, 230.0);
    float sunAboveHorizon = smoothstep(-0.015, 0.025, sunElevation);
    float sunVisibility = sunAboveHorizon
        * clamp(scene.skySettings.w, 0.0, 1.0)
        * (1.0 - cloudOpacity * 0.98);
    vec3 sunTint = mix(vec3(1.35, 0.34, 0.075),
        vec3(1.20, 0.82, 0.36),
        smoothstep(0.02, 0.42, sunElevation));
    // Radiância deliberadamente acima de 1.0: o ACES comprime o disco para
    // branco quente, enquanto o glare HDR conserva a sensação incômoda de
    // olhar diretamente para uma fonte muito mais intensa que o céu.
    sky += sunTint * sunGlow * 0.92 * sunVisibility;
    sky += mix(vec3(5.30, 2.10, 0.58), vec3(4.80, 3.85, 2.45),
        smoothstep(0.02, 0.35, sunElevation))
        * sunCore * sunVisibility;

    float moonAboveHorizon = smoothstep(-0.02, 0.05, moonDirection.z);
    float moonVisibility = moonAboveHorizon * nightFactor
        * (1.0 - cloudOpacity * 0.96);
    if (moonVisibility > 0.001) {
        float moonAlignment = max(
            dot(viewDirection, moonDirection), 0.0);
        float moonDisc = smoothstep(0.99958, 0.99982, moonAlignment);
        float moonHalo = pow(moonAlignment, 310.0);
        float lunarDetail = mix(0.72, 1.05,
            valueNoise3D(viewDirection * 125.0 + 19.0));
        sky += vec3(0.15, 0.21, 0.34)
            * moonHalo * moonVisibility;
        sky += vec3(1.05, 1.10, 1.18) * moonDisc
            * lunarDetail * moonVisibility * 1.35;
    }
    outColor = vec4(sky, 1.0);
    outMotionVector = vec2(0.0);
}
