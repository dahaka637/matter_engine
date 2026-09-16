#version 450

// Reflexos em espaco de tela (SSR): traça um raio em espaco de mundo a
// partir de cada pixel refletor, seguindo a profundidade da cena já
// renderizada, e acumula o resultado no tempo (mesma técnica do resolve de
// TAA, ver taa_resolve.frag). Roda em MEIA resolução (ver
// VulkanDevice::ensureScene3DTarget) - metade linear é 1/4 dos pixels,
// o corte de custo mais efetivo pra hardware antigo (GTX 750 e
// equivalentes, ver AGENTS.md/plano de reflexos). Reaproveita
// tonemap.vert como estágio de vértice, igual ao resolve de TAA.

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, nao usada aqui
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter (reconstrucao de posicao)
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - nao usadas pelo SSR
    vec4 cascadeSplits;                // idem - so aqui pra bater o layout do UBO
    vec4 cascadeTexelWorldSizes;       // idem - so aqui pra bater o layout do UBO
    vec4 cascadeDepthRanges;           // idem - so aqui pra bater o layout do UBO
    mat4 previousCameraViewProjection; // nao usada aqui
    vec4 cameraPosition;
    vec4 settings;    // x=sombras, y=luzes, z=historico temporal valido, w=ambiente
    vec4 skySettings; // nao usada aqui
    vec4 fogSettings; // nao usada aqui
    vec4 fogColor;    // nao usada aqui
    vec4 windOffset;  // nao usada aqui
} scene;

// Mesmo layout de luz generica usado por scene3d_mesh.frag/scene3d_sky.frag
// (ver GpuLightData3D/packSceneLights) - so lights[0] importa aqui, a
// direcao do sol usada pelo fallback analitico (ver
// reflectionEnvironmentColor abaixo).
struct GpuLight {
    vec4 positionRange;
    vec4 directionOuterCosine;
    vec4 colorIntensity;
    vec4 parameters;
};

layout(std430, set = 0, binding = 2) readonly buffer SceneLights {
    GpuLight lights[];
} sceneLights;

// Todas as texturas de entrada sao cheias resolucao, exceto historyMap
// (meia resolucao, o mesmo alvo que este shader escreve).
layout(set = 1, binding = 0) uniform sampler2D hdrColorMap;
layout(set = 1, binding = 1) uniform sampler2D depthMap;
layout(set = 1, binding = 2) uniform sampler2D normalRoughnessMap;
layout(set = 1, binding = 3) uniform sampler2D reflectanceMap;
layout(set = 1, binding = 4) uniform sampler2D motionVectorMap;
layout(set = 1, binding = 5) uniform sampler2D historyMap;

layout(location = 0) in vec2 texCoord;
layout(location = 0) out vec4 outColor;

// Inverso de octEncode (scene3d_mesh.frag) - ver Meyer et al., "On
// Floating-Point Normal Vectors", 2010.
vec3 octDecode(vec2 encoded) {
    encoded = encoded * 2.0 - 1.0;
    vec3 normal = vec3(encoded.x, encoded.y, 1.0 - abs(encoded.x) - abs(encoded.y));
    float t = clamp(-normal.z, 0.0, 1.0);
    normal.x += normal.x >= 0.0 ? -t : t;
    normal.y += normal.y >= 0.0 ? -t : t;
    return normalize(normal);
}

// Desfaz profundidade+uv de volta pra posicao no mundo. Mesma convencao de
// scene3d_sky.frag (flip de Y - o triangulo cheio de tela deste shader,
// herdado de tonemap.vert, NAO flipa Y sozinho, entao a UV aqui esta na
// convencao direta [0,0]=topo-esquerda; scene.inverseCameraViewProjection
// espera NDC no mesmo sistema que o vertex shader das meshes produz, que
// SIM flipa - ver scene3d_mesh.vert). Usa a inversa SEM jitter (unica
// disponivel no UBO); o erro de nao contabilizar o jitter e sub-pixel,
// irrelevante pra um raio com folga de espessura em metros (ver a
// espessura proporcional a distancia em traceReflection).
vec3 reconstructWorldPosition(vec2 uv, float ndcDepth) {
    vec2 ndc = uv * 2.0 - 1.0;
    vec4 clip = vec4(ndc.x, -ndc.y, ndcDepth, 1.0);
    vec4 world = scene.inverseCameraViewProjection * clip;
    return world.xyz / world.w;
}

// Projeta um ponto do mundo pra UV de tela, com o MESMO flip de Y que
// scene3d_mesh.vert aplica em gl_Position. O ray march usa a VP estavel:
// ela e a inversa exata da matriz usada em reconstructWorldPosition e evita
// que o jitter do TAA introduza um erro de ida-e-volta a cada passo.
bool projectToScreen(vec3 worldPosition, out vec2 uv) {
    vec4 clip = scene.cameraViewProjectionUnjittered
        * vec4(worldPosition, 1.0);
    clip.y = -clip.y;
    if (clip.w <= 0.001) {
        uv = vec2(0.0);
        return false;
    }
    uv = (clip.xy / clip.w) * 0.5 + 0.5;
    return true;
}

// Mesmo ambiente analitico que o shader da mesh usava diretamente antes
// desta fase (ver historico de scene3d_mesh.frag) - agora serve só de
// fallback, para quando o traçado de tela nao acerta nada (fora da tela,
// atras da camera, ou geometria ausente).
vec3 reflectionEnvironmentColor(vec3 direction) {
    int lightCount = int(scene.settings.y);
    vec3 sunDirection = lightCount > 0
        ? normalize(sceneLights.lights[0].directionOuterCosine.xyz)
        : vec3(0.0, 0.0, 1.0);
    vec3 groundColor = vec3(0.16, 0.15, 0.14);
    vec3 horizon = vec3(0.58, 0.72, 0.86);
    vec3 zenith = vec3(0.03, 0.12, 0.32);
    vec3 environment;
    if (direction.z >= 0.0) {
        environment = mix(horizon, zenith, pow(direction.z, 0.55));
    } else {
        float depth = -direction.z;
        environment = mix(horizon, groundColor, pow(depth, 0.4));
        environment = mix(environment, groundColor * 0.55, pow(depth, 2.5));
    }
    float sunDot = max(dot(direction, sunDirection), 0.0);
    float sunGlow = pow(sunDot, 220.0);
    environment += vec3(1.0, 0.78, 0.42) * sunGlow * 2.2;
    float sunCore = smoothstep(0.99900, 0.99985, sunDot);
    environment = mix(environment, vec3(1.0, 0.97, 0.88), sunCore);
    return environment;
}

const int SsrStepCount = 40;
const float SsrMaxDistanceMeters = 20.0;
const int SsrRefineSteps = 7;

// Mede, em metros, se o ponto marchado esta na frente (<0) ou atras (>0) da
// primeira superficie visivel naquele pixel. Os dois pontos pertencem ao
// mesmo raio da camera, portanto a diferenca de distancias e uma medida
// linear mesmo com Reversed-Z.
bool sampleRayGap(vec3 marched, out vec2 uv, out float gap,
        out vec3 scenePosition) {
    if (!projectToScreen(marched, uv)
        || any(lessThanEqual(uv, vec2(0.0)))
        || any(greaterThanEqual(uv, vec2(1.0)))) {
        return false;
    }
    float sceneNdcDepth = texture(depthMap, uv).r;
    if (sceneNdcDepth <= 0.0001) return false;
    scenePosition = reconstructWorldPosition(uv, sceneNdcDepth);
    gap = distance(marched, scene.cameraPosition.xyz)
        - distance(scenePosition, scene.cameraPosition.xyz);
    return true;
}

// Marcha grosseira seguida de refinamento binario. Um hit so existe quando
// o raio CRUZA a profundidade (frente -> tras); a versao anterior aceitava
// qualquer amostra que ja estivesse atras e "perto o bastante", fazendo o
// espelho acertar a propria superficie e esticar silhuetas pela tela.
bool traceReflection(vec3 worldStart, vec3 worldDirection, vec2 sourceUv,
        out vec3 hitColor, out float hitConfidence) {
    hitColor = vec3(0.0);
    hitConfidence = 0.0;
    float startDistance = distance(worldStart, scene.cameraPosition.xyz);
    float maxDistance = clamp(startDistance * 2.5, 3.0,
        SsrMaxDistanceMeters);
    float minDistance = clamp(startDistance * 0.004, 0.035, 0.12);

    float previousT = minDistance;
    float previousGap = 0.0;
    bool previousSampleValid = false;
    float lowT = minDistance;
    float highT = minDistance;
    bool found = false;

    for (int i = 1; i <= SsrStepCount; ++i) {
        float t01 = float(i) / float(SsrStepCount);
        float currentT = mix(minDistance, maxDistance, t01 * t01);
        vec3 marched = worldStart + worldDirection * currentT;
        vec2 uv;
        float gap;
        vec3 scenePosition;
        if (!sampleRayGap(marched, uv, gap, scenePosition)) {
            previousSampleValid = false;
            continue;
        }

        if (previousSampleValid && previousGap <= 0.0 && gap > 0.0) {
            lowT = previousT;
            highT = currentT;
            found = true;
            break;
        }
        previousT = currentT;
        previousGap = gap;
        previousSampleValid = true;
    }
    if (!found) return false;

    // Mantem lowT na frente da geometria e highT atras dela.
    for (int i = 0; i < SsrRefineSteps; ++i) {
        float midT = (lowT + highT) * 0.5;
        vec3 marched = worldStart + worldDirection * midT;
        vec2 uv;
        float gap;
        vec3 scenePosition;
        if (!sampleRayGap(marched, uv, gap, scenePosition)) {
            lowT = midT;
            continue;
        }
        if (gap > 0.0) {
            highT = midT;
        } else {
            lowT = midT;
        }
    }

    vec3 marched = worldStart + worldDirection * highT;
    vec2 hitUv;
    float finalGap;
    vec3 hitPosition;
    if (!sampleRayGap(marched, hitUv, finalGap, hitPosition)) return false;

    // Tolerancia pequena e proporcional a distancia. Os antigos 5% do
    // alcance chegavam a UM METRO e transformavam qualquer silhueta proxima
    // num acerto; aqui a espessura representa apenas a incerteza do depth.
    float hitDistance = distance(hitPosition, scene.cameraPosition.xyz);
    float thickness = clamp(hitDistance * 0.003, 0.025, 0.18);
    if (finalGap < -0.002 || finalGap > thickness
        || distance(hitPosition, worldStart) < minDistance * 1.5) {
        return false;
    }

    vec2 edgeDistance = min(hitUv, vec2(1.0) - hitUv);
    float edgeFade = smoothstep(0.0, 0.08, min(edgeDistance.x, edgeDistance.y));
    vec2 fullResolution = vec2(textureSize(depthMap, 0));
    float separationPixels = length((hitUv - sourceUv) * fullResolution);
    float selfIntersectionFade = smoothstep(2.0, 8.0, separationPixels);
    float distanceFade = 1.0 - smoothstep(
        maxDistance * 0.70, maxDistance, highT);
    hitConfidence = edgeFade * selfIntersectionFade * distanceFade;
    if (hitConfidence <= 0.001) return false;

    hitColor = texture(hdrColorMap, hitUv).rgb;
    return true;
}

void main() {
    float depth = texture(depthMap, texCoord).r;
    if (depth <= 0.0001) {
        outColor = vec4(0.0); // ceu - nada a refletir a partir daqui
        return;
    }
    vec3 reflectance = texture(reflectanceMap, texCoord).rgb;
    if (max(reflectance.r, max(reflectance.g, reflectance.b)) < 0.02) {
        // Reflectancia baixa demais pra valer o traçado - a maior parte da
        // cena (paredes, chao nao-metalico) cai aqui, entao isto e o que
        // mantem o custo medio deste passe baixo.
        outColor = vec4(0.0);
        return;
    }

    vec4 normalRoughnessSample = texture(normalRoughnessMap, texCoord);
    vec3 normal = octDecode(normalRoughnessSample.rg);
    float roughness = normalRoughnessSample.b;

    vec3 worldPosition = reconstructWorldPosition(texCoord, depth);
    vec3 viewDirection = normalize(scene.cameraPosition.xyz - worldPosition);
    vec3 reflectionDirection = reflect(-viewDirection, normal);
    vec3 environment = reflectionEnvironmentColor(reflectionDirection);

    vec3 hitColor = vec3(0.0);
    float hitConfidence = 0.0;
    // Materiais muito rugosos precisam do ambiente especular, mas nao de um
    // ray march: sem uma piramide de mips de cor, SSR neles so produz ruido
    // caro. Superficies lisas (espelho/cromo) recebem o tracado completo.
    float traceWeight = 1.0 - smoothstep(0.28, 0.72, roughness);
    if (traceWeight > 0.001) {
        vec3 facingNormal = dot(normal, viewDirection) >= 0.0
            ? normal : -normal;
        vec3 rayStart = worldPosition + facingNormal * 0.012
            + reflectionDirection * 0.02;
        traceReflection(rayStart, reflectionDirection, texCoord,
            hitColor, hitConfidence);
    }
    float currentConfidence = hitConfidence * traceWeight;
    // Baixa confianca mistura de volta ao ambiente em vez de escurecer ate
    // preto. Isso elimina as manchas pretas nas bordas e nas desoclusoes.
    vec3 traced = mix(environment, hitColor, currentConfidence);

    // Mesmo "borrao" barato que o fallback analitico sempre usou no lugar
    // de um ambiente pre-filtrado de verdade: achata a cor tracada em
    // direcao a sua propria media de brilho conforme a superficie fica mais
    // rugosa.
    float averageBrightness = dot(traced, vec3(0.299, 0.587, 0.114));
    vec3 currentRadiance = mix(traced, vec3(averageBrightness),
        clamp(roughness * roughness * 1.1, 0.0, 0.9));

    // settings.z (mesma flag que o resolve de TAA usa, ver
    // taa_resolve.frag) so fica ligada quando existe historico de MEIA
    // resolucao valido da MESMA cena/dimensoes - primeiro quadro, resize e
    // previews sempre caem aqui, semeando historico limpo.
    if (scene.settings.z < 0.5) {
        outColor = vec4(currentRadiance, currentConfidence);
        return;
    }

    // Reprojeta com o MESMO vetor de movimento da superficie refletora (ver
    // scene3d_mesh.vert/frag) - uma aproximacao (ele descreve como a
    // superficie se moveu, nao como o conteudo refletido se moveu), mas bem
    // precedentada pra reflexos de tela em superficies majoritariamente
    // planas/estaticas (espelho, metal), e muito mais barata que rastrear
    // um vetor de movimento dedicado pro ponto de acerto do raio.
    vec2 motionVector = texture(motionVectorMap, texCoord).rg;
    vec2 previousTexCoord = texCoord - motionVector;
    vec2 texelSize = 1.0 / vec2(textureSize(historyMap, 0));
    vec2 border = texelSize * 0.5;
    bool historyValid = all(greaterThanEqual(previousTexCoord, border))
        && all(lessThanEqual(previousTexCoord, vec2(1.0) - border));
    if (!historyValid) {
        outColor = vec4(currentRadiance, currentConfidence);
        return;
    }

    vec4 historySample = texture(historyMap, previousTexCoord);
    vec3 historyRadiance = historySample.rgb;
    float velocityPixels = length(motionVector / texelSize);
    float motionReactivity = smoothstep(0.25, 8.0, velocityPixels);
    float currentLuminance = dot(currentRadiance,
        vec3(0.299, 0.587, 0.114));
    float historyLuminance = dot(historyRadiance,
        vec3(0.299, 0.587, 0.114));
    float luminanceDelta = abs(currentLuminance - historyLuminance)
        / max(max(currentLuminance, historyLuminance), 0.08);
    float confidenceDelta = abs(currentConfidence - historySample.a);
    float reactivity = max(motionReactivity,
        max(smoothstep(0.08, 0.55, luminanceDelta),
            smoothstep(0.08, 0.45, confidenceDelta)));

    // Clamp local ao valor atual: SSR muda de alvo em desoclusoes, e o
    // vetor da superficie refletora nao descreve esse movimento. Sem este
    // limite, caixas/arma continuavam "impressas" no espelho por segundos.
    vec3 historyEnvelope = max(vec3(0.10),
        currentRadiance * 0.45 + vec3(0.04));
    historyRadiance = clamp(historyRadiance,
        max(vec3(0.0), currentRadiance - historyEnvelope),
        currentRadiance + historyEnvelope);
    float historyWeight = mix(0.52, 0.08, reactivity);
    vec3 resolved = mix(currentRadiance, historyRadiance, historyWeight);
    float resolvedConfidence = mix(currentConfidence, historySample.a,
        historyWeight);
    outColor = vec4(max(resolved, vec3(0.0)),
        clamp(resolvedConfidence, 0.0, 1.0));
}
