#version 450

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
} scene;

// Um mapa por cascata (ver ShadowCascadeCount em Scene3D.hpp - 4 hardcoded
// aqui tambem, GLSL nao compartilha header com o C++ neste projeto).
layout(set = 0, binding = 1) uniform sampler2DShadow shadowMap[4];
// Mesmas imagens do array acima, SEM comparacao - sampler2DShadow so
// devolve o resultado 0/1 de um teste de profundidade, nunca o valor de
// profundidade em si; PCSS (ver shadowVisibility) precisa ler profundidade
// crua pra buscar bloqueadores antes de decidir o tamanho da penumbra.
layout(set = 0, binding = 3) uniform sampler2D shadowMapRaw[4];

// Uma entrada do buffer de luzes generico (ver Scene3DFrame::lights e
// packSceneLights) - mesmo layout para os 3 tipos (direcional, ponto, spot),
// cada um usando so os campos que fazem sentido pro seu tipo (ver
// accumulateLighting abaixo e o comentario de tipos-default em
// SceneLightPacking.cpp).
struct GpuLight {
    vec4 positionRange;        // xyz=posicao, w=alcance (spot/ponto, nao usado por direcional)
    vec4 directionOuterCosine; // xyz=direcao, w=cosseno do angulo externo do cone (so spot)
    vec4 colorIntensity;       // rgb=cor, a=intensidade
    vec4 parameters;           // x=cosseno interno do cone (so spot), y=tipo (0=dir,1=ponto,2=spot), z=projeta sombra, w reservado
};

layout(std430, set = 0, binding = 2) readonly buffer SceneLights {
    GpuLight lights[];
} sceneLights;

const int LightTypeDirectional = 0;
const int LightTypePoint = 1;
const int LightTypeSpot = 2;
const float PI = 3.14159265359;

// Per-material albedo map (see VulkanDevice::createTexture2D / pushMesh) -
// a 1x1 white texture when the mesh has none, so solid-colored procedural
// meshes and textured imported ones share this exact same shader unchanged.
layout(set = 1, binding = 0) uniform sampler2D albedoMap;
layout(set = 2, binding = 0) uniform sampler2D metallicRoughnessMap;
// Cena vista pela camera espelhada. So objectParameters.y != 0 amostra este
// binding; os demais materiais usam somente o ambiente analitico barato.
layout(set = 3, binding = 0) uniform sampler2D planarReflectionMap;

layout(location = 0) in vec3 worldPosition;
layout(location = 1) in vec3 worldNormal;
layout(location = 2) in vec2 vertexUv;
layout(location = 3) in vec3 vertexColor;
// Local 4 nao e mais usado (era lightClipPosition, calculado no vertex
// shader - agora recalculado aqui mesmo dentro de shadowVisibility, ver
// mais abaixo, porque a cascata so pode ser escolhida por pixel).
layout(location = 5) flat in ivec4 objectParameters;
layout(location = 6) flat in vec2 objectMetallicRoughness;
layout(location = 7) in vec4 currentClipPosition;
layout(location = 8) in vec4 previousMotionClipPosition;
layout(location = 9) flat in vec3 objectTint;
layout(location = 0) out vec4 outColor;
// Vetor de movimento em espaco de tela (UV, [0,1]) entre a posicao deste
// pixel na grade estavel e onde o MESMO ponto do objeto estava projetado no
// quadro anterior. Os dois clips excluem o jitter; incluir a diferenca entre
// amostras Halton aqui faria o historico deslizar e a imagem tremer. O valor e
// consumido pelo resolve de TAA no lugar da reconstrucao de
// posicao via profundidade (ver taa_resolve.frag e o comentario em
// scene3d_mesh.vert sobre por que isto e mais robusto que a tecnica
// anterior).
layout(location = 1) out vec2 outMotionVector;

// Mesma convencao uv<->clip usada em tonemap.vert (texCoord = clipPosition *
// 0.5 + 0.5, sem flip em Y - gl_Position.y ja saiu invertido la no vertex
// shader, ver main() abaixo, entao o NDC aqui ja esta na convencao Y-pra-
// baixo do Vulkan, igual a imagem). SEM esse flip extra: uma versao anterior
// desta funcao espelhava o eixo Y de novo (bug herdado de quando ela imitava
// a reconstrucao por profundidade que este arquivo usava antes dos vetores
// de movimento reais, ja removida de taa_resolve.frag) - isso invertia o
// sinal do componente Y do vetor de movimento, fazendo o resolve do TAA
// reprojetar o historico sempre pro lado errado do eixo Y a cada quadro
// (causa raiz do "tremer" mesmo com a camera parada).
vec2 clipPositionToUv(vec4 clipPosition) {
    vec2 ndc = clipPosition.xy / clipPosition.w;
    return ndc * 0.5 + 0.5;
}

// O attachment RG16F de movimento já existe nos dois modos. Valores reais
// de deslocamento vivem em [-1,1]; somar um sentinela fora dessa faixa
// transporta também a classe "prop físico com leitura prioritária" sem
// criar outro MRT, outra imagem ou mais largura de banda. O TAA remove o
// sentinela antes de reprojetar (taa_resolve.frag).
vec2 encodedMotionVector() {
    vec2 motion = clipPositionToUv(currentClipPosition)
        - clipPositionToUv(previousMotionClipPosition);
    if ((objectParameters.x & 4) != 0) motion.x += 4.0;
    return motion;
}

// Disco de Poisson deterministico: distribui as amostras sem os eixos e
// blocos visiveis de um kernel quadrado 3x3. O padrao permanece fixo em
// espaco da sombra, portanto nao introduz ruido temporal durante movimento.
const vec2 ShadowPoissonDisk[16] = vec2[](
    // As quatro primeiras cobrem os quatro quadrantes. Assim o preset de
    // desempenho continua equilibrado espacialmente com apenas 4+4 taps.
    vec2(-0.94201624, -0.39906216), vec2(-0.91588581,  0.45771432),
    vec2( 0.97484398,  0.75648379), vec2( 0.44323325, -0.97511554),
    vec2( 0.34495938,  0.29387760), vec2(-0.26496911, -0.41893023),
    vec2( 0.79197514,  0.19090188), vec2(-0.24188840,  0.99706507),
    vec2( 0.94558609, -0.76890725), vec2(-0.09418410, -0.92938870),
    vec2(-0.81544232, -0.87912464), vec2(-0.38277543,  0.27676845),
    vec2( 0.53742981, -0.47373420), vec2(-0.81409955,  0.91437590),
    vec2( 0.19984126,  0.78641367), vec2( 0.14383161, -0.14100790)
);

// Os samplers formam um array de descritores. Indexa-los dinamicamente
// exigiria habilitar uma feature opcional de descriptor indexing no device.
// Os acessos abaixo mantem indices constantes em cada ramo, funcionando em
// qualquer GPU Vulkan 1.4 aceita pela engine sem relaxar seu contrato.
ivec2 shadowMapExtent(int cascade) {
    if (cascade == 0) return textureSize(shadowMap[0], 0);
    if (cascade == 1) return textureSize(shadowMap[1], 0);
    if (cascade == 2) return textureSize(shadowMap[2], 0);
    return textureSize(shadowMap[3], 0);
}

float sampleRawShadow(int cascade, vec2 uv) {
    if (cascade == 0) return texture(shadowMapRaw[0], uv).r;
    if (cascade == 1) return texture(shadowMapRaw[1], uv).r;
    if (cascade == 2) return texture(shadowMapRaw[2], uv).r;
    return texture(shadowMapRaw[3], uv).r;
}

float sampleComparedShadow(int cascade, vec3 uvDepth) {
    if (cascade == 0) return texture(shadowMap[0], uvDepth);
    if (cascade == 1) return texture(shadowMap[1], uvDepth);
    if (cascade == 2) return texture(shadowMap[2], uvDepth);
    return texture(shadowMap[3], uvDepth);
}

// Avalia uma unica cascata. Separar esta operacao permite amostrar duas
// cascatas na faixa de transicao e elimina a troca brusca de resolucao que
// aparecia como uma linha ou uma sombra que mudava ao caminhar.
float sampleShadowCascade(int cascade, vec3 normal, vec3 lightDirection) {
    float nDotL = clamp(dot(normal, lightDirection), 0.0, 1.0);
    float texelWorldSize = max(scene.cascadeTexelWorldSizes[cascade], 0.0001);
    float depthRangeMeters = max(scene.cascadeDepthRanges[cascade], 0.001);

    // O normal-offset e expresso em texels reais da cascata. Ele cresce em
    // angulos rasantes, onde a quantizacao do mapa causa acne, mas permanece
    // pequeno em superficies de frente para a luz para nao separar a sombra
    // dos pes dos objetos (peter-panning).
    float normalOffsetTexels = mix(1.10, 0.35, nDotL);
    vec3 offsetWorldPosition = worldPosition
        + normal * (texelWorldSize * normalOffsetTexels);
    vec4 lightClipPosition = scene.cascadeViewProjections[cascade]
        * vec4(offsetWorldPosition, 1.0);
    lightClipPosition.y = -lightClipPosition.y;
    if (lightClipPosition.w <= 0.0) return 1.0;

    vec3 projected = lightClipPosition.xyz / lightClipPosition.w;
    vec2 uv = projected.xy * 0.5 + 0.5;
    if (projected.z <= 0.0 || projected.z >= 1.0
        || any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
        return 1.0;
    }

    // A projecao ortografica possui profundidade linear. Converter um bias
    // em metros pela extensao near/far torna o resultado independente do
    // tamanho da cascata e da folga reservada para casters fora da camera.
    float slope = 1.0 - nDotL;
    float depthBiasMeters = texelWorldSize * (0.12 + slope * 0.55);
    float referenceDepth = projected.z - depthBiasMeters / depthRangeMeters;
    vec2 texel = 1.0 / vec2(shadowMapExtent(cascade));

    // PCSS, etapa 1: estima a profundidade media dos bloqueadores numa
    // vizinhanca compacta. O sampler cru usa NEAREST para nao inventar
    // profundidades interpoladas entre um caster e o fundo do mapa.
    float blockerDepthSum = 0.0;
    float blockerCount = 0.0;
    const float BlockerSearchRadiusTexels = 3.0;
    int shadowSampleCount = clamp(int(scene.settings.x + 0.5), 4, 16);
    for (int sampleIndex = 0; sampleIndex < 16; ++sampleIndex) {
        if (sampleIndex >= shadowSampleCount) break;
        float sampleDepth = sampleRawShadow(cascade,
            uv + ShadowPoissonDisk[sampleIndex]
                * texel * BlockerSearchRadiusTexels);
        if (sampleDepth < referenceDepth) {
            blockerDepthSum += sampleDepth;
            blockerCount += 1.0;
        }
    }
    if (blockerCount < 0.5) return 1.0;
    float averageBlockerDepth = blockerDepthSum / blockerCount;

    // PCSS, etapa 2: calcula a penumbra no mundo. A versao anterior dividia
    // duas profundidades normalizadas; isso fazia a sombra mudar de maciez
    // quando o fitting da cascata mudava. Para uma luz direcional, a abertura
    // angular aparente do Sol determina o crescimento fisico da penumbra.
    const float SunAngularRadiusRadians = 0.00465;
    float blockerSeparationMeters = max(0.0,
        referenceDepth - averageBlockerDepth) * depthRangeMeters;
    float penumbraWorldMeters = blockerSeparationMeters
        * tan(SunAngularRadiusRadians);
    float penumbraTexels = clamp(
        0.75 + penumbraWorldMeters / texelWorldSize, 0.75, 5.0);

    // PCSS, etapa 3: PCF com o raio encontrado acima. As 16 amostras
    // preservam silhuetas finas muito melhor que o antigo 3x3 esparso.
    float visibility = 0.0;
    for (int sampleIndex = 0; sampleIndex < 16; ++sampleIndex) {
        if (sampleIndex >= shadowSampleCount) break;
        vec2 sampleUv = uv + ShadowPoissonDisk[sampleIndex]
            * texel * penumbraTexels;
        visibility += sampleComparedShadow(cascade,
            vec3(sampleUv, referenceDepth));
    }
    return visibility / float(shadowSampleCount);
}

// viewSpaceDistance e a distancia ao longo do eixo da camera: o mesmo
// criterio usado por computeCascadeSplits. currentClipPosition.w vem da VP
// sem jitter, portanto a escolha da cascata tambem nao oscila com o TAA.
float shadowVisibility(vec3 normal, vec3 lightDirection, float viewSpaceDistance) {
    if (scene.settings.x < 0.5) return 1.0;

    int cascade = 3;
    if (viewSpaceDistance < scene.cascadeSplits.x) cascade = 0;
    else if (viewSpaceDistance < scene.cascadeSplits.y) cascade = 1;
    else if (viewSpaceDistance < scene.cascadeSplits.z) cascade = 2;

    float visibility = sampleShadowCascade(cascade, normal, lightDirection);

    // Mistura 12% do final de cada fatia com a cascata seguinte. O fitting
    // na CPU inclui a mesma sobreposicao, logo ambas possuem geometria
    // valida durante toda a transicao.
    const float CascadeBlendFraction = 0.12;
    if (cascade < 3) {
        float cascadeNear = cascade == 0
            ? 0.08 : scene.cascadeSplits[cascade - 1];
        float cascadeFar = scene.cascadeSplits[cascade];
        float blendStart = cascadeFar
            - (cascadeFar - cascadeNear) * CascadeBlendFraction;
        float blend = smoothstep(blendStart, cascadeFar, viewSpaceDistance);
        if (blend > 0.0) {
            float nextVisibility = sampleShadowCascade(
                cascade + 1, normal, lightDirection);
            visibility = mix(visibility, nextVisibility, blend);
        }
    } else {
        // Sombras deixam de ser calculadas apenas depois de 450 m. Uma
        // pequena faixa final evita o corte repentino no limite e coincide
        // com a distancia em que a neblina ja domina o contraste.
        float fadeStart = mix(scene.cascadeSplits.z,
            scene.cascadeSplits.w, 0.92);
        visibility = mix(visibility, 1.0,
            smoothstep(fadeStart, scene.cascadeSplits.w, viewSpaceDistance));
    }
    return visibility;
}

// Termo de distribuicao normal GGX/Trowbridge-Reitz - a fracao dos
// microfacetos alinhados exatamente com o vetor-metade H. alpha=roughness^2
// e o remapeamento padrao (Disney/Karis) que faz o slider de rugosidade se
// comportar de forma perceptualmente linear em vez de concentrar toda a
// variação nos primeiros valores.
float distributionGGX(vec3 normal, vec3 halfVector, float roughness) {
    float alpha = roughness * roughness;
    float alpha2 = alpha * alpha;
    float nDotH = max(dot(normal, halfVector), 0.0);
    float denom = nDotH * nDotH * (alpha2 - 1.0) + 1.0;
    return alpha2 / max(PI * denom * denom, 0.0001);
}

// Termo de geometria Smith com a aproximacao Schlick-GGX (k para luz direta,
// nao a variante de IBL) - modela a auto-sombra e o auto-mascaramento entre
// microfacetos, o motivo de superficies rugosas escurecerem nas bordas em
// vez de so espalharem o brilho especular igualmente em todo lugar.
float geometrySchlickGGX(float cosTheta, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return cosTheta / (cosTheta * (1.0 - k) + k);
}

float geometrySmith(float nDotV, float nDotL, float roughness) {
    return geometrySchlickGGX(nDotV, roughness) * geometrySchlickGGX(nDotL, roughness);
}

// Fresnel-Schlick: a fracao de luz refletida (nao absorvida/refratada) sobe
// para perto de 100% em angulo raso em qualquer material, mesmo os mais
// foscos - e o motivo do chao parecer mais espelhado olhando quase de raspao
// do que olhando de cima.
vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (vec3(1.0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Variante com rugosidade (Karis 2013) usada so no termo ambiente/especular
// indireto: sem isso, o "brilho de borda" do Fresnel apareceria com a mesma
// forca numa esfera de gesso rugosa e numa bola de bilhar polida, o que nao
// acontece na realidade - uma superficie rugosa nao tem a borda brilhante
// caracteristica de vidro/metal liso mesmo em angulo raso.
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    vec3 maxReflectance = max(vec3(1.0 - roughness), F0);
    return F0 + (maxReflectance - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Reflexão ambiente analítica de custo constante. Não consulta profundidade,
// histórico nem pixels vizinhos: materiais metálicos apenas avaliam uma
// aproximação barata do céu/horizonte/chão na direção refletida.
vec3 reflectionEnvironmentColor(vec3 direction) {
    int lightCount = int(scene.settings.y);
    vec3 sunDirection = lightCount > 0
        ? normalize(sceneLights.lights[0].directionOuterCosine.xyz)
        : vec3(0.0, 0.0, 1.0);
    vec3 moonDirection = lightCount > 1
        ? normalize(sceneLights.lights[1].directionOuterCosine.xyz)
        : -sunDirection;
    float dayFactor = smoothstep(-0.10, 0.10, sunDirection.z);
    float environmentEnergy = clamp(scene.settings.w / 0.31, 0.015, 1.0);
    float overcast = smoothstep(0.55, 0.98,
        clamp(scene.skySettings.z, 0.0, 1.0));
    vec3 groundColor = mix(vec3(0.006, 0.008, 0.014),
        vec3(0.16, 0.15, 0.14), dayFactor);
    vec3 horizon = mix(vec3(0.015, 0.024, 0.055),
        vec3(0.58, 0.72, 0.86), dayFactor);
    vec3 zenith = mix(vec3(0.0015, 0.004, 0.018),
        vec3(0.03, 0.12, 0.32), dayFactor);
    vec3 environment;
    if (direction.z >= 0.0) {
        environment = mix(horizon, zenith, pow(direction.z, 0.55));
    } else {
        float depth = -direction.z;
        environment = mix(horizon, groundColor, pow(depth, 0.4));
        environment = mix(environment, groundColor * 0.55,
            pow(depth, 2.5));
    }
    environment *= mix(0.62, 1.0, sqrt(environmentEnergy));
    float environmentLuma = dot(environment,
        vec3(0.299, 0.587, 0.114));
    environment = mix(environment, vec3(environmentLuma),
        overcast * 0.42);

    float sunDot = max(dot(direction, sunDirection), 0.0);
    float sunGlow = pow(sunDot, 220.0);
    float sunIntensity = lightCount > 0
        ? sceneLights.lights[0].colorIntensity.a : 0.0;
    environment += vec3(1.0, 0.78, 0.42) * sunGlow
        * sunIntensity * 2.0;
    float sunCore = smoothstep(0.99900, 0.99985, sunDot);
    environment = mix(environment, vec3(1.0, 0.97, 0.88),
        sunCore * clamp(sunIntensity, 0.0, 1.0));
    float moonDot = max(dot(direction, moonDirection), 0.0);
    float moonIntensity = lightCount > 1
        ? sceneLights.lights[1].colorIntensity.a : 0.0;
    environment += vec3(0.24, 0.34, 0.62)
        * pow(moonDot, 260.0) * moonIntensity * 2.5;
    return environment;
}

// Acumula a contribuicao de todas as luzes da cena via Cook-Torrance/GGX
// (D*G*F especular + difusa Lambertiana com conservacao de energia). So a
// luz de indice 0 pode projetar sombra: o backend so mantem Cascaded Shadow
// Maps pra UMA luz (ver Scene3DFrame::cascadeViewProjections), que so faz
// sentido pareado com essa convencao (ver comentario em Scene3DFrame) - as
// demais luzes nunca ficam em sombra, exatamente como a lanterna da physgun
// nunca ficou no modelo anterior de campos fixos. Isto ilumina so a luz
// direta das luzes analiticas do SSBO - ambiente/indireta e tratada a parte
// em main() (sem IBL de verdade, ver ROADMAP Marco 4).
vec3 accumulateLighting(vec3 normal, vec3 viewDirection, vec3 albedo,
        float metallic, float roughness, vec3 F0, float specularWeight,
        out float primaryShadowVisibility,
        out vec3 primaryRadiance) {
    vec3 result = vec3(0.0);
    primaryShadowVisibility = 1.0;
    primaryRadiance = vec3(0.0);
    float nDotV = max(dot(normal, viewDirection), 0.0001);
    int lightCount = int(scene.settings.y);
    for (int i = 0; i < lightCount; ++i) {
        GpuLight light = sceneLights.lights[i];
        int type = int(light.parameters.y);

        vec3 lightDirection;
        vec3 radiance;
        if (type == LightTypeDirectional) {
            lightDirection = normalize(light.directionOuterCosine.xyz);
            radiance = light.colorIntensity.rgb * light.colorIntensity.a;
            if (i == 0) primaryRadiance = radiance;
        } else if (type == LightTypePoint) {
            vec3 toLight = light.positionRange.xyz - worldPosition;
            float distanceSquared = max(dot(toLight, toLight), 0.01);
            lightDirection = normalize(toLight);
            float attenuation = 1.9 / (1.0 + 0.020 * distanceSquared);
            radiance = light.colorIntensity.rgb * light.colorIntensity.a * attenuation;
        } else { // Spot
            vec3 spotOffset = worldPosition - light.positionRange.xyz;
            float spotDistance = length(spotOffset);
            float spotRange = light.positionRange.w;
            if (spotRange <= 0.0 || spotDistance >= spotRange) continue;
            vec3 fromSpot = spotOffset / max(spotDistance, 0.001);
            float cone = smoothstep(light.directionOuterCosine.w,
                light.parameters.x,
                dot(normalize(light.directionOuterCosine.xyz), fromSpot));
            float rangeFalloff = 1.0 - spotDistance / spotRange;
            rangeFalloff *= rangeFalloff;
            lightDirection = -fromSpot;
            radiance = light.colorIntensity.rgb * light.colorIntensity.a
                * cone * rangeFalloff
                / (1.0 + spotDistance * spotDistance * 0.035);
        }

        bool castsShadow = i == 0 && light.parameters.z > 0.5;
        // currentClipPosition.w (varying global, ver o topo do arquivo) e
        // reaproveitado aqui como a distancia de camera pra escolher a
        // cascata - o mesmo dado que os vetores de movimento ja usam, sem
        // custo nem varying novo (ver o comentario de shadowVisibility).
        float visibility = castsShadow
            ? shadowVisibility(normal, lightDirection, currentClipPosition.w)
            : 1.0;
        if (castsShadow) {
            // Uma única consulta de sombra serve também como aproximação
            // barata da abertura para o céu no Marco 1. O Marco 2 troca
            // esta aproximação por probes de visibilidade multidirecionais.
            primaryShadowVisibility = visibility;
        }

        float nDotL = max(dot(normal, lightDirection), 0.0);
        if (nDotL <= 0.0) continue;
        if (visibility <= 0.0) continue;

        vec3 halfVector = normalize(viewDirection + lightDirection);
        float distribution = distributionGGX(normal, halfVector, roughness);
        float geometry = geometrySmith(nDotV, nDotL, roughness);
        vec3 fresnel = fresnelSchlick(max(dot(halfVector, viewDirection), 0.0), F0);

        vec3 weightedFresnel = fresnel * specularWeight;
        vec3 specular = (distribution * geometry * weightedFresnel)
            / max(4.0 * nDotV * nDotL, 0.0001);
        // Metais nao tem componente difusa (toda a luz nao absorvida sai
        // como reflexo especular, nao espalhada) - kd zera a difusa deles
        // automaticamente, alem de subtrair a fracao ja contabilizada pelo
        // Fresnel especular (conservacao de energia: difusa+especular nunca
        // passa de 1.0 da luz recebida).
        vec3 kd = (vec3(1.0) - weightedFresnel) * (1.0 - metallic);
        vec3 diffuse = kd * albedo / PI;

        result += (diffuse + specular) * radiance * nDotL * visibility;
    }
    return result;
}

float atmosphericFogFactor(float cameraDistance, float worldHeight) {
    float startDistance = max(scene.fogSettings.w, 0.0);
    float endDistance = max(scene.fogColor.a, startDistance + 1.0);
    float distancePastStart = max(0.0, cameraDistance - startDistance);
    float heightVisibility = smoothstep(-2.0, 5.0,
        worldHeight + distancePastStart * scene.fogSettings.y);
    float exponentialFog =
        (1.0 - exp(-distancePastStart * scene.fogSettings.x))
        * heightVisibility;

    // A parte final não é assintótica: converge exatamente para a cor do
    // céu e forma uma barreira visual antes do far plane. Isso elimina
    // silhuetas residuais no horizonte mesmo em objetos muito altos.
    float terminalStart = mix(startDistance, endDistance, 0.78);
    float terminalFog = smoothstep(
        terminalStart, endDistance, cameraDistance);
    return clamp(max(
        min(exponentialFog, scene.fogSettings.z),
        terminalFog), 0.0, 1.0);
}

void main() {
    float distanceFromCamera =
        length(scene.cameraPosition.xyz - worldPosition);
    if (scene.skySettings.x > 0.5
            && distanceFromCamera >= scene.fogColor.a) {
        // Não lê texturas nem executa PBR em fragmentos que a barreira
        // atmosférica substituiria integralmente.
        outColor = vec4(scene.fogColor.rgb, 1.0);
        outMotionVector = vec2(0.0);
        return;
    }

    vec3 normal = normalize(worldNormal);
    if ((objectParameters.x & 16) != 0) {
        vec3 faceNormal = normalize(cross(dFdx(worldPosition), dFdy(worldPosition)));
        normal = dot(faceNormal, normal) < 0.0 ? -faceNormal : faceNormal;
    }
    vec3 viewDirection = normalize(scene.cameraPosition.xyz - worldPosition);
    vec2 materialUv = vertexUv;
    vec3 albedo = texture(albedoMap, materialUv).rgb * vertexColor * objectTint;
    if (objectParameters.z != 0) {
        albedo = mix(albedo, vec3(1.0, 0.62, 0.08), 0.24);
    }
    vec4 metallicRoughnessSample = texture(metallicRoughnessMap, materialUv);
    float metallic = clamp(objectMetallicRoughness.x
        * metallicRoughnessSample.b, 0.0, 1.0);
    float roughness = clamp(objectMetallicRoughness.y
        * metallicRoughnessSample.g, 0.045, 1.0);
    bool matteSurface = (objectParameters.x & 8) != 0;
    if (matteSurface) {
        // Folhas e fibras espalham a luz em muitas direcoes. Forcar a
        // resposta microfacetada para o extremo rugoso impede que o Sol
        // desenhe uma mancha de verniz sobre o gramado.
        metallic = 0.0;
        roughness = max(roughness, 0.98);
    }
    float specularWeight = matteSurface ? 0.025 : 1.0;
    // Dieletricos (nao-metal) refletem uns 4% da luz de volta mesmo olhando
    // de frente (vidro, plastico, pele...) - metais usam o proprio albedo
    // como reflectancia de base, o que tambem tinge o reflexo com a cor do
    // metal (uma bola de ouro reflete dourado, uma de cromo reflete neutro).
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    if (objectParameters.y != 0) {
        vec2 reflectionUv = clipPositionToUv(currentClipPosition);
        vec3 planarRadiance = texture(planarReflectionMap,
            reflectionUv).rgb;
        outColor = vec4(planarRadiance, 1.0);
        outMotionVector = encodedMotionVector();
        return;
    }

    float primaryShadowVisibility = 1.0;
    vec3 primaryRadiance = vec3(0.0);
    vec3 direct = accumulateLighting(normal, viewDirection, albedo,
        metallic, roughness, F0, specularWeight, primaryShadowVisibility,
        primaryRadiance);

    // Ambiente/indireta simples. Metais comuns usam somente o ambiente
    // analítico abaixo; o espelho perfeito já retornou pelo passe planar.
    // Isso evita artefatos e custo variável de SSR mantendo resposta a
    // metallic, roughness, Fresnel e à cor do próprio metal.
    // Zero é válido: durante a noite ou numa tempestade o sistema ambiental
    // pode remover completamente a irradiância do céu.
    float ambient = max(scene.settings.w, 0.0);
    float minimumVisibility =
        clamp(scene.environmentSettings.x, 0.0, 1.0);
    float upwardFacing = max(normal.z, 0.0);
    float verticalFacing = 1.0 - abs(normal.z);
    float downwardFacing = max(-normal.z, 0.0);
    // Aproxima o hemisfério realmente visível para cada orientação. Pisos
    // veem quase todo o céu, paredes veem aproximadamente metade e tetos
    // recebem sobretudo luz já rebatida. A sombra solar reduz contraste,
    // mas não apaga a parcela difusa do céu — exatamente o comportamento de
    // uma área atrás de uma caixa num dia claro.
    float skyHemisphere = clamp(
        0.16 + upwardFacing * 0.84 + verticalFacing * 0.38,
        minimumVisibility, 1.0);
    float indirectVisibility = max(minimumVisibility,
        skyHemisphere * mix(0.56, 1.0, primaryShadowVisibility));
    vec3 ambientFresnel = fresnelSchlickRoughness(
        max(dot(normal, viewDirection), 0.0), F0, roughness);
    ambientFresnel *= specularWeight;
    vec3 ambientDiffuse = albedo * ambient * (1.0 - metallic)
        * (vec3(1.0) - ambientFresnel) * indirectVisibility;
    float sunBounceStrength =
        max(scene.environmentSettings.y, 0.0);
    // Primeiro rebote difuso direcional: superfícies verticais e voltadas
    // para baixo enxergam melhor o chão iluminado, mas o piso ainda conserva
    // preenchimento lateral em sombra. Uma pequena parcela permanece em
    // áreas diretamente iluminadas para a transição da penumbra não marcar.
    float groundBounceAccess = clamp(0.46
        + verticalFacing * 0.30 + downwardFacing * 0.42,
        0.0, 1.0);
    float shadowBounce = mix(0.16, 1.0,
        1.0 - primaryShadowVisibility);
    vec3 bouncedSun = primaryRadiance * sunBounceStrength
        * groundBounceAccess * shadowBounce
        * vec3(0.72, 0.66, 0.56);
    vec3 bouncedDiffuse = bouncedSun * albedo
        * (1.0 - metallic) * (vec3(1.0) - ambientFresnel);
    vec3 reflectionDirection =
        reflect(-viewDirection, normal);
    vec3 environment =
        reflectionEnvironmentColor(reflectionDirection);
    float averageBrightness = dot(environment,
        vec3(0.299, 0.587, 0.114));
    vec3 blurredEnvironment = mix(environment,
        vec3(averageBrightness),
        clamp(roughness * roughness * 1.1, 0.0, 0.9));
    vec3 ambientSpecular =
        blurredEnvironment * ambientFresnel * indirectVisibility;

    vec3 shaded = ambientDiffuse + bouncedDiffuse
        + ambientSpecular + direct;

    if (objectParameters.w != 0) {
        float rim = pow(1.0 - clamp(dot(normal, viewDirection), 0.0, 1.0), 7.0);
        shaded += vec3(0.25, 0.55, 1.0) * rim * 3.0;
    }
    if (scene.skySettings.x > 0.5) {
        float fog = atmosphericFogFactor(
            distanceFromCamera, worldPosition.z);
        shaded = mix(shaded, scene.fogColor.rgb, fog);
    }
    outColor = vec4(shaded, 1.0);

    outMotionVector = encodedMotionVector();

}
