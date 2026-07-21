#version 450

// Reaproveita tonemap.vert como estagio de vertice (ver createScene3DResources)
// - o mesmo triangulo cheio de tela, sem nenhum binding/push constant, serve
// pros dois passes.

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;         // jitterizada quando TAA ativo (Fase 6)
    mat4 cameraViewProjectionUnjittered;// VP atual estavel, usada nos motion vectors
    mat4 inverseCameraViewProjection;  // inversa da VP sem jitter (ceu estavel)
    mat4 cascadeViewProjections[4];    // Cascaded Shadow Maps - nao usadas pelo resolve de TAA
    vec4 cascadeSplits;                // idem - so aqui pra bater o layout do UBO
    vec4 cascadeTexelWorldSizes;       // idem - so aqui pra bater o layout do UBO
    vec4 cascadeDepthRanges;           // idem - so aqui pra bater o layout do UBO
    mat4 previousCameraViewProjection; // VP anterior estavel, usada pelos meshes
    vec4 cameraPosition;
    vec4 settings;    // x=sombras, y=luzes, z=historico TAA valido, w=ambiente
    vec4 skySettings; // x=mostrar ceu, y=tempo do ceu, z=cobertura de nuvens, w reservado
    vec4 fogSettings; // x=densidade, y=acoplamento altura-distancia, z=opacidade maxima, w reservado
    vec4 fogColor;    // rgb=cor da neblina, a reservado
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
} scene;

layout(set = 1, binding = 0) uniform sampler2D currentColorMap;
layout(set = 1, binding = 1) uniform sampler2D depthMap;
layout(set = 1, binding = 2) uniform sampler2D motionVectorMap;
layout(set = 1, binding = 3) uniform sampler2D historyMap;

layout(location = 0) in vec2 texCoord;
layout(location = 0) out vec4 outColor;

// YCoCg separa luminancia de crominancia. Fazer o clipping temporal nesse
// espaco evita que a caixa RGB aceite combinacoes de cor inexistentes em
// bordas de alto contraste, uma fonte comum de cintilacao colorida.
vec3 rgbToYCoCg(vec3 color) {
    float co = color.r - color.b;
    float temporary = color.b + co * 0.5;
    float cg = color.g - temporary;
    float y = temporary + cg * 0.5;
    return vec3(y, co, cg);
}

vec3 yCoCgToRgb(vec3 color) {
    float temporary = color.x - color.z * 0.5;
    float green = color.z + temporary;
    float blue = temporary - color.y * 0.5;
    float red = blue + color.y;
    return vec3(red, green, blue);
}

void main() {
    float currentDepth = texture(depthMap, texCoord).r;
    vec3 currentColor = texture(currentColorMap, texCoord).rgb;

    // settings.z e definido pelo backend somente quando existe um quadro
    // anterior da MESMA cena, dimensoes e sequencia temporal. Primeiro frame,
    // resize, camera cut e previews entram aqui e semeiam historico limpo.
    if (scene.settings.z < 0.5) {
        outColor = vec4(currentColor, 1.0);
        return;
    }

    // O ceu (fundo) nao escreve profundidade no pre-pass - fica no valor de
    // clear. Com profundidade INVERTIDA (perto=1, longe=0, ver
    // Mat4::perspective) o clear agora e 0.0, nao 1.0. O ceu tambem nao
    // escreve um vetor de movimento util (sempre zero, ver
    // scene3d_sky.frag) - ele ja e procedural e nao serrilha por si so,
    // entao so passa a cor atual direto.
    if (currentDepth <= 0.0001) {
        outColor = vec4(currentColor, 1.0);
        return;
    }

    // Vetor de movimento por pixel (ver scene3d_mesh.vert/frag) - escrito
    // no proprio passe de geometria a partir da transformacao de CADA
    // objeto (camera E o objeto, quando ele se move), nao reconstruido
    // aqui a partir da profundidade. Isso substitui a tecnica original da
    // Fase 6 (reconstruir posicao no mundo via profundidade + reprojetar
    // so pela camera): aquela abordagem (a) so sabia reprojetar movimento
    // de CAMERA, deixando objetos em movimento (um prop lancado pela
    // physgun) com rastro/ghosting, e (b) sofria de precisao ruim em
    // distancias medias/longas (o "ida e volta" por profundidade amplifica
    // qualquer variacao minima causada pelo jitter sub-pixel), causando
    // cintilacao visivel mesmo parado - a causa raiz de um bug relatado
    // ("a tela treme"), que Reversed-Z sozinho (ver Mat4::perspective)
    // reduziu mas nao eliminou. Vetores de movimento reais sao a tecnica
    // usada por engines AAA (Unreal, Frostbite, id Tech) exatamente por
    // nao ter nenhum desses dois problemas.
    vec2 motionVector = texture(motionVectorMap, texCoord).rg;
    vec2 previousTexCoord = texCoord - motionVector;

    vec2 texelSize = 1.0 / vec2(textureSize(currentColorMap, 0));
    vec2 historyBorder = texelSize * 0.5;
    bool historyValid = all(greaterThanEqual(previousTexCoord, historyBorder))
        && all(lessThanEqual(previousTexCoord, vec2(1.0) - historyBorder));
    if (!historyValid) {
        outColor = vec4(currentColor, 1.0);
        return;
    }

    // Clipping por variancia (Karis 2014, "High Quality Temporal
    // Supersampling") em vez de um clamp min/max bruto da vizinhanca 3x3: em
    // regiao de alto contraste (ex.: o chao xadrez), um clamp min/max puro
    // forma uma caixa larga demais - o historico reprojetado (que deveria
    // estar convergido/estavel) fica livre pra saltar pra qualquer
    // combinacao de cor dentro dessa caixa larga a cada quadro. Usar media +
    // desvio padrao da vizinhanca forma uma caixa mais fiel a distribuicao
    // real de cor local - ainda rejeita historico genuinamente invalido
    // (ghosting/desoclusao), mas para de descartar historico bom so porque
    // ele calha de estar perto da borda da caixa larga.
    vec3 currentTemporal = rgbToYCoCg(currentColor);
    vec3 neighborSum = vec3(0.0);
    vec3 neighborSumSquared = vec3(0.0);
    vec3 minNeighbor = currentTemporal;
    vec3 maxNeighbor = currentTemporal;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            vec3 neighbor = (x == 0 && y == 0) ? currentTemporal
                : rgbToYCoCg(texture(currentColorMap,
                    texCoord + vec2(x, y) * texelSize).rgb);
            neighborSum += neighbor;
            neighborSumSquared += neighbor * neighbor;
            minNeighbor = min(minNeighbor, neighbor);
            maxNeighbor = max(maxNeighbor, neighbor);
        }
    }
    const float sampleCount = 9.0;
    vec3 neighborMean = neighborSum / sampleCount;
    vec3 neighborVariance = max(neighborSumSquared / sampleCount
        - neighborMean * neighborMean, 0.0);
    vec3 neighborStdDev = sqrt(neighborVariance);

    // gamma = largura da caixa em desvios-padrao (1.0 e o valor classico de
    // Karis). Interseccao com a caixa min/max bruta garante que o clip
    // nunca aceita uma cor que nem apareceu de verdade na vizinhanca.
    const float gamma = 1.0;
    vec3 clipMin = max(minNeighbor, neighborMean - neighborStdDev * gamma);
    vec3 clipMax = min(maxNeighbor, neighborMean + neighborStdDev * gamma);

    vec3 historyTemporal = rgbToYCoCg(
        texture(historyMap, previousTexCoord).rgb);
    vec3 clampedHistory = clamp(historyTemporal, clipMin, clipMax);

    // Historico forte em pixels estaticos e resposta progressivamente mais
    // rapida em movimento ou mudanca grande de luminancia. Peso fixo alto
    // mantinha rastros e realimentava erros de reprojecao por muitos quadros.
    float velocityPixels = length(motionVector / texelSize);
    float motionReactivity = smoothstep(0.35, 14.0, velocityPixels);
    float luminanceDelta = abs(currentTemporal.x - clampedHistory.x)
        / max(max(currentTemporal.x, clampedHistory.x), 0.08);
    float luminanceReactivity = smoothstep(0.04, 0.45, luminanceDelta);
    float reactivity = max(motionReactivity, luminanceReactivity);
    float historyWeight = mix(0.94, 0.76, reactivity);
    vec3 resolvedTemporal = mix(currentTemporal, clampedHistory,
        historyWeight);
    vec3 resolved = max(yCoCgToRgb(resolvedTemporal), vec3(0.0));
    outColor = vec4(resolved, 1.0);
}
