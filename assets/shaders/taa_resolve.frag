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
    vec4 fogSettings; // x=densidade (por metro apos w), y=acoplamento altura-distancia, z=opacidade maxima, w=distancia de inicio (m)
    vec4 fogColor;    // rgb=cor da neblina, a reservado
    vec4 windOffset;  // xy=deslocamento acumulado do vento nas nuvens, zw reservado
    vec4 environmentSettings; // layout compartilhado; não usado aqui
    vec4 renderSettings; // x=Pixel Art, y=tamanho base, z=TAA completo
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

vec2 clipPositionToUv(vec4 clipPosition) {
    return clipPosition.xy / clipPosition.w * 0.5 + 0.5;
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
    bool skyPixel = currentDepth <= 0.0001;
    bool pixelArtMode = scene.renderSettings.x > 0.5;
    if (skyPixel && !pixelArtMode) {
        // O modo normal preserva o caminho barato anterior; a cobertura
        // analítica e o histórico direcional são necessários apenas para a
        // grade deliberadamente grossa do Mosaic.
        outColor = vec4(currentColor, 1.0);
        return;
    }
    if (pixelArtMode && !skyPixel && scene.renderSettings.z < 0.5) {
        // Sem o TAA experimental, somente o céu participa do histórico.
        // Mundo, arma e props seguem o caminho antigo exatamente, portanto
        // não podem produzir rastro ou alterar iluminação por acumulação.
        outColor = vec4(currentColor, 1.0);
        return;
    }
    vec2 motionVector;
    if (skyPixel) {
        // O céu está ancorado em direções do mundo e não escreve motion
        // vectors. Reconstruir o raio atual e projetar um ponto muito
        // distante com a VP anterior fornece somente a rotação da câmera;
        // assim estrelas/nuvens encontram seu histórico em vez de piscarem.
        vec2 clip = texCoord * 2.0 - 1.0;
        vec4 farPoint = scene.inverseCameraViewProjection
            * vec4(clip.x, -clip.y, 1.0, 1.0);
        vec3 worldFar = farPoint.xyz / max(abs(farPoint.w), 0.00001);
        vec3 direction = normalize(
            worldFar - scene.cameraPosition.xyz);
        vec3 distantPoint = scene.cameraPosition.xyz
            + direction * 100000.0;
        vec4 previousClip = scene.previousCameraViewProjection
            * vec4(distantPoint, 1.0);
        previousClip.y = -previousClip.y;
        vec2 previousSkyUv = clipPositionToUv(previousClip);
        motionVector = texCoord - previousSkyUv;
    } else {
        motionVector = texture(motionVectorMap, texCoord).rg;
    }
    // scene3d_mesh.frag usa +4 em X como classe de detalhe do Mosaic. Isso
    // fica fora da faixa possível de um deslocamento de tela e é removido
    // aqui antes da reprojeção, mantendo o modo Normal bit-a-bit compatível
    // com os vetores reais.
    if (!skyPixel && motionVector.x > 2.0) motionVector.x -= 4.0;
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
    vec3 clipMin;
    vec3 clipMax;
    if (pixelArtMode && skyPixel) {
        // Céu procedural não possui desoclusões locais. Uma faixa compacta
        // basta para conservar estrelas subpixel sem espalhar seu brilho.
        vec3 temporalAllowance = vec3(
            max(currentTemporal.x * 0.045, 0.012),
            0.040, 0.040);
        clipMin = currentTemporal - temporalAllowance;
        clipMax = currentTemporal + temporalAllowance;
    } else {
        vec3 neighborSum = vec3(0.0);
        vec3 neighborSumSquared = vec3(0.0);
        vec3 minNeighbor = currentTemporal;
        vec3 maxNeighbor = currentTemporal;
        // Cruz de cinco taps: preserva os extremos horizontal/vertical que
        // definem silhuetas, mas elimina quatro leituras diagonais por pixel.
        const vec2 temporalOffsets[5] = vec2[](
            vec2(0.0), vec2(-1.0, 0.0), vec2(1.0, 0.0),
            vec2(0.0, -1.0), vec2(0.0, 1.0));
        for (int sampleIndex = 0; sampleIndex < 5; ++sampleIndex) {
            vec3 neighbor = sampleIndex == 0 ? currentTemporal
                : rgbToYCoCg(texture(currentColorMap,
                    texCoord + temporalOffsets[sampleIndex]
                        * texelSize).rgb);
            neighborSum += neighbor;
            neighborSumSquared += neighbor * neighbor;
            minNeighbor = min(minNeighbor, neighbor);
            maxNeighbor = max(maxNeighbor, neighbor);
        }
        const float sampleCount = 5.0;
        vec3 neighborMean = neighborSum / sampleCount;
        vec3 neighborVariance = max(neighborSumSquared / sampleCount
            - neighborMean * neighborMean, 0.0);
        vec3 neighborStdDev = sqrt(neighborVariance);

        // gamma = largura da caixa em desvios-padrao (1.0 e o valor
        // classico de Karis). A interseção nunca aceita uma cor ausente da
        // vizinhança real.
        const float gamma = 1.0;
        clipMin = max(minNeighbor,
            neighborMean - neighborStdDev * gamma);
        clipMax = min(maxNeighbor,
            neighborMean + neighborStdDev * gamma);
    }

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
    float historyWeight = pixelArtMode && skyPixel
        ? mix(0.95, 0.65, reactivity)
        : mix(0.94, 0.76, reactivity);
    vec3 resolvedTemporal = mix(currentTemporal, clampedHistory,
        historyWeight);
    vec3 resolved = max(yCoCgToRgb(resolvedTemporal), vec3(0.0));
    outColor = vec4(resolved, 1.0);
}
