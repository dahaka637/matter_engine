#version 450

// Passe de tonemap HDR->LDR: nao usa o SceneUniform nem sceneDescriptorSetLayout
// (ver createScene3DResources) porque so precisa amostrar o alvo de cor HDR
// que o passe opaco (ceu+mesh) acabou de renderizar - descriptor set
// dedicado, independente do resto da cena.
layout(set = 0, binding = 0) uniform sampler2D hdrColorMap;
// Resultado 1x1 do passe de medição/adaptação. O canal R contém somente o
// multiplicador automático; 1.0 preserva exatamente a exposição manual.
layout(set = 0, binding = 2) uniform sampler2D automaticExposureMap;
// Glare/flare solar acumulados em HDR a 1/8 da largura e altura. Não há
// bloom geral; o sampler LINEAR faz o upscale suave no passe final.
layout(set = 0, binding = 3) uniform sampler2D bloomGlareMap;
// Profundidade nearest da cena, usada somente pelo Matter Mosaic para
// extrair silhuetas sem contornar o ruído interno das texturas.
layout(set = 0, binding = 4) uniform sampler2D sceneDepthMap;
// RG contém movimento de tela; meshes de props físicos somam 4 em X para
// transportar uma classe de maior legibilidade sem alocar outro attachment.
layout(set = 0, binding = 5) uniform sampler2D motionVectorMap;

// Ver ToneMappingSettings3D (Scene3D.hpp) para o que cada campo faz - exposure
// age em espaco linear (antes da curva ACES), os outros tres em espaco de
// tela (depois da curva e da codificacao sRGB).
layout(push_constant) uniform TonemapPushConstants {
    float exposure;
    float brightness;
    float contrast;
    float saturation;
    float oceanSubmersion;
    float animationTimeSeconds;
    float pixelArtEnabled;
    float luminanceLevelCount;
    float chromaLevelCount;
    float ditherStrength;
    float reservedPixelArt0;
    float pixelGridHeight;
    float worldPixelation;
    float physicalPropPixelation;
    float distanceLodStrength;
    float distanceLodStartMeters;
    float distanceLodEndMeters;
    float flatteningStrength;
} pushConstants;

layout(location = 0) in vec2 texCoord;
layout(location = 0) out vec4 outColor;

// Aproximacao de Narkowicz para a curva filmica do ACES - barata (sem LUT,
// sem as matrizes RRT/ODT completas) e ainda assim da o "joelho" suave nos
// realces que faltava com o armazenamento direto em UNORM sem tonemap algum:
// antes, qualquer valor de luz acima de 1.0 simplesmente estourava para
// branco solido (clamp implicito do formato); agora ele comprime com
// gradacao.
vec3 acesFilmicTonemap(vec3 color) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((color * (a * color + b)) / (color * (c * color + d) + e), 0.0, 1.0);
}

// O swapchain e UNORM, nao sRGB (ver VulkanDevice::createSwapchain). Isso
// significa que nenhum hardware fixed-function aplica essa conversao para
// nos; ela precisa acontecer aqui, no ultimo passe antes da imagem final, ou
// a cena inteira fica mais escura do que deveria (grava valores lineares num
// destino que o monitor vai interpretar como se ja estivessem codificados em
// sRGB).
vec3 linearToSrgb(vec3 linearColor) {
    vec3 lower = linearColor * 12.92;
    vec3 higher = 1.055 * pow(linearColor, vec3(1.0 / 2.4)) - 0.055;
    return mix(higher, lower, step(linearColor, vec3(0.0031308)));
}

float bayer4x4(ivec2 pixel) {
    const float values[16] = float[](
         0.0,  8.0,  2.0, 10.0,
        12.0,  4.0, 14.0,  6.0,
         3.0, 11.0,  1.0,  9.0,
        15.0,  7.0, 13.0,  5.0
    );
    ivec2 wrapped = pixel & ivec2(3);
    return (values[wrapped.y * 4 + wrapped.x] + 0.5) / 16.0;
}

// Encaixa a amostra HDR numa grade deliberada sem reduzir a resolução real
// do render. Props físicos marcados usam progressivamente células menores a
// partir da média distância; de perto compartilham exatamente a mesma grade
// do cenário, evitando o aspecto de recorte HD colado sobre o mundo.
vec2 matterMosaicSampleUv(vec2 uv, out float pixelSize,
        out ivec2 sourcePixel) {
    ivec2 extent = textureSize(hdrColorMap, 0);
    ivec2 maximumPixel = max(extent - ivec2(1), ivec2(0));
    ivec2 pixel = clamp(ivec2(uv * vec2(extent)),
        ivec2(0), maximumPixel);
    float encodedMotionX = texelFetch(
        motionVectorMap, pixel, 0).r;
    bool highDetailProp = encodedMotionX > 2.0;
    float depth = texelFetch(sceneDepthMap, pixel, 0).r;

    float targetGridHeight = max(pushConstants.pixelGridHeight, 1.0);
    float basePixelSize = max(round(
        float(extent.y) / targetGridHeight), 1.0);

    // Reversed-Z do laboratório, near=0.08 m: near/depth aproxima distância
    // em perspectiva com erro irrelevante para esta transição artística.
    float distanceMeters = depth > 0.000001 ? 0.08 / depth : 100000.0;
    float lodStart = max(pushConstants.distanceLodStartMeters, 0.0);
    float lodEnd = max(pushConstants.distanceLodEndMeters,
        lodStart + 1.0);
    // Céu não possui distância física. Estrelas conservam apenas a grade
    // base, equivalente ao comportamento antigo com LOD adicional em zero.
    bool skyPixel = depth <= 0.000001;
    float distanceLod = skyPixel ? 0.0
        : smoothstep(lodStart, lodEnd, distanceMeters)
            * clamp(pushConstants.distanceLodStrength, 0.0, 1.0);
    float pixelation = clamp(highDetailProp
        ? pushConstants.physicalPropPixelation
        : pushConstants.worldPixelation, 0.0, 1.0);
    float closePixelSize = mix(1.0, basePixelSize, pixelation);
    // O cenário assume blocos maiores conforme afasta. Props físicos também
    // entram no LOD, mas só com 28% do crescimento: continuam pertencendo à
    // arte de perto e permanecem legíveis no campo/horizonte.
    float lodGrowth = basePixelSize * 2.15 * distanceLod
        * (highDetailProp ? 0.28 : 1.0);
    pixelSize = max(round(closePixelSize
        + lodGrowth * pixelation), 1.0);

    ivec2 cell = ivec2(pixelSize);
    ivec2 snappedPixel = (pixel / cell) * cell + cell / 2;
    snappedPixel = clamp(snappedPixel, ivec2(0), maximumPixel);
    sourcePixel = pixel;
    return (vec2(snappedPixel) + 0.5) / vec2(extent);
}

vec3 applyMatterMosaic(vec3 color, float pixelSize, ivec2 sourcePixel) {
    ivec2 artPixel = sourcePixel
        / max(ivec2(pixelSize), ivec2(1));

    // Quantizar luminância e crominância separadamente preserva a família
    // da cor. Arredondar RGB diretamente criaria matizes falsos entre duas
    // bandas de iluminação.
    float y = dot(color, vec3(0.25, 0.50, 0.25));
    // A trama só participa de meios-tons. Pretos sólidos, silhuetas e
    // realces do céu permanecem limpos em vez de parecerem cobertos por uma
    // transparência quadriculada.
    float ditherMask = smoothstep(0.055, 0.20, y)
        * (1.0 - smoothstep(0.72, 0.94, y));
    float orderedDither = (bayer4x4(artPixel) - 0.5)
        * clamp(pushConstants.ditherStrength, 0.0, 1.0)
        * ditherMask;
    float co = (color.r - color.b) * 0.5;
    float cg = (-color.r + 2.0 * color.g - color.b) * 0.25;
    float lightSteps = max(round(pushConstants.luminanceLevelCount), 2.0);
    float chromaSteps = max(round(pushConstants.chromaLevelCount), 2.0);
    y = floor(y * (lightSteps - 1.0) + 0.5 + orderedDither)
        / (lightSteps - 1.0);
    co = floor((co + 0.5) * chromaSteps + 0.5)
        / chromaSteps - 0.5;
    cg = floor((cg + 0.5) * chromaSteps + 0.5)
        / chromaSteps - 0.5;
    vec3 mosaicColor = vec3(
        y + co - cg,
        y + cg,
        y - co - cg
    );
    // Um split-tone mínimo é parte da identidade Matter: sombras ganham
    // profundidade azul-petróleo e luzes recebem calor. A transformação
    // segue a luminância da própria cena, então amanhecer, noite e clima
    // continuam definindo a paleta dominante.
    vec3 shadowInk = vec3(0.94, 0.98, 1.035);
    vec3 highlightPaper = vec3(1.025, 1.008, 0.965);
    mosaicColor *= mix(shadowInk, highlightPaper,
        smoothstep(0.16, 0.78, y));

    return clamp(mix(color, mosaicColor,
        clamp(pushConstants.flatteningStrength, 0.0, 1.0)),
        0.0, 1.0);
}

vec3 applyDisplayGrade(vec3 color) {
    color += pushConstants.brightness;
    color = (color - 0.5) * max(pushConstants.contrast, 0.0) + 0.5;
    float luma = dot(color, vec3(0.299, 0.587, 0.114));
    return mix(vec3(luma), color,
        max(pushConstants.saturation, 0.0));
}

void main() {
    float submersion = clamp(pushConstants.oceanSubmersion, 0.0, 1.0);
    vec2 refractionOffset = vec2(0.0);
    // A condição é uniforme para o quadro inteiro: acima da água o caminho
    // comum não paga pelas funções trigonométricas do efeito submerso.
    if (submersion > 0.001) {
        refractionOffset = vec2(
            sin(texCoord.y * 43.0
                + pushConstants.animationTimeSeconds * 1.35),
            cos(texCoord.x * 37.0
                - pushConstants.animationTimeSeconds * 1.08))
            * (0.0014 * submersion);
    }
    vec2 sampledUv = clamp(texCoord + refractionOffset,
        vec2(0.001), vec2(0.999));
    float mosaicPixelSize = 1.0;
    ivec2 mosaicSourcePixel = ivec2(0);
    if (pushConstants.pixelArtEnabled > 0.5) {
        sampledUv = matterMosaicSampleUv(
            sampledUv, mosaicPixelSize, mosaicSourcePixel);
    }
    float automaticExposure = texture(automaticExposureMap, vec2(0.5)).r;
    float exposureMultiplier = max(pushConstants.exposure, 0.0)
        * max(automaticExposure, 0.0);
    vec3 sceneHdr = texture(hdrColorMap, sampledUv).rgb
        * exposureMultiplier;
    // O Sol/disco continua preso à grade porque pertence ao HDR da cena.
    // Apenas a resposta óptica usa UV nativa no Pixel Art, como um efeito
    // de lente suave composto por cima da imagem já pixelizada.
    vec2 opticsUv = pushConstants.pixelArtEnabled > 0.5
        ? texCoord : sampledUv;
    vec3 opticsHdr = texture(bloomGlareMap, opticsUv).rgb
        * exposureMultiplier;
    vec3 color;
    vec3 smoothOptics = vec3(0.0);
    if (pushConstants.pixelArtEnabled > 0.5) {
        // O Mosaic precisa separar a resposta optica para compô-la depois da
        // quantizacao. Somente este caminho paga por duas curvas ACES.
        vec3 baseColor = applyDisplayGrade(
            linearToSrgb(acesFilmicTonemap(sceneHdr)));
        vec3 combinedColor = applyDisplayGrade(
            linearToSrgb(acesFilmicTonemap(sceneHdr + opticsHdr)));
        smoothOptics = max(combinedColor - baseColor, vec3(0.0));
        color = baseColor;
    } else {
        // No modo normal a antiga baseColor era calculada e descartada em
        // todos os pixels. Uma unica curva sobre cena+optica produz
        // exatamente o mesmo resultado final.
        color = applyDisplayGrade(
            linearToSrgb(acesFilmicTonemap(sceneHdr + opticsHdr)));
    }

    // Visão submersa integrada ao passe já existente: absorção seletiva,
    // espalhamento azul-esverdeado e refração de uma única amostra.
    if (submersion > 0.001) {
        vec3 absorbed = color * vec3(0.44, 0.78, 0.91);
        vec3 scattered = mix(
            absorbed, vec3(0.018, 0.19, 0.25), 0.36);
        float edgeDistance = length(texCoord - vec2(0.5)) * 1.4142;
        scattered *= 1.0
            - smoothstep(0.55, 1.0, edgeDistance) * 0.16;
        color = mix(color, scattered, submersion);
    }

    if (pushConstants.pixelArtEnabled > 0.5) {
        color = applyMatterMosaic(
            color, mosaicPixelSize, mosaicSourcePixel);
        // Glare e ghosts não passam pela quantização de paleta nem pela
        // grade do Mosaic. O disco do Sol, vindo de sceneHdr, passa.
        color += smoothOptics * (1.0 - submersion);
    }

    outColor = vec4(clamp(color, 0.0, 1.0), 1.0);
}
