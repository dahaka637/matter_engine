#version 450

// Passe de tonemap HDR->LDR: nao usa o SceneUniform nem sceneDescriptorSetLayout
// (ver createScene3DResources) porque so precisa amostrar o alvo de cor HDR
// que o passe opaco (ceu+mesh) acabou de renderizar - descriptor set
// dedicado, independente do resto da cena.
layout(set = 0, binding = 0) uniform sampler2D hdrColorMap;

// Ver ToneMappingSettings3D (Scene3D.hpp) para o que cada campo faz - exposure
// age em espaco linear (antes da curva ACES), os outros tres em espaco de
// tela (depois da curva e da codificacao sRGB).
layout(push_constant) uniform TonemapPushConstants {
    float exposure;
    float brightness;
    float contrast;
    float saturation;
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

// O swapchain e UNORM, nao sRGB (ver VulkanDevice::createSwapchain, escolhido
// assim de proposito para o blit de upscale pixel-art nao reamostrar em
// espaco de cor errado). Isso significa que nenhum hardware fixed-function
// aplica essa conversao para nos; ela precisa acontecer aqui, no ultimo passe
// antes da imagem final, ou a cena inteira fica mais escura do que deveria
// (grava valores lineares num destino que o monitor vai interpretar como se
// ja estivessem codificados em sRGB).
vec3 linearToSrgb(vec3 linearColor) {
    vec3 lower = linearColor * 12.92;
    vec3 higher = 1.055 * pow(linearColor, vec3(1.0 / 2.4)) - 0.055;
    return mix(higher, lower, step(linearColor, vec3(0.0031308)));
}

void main() {
    vec3 hdrColor = texture(hdrColorMap, texCoord).rgb * max(pushConstants.exposure, 0.0);
    vec3 tonemapped = acesFilmicTonemap(hdrColor);
    vec3 color = linearToSrgb(tonemapped);

    // Brilho/contraste/saturacao - ja em espaco de tela (0..1, pos-sRGB),
    // do mesmo jeito que um controle de imagem de monitor ou editor de
    // fotos. Contraste pivota em 0.5 (o cinza medio da tela), nao em 0.0,
    // ou girar o slider mudaria o brilho geral junto com o contraste.
    color += pushConstants.brightness;
    color = (color - 0.5) * max(pushConstants.contrast, 0.0) + 0.5;
    float luma = dot(color, vec3(0.299, 0.587, 0.114));
    color = mix(vec3(luma), color, max(pushConstants.saturation, 0.0));

    outColor = vec4(clamp(color, 0.0, 1.0), 1.0);
}
