#version 450

layout(set = 0, binding = 0) uniform sampler2D hdrColorMap;
layout(set = 0, binding = 1) uniform sampler2D depthMap;

layout(push_constant) uniform BloomGlarePushConstants {
    float glareStrength;
    float flareStrength;
    float reserved0;
    float reserved1;
    vec2 sunUv;
    float sunOnScreen;
    float enabled;
} pushConstants;

layout(location = 0) in vec2 texCoord;
layout(location = 0) out vec4 outOptics;

float luminance(vec3 color) {
    return dot(max(color, vec3(0.0)),
        vec3(0.2126, 0.7152, 0.0722));
}

float disc(vec2 point, vec2 center, float radius, float feather) {
    return 1.0 - smoothstep(radius - feather,
        radius + feather, length(point - center));
}

void main() {
    if (pushConstants.enabled < 0.5) {
        outOptics = vec4(0.0);
        return;
    }

    float sunVisibility = 0.0;
    vec3 sunColor = vec3(1.0, 0.86, 0.62);
    if (pushConstants.sunOnScreen > 0.5) {
        float sunDepth = texture(depthMap, pushConstants.sunUv).r;
        vec3 sunRadiance =
            texture(hdrColorMap, pushConstants.sunUv).rgb;
        // Reversed-Z: profundidade zero é céu. A luminância impede flare
        // forte quando nuvens densas cobrem visualmente o disco do Sol.
        float depthVisibility =
            1.0 - smoothstep(0.00005, 0.0008, sunDepth);
        float radianceVisibility = smoothstep(
            0.80, 2.40, luminance(sunRadiance));
        sunVisibility = depthVisibility * radianceVisibility;
        sunColor = mix(sunColor, max(sunRadiance, sunColor), 0.22);
    }

    vec2 aspect = vec2(
        float(textureSize(hdrColorMap, 0).x)
            / float(textureSize(hdrColorMap, 0).y), 1.0);
    vec2 fromSun = (texCoord - pushConstants.sunUv) * aspect;
    float radialDistance = length(fromSun);
    vec2 center = vec2(0.5);
    float gazeDistance = length(
        (pushConstants.sunUv - center) * aspect);
    // Ghosts aparecem apenas quando o jogador está realmente olhando para
    // perto do Sol. O glare ainda dá uma indicação discreta mais perto da
    // borda, mas também cresce bastante no olhar direto.
    float directGaze = 1.0 - smoothstep(0.075, 0.32, gazeDistance);

    // Forma sem exp(): além de não haver mais as 13 leituras do bloom, o
    // núcleo/riscos usam apenas smoothstep e uma divisão.
    float sunHalo = 1.0
        / (1.0 + radialDistance * radialDistance * 105.0);
    float horizontal = (1.0 - smoothstep(
        0.006, 0.030, abs(fromSun.y)))
        * (1.0 - smoothstep(0.18, 0.95, abs(fromSun.x)));
    float vertical = (1.0 - smoothstep(
        0.004, 0.024, abs(fromSun.x)))
        * (1.0 - smoothstep(0.12, 0.72, abs(fromSun.y)));
    float glareFocus = mix(0.48, 1.35, directGaze);
    vec3 glare = sunColor * sunVisibility
        * pushConstants.glareStrength
        * glareFocus
        * (sunHalo * 1.10 + horizontal * 0.09
            + vertical * 0.055);

    // Ghosts baratos ao longo do eixo Sol-centro. Posições, tamanhos e cores
    // diferentes quebram a aparência de círculos idênticos sobrepostos. A
    // janela de olhar direto evita círculos atravessando a tela toda vez que
    // o Sol apenas encosta no campo de visão.
    vec2 axis = center - pushConstants.sunUv;
    float ghostA = disc(texCoord * aspect,
        (pushConstants.sunUv + axis * 0.72) * aspect, 0.035, 0.014);
    float ghostB = disc(texCoord * aspect,
        (pushConstants.sunUv + axis * 1.25) * aspect, 0.060, 0.022);
    float ghostC = disc(texCoord * aspect,
        (pushConstants.sunUv + axis * 1.72) * aspect, 0.025, 0.012);
    vec3 ghosts = (vec3(0.20, 0.46, 0.72) * ghostA
        + vec3(0.65, 0.24, 0.12) * ghostB
        + vec3(0.32, 0.62, 0.30) * ghostC)
        * sunVisibility * pushConstants.flareStrength
        * directGaze * directGaze;

    outOptics = vec4(max(glare + ghosts, vec3(0.0)),
        sunVisibility);
}
