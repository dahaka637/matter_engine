#version 450

// A mesma família de descriptor sets do tonemap é usada para evitar sets
// mutáveis durante frames em voo. Binding 0 é o HDR resolvido deste quadro;
// binding 1 é o valor de exposição automática produzido no quadro anterior.
layout(set = 0, binding = 0) uniform sampler2D hdrColorMap;
layout(set = 0, binding = 1) uniform sampler2D previousExposureMap;

layout(push_constant) uniform AutoExposurePushConstants {
    float minimumExposure;
    float maximumExposure;
    float meteringKey;
    float deltaTime;
    float brightAdaptationSpeed;
    float darkAdaptationSpeed;
    float historyValid;
    float enabled;
} pushConstants;

layout(location = 0) out vec4 outExposure;

float luminance(vec3 color) {
    return dot(max(color, vec3(0.0)), vec3(0.2126, 0.7152, 0.0722));
}

void main() {
    if (pushConstants.enabled < 0.5) {
        outExposure = vec4(1.0);
        return;
    }

    // 8x6 amostras determinísticas: custo constante (48 fetches num único
    // fragmento), qualquer que seja a resolução. A malha mais fina evita
    // saltos quando uma borda de céu cruza uma amostra isolada. A região
    // central recebe um pouco mais de peso,
    // imitando a medição center-weighted de uma câmera e impedindo uma faixa
    // pequena de céu/sol na borda de dominar toda a exposição.
    float weightedLogLuminance = 0.0;
    float totalWeight = 0.0;
    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 8; ++x) {
            vec2 uv = (vec2(x, y) + vec2(0.5)) / vec2(8.0, 6.0);
            vec2 centered = uv * 2.0 - 1.0;
            float centerWeight = mix(0.48, 1.0,
                exp(-dot(centered, centered) * 1.35));
            float sampleLuminance = clamp(
                luminance(textureLod(hdrColorMap, uv, 0.0).rgb),
                0.005, 32.0);
            weightedLogLuminance += log(sampleLuminance) * centerWeight;
            totalWeight += centerWeight;
        }
    }

    float averageLuminance = exp(weightedLogLuminance / totalWeight);
    float targetExposure = clamp(
        pushConstants.meteringKey / max(averageLuminance, 0.005),
        pushConstants.minimumExposure,
        pushConstants.maximumExposure);

    float exposure = targetExposure;
    if (pushConstants.historyValid > 0.5) {
        float previousExposure =
            texture(previousExposureMap, vec2(0.5)).r;
        // A forma exponencial é independente do FPS. Mudanças grandes em
        // stops recebem um reforço transitório: a câmera acompanha chão->céu
        // rapidamente, mas a zona morta abaixo impede micro-pulsos quando a
        // composição já está estável.
        float speed = targetExposure < previousExposure
            ? pushConstants.brightAdaptationSpeed
            : pushConstants.darkAdaptationSpeed;
        float stopDifference = abs(log2(max(targetExposure, 0.001)
            / max(previousExposure, 0.001)));
        float responseBoost = mix(1.0, 2.8,
            smoothstep(0.22, 1.80, stopDifference));
        if (stopDifference < 0.018) {
            exposure = previousExposure;
        } else {
            float blend = 1.0 - exp(-max(speed, 0.0)
                * responseBoost
                * clamp(pushConstants.deltaTime, 0.0, 0.1));
            exposure = mix(previousExposure, targetExposure, blend);
        }
    }

    outExposure = vec4(exposure, exposure, exposure, 1.0);
}
