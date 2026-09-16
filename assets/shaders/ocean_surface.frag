#version 450

layout(set = 0, binding = 0, std140) uniform SceneUniform {
    mat4 cameraViewProjection;
    mat4 cameraViewProjectionUnjittered;
    mat4 inverseCameraViewProjection;
    mat4 cascadeViewProjections[4];
    vec4 cascadeSplits;
    vec4 cascadeTexelWorldSizes;
    vec4 cascadeDepthRanges;
    mat4 previousCameraViewProjection;
    vec4 cameraPosition;
    vec4 settings;
    vec4 skySettings;
    vec4 fogSettings;
    vec4 fogColor; // rgb=cor dinamica, a=distancia de ocultacao total
    vec4 windOffset;
    vec4 environmentSettings;
} scene;

struct GpuLight {
    vec4 positionRange;
    vec4 directionOuterCosine;
    vec4 colorIntensity;
    vec4 parameters;
};

layout(std430, set = 0, binding = 2) readonly buffer SceneLights {
    GpuLight lights[];
} sceneLights;

layout(set = 1, binding = 0) uniform sampler2D opaqueDepth;

layout(push_constant) uniform OceanPushConstants {
    vec2 meshOrigin;
    float meanSeaLevel;
    float timeSeconds;
    vec2 worldCenter;
    float worldHalfExtent;
    float reserved;
} ocean;

layout(location = 0) in vec3 surfacePosition;
layout(location = 1) in vec3 geometricNormal;
layout(location = 2) in vec4 opaqueClipPosition;
layout(location = 3) in vec4 motionClipNow;
layout(location = 4) in vec4 motionClipBefore;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec2 outMotion;

vec2 clipUv(vec4 clipPosition) {
    return clipPosition.xy / clipPosition.w * 0.5 + 0.5;
}

vec3 reconstructPosition(vec2 uv, float depth) {
    vec4 clip = vec4(uv.x * 2.0 - 1.0,
        -(uv.y * 2.0 - 1.0), depth, 1.0);
    vec4 world = scene.inverseCameraViewProjection * clip;
    float divisor = abs(world.w) > 0.00001
        ? world.w : (world.w < 0.0 ? -0.00001 : 0.00001);
    return world.xyz / divisor;
}

float fifthPower(float value) {
    float squared = value * value;
    return squared * squared * value;
}

vec2 fineSurfaceDerivative(vec2 position, float time) {
    vec2 a = vec2(0.857493, 0.514496);
    vec2 b = vec2(-0.371391, 0.928477);
    vec2 c = vec2(0.980581, -0.196116);
    float phaseA = dot(position, a) * 1.58 - time * 1.72;
    float phaseB = dot(position, b) * 2.31 - time * 2.26;
    float phaseC = dot(position, c) * 3.47 - time * 2.91;

    // Filtragem analítica: quando uma oscilação ocupa menos de alguns pixels
    // ela desaparece gradualmente, em vez de virar moiré/serrilhado no
    // horizonte. fwidth mede a variação da fase entre pixels vizinhos.
    float filterA = 1.0
        - smoothstep(0.55, 1.35, fwidth(phaseA));
    float filterB = 1.0
        - smoothstep(0.50, 1.20, fwidth(phaseB));
    float filterC = 1.0
        - smoothstep(0.45, 1.05, fwidth(phaseC));
    return
        a * (cos(phaseA) * 0.018 * filterA)
        + b * (cos(phaseB) * 0.010 * filterB)
        + c * (cos(phaseC) * 0.004 * filterC);
}

vec3 fineSurfaceNormal(vec3 base, vec2 position, float time,
        float cameraDistance) {
    // A região próxima é animada em velocidade normal. Entre 220 e 300 m
    // ocorre uma mistura para um campo estático separado: a onda distante
    // não desacelera, ela simplesmente deixa de se mover.
    float movingDetail = 1.0
        - smoothstep(220.0, 300.0, cameraDistance);
    vec2 animated = fineSurfaceDerivative(position, time);
    vec2 frozen = fineSurfaceDerivative(position + vec2(17.3, -9.7), 0.0);
    vec2 derivative = mix(frozen, animated, movingDetail);

    // Mantém a textura óptica do mar por quase todo o horizonte. Só os
    // últimos metros perdem detalhe para evitar aliasing subpixel.
    float horizonFade =
        1.0 - smoothstep(1180.0, 1470.0, cameraDistance);
    return normalize(base + vec3(-derivative * horizonFade, 0.0));
}

vec3 skyReflection(vec3 direction, vec3 sunDirection,
        float day, float energy) {
    vec3 nightHorizon = vec3(0.012, 0.020, 0.050);
    vec3 dayHorizon = vec3(0.48, 0.66, 0.82);
    vec3 nightZenith = vec3(0.001, 0.004, 0.020);
    vec3 dayZenith = vec3(0.025, 0.115, 0.30);
    float elevation = sqrt(clamp(direction.z, 0.0, 1.0));
    vec3 horizon = mix(nightHorizon, dayHorizon, day);
    vec3 zenith = mix(nightZenith, dayZenith, day);
    vec3 result = mix(horizon, zenith, elevation)
        * mix(0.34, 1.0, sqrt(energy));

    float alignment = max(dot(direction, sunDirection), 0.0);
    float glitter = smoothstep(0.985, 0.9997, alignment);
    glitter *= glitter;
    result += vec3(1.0, 0.78, 0.45) * glitter * day * 1.8;
    return result;
}

void main() {
    vec2 fromCenter = abs(surfacePosition.xy - ocean.worldCenter);
    if (max(fromCenter.x, fromCenter.y) > ocean.worldHalfExtent) discard;

    vec3 towardCamera = scene.cameraPosition.xyz - surfacePosition;
    float cameraDistance = length(towardCamera);
    if (cameraDistance >= scene.fogColor.a) {
        outColor = vec4(scene.fogColor.rgb, 1.0);
        outMotion = vec2(0.0);
        return;
    }
    vec3 view = towardCamera / max(cameraDistance, 0.0001);
    vec3 normal = fineSurfaceNormal(normalize(geometricNormal),
        surfacePosition.xy, ocean.timeSeconds, cameraDistance);
    // O clipmap é dupla-face para continuar visível debaixo d'água. Orientar
    // a normal para a câmera evita depender da convenção de winding depois
    // da inversão de Y feita para Vulkan.
    if (dot(normal, view) < 0.0) normal = -normal;

    vec2 uv = clipUv(opaqueClipPosition);
    float sceneDepth = texture(opaqueDepth, uv).r;
    float depthMeters = 35.0;
    if (sceneDepth > 0.00001) {
        vec3 bottom = reconstructPosition(uv, sceneDepth);
        depthMeters = clamp(surfacePosition.z - bottom.z, 0.0, 35.0);
    }

    int lightCount = int(scene.settings.y);
    vec3 sunDirection = lightCount > 0
        ? normalize(sceneLights.lights[0].directionOuterCosine.xyz)
        : vec3(0.0, 0.0, 1.0);
    float day = smoothstep(-0.08, 0.12, sunDirection.z);
    float environmentEnergy = clamp(scene.settings.w / 0.31, 0.02, 1.0);

    float facing = clamp(dot(normal, view), 0.0, 1.0);
    float fresnel = 0.0204
        + (1.0 - 0.0204) * fifthPower(1.0 - facing);
    vec3 reflectedDirection = reflect(-view, normal);
    vec3 reflection = skyReflection(reflectedDirection,
        sunDirection, day, environmentEnergy);

    // Beer-Lambert compacto: vermelho desaparece primeiro, azul penetra mais.
    vec3 transmission = exp(-vec3(0.24, 0.075, 0.035) * depthMeters);
    vec3 inScattering = vec3(0.006, 0.105, 0.145)
        * (vec3(1.0) - transmission)
        * mix(0.10, 1.0, day * environmentEnergy);
    vec3 color = mix(inScattering, reflection, fresnel);

    // Uma faixa costeira contínua e estreita. A variação altera suavemente a
    // espessura/brilho, mas nunca recorta círculos isolados na areia.
    float depthPixelWidth = max(fwidth(depthMeters) * 1.5, 0.025);
    float foamWidth = 0.18 + depthPixelWidth;
    float shore = 1.0 - smoothstep(0.025, foamWidth, depthMeters);
    float foamVariation = clamp(0.72
        + 0.13 * sin(dot(surfacePosition.xy, vec2(0.31, -0.47)) * 0.83
            + ocean.timeSeconds * 0.72)
        + 0.09 * sin(dot(surfacePosition.xy, vec2(-0.61, -0.22)) * 1.37
            - ocean.timeSeconds * 0.49),
        0.48, 0.94);
    float foam = shore * foamVariation;
    color = mix(color, vec3(0.72, 0.82, 0.84)
        * mix(0.15, 1.0, day), foam * 0.52);

    float fogDistance =
        max(0.0, cameraDistance - scene.fogSettings.w);
    float exponentialFog =
        (1.0 - exp(-fogDistance * scene.fogSettings.x))
        * smoothstep(-2.0, 5.0,
            surfacePosition.z + fogDistance * scene.fogSettings.y);
    float terminalStart = mix(
        scene.fogSettings.w, scene.fogColor.a, 0.78);
    float terminalFog = smoothstep(
        terminalStart, scene.fogColor.a, cameraDistance);
    float fog = max(
        min(exponentialFog, scene.fogSettings.z),
        terminalFog);
    color = mix(color, scene.fogColor.rgb,
        clamp(fog, 0.0, 1.0));

    float opticalDepth = 1.0 - dot(transmission,
        vec3(0.2126, 0.7152, 0.0722));
    float alpha = mix(0.24, 0.90, clamp(opticalDepth, 0.0, 1.0));
    alpha = max(alpha, foam * 0.84);
    outColor = vec4(color, alpha);
    outMotion = clipUv(motionClipNow) - clipUv(motionClipBefore);
}
