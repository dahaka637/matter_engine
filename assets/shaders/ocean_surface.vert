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
    vec4 fogColor;
    vec4 windOffset;
    vec4 environmentSettings;
} scene;

layout(push_constant) uniform OceanPushConstants {
    vec2 meshOrigin;
    float meanSeaLevel;
    float timeSeconds;
    vec2 worldCenter;
    float worldHalfExtent;
    float reserved;
} ocean;

layout(location = 0) in vec2 inLocalPosition;

layout(location = 0) out vec3 surfacePosition;
layout(location = 1) out vec3 geometricNormal;
layout(location = 2) out vec4 opaqueClipPosition;
layout(location = 3) out vec4 motionClipNow;
layout(location = 4) out vec4 motionClipBefore;

struct SpectrumBand {
    vec2 direction;
    float wavelength;
    float amplitude;
    float phase;
    float speedScale;
};

// Espelho exato de OceanSurface.hpp. O deslocamento é um campo de altura
// suave e determinístico; os detalhes curtos do fragment shader não alteram
// a superfície física.
const SpectrumBand Bands[4] = SpectrumBand[](
    SpectrumBand(vec2(0.894427, 0.447214), 36.0, 0.155, 0.2, 0.82),
    SpectrumBand(vec2(0.316228, -0.948683), 19.0, 0.072, 2.1, 0.95),
    SpectrumBand(vec2(-0.780869, 0.624695), 9.5, 0.031, 4.0, 1.08),
    SpectrumBand(vec2(0.980581, -0.196116), 4.8, 0.012, 5.3, 1.18)
);

void sampleSurface(vec2 position, float detailFade,
        out float height, out vec3 normal) {
    height = ocean.meanSeaLevel;
    vec2 derivative = vec2(0.0);
    for (int index = 0; index < 4; ++index) {
        float waveNumber = 6.28318530718 / Bands[index].wavelength;
        float angularSpeed =
            sqrt(9.81 * waveNumber) * Bands[index].speedScale;
        float phase = waveNumber * dot(Bands[index].direction, position)
            - angularSpeed * ocean.timeSeconds + Bands[index].phase;
        float amplitude = Bands[index].amplitude * detailFade;
        height += amplitude * sin(phase);
        derivative += Bands[index].direction
            * (amplitude * waveNumber * cos(phase));
    }
    normal = normalize(vec3(-derivative, 1.0));
}

void main() {
    vec2 worldXY = ocean.meshOrigin + inLocalPosition;
    float distanceFromCamera = length(inLocalPosition);
    // Ondas geométricas só são úteis perto do observador. A malha distante
    // passa a ser exatamente plana antes que os LODs grossos consigam
    // transformar ondas pequenas em ondulações lentas e gigantes.
    float displacementFade =
        1.0 - smoothstep(190.0, 270.0, distanceFromCamera);

    float height;
    sampleSurface(worldXY, displacementFade, height, geometricNormal);
    surfacePosition = vec3(worldXY, height);

    opaqueClipPosition =
        scene.cameraViewProjection * vec4(surfacePosition, 1.0);
    opaqueClipPosition.y = -opaqueClipPosition.y;
    gl_Position = opaqueClipPosition;

    motionClipNow =
        scene.cameraViewProjectionUnjittered * vec4(surfacePosition, 1.0);
    motionClipNow.y = -motionClipNow.y;
    motionClipBefore =
        scene.previousCameraViewProjection * vec4(surfacePosition, 1.0);
    motionClipBefore.y = -motionClipBefore.y;
}
