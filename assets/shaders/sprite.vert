#version 450

// Draws an arbitrary quad (not just fullscreen) by picking 4 corners out of
// push constants via gl_VertexIndex - no vertex/index buffer needed, same
// "vertexless" trick as blit's old fullscreen triangle, generalized to take
// any 4 screen-space corners. Used both for the final low-res-to-swapchain
// upscale (corners = the whole target) and for drawing pre-baked static
// textures like the grass field (corners = wherever that texture's world
// bounds currently project to on screen).
layout(push_constant) uniform SpritePush {
    vec2 corners[4]; // top-left, top-right, bottom-right, bottom-left, in the current target's pixel space
    float perspectiveDepths[4];
    vec2 viewportSize;
} push;

layout(location = 0) out vec2 uv;

const int cornerIndex[6] = int[](0, 1, 2, 0, 2, 3);
const vec2 cornerUV[4] = vec2[](vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));

void main() {
    int idx = cornerIndex[gl_VertexIndex];
    vec2 pixelPos = push.corners[idx];
    uv = cornerUV[idx];
    vec2 ndc = vec2(
        pixelPos.x / push.viewportSize.x * 2.0 - 1.0,
        pixelPos.y / push.viewportSize.y * 2.0 - 1.0);
    // Giving ground-plane vertices their camera-space depth lets the GPU's
    // normal perspective-correct interpolation map baked turf cleanly across
    // a trapezoid. UI/final-blit quads pass 1.0 for all four values.
    float perspectiveDepth = push.perspectiveDepths[idx];
    gl_Position = vec4(ndc * perspectiveDepth, 0.0, perspectiveDepth);
}
