#version 450

layout(location = 0) in vec2 inPosition;
layout(location = 1) in vec4 inColor;

layout(push_constant) uniform FrameConstants {
    vec2 viewportSize;
} frame;

layout(location = 0) out vec4 vertexColor;

void main() {
    // Vulkan's NDC is already Y-down (top = -1, bottom = +1), matching the
    // top-left-origin, Y-down pixel space every screen<->world conversion in
    // the game already assumes (mouse coordinates, worldToScreen/
    // screenToWorld, ImGui). A "1.0 - ..." flip here would mirror the whole
    // Render2D-drawn world vertically against that convention - invisible on
    // a vertically symmetric pitch, but wrong for anything direction-
    // dependent (character facing toward the cursor).
    vec2 ndc = vec2(
        inPosition.x / frame.viewportSize.x * 2.0 - 1.0,
        inPosition.y / frame.viewportSize.y * 2.0 - 1.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
    vertexColor = inColor;
}
