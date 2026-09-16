#version 450

layout(location = 0) out vec2 clipPosition;

void main() {
    const vec2 positions[3] = vec2[](
        vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0)
    );
    clipPosition = positions[gl_VertexIndex];
    // A camera usa profundidade invertida (perto=1, longe=0). O prepass limpa
    // o depth para exatamente zero; com compare EQUAL o shader caro do ceu
    // so roda nos pixels em que nenhuma geometria foi desenhada.
    gl_Position = vec4(clipPosition, 0.0, 1.0);
}
