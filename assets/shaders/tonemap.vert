#version 450

// Triangulo unico cobrindo a tela inteira (sem vertex buffer) - mesmo truque
// usado em scene3d_sky.vert: os dois vertices fora de [-1,1] ficam fora do
// frustum e sao recortados pelo rasterizador, sobrando um retangulo cobrindo
// exatamente a viewport inteira.
layout(location = 0) out vec2 texCoord;

void main() {
    const vec2 positions[3] = vec2[](
        vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0)
    );
    vec2 clipPosition = positions[gl_VertexIndex];
    texCoord = clipPosition * 0.5 + 0.5;
    gl_Position = vec4(clipPosition, 0.0, 1.0);
}
