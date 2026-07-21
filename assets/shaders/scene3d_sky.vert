#version 450

layout(location = 0) out vec2 clipPosition;

void main() {
    const vec2 positions[3] = vec2[](
        vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0)
    );
    clipPosition = positions[gl_VertexIndex];
    // Sem efeito pratico hoje (o pipeline do ceu roda com depthTestEnable e
    // depthWriteEnable desligados, ver skyDepth em createScene3DResources -
    // este valor nunca chega a ser escrito/testado), mas mantido consistente
    // com a convencao de profundidade INVERTIDA da camera principal (perto=1,
    // longe=0, ver Mat4::perspective) para nao confundir se o teste/escrita
    // de profundidade do ceu for religado no futuro.
    gl_Position = vec4(clipPosition, 0.000001, 1.0);
}
