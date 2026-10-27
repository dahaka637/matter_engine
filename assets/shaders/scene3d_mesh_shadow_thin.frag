#version 450

// A sombra da geometria fina (os fios da rede). O vertex shader de sombra
// engrossa o fio ate ~0,7 texel da cascata (mais fino, ele virava pontilhado
// piscando); aqui so a fracao do fio de verdade bloqueia a luz, sorteada por
// um hash da posicao no mundo (parado, o fio da sempre o mesmo desenho; nao
// cintila). O filtro da sombra junta os pontos numa linha fraca e macia, como
// a sombra de um fio de milimetros ao sol.
layout(location = 0) in vec3 worldPosition;
layout(location = 1) flat in float coverage;

float hash(vec3 p) {
    p = fract(p * 0.1031);
    p += dot(p, p.yzx + 33.33);
    return fract((p.x + p.y) * p.z);
}

void main() {
    if (hash(floor(worldPosition / 0.003)) >= coverage) {
        discard;
    }
}
