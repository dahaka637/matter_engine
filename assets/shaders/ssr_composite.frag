#version 450

// Ultimo passo do SSR: soma a radiancia resolvida (ssr_trace_resolve.frag,
// meia resolucao) de volta no HDR em resolucao cheia, pesada pela
// reflectancia por pixel (ver outReflectance em scene3d_mesh.frag/
// scene3d_sky.frag). Passe deliberadamente simples (2 leituras + 1
// multiplicacao) - o custo pesado (traçado + acumulo temporal) ja aconteceu
// no passe anterior, este so precisa espalhar o resultado de volta pra
// resolucao cheia. Reaproveita tonemap.vert como estagio de vertice.
//
// Escreve com blend ADITIVO (ver sceneSsrCompositePipeline em
// VulkanDevice.cpp) direto no MESMO alvo HDR que o passe opaco (cor+direta,
// sem especular ambiente - ver scene3d_mesh.frag) ja preencheu: soma, nunca
// substitui.

layout(set = 0, binding = 0) uniform sampler2D reflectanceMap;
layout(set = 0, binding = 1) uniform sampler2D ssrResolvedMap;

layout(location = 0) in vec2 texCoord;
layout(location = 0) out vec4 outColor;

void main() {
    vec3 reflectance = texture(reflectanceMap, texCoord).rgb;
    vec3 radiance = texture(ssrResolvedMap, texCoord).rgb;
    // Alfa 0: o blend aditivo (srcAlpha=dstAlpha=UM) preserva o alfa 1.0 que
    // o passe opaco ja escreveu, em vez de zera-lo.
    outColor = vec4(reflectance * radiance, 0.0);
}
