#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/RL/Contract/VectorLayout3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// O espelho do corpo (plano_claude_rl.md secao 8): o lado esquerdo vira o
// direito. O plano do espelho e o vertical que passa pela pelve no rumo dela:
// no referencial do rumo (x frente, y esquerda, z cima), y troca de sinal.
//
// O mapa sai das poses, nao da mao. Para cada eixo de cada junta, o sinal do
// eixo do outro lado e o que reproduz a pose refletida, medido nos pontos dos
// colisores (o giro de um link sem filho, como a cabeca ou a ponta, tambem
// aparece). O erro que sobra e a assimetria do corpo.

// O eixo do outro lado e o sinal.
struct MirrorAxis3D {
    std::uint32_t link = 0;
    float sign = 1.0f;
};

// Um layout espelhado: cada canal vem do canal source vezes scale mais offset.
struct MirrorChannelMap3D {
    std::vector<std::uint32_t> source;
    std::vector<float> scale;
    std::vector<float> offset;
    void apply(const float* in, float* out) const;
};

class Mirror3D {
public:
    explicit Mirror3D(const RLBody3D& body);

    // O link do outro lado (o mesmo no centro).
    [[nodiscard]] std::span<const std::uint32_t> linkCounterpart() const { return m_linkCounterpart; }
    // Por link e eixo (0 twist, 1 swing1, 2 swing2).
    [[nodiscard]] const MirrorAxis3D& axis(std::size_t link, std::size_t axis) const { return m_axes[link][axis]; }
    // Os eixos atuados: o indice do outro lado em RLBody3D::dofs e o sinal.
    [[nodiscard]] std::span<const std::uint32_t> dofCounterpart() const { return m_dofCounterpart; }
    [[nodiscard]] std::span<const float> dofSign() const { return m_dofSign; }
    // O quanto a pose espelhada fica longe da refletida no pior eixo (m) e a
    // assimetria da pose de referencia (todos os eixos em zero) (m).
    [[nodiscard]] float worstAxisError() const { return m_worstAxisError; }
    [[nodiscard]] float bindAsymmetry() const { return m_bindAsymmetry; }
    // A assimetria de cada link na pose de referencia (m): o colisor dele
    // contra o refletido do outro lado.
    [[nodiscard]] std::span<const float> linkAsymmetry() const { return m_linkAsymmetry; }
    // Os eixos em que o sinal nao se decide pelos pontos (o giro nao move nada).
    [[nodiscard]] std::span<const std::string> undecided() const { return m_undecided; }

    // As coordenadas de junta (por link) espelhadas.
    void mirrorCoordinates(std::span<const Vec3> in, std::vector<Vec3>& out) const;
    // Um vetor no referencial do rumo e um giro (pseudovetor) espelhados.
    [[nodiscard]] static Vec3 mirrorVector(Vec3 v) { return { v.x, -v.y, v.z }; }
    [[nodiscard]] static Vec3 mirrorSpin(Vec3 w) { return { -w.x, w.y, -w.z }; }
    // A orientacao da raiz espelhada no plano y = 0 do mundo (x frente).
    [[nodiscard]] Quaternion mirrorRoot(Quaternion root) const;

    // O espelho de um layout pelos nomes dos canais (a entrada do Corpo, o
    // critico, a acao, o estilo). Lanca num canal sem regra.
    [[nodiscard]] MirrorChannelMap3D layoutMap(const RLVectorLayout3D& layout) const;

private:
    RLBody3D m_body;
    std::vector<std::uint32_t> m_linkCounterpart;
    std::vector<std::array<MirrorAxis3D, 3>> m_axes;
    std::vector<std::uint32_t> m_dofCounterpart;
    std::vector<float> m_dofSign;
    std::vector<std::string> m_undecided;
    std::vector<float> m_linkAsymmetry;
    float m_worstAxisError = 0.0f;
    float m_bindAsymmetry = 0.0f;
};

} // namespace MatterEngine
