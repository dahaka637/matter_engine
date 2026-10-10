#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Mesh/MeshRelief.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <vector>

// Pecas finas deitadas sobre a malha que ja existe (cadarco, fitas, bainhas):
// cada tira le a altura de verdade do que esta embaixo (os triangulos, nao a
// forma lisa) e passa com folga, sem atravessar nada.
namespace MatterEngine::Modeling {

class MeshBuilder;

// Uma malha pequena no espaco local de uma peca: montada aqui, usada como
// apoio para as proximas e so depois emitida no MeshBuilder.
struct LocalMesh {
    std::vector<Vec3> positions;
    std::vector<Triangle> triangles;

    std::uint32_t vertex(Vec3 position);
    // a, b, c, d em volta, anti-horario visto de fora (corta em a-c).
    void quad(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d);
    void append(const LocalMesh& other);
};

// Os triangulos de firstTriangle ate o fim da malha, no espaco local.
[[nodiscard]] LocalMesh localMeshFrom(const Mesh& mesh, std::size_t firstTriangle, const Transform& toLocal);
// Os triangulos dados (indices na malha), no espaco local.
[[nodiscard]] LocalMesh localMeshOf(const Mesh& mesh, std::span<const std::uint32_t> triangles, const Transform& toLocal);
// Emite a malha local no builder (parte e material correntes) pelo referencial.
void emitLocalMesh(MeshBuilder& builder, const Transform& frame, const LocalMesh& mesh);

// O que esta por baixo, consultado de cima (+z): camadas de triangulos no
// espaco local. A primeira e a superficie (a inclinacao das tiras vem dela).
// Uma camada "base" e onde a tira encaixa (pode afundar nela, ver
// StrapSpec::hostClearance); nas outras ela apoia com folga.
class DrapeField {
public:
    void addLayer(LocalMesh mesh, bool host = false);
    // A altura do mais alto em (x, y) (nada embaixo = nullopt).
    [[nodiscard]] std::optional<float> height(float x, float y) const;
    // So a primeira camada.
    [[nodiscard]] std::optional<float> surfaceHeight(float x, float y) const;
    // So as camadas base.
    [[nodiscard]] std::optional<float> hostHeight(float x, float y) const;
    // A altura minima da face de baixo de uma tira em (x, y): cada camada com
    // a sua folga (base ou empilhada).
    [[nodiscard]] std::optional<float> requirement(float x, float y, float hostClearance, float clearance) const;
    // Os triangulos de todas as camadas cuja projecao de cima cai na caixa
    // (x, y) dada (e se a camada e base).
    void forEachTriangleNear(Point2 minimum, Point2 maximum,
        const std::function<void(const std::array<Vec3, 3>&, bool host)>& visit) const;

private:
    struct Layer {
        LocalMesh mesh;
        std::unique_ptr<TriangleBvh> bvh;
        // A caixa de cada triangulo vista de cima: minimo e maximo em (x, y).
        std::vector<std::array<Point2, 2>> bounds;
        bool host = false;
    };
    [[nodiscard]] static std::optional<float> layerHeight(const Layer& layer, float x, float y);

    std::vector<std::unique_ptr<Layer>> m_layers;
};

// Uma tira de secao retangular.
struct StrapSpec {
    float width = 0.006f;
    float thickness = 0.0015f;
    // A folga minima entre a face de baixo e o que estiver embaixo (nas
    // camadas empilhadas) e nas camadas base: negativa = encaixa (afunda).
    float clearance = 0.0003f;
    float hostClearance = 0.0003f;
    // As pontas descem ("dive" metros de cada ponta) ate deitar no que tem
    // embaixo (menos "diveDepth"): a tira esticada sai do arco e chega ao
    // ilhos.
    float dive = 0.0f;
    float diveDepth = 0.0f;
    // Esticada: passa em arco liso por cima do que tiver embaixo (como um
    // cadarco puxado); senao acompanha a superficie.
    bool taut = false;
    // A largura nas pontas que mergulham (0 = a mesma): afina ao longo do
    // mergulho (duas pontas lado a lado nao se sobrepoem).
    float endWidth = 0.0f;
};

// A tira ao longo do caminho (x, y no espaco local, ja amostrado), deitada
// sobre o campo e inclinada com a superficie; as pontas fechadas.
[[nodiscard]] LocalMesh drapeStrap(const DrapeField& field, std::span<const Point2> path, const StrapSpec& spec);
// Uma tira com as duas pontas entrando na base (o cadarco no ilhos): os
// aneis do corpo, todos por cima, e em cada ponta o furo onde ela entra (os
// 4 cantos na base: o fundo desce reto do ultimo anel, o topo inclina e
// entra mais adiante). Cantos: 0 baixo-esquerda, 1 cima-esquerda, 2
// cima-direita, 3 baixo-direita.
struct WeldedStrap {
    std::vector<std::array<Vec3, 4>> rings;
    std::array<Vec3, 4> startHole {};
    std::array<Vec3, 4> endHole {};
    bool ok = false;
};

// A tira deitada como drapeStrap (spec.dive desce as pontas ate "endLift"
// acima do apoio); cada ponta entra no furo, que vai de um quarto de
// "endLength" a "endLength" adiante do ultimo anel. Falha (ok = false) se o
// fundo de uma ponta ficar abaixo da base.
[[nodiscard]] WeldedStrap drapeWeldedStrap(const DrapeField& field, std::span<const Point2> path, const StrapSpec& spec,
    float endLength, float endLift);
// A tira como malha (os furos das pontas abertos): apoio para as proximas.
[[nodiscard]] LocalMesh weldedStrapMesh(const WeldedStrap& strap);
// Emite a tira no builder (parte e material correntes; "frame" leva do
// espaco local ao do modelo), costurada nos furos ja recortados (a volta de
// cada furo com os cortes e os 4 cantos dados, de ImprintResult). Nas pontas
// as paredes do lado saem em leque do canto de cima, o topo desce ate a
// borda de fora do furo e o fundo desce coluna a coluna (a aresta de baixo
// do anel dividida acima de cada corte): nada rente a superficie.
void emitWeldedStrap(MeshBuilder& builder, const Transform& frame, const WeldedStrap& strap,
    std::span<const std::uint32_t> startLoop, std::span<const std::uint32_t> startCorners,
    std::span<const std::uint32_t> endLoop, std::span<const std::uint32_t> endCorners);
// Os triangulos entre dois aneis de tira (cantos 0..3), cortados como a
// checagem de folga os confere (a mesma diagonal em cima e embaixo).
template <typename Emit>
void strapSegment(const std::array<std::uint32_t, 4>& from, const std::array<std::uint32_t, 4>& to, Emit&& emit) {
    for (std::size_t k = 0; k < 4; ++k) {
        const std::size_t next = (k + 1) % 4;
        if (k == 1) {
            emit(from[next], to[next], to[k]);
            emit(from[next], to[k], from[k]);
        } else {
            emit(from[k], from[next], to[next]);
            emit(from[k], to[next], to[k]);
        }
    }
}
// A peca levada do espaco local para o do modelo.
[[nodiscard]] ReliefPiece transformedRelief(const ReliefPiece& piece, const Transform& frame);
// A reta de a ate b em pedacos de no maximo "step" metros (pelo menos 2).
[[nodiscard]] std::vector<Point2> sampleLine(Point2 a, Point2 b, float step);
// O mesmo, com pedacos de "endStep" nos ultimos "endLength" de cada ponta
// (o mergulho das pontas bem amostrado).
[[nodiscard]] std::vector<Point2> sampleLine(Point2 a, Point2 b, float step, float endStep, float endLength);
// Ergue uma peca rigida ate a folga exata, sem alterar a forma das facetas.
float liftAboveSurface(LocalMesh& mesh, const DrapeField& field, float clearance);
[[nodiscard]] float surfaceClearanceDeficit(const std::array<Vec3, 3>& face, const DrapeField& field, float clearance);
// A altura da camada base em (x, y) vista de baixo (-z para +z): a primeira
// face por baixo (a sola vista de baixo, para encaixar as travas).
[[nodiscard]] std::optional<float> heightFromBelow(const LocalMesh& mesh, float x, float y);

} // namespace MatterEngine::Modeling
