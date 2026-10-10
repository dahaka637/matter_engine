#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// Como fechar a ponta de um loft.
enum class Cap : std::uint8_t {
    Open,
    // Leque a partir do centro do anel (uma tampa plana).
    Flat,
    // Cone ate um ponto (os polos de um elipsoide, a ponta de um dedo).
    Point,
};

// O kit de modelagem: constroi a malha parte por parte, com precisao.
//
// Convencoes:
//  - aneis: pontos 2D (y, z) no plano de um referencial cujo x e o eixo do
//    loft, em sentido anti-horario vistos de +x; assim os quads entre aneis
//    ficam com a normal para fora;
//  - toda geometria vai para a parte e o material correntes;
//  - nada e soldado sozinho: weld() funde o que coincide.
class MeshBuilder {
public:
    explicit MeshBuilder(Mesh& mesh);

    // Parte corrente (criada na primeira vez) e a regiao do corpo dela.
    MeshBuilder& part(std::string_view name, std::string_view region = {});
    // A parte corrente forma uma superficie so com "other" (ver MeshPart):
    // a analise cobra a costura entre as duas.
    MeshBuilder& connectTo(std::string_view other);
    // Uma borda aberta de proposito na parte corrente (ex.: "cintura").
    MeshBuilder& opening(std::string_view name);
    // A parte corrente e raiz (o corpo ao qual tudo se liga).
    MeshBuilder& root(bool value = true);
    // Define (ou atualiza a cor de) um material e passa a usa-lo.
    MeshBuilder& material(std::string_view name, Color color);
    MeshBuilder& useMaterial(std::string_view name);
    [[nodiscard]] int currentPart() const { return m_part; }
    [[nodiscard]] int currentMaterial() const { return m_material; }
    [[nodiscard]] const Mesh& mesh() const { return m_mesh; }
    [[nodiscard]] Mesh& mesh() { return m_mesh; }

    std::uint32_t vertex(Vec3 position);
    void triangle(std::uint32_t a, std::uint32_t b, std::uint32_t c);
    // a, b, c, d em volta, anti-horario visto de fora (corta na diagonal
    // mais curta).
    void quad(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d);
    // Poligono convexo em leque, anti-horario visto de fora.
    void polygon(std::span<const std::uint32_t> loop);

    // Anel com os pontos do perfil no plano (y, z) do referencial.
    std::vector<std::uint32_t> ring(const Transform& frame, std::span<const Point2> profile);
    // Elipse com "count" pontos (raios no y e no z do referencial).
    std::vector<std::uint32_t> ellipseRing(const Transform& frame, float radiusY, float radiusZ, int count,
        float phaseRadians = 0.0f);
    // Quads entre dois aneis do mesmo tamanho (from antes de to no eixo).
    void bridge(std::span<const std::uint32_t> from, std::span<const std::uint32_t> to);
    // Fecha um anel: no inicio do loft a tampa olha para -x, no fim para +x.
    void cap(std::span<const std::uint32_t> ring, bool atStart, Cap kind, Vec3 tip = {});
    // Loft completo pelos aneis, com as tampas (tips para Cap::Point).
    void loft(std::span<const std::vector<std::uint32_t>> rings, Cap startCap, Cap endCap, Vec3 startTip = {},
        Vec3 endTip = {});

    // Primitivas (no referencial dado).
    void box(const Transform& frame, Vec3 halfExtents);
    void ellipsoid(const Transform& frame, Vec3 radii, int segments, int rings);
    void tube(Vec3 from, Vec3 to, float radiusFrom, float radiusTo, int segments, Cap caps);

    // Cria o lado espelhado (plano y = 0) das partes dadas: nomes com
    // esquerda/direita trocados; jointMirror mapeia as juntas do skin. Os
    // vertices divididos entre partes continuam divididos no espelho, os do
    // plano y = 0 sao os mesmos dos dois lados (a costura do meio) e uma
    // parte do centro (sem lado no nome) recebe a propria metade espelhada.
    void mirrorParts(std::span<const std::string> names, std::span<const int> jointMirror = {});
    // Funde vertices a menos de "tolerance" metros; samePart evita colar
    // partes diferentes. Remove triangulos degenerados e vertices soltos.
    std::size_t weld(float tolerance, bool samePart = true);
    void transformPart(int part, const Transform& transform);
    // Inverte a orientacao dos triangulos da parte.
    void flipPart(int part);
    // Remove os triangulos (e vertices que sobrarem) de uma parte.
    void removePart(std::string_view name);
    // Tira triangulos degenerados e vertices fora de triangulos (renumera).
    void compact();

private:
    std::uint16_t ensurePart(std::string_view name, std::string_view region);

    Mesh& m_mesh;
    int m_part = -1;
    int m_material = -1;
};

} // namespace MatterEngine::Modeling
