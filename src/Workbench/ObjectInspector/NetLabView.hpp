#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Objects/ObjectNetLab3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"

#include <memory>
#include <vector>

namespace MatterEngine::Workbench {

// A trave na tela "Objetos" (um objeto com rede): o laboratorio dela (a cena
// com o campo, a trave, a rede assentada e os chutes) e o que o viewport
// desenha - a trave, as bolas (com a sombra no chao), os objetos atirados e
// os fios da rede na posicao de agora. Sem interface: a tela desenha o painel
// e repassa os comandos.
class NetLabView {
public:
    NetLabView(PhysicsEngine3D& physics, const MaterialLibrary& materials);

    // Monta a cena com o objeto (a rede assenta); falso com o erro.
    bool start(const Modeling::ProceduralObject& object, std::string* error = nullptr);
    void stop();
    [[nodiscard]] bool active() const { return m_lab.active(); }
    // Avanca a cena (o tempo ja com o ritmo) e atualiza o desenho; devolve se
    // algo se mexeu.
    bool update(float seconds);

    void shoot();
    // Um chute com a mira e a forca sorteadas dentro do gol.
    void shootRandom();
    void throwObject(const Modeling::ProceduralObject& object);
    void clearBodies();
    // A rede de volta ao repouso, sem bolas.
    void resetNet();

    [[nodiscard]] NetShotSettings3D& shot() { return m_shot; }
    [[nodiscard]] const ObjectNetLab3D& lab() const { return m_lab; }
    // A cena para o renderizador (a malha, as posicoes de agora e os fios).
    void fill(Modeling::InspectionScene& scene) const;
    // O enquadramento: a trave e um pouco do campo na frente.
    [[nodiscard]] Modeling::Box3 bounds() const;

private:
    void rebuild();
    void pose();

    ObjectNetLab3D m_lab;
    NetShotSettings3D m_shot;
    std::uint32_t m_random = 12345u;
    // A malha: a trave parada, depois um pedaco por corpo (bola ou objeto) e
    // a sombra de cada um.
    Modeling::Mesh m_mesh;
    std::vector<Vec3> m_local;
    std::vector<Vec3> m_positions;
    struct Piece {
        std::size_t first = 0;
        std::size_t count = 0;
        std::size_t shadow = 0;
        float shadowRadius = 0.0f;
    };
    std::vector<Piece> m_pieces;
    std::size_t m_bodyCount = 0;
    std::vector<Vec3> m_drawn;
    std::vector<Vec3> m_wirePoints;
    Modeling::InspectionWires m_wires;
};

} // namespace MatterEngine::Workbench
