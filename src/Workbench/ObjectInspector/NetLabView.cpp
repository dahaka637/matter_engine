#include "Workbench/ObjectInspector/NetLabView.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Workbench {

using namespace Modeling;

namespace {

constexpr int ShadowSides = 12;
constexpr float ShadowLift = 0.003f;
constexpr float Pi = 3.14159265f;

std::uint16_t materialIndex(Mesh& mesh, const char* name, Color color) {
    const int found = mesh.findMaterial(name);
    if (found >= 0) return static_cast<std::uint16_t>(found);
    mesh.materials.push_back({ name, color, -1 });
    return static_cast<std::uint16_t>(mesh.materials.size() - 1);
}

} // namespace

NetLabView::NetLabView(PhysicsEngine3D& physics, const MaterialLibrary& materials) : m_lab(physics, materials) {}

bool NetLabView::start(const ProceduralObject& object, std::string* error) {
    if (!m_lab.start(object, error)) {
        stop();
        return false;
    }
    rebuild();
    return true;
}

void NetLabView::stop() {
    m_lab.stop();
    m_mesh = {};
    m_local.clear();
    m_positions.clear();
    m_pieces.clear();
    m_wirePoints.clear();
    m_bodyCount = 0;
}

bool NetLabView::update(float seconds) {
    if (!m_lab.active()) return false;
    const std::vector<Vec3> before = [&] {
        std::vector<Vec3> positions;
        for (const NetLabBody3D& body : m_lab.bodies()) positions.push_back(body.position);
        return positions;
    }();
    const bool wasAwake = !m_lab.netStatus().sleeping;
    m_lab.advance(seconds);
    if (m_lab.bodies().size() != m_bodyCount) {
        rebuild();
        return true;
    }
    pose();
    // Mudou: a rede acordada (ou que acabou de dormir) ou um corpo andou.
    bool moved = wasAwake || !m_lab.netStatus().sleeping;
    for (std::size_t k = 0; !moved && k < before.size() && k < m_lab.bodies().size(); ++k)
        moved = (m_lab.bodies()[k].position - before[k]).lengthSquared() > 1e-8f;
    return moved;
}

void NetLabView::shoot() {
    m_lab.shoot(m_shot);
}

void NetLabView::shootRandom() {
    const auto next = [&] {
        m_random = m_random * 1664525u + 1013904223u;
        return static_cast<float>((m_random >> 8) & 0xFFFFu) / 65535.0f;
    };
    NetShotSettings3D shot = m_shot;
    shot.aimAcross = next() * 2.0f - 1.0f;
    shot.aimHeight = 0.05f + next() * 0.9f;
    shot.speed = 14.0f + next() * 20.0f;
    shot.angleDegrees = (next() * 2.0f - 1.0f) * 35.0f;
    shot.sideSpin = (next() * 2.0f - 1.0f) * 8.0f;
    m_lab.shoot(shot);
}

void NetLabView::throwObject(const ProceduralObject& object) {
    m_lab.throwObject(object, m_shot);
}

void NetLabView::clearBodies() {
    m_lab.clearBodies();
    rebuild();
}

void NetLabView::resetNet() {
    m_lab.reset();
    rebuild();
}

void NetLabView::rebuild() {
    m_mesh = {};
    m_local.clear();
    m_pieces.clear();
    m_bodyCount = m_lab.bodies().size();
    const ProceduralObject* object = m_lab.object();
    if (object == nullptr) return;
    Mesh& mesh = m_mesh;
    mesh.parts = { MeshPart { "cena", "", {}, {}, true } };
    // Uma peca: as posicoes no corpo (ou no mundo, a trave) e os triangulos,
    // toda num material.
    const auto append = [&](const Mesh& source, std::uint16_t material, Vec3 offset) {
        const auto first = static_cast<std::uint32_t>(m_local.size());
        for (const Vec3& p : source.positions) m_local.push_back(p + offset);
        for (const Triangle& triangle : source.triangles) {
            mesh.triangles.push_back({ triangle[0] + first, triangle[1] + first, triangle[2] + first });
            mesh.trianglePart.push_back(0);
            mesh.triangleMaterial.push_back(material);
        }
        return static_cast<std::size_t>(first);
    };
    const Color metal = object->mesh.materials.empty() ? rgb(242, 242, 238) : object->mesh.materials.front().color;
    append(object->mesh, materialIndex(mesh, "trave", metal), m_lab.objectPosition());
    const std::uint16_t ball = materialIndex(mesh, "bola", rgb(236, 236, 230));
    const std::uint16_t wood = materialIndex(mesh, "objeto", rgb(198, 132, 64));
    const std::uint16_t shade = materialIndex(mesh, "sombra", rgb(4, 6, 9));
    for (const NetLabBody3D& body : m_lab.bodies()) {
        Piece piece;
        if (body.ball) {
            Mesh sphere;
            MeshBuilder(sphere).part("bola").material("bola", rgb(236, 236, 230)).ellipsoid({}, { body.radius, body.radius, body.radius }, 20, 14);
            piece.first = append(sphere, ball, {});
            piece.count = sphere.positions.size();
            piece.shadowRadius = body.radius;
        } else if (body.object < m_lab.thrownObjects().size()) {
            const ProceduralObject& thrown = m_lab.thrownObjects()[body.object];
            piece.first = append(thrown.mesh, wood, {});
            piece.count = thrown.mesh.positions.size();
            piece.shadowRadius = 0.5f * std::max(thrown.size.x, thrown.size.y);
        }
        m_pieces.push_back(piece);
    }
    // As sombras (um leque no chao por corpo; o lugar sai em pose()).
    for (Piece& piece : m_pieces) {
        piece.shadow = m_local.size();
        const auto first = static_cast<std::uint32_t>(piece.shadow);
        for (int k = 0; k <= ShadowSides; ++k) m_local.emplace_back();
        for (int k = 0; k < ShadowSides; ++k) {
            mesh.triangles.push_back({ first, first + 1 + static_cast<std::uint32_t>(k), first + 1 + static_cast<std::uint32_t>((k + 1) % ShadowSides) });
            mesh.trianglePart.push_back(0);
            mesh.triangleMaterial.push_back(shade);
        }
    }
    mesh.positions = m_local;
    m_positions = m_local;
    if (object->net) {
        m_wires.color = object->net->color;
        m_wires.widthMeters = object->net->twineDiameter;
    }
    pose();
}

void NetLabView::pose() {
    const std::vector<NetLabBody3D>& bodies = m_lab.bodies();
    for (std::size_t b = 0; b < bodies.size() && b < m_pieces.size(); ++b) {
        const Piece& piece = m_pieces[b];
        for (std::size_t i = piece.first; i < piece.first + piece.count; ++i)
            m_positions[i] = bodies[b].position + bodies[b].orientation.rotate(m_local[i]);
        // A sombra: um disco no chao, embaixo do corpo (some longe dele).
        const Vec3 center { bodies[b].position.x, bodies[b].position.y, ShadowLift };
        const float radius = bodies[b].position.z > 4.0f ? 0.0f : piece.shadowRadius;
        m_positions[piece.shadow] = center;
        for (int k = 0; k < ShadowSides; ++k) {
            const float angle = 2.0f * Pi * static_cast<float>(k) / static_cast<float>(ShadowSides);
            m_positions[piece.shadow + 1 + static_cast<std::size_t>(k)] = center + Vec3 { std::cos(angle), std::sin(angle), 0.0f } * radius;
        }
    }
    // Os fios de verdade na posicao de agora (sobre a malha da fisica).
    m_wirePoints.clear();
    const std::span<const Vec3> knots = m_lab.netPositions();
    const ProceduralObject* object = m_lab.object();
    if (object != nullptr && object->net && knots.size() == object->net->knots.size()) {
        drawnNetKnots(*object->net, knots, m_drawn);
        m_wirePoints.reserve(object->net->drawnSegments.size() * 2);
        for (const auto& [a, b] : object->net->drawnSegments) {
            m_wirePoints.push_back(m_drawn[a]);
            m_wirePoints.push_back(m_drawn[b]);
        }
    }
    m_wires.points = m_wirePoints;
}

void NetLabView::fill(InspectionScene& scene) const {
    if (!m_lab.active()) return;
    scene.mesh = &m_mesh;
    scene.meshPositions = m_positions;
    scene.wires = std::span<const InspectionWires>(&m_wires, 1);
}

Box3 NetLabView::bounds() const {
    Box3 box;
    const ProceduralObject* object = m_lab.object();
    if (object == nullptr) {
        box.include({ -1.0f, -1.0f, 0.0f });
        box.include({ 1.0f, 1.0f, 1.0f });
        return box;
    }
    const Vec3 half = object->size * 0.5f;
    box.include(m_lab.objectPosition() - half);
    box.include(m_lab.objectPosition() + half);
    box.include({ 0.4f, 0.0f, 0.0f });
    return box;
}

} // namespace MatterEngine::Workbench
