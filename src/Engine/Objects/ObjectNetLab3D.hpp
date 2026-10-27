#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <memory>
#include <span>
#include <string>
#include <vector>

// O laboratorio da trave (um objeto com rede): uma cena propria com o campo
// (grama), a trave fixa com a rede ja assentada, e os chutes - a bola inflada
// sai do campo rumo a um ponto do gol, com a velocidade e o efeito dados - e o
// arremesso de outro objeto (a caixa) contra a rede. Mede o chute: a
// velocidade na linha, se foi gol, o quanto a rede estufou, quando a bola
// parou, e o custo da rede por tick. O inspetor de objetos roda ao vivo
// (advance); os testes, tick a tick.
namespace MatterEngine {

struct NetShotSettings3D {
    // m/s, e de onde: a distancia da linha (m) e o angulo (graus; 0 de
    // frente, positivo vindo da esquerda do gol).
    float speed = 25.0f;
    float distance = 11.0f;
    float angleDegrees = 0.0f;
    // O ponto do gol: de -1 (o poste da direita) a 1 (o da esquerda) e de 0
    // (o chao) a 1 (o travessao), a bola inteira dentro do vao.
    float aimAcross = 0.0f;
    float aimHeight = 0.5f;
    // Efeito (rad/s em volta da vertical: a curva) e por cima (em volta do
    // eixo lateral: + e a bola caindo mais).
    float sideSpin = 0.0f;
    float topSpin = 0.0f;
};

struct NetShotMetrics3D {
    int shots = 0;
    // O ultimo chute: a velocidade quando cruzou a frente do gol (m/s; -1
    // ainda nao), se entrou inteira (gol), o quanto a rede saiu do lugar (o no
    // que mais andou, m) e quando a bola parou depois da linha (s; -1 nao).
    float speedAtLine = -1.0f;
    bool goal = false;
    bool crossed = false;
    float bulge = 0.0f;
    float stopTime = -1.0f;
    float elapsed = 0.0f;
    // A rede: o custo do ultimo tick, o pico e a media do chute (ms), se esta
    // dormindo e quanto o fio mais esticado esticou no chute.
    float netMilliseconds = 0.0f;
    float netPeakMilliseconds = 0.0f;
    float netAverageMilliseconds = 0.0f;
    float peakStrain = 0.0f;
    bool netSleeping = false;
};

// Um corpo para desenhar: a bola (radius) ou um objeto arremessado.
struct NetLabBody3D {
    Vec3 position;
    Quaternion orientation;
    bool ball = false;
    float radius = 0.0f;
    // O objeto (o arremessado), no indice de thrownObjects().
    std::size_t object = 0;
};

class ObjectNetLab3D {
public:
    ObjectNetLab3D(PhysicsEngine3D& engine, const MaterialLibrary& materials);
    ~ObjectNetLab3D();
    ObjectNetLab3D(const ObjectNetLab3D&) = delete;
    ObjectNetLab3D& operator=(const ObjectNetLab3D&) = delete;

    // Monta a cena com o objeto (precisa ter rede) e assenta a rede. Falso com
    // o erro.
    bool start(const Modeling::ProceduralObject& object, std::string* error = nullptr);
    void stop();
    // A rede de volta ao repouso e sem bolas nem objetos.
    void reset();
    void shoot(const NetShotSettings3D& shot);
    // Um objeto (sem rede: a caixa) atirado contra o gol, do mesmo jeito.
    void throwObject(const Modeling::ProceduralObject& object, const NetShotSettings3D& shot);
    void clearBodies();
    // Avanca em passos de 1/120 s.
    void advance(float seconds);
    void step();

    [[nodiscard]] bool active() const { return m_scene != nullptr; }
    [[nodiscard]] const Modeling::ProceduralObject* object() const { return m_object.get(); }
    // A pose do objeto no mundo (o centro dele; a frente do gol em x = 0, o
    // chao em z = 0) e a linha do gol (x do fundo da linha).
    [[nodiscard]] Vec3 objectPosition() const { return m_objectPosition; }
    [[nodiscard]] float goalLineX() const { return m_goalLineX; }
    [[nodiscard]] std::span<const Vec3> netPositions() const;
    [[nodiscard]] std::span<const Vec3> netRest() const { return m_rest; }
    [[nodiscard]] const std::vector<NetLabBody3D>& bodies() const { return m_bodies; }
    [[nodiscard]] const std::vector<Modeling::ProceduralObject>& thrownObjects() const { return m_thrown; }
    [[nodiscard]] const NetShotMetrics3D& metrics() const { return m_metrics; }
    [[nodiscard]] NetStatus3D netStatus() const;
    [[nodiscard]] PhysicsScene3D* scene() { return m_scene.get(); }
    [[nodiscard]] NetHandle3D net() const { return m_net; }
    [[nodiscard]] const std::vector<InflatedBallHandle3D>& balls() const { return m_balls; }

    // A velocidade inicial que leva a bola de "from" ate o ponto "target" na
    // frente do gol com a velocidade dada (a elevacao acertada pelo ar da bola:
    // o arrasto e o Magnus do modelo dela).
    [[nodiscard]] static Vec3 launchVelocity(Vec3 from, Vec3 target, float speed, Vec3 spin);

private:
    void refresh();

    PhysicsEngine3D& m_engine;
    const MaterialLibrary& m_materials;
    std::unique_ptr<PhysicsScene3D> m_scene;
    std::unique_ptr<Modeling::ProceduralObject> m_object;
    Vec3 m_objectPosition;
    float m_goalLineX = 0.0f;
    float m_postInnerY = 3.66f;
    float m_crossbarBottom = 2.44f;
    NetHandle3D m_net;
    std::vector<Vec3> m_rest;
    std::vector<InflatedBallHandle3D> m_balls;
    std::vector<PhysicsBodyHandle3D> m_thrownBodies;
    std::vector<Modeling::ProceduralObject> m_thrown;
    std::vector<NetLabBody3D> m_bodies;
    NetShotMetrics3D m_metrics;
    float m_accumulator = 0.0f;
    float m_stillSeconds = 0.0f;
    float m_costSum = 0.0f;
    int m_costTicks = 0;
};

} // namespace MatterEngine
