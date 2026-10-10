#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Physics/InflatedBall3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"

#include <memory>
#include <string>
#include <vector>

// O laboratorio de fisica dos objetos procedurais: uma cena propria (o chao,
// a rampa) com o objeto num roteiro - a queda, o arremesso, a rampa, a pilha
// e a bola de futebol contra ele - e as medidas de cada um. O inspetor de
// objetos roda ao vivo (advance) e os testes de uma vez (run).
namespace MatterEngine {

enum class ObjectTestKind3D : std::uint8_t { Drop, Throw, Ramp, Stack, BallShot };

struct ObjectTestSettings3D {
    ObjectTestKind3D kind = ObjectTestKind3D::Drop;
    // Queda: a altura da base (m) e a inclinacao de saida (graus).
    float height = 1.5f;
    float tiltDegrees = 12.0f;
    // Arremesso: a velocidade (m/s) e o giro (rad/s); a bola: a velocidade
    // dela (m/s).
    float speed = 6.0f;
    float spin = 4.0f;
    // Rampa (graus).
    float rampDegrees = 25.0f;
    // Pilha: quantos.
    int count = 3;
};

// Um corpo para desenhar (o objeto ou a bola) e uma peca parada do cenario
// (o chao, a rampa: caixas).
struct ObjectTestBody3D {
    Vec3 position;
    Quaternion orientation;
    bool ball = false;
    float radius = 0.0f;
};

struct ObjectTestStatic3D {
    Vec3 center;
    Quaternion orientation;
    Vec3 halfExtents;
};

struct ObjectTestMetrics3D {
    float elapsed = 0.0f;
    // A velocidade do primeiro toque no chao (m/s) e o quanto a parte mais
    // baixa subiu depois dele (m).
    float firstImpactSpeed = -1.0f;
    float bounceHeight = 0.0f;
    // Quando tudo parou (s; -1 ainda nao).
    float settleTime = -1.0f;
    // O primeiro objeto: o caminho no plano desde a saida (m) e, na rampa, o
    // quanto desceu ao longo dela.
    float travel = 0.0f;
    float slide = 0.0f;
    float peakSpeed = 0.0f;
    // A pilha: o maior desvio do de cima em relacao ao de baixo (m) e se caiu.
    float stackDrift = 0.0f;
    bool toppled = false;
    // A bola: a velocidade dela logo depois do toque e a maior do objeto.
    float ballSpeedAfter = -1.0f;
    float objectSpeedAfter = 0.0f;
    bool settled = false;
    // A face para baixo no fim: "base", "tampa" ou "lado".
    std::string restFace;
};

class ObjectPhysicsLab3D {
public:
    ObjectPhysicsLab3D(PhysicsEngine3D& engine, const MaterialLibrary& materials);
    ~ObjectPhysicsLab3D();
    ObjectPhysicsLab3D(const ObjectPhysicsLab3D&) = delete;
    ObjectPhysicsLab3D& operator=(const ObjectPhysicsLab3D&) = delete;

    // Monta o roteiro (o anterior sai). Falso com o erro.
    bool start(const Modeling::ProceduralObject& object, const ObjectTestSettings3D& settings, std::string* error = nullptr);
    void stop();
    // Avanca em passos de 1/120 s.
    void advance(float seconds);
    // De uma vez: ate tudo parar (e meio segundo mais) ou maximumSeconds.
    ObjectTestMetrics3D run(const Modeling::ProceduralObject& object, const ObjectTestSettings3D& settings, float maximumSeconds = 8.0f);

    [[nodiscard]] bool active() const { return m_scene != nullptr; }
    [[nodiscard]] const std::vector<ObjectTestBody3D>& bodies() const { return m_bodies; }
    [[nodiscard]] const std::vector<ObjectTestStatic3D>& statics() const { return m_statics; }
    [[nodiscard]] const ObjectTestMetrics3D& metrics() const { return m_metrics; }
    [[nodiscard]] const ObjectTestSettings3D& settings() const { return m_settings; }

private:
    void step();
    void refresh();
    [[nodiscard]] float lowestPoint(const PhysicsBodyState3D& state) const;

    PhysicsEngine3D& m_engine;
    const MaterialLibrary& m_materials;
    std::unique_ptr<PhysicsScene3D> m_scene;
    ObjectTestSettings3D m_settings;
    std::vector<PhysicsBodyHandle3D> m_objects;
    InflatedBallHandle3D m_ball;
    bool m_hasBall = false;
    float m_ballRadius = 0.11f;
    std::vector<Modeling::ObjectCollider> m_colliders;
    std::vector<ObjectTestBody3D> m_bodies;
    std::vector<ObjectTestStatic3D> m_statics;
    ObjectTestMetrics3D m_metrics;
    float m_accumulator = 0.0f;
    float m_stillSeconds = 0.0f;
    bool m_touched = false;
    bool m_ballHit = false;
    float m_ballSpeedBefore = 0.0f;
    Vec3 m_start;
    Vec3 m_rampDown;
    float m_stackHeight = 0.0f;
    float m_stackPiece = 0.0f;
};

} // namespace MatterEngine
