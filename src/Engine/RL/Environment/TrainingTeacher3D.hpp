#pragma once

#include "Engine/Physics/PhysicsScene3D.hpp"

#include <cstddef>
#include <cstdint>

namespace MatterEngine {

// Um mundo de treino visto pelo professor: a cena (so dele), o boneco, o
// centro do mundo e o rumo do corpo no comeco do episodio.
struct TrainingTeacherWorld3D {
    std::size_t index = 0;
    PhysicsScene3D* scene = nullptr;
    RagdollHandle3D ragdoll;
    Vec3 origin;
    float heading = 0.0f;
};

// Um professor dos mundos de treino: outro controle conduzindo o corpo no
// lugar da politica (a locomocao fisica do jogo, em RL/Teacher). O lote segue
// calculando a observacao e a recompensa de sempre; a acao do professor, em
// unidades do RL (o alvo que daria o mesmo torque), e o rotulo que a politica
// imita. Mundos diferentes podem rodar em paralelo; o mesmo mundo, nunca.
class TrainingTeacher3D {
public:
    virtual ~TrainingTeacher3D() = default;
    // Uma vez por mundo, logo depois de criado (a cena so tem ele).
    virtual void attach(const TrainingTeacherWorld3D& world) = 0;
    // O comeco de cada episodio, com o corpo ja na pose inicial.
    virtual void beginEpisode(const TrainingTeacherWorld3D& world, const RagdollState3D& state, std::uint64_t seed) = 0;
    // Antes de cada passo de fisica: o professor manda no corpo (os alvos das
    // juntas e as forcas dele) pela cena.
    virtual void control(const TrainingTeacherWorld3D& world, const RagdollState3D& state, float deltaTime) = 0;
    // O que ele tenta agora: a velocidade no plano (m/s, no mundo) e o giro
    // (rad/s, em torno da vertical), em x, y e z.
    [[nodiscard]] virtual Vec3 command(std::size_t world) const = 0;
    // A fase da passada no relogio do RL (o pe esquerdo apoia de 0 a 0,5 e o
    // direito de 0,5 a 1); negativa sem passada.
    [[nodiscard]] virtual float gaitPhase(std::size_t world) const = 0;
};

} // namespace MatterEngine
