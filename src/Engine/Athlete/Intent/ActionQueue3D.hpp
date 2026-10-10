#pragma once

#include "Engine/Athlete/Ball/BallOutcome3D.hpp"

#include <cstdint>
#include <vector>

namespace MatterEngine {

// O ciclo de vida dos pedidos de bola (rework do manejo, 10/10/2026, Etapa
// 5): soltar o botao gera um pedido com id; ele sempre termina num desfecho
// registrado, com o motivo - nenhum some.
//  - Buscando: a bola ainda nao esta ao alcance de um gesto (prazo de
//    aquisicao).
//  - Comprometido: o pe, a superficie e o instante escolhidos; o gesto
//    comecou.
//  - Fila curta: um pedido em andamento e um guardado. Um novo substitui o
//    guardado; o em andamento so e substituido antes do compromisso.
enum class BallActionStage3D : std::uint8_t { Seeking, Committed, Done };
enum class BallActionOutcome3D : std::uint8_t {
    None,
    Touched,
    Expired,
    Replaced,
    Interrupted,
    Missed,
    Unreachable,
    InvalidTarget
};

[[nodiscard]] const char* ballActionOutcomeName3D(BallActionOutcome3D outcome);

struct BallActionRecord3D {
    std::uint32_t id = 0;
    BallOutcomeRequest3D intent;
    // A bola-alvo (o handle empacotado: o pedido nunca passa para outra).
    std::uint64_t ballKey = 0;
    BallActionStage3D stage = BallActionStage3D::Seeking;
    BallActionOutcome3D outcome = BallActionOutcome3D::None;
    const char* reason = "";
    // Desde o pedido (s) e o atraso ate o gesto comecar (s; -1 sem gesto).
    float age = 0.0f;
    float latency = -1.0f;
};

class ActionQueue3D {
public:
    // Um pedido novo: substitui o em andamento ainda sem compromisso, ou fica
    // guardado (no lugar de outro guardado).
    std::uint32_t push(const BallOutcomeRequest3D& intent, std::uint64_t ballKey);
    [[nodiscard]] BallActionRecord3D* active() { return m_hasActive ? &m_active : nullptr; }
    [[nodiscard]] const BallActionRecord3D* active() const { return m_hasActive ? &m_active : nullptr; }
    void advance(float deltaTime);
    // O em andamento termina; o guardado (se houver) assume.
    void finish(BallActionOutcome3D outcome, const char* reason);
    void clear();
    // Os ultimos desfechos (telemetria e sonda).
    [[nodiscard]] const std::vector<BallActionRecord3D>& history() const { return m_history; }

private:
    void archive(BallActionRecord3D record);

    BallActionRecord3D m_active;
    BallActionRecord3D m_queued;
    bool m_hasActive = false;
    bool m_hasQueued = false;
    std::uint32_t m_nextId = 1;
    std::vector<BallActionRecord3D> m_history;
};

} // namespace MatterEngine
