#include "Engine/Athlete/Intent/ActionQueue3D.hpp"

namespace MatterEngine {

const char* ballActionOutcomeName3D(BallActionOutcome3D outcome) {
    switch (outcome) {
    case BallActionOutcome3D::None: return "nenhum";
    case BallActionOutcome3D::Touched: return "tocou";
    case BallActionOutcome3D::Expired: return "expirou";
    case BallActionOutcome3D::Replaced: return "substituido";
    case BallActionOutcome3D::Interrupted: return "interrompido";
    case BallActionOutcome3D::Missed: return "errou";
    case BallActionOutcome3D::Unreachable: return "inalcancavel";
    case BallActionOutcome3D::InvalidTarget: return "alvo invalido";
    }
    return "?";
}

std::uint32_t ActionQueue3D::push(const BallOutcomeRequest3D& intent, std::uint64_t ballKey) {
    BallActionRecord3D record;
    record.id = m_nextId++;
    record.intent = intent;
    record.ballKey = ballKey;
    if (!m_hasActive) {
        m_active = record;
        m_hasActive = true;
    } else if (m_active.stage == BallActionStage3D::Seeking) {
        m_active.outcome = BallActionOutcome3D::Replaced;
        m_active.reason = "pedido novo antes do compromisso";
        archive(m_active);
        m_active = record;
    } else {
        if (m_hasQueued) {
            m_queued.outcome = BallActionOutcome3D::Replaced;
            m_queued.reason = "pedido novo no lugar do guardado";
            archive(m_queued);
        }
        m_queued = record;
        m_hasQueued = true;
    }
    return record.id;
}

void ActionQueue3D::advance(float deltaTime) {
    if (m_hasActive) m_active.age += deltaTime;
    if (m_hasQueued) m_queued.age += deltaTime;
}

void ActionQueue3D::finish(BallActionOutcome3D outcome, const char* reason) {
    if (!m_hasActive) return;
    m_active.stage = BallActionStage3D::Done;
    m_active.outcome = outcome;
    m_active.reason = reason;
    archive(m_active);
    m_hasActive = m_hasQueued;
    if (m_hasQueued) {
        // (O tempo do guardado conta daqui: ele esperou o pe liberar.)
        m_active = m_queued;
        m_active.age = 0.0f;
        m_hasQueued = false;
    }
}

void ActionQueue3D::clear() {
    if (m_hasActive) finish(BallActionOutcome3D::Interrupted, "fila limpa");
    if (m_hasActive) finish(BallActionOutcome3D::Interrupted, "fila limpa");
    m_hasActive = m_hasQueued = false;
}

void ActionQueue3D::archive(BallActionRecord3D record) {
    m_history.push_back(record);
    if (m_history.size() > 32) m_history.erase(m_history.begin());
}

} // namespace MatterEngine
