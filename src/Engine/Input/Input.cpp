#include "Engine/Input/Input.hpp"

#include <algorithm>

namespace MatterEngine {

bool InputState::keyDown(Key key) const {
    const std::size_t index = static_cast<std::size_t>(key);
    return index < m_keys.size() && m_keys[index];
}

void InputState::setKey(Key key, bool down) {
    const std::size_t index = static_cast<std::size_t>(key);
    if (index < m_keys.size()) {
        m_keys[index] = down;
    }
}

void InputState::clear() {
    std::fill(m_keys.begin(), m_keys.end(), false);
}

} // namespace MatterEngine
