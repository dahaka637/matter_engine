#pragma once

#include "Engine/Math/Vec2.hpp"
#include <array>
#include <cstddef>

namespace MatterEngine {

enum class Key : std::size_t {
    W,
    A,
    S,
    D,
    E,
    Q,
    R,
    V,
    Z,
    Escape,
    Insert,
    Quote,
    Tab,
    F,
    Space,
    LeftShift,
    RightShift,
    LeftControl,
    RightControl,
    LeftAlt,
    RightAlt,
    Count,
    Unknown = Count
};

enum class MouseButton {
    Left,
    Right,
    Middle,
    Unknown
};

enum class EventType {
    KeyDown,
    KeyUp,
    MouseMove,
    MouseButtonDown,
    MouseButtonUp,
    MouseWheel,
    WindowFocusGained,
    WindowFocusLost,
    WindowClose,
    Quit,
    Unknown
};

struct Event {
    EventType type = EventType::Unknown;
    Key key = Key::Unknown;
    MouseButton button = MouseButton::Unknown;
    Vec2 mousePosition;
    Vec2 mouseDelta;
    float wheelY = 0.0f;
    bool repeat = false;
};

class InputState final {
public:
    [[nodiscard]] bool keyDown(Key key) const;
    [[nodiscard]] Vec2 mousePosition() const { return m_mousePosition; }

    void setKey(Key key, bool down);
    void setMousePosition(Vec2 position) { m_mousePosition = position; }
    void clear();

private:
    std::array<bool, static_cast<std::size_t>(Key::Count)> m_keys {};
    Vec2 m_mousePosition;
};

} // namespace MatterEngine
