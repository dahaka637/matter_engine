#include "Engine/Platform/SDL3/SDLInputAdapter.hpp"

#include <SDL3/SDL.h>

namespace MatterEngine::Platform {

namespace {

Key toKey(SDL_Scancode scancode) {
    switch (scancode) {
    case SDL_SCANCODE_W: return Key::W;
    case SDL_SCANCODE_A: return Key::A;
    case SDL_SCANCODE_S: return Key::S;
    case SDL_SCANCODE_D: return Key::D;
    case SDL_SCANCODE_E: return Key::E;
    case SDL_SCANCODE_Q: return Key::Q;
    case SDL_SCANCODE_R: return Key::R;
    case SDL_SCANCODE_V: return Key::V;
    case SDL_SCANCODE_Z: return Key::Z;
    case SDL_SCANCODE_ESCAPE: return Key::Escape;
    case SDL_SCANCODE_INSERT: return Key::Insert;
    case SDL_SCANCODE_APOSTROPHE: return Key::Quote;
    case SDL_SCANCODE_TAB: return Key::Tab;
    case SDL_SCANCODE_F: return Key::F;
    case SDL_SCANCODE_SPACE: return Key::Space;
    case SDL_SCANCODE_LSHIFT: return Key::LeftShift;
    case SDL_SCANCODE_RSHIFT: return Key::RightShift;
    case SDL_SCANCODE_LCTRL: return Key::LeftControl;
    case SDL_SCANCODE_RCTRL: return Key::RightControl;
    case SDL_SCANCODE_LALT: return Key::LeftAlt;
    case SDL_SCANCODE_RALT: return Key::RightAlt;
    default: return Key::Unknown;
    }
}

MouseButton toMouseButton(Uint8 button) {
    switch (button) {
    case SDL_BUTTON_LEFT: return MouseButton::Left;
    case SDL_BUTTON_RIGHT: return MouseButton::Right;
    case SDL_BUTTON_MIDDLE: return MouseButton::Middle;
    default: return MouseButton::Unknown;
    }
}

} // namespace

namespace {

// SDL3 reports mouse coordinates in logical window space, which only equals
// physical pixels at 100% OS display scaling. The renderer (and every
// world<->screen conversion in the game) measures extent in physical pixels
// via SDL_GetWindowSizeInPixels, so mouse coordinates must be scaled by the
// same factor or the two disagree on "how big the screen is" whenever
// Windows scaling isn't exactly 100%.
float pixelDensity(SDL_Window* window) {
    const float density = window != nullptr ? SDL_GetWindowPixelDensity(window) : 1.0f;
    // SDL can report 0 for a window that hasn't been fully realized yet (or on
    // driver edge cases); a 0 multiplier would collapse every mouse
    // coordinate to (0,0) regardless of actual cursor movement, which reads
    // exactly like a "frozen" mouse to the game logic above.
    return density > 0.0f ? density : 1.0f;
}

}

std::optional<Event> SDLInputAdapter::translate(const SDL_Event& event, SDL_Window* window) {
    Event translated;
    const float density = pixelDensity(window);
    switch (event.type) {
    case SDL_EVENT_QUIT:
        translated.type = EventType::Quit;
        break;
    case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
        translated.type = EventType::WindowClose;
        break;
    case SDL_EVENT_KEY_DOWN:
    case SDL_EVENT_KEY_UP:
        translated.type = event.type == SDL_EVENT_KEY_DOWN ? EventType::KeyDown : EventType::KeyUp;
        // Scancodes are physical key positions (US layout reference), which
        // is exactly what WASD-style controls want. A "type this character"
        // hotkey like the quote key needs the opposite: SDL's layout-aware
        // keycode, so it lands on whatever key actually produces '/" on the
        // active keyboard layout (e.g. apostrophe sits elsewhere on ABNT2).
        if (event.key.key == SDLK_APOSTROPHE) {
            translated.key = Key::Quote;
        } else {
            translated.key = toKey(event.key.scancode);
        }
        translated.repeat = event.key.repeat;
        break;
    case SDL_EVENT_MOUSE_MOTION:
        translated.type = EventType::MouseMove;
        translated.mousePosition = { event.motion.x * density, event.motion.y * density };
        translated.mouseDelta = { event.motion.xrel * density, event.motion.yrel * density };
        break;
    case SDL_EVENT_MOUSE_BUTTON_DOWN:
    case SDL_EVENT_MOUSE_BUTTON_UP:
        translated.type = event.type == SDL_EVENT_MOUSE_BUTTON_DOWN
            ? EventType::MouseButtonDown : EventType::MouseButtonUp;
        translated.button = toMouseButton(event.button.button);
        translated.mousePosition = { event.button.x * density, event.button.y * density };
        break;
    case SDL_EVENT_MOUSE_WHEEL:
        translated.type = EventType::MouseWheel;
        translated.wheelY = event.wheel.y;
        break;
    case SDL_EVENT_WINDOW_FOCUS_GAINED:
        translated.type = EventType::WindowFocusGained;
        break;
    case SDL_EVENT_WINDOW_FOCUS_LOST:
        translated.type = EventType::WindowFocusLost;
        break;
    default:
        return std::nullopt;
    }
    return translated;
}

void SDLInputAdapter::synchronize(InputState& input, SDL_Window* window) {
    const bool* keys = SDL_GetKeyboardState(nullptr);
    input.setKey(Key::W, keys[SDL_SCANCODE_W]);
    input.setKey(Key::A, keys[SDL_SCANCODE_A]);
    input.setKey(Key::S, keys[SDL_SCANCODE_S]);
    input.setKey(Key::D, keys[SDL_SCANCODE_D]);
    input.setKey(Key::E, keys[SDL_SCANCODE_E]);
    input.setKey(Key::Q, keys[SDL_SCANCODE_Q]);
    input.setKey(Key::R, keys[SDL_SCANCODE_R]);
    input.setKey(Key::V, keys[SDL_SCANCODE_V]);
    input.setKey(Key::Z, keys[SDL_SCANCODE_Z]);
    input.setKey(Key::Escape, keys[SDL_SCANCODE_ESCAPE]);
    input.setKey(Key::Insert, keys[SDL_SCANCODE_INSERT]);
    // Layout-aware: ask SDL which physical key currently produces the
    // apostrophe/quote character (see the matching comment in translate()).
    const SDL_Scancode quoteScancode = SDL_GetScancodeFromKey(SDLK_APOSTROPHE, nullptr);
    input.setKey(Key::Quote, quoteScancode != SDL_SCANCODE_UNKNOWN && keys[quoteScancode]);
    input.setKey(Key::Tab, keys[SDL_SCANCODE_TAB]);
    input.setKey(Key::F, keys[SDL_SCANCODE_F]);
    input.setKey(Key::Space, keys[SDL_SCANCODE_SPACE]);
    input.setKey(Key::LeftShift, keys[SDL_SCANCODE_LSHIFT]);
    input.setKey(Key::RightShift, keys[SDL_SCANCODE_RSHIFT]);
    input.setKey(Key::LeftControl, keys[SDL_SCANCODE_LCTRL]);
    input.setKey(Key::RightControl, keys[SDL_SCANCODE_RCTRL]);
    input.setKey(Key::LeftAlt, keys[SDL_SCANCODE_LALT]);
    input.setKey(Key::RightAlt, keys[SDL_SCANCODE_RALT]);
    float x = 0.0f;
    float y = 0.0f;
    SDL_GetMouseState(&x, &y);
    const float density = pixelDensity(window);
    input.setMousePosition({ x * density, y * density });
}

} // namespace MatterEngine::Platform
