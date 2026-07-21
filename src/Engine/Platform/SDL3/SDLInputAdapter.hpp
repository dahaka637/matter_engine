#pragma once

#include "Engine/Input/Input.hpp"
#include <optional>

union SDL_Event;
struct SDL_Window;

namespace MatterEngine::Platform {

class SDLInputAdapter final {
public:
    // `window` lets mouse coordinates be scaled from SDL3's logical window
    // space into physical pixels, matching what the renderer (and therefore
    // every world<->screen conversion in the game) actually measures in via
    // SDL_GetWindowSizeInPixels. Passing nullptr leaves coordinates unscaled.
    [[nodiscard]] static std::optional<Event> translate(const SDL_Event& event, SDL_Window* window);
    static void synchronize(InputState& input, SDL_Window* window);
};

} // namespace MatterEngine::Platform
