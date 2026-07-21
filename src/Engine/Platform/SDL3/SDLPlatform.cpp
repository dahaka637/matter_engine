#include "Engine/Platform/SDL3/SDLPlatform.hpp"
#include "Engine/Platform/PlatformServices.hpp"

#include <SDL3/SDL.h>
#include <stdexcept>
#include <string>

namespace MatterEngine::Platform {

SDLPlatform::~SDLPlatform() {
    shutdown();
}

void SDLPlatform::initialize() {
    if (m_initialized) {
        return;
    }
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS)) {
        throw std::runtime_error(std::string("SDL_Init failed: ") + SDL_GetError());
    }
    m_initialized = true;
}

void SDLPlatform::shutdown() {
    if (!m_initialized) {
        return;
    }
    SDL_Quit();
    m_initialized = false;
}

std::string executableBasePath() {
    const char* path = SDL_GetBasePath();
    return path != nullptr ? path : "./";
}

} // namespace MatterEngine::Platform
