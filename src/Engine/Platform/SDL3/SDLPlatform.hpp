#pragma once

namespace MatterEngine::Platform {

// Owns SDL's process-wide lifetime. Window, audio and GPU resources must be
// destroyed before this object is shut down.
class SDLPlatform final {
public:
    SDLPlatform() = default;
    ~SDLPlatform();

    SDLPlatform(const SDLPlatform&) = delete;
    SDLPlatform& operator=(const SDLPlatform&) = delete;

    void initialize();
    void shutdown();
    [[nodiscard]] bool initialized() const { return m_initialized; }

private:
    bool m_initialized = false;
};

} // namespace MatterEngine::Platform
