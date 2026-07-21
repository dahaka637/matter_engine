#pragma once

#include "Engine/Platform/DisplayMode.hpp"
#include <optional>
#include <string>

namespace MatterEngine::Data {

struct VideoSettings {
    int width = 1280;
    int height = 720;
    DisplayMode mode = DisplayMode::Windowed;
};

// Small persistence layer over a single SQLite file: a generic key/value
// `settings` table so new preferences (audio volume, control bindings, last
// used seed, ...) can be added later without a schema migration, plus typed
// helpers (loadVideoSettings/saveVideoSettings) for the settings the game
// actually reads every frame or every startup.
class SettingsRepository {
public:
    explicit SettingsRepository(std::string dbPath);

    [[nodiscard]] VideoSettings loadVideoSettings();
    void saveVideoSettings(const VideoSettings& settings);

    [[nodiscard]] std::optional<std::string> getString(const std::string& key);
    void setString(const std::string& key, const std::string& value);

    [[nodiscard]] std::optional<int> getInt(const std::string& key);
    void setInt(const std::string& key, int value);

private:
    std::string m_dbPath;
};

}
