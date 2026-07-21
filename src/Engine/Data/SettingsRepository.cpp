#include "Engine/Data/SettingsRepository.hpp"

#include "Engine/Data/Database.hpp"
#include <algorithm>
#include <charconv>
#include <utility>

namespace MatterEngine::Data {

namespace {

constexpr const char* KeyVideoWidth = "video.width";
constexpr const char* KeyVideoHeight = "video.height";
constexpr const char* KeyVideoMode = "video.mode";

bool ensureSchema(Database& db) {
    return db.execute(
        "CREATE TABLE IF NOT EXISTS settings ("
        "key TEXT PRIMARY KEY,"
        "value TEXT NOT NULL"
        ");");
}

std::optional<int> parseInt(const std::string& text) {
    int value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc {}) {
        return std::nullopt;
    }
    return value;
}

}

SettingsRepository::SettingsRepository(std::string dbPath)
    : m_dbPath(std::move(dbPath)) {
}

std::optional<std::string> SettingsRepository::getString(const std::string& key) {
    Database db;
    if (!db.open(m_dbPath) || !ensureSchema(db)) {
        return std::nullopt;
    }
    Statement statement = db.prepare("SELECT value FROM settings WHERE key = ?1;");
    if (!statement.valid()) {
        return std::nullopt;
    }
    statement.bindText(1, key);
    if (!statement.step()) {
        return std::nullopt;
    }
    return statement.columnText(0);
}

void SettingsRepository::setString(const std::string& key, const std::string& value) {
    Database db;
    if (!db.open(m_dbPath) || !ensureSchema(db)) {
        return;
    }
    Statement statement = db.prepare("INSERT INTO settings(key, value) VALUES (?1, ?2) "
                                      "ON CONFLICT(key) DO UPDATE SET value = excluded.value;");
    if (!statement.valid()) {
        return;
    }
    statement.bindText(1, key);
    statement.bindText(2, value);
    statement.step();
}

std::optional<int> SettingsRepository::getInt(const std::string& key) {
    const std::optional<std::string> text = getString(key);
    if (!text.has_value()) {
        return std::nullopt;
    }
    return parseInt(*text);
}

void SettingsRepository::setInt(const std::string& key, int value) {
    setString(key, std::to_string(value));
}

VideoSettings SettingsRepository::loadVideoSettings() {
    VideoSettings settings;
    if (const std::optional<int> width = getInt(KeyVideoWidth)) {
        settings.width = *width;
    }
    if (const std::optional<int> height = getInt(KeyVideoHeight)) {
        settings.height = *height;
    }
    if (const std::optional<int> mode = getInt(KeyVideoMode)) {
        const int clamped = std::clamp(*mode, 0, 2);
        settings.mode = static_cast<DisplayMode>(clamped);
    }
    return settings;
}

void SettingsRepository::saveVideoSettings(const VideoSettings& settings) {
    setInt(KeyVideoWidth, settings.width);
    setInt(KeyVideoHeight, settings.height);
    setInt(KeyVideoMode, static_cast<int>(settings.mode));
}

}
