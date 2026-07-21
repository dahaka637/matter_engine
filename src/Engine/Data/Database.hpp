#pragma once

#include <cstdint>
#include <string>

struct sqlite3;
struct sqlite3_stmt;

namespace MatterEngine::Data {

// Thin RAII wrapper around a single prepared statement. Move-only, since it
// owns a live sqlite3_stmt* handle that must be finalized exactly once.
class Statement {
public:
    Statement() = default;
    explicit Statement(sqlite3_stmt* stmt);
    ~Statement();

    Statement(const Statement&) = delete;
    Statement& operator=(const Statement&) = delete;
    Statement(Statement&& other) noexcept;
    Statement& operator=(Statement&& other) noexcept;

    void bindText(int index, const std::string& value);
    void bindInt64(int index, std::int64_t value);
    void bindDouble(int index, double value);
    void bindNull(int index);

    // Advances to the next row. Returns true while a row is available (maps
    // to SQLITE_ROW); false once the statement is exhausted (SQLITE_DONE) or
    // it fails - callers that need INSERT/UPDATE completion just call this
    // once and ignore the result.
    bool step();
    void reset();

    [[nodiscard]] std::string columnText(int index) const;
    [[nodiscard]] std::int64_t columnInt64(int index) const;
    [[nodiscard]] double columnDouble(int index) const;
    [[nodiscard]] bool columnIsNull(int index) const;

    [[nodiscard]] bool valid() const { return m_stmt != nullptr; }

private:
    sqlite3_stmt* m_stmt = nullptr;
};

// Small RAII wrapper around a single sqlite3 connection. Not thread-safe by
// itself (matches the game's single-threaded update/render loop) - open one
// per file path and keep it as long as you need it, or open/close around a
// one-off read or write, both are cheap with SQLite.
class Database {
public:
    Database() = default;
    ~Database();

    Database(const Database&) = delete;
    Database& operator=(const Database&) = delete;
    Database(Database&& other) noexcept;
    Database& operator=(Database&& other) noexcept;

    [[nodiscard]] bool open(const std::string& path);
    void close();
    [[nodiscard]] bool isOpen() const { return m_db != nullptr; }

    [[nodiscard]] bool execute(const std::string& sql);
    [[nodiscard]] Statement prepare(const std::string& sql);

    [[nodiscard]] std::string lastError() const;

private:
    sqlite3* m_db = nullptr;
};

}
