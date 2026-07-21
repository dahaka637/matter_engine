#include "Engine/Data/Database.hpp"

#include "Engine/Core/Log.hpp"
#include <sqlite3.h>
#include <utility>

namespace MatterEngine::Data {

Statement::Statement(sqlite3_stmt* stmt)
    : m_stmt(stmt) {
}

Statement::~Statement() {
    if (m_stmt != nullptr) {
        sqlite3_finalize(m_stmt);
    }
}

Statement::Statement(Statement&& other) noexcept
    : m_stmt(std::exchange(other.m_stmt, nullptr)) {
}

Statement& Statement::operator=(Statement&& other) noexcept {
    if (this != &other) {
        if (m_stmt != nullptr) {
            sqlite3_finalize(m_stmt);
        }
        m_stmt = std::exchange(other.m_stmt, nullptr);
    }
    return *this;
}

void Statement::bindText(int index, const std::string& value) {
    if (m_stmt != nullptr) {
        sqlite3_bind_text(m_stmt, index, value.c_str(), static_cast<int>(value.size()), SQLITE_TRANSIENT);
    }
}

void Statement::bindInt64(int index, std::int64_t value) {
    if (m_stmt != nullptr) {
        sqlite3_bind_int64(m_stmt, index, static_cast<sqlite3_int64>(value));
    }
}

void Statement::bindDouble(int index, double value) {
    if (m_stmt != nullptr) {
        sqlite3_bind_double(m_stmt, index, value);
    }
}

void Statement::bindNull(int index) {
    if (m_stmt != nullptr) {
        sqlite3_bind_null(m_stmt, index);
    }
}

bool Statement::step() {
    if (m_stmt == nullptr) {
        return false;
    }
    return sqlite3_step(m_stmt) == SQLITE_ROW;
}

void Statement::reset() {
    if (m_stmt != nullptr) {
        sqlite3_reset(m_stmt);
    }
}

std::string Statement::columnText(int index) const {
    if (m_stmt == nullptr) {
        return {};
    }
    const unsigned char* text = sqlite3_column_text(m_stmt, index);
    return text != nullptr ? reinterpret_cast<const char*>(text) : std::string {};
}

std::int64_t Statement::columnInt64(int index) const {
    return m_stmt != nullptr ? static_cast<std::int64_t>(sqlite3_column_int64(m_stmt, index)) : 0;
}

double Statement::columnDouble(int index) const {
    return m_stmt != nullptr ? sqlite3_column_double(m_stmt, index) : 0.0;
}

bool Statement::columnIsNull(int index) const {
    return m_stmt == nullptr || sqlite3_column_type(m_stmt, index) == SQLITE_NULL;
}

Database::~Database() {
    close();
}

Database::Database(Database&& other) noexcept
    : m_db(std::exchange(other.m_db, nullptr)) {
}

Database& Database::operator=(Database&& other) noexcept {
    if (this != &other) {
        close();
        m_db = std::exchange(other.m_db, nullptr);
    }
    return *this;
}

bool Database::open(const std::string& path) {
    close();
    if (sqlite3_open(path.c_str(), &m_db) != SQLITE_OK) {
        Log::error("Falha ao abrir banco de dados '" + path + "': " + lastError());
        close();
        return false;
    }
    sqlite3_busy_timeout(m_db, 2000);
    return true;
}

void Database::close() {
    if (m_db != nullptr) {
        sqlite3_close(m_db);
        m_db = nullptr;
    }
}

bool Database::execute(const std::string& sql) {
    if (m_db == nullptr) {
        return false;
    }
    char* errorMessage = nullptr;
    const int result = sqlite3_exec(m_db, sql.c_str(), nullptr, nullptr, &errorMessage);
    if (result != SQLITE_OK) {
        Log::error("Falha ao executar SQL: " + std::string(errorMessage != nullptr ? errorMessage : "erro desconhecido"));
        sqlite3_free(errorMessage);
        return false;
    }
    return true;
}

Statement Database::prepare(const std::string& sql) {
    if (m_db == nullptr) {
        return Statement {};
    }
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, sql.c_str(), static_cast<int>(sql.size()), &stmt, nullptr) != SQLITE_OK) {
        Log::error("Falha ao preparar SQL '" + sql + "': " + lastError());
        return Statement {};
    }
    return Statement { stmt };
}

std::string Database::lastError() const {
    return m_db != nullptr ? sqlite3_errmsg(m_db) : "banco de dados nao aberto";
}

}
