#include "Engine/RL/Supervisor/TrainingHistory3D.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <type_traits>

namespace MatterEngine {
namespace {

double columnDouble(sqlite3_stmt* statement, int column) {
    return sqlite3_column_type(statement, column) == SQLITE_NULL ? 0.0 : sqlite3_column_double(statement, column);
}

std::string columnText(sqlite3_stmt* statement, int column) {
    const unsigned char* value = sqlite3_column_text(statement, column);
    return value ? reinterpret_cast<const char*>(value) : "";
}

} // namespace

TrainingHistory3D readTrainingHistory3D(const std::filesystem::path& database, const std::string& run,
    double sinceSeconds, std::size_t maximumPoints) {
    TrainingHistory3D history;
    if (!std::filesystem::exists(database)) return history;
    sqlite3* db = nullptr;
    if (sqlite3_open_v2(database.string().c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        return history;
    }
    sqlite3_busy_timeout(db, 500);
    // O periodo conta da rodada mais recente do treino (vivo, e agora).
    sqlite3_stmt* statement = nullptr;
    double newest = 0.0;
    if (sinceSeconds > 0.0
        && sqlite3_prepare_v2(db, "SELECT MAX(time) FROM iterations WHERE run=?1", -1, &statement, nullptr) == SQLITE_OK) {
        sqlite3_bind_text(statement, 1, run.c_str(), -1, SQLITE_TRANSIENT);
        if (sqlite3_step(statement) == SQLITE_ROW && sqlite3_column_type(statement, 0) != SQLITE_NULL)
            newest = sqlite3_column_double(statement, 0);
        sqlite3_finalize(statement);
    }
    if (sinceSeconds > 0.0 && newest <= 0.0)
        newest = static_cast<double>(
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const double since = sinceSeconds > 0.0 ? newest - sinceSeconds : 0.0;

    // A amostra e por tempo (a rodada mais recente de cada janela), e nao pelo
    // numero da rodada: as antigas, ja compactadas em uma a cada 10 min,
    // continuam no grafico.
    double first = 0.0;
    double last = 0.0;
    bool any = false;
    if (sqlite3_prepare_v2(db, "SELECT MIN(time), MAX(time) FROM iterations WHERE run=?1 AND time>=?2", -1, &statement,
            nullptr)
        == SQLITE_OK) {
        sqlite3_bind_text(statement, 1, run.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(statement, 2, since);
        if (sqlite3_step(statement) == SQLITE_ROW && sqlite3_column_type(statement, 0) != SQLITE_NULL) {
            first = sqlite3_column_double(statement, 0);
            last = sqlite3_column_double(statement, 1);
            any = true;
        }
        sqlite3_finalize(statement);
    }
    if (any) {
        const double window
            = std::max((last - first) / static_cast<double>(std::max<std::size_t>(1, maximumPoints)), 1.0e-6);
        // Com um MAX() so, o SQLite tira as outras colunas da mesma linha.
        const char* sql = "SELECT MAX(iteration), time, steps, reward, episode_seconds, fall_fraction, sps "
                          "FROM iterations WHERE run=?1 AND time>=?2 GROUP BY CAST((time - ?3) / ?4 AS INTEGER) "
                          "ORDER BY 1";
        if (sqlite3_prepare_v2(db, sql, -1, &statement, nullptr) == SQLITE_OK) {
            sqlite3_bind_text(statement, 1, run.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_double(statement, 2, since);
            sqlite3_bind_double(statement, 3, first);
            sqlite3_bind_double(statement, 4, window);
            while (sqlite3_step(statement) == SQLITE_ROW) {
                TrainingIterationPoint3D point;
                point.iteration = static_cast<std::uint64_t>(sqlite3_column_int64(statement, 0));
                point.time = columnDouble(statement, 1);
                point.steps = columnDouble(statement, 2);
                point.reward = columnDouble(statement, 3);
                point.episodeSeconds = columnDouble(statement, 4);
                point.fallFraction = columnDouble(statement, 5);
                point.sps = columnDouble(statement, 6);
                history.iterations.push_back(point);
            }
            sqlite3_finalize(statement);
        }
    }
    // Cada exercicio em cada rodada (o treino, nao a avaliacao): json_each abre o details.drills.
    if (sqlite3_prepare_v2(db,
            "SELECT i.iteration, d.key, json_extract(d.value, '$.episodes'), json_extract(d.value, '$.falls'), "
            "json_extract(d.value, '$.return'), json_extract(d.value, '$.seconds'), json_extract(d.value, '$.stood_fraction'), "
            "i.time, json_extract(d.value, '$.velocity_error_window_median'), json_extract(d.value, '$.speed'), "
            "json_extract(d.value, '$.requested'), json_extract(d.value, '$.facing_error'), json_extract(d.value, '$.slip'), "
            "json_extract(d.value, '$.posture') FROM iterations i, json_each(i.details, '$.drills') d "
            "WHERE i.run=?1 AND i.time>=?2 ORDER BY i.iteration",
            -1, &statement, nullptr)
        == SQLITE_OK) {
        sqlite3_bind_text(statement, 1, run.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(statement, 2, since);
        while (sqlite3_step(statement) == SQLITE_ROW) {
            TrainingDrillPoint3D point;
            point.iteration = static_cast<std::uint64_t>(sqlite3_column_int64(statement, 0));
            point.drill = columnText(statement, 1);
            point.episodes = columnDouble(statement, 2);
            point.falls = columnDouble(statement, 3);
            if (sqlite3_column_type(statement, 4) != SQLITE_NULL) point.episodeReturn = columnDouble(statement, 4);
            point.seconds = columnDouble(statement, 5);
            if (sqlite3_column_type(statement, 6) != SQLITE_NULL) point.stood = columnDouble(statement, 6);
            point.time = columnDouble(statement, 7);
            const auto optional = [statement](int column) {
                return sqlite3_column_type(statement, column) == SQLITE_NULL ? std::numeric_limits<double>::quiet_NaN()
                                                                             : sqlite3_column_double(statement, column);
            };
            point.velocityError = optional(8);
            point.bodySpeed = optional(9);
            point.requestedSpeed = optional(10);
            point.facingError = optional(11);
            point.slip = optional(12);
            point.postureError = optional(13);
            history.drills.push_back(point);
        }
        sqlite3_finalize(statement);
    }
    // As medidas de cada exercicio ficam num JSON (o treinador acrescenta outras sem mudar o banco).
    const char* evaluations = "SELECT iteration, time, steps, solver, drill, json_extract(data, '$.falls'), "
                              "json_extract(data, '$.velocity_error_median'), json_extract(data, '$.useful_steps'), "
                              "json_extract(data, '$.return'), json_extract(data, '$.stood_fraction'), "
                              "json_extract(data, '$.speed'), json_extract(data, '$.requested') "
                              "FROM body_evaluations WHERE run=?1 AND time>=?2 ORDER BY iteration, drill, solver";
    if (sqlite3_prepare_v2(db, evaluations, -1, &statement, nullptr) == SQLITE_OK) {
        sqlite3_bind_text(statement, 1, run.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_double(statement, 2, since);
        while (sqlite3_step(statement) == SQLITE_ROW) {
            TrainingEvaluationPoint3D point;
            point.iteration = static_cast<std::uint64_t>(sqlite3_column_int64(statement, 0));
            point.time = columnDouble(statement, 1);
            point.steps = columnDouble(statement, 2);
            point.solver = columnText(statement, 3);
            point.drill = columnText(statement, 4);
            point.falls = columnDouble(statement, 5);
            point.velocityError = sqlite3_column_type(statement, 6) == SQLITE_NULL ? -1.0 : columnDouble(statement, 6);
            point.usefulSteps = sqlite3_column_type(statement, 7) == SQLITE_NULL ? -1.0 : columnDouble(statement, 7);
            if (sqlite3_column_type(statement, 8) != SQLITE_NULL) point.episodeReturn = columnDouble(statement, 8);
            if (sqlite3_column_type(statement, 9) != SQLITE_NULL) point.stood = columnDouble(statement, 9);
            if (sqlite3_column_type(statement, 10) != SQLITE_NULL) point.bodySpeed = columnDouble(statement, 10);
            if (sqlite3_column_type(statement, 11) != SQLITE_NULL) point.requestedSpeed = columnDouble(statement, 11);
            history.evaluations.push_back(point);
        }
        sqlite3_finalize(statement);
    }
    const char* events = "SELECT time, kind, author, message FROM events WHERE run=?1 ORDER BY time DESC LIMIT 40";
    if (sqlite3_prepare_v2(db, events, -1, &statement, nullptr) == SQLITE_OK) {
        sqlite3_bind_text(statement, 1, run.c_str(), -1, SQLITE_TRANSIENT);
        while (sqlite3_step(statement) == SQLITE_ROW)
            history.events.push_back({ columnDouble(statement, 0), columnText(statement, 1), columnText(statement, 2),
                columnText(statement, 3) });
        sqlite3_finalize(statement);
    }
    sqlite3_close(db);
    return history;
}

std::optional<TrainingLineageLink3D> readTrainingLineage3D(const std::filesystem::path& runDirectory) {
    std::ifstream file(runDirectory / "linhagem.json");
    if (!file) return std::nullopt;
    const nlohmann::json json = nlohmann::json::parse(file, nullptr, false);
    if (json.is_discarded() || !json.is_object() || !json.contains("de") || !json["de"].is_string()
        || !json.contains("rodada") || !json["rodada"].is_number_integer())
        return std::nullopt;
    TrainingLineageLink3D link { json["de"].get<std::string>(), json["rodada"].get<std::uint64_t>() };
    if (link.run.empty()) return std::nullopt;
    return link;
}

void prependTrainingAncestor3D(TrainingHistory3D& history, const TrainingHistory3D& parent,
    std::uint64_t untilIteration, double since) {
    const auto prepend = [&](auto& into, const auto& from) {
        std::remove_cvref_t<decltype(into)> merged;
        for (const auto& point : from)
            if (point.iteration <= untilIteration && point.time >= since) merged.push_back(point);
        merged.insert(merged.end(), into.begin(), into.end());
        into = std::move(merged);
    };
    prepend(history.iterations, parent.iterations);
    prepend(history.evaluations, parent.evaluations);
    prepend(history.drills, parent.drills);
}

} // namespace MatterEngine
