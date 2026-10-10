#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine::Modeling {

enum class Severity : std::uint8_t { Info, Warning, Error };

[[nodiscard]] const char* severityName(Severity severity);

// Um problema (ou observacao) encontrado na malha, com onde olhar e o que
// destacar no desenho.
struct Finding {
    Severity severity = Severity::Warning;
    // Identificador estavel (borda_aberta, autointersecao, ...).
    std::string code;
    std::string message;
    std::string part;
    // A outra parte envolvida (partes que se cruzam).
    std::string otherPart;
    std::string region;
    Vec3 location;
    bool hasLocation = false;
    std::vector<std::uint32_t> triangles;
    std::vector<std::pair<Vec3, Vec3>> segments;
    std::vector<Vec3> points;
    // A caixa do que destacar (para o close-up do problema).
    [[nodiscard]] Box3 bounds() const;
};

class Report {
public:
    void add(Finding finding);
    void statistic(std::string name, std::string value);

    [[nodiscard]] const std::vector<Finding>& findings() const { return m_findings; }
    [[nodiscard]] const std::vector<std::pair<std::string, std::string>>& statistics() const { return m_statistics; }
    [[nodiscard]] std::size_t count(Severity severity) const;
    // Juntando outro relatorio (as checagens por partes).
    void merge(const Report& other);
    // Mais graves primeiro.
    void sort();

    // Texto para o terminal e JSON para maquinas.
    [[nodiscard]] std::string text(std::size_t maximumFindings = 200) const;
    [[nodiscard]] std::string json() const;

private:
    std::vector<Finding> m_findings;
    std::vector<std::pair<std::string, std::string>> m_statistics;
};

// "1,23 cm" / "4,5 mm" / "1,80 m": medida legivel.
[[nodiscard]] std::string formatLength(float meters);
[[nodiscard]] std::string formatNumber(float value, int decimals);

} // namespace MatterEngine::Modeling
