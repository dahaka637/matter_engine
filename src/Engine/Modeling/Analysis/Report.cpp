#include "Engine/Modeling/Analysis/Report.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace MatterEngine::Modeling {

const char* severityName(Severity severity) {
    switch (severity) {
    case Severity::Error: return "ERRO";
    case Severity::Warning: return "AVISO";
    case Severity::Info: break;
    }
    return "INFO";
}

Box3 Finding::bounds() const {
    Box3 box;
    if (hasLocation) box.include(location);
    for (const Vec3& point : points) box.include(point);
    for (const auto& [a, b] : segments) {
        box.include(a);
        box.include(b);
    }
    return box;
}

void Report::add(Finding finding) {
    m_findings.push_back(std::move(finding));
}

void Report::statistic(std::string name, std::string value) {
    m_statistics.emplace_back(std::move(name), std::move(value));
}

std::size_t Report::count(Severity severity) const {
    return static_cast<std::size_t>(std::count_if(m_findings.begin(), m_findings.end(),
        [severity](const Finding& finding) { return finding.severity == severity; }));
}

void Report::merge(const Report& other) {
    m_findings.insert(m_findings.end(), other.m_findings.begin(), other.m_findings.end());
    m_statistics.insert(m_statistics.end(), other.m_statistics.begin(), other.m_statistics.end());
}

void Report::sort() {
    std::stable_sort(m_findings.begin(), m_findings.end(),
        [](const Finding& a, const Finding& b) { return a.severity > b.severity; });
}

std::string formatNumber(float value, int decimals) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, static_cast<double>(value));
    std::string text = buffer;
    std::replace(text.begin(), text.end(), '.', ',');
    return text;
}

std::string formatLength(float meters) {
    const float magnitude = std::abs(meters);
    if (magnitude >= 1.0f) return formatNumber(meters, 3) + " m";
    if (magnitude >= 0.01f) return formatNumber(meters * 100.0f, 1) + " cm";
    return formatNumber(meters * 1000.0f, 1) + " mm";
}

std::string Report::text(std::size_t maximumFindings) const {
    std::ostringstream out;
    out << "Resumo: " << count(Severity::Error) << " erro(s), " << count(Severity::Warning) << " aviso(s), "
        << count(Severity::Info) << " observacao(oes)\n";
    if (!m_statistics.empty()) {
        out << "\nEstatisticas:\n";
        for (const auto& [name, value] : m_statistics) out << "  " << name << ": " << value << '\n';
    }
    if (!m_findings.empty()) out << "\nAchados:\n";
    std::size_t shown = 0;
    for (std::size_t i = 0; i < m_findings.size() && shown < maximumFindings; ++i, ++shown) {
        const Finding& finding = m_findings[i];
        out << "  [" << i << "] " << severityName(finding.severity) << " " << finding.code << ": " << finding.message;
        if (!finding.part.empty()) out << " (parte " << finding.part << ")";
        if (!finding.region.empty()) out << " (regiao " << finding.region << ")";
        if (finding.hasLocation)
            out << " em (" << formatNumber(finding.location.x, 3) << "; " << formatNumber(finding.location.y, 3) << "; "
                << formatNumber(finding.location.z, 3) << ")";
        out << '\n';
    }
    if (m_findings.size() > shown) out << "  ... e mais " << (m_findings.size() - shown) << '\n';
    return out.str();
}

std::string Report::json() const {
    nlohmann::json document;
    document["errors"] = count(Severity::Error);
    document["warnings"] = count(Severity::Warning);
    document["infos"] = count(Severity::Info);
    nlohmann::json statistics = nlohmann::json::object();
    for (const auto& [name, value] : m_statistics) statistics[name] = value;
    document["statistics"] = statistics;
    nlohmann::json findings = nlohmann::json::array();
    for (const Finding& finding : m_findings) {
        nlohmann::json entry;
        entry["severity"] = severityName(finding.severity);
        entry["code"] = finding.code;
        entry["message"] = finding.message;
        if (!finding.part.empty()) entry["part"] = finding.part;
        if (!finding.otherPart.empty()) entry["otherPart"] = finding.otherPart;
        if (!finding.region.empty()) entry["region"] = finding.region;
        if (finding.hasLocation) entry["location"] = { finding.location.x, finding.location.y, finding.location.z };
        if (!finding.triangles.empty()) {
            entry["triangles"] = finding.triangles.size();
            // Os indices (os primeiros 200): o "pesos --triangulos" e o
            // render olham cada um.
            entry["triangleIds"] = std::vector<std::uint32_t>(finding.triangles.begin(),
                finding.triangles.begin() + static_cast<std::ptrdiff_t>(std::min<std::size_t>(finding.triangles.size(), 200)));
        }
        if (!finding.segments.empty()) entry["segments"] = finding.segments.size();
        findings.push_back(entry);
    }
    document["findings"] = findings;
    return document.dump(2);
}

} // namespace MatterEngine::Modeling
