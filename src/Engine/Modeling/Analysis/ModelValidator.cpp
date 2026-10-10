#include "Engine/Modeling/Analysis/ModelValidator.hpp"

#include "Engine/Modeling/Analysis/CrossSection.hpp"
#include "Engine/Modeling/Analysis/SkinCheck.hpp"

namespace MatterEngine::Modeling {

Report validateModel(const Mesh& mesh, const Skeleton* skeleton, const BodyRegions* regions,
    std::span<const int> jointRegion, const ValidationOptions& options) {
    Report report;
    for (const std::string& problem : mesh.validateLayout()) {
        Finding finding;
        finding.severity = Severity::Error;
        finding.code = "estrutura_invalida";
        finding.message = problem;
        report.add(std::move(finding));
    }
    if (report.count(Severity::Error) > 0) return report;
    if (mesh.empty()) {
        Finding finding;
        finding.severity = Severity::Info;
        finding.code = "sem_malha";
        finding.message = "ainda nao ha malha (so o esqueleto)";
        report.add(std::move(finding));
        return report;
    }

    checkTopology(mesh, report, options.topology);
    checkConnectivity(mesh, report, options.connectivity);
    if (options.intersections) checkIntersections(mesh, mesh.positions, report);
    if (options.symmetry) checkSymmetry(mesh, report, options.symmetrySettings);
    if (skeleton != nullptr) {
        if (options.skin) checkSkin(mesh, *skeleton, regions, jointRegion, report);
        if (options.jointCentering) checkJointCentering(mesh, *skeleton, regions, jointRegion, report);
        if (!options.poses.empty()) checkDeformation(mesh, *skeleton, options.poses, report, options.deformation);
        const Box3 bounds = mesh.bounds();
        report.statistic("altura da malha (do chao)", formatLength(bounds.maximum.z - skeleton->groundHeight));
        report.statistic("pe mais baixo (do chao)", formatLength(bounds.minimum.z - skeleton->groundHeight));
    }
    for (std::size_t part = 0; part < mesh.parts.size(); ++part)
        report.statistic("triangulos em " + mesh.parts[part].name,
            std::to_string(mesh.partTriangleCount(static_cast<int>(part))));
    report.sort();
    return report;
}

} // namespace MatterEngine::Modeling
