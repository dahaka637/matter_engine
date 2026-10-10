#include "Commands.hpp"

#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"

#include <chrono>
#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

int runRender(const ModelContext& context, const CommandLine& line) {
    const std::optional<ViewRequest> request = viewFromOptions(context, line);
    if (!request) return 2;
    const auto start = std::chrono::steady_clock::now();
    const Image image = renderInspection(context.scene(), request->style, request->camera, request->width,
        request->height);
    // --tempo: quanto o desenho levou (sem a gravacao do PNG).
    if (line.has("tempo"))
        std::cout << "render: "
                  << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()
                  << " ms\n";
    const std::string name = "render_" + context.model().regions.regions[static_cast<std::size_t>(request->region)].id
        + "_" + request->viewId + "_" + std::string(surfaceModeId(request->style.surface)) + ".png";
    return saveImage(image, context.output(line, name)) ? 0 : 1;
}

// Folha: varias vistas (ou regioes, ou modos) lado a lado.
//   --vistas frente,direita,costas,34   --regioes cabeca,mao_direita   --modos solido,raiox
int runSheet(const ModelContext& context, const CommandLine& line) {
    std::vector<std::string> views = line.list("vistas");
    std::vector<std::string> regions = line.list("regioes");
    std::vector<std::string> modes = line.list("modos");
    if (views.empty() && regions.empty() && modes.empty()) views = { "frente", "direita", "costas", "34" };
    if (views.empty()) views = { line.text("vista", "") };
    if (regions.empty()) regions = { line.text("regiao", "visao_geral") };
    if (modes.empty()) modes = { line.text("modo", context.model().mesh.empty() ? "ossos" : "solido") };
    const int panelWidth = line.integer("largura", 640);
    const int panelHeight = line.integer("altura", 560);

    std::vector<SheetPanel> panels;
    for (const std::string& region : regions)
        for (const std::string& view : views)
            for (const std::string& mode : modes) {
                CommandLine panelLine = line;
                panelLine.options["regiao"] = region;
                if (!view.empty()) panelLine.options["vista"] = view;
                panelLine.options["modo"] = mode;
                panelLine.options["largura"] = std::to_string(panelWidth);
                panelLine.options["altura"] = std::to_string(panelHeight);
                panelLine.options["sem-hud"] = "1";
                const std::optional<ViewRequest> request = viewFromOptions(context, panelLine, panelWidth, panelHeight);
                if (!request) return 2;
                const BodyRegion& definition = context.model().regions.regions[static_cast<std::size_t>(request->region)];
                const ViewPreset* preset = view.empty() ? nullptr : findViewPreset(view);
                std::string caption = definition.label;
                if (preset != nullptr) caption += " · " + std::string(preset->label);
                if (modes.size() > 1) caption += " · " + std::string(surfaceModeLabel(request->style.surface));
                panels.push_back({ request->camera, request->style, caption });
            }
    const int columns = std::max(1, line.integer("colunas", static_cast<int>(std::min<std::size_t>(panels.size(), 4))));
    std::string title = line.text("titulo", context.model().source);
    if (!context.poseLabel.empty()) title += " · pose " + context.poseLabel;
    const Image sheet = renderSheet(context.scene(), panels, columns, panelWidth, panelHeight, title);
    return saveImage(sheet, context.output(line, "folha.png")) ? 0 : 1;
}

// Gira em volta (yaw de 0 a 360) numa folha: --passos 8.
int runTurntable(const ModelContext& context, const CommandLine& line) {
    const int steps = std::clamp(line.integer("passos", 8), 2, 24);
    const int panelWidth = line.integer("largura", 480);
    const int panelHeight = line.integer("altura", 560);
    std::vector<SheetPanel> panels;
    for (int i = 0; i < steps; ++i) {
        CommandLine panelLine = line;
        const float yaw = 360.0f * static_cast<float>(i) / static_cast<float>(steps);
        panelLine.options.erase("vista");
        panelLine.options["yaw"] = std::to_string(yaw);
        panelLine.options["pitch"] = line.text("pitch", "8");
        panelLine.options["largura"] = std::to_string(panelWidth);
        panelLine.options["altura"] = std::to_string(panelHeight);
        panelLine.options["sem-hud"] = "1";
        const std::optional<ViewRequest> request = viewFromOptions(context, panelLine, panelWidth, panelHeight);
        if (!request) return 2;
        panels.push_back({ request->camera, request->style, "yaw " + std::to_string(static_cast<int>(yaw)) + "°" });
    }
    const Image sheet = renderSheet(context.scene(), panels, std::min(steps, 4), panelWidth, panelHeight,
        "Volta completa · " + context.model().source);
    return saveImage(sheet, context.output(line, "volta.png")) ? 0 : 1;
}

} // namespace MatterEngine::ModelInspector
