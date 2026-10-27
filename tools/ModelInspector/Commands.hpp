#pragma once

#include "CommandLine.hpp"
#include "ModelContext.hpp"

#include <optional>
#include <string>

namespace MatterEngine::ModelInspector {

// Camera, estilo e tamanho pelas opcoes de desenho:
//   --regiao <id>  --vista <id> (ou --yaw/--pitch em graus)  --zoom <x>  --orto
//   --modo <id>  --rotulos  --isolar  --junta <nome>  --ssaa <n>
//   --largura <px>  --altura <px>  --sem-grade  --sem-hud  --sem-ossos  --sem-avesso
struct ViewRequest {
    Modeling::OrbitCamera camera;
    Modeling::InspectionStyle style;
    int width = 1200;
    int height = 900;
    int region = 0;
    std::string viewId;
};

[[nodiscard]] std::optional<ViewRequest> viewFromOptions(const ModelContext& context, const CommandLine& line,
    int defaultWidth = 1200, int defaultHeight = 900);
// Grava e imprime o caminho (para quem chama achar a imagem).
bool saveImage(const Modeling::Image& image, const std::filesystem::path& path);

int runHelp();
int runInfo(const ModelContext& context, const CommandLine& line);
int runRender(const ModelContext& context, const CommandLine& line);
int runSheet(const ModelContext& context, const CommandLine& line);
int runCheck(const ModelContext& context, const CommandLine& line);
int runMeasure(const ModelContext& context, const CommandLine& line);
int runSection(const ModelContext& context, const CommandLine& line);
int runCompare(const ModelContext& context, const CommandLine& line);
int runPoses(const ModelContext& context, const CommandLine& line);
int runTurntable(const ModelContext& context, const CommandLine& line);
int runTexture(const ModelContext& context, const CommandLine& line);
int runMold(const ModelContext& context, const CommandLine& line);
int runInverted(const ModelContext& context, const CommandLine& line);
int runPaint(const ModelContext& context, const CommandLine& line);
int runExport(const ModelContext& context, const CommandLine& line);
int runBlenderExport(const ModelContext& context, const CommandLine& line);
int runUniform(const ModelContext& context, const CommandLine& line);
int runPosture(const ModelContext& context, const CommandLine& line);
int runWeights(const ModelContext& context, const CommandLine& line);
int runFrames(const ModelContext& context, const CommandLine& line);
#if defined(MATTERENGINE_MODEL_INSPECTOR_RL)
int runColliders(const ModelContext& context, const CommandLine& line);
#endif

} // namespace MatterEngine::ModelInspector
