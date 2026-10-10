#pragma once

// As telas do Workbench nos testes, sem janela e sem GPU: um contexto ImGui
// com a fonte e o tema do Workbench (os erros que o ImGui recupera ficam
// guardados para virar falha), o canvas falso que guarda a imagem enviada e a
// "foto" da tela inteira (MATTERENGINE_UI_SNAPSHOT=<pasta>).

#include "Engine/Modeling/Raster/Image.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"

#include <string>
#include <vector>

struct ImDrawData;

namespace MatterEngine::TestSupport {

// O canvas sem GPU: a ultima imagem e quantas vezes ela foi enviada.
struct FakeCanvas final : Workbench::CanvasTarget {
    int presents = 0;
    int uploads = 0;
    int width = 0;
    int height = 0;
    Modeling::Image last;
    std::uint64_t present(const Modeling::Image& image, bool changed) override;
    void release() override {}
};

// Os triangulos do ImGui rasterizados com a textura da fonte (e a imagem do
// canvas no lugar da textura 1).
[[nodiscard]] Modeling::Image rasterizeImGui(const ImDrawData* data, const Modeling::Image& canvas);

// A foto do ultimo quadro quando MATTERENGINE_UI_SNAPSHOT aponta uma pasta.
void snapshot(const FakeCanvas& canvas, const char* name);

// O contexto ImGui da tela (um por vez).
class HeadlessUi {
public:
    explicit HeadlessUi(float width = 1920.0f, float height = 1080.0f);
    ~HeadlessUi();
    HeadlessUi(const HeadlessUi&) = delete;
    HeadlessUi& operator=(const HeadlessUi&) = delete;

    // O que o ImGui reclamou desde a criacao.
    [[nodiscard]] const std::vector<std::string>& errors() const;
};

} // namespace MatterEngine::TestSupport
