#pragma once

// As telas do Workbench nos testes, sem janela e sem GPU: um contexto ImGui
// com a fonte e o tema do Workbench (os erros que o ImGui recupera ficam
// guardados para virar falha), o canvas falso que guarda a imagem enviada e a
// "foto" da tela inteira (MATTERENGINE_UI_SNAPSHOT=<pasta>).

#include "Engine/Modeling/Raster/Image.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"

#include <functional>
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

// Os conflitos de ID do ImGui (o "N visible items with conflicting ID" que so
// aparece com o mouse em cima de um dos itens): o mouse varre a tela numa grade
// e, em cada ponto, o quadro seguinte conta os itens com o ID do que ficou
// embaixo. drawFrame desenha a tela (entre NewFrame e Render, sem avancar
// nada). Devolve um texto por ponto com conflito.
[[nodiscard]] std::vector<std::string> sweepIdConflicts(const std::function<void()>& drawFrame, float step = 24.0f);

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
