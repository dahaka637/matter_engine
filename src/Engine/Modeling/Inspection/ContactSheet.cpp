#include "Engine/Modeling/Inspection/ContactSheet.hpp"

#include "Engine/Modeling/Raster/TextRenderer.hpp"

#include <algorithm>
#include <vector>

namespace MatterEngine::Modeling {

namespace {

constexpr int Margin = 12;
constexpr int TitleHeight = 44;
constexpr int CaptionHeight = 28;

} // namespace

Image composeSheet(std::span<const Image> images, std::span<const std::string> captions, int columns,
    std::string_view title) {
    if (images.empty()) return {};
    columns = std::clamp(columns, 1, static_cast<int>(images.size()));
    const int rows = (static_cast<int>(images.size()) + columns - 1) / columns;
    int cellWidth = 0, cellHeight = 0;
    for (const Image& image : images) {
        cellWidth = std::max(cellWidth, image.width());
        cellHeight = std::max(cellHeight, image.height());
    }
    const int width = Margin + columns * (cellWidth + Margin);
    const int height = TitleHeight + rows * (cellHeight + CaptionHeight + Margin) + Margin;
    Image sheet(width, height, rgb(10, 13, 17));
    const TextRenderer& text = TextRenderer::standard();
    text.draw(sheet, static_cast<float>(Margin), 12.0f, title, 20.0f, Palette::Text);
    for (std::size_t i = 0; i < images.size(); ++i) {
        const int column = static_cast<int>(i) % columns, row = static_cast<int>(i) / columns;
        const int x = Margin + column * (cellWidth + Margin);
        const int y = TitleHeight + row * (cellHeight + CaptionHeight + Margin);
        sheet.drawImage(images[i], x, y);
        if (i < captions.size())
            text.draw(sheet, static_cast<float>(x + 4), static_cast<float>(y + images[i].height() + 5), captions[i],
                15.0f, Palette::TextMuted);
    }
    return sheet;
}

Image renderSheet(const InspectionScene& scene, std::span<const SheetPanel> panels, int columns, int panelWidth,
    int panelHeight, std::string_view title) {
    std::vector<Image> images;
    std::vector<std::string> captions;
    images.reserve(panels.size());
    for (const SheetPanel& panel : panels) {
        images.push_back(renderInspection(scene, panel.style, panel.camera, panelWidth, panelHeight));
        captions.push_back(panel.caption);
    }
    return composeSheet(images, captions, columns, title);
}

} // namespace MatterEngine::Modeling
