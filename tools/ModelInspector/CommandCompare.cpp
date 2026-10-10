#include "Commands.hpp"

#include "Engine/Modeling/Inspection/ContactSheet.hpp"
#include "Engine/Modeling/Inspection/ReferenceComparison.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"

#include <iostream>

namespace MatterEngine::ModelInspector {

using namespace Modeling;

// O modelo sobre as imagens de referencia (frente, costas, lado), na mesma
// escala: sobreposicao, diferenca de silhueta e larguras por altura.
//   --referencia <png> --vista <frente|costas|direita|esquerda>  (uma so)
//   sem elas: todas as referencias de corpo do manifesto
int runCompare(const ModelContext& context, const CommandLine& line) {
    std::vector<ReferenceImage> references;
    if (line.has("referencia")) {
        references.push_back({ line.text("vista", "frente"), line.text("referencia"), line.text("referencia") });
    } else {
        for (const ReferenceImage& reference : context.loaded.manifest.references)
            if (findViewPreset(reference.view) != nullptr) references.push_back(reference);
    }
    if (references.empty()) {
        std::cerr << "nenhuma referencia de corpo (frente/costas/direita/esquerda) no manifesto\n";
        return 2;
    }
    int status = 0;
    for (const ReferenceImage& reference : references) {
        const ViewPreset* view = findViewPreset(reference.view);
        if (view == nullptr) {
            std::cerr << "vista desconhecida: " << reference.view << '\n';
            status = 2;
            continue;
        }
        std::string error;
        const std::optional<Image> image = readImage(reference.path.string(), &error);
        if (!image) {
            std::cerr << error << '\n';
            status = 1;
            continue;
        }
        const std::string title = "Referência × modelo · " + std::string(view->label);
        const ReferenceComparison comparison = compareWithReference(context.scene(), *image, *view, title);
        std::cout << "\n" << title << " (" << reference.path.filename().string() << ")\n";
        if (comparison.hasMesh)
            std::cout << "  silhueta em comum (IoU): " << formatNumber(comparison.intersectionOverUnion * 100.0f, 1) << "%\n";
        else
            std::cout << "  sem malha: so os ossos sobre a referencia (proporcoes)\n";
        for (const std::string& text : comparison.lines) std::cout << "  " << text << '\n';
        const std::string base = "comparar_" + reference.view;
        const std::vector<Image> images { *image, comparison.overlay, comparison.difference };
        const std::vector<std::string> captions { "referência", "modelo por cima (contorno, ossos)",
            "diferença: vermelho só o modelo, azul só a referência" };
        if (!saveImage(composeSheet(images, captions, 3, title), context.outputDirectory / (base + ".png"))) status = 1;
        if (!saveImage(comparison.overlay, context.outputDirectory / (base + "_sobreposicao.png"))) status = 1;
        if (!comparison.solid.empty()) {
            // Lado a lado na mesma escala: a referencia e o modelo solido.
            const std::vector<Image> pair { *image, comparison.solid };
            const std::vector<std::string> labels { "referência", "modelo (sólido, mesma câmera)" };
            if (!saveImage(composeSheet(pair, labels, 2, title + " · lado a lado"),
                    context.outputDirectory / (base + "_lado_a_lado.png")))
                status = 1;
        }
    }
    return status;
}

} // namespace MatterEngine::ModelInspector
