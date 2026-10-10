#pragma once

#include "Engine/Modeling/Raster/Image.hpp"

#include <optional>
#include <string>

namespace MatterEngine::Modeling {

// PNG para as imagens geradas; PNG/JPEG na leitura (as referencias).
[[nodiscard]] bool writePng(const Image& image, const std::string& path, std::string* error = nullptr);
[[nodiscard]] std::optional<Image> readImage(const std::string& path, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
