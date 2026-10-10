#include "Engine/Modeling/Raster/ImageIO.hpp"

#include <filesystem>

// So as declaracoes: a implementacao do escritor esta em StbModelingImpl.cpp
// e a do leitor na biblioteca stb_image.
#include "stb_image_write.h"
#include "stb_image.h"

namespace MatterEngine::Modeling {

bool writePng(const Image& image, const std::string& path, std::string* error) {
    if (image.empty()) {
        if (error != nullptr) *error = "imagem vazia";
        return false;
    }
    std::error_code ignored;
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ignored);
    const int written = stbi_write_png(path.c_str(), image.width(), image.height(), 4, image.pixels().data(),
        image.width() * 4);
    if (written == 0 && error != nullptr) *error = "falha ao gravar " + path;
    return written != 0;
}

std::optional<Image> readImage(const std::string& path, std::string* error) {
    int width = 0, height = 0, channels = 0;
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 4);
    if (data == nullptr) {
        if (error != nullptr) *error = std::string("falha ao ler ") + path + ": " + stbi_failure_reason();
        return std::nullopt;
    }
    Image image = Image::fromRgba8(width, height,
        std::span<const std::uint8_t>(data, static_cast<std::size_t>(width) * height * 4));
    stbi_image_free(data);
    return image;
}

} // namespace MatterEngine::Modeling
