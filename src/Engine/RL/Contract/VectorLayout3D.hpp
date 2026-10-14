#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace MatterEngine {

// Um canal nomeado de um vetor do contrato (observacao, critico, acao).
struct RLChannel3D {
    std::string name;
    std::string unit;
};

struct RLVectorLayout3D {
    std::string id;
    std::uint32_t version = 0;
    std::vector<RLChannel3D> channels;

    [[nodiscard]] std::size_t size() const { return channels.size(); }
};

} // namespace MatterEngine
