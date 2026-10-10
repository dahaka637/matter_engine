#pragma once

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::ModelInspector {

// "comando --opcao valor --flag posicional": opcoes sem valor viram "1".
struct CommandLine {
    std::string command;
    std::map<std::string, std::string, std::less<>> options;
    std::vector<std::string> positional;

    [[nodiscard]] bool has(std::string_view name) const;
    [[nodiscard]] std::string text(std::string_view name, std::string fallback = {}) const;
    [[nodiscard]] float number(std::string_view name, float fallback) const;
    [[nodiscard]] int integer(std::string_view name, int fallback) const;
    // Lista separada por virgulas.
    [[nodiscard]] std::vector<std::string> list(std::string_view name) const;
};

[[nodiscard]] CommandLine parseCommandLine(int argc, char** argv);

} // namespace MatterEngine::ModelInspector
