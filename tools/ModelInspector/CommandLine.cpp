#include "CommandLine.hpp"

#include <cstdlib>
#include <sstream>

namespace MatterEngine::ModelInspector {

bool CommandLine::has(std::string_view name) const {
    return options.find(name) != options.end();
}

std::string CommandLine::text(std::string_view name, std::string fallback) const {
    const auto found = options.find(name);
    return found != options.end() ? found->second : fallback;
}

float CommandLine::number(std::string_view name, float fallback) const {
    const auto found = options.find(name);
    if (found == options.end()) return fallback;
    std::string value = found->second;
    for (char& c : value)
        if (c == ',') c = '.';
    char* end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    return end != value.c_str() ? parsed : fallback;
}

int CommandLine::integer(std::string_view name, int fallback) const {
    return static_cast<int>(number(name, static_cast<float>(fallback)));
}

std::vector<std::string> CommandLine::list(std::string_view name) const {
    std::vector<std::string> items;
    std::stringstream stream(text(name));
    std::string item;
    while (std::getline(stream, item, ','))
        if (!item.empty()) items.push_back(item);
    return items;
}

CommandLine parseCommandLine(int argc, char** argv) {
    CommandLine line;
    for (int i = 1; i < argc; ++i) {
        const std::string token = argv[i];
        if (token.rfind("--", 0) == 0) {
            const std::string name = token.substr(2);
            const bool hasValue = i + 1 < argc && std::string_view(argv[i + 1]).rfind("--", 0) != 0;
            line.options[name] = hasValue ? argv[++i] : "1";
        } else if (line.command.empty()) {
            line.command = token;
        } else {
            line.positional.push_back(token);
        }
    }
    return line;
}

} // namespace MatterEngine::ModelInspector
