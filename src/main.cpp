#include "Workbench/WorkbenchApp.hpp"
#include <SDL3/SDL_main.h>
#include <exception>
#include <iostream>

int main(int argc, char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    try {
        MatterEngine::Workbench::WorkbenchApp workbench;
        workbench.run();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << '\n';
        return 1;
    }
}
