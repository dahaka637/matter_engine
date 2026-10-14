#include <exception>
#include <iostream>

// Os testes do treino de RL (corpo e lote). Cada arquivo registra os seus.
void runRLBodyTests();
void runBodyContractTests();
void runTrainingBatchTests();

int main() {
    try {
        runRLBodyTests();
        runBodyContractTests();
        runTrainingBatchTests();
        std::cout << "RL training tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "RL training tests failed: " << error.what() << '\n';
        return 1;
    }
}
