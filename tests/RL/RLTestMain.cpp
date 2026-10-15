#include <exception>
#include <iostream>

// Os testes do treino de RL (corpo e lote). Cada arquivo registra os seus.
void runRLBodyTests();
void runBodyContractTests();
void runTrainingBatchTests();
void runBodyBatchTests();
void runStyleTests();
void runMirrorTests();
void runBodyBrainTests();
void runCenterOfMassTests();

int main() {
    try {
        runRLBodyTests();
        runBodyContractTests();
        runTrainingBatchTests();
        runBodyBatchTests();
        runStyleTests();
        runMirrorTests();
        runBodyBrainTests();
        runCenterOfMassTests();
        std::cout << "RL training tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "RL training tests failed: " << error.what() << '\n';
        return 1;
    }
}
