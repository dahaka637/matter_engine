#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"

#include <functional>
#include <span>
#include <string>
#include <vector>

// O corte da malha por um plano: os triangulos que o cruzam sao divididos
// (um vertice novo em cada aresta cortada, o mesmo para os dois vizinhos) e
// os do lado de tras saem. A borda nova fica exatamente sobre o plano, em
// ordem de borda (a superficie a esquerda de quem anda nela, vista de fora):
// voltas fechadas, ou trechos abertos quando a malha ja tinha borda ali (a
// metade de um corpo cortado no meio). Os vertices que sobram do lado de
// tras ficam soltos ate compactar.
namespace MatterEngine::Modeling {

struct CutPlane {
    Vec3 point;
    // Fica o lado para onde a normal aponta.
    Vec3 normal { 0.0f, 0.0f, 1.0f };
};

// Um corte por uma superficie qualquer: a distancia (com sinal) do ponto a
// ela, mais ou menos em metros; fica o lado positivo.
using CutField = std::function<float(Vec3)>;
// A de um plano (fica o lado da normal).
[[nodiscard]] CutField planeField(const CutPlane& plane);

struct CutSettings {
    // Vertices a menos disso do plano vao para ele (nenhuma lasca fina); os
    // da borda que a malha ja tem andam ao longo dela (o meio fica no meio).
    float snap = 1e-4f;
};

struct CutBorder {
    std::vector<std::uint32_t> vertices;
    // Fechada (volta) ou aberta (trecho de uma borda a outra).
    bool closed = true;
};

struct CutResult {
    bool ok = false;
    std::string error;
    std::vector<CutBorder> borders;
};

[[nodiscard]] CutResult cutByPlane(Mesh& mesh, const CutPlane& plane, const CutSettings& settings = {});

// O mesmo corte so nas curvas do plano que passam a menos de "radius" de
// "center" (seguidas inteiras, de triangulo em triangulo), sem tirar nada:
// a volta nova em volta de um membro (o plano infinito nao corta o resto do
// corpo). A volta sai na ordem de borda do lado da frente do plano; um
// trecho aberto tem que comecar e acabar numa borda que a malha ja tinha.
[[nodiscard]] CutResult splitNear(Mesh& mesh, const CutPlane& plane, Vec3 center, float radius,
    const CutSettings& settings = {});
// O mesmo por uma superficie qualquer (a gola em V em volta do pescoco).
[[nodiscard]] CutResult splitNear(Mesh& mesh, const CutField& field, Vec3 center, float radius,
    const CutSettings& settings = {});

// Tira o pedaco ligado que contem o triangulo mais perto de "seed", sem
// atravessar as voltas dadas (as arestas delas sao barreira; as abertas nao
// fecham do fim ao comeco). Os vertices ficam soltos ate compactar.
// Devolve quantos triangulos sairam.
std::size_t removeRegion(Mesh& mesh, Vec3 seed, std::span<const CutBorder> barriers);

// O sentido de uma volta em torno de um eixo: positivo = anti-horaria vista
// da ponta do eixo.
[[nodiscard]] float loopWinding(const Mesh& mesh, std::span<const std::uint32_t> loop, Vec3 axis);
// O centro (media dos pontos) de uma volta.
[[nodiscard]] Vec3 loopCenter(const Mesh& mesh, std::span<const std::uint32_t> loop);

} // namespace MatterEngine::Modeling
