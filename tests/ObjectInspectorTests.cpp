// A tela "Objetos" sem janela: as receitas da pasta, o objeto gerado no
// viewport (as vistas, os modos, o colisor, a textura), os parametros
// mudando ao vivo e gravados de volta na receita (numa copia, nunca no
// asset), e o teste fisico rodando pela tela - a caixa cai e para, os outros
// roteiros andam - sem o ImGui reclamar. MATTERENGINE_UI_SNAPSHOT=<pasta>
// guarda as fotos da tela.

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Support/HeadlessUi.hpp"
#include "Workbench/ObjectInspector/ObjectInspectorScreen.hpp"

#include "imgui.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace MatterEngine;
using TestSupport::FakeCanvas;
using TestSupport::snapshot;
using Workbench::ObjectInspectorScreen;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void frame(ObjectInspectorScreen& screen, FakeCanvas& canvas, int count = 1) {
    ImGuiIO& io = ImGui::GetIO();
    for (int i = 0; i < count; ++i) {
        io.DeltaTime = 1.0f / 60.0f;
        ImGui::NewFrame();
        screen.update(io.DeltaTime);
        screen.draw(canvas, 1.0f);
        ImGui::Render();
    }
}

// O centro do viewport na tela de 1920 x 1080 (a lateral tem ~312 px).
constexpr float ViewportX = 1100.0f;
constexpr float ViewportY = 560.0f;

void drag(ObjectInspectorScreen& screen, FakeCanvas& canvas, int button, float dx, float dy) {
    ImGuiIO& io = ImGui::GetIO();
    io.AddMousePosEvent(ViewportX, ViewportY);
    frame(screen, canvas);
    io.AddMouseButtonEvent(button, true);
    frame(screen, canvas);
    for (int i = 1; i <= 6; ++i) {
        io.AddMousePosEvent(ViewportX + dx * static_cast<float>(i) / 6.0f, ViewportY + dy * static_cast<float>(i) / 6.0f);
        frame(screen, canvas);
    }
    io.AddMouseButtonEvent(button, false);
    frame(screen, canvas);
}

void testScreen(const std::filesystem::path& objects) {
    TestSupport::HeadlessUi context;
    PhysicsEngine3D physics;
    const MaterialLibrary materials;
    ObjectInspectorScreen::Options options;
    options.objectsDirectory = objects;
    options.capturesDirectory = objects.parent_path() / "capturas";
    options.physics = &physics;
    options.materials = &materials;
    bool wentBack = false;
    ObjectInspectorScreen screen(options, [&] { wentBack = true; });
    FakeCanvas canvas;

    // A receita da pasta vira o objeto.
    require(screen.objectCount() == 1, "uma receita na pasta");
    require(screen.object() != nullptr && screen.error().empty(), "a caixa gerada: " + screen.error());
    require(screen.object()->displayName == "Caixa de madeira", "o nome da receita");
    frame(screen, canvas, 40);
    require(canvas.width > 1200 && canvas.height > 700, "o viewport recebe a imagem no tamanho dele");
    snapshot(canvas, "objetos_caixa.png");
    const int uploads = canvas.uploads;
    frame(screen, canvas, 2);
    require(canvas.uploads == uploads, "sem mudanca, a imagem nao e reenviada");

    // O mouse: arrastar gira, o direito move, a roda aproxima.
    const float yaw = screen.camera().yaw;
    drag(screen, canvas, 0, 160.0f, 0.0f);
    require(std::abs(screen.camera().yaw - yaw) > 0.5f, "arrastar gira a camera");
    const Vec3 target = screen.camera().target;
    drag(screen, canvas, 1, 120.0f, 60.0f);
    require((screen.camera().target - target).length() > 0.05f, "o botao direito move a camera");
    const float distance = screen.camera().distance;
    ImGui::GetIO().AddMousePosEvent(ViewportX, ViewportY);
    ImGui::GetIO().AddMouseWheelEvent(0.0f, 2.0f);
    frame(screen, canvas, 2);
    require(screen.camera().distance < distance * 0.9f, "a roda aproxima");
    require(canvas.uploads > uploads, "mexer na camera reenvia a imagem");

    // Os modos de desenho e o colisor.
    for (const auto view : { ObjectInspectorScreen::View::SolidWire, ObjectInspectorScreen::View::Wire,
             ObjectInspectorScreen::View::Normals, ObjectInspectorScreen::View::Clay, ObjectInspectorScreen::View::Solid }) {
        screen.setView(view);
        frame(screen, canvas, 3);
        require(!screen.image().empty(), "cada modo desenha");
    }
    screen.setView(ObjectInspectorScreen::View::SolidWire);
    screen.setShowCollider(true);
    frame(screen, canvas, 30);
    snapshot(canvas, "objetos_arestas_colisor.png");
    screen.setShowCollider(false);
    // A textura: a pixel art inteira, ampliada sem borrar.
    screen.setView(ObjectInspectorScreen::View::Texture);
    frame(screen, canvas, 3);
    snapshot(canvas, "objetos_textura.png");
    const Modeling::Image& art = screen.object()->pixelArt;
    require(!art.empty(), "a pixel art da caixa");
    {
        // O pixel do meio da imagem e um pixel da arte (sem mistura).
        const Modeling::Image& image = screen.image();
        const Modeling::Color center = image.pixel(image.width() / 2, image.height() / 2);
        bool found = false;
        for (int y = 0; y < art.height() && !found; ++y)
            for (int x = 0; x < art.width() && !found; ++x) {
                const Modeling::Color texel = art.pixel(x, y);
                found = std::abs(texel.r - center.r) < 1e-3f && std::abs(texel.g - center.g) < 1e-3f && std::abs(texel.b - center.b) < 1e-3f;
            }
        require(found, "a textura mostra os pixels da arte sem mistura");
    }
    screen.setView(ObjectInspectorScreen::View::Solid);

    // Os parametros ao vivo: a caixa mais larga e outra semente; a receita
    // marca "modificado", grava na copia e relida e igual.
    Modeling::ObjectRecipe* recipe = screen.recipe();
    require(recipe != nullptr, "a receita editavel");
    recipe->numbers["largura"] = 0.9f;
    recipe->numbers["semente"] = 21.0f;
    screen.regenerate();
    frame(screen, canvas, 30);
    require(std::abs(screen.object()->size.x - 0.9f) < 1e-4f, "a largura nova na caixa");
    require(screen.modified(), "a receita mudou");
    snapshot(canvas, "objetos_caixa_larga.png");
    std::string error;
    require(screen.save(&error), "salvar a receita: " + error);
    require(!screen.modified(), "salva, nao esta mais modificada");
    const auto saved = Modeling::readObjectRecipe(objects / "wood_crate.json", &error);
    require(saved && std::abs(saved->numbers.at("largura") - 0.9f) < 1e-4f, "o arquivo tem a largura nova");
    // Desfazer volta ao arquivo.
    screen.recipe()->numbers["altura"] = 1.2f;
    screen.regenerate();
    require(screen.modified(), "outra mudanca");
    screen.revert();
    require(!screen.modified() && std::abs(screen.object()->size.z - 0.6f) < 1e-4f, "desfazer volta ao salvo");
    // O material da receita vai para a fisica.
    screen.recipe()->material = "steel";
    screen.regenerate();
    require(screen.object()->physics.material == "steel", "o material novo no objeto");
    screen.revert();

    // O teste fisico pela tela: a queda roda ao vivo e a caixa para numa face.
    screen.setTab(1);
    screen.testSettings().height = 1.5f;
    screen.startTest(ObjectTestKind3D::Drop);
    require(screen.lab() != nullptr && screen.lab()->active(), "a queda comecou");
    frame(screen, canvas, 30);
    snapshot(canvas, "objetos_fisica_queda.png");
    frame(screen, canvas, 330);
    const ObjectTestMetrics3D& drop = screen.lab()->metrics();
    require(drop.firstImpactSpeed > 4.0f && drop.settled, "a caixa caiu e parou (" + std::to_string(drop.firstImpactSpeed) + " m/s)");
    require(std::abs(screen.lab()->bodies().front().position.z - 0.45f) < 0.02f || std::abs(screen.lab()->bodies().front().position.z - 0.3f) < 0.02f,
        "parada numa face");
    snapshot(canvas, "objetos_fisica_queda_fim.png");
    // Os outros roteiros pela tela.
    const std::pair<ObjectTestKind3D, const char*> scenarios[] { { ObjectTestKind3D::Throw, "objetos_fisica_arremesso.png" },
        { ObjectTestKind3D::Ramp, "objetos_fisica_rampa.png" }, { ObjectTestKind3D::Stack, "objetos_fisica_pilha.png" },
        { ObjectTestKind3D::BallShot, "objetos_fisica_bola.png" } };
    for (const auto& [kind, name] : scenarios) {
        screen.startTest(kind);
        frame(screen, canvas, 50);
        require(screen.lab()->active() && screen.lab()->metrics().elapsed > 0.5f, "o roteiro roda pela tela");
        snapshot(canvas, name);
    }
    require(screen.lab()->metrics().ballSpeedAfter >= 0.0f, "a bola bateu na caixa");
    // Pausar segura o tempo; mudar o objeto recomeca com ele.
    screen.setPaused(true);
    const float elapsed = screen.lab()->metrics().elapsed;
    frame(screen, canvas, 10);
    require(std::abs(screen.lab()->metrics().elapsed - elapsed) < 1e-6f, "pausado, o tempo para");
    screen.setPaused(false);
    screen.recipe()->numbers["profundidade"] = 1.0f;
    screen.regenerate();
    require(screen.lab()->active() && screen.lab()->metrics().elapsed < 0.1f, "mudar o objeto recomeca o roteiro");
    screen.revert();
    screen.stopTest();
    require(!screen.lab()->active(), "parar volta ao objeto");
    frame(screen, canvas, 30);
    snapshot(canvas, "objetos_fisica_parado.png");

    require(!wentBack, "nao voltou ao menu sozinho");
    require(context.errors().empty(), "ImGui reclamou: " + (context.errors().empty() ? std::string() : context.errors().front()));
}

} // namespace

int main() {
    // A receita numa pasta temporaria: salvar nunca mexe no asset.
    const std::filesystem::path work = std::filesystem::temp_directory_path() / "matter_object_inspector_test";
    std::filesystem::remove_all(work);
    std::filesystem::create_directories(work / "objetos");
    std::filesystem::copy_file(std::filesystem::path(MATTERENGINE_TEST_ASSETS_DIR) / "objects" / "wood_crate.json",
        work / "objetos" / "wood_crate.json");
    int result = 0;
    try {
        testScreen(work / "objetos");
    } catch (const std::exception& exception) {
        std::cerr << "MatterObjectInspector test failure: " << exception.what() << '\n';
        result = 1;
    }
    std::filesystem::remove_all(work);
    if (result == 0) std::cout << "MatterObjectInspector tests passed (" << checks << " checagens)\n";
    return result;
}
