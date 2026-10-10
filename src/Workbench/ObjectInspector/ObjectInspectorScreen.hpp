#pragma once

#include "Engine/Materials/MaterialLibrary.hpp"
#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"
#include "Engine/Modeling/Objects/ProceduralObject.hpp"
#include "Engine/Modeling/Raster/Camera.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"
#include "Engine/Objects/ObjectPhysicsLab3D.hpp"
#include "Engine/Physics/PhysicsEngine3D.hpp"
#include "Workbench/PlayerInspector/CanvasTarget.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace MatterEngine::Workbench {

// A tela "Objetos": os objetos procedurais (as receitas de assets/objects)
// na inspecao - a lista, os parametros do tipo editaveis ao vivo (gravados de
// volta na receita), o viewport no mesmo renderizador da inspecao de jogador
// (vistas, modos, o colisor, a textura em pixel art) e o laboratorio de
// fisica (queda, arremesso, rampa, pilha, a bola contra o objeto) com as
// medidas. So desenha e repassa o mouse; a GPU so pelo CanvasTarget.
class ObjectInspectorScreen {
public:
    struct Options {
        std::filesystem::path objectsDirectory;
        // Onde "Salvar imagem" grava.
        std::filesystem::path capturesDirectory;
        PhysicsEngine3D* physics = nullptr;
        const MaterialLibrary* materials = nullptr;
    };

    enum class View : std::uint8_t { Solid, SolidWire, Wire, Normals, Clay, Texture };

    ObjectInspectorScreen(Options options, std::function<void()> onBack);
    ~ObjectInspectorScreen();

    void update(float deltaSeconds);
    void draw(CanvasTarget& canvas, float uiScale);

    // Automacao (testes, capturas): o mesmo que os controles da tela.
    void reload();
    void select(std::size_t index);
    [[nodiscard]] std::size_t objectCount() const { return m_recipes.size(); }
    [[nodiscard]] std::size_t selected() const { return m_selected; }
    [[nodiscard]] const Modeling::ProceduralObject* object() const { return m_object ? &*m_object : nullptr; }
    [[nodiscard]] const std::string& error() const { return m_error; }
    [[nodiscard]] Modeling::ObjectRecipe* recipe();
    // Depois de mudar a receita (recipe()).
    void regenerate();
    [[nodiscard]] bool modified() const;
    bool save(std::string* error = nullptr);
    void revert();
    void setView(View view);
    void setTab(int tab) { m_tab = tab; }
    void setShowCollider(bool show);
    void startTest(ObjectTestKind3D kind);
    void stopTest();
    [[nodiscard]] ObjectTestSettings3D& testSettings() { return m_test; }
    void setPaused(bool paused) { m_paused = paused; }
    [[nodiscard]] const ObjectPhysicsLab3D* lab() const { return m_lab.get(); }
    [[nodiscard]] const Modeling::Image& image() const { return m_image; }
    [[nodiscard]] const Modeling::OrbitCamera& camera() const { return m_camera; }

private:
    void drawSidebar(float height);
    void drawProperties();
    void drawPhysics();
    // Os botoes de baixo da lateral (os da aba).
    void drawActions();
    void drawToolbar();
    void drawViewport(CanvasTarget& canvas, float footerHeight);
    void drawFooter();
    // A camera: o enquadramento da cena atual (o objeto, ou o roteiro da
    // fisica) no angulo dado.
    void frame(float yawDegrees, float pitchDegrees, bool animate);
    [[nodiscard]] Modeling::Box3 sceneBounds() const;
    // A malha da cena do laboratorio (as copias do objeto, a bola, a rampa)
    // e as posicoes delas agora.
    void buildLabScene();
    void poseLabScene();
    // (Re)comeca o roteiro com o objeto de agora; reframe = a camera vai
    // para ele.
    void launchTest(bool reframe);
    bool render(int width, int height);
    [[nodiscard]] Modeling::Image renderTexture(int width, int height) const;
    void saveCapture();
    void notify(std::string text);
    [[nodiscard]] float ui(float value) const { return value * m_uiScale; }

    Options m_options;
    std::function<void()> m_onBack;
    std::vector<std::filesystem::path> m_files;
    std::vector<Modeling::ObjectRecipe> m_recipes;
    // A receita como esta no arquivo (o "modificado").
    std::vector<std::string> m_savedJson;
    std::size_t m_selected = 0;
    std::optional<Modeling::ProceduralObject> m_object;
    // O objeto em pe no chao (a base em z = 0), para desenhar.
    Modeling::Mesh m_display;
    std::string m_error;
    float m_uiScale = 1.0f;
    int m_tab = 0;

    // O viewport.
    View m_view = View::Solid;
    bool m_showCollider = false;
    bool m_grid = true;
    bool m_orthographic = false;
    Modeling::OrbitCamera m_camera;
    Modeling::OrbitCamera m_goal;
    bool m_animating = false;
    bool m_dirty = true;
    float m_interactionSeconds = 0.0f;
    bool m_lastFast = false;
    int m_lastWidth = 0;
    int m_lastHeight = 0;
    float m_aspect = 1.6f;
    float m_yawDegrees = 35.0f;
    float m_pitchDegrees = 22.0f;
    // A camera mexida pelo mouse (mudar o tamanho da tela nao a recentra).
    bool m_userAdjusted = false;
    Modeling::Image m_image;
    Modeling::Skeleton m_ground;

    // O laboratorio de fisica.
    std::unique_ptr<ObjectPhysicsLab3D> m_lab;
    ObjectTestSettings3D m_test;
    bool m_paused = false;
    float m_timeScale = 1.0f;
    Modeling::Mesh m_labMesh;
    struct LabPiece {
        std::size_t first = 0;
        std::size_t count = 0;
    };
    std::vector<LabPiece> m_labPieces;
    // A sombra de cada corpo (o primeiro vertice dela na cena).
    std::vector<std::size_t> m_labShadows;
    std::vector<Vec3> m_labLocal;
    std::vector<Vec3> m_labPositions;

    std::string m_notice;
    float m_noticeSeconds = 0.0f;
};

} // namespace MatterEngine::Workbench
