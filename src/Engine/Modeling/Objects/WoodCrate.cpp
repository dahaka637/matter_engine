#include "Engine/Modeling/Objects/WoodCrate.hpp"

#include "Engine/Modeling/Objects/ObjectMesh.hpp"
#include "Engine/Modeling/Objects/PixelCanvas.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Modeling {

namespace {

struct CratePaint {
    Color wood;
    Color frame;
    Color light;
    Color dark;
    Color nail;
    float grain = 0.5f;
    float tone = 0.08f;
};

// Uma tabua (so o miolo: as frestas entram depois, uma linha so entre as
// vizinhas): o tom proprio, os veios ao comprido, o realce de um lado, a
// sombra do outro e, as vezes, um no.
void paintBoard(PixelCanvas& canvas, int x0, int y0, int x1, int y1, Color base, const CratePaint& paint, bool horizontal,
    std::uint32_t seed) {
    if (x1 < x0 || y1 < y0) return;
    const float tone = 1.0f + (pixelNoise(x0, y0, seed) * 2.0f - 1.0f) * paint.tone;
    canvas.fillRect(x0, y0, x1, y1, scaled(base, tone));
    // Os veios: tracos curtos mais escuros ao comprido da tabua.
    const int along0 = horizontal ? x0 : y0, along1 = horizontal ? x1 : y1;
    const int across0 = horizontal ? y0 : x0, across1 = horizontal ? y1 : x1;
    for (int across = across0 + 1; across <= across1 - 1; ++across)
        for (int along = along0; along <= along1; ++along) {
            const float streak = pixelNoise(along / 4, across * 7 + 3, seed + 11);
            if (streak < paint.grain * 0.32f) {
                if (horizontal) canvas.shade(along, across, 0.88f);
                else canvas.shade(across, along, 0.88f);
            }
        }
    canvas.jitter(x0, y0, x1, y1, 0.03f, seed + 5);
    // Realce na beira de cima (ou da esquerda) e sombra na de baixo (ou da
    // direita), so com largura para isso.
    if (across1 - across0 >= 2) {
        for (int along = along0; along <= along1; ++along) {
            if (horizontal) {
                canvas.set(along, across0, mix(canvas.at(along, across0), paint.light, 0.45f));
                canvas.shade(along, across1, 0.86f);
            } else {
                canvas.set(across0, along, mix(canvas.at(across0, along), paint.light, 0.45f));
                canvas.shade(across1, along, 0.86f);
            }
        }
    }
    // Um no de vez em quando, no meio da largura da tabua.
    if (pixelNoise(x1, y1, seed + 23) < 0.35f && along1 - along0 >= 8 && across1 - across0 >= 2) {
        const int along = along0 + 3 + static_cast<int>(pixelNoise(x0, y1, seed + 29) * static_cast<float>(along1 - along0 - 6));
        const int across = (across0 + across1) / 2;
        const Color knot = mix(base, paint.dark, 0.65f);
        for (int k = 0; k < 2; ++k) {
            if (horizontal) canvas.set(along + k, across, knot);
            else canvas.set(across, along + k, knot);
        }
    }
}

// Um prego: a cabeca com o brilho em cima e a sombra embaixo (2x2 numa
// moldura de 3 pixels ou mais; 1 pixel numa fina).
void paintNail(PixelCanvas& canvas, int x, int y, const CratePaint& paint, bool large) {
    if (!large) {
        canvas.set(x, y, paint.nail);
        return;
    }
    canvas.set(x, y, mix(paint.nail, paint.light, 0.55f));
    canvas.set(x + 1, y, paint.nail);
    canvas.set(x, y + 1, paint.nail);
    canvas.set(x + 1, y + 1, scaled(paint.nail, 0.6f));
}

// As tabuas de uma area (count delas, ao comprido de "horizontal"), com as
// larguras repartidas inteiras e uma fresta de um pixel entre as vizinhas.
void paintPlanks(PixelCanvas& canvas, int x0, int y0, int x1, int y1, int count, Color base, const CratePaint& paint,
    bool horizontal, std::uint32_t seed) {
    if (x1 < x0 || y1 < y0) return;
    const int span = horizontal ? y1 - y0 + 1 : x1 - x0 + 1;
    count = std::clamp(count, 1, std::max(1, span / 3));
    for (int k = 0; k < count; ++k) {
        const int from = (horizontal ? y0 : x0) + span * k / count;
        const int to = (horizontal ? y0 : x0) + span * (k + 1) / count - 1;
        if (horizontal) paintBoard(canvas, x0, from, x1, to, base, paint, true, seed + static_cast<std::uint32_t>(k) * 97u);
        else paintBoard(canvas, from, y0, to, y1, base, paint, false, seed + static_cast<std::uint32_t>(k) * 97u);
        if (k == 0) continue;
        // A fresta: a primeira fileira desta tabua.
        if (horizontal) canvas.fillRect(x0, from, x1, from, paint.dark);
        else canvas.fillRect(from, y0, from, y1, paint.dark);
    }
}

// A moldura: os montantes inteiros de cima a baixo e as travessas entre eles,
// com a fresta de um pixel na beira de dentro, nas juntas e na borda.
void paintFrame(PixelCanvas& canvas, int w, int h, int frame, const CratePaint& paint, std::uint32_t seed) {
    if (frame <= 0) {
        canvas.strokeRect(0, 0, w - 1, h - 1, paint.dark);
        return;
    }
    paintBoard(canvas, 0, 0, frame - 1, h - 1, paint.frame, paint, false, seed + 1);
    paintBoard(canvas, w - frame, 0, w - 1, h - 1, paint.frame, paint, false, seed + 2);
    paintBoard(canvas, frame, 0, w - frame - 1, frame - 1, paint.frame, paint, true, seed + 3);
    paintBoard(canvas, frame, h - frame, w - frame - 1, h - 1, paint.frame, paint, true, seed + 4);
    if (frame >= 2) {
        // A beira de dentro (contra o painel) e as juntas montante-travessa.
        canvas.fillRect(frame - 1, frame - 1, w - frame, frame - 1, paint.dark);
        canvas.fillRect(frame - 1, h - frame, w - frame, h - frame, paint.dark);
        canvas.fillRect(frame - 1, 0, frame - 1, h - 1, paint.dark);
        canvas.fillRect(w - frame, 0, w - frame, h - 1, paint.dark);
    }
    canvas.strokeRect(0, 0, w - 1, h - 1, paint.dark);
}

// Um lado: o painel de tabuas deitadas, o reforco em diagonal (de baixo a
// esquerda ate em cima a direita, por cima do painel), a moldura e os pregos.
PixelCanvas paintSide(int w, int h, int frame, int planks, bool brace, bool nails, const CratePaint& paint,
    std::uint32_t seed) {
    PixelCanvas canvas(w, h, paint.dark);
    paintPlanks(canvas, frame, frame, w - frame - 1, h - frame - 1, planks, paint.wood, paint, true, seed + 100);
    const bool hasBrace = brace && w - 2 * frame >= 6 && h - 2 * frame >= 6;
    if (hasBrace) {
        const float thickness = static_cast<float>(std::max(2, frame));
        const float fx0 = static_cast<float>(frame), fy0 = static_cast<float>(h - frame);
        const float fx1 = static_cast<float>(w - frame), fy1 = static_cast<float>(frame);
        canvas.line(fx0, fy0, fx1, fy1, thickness + 2.0f, paint.dark);
        PixelCanvas band(w, h, { 0.0f, 0.0f, 0.0f, 0.0f });
        band.line(fx0, fy0, fx1, fy1, thickness, paint.frame);
        const float length = std::hypot(fx1 - fx0, fy1 - fy0);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                if (band.at(x, y).a < 0.5f) continue;
                // Ao comprido do reforco: os veios e o tom.
                const float px = static_cast<float>(x) + 0.5f - fx0, py = static_cast<float>(y) + 0.5f - fy0;
                const float along = (px * (fx1 - fx0) + py * (fy1 - fy0)) / length;
                const float across = (px * (fy1 - fy0) - py * (fx1 - fx0)) / length;
                Color color = scaled(paint.frame, 1.0f + (pixelNoise(x, y, seed + 300) * 2.0f - 1.0f) * 0.03f);
                if (pixelNoise(static_cast<int>(along / 4.0f), static_cast<int>(std::floor(across)) + 40, seed + 301) < paint.grain * 0.32f)
                    color = scaled(color, 0.88f);
                // O lado de cima do reforco pega luz, o de baixo fica na sombra.
                if (across < -thickness * 0.5f + 1.0f) color = mix(color, paint.light, 0.4f);
                if (across > thickness * 0.5f - 1.0f) color = scaled(color, 0.86f);
                canvas.set(x, y, color);
            }
    }
    paintFrame(canvas, w, h, frame, paint, seed + 200);
    if (nails && frame >= 1) {
        const bool large = frame >= 3;
        const int inset = std::max(0, (frame - (large ? 2 : 1)) / 2);
        paintNail(canvas, inset, inset, paint, large);
        paintNail(canvas, w - frame + inset, inset, paint, large);
        paintNail(canvas, inset, h - frame + inset, paint, large);
        paintNail(canvas, w - frame + inset, h - frame + inset, paint, large);
        if (hasBrace) {
            // Nas pontas do reforco, na travessa.
            paintNail(canvas, frame + inset + 1, h - frame + inset, paint, large);
            paintNail(canvas, w - frame - (large ? 3 : 2) - inset, inset, paint, large);
        }
    }
    return canvas;
}

// A tampa (e o fundo): a moldura e as tabuas em pe, sem reforco.
PixelCanvas paintTop(int w, int h, int frame, int planks, bool nails, const CratePaint& paint, std::uint32_t seed) {
    PixelCanvas canvas(w, h, paint.dark);
    paintPlanks(canvas, frame, frame, w - frame - 1, h - frame - 1, planks, paint.wood, paint, false, seed + 500);
    paintFrame(canvas, w, h, frame, paint, seed + 600);
    if (nails && frame >= 1) {
        const bool large = frame >= 3;
        const int inset = std::max(0, (frame - (large ? 2 : 1)) / 2);
        paintNail(canvas, inset, inset, paint, large);
        paintNail(canvas, w - frame + inset, inset, paint, large);
        paintNail(canvas, inset, h - frame + inset, paint, large);
        paintNail(canvas, w - frame + inset, h - frame + inset, paint, large);
    }
    return canvas;
}

bool generateCrate(const ObjectRecipe& recipe, const ObjectType& type, ProceduralObject& object, std::string& error) {
    const Vec3 size { objectNumber(recipe, type, "largura"), objectNumber(recipe, type, "profundidade"),
        objectNumber(recipe, type, "altura") };
    const float largest = std::max({ size.x, size.y, size.z });
    const int pixels = objectInteger(recipe, type, "pixels");
    // Os pixels de cada lado pela medida dele (a mesma densidade em todos).
    const auto texels = [&](float meters) { return std::max(4, static_cast<int>(std::lround(static_cast<float>(pixels) * meters / largest))); };
    const int frame = objectInteger(recipe, type, "moldura");
    const int planks = objectInteger(recipe, type, "tabuas");
    const bool brace = objectToggle(recipe, type, "reforco");
    const bool nails = objectToggle(recipe, type, "pregos");
    const auto seed = static_cast<std::uint32_t>(objectInteger(recipe, type, "semente"));
    CratePaint paint;
    paint.wood = objectColor(recipe, type, "cor_madeira");
    paint.frame = objectColor(recipe, type, "cor_moldura");
    paint.light = objectColor(recipe, type, "cor_realce");
    paint.dark = objectColor(recipe, type, "cor_sombra");
    paint.nail = objectColor(recipe, type, "cor_prego");
    paint.grain = objectNumber(recipe, type, "veios");
    paint.tone = objectNumber(recipe, type, "variacao");

    // O atlas: o lado X (profundidade x altura), o lado Y (largura x altura)
    // e a tampa (largura x profundidade), lado a lado.
    const int frameFor = std::clamp(frame, 0, std::max(0, std::min(texels(std::min({ size.x, size.y, size.z })) / 2 - 2, frame)));
    const PixelCanvas sideX = paintSide(texels(size.y), texels(size.z), frameFor, planks, brace, nails, paint, seed);
    const PixelCanvas sideY = paintSide(texels(size.x), texels(size.z), frameFor, planks, brace, nails, paint, seed + 1000);
    const PixelCanvas top = paintTop(texels(size.x), texels(size.y), frameFor, planks, nails, paint, seed + 2000);
    const int atlasWidth = sideX.width() + sideY.width() + top.width();
    const int atlasHeight = std::max({ sideX.height(), sideY.height(), top.height() });
    PixelCanvas atlas(atlasWidth, atlasHeight, paint.dark);
    atlas.blit(sideX, 0, 0);
    atlas.blit(sideY, sideX.width(), 0);
    atlas.blit(top, sideX.width() + sideY.width(), 0);
    object.pixelArt = atlas.image();
    // Ampliada sem filtro: quem amostra bilinear mantem o pixel nitido.
    const int scale = atlasWidth <= 128 ? 8 : 4;

    const auto tile = [&](int x, const PixelCanvas& canvas) {
        return UvRect { static_cast<float>(x) / static_cast<float>(atlasWidth), 0.0f,
            static_cast<float>(x + canvas.width()) / static_cast<float>(atlasWidth),
            static_cast<float>(canvas.height()) / static_cast<float>(atlasHeight) };
    };
    BoxFaceUvs uvs;
    uvs.faces[0] = uvs.faces[1] = tile(0, sideX);
    uvs.faces[2] = uvs.faces[3] = tile(sideX.width(), sideY);
    uvs.faces[4] = uvs.faces[5] = tile(sideX.width() + sideY.width(), top);
    // Meio texel da imagem ampliada para dentro: a borda nao puxa o vizinho.
    uvs.inset = 0.5f / static_cast<float>(atlasWidth * scale);
    Mesh& mesh = object.mesh;
    mesh = {};
    if (!buildTexturedBox(mesh, size, uvs, "caixa", "madeira", error)) return false;
    mesh.textures.push_back({ "pixel_art", atlas.upscaled(scale) });
    mesh.materials.front().texture = 0;

    object.size = size;
    object.physics.material = recipe.material;
    object.physics.massKg = objectNumber(recipe, type, "massa");
    ObjectCollider collider;
    collider.shape = ObjectColliderShape::Box;
    collider.halfExtents = size * 0.5f;
    object.physics.colliders = { collider };
    return true;
}

} // namespace

ObjectType woodCrateObjectType() {
    ObjectType type;
    type.id = "caixa";
    type.label = "Caixa";
    const auto number = [&](const char* key, const char* label, const char* group, float value, float minimum, float maximum,
                            const char* unit = "") {
        type.parameters.push_back({ key, label, group, ObjectParameterKind::Number, value, minimum, maximum, unit, {} });
    };
    const auto integer = [&](const char* key, const char* label, const char* group, float value, float minimum, float maximum,
                             const char* unit = "") {
        type.parameters.push_back({ key, label, group, ObjectParameterKind::Integer, value, minimum, maximum, unit, {} });
    };
    const auto toggle = [&](const char* key, const char* label, const char* group, bool value) {
        type.parameters.push_back({ key, label, group, ObjectParameterKind::Toggle, value ? 1.0f : 0.0f, 0.0f, 1.0f, "", {} });
    };
    const auto color = [&](const char* key, const char* label, Color value) {
        type.parameters.push_back({ key, label, "Cores", ObjectParameterKind::Color, 0.0f, 0.0f, 1.0f, "", value });
    };
    number("largura", "Largura", "Forma", 0.6f, 0.2f, 2.0f, "m");
    number("profundidade", "Profundidade", "Forma", 0.6f, 0.2f, 2.0f, "m");
    number("altura", "Altura", "Forma", 0.6f, 0.2f, 2.0f, "m");
    number("massa", "Massa", "Física", 14.0f, 0.5f, 200.0f, "kg");
    integer("pixels", "Resolução", "Textura", 32.0f, 8.0f, 96.0f, "px");
    integer("tabuas", "Tábuas", "Textura", 4.0f, 1.0f, 10.0f);
    integer("moldura", "Moldura", "Textura", 4.0f, 0.0f, 10.0f, "px");
    toggle("reforco", "Reforço", "Textura", true);
    toggle("pregos", "Pregos", "Textura", true);
    number("veios", "Veios", "Textura", 0.5f, 0.0f, 1.0f);
    number("variacao", "Variação de tom", "Textura", 0.08f, 0.0f, 0.3f);
    integer("semente", "Semente", "Textura", 7.0f, 0.0f, 999.0f);
    color("cor_madeira", "Madeira", rgb(217, 142, 58));
    color("cor_moldura", "Moldura", rgb(233, 168, 76));
    color("cor_realce", "Realce", rgb(248, 204, 120));
    color("cor_sombra", "Sombra", rgb(110, 56, 22));
    color("cor_prego", "Prego", rgb(86, 78, 72));
    type.generate = generateCrate;
    return type;
}

} // namespace MatterEngine::Modeling
