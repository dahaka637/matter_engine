#include "Engine/Modeling/Parts/HeadFace.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace MatterEngine::Modeling {

namespace {

float smooth(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Um sino de 1 (no meio) a 0 (em |x| = 1), liso nas duas pontas.
float bump(float x) {
    x = std::abs(x);
    return x >= 1.0f ? 0.0f : (1.0f - x * x) * (1.0f - x * x);
}

// Uma cupula (meia esfera achatada): a borda sobe a pique.
float dome(float distance) { return distance >= 1.0f ? 0.0f : std::sqrt(1.0f - distance * distance); }

// O arco da testa, a orbita e as almofadas dos labios (o x a mais), sem o
// perfil do meio.
float sideFields(const HeadRecipe& r, float lateral, float height) {
    const float across = 1.0f - smooth(0.6f * r.ridgeReach, r.ridgeReach, lateral);
    float dx = r.ridgeDepth * across * bump((height - r.ridgeHeight) / r.ridgeHalfHeight);
    const float socket = std::hypot((lateral - r.eyeLateral) / r.socketWidth, (height - r.eyeHeight) / r.socketHeight);
    dx -= r.socketDepth * bump(socket);
    dx += r.upperPadDepth * bump(lateral / r.upperPadWidth)
        * bump((height - (r.mouthHeight + 0.55f * r.upperLip)) / (0.75f * r.upperLip));
    dx += r.lowerPadDepth * bump(lateral / r.lowerPadWidth)
        * bump((height - (r.mouthHeight - 0.5f * r.lowerLip)) / (0.7f * r.lowerLip));
    return dx;
}

// O maior dos dois com a emenda arredondada (raio "blend").
float smoothMaximum(float a, float b, float blend) {
    return 0.5f * (a + b + std::sqrt((a - b) * (a - b) + blend * blend)) - 0.5f * blend;
}

float insideRange(const ProfileCurve& curve, float height, float below, float above) {
    const float first = curve.points().front().first, last = curve.points().back().first;
    return smooth(first - below, first, height) * (1.0f - smooth(last, last + above, height));
}

// A altura da ponta do nariz (o ponto mais a frente do perfil dele).
float noseTip(const HeadRecipe& r) {
    float best = r.noseProfile.points().front().first, value = -1e9f;
    for (const auto& [height, x] : r.noseProfile.points())
        if (x > value) {
            value = x;
            best = height;
        }
    return best;
}

} // namespace

float HeadShape::angleForLateral(float height, float lateral) const {
    if (lateral <= 0.0f) return 0.0f;
    const SuperSection section = at(height);
    float low = 0.0f, high = 0.5f * Pi;
    for (int i = 0; i < 40; ++i) {
        const float mid = 0.5f * (low + high);
        if (section.radius(mid) * std::sin(mid) < lateral) low = mid;
        else high = mid;
    }
    return 0.5f * (low + high);
}

float HeadShape::angleForX(float height, float x) const {
    const SuperSection section = at(height);
    float low = 0.3f * Pi, high = Pi;
    for (int i = 0; i < 40; ++i) {
        const float mid = 0.5f * (low + high);
        if (section.point(-mid).x > x) low = mid;
        else high = mid;
    }
    return 0.5f * (low + high);
}

float HeadShape::meanRadius(float height) const {
    const SuperSection section = at(height);
    return 0.25f * (section.front + section.back) + 0.5f * section.side;
}

Vec3 HeadShape::faceSurface(float angle, float height) const {
    const HeadRecipe& r = recipe;
    const SuperSection section = at(height);
    Vec3 p = section.point(-angle);
    if (angle <= 1e-5f || angle >= Pi - 1e-5f) p.y = 0.0f;
    const float lateral = std::abs(p.y);
    const Vec3 out = safeNormalized({ p.x - section.centerX, p.y, 0.0f }, { 1.0f, 0.0f, 0.0f });

    // A borda da mandibula salta um pouco logo acima da linha (do lado do
    // queixo ate a subida para a orelha).
    float radial = 0.0f;
    if (!r.jawLine.empty()) {
        const float degrees = angle / DegreesToRadians;
        const float last = r.jawLine.points().back().first;
        const float along = smooth(3.0f, 12.0f, degrees) * (1.0f - smooth(last - 6.0f, last, degrees));
        const float above = height - r.jawLine(degrees);
        if (along > 0.0f && above >= 0.0f) radial += r.jawRidge * along * bump(above / r.jawRidgeHeight);
    }

    // Os campos do rosto so na metade da frente.
    const float front = 1.0f - smooth(0.33f * Pi, 0.5f * Pi, angle);
    if (front > 0.0f) {
        float dx = sideFields(r, lateral, height);
        // O perfil do meio: a diferenca entre ele e o que o meio ja tem,
        // espalhada ate a meia largura (fora da faixa de alturas, some).
        if (!r.profile.empty()) {
            const float inside = insideRange(r.profile, height, 0.006f, 0.02f);
            const float middle = section.centerX + section.front + sideFields(r, 0.0f, height);
            const float width = std::max(0.004f, r.profileWidth(height));
            // Os labios afinam ate os cantos: perto da boca o perfil e lido
            // numa altura apertada para a linha (o labio de baixo sobe nos
            // lados, o de cima desce).
            const float side = std::clamp(lateral / r.mouthWidth, 0.0f, 1.0f);
            const float squeeze = r.lipTaper * side * side * bump((height - r.mouthHeight) / 0.018f);
            const float sample = r.mouthHeight + (height - r.mouthHeight) * (1.0f + squeeze);
            dx += (r.profile(sample) - middle) * inside * bump(lateral / width);
        }
        // O nariz por cima (o corte: frente mais reta, lados a pique ate onde
        // encontra o rosto) e as asas em cupula dos lados da ponta.
        float nose = 0.0f;
        if (!r.noseProfile.empty()) {
            const float first = r.noseProfile.points().front().first, last = r.noseProfile.points().back().first;
            if (height > first && height < last) {
                const float face = r.profile.empty() ? section.centerX + section.front : r.profile(height);
                const float protrusion = std::max(0.0f, r.noseProfile(height) - face);
                const float width = r.noseWidth(height);
                if (lateral < width) nose = protrusion * (1.0f - std::pow(lateral / width, r.noseShape(height)));
            }
        }
        const float ala = r.alaDepth
            * dome(std::hypot((lateral - r.alaLateral) / r.alaRadius, (height - r.alaHeight) / r.alaHalfHeight));
        dx += smoothMaximum(nose, ala, 0.003f);
        // As narinas entram por baixo da ponta.
        dx -= r.nostrilDepth
            * bump(std::hypot((lateral - r.nostrilLateral) / r.nostrilRadius, (height - r.nostrilHeight) / r.nostrilHalfHeight));
        p.x += front * dx;
        // As macas saltam para fora; embaixo delas, do lado da boca, a
        // bochecha afunda.
        const float cheek = std::hypot((lateral - r.cheekLateral) / r.cheekRadius, (height - r.cheekHeight) / (0.8f * r.cheekRadius));
        radial += front * r.cheekDepth * bump(cheek);
        const float hollow = std::hypot((lateral - r.hollowLateral) / r.hollowRadius, (height - r.hollowHeight) / r.hollowRadius);
        radial -= front * r.hollowDepth * bump(hollow);
    }
    return p + out * radial;
}

Vec3 HeadShape::surface(float angle, float height) const {
    const HeadRecipe& r = recipe;
    Vec3 p = faceSurface(angle, height);
    // Embaixo da mandibula (do lado do queixo ate o canto): a pele vai da
    // volta do pescoco ate a linha da mandibula, quase reta no pescoco e
    // virando para baixo perto da linha (a quina do osso).
    if (!r.jawLine.empty()) {
        const float degrees = angle / DegreesToRadians;
        const float last = r.jawLine.points().back().first;
        const float weight = smooth(8.0f, 20.0f, degrees) * (1.0f - smooth(last - 5.0f, last, degrees));
        const float line = r.jawLine(degrees);
        const float ring = neckFront + (neckBack - neckFront) * (0.5f - 0.5f * std::cos(angle));
        if (weight > 0.0f && height < line && line > ring + 0.004f) {
            const float u = std::clamp((height - ring) / (line - ring), 0.0f, 1.0f);
            // Embaixo, a secao do alto do pescoco (sem o queixo inflado).
            const Vec3 top = faceSurface(angle, line);
            const Vec3 bottom = at(neckFront).point(-angle);
            Vec3 under = bottom + (top - bottom) * std::pow(u, r.jawCurve);
            under.z = ground + height;
            p = p + (under - p) * weight;
        }
    }
    return p;
}

Vec3 HeadShape::normal(float angle, float height) const {
    const float step = 0.002f;
    const Vec3 a0 = surface(std::max(0.0f, angle - step), height), a1 = surface(std::min(Pi, angle + step), height);
    const Vec3 h0 = surface(angle, height - 0.5f * step), h1 = surface(angle, height + 0.5f * step);
    const Vec3 fallback = safeNormalized(surface(angle, height) - Vec3 { at(height).centerX, 0.0f, ground + height }, { 1.0f, 0.0f, 0.0f });
    return safeNormalized(cross(h1 - h0, a1 - a0), fallback);
}

std::vector<int> FaceLayout::browLoop() const {
    std::vector<int> loop { browTip };
    loop.insert(loop.end(), browBottom.begin(), browBottom.end());
    loop.insert(loop.end(), browTop.rbegin(), browTop.rend());
    return loop;
}

float faceDetail(const HeadRecipe& r, float lateral, float height) {
    // O olho e a sobrancelha.
    const float eye = std::hypot((lateral - r.eyeLateral) / (r.eyeHalfWidth + 0.01f),
        (height - (r.eyeHeight + 0.006f)) / (0.5f * r.eyeOpening + 0.016f));
    float detail = 1.0f - smooth(0.85f, 1.25f, eye);
    // O nariz (da base a raiz) e a boca (do sulco sob o labio a base do
    // nariz, ate depois do canto).
    if (!r.noseProfile.empty()) {
        const float base = r.noseProfile.points().front().first, root = r.noseProfile.points().back().first;
        const float across = 1.0f - smooth(r.alaLateral + r.alaRadius, r.alaLateral + r.alaRadius + 0.008f, lateral);
        const float along = smooth(base - 0.006f, base, height) * (1.0f - smooth(root - 0.012f, root, height));
        detail = std::max(detail, across * along);
        const float mouthAcross = 1.0f - smooth(r.mouthWidth + 0.002f, r.mouthWidth + 0.01f, lateral);
        const float mouthAlong = smooth(r.mouthHeight - r.lowerLip - 0.008f, r.mouthHeight - r.lowerLip, height)
            * (1.0f - smooth(base, base + 0.004f, height));
        detail = std::max(detail, mouthAcross * mouthAlong);
    }
    return detail;
}

FaceLayout faceLayout(const HeadShape& shape) {
    const HeadRecipe& r = shape.recipe;
    FaceLayout layout;
    // Um vertice pela distancia do meio (ou pelo angulo) e a altura, com um
    // ajuste para fora (+) ou para dentro (-) no x.
    const auto byLateral = [&](float lateral, float height, float depth = 0.0f) {
        FaceVertex vertex;
        vertex.angle = shape.angleForLateral(height, lateral);
        vertex.height = height;
        vertex.position = shape.surface(vertex.angle, height);
        vertex.position.x += depth;
        if (lateral <= 0.0f) vertex.position.y = 0.0f;
        layout.vertices.push_back(vertex);
        return static_cast<int>(layout.vertices.size() - 1);
    };
    const auto byAngle = [&](float angle, float height) {
        FaceVertex vertex;
        vertex.angle = angle;
        vertex.height = height;
        vertex.position = shape.surface(angle, height);
        layout.vertices.push_back(vertex);
        return static_cast<int>(layout.vertices.size() - 1);
    };
    const auto chain = [&](std::initializer_list<int> indices) {
        const std::vector<int> list(indices);
        for (std::size_t i = 0; i + 1 < list.size(); ++i) layout.segments.push_back({ list[i], list[i + 1] });
    };
    const auto closed = [&](const std::vector<int>& list) {
        for (std::size_t i = 0; i < list.size(); ++i) layout.segments.push_back({ list[i], list[(i + 1) % list.size()] });
    };

    const float mouth = r.mouthHeight;
    const float noseBase = r.noseProfile.points().front().first;
    const float noseRoot = r.noseProfile.points().back().first;
    const float tip = noseTip(r);

    // ---- O meio da frente (de baixo para cima): o perfil ----
    const int chinBottom = byLateral(0.0f, r.jawLine(0.0f));
    const int chinLow = byLateral(0.0f, 1.68f);
    const int chinFront = byLateral(0.0f, 1.688f);
    const int sulcus = byLateral(0.0f, 1.698f);
    const int lowerBorder = byLateral(0.0f, mouth - r.lowerLip);
    const int lowerLip = byLateral(0.0f, mouth - 0.45f * r.lowerLip);
    const int mouthMiddle = byLateral(0.0f, mouth, -r.mouthDepth);
    const int upperBorder = byLateral(0.0f, mouth + r.upperLip);
    const int philtrum = byLateral(0.0f, 0.5f * (mouth + r.upperLip + noseBase));
    const int subnasale = byLateral(0.0f, noseBase);
    const int columella = byLateral(0.0f, noseBase + 0.45f * (tip - 0.004f - noseBase));
    const int tipLow = byLateral(0.0f, tip - 0.004f);
    const int tipFront = byLateral(0.0f, tip);
    const int tipHigh = byLateral(0.0f, tip + 0.0045f);
    const int bridge1 = byLateral(0.0f, tip + 0.01f);
    const int bridge2 = byLateral(0.0f, 0.5f * (tip + 0.01f + noseRoot));
    const int bridge3 = byLateral(0.0f, noseRoot - 0.009f);
    const int nasion = byLateral(0.0f, noseRoot);
    const int glabella = byLateral(0.0f, r.ridgeHeight - 0.002f);
    const int brow = byLateral(0.0f, r.ridgeHeight + 0.009f);
    const int forehead = byLateral(0.0f, 1.832f);
    const int foreheadHigh = byLateral(0.0f, 1.856f);
    layout.middle = { chinBottom, chinLow, chinFront, sulcus, lowerBorder, lowerLip, mouthMiddle, upperBorder, philtrum,
        subnasale, columella, tipLow, tipFront, tipHigh, bridge1, bridge2, bridge3, nasion, glabella, brow, forehead,
        foreheadHigh };

    // ---- O olho: a amendoa (rosto.png), a palpebra de cima saltando ----
    {
        const float l = r.eyeLateral, h0 = r.eyeHeight, w = r.eyeHalfWidth, h = 0.5f * r.eyeOpening, t = r.eyeTilt;
        const float d = -r.eyeDepth;
        // Anti-horaria vista de fora (de frente): o canto de fora, a palpebra
        // de baixo, o canto de dentro (pontudo, mais baixo) e a de cima, que
        // sobe mais do lado de dentro. As fracoes medidas na referencia.
        layout.eye = { byLateral(l + w, h0 - 0.37f * h + t, d), byLateral(l + 0.74f * w, h0 - 0.77f * h + 0.6f * t, d),
            byLateral(l + 0.09f * w, h0 - h, d), byLateral(l - 0.71f * w, h0 - 0.93f * h - 0.6f * t, d - 0.0004f),
            byLateral(l - w, h0 - 0.69f * h - t, d - 0.001f), byLateral(l - 0.43f * w, h0 + 0.94f * h - 0.6f * t, d + 0.0008f),
            byLateral(l + 0.14f * w, h0 + 0.96f * h, d + 0.001f), byLateral(l + 0.66f * w, h0 + 0.67f * h + 0.6f * t, d + 0.0006f) };
        Vec3 center {};
        for (const int v : layout.eye) center += layout.vertices[static_cast<std::size_t>(v)].position / 8.0f;
        layout.eyeCenter = center;
        // A linha da palpebra de baixo.
        const int lidInner = byLateral(l - 0.68f * w, h0 - 0.93f * h - 0.0027f, -0.0004f);
        const int lidMiddle = byLateral(l + 0.09f * w, h0 - h - 0.003f, -0.0004f);
        const int lidOuter = byLateral(l + 0.74f * w, h0 - 0.77f * h - 0.0028f, -0.0004f);
        chain({ lidInner, lidMiddle, lidOuter });
    }

    // ---- A sobrancelha: a pegada (o bloco sobe dela em HeadBrow) ----
    {
        const float innerTop = r.browTop.points().front().first, innerBottom = r.browBottom.points().front().first;
        const float outer = std::min(r.browTop.points().back().first, r.browBottom.points().back().first);
        constexpr int Stations = 5;
        layout.browTip = byLateral(r.browTipLateral, r.browTipHeight);
        for (int k = 0; k < Stations; ++k) {
            const float f = static_cast<float>(k) / static_cast<float>(Stations - 1);
            const float bottom = outer + (innerBottom - outer) * f, top = outer + (innerTop - outer) * f;
            layout.browBottom.push_back(byLateral(bottom, r.browBottom(bottom)));
            layout.browTop.push_back(byLateral(top, r.browTop(top)));
        }
    }

    // ---- O nariz: a ponte, a base, as asas, as narinas e o sulco ----
    {
        // A quina da frente da ponte (ate o lado da ponta).
        const int edge1 = byLateral(0.0068f, noseRoot - 0.003f);
        const int edge2 = byLateral(0.0064f, noseRoot - 0.012f);
        const int edge3 = byLateral(0.0065f, tip + 0.02f);
        const int edge4 = byLateral(0.0069f, tip + 0.011f);
        const int tipSide = byLateral(0.0074f, tip + 0.001f);
        chain({ edge1, edge2, edge3, edge4, tipSide });
        byLateral(0.0045f, tip - 0.0035f);
        // A asa: a volta de fora da cupula (o vinco com o rosto) e o miolo.
        std::vector<int> ala;
        for (const float degrees : { 100.0f, 55.0f, 15.0f, -30.0f, -70.0f, -105.0f }) {
            const float a = degrees * DegreesToRadians;
            ala.push_back(byLateral(r.alaLateral + r.alaRadius * std::cos(a), r.alaHeight + r.alaHalfHeight * std::sin(a)));
        }
        for (std::size_t k = 0; k + 1 < ala.size(); ++k) layout.segments.push_back({ ala[k], ala[k + 1] });
        byLateral(r.alaLateral, r.alaHeight);
        for (const float degrees : { 60.0f, -10.0f, -80.0f }) {
            const float a = degrees * DegreesToRadians;
            byLateral(r.alaLateral + 0.55f * r.alaRadius * std::cos(a), r.alaHeight + 0.55f * r.alaHalfHeight * std::sin(a));
        }
        // A base do nariz (onde o lado encontra o rosto) ate o alto da asa.
        const int base1 = byLateral(0.0142f, noseRoot - 0.003f);
        const int base2 = byLateral(0.0142f, noseRoot - 0.013f);
        const int base3 = byLateral(0.0136f, tip + 0.017f);
        chain({ base1, base2, base3, ala.front() });
        // A narina: a borda de dentro da asa, a ponta por cima, a columela.
        const float nl = r.nostrilLateral, nh = r.nostrilHeight, nr = r.nostrilRadius, nhh = r.nostrilHalfHeight;
        const std::vector<int> nostril { ala.back(), byLateral(nl + 0.45f * nr, nh + 0.9f * nhh),
            byLateral(nl - 0.35f * nr, nh + 0.95f * nhh), byLateral(nl - 0.9f * nr, nh + 0.1f * nhh),
            byLateral(nl - 0.4f * nr, nh - 0.95f * nhh), byLateral(nl + 0.4f * nr, nh - nhh) };
        closed(nostril);
        layout.regions.push_back({ nostril, "narina" });
        // O sulco do nariz ao canto da boca.
        const int fold1 = byLateral(0.022f, noseBase - 0.002f);
        const int fold2 = byLateral(0.0275f, mouth + 0.0085f);
        const int fold3 = byLateral(r.mouthWidth + 0.004f, mouth - 0.001f);
        const int fold4 = byLateral(r.mouthWidth + 0.0065f, mouth - r.lowerLip - 0.002f);
        chain({ ala[3], fold1, fold2, fold3, fold4 });
    }

    // ---- A boca: a linha, os contornos dos labios e o filtro ----
    {
        const float m = r.mouthWidth;
        // A linha entra mais no canto; o labio de cima afina ate ele.
        const auto lineDepth = [&](float f) { return -(r.mouthDepth + (r.mouthCornerDepth - r.mouthDepth) * f * f); };
        const int line1 = byLateral(0.3f * m, mouth + 0.0001f, lineDepth(0.3f));
        const int line2 = byLateral(0.62f * m, mouth + 0.0003f, lineDepth(0.62f));
        const int corner = byLateral(m, mouth + 0.0005f, lineDepth(1.0f));
        chain({ mouthMiddle, line1, line2, corner });
        const int peak = byLateral(0.19f * m, mouth + r.lipPeak, 0.0004f);
        const int upper2 = byLateral(0.45f * m, mouth + 0.5f * (r.upperLip + r.lipPeak) - 0.0011f, 0.0002f);
        const int upper3 = byLateral(0.75f * m, mouth + 0.53f * r.upperLip);
        chain({ upperBorder, peak, upper2, upper3, corner });
        const int column1 = byLateral(0.11f * m, noseBase - 0.0012f);
        const int column2 = byLateral(0.16f * m, 0.5f * (mouth + r.lipPeak + noseBase), 0.0003f);
        chain({ subnasale, column1, column2, peak });
        const int lower1 = byLateral(0.3f * m, mouth - 0.95f * r.lowerLip);
        const int lower2 = byLateral(0.55f * m, mouth - 0.72f * r.lowerLip);
        const int lower3 = byLateral(0.76f * m, mouth - 0.36f * r.lowerLip);
        chain({ lowerBorder, lower1, lower2, lower3, corner });
        // A linha mais cheia de cada labio (a almofada aparece nela) e a
        // volta do canto (a covinha fica so nele).
        const int full1 = byLateral(0.24f * m, mouth - 0.47f * r.lowerLip);
        const int full2 = byLateral(0.48f * m, mouth - 0.38f * r.lowerLip);
        const int full3 = byLateral(0.72f * m, mouth - 0.22f * r.lowerLip);
        chain({ lowerLip, full1, full2, full3 });
        const int roll1 = byLateral(0.3f * m, mouth + 0.5f * r.upperLip);
        const int roll2 = byLateral(0.58f * m, mouth + 0.36f * r.upperLip);
        chain({ roll1, roll2 });
        byLateral(m + 0.0028f, mouth - 0.0034f);
        byLateral(m - 0.0005f, mouth + 0.0042f);
    }

    // ---- O queixo e a mandibula ----
    {
        byLateral(0.017f, 1.69f);
        byLateral(0.015f, 1.679f);
        std::vector<int> jaw { chinBottom };
        for (const float degrees : { 10.0f, 18.5f, 26.0f, 39.0f, 52.0f, 65.0f, 79.0f, 90.0f, 96.0f, 98.5f })
            jaw.push_back(byAngle(degrees * DegreesToRadians, r.jawLine(degrees)));
        for (std::size_t k = 0; k + 1 < jaw.size(); ++k) layout.segments.push_back({ jaw[k], jaw[k + 1] });
    }
    // A maca do rosto.
    byLateral(r.cheekLateral, r.cheekHeight);

    // ---- A raiz da orelha: em volta da concha, na direcao de cada ponto do
    // contorno ----
    for (const auto& point : r.earOutline) {
        const float direction = std::atan2(point[1], point[0]);
        const float x = r.earX + r.earRoot * r.earConchaWidth * std::cos(direction);
        const float height = r.earHeight + r.earRoot * r.earConchaHeight * std::sin(direction);
        layout.earRoot.push_back(byAngle(shape.angleForX(height, x), height));
    }
    return layout;
}

} // namespace MatterEngine::Modeling
