#include "Engine/Math/Frustum3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

Plane3D normalizedPlane(float x, float y, float z, float distance) {
    const float length = std::sqrt(x * x + y * y + z * z);
    if (length <= 1.0e-8f) return {};
    const float inverse = 1.0f / length;
    return { { x * inverse, y * inverse, z * inverse },
        distance * inverse };
}

} // namespace

Frustum3D Frustum3D::fromViewProjection(const Mat4& matrix,
    bool reversedDepth) {
    const auto planeFromRows = [&](int firstRow, float firstScale,
            int secondRow, float secondScale) {
        return normalizedPlane(
            matrix.at(firstRow, 0) * firstScale
                + matrix.at(secondRow, 0) * secondScale,
            matrix.at(firstRow, 1) * firstScale
                + matrix.at(secondRow, 1) * secondScale,
            matrix.at(firstRow, 2) * firstScale
                + matrix.at(secondRow, 2) * secondScale,
            matrix.at(firstRow, 3) * firstScale
                + matrix.at(secondRow, 3) * secondScale);
    };

    Frustum3D result;
    result.m_planes[0] = planeFromRows(3, 1.0f, 0, 1.0f);  // esquerda
    result.m_planes[1] = planeFromRows(3, 1.0f, 0, -1.0f); // direita
    result.m_planes[2] = planeFromRows(3, 1.0f, 1, 1.0f);  // inferior
    result.m_planes[3] = planeFromRows(3, 1.0f, 1, -1.0f); // superior

    // Em clip space Vulkan de profundidade PADRAO (perto=0, longe=1), o
    // plano proximo e z >= 0 (linha 2 pura) e o distante e w - z >= 0
    // (linha 3 menos linha 2). Com profundidade INVERTIDA (perto=1,
    // longe=0, ver Mat4::perspective) essas duas formulas trocam de lugar:
    // "dentro do frustum perto do olho" passa a ser w - z >= 0, e "antes do
    // plano distante" passa a ser z >= 0.
    const Plane3D nearFromRow2 = normalizedPlane(matrix.at(2, 0),
        matrix.at(2, 1), matrix.at(2, 2), matrix.at(2, 3));
    const Plane3D farFromRow3MinusRow2 = planeFromRows(3, 1.0f, 2, -1.0f);
    if (reversedDepth) {
        result.m_planes[4] = farFromRow3MinusRow2; // proximo
        result.m_planes[5] = nearFromRow2;          // distante
    } else {
        result.m_planes[4] = nearFromRow2;          // proximo
        result.m_planes[5] = farFromRow3MinusRow2;  // distante
    }
    return result;
}

bool Frustum3D::intersectsSphere(Vec3 center, float radius) const {
    radius = std::max(0.0f, radius);
    for (const Plane3D& plane : m_planes) {
        if (dot(plane.normal, center) + plane.distance < -radius) {
            return false;
        }
    }
    return true;
}

} // namespace MatterEngine
