#include "Engine/Math/Mat4.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace MatterEngine {

Mat4 Mat4::identity() {
    Mat4 result;
    result.at(0, 0) = 1.0f;
    result.at(1, 1) = 1.0f;
    result.at(2, 2) = 1.0f;
    result.at(3, 3) = 1.0f;
    return result;
}

Mat4 Mat4::translation(Vec3 position) {
    Mat4 result = identity();
    result.at(0, 3) = position.x;
    result.at(1, 3) = position.y;
    result.at(2, 3) = position.z;
    return result;
}

Mat4 Mat4::scale(Vec3 scaleValue) {
    Mat4 result;
    result.at(0, 0) = scaleValue.x;
    result.at(1, 1) = scaleValue.y;
    result.at(2, 2) = scaleValue.z;
    result.at(3, 3) = 1.0f;
    return result;
}

Mat4 Mat4::rotation(Quaternion input) {
    const Quaternion q = input.normalized();
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    Mat4 result = identity();
    result.at(0, 0) = 1.0f - 2.0f * (yy + zz);
    result.at(0, 1) = 2.0f * (xy - wz);
    result.at(0, 2) = 2.0f * (xz + wy);
    result.at(1, 0) = 2.0f * (xy + wz);
    result.at(1, 1) = 1.0f - 2.0f * (xx + zz);
    result.at(1, 2) = 2.0f * (yz - wx);
    result.at(2, 0) = 2.0f * (xz - wy);
    result.at(2, 1) = 2.0f * (yz + wx);
    result.at(2, 2) = 1.0f - 2.0f * (xx + yy);
    return result;
}

Mat4 Mat4::perspective(float verticalFovRadians, float aspect, float nearPlane, float farPlane) {
    if (aspect <= 0.0f || nearPlane <= 0.0f || farPlane <= nearPlane) {
        throw std::invalid_argument("Invalid perspective projection parameters");
    }
    // Profundidade INVERTIDA (Reversed-Z): o plano proximo mapeia para 1,
    // o distante para 0 - ao contrario da convencao "ingenua" (perto=0,
    // longe=1) que este motor usava antes. Padrao usado por engines AAA
    // (Unreal, Frostbite, id Tech) desde ~2010: a projecao perspectiva
    // comprime severamente a precisao do buffer de profundidade em
    // distancias medias/longas (a relacao e ~1/z, nao linear); Reversed-Z
    // redistribui a precisao de forma muito mais uniforme ao longo do
    // intervalo proximo-distante mesmo com profundidade em float32. Sem
    // isso, o resolve de TAA (que reconstroi posicao no mundo a partir da
    // profundidade para reprojetar o historico) acumula erro de precisao
    // que aparece como tremor visivel mesmo com a camera parada - a causa
    // raiz de um bug relatado nesta engine, nao so um ajuste teorico.
    //
    // Consequencias em cascata que TODO consumidor de profundidade desta
    // camera precisa respeitar (ver comentarios nos respectivos arquivos):
    // VulkanDevice.cpp (compareOp do pre-pass GREATER_OR_EQUAL em vez de
    // LESS_OR_EQUAL, clear value 0.0 em vez de 1.0), taa_resolve.frag
    // (checagem de ceu invertida) e Frustum3D::fromViewProjection
    // (formulas de plano proximo/distante trocadas, parametro
    // reversedDepth).
    const float focal = 1.0f / std::tan(verticalFovRadians * 0.5f);
    Mat4 result;
    result.at(0, 0) = focal / aspect;
    result.at(1, 1) = focal;
    result.at(2, 2) = nearPlane / (farPlane - nearPlane);
    result.at(2, 3) = (nearPlane * farPlane) / (farPlane - nearPlane);
    result.at(3, 2) = -1.0f;
    return result;
}

Mat4 Mat4::perspectiveJittered(float verticalFovRadians, float aspect,
    float nearPlane, float farPlane, Vec2 ndcJitter) {
    Mat4 result = perspective(verticalFovRadians, aspect, nearPlane, farPlane);
    // Um deslocamento em NDC (apos a divisao por w) equivale a somar
    // jitter*clipW a clipX/clipY antes da divisao. Como clipW = -viewZ
    // nesta matriz (linha 3, coluna 2 = -1), isso vira -jitter*viewZ - o
    // termo que multiplica viewZ em cada linha e exatamente at(0,2)/at(1,2)
    // (ambos zero na matriz sem jitter, ja que a projecao e simetrica).
    result.at(0, 2) = -ndcJitter.x;
    result.at(1, 2) = -ndcJitter.y;
    return result;
}

Mat4 Mat4::orthographic(float left, float right, float bottom, float top,
    float nearPlane, float farPlane) {
    if (right <= left || top <= bottom || nearPlane <= 0.0f || farPlane <= nearPlane) {
        throw std::invalid_argument("Invalid orthographic projection parameters");
    }
    Mat4 result = identity();
    result.at(0, 0) = 2.0f / (right - left);
    result.at(1, 1) = 2.0f / (top - bottom);
    result.at(2, 2) = 1.0f / (nearPlane - farPlane);
    result.at(0, 3) = -(right + left) / (right - left);
    result.at(1, 3) = -(top + bottom) / (top - bottom);
    result.at(2, 3) = nearPlane / (nearPlane - farPlane);
    return result;
}

Mat4 Mat4::inverse() const {
    float augmented[4][8] {};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            augmented[row][column] = at(row, column);
        }
        augmented[row][row + 4] = 1.0f;
    }

    for (int column = 0; column < 4; ++column) {
        int pivotRow = column;
        for (int row = column + 1; row < 4; ++row) {
            if (std::abs(augmented[row][column]) > std::abs(augmented[pivotRow][column])) {
                pivotRow = row;
            }
        }
        if (std::abs(augmented[pivotRow][column]) <= 0.0000001f) {
            throw std::invalid_argument("Cannot invert a singular matrix");
        }
        if (pivotRow != column) {
            for (int entry = 0; entry < 8; ++entry) {
                std::swap(augmented[column][entry], augmented[pivotRow][entry]);
            }
        }

        const float pivot = augmented[column][column];
        for (int entry = 0; entry < 8; ++entry) {
            augmented[column][entry] /= pivot;
        }
        for (int row = 0; row < 4; ++row) {
            if (row == column) {
                continue;
            }
            const float factor = augmented[row][column];
            for (int entry = 0; entry < 8; ++entry) {
                augmented[row][entry] -= factor * augmented[column][entry];
            }
        }
    }

    Mat4 result;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result.at(row, column) = augmented[row][column + 4];
        }
    }
    return result;
}

Mat4 Mat4::lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 forward = (target - eye).normalized();
    const Vec3 right = cross(forward, up).normalized();
    const Vec3 correctedUp = cross(right, forward);
    Mat4 result = identity();
    result.at(0, 0) = right.x;
    result.at(0, 1) = right.y;
    result.at(0, 2) = right.z;
    result.at(0, 3) = -dot(right, eye);
    result.at(1, 0) = correctedUp.x;
    result.at(1, 1) = correctedUp.y;
    result.at(1, 2) = correctedUp.z;
    result.at(1, 3) = -dot(correctedUp, eye);
    result.at(2, 0) = -forward.x;
    result.at(2, 1) = -forward.y;
    result.at(2, 2) = -forward.z;
    result.at(2, 3) = dot(forward, eye);
    return result;
}

Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 result;
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int k = 0; k < 4; ++k) {
                result.at(row, column) += a.at(row, k) * b.at(k, column);
            }
        }
    }
    return result;
}

Mat4 Transform3D::matrix() const {
    return Mat4::translation(position) * Mat4::rotation(rotation) * Mat4::scale(scale);
}

} // namespace MatterEngine
