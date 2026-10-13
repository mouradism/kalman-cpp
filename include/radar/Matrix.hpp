#pragma once

#include <cstddef>
#include <array>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace radar::math {

template <std::size_t Rows, std::size_t Cols>
class Matrix {
public:
    std::array<float, Rows * Cols> data{};

    constexpr Matrix() {
        data.fill(0.0f);
    }

    static Matrix identity() {
        static_assert(Rows == Cols, "Identity matrix must be square.");
        Matrix mat;
        for (std::size_t i = 0; i < Rows; ++i) {
            mat(i, i) = 1.0f;
        }
        return mat;
    }

    static Matrix diagonal(const std::array<float, Rows>& diag) {
        static_assert(Rows == Cols, "Diagonal matrix must be square.");
        Matrix mat;
        for (std::size_t i = 0; i < Rows; ++i) {
            mat(i, i) = diag[i];
        }
        return mat;
    }

    constexpr float& operator()(std::size_t r, std::size_t c) {
        return data[r * Cols + c];
    }

    constexpr const float& operator()(std::size_t r, std::size_t c) const {
        return data[r * Cols + c];
    }

    Matrix operator+(const Matrix& other) const {
        Matrix res;
        for (std::size_t i = 0; i < Rows * Cols; ++i) {
            res.data[i] = data[i] + other.data[i];
        }
        return res;
    }

    Matrix operator-(const Matrix& other) const {
        Matrix res;
        for (std::size_t i = 0; i < Rows * Cols; ++i) {
            res.data[i] = data[i] - other.data[i];
        }
        return res;
    }

    Matrix operator*(float scalar) const {
        Matrix res;
        for (std::size_t i = 0; i < Rows * Cols; ++i) {
            res.data[i] = data[i] * scalar;
        }
        return res;
    }

    template <std::size_t OtherCols>
    Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols>& other) const {
        Matrix<Rows, OtherCols> res;
        for (std::size_t i = 0; i < Rows; ++i) {
            for (std::size_t k = 0; k < Cols; ++k) {
                const float a_ik = (*this)(i, k);
                for (std::size_t j = 0; j < OtherCols; ++j) {
                    res(i, j) += a_ik * other(k, j);
                }
            }
        }
        return res;
    }

    Matrix<Cols, Rows> transpose() const {
        Matrix<Cols, Rows> res;
        for (std::size_t i = 0; i < Rows; ++i) {
            for (std::size_t j = 0; j < Cols; ++j) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }

    float trace() const {
        static_assert(Rows == Cols, "Trace defined only for square matrices.");
        float sum = 0.0f;
        for (std::size_t i = 0; i < Rows; ++i) {
            sum += (*this)(i, i);
        }
        return sum;
    }

    void makeSymmetric() {
        static_assert(Rows == Cols, "Matrix must be square to symmetrize.");
        for (std::size_t i = 0; i < Rows; ++i) {
            for (std::size_t j = i + 1; j < Cols; ++j) {
                float avg = 0.5f * ((*this)(i, j) + (*this)(j, i));
                (*this)(i, j) = avg;
                (*this)(j, i) = avg;
            }
        }
    }
};

template <std::size_t Dim>
using Vector = Matrix<Dim, 1>;

template <std::size_t Dim>
inline float dot(const Vector<Dim>& a, const Vector<Dim>& b) {
    float sum = 0.0f;
    for (std::size_t i = 0; i < Dim; ++i) {
        sum += a(i, 0) * b(i, 0);
    }
    return sum;
}

// Exact analytical inversion for 3x3 matrices
inline bool invert3x3(const Matrix<3, 3>& mat, Matrix<3, 3>& out_inv, float& out_det) {
    const float m00 = mat(0, 0), m01 = mat(0, 1), m02 = mat(0, 2);
    const float m10 = mat(1, 0), m11 = mat(1, 1), m12 = mat(1, 2);
    const float m20 = mat(2, 0), m21 = mat(2, 1), m22 = mat(2, 2);

    const float c00 = m11 * m22 - m12 * m21;
    const float c01 = m12 * m20 - m10 * m22;
    const float c02 = m10 * m21 - m11 * m20;

    out_det = m00 * c00 + m01 * c01 + m02 * c02;

    if (std::abs(out_det) < 1e-9f) {
        return false;
    }

    const float inv_det = 1.0f / out_det;

    const float c10 = m02 * m21 - m01 * m22;
    const float c11 = m00 * m22 - m02 * m20;
    const float c12 = m01 * m20 - m00 * m21;

    const float c20 = m01 * m12 - m02 * m11;
    const float c21 = m02 * m10 - m00 * m12;
    const float c22 = m00 * m11 - m01 * m10;

    out_inv(0, 0) = c00 * inv_det;
    out_inv(0, 1) = c10 * inv_det;
    out_inv(0, 2) = c20 * inv_det;

    out_inv(1, 0) = c01 * inv_det;
    out_inv(1, 1) = c11 * inv_det;
    out_inv(1, 2) = c21 * inv_det;

    out_inv(2, 0) = c02 * inv_det;
    out_inv(2, 1) = c12 * inv_det;
    out_inv(2, 2) = c22 * inv_det;

    return true;
}

// Azimuth angle wrapping to [-pi, pi]
inline float wrapToPi(float angle) {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TWO_PI = 2.0f * PI;
    angle = std::fmod(angle + PI, TWO_PI);
    if (angle < 0.0f) {
        angle += TWO_PI;
    }
    return angle - PI;
}

} // namespace radar::math

