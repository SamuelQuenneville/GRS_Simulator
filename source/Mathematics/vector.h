/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_VECTOR_H
#define GRS_SIMULATOR_VECTOR_H

#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <ostream>
#include <type_traits>

namespace grs {

template <typename T, std::size_t N>
struct Vector {
    static_assert(N > 0, "Vector dimension must be > 0");
    static_assert(std::is_arithmetic_v<T>, "Vector scalar type must be arithmetic");

    std::array<T, N> data{};

    // Constructor from initializer list
    constexpr Vector(std::initializer_list<T> values) {
        std::size_t count = std::min(values.size(), data.size());
        std::copy_n(values.begin(), count, data.begin());
        if (count < data.size()) {
            std::fill(data.begin() + count, data.end(), T{});
        }
    }

    template <typename U>
    constexpr explicit Vector(const Vector<U, N>& vector) {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] = static_cast<T>(vector[i]);
        }
    }

    // Dimension-checked convenience constructors
    constexpr Vector(T x, T y) {
        static_assert(N == 2, "This constructor is only valid for Vector<T,2>");
        data[0] = x;
        data[1] = y;
    }

    constexpr Vector(T x, T y, T z) {
        static_assert(N == 3, "This constructor is only valid for Vector<T,3>");
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    constexpr Vector(T x, T y, T z, T w) {
        static_assert(N == 4, "This constructor is only valid for Vector<T,4>");
        data[0] = x;
        data[1] = y;
        data[2] = z;
        data[3] = w;
    }

    // Element access
    constexpr T& operator[](std::size_t i) noexcept { return data[i]; }
    constexpr const T& operator[](std::size_t i) const noexcept { return data[i]; }

    [[nodiscard]] static constexpr Vector zeros() noexcept {
        Vector R{};
        for (std::size_t i = 0; i < N; ++i) {
            R[i] = T(0);
        }
        return R;
    }

    // += operator
    constexpr Vector& operator+=(const Vector& vector) noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] += vector[i];
        }
        return *this;
    }

    // -= operator
    constexpr Vector& operator-=(const Vector& vector) noexcept {
        for (std::size_t i = 0; i < N; ++i) {
            data[i] -= vector[i];
        }
        return *this;
    }

    [[nodiscard]] static constexpr std::size_t size() noexcept { return N; }

    [[nodiscard]] T norm() const noexcept {
        T sum{};
        for (std::size_t i = 0; i < N; ++i) {
            sum += data[i] * data[i];
        }
        return std::sqrt(sum);
    }

    // OpenGL reference mapping
    [[nodiscard]] constexpr Vector toOpenglRef() const noexcept {
        if constexpr (N == 3) {
            return Vector<T,3>{(*this)[0], -(*this)[2], (*this)[1]};
        } else {
            static_assert(N == 3, "Not implemented for this size");
            return Vector<T,3>{}; // never executed
        }
    }
};

// equality
template <typename T, std::size_t N>
constexpr bool operator==(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    for (std::size_t i = 0; i < N; ++i) {
        if (vector1[i] != vector2[i]) return false;
    }
    return true;
}

// addition
template <typename T, std::size_t N>
constexpr Vector<T,N> operator+(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    Vector<T,N> r{};
    for (std::size_t i = 0; i < N; ++i) {
        r[i] = vector1[i] + vector2[i];
    }
    return r;
}

// Subtraction
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> operator-(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    Vector<T,N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = vector1[i] - vector2[i];
    }
    return result;
}

template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> operator-(const Vector<T,N>& v) noexcept {
    Vector<T,N> result{};
    for (std::size_t i = 0; i < N; ++i) {
        result[i] = -v[i];
    }
    return result;
}

// scalar multiplication
template <typename T, std::size_t N>
constexpr Vector<T,N> operator*(const Vector<T,N>& vector, T scalar) noexcept {
    Vector<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        R[i] = vector[i] * scalar;
    }
    return R;
}

template <typename T, std::size_t N>
constexpr Vector<T,N> operator*(T scalar, const Vector<T,N>& vector) noexcept {
    return vector * scalar;
}

// dot product
template <typename T, std::size_t N>
[[nodiscard]] constexpr T dot(const Vector<T,N>& vector1, const Vector<T,N>& vector2) noexcept {
    T sum{};
    for (std::size_t i = 0; i < N; ++i) {
        sum += vector1[i] * vector2[i];
    }
    return sum;
}

// normalization
template <typename T, std::size_t N>
[[nodiscard]] Vector<T,N> normalize(const Vector<T,N>& vector) noexcept {
    T n = vector.norm();
    return (n > T(0)) ? (vector * (1/n)) : vector;
}

// cross product (only for 3D vectors)
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> cross(const Vector<T,N>&, const Vector<T,N>&) = delete;

template <typename T>
[[nodiscard]] constexpr Vector<T,3> cross(const Vector<T,3>& vector1, const Vector<T,3>& vector2) noexcept {
    return Vector<T,3>{
        vector1[1]*vector2[2] - vector1[2]*vector2[1],
        vector1[2]*vector2[0] - vector1[0]*vector2[2],
        vector1[0]*vector2[1] - vector1[1]*vector2[0]
    };
}

// Linear Interpolation
template <typename T, std::size_t N>
[[nodiscard]] constexpr Vector<T,N> lerp(const Vector<T,N>& vector1, const Vector<T,N>& vector2, T t) noexcept {
    Vector<T,N> R{};
    for (std::size_t i = 0; i < N; ++i) {
        R[i] = vector1[i] + (vector2[i] - vector1[i]) * t;
    }
    return R;
}

template <typename T, std::size_t N, typename U>
Vector<T,N> cast(const Vector<U,N>& v) {
    return Vector<T,N>(v);
}

// stream output
template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<T,N>& v) {
    os << '[';
    for (std::size_t i = 0; i < N; ++i) {
        os << v[i];
        if (i + 1 < N) os << ", ";
    }
    os << ']';
    return os;
}

}

#endif //GRS_SIMULATOR_VECTOR_H
