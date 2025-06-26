/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_VECTOR4_H
#define GRS_SIMULATOR_VECTOR4_H

#pragma once

#include <iostream>
#include <cmath>

template<typename T>
class Vector4 {
private:
    T m_elements[4];

public:
    Vector4() : m_elements{0, 0, 0, 0} {}

    Vector4(T x, T y, T z, T w) : m_elements{x, y, z, w} {}

    T& operator[](int index) {
        if (index >= 0 && index < 4) {
            return m_elements[index];
        } else {
            throw std::out_of_range("Vector4 index out of range"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    const T& operator[](int index) const {
        if (index >= 0 && index < 4) {
            return m_elements[index];
        } else {
            throw std::out_of_range("Vector4 index out of range"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    T x() const { return m_elements[0]; }
    T y() const { return m_elements[1]; }
    T z() const { return m_elements[2]; }
    T w() const { return m_elements[3]; }

    void setX(T newX) { m_elements[0] = newX; }
    void setY(T newY) { m_elements[1] = newY; }
    void setZ(T newZ) { m_elements[2] = newZ; }
    void setW(T newW) { m_elements[3] = newW; }

    void setAllValues(T newX, T newY, T newZ, T newW) { m_elements[0] = newX; m_elements[1] = newY; m_elements[2] = newZ, m_elements[3] = newW;}

    T length() const {
        return std::sqrt(m_elements[0] * m_elements[0] + m_elements[1] * m_elements[1] + m_elements[2] * m_elements[2] + m_elements[3] * m_elements[3]);
    }

    Vector4 normalize() const {
        T len = length();
        if (len != 0) {
            return Vector4(m_elements[0] / len, m_elements[1] / len, m_elements[2] / len, m_elements[3] / len);
        } else {
            throw std::invalid_argument("Can't normalize zero vector"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    Vector4 operator+(const Vector4& other) const {
        return Vector4(m_elements[0] + other.m_elements[0], m_elements[1] + other.m_elements[1], m_elements[2] + other.m_elements[2] , m_elements[3] + other.m_elements[3]);
    }

    Vector4& operator+=(const Vector4& other) {
        m_elements[0] += other.m_elements[0];
        m_elements[1] += other.m_elements[1];
        m_elements[2] += other.m_elements[2];
        m_elements[3] += other.m_elements[3];
        return *this;
    }

    Vector4 operator-(const Vector4& other) const {
        return Vector4(m_elements[0] - other.m_elements[0], m_elements[1] - other.m_elements[1], m_elements[2] - other.m_elements[2] , m_elements[3] - other.m_elements[3]);
    }

    Vector4 operator-() const {
        return Vector4(-m_elements[0], -m_elements[1], -m_elements[2], -m_elements[3]);
    }

    constexpr friend Vector4 operator*(const T& scalar, Vector4 vector) {
        return Vector4(vector.m_elements[0] * scalar, vector.m_elements[1] * scalar, vector.m_elements[2] * scalar , vector.m_elements[3] * scalar);
    }

    constexpr friend Vector4 operator*(Vector4 vector, const T& scalar) {
        return Vector4(vector.m_elements[0] * scalar, vector.m_elements[1] * scalar, vector.m_elements[2] * scalar , vector.m_elements[3] * scalar);
    }

    Vector4 operator/(const T& scalar) const {
        if (scalar == 0) throw std::invalid_argument("Division by zero"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        return Vector4(m_elements[0] / scalar, m_elements[1] / scalar, m_elements[2] / scalar , m_elements[3] / scalar);
    }

    T dot(const Vector4& other) const {
        return m_elements[0] * other.m_elements[0] + m_elements[1] * other.m_elements[1] + m_elements[2] * other.m_elements[2] + m_elements[3] * other.m_elements[3];
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector4& vec) {
        os << vec.m_elements[0] << "," << vec.m_elements[1] << "," << vec.m_elements[2];
        return os;
    }
};

#endif //GRS_SIMULATOR_VECTOR4_H
