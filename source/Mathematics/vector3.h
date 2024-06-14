/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_VECTOR3_H
#define GRS_SIMULATOR_VECTOR3_H

#include <iostream>
#include <cmath>

template<typename T>
class Vector3 {
private:
    T elements[3];

public:
    Vector3() : elements{0, 0, 0} {}

    Vector3(T x, T y, T z) : elements{x, y, z} {}

    T& operator[](int index) {
        if (index >= 0 && index < 3) {
            return elements[index];
        } else {
            throw std::out_of_range("Vector3 index out of range"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    const T& operator[](int index) const {
        if (index >= 0 && index < 3) {
            return elements[index];
        } else {
            throw std::out_of_range("Vector3 index out of range"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    T x() const { return elements[0]; }
    T y() const { return elements[1]; }
    T z() const { return elements[2]; }

    void setX(T newX) { elements[0] = newX; }
    void setY(T newY) { elements[1] = newY; }
    void setZ(T newZ) { elements[2] = newZ; }

    T magnitude() const {
        return std::sqrt(elements[0] * elements[0] + elements[1] * elements[1] + elements[2] * elements[2]);
    }

    Vector3<T> normalize() const {
        T mag = magnitude();
        if (mag != 0) {
            return Vector3<T>(elements[0] / mag, elements[1] / mag, elements[2] / mag);
        } else {
            throw std::invalid_argument("Can't normalize zero vector"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    Vector3 operator+(const Vector3& other) const {
        return Vector3(elements[0] + other.elements[0], elements[1] + other.elements[1], elements[2] + other.elements[2]);
    }

    Vector3 operator+=(const Vector3& other) const {
        return Vector3(elements[0] + other.elements[0], elements[1] + other.elements[1], elements[2] + other.elements[2]);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(elements[0] - other.elements[0], elements[1] - other.elements[1], elements[2] - other.elements[2]);
    }

    Vector3 operator-() const {
        return Vector3(-elements[0], -elements[1], -elements[2]);
    }

    constexpr friend inline Vector3 operator*(const T& scalar, Vector3 vector) {
        return Vector3(vector.elements[0] * scalar, vector.elements[1] * scalar, vector.elements[2] * scalar);
    }

    constexpr friend inline Vector3 operator*(Vector3 vector, const T& scalar) {
        return Vector3(vector.elements[0] * scalar, vector.elements[1] * scalar, vector.elements[2] * scalar);
    }

    Vector3 operator/(const T& scalar) const {
        if (scalar == 0) throw std::invalid_argument("Division by zero"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        return Vector3(elements[0] / scalar, elements[1] / scalar, elements[2] / scalar);
    }

    T dot(const Vector3& other) const {
        return elements[0] * other.elements[0] + elements[1] * other.elements[1] + elements[2] * other.elements[2];
    }

    Vector3 cross(const Vector3& other) const {
        return Vector3(elements[1] * other.elements[2] - elements[2] * other.elements[1],
                       elements[2] * other.elements[0] - elements[0] * other.elements[2],
                       elements[0] * other.elements[1] - elements[1] * other.elements[0]);
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector3& vec) {
        os << "[" << vec.elements[0] << ", " << vec.elements[1] << ", " << vec.elements[2] << "]";
        return os;
    }
};

#endif //GRS_SIMULATOR_VECTOR3_H
