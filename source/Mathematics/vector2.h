/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_VECTOR2_H
#define GRS_SIMULATOR_VECTOR2_H

#include "iostream"

template<typename T>
class Vector2 {
private:
    T elements[2];

public:
    Vector2() : elements{0, 0} {}

    Vector2(T x, T y) : elements{x, y} {}

    T& operator[](int index) {
        if (index >= 0 && index < 2) {
            return elements[index];
        } else {
            throw std::out_of_range("Vector2 index out of range"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    const T& operator[](int index) const {
        if (index >= 0 && index < 2) {
            return elements[index];
        } else {
            throw std::out_of_range("Vector2 index out of range"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    T x() const { return elements[0]; }
    T y() const { return elements[1]; }

    void setX(T newX) { elements[0] = newX; }
    void setY(T newY) { elements[1] = newY; }

    T magnitude() const {
        return sqrt(elements[0] * elements[0] + elements[1] * elements[1]);
    }

    Vector2<T> normalize() const {
        T mag = magnitude();
        if (mag != 0) {
            return Vector2<T>(elements[0] / mag, elements[1] / mag);
        } else {
            throw std::invalid_argument("Can't normalize zero vector"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }

    Vector2 operator+(const Vector2& other) const {
        return Vector2(elements[0] + other.elements[0], elements[1] + other.elements[1]);
    }

    Vector2 operator-(const Vector2& other) const {
        return Vector2(elements[0] - other.elements[0], elements[1] - other.elements[1]);
    }

    Vector2 operator+=(const Vector2& other) const {
        return Vector2(elements[0] + other.elements[0], elements[1] + other.elements[1]);
    }

    constexpr friend inline Vector2 operator*(const T& scalar, Vector2 vector) {
        return Vector2(vector.elements[0] * scalar, vector.elements[1] * scalar);
    }

    constexpr friend inline Vector2 operator*(Vector2 vector, const T& scalar) {
        return Vector2(vector.elements[0] * scalar, vector.elements[1] * scalar);
    }

    T dot(const Vector2& other) const {
        return elements[0] * other.elements[0] + elements[1] * other.elements[1];
    }

    friend std::ostream& operator<<(std::ostream& os, const Vector2& vec) {
        os << "[" << vec.elements[0] << ", " << vec.elements[1] << "]";
        return os;
    }
};

#endif //GRS_SIMULATOR_VECTOR2_H
