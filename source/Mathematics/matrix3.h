/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MATRIX3_H
#define GRS_SIMULATOR_MATRIX3_H

#ifndef Matrix3_H
#define Matrix3_H

#include <cmath>
#include <stdexcept>

#include "Mathematics/vector2.h"
#include "Mathematics/vector3.h"

template<typename T>
class Matrix2 {
private:
    Vector2<Vector2<T>> data;

public:
    Matrix2() : data(Vector2<T>(), Vector2<T>()) {}

    Matrix2(std::initializer_list<std::initializer_list<T>> row_list) : data(Vector2<T>(), Vector2<T>()) {
        if (row_list.size() != 2) {
            throw std::invalid_argument("Matrix2 initializer_list should have 2 rows");
        }

        int i = 0;
        for (auto& row : row_list) {
            if (row.size() != 2) {
                throw std::invalid_argument("Each row in Matrix2 initializer_list should have 2 elements");
            }
            int j = 0;
            for (auto& val : row) {
                data[i][j] = val;
                ++j;
            }
            ++i;
        }
    }

    // Access by index
    Vector2<T>& operator[](int index) {
        if (index >= 0 && index < 2) {
            return data[index];
        } else {
            throw std::out_of_range("Matrix2 index out of range");
        }
    }

    // Const access by index
    const Vector2<T>& operator[](int index) const {
        if (index >= 0 && index < 2) {
            return data[index];
        } else {
            throw std::out_of_range("Matrix2 index out of range");
        }
    }

    // Access the individual elements
    T& operator()(const unsigned& row, const unsigned& col) {
        return this->data[row][col];
    }

    // Access the individual elements (const)
    const T& operator()(const unsigned& row, const unsigned& col) const {
        return this->data[row][col];
    }

    T determinant() {
        T det = (this->data[0][0] * this->data[1][1]) - (this->data[0][1] * this->data[1][0]);

        return det;
    }

};



template<typename T>
class Matrix3 {
private:
    Vector3<Vector3<T>> data;

public:
    Matrix3() : data(Vector3<T>(), Vector3<T>(), Vector3<T>()) {}

    Matrix3(std::initializer_list<std::initializer_list<T>> row_list) : data(Vector3<T>(), Vector3<T>(), Vector3<T>()) {
        if (row_list.size() != 3) {
            throw std::invalid_argument("Matrix3 initializer_list should have 3 rows");
        }

        int i = 0;
        for (auto& row : row_list) {
            if (row.size() != 3) {
                throw std::invalid_argument("Each row in Matrix3 initializer_list should have 3 elements");
            }
            int j = 0;
            for (auto& val : row) {
                data[i][j] = val;
                ++j;
            }
            ++i;
        }
    }

    // Access by index
    Vector3<T>& operator[](int index) {
        if (index >= 0 && index < 3) {
            return data[index];
        } else {
            throw std::out_of_range("Matrix3 index out of range");
        }
    }

    // Const access by index
    const Vector3<T>& operator[](int index) const {
        if (index >= 0 && index < 3) {
            return data[index];
        } else {
            throw std::out_of_range("Matrix3 index out of range");
        }
    }

    // Rotation Matrix3
    // Create rotation matrix around the X-axis
    //
    //        | 1             0            0  |
    // R_AB = | 0    cos(theta)    sin(theta) |
    //        | 0   -sin(theta)    cos(theta) |
    //
    static Matrix3<T> makeXRotation(const T &angle) {
        const T cosAngle = std::cos(angle);
        const T sinAngle = std::sin(angle);

        Matrix3<T> result
                {
                        {1,         0,        0},
                        {0,  cosAngle, sinAngle},
                        {0, -sinAngle, cosAngle}
                };

        return result;
    }

    // Create rotation matrix around the Y-axis
    //
    //        |  cos(theta)   0   -sin(theta) |
    // R_AB = |          0    1            0  |
    //        |  sin(theta)   0    cos(theta) |
    //
    static Matrix3<T> makeYRotation(const T &angle) {

        const T cosAngle = std::cos(angle);
        const T sinAngle = std::sin(angle);

        Matrix3<T> result
                {
                        { cosAngle, 0, -sinAngle},
                        {        0, 1,         0},
                        { sinAngle, 0,  cosAngle}
                };

        return result;
    }

    // Create rotation matrix around the Z-axis
    //
    //        |  cos(theta)   sin(theta)   0 |
    // R_AB = | -sin(theta)   cos(theta)   0 |
    //        |          0            0    1 |
    //
    static Matrix3<T> makeZRotation(const T &angle) {

        const T cosAngle = std::cos(angle);
        const T sinAngle = std::sin(angle);

        Matrix3<T> result
                {
                        { cosAngle,  sinAngle, 0},
                        {-sinAngle,  cosAngle, 0},
                        {        0,         0, 1}
                };

        return result;
    }

    // Create identity matrix
    static Matrix3<T> eye() {

        Matrix3<T> result
                {
                        {1, 0, 0},
                        {0, 1, 0},
                        {0, 0, 1}
                };

        return result;
    }

    // Create identity matrix
    static Matrix3<T> zeros() {

        Matrix3<T> result
                {
                        {0, 0, 0},
                        {0, 0, 0},
                        {0, 0, 0}
                };

        return result;
    }

    // Access the individual elements
    T& operator()(const unsigned& row, const unsigned& col) {
        return this->data[row][col];
    }

    // Access the individual elements (const)
    const T& operator()(const unsigned& row, const unsigned& col) const {
        return this->data[row][col];
    }

    // Addition of two matrices
    Matrix3<T> operator+(const Matrix3<T>& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[i][j] + rhs(i,j);
            }
        }

        return result;
    }

    // Cumulative addition of this matrix and another
    Matrix3<T>& operator+=(const Matrix3<T>& rhs) {
        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                this->data[i][j] += rhs(i,j);
            }
        }

        return *this;
    }

    // Subtraction of this matrix and another
    Matrix3<T> operator-(const Matrix3<T>& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[i][j] - rhs(i,j);
            }
        }

        return result;
    }

    // Cumulative subtraction of this matrix and another
    Matrix3<T>& operator-=(const Matrix3<T>& rhs) {
        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                this->data[i][j] -= rhs(i,j);
            }
        }

        return *this;
    }

    // Left multiplication of this matrix and another
    Matrix3<T> operator*(const Matrix3<T>& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                for (unsigned k=0; k<3; k++) {
                    result(i,j) += this->data[i][k] * rhs(k,j);
                }
            }
        }

        return result;
    }

    // Cumulative left multiplication of this matrix and another
    Matrix3<T>& operator*=(const Matrix3<T>& rhs) {
        Matrix3 result = (*this) * rhs;
        (*this) = result;
        return *this;
    }

    // Calculate a transpose of this matrix
    Matrix3<T> transpose() {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[j][i];
            }
        }

        return result;
    }

    T determinant() {
        Matrix2<double> minor1{{data[1][1], data[1][2]}, {data[2][1], data[2][2]}};
        Matrix2<double> minor2{{data[1][0], data[1][2]}, {data[2][0], data[2][2]}};
        Matrix2<double> minor3{{data[1][0], data[1][1]}, {data[2][0], data[2][1]}};

        T det1 = minor1.determinant();
        T det2 = minor2.determinant();
        T det3 = minor3.determinant();

        return data[0][0] * det1 - data[0][1] * det2 + data[0][2] * det3;
    }

    // Matrix/scalar addition
    Matrix3<T> operator+(const T& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[i][j] + rhs;
            }
        }

        return result;
    }

    // Matrix/scalar subtraction
    Matrix3<T> operator-(const T& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[i][j] - rhs;
            }
        }

        return result;
    }

    // Matrix/scalar multiplication
    Matrix3<T> operator*(const T& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[i][j] * rhs;
            }
        }

        return result;
    }

    // Matrix/scalar division
    Matrix3<T> operator/(const T& rhs) {
        Matrix3 result = zeros();

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result(i,j) = this->data[i][j] / rhs;
            }
        }

        return result;
    }

    // Multiply a matrix with a vector
    Vector3<T> operator*(const Vector3<T>& rhs) {
        Vector3<T> result;

        for (unsigned i=0; i<3; i++) {
            for (unsigned j=0; j<3; j++) {
                result[i] += this->data[i][j] * rhs[j];
            }
        }

        return result;
    }

    // Print matrix values
    void print() const {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                std::cout << data[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

protected:

    void _check_square(const unsigned Matrix3Cols, const unsigned Matrix3Lines){
        if(Matrix3Cols != Matrix3Lines) {
            throw std::invalid_argument("Matrix3 not squared ->"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        };
    }

    void _check_dimensions(const unsigned Matrix3Cols, const unsigned Matrix3Lines){
        if(Matrix3Cols != Matrix3Lines) {
            throw std::invalid_argument("Dimensions mismatch ->"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
        }
    }
};

#endif


#endif //GRS_SIMULATOR_MATRIX3_H
