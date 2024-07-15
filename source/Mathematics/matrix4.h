/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MATRIX4_H
#define GRS_SIMULATOR_MATRIX4_H

#include <cmath>
#include <cassert>
#include <iostream>
#include "grsMath.h"
#include "vector3.h"
#include "vector4.h"
#include "matrix3.h"

template<typename T>
class Matrix4 {
private:
    Vector4<Vector4<T>> m_data;

public:
    Matrix4() : m_data(Vector4<T>(), Vector4<T>(), Vector4<T>() , Vector4<T>()) {}

    Matrix4(std::initializer_list<std::initializer_list<T>> rowList) : m_data(Vector4<T>(), Vector4<T>(), Vector4<T>() , Vector4<T>()) {
        if (rowList.size() != 4) {
            throw std::invalid_argument("Matrix4 initializer_list should have 4 rows");
        }

        int i = 0;
        for (auto& row : rowList) {
            if (row.size() != 4) {
                throw std::invalid_argument("Each row in Matrix4 initializer_list should have 4 elements");
            }
            int j = 0;
            for (auto& val : row) {
                m_data[i][j] = val;
                ++j;
            }
            ++i;
        }
    }
    // Constructor
    Matrix4(float m_00=0, float m_01=0, float m_02=0, float m_03=0,
            float m_10=0, float m_11=0, float m_12=0, float m_13=0,
            float m_20=0, float m_21=0, float m_22=0, float m_23=0,
            float m_30=0, float m_31=0, float m_32=0, float m_33=0) {
        m_data[0][0] = m_00; m_data[0][1] = m_01; m_data[0][2] = m_02;  m_data[0][3] = m_03;
        m_data[1][0] = m_10; m_data[1][1] = m_11; m_data[1][2] = m_12;  m_data[1][3] = m_13;
        m_data[2][0] = m_20; m_data[2][1] = m_21; m_data[2][2] = m_22;  m_data[2][3] = m_23;
        m_data[3][0] = m_30; m_data[3][1] = m_31; m_data[3][2] = m_32;  m_data[3][3] = m_33;
    }

    // Constructor
    Matrix4(float n[4][4]) {
        m_data[0][0]=n[0][0]; m_data[0][1]=n[0][1]; m_data[0][2]=n[0][2]; m_data[0][3]=n[0][3];
        m_data[1][0]=n[1][0]; m_data[1][1]=n[1][1]; m_data[1][2]=n[1][2]; m_data[1][3]=n[1][3];
        m_data[2][0]=n[2][0]; m_data[2][1]=n[2][1]; m_data[2][2]=n[2][2]; m_data[2][3]=n[2][3];
        m_data[3][0]=n[3][0]; m_data[3][1]=n[3][1]; m_data[3][2]=n[3][2]; m_data[3][3]=n[3][3];
    }

    // Constructor
    Matrix4(const Vector3<T>& a1, const Vector3<T>& a2, const Vector3<T>& a3) {
        m_data[0][0] = a1.x; m_data[0][1] = a2.x; m_data[0][2] = a3.x;  m_data[0][3] = 0.f;
        m_data[1][0] = a1.y; m_data[1][1] = a2.y; m_data[1][2] = a3.y;  m_data[1][3] = 0.f;
        m_data[2][0] = a1.z; m_data[2][1] = a2.z; m_data[2][2] = a3.z;  m_data[2][3] = 0.f;
        m_data[3][0] = 0.f;  m_data[3][1] = 0.f;  m_data[3][2] = 0.f;   m_data[3][3] = 1.f;
    }

    // Constructor
    Matrix4(const Vector4<T>& a1, const Vector4<T>& a2, const Vector4<T>& a3) {
        m_data[0][0] = a1.x; m_data[0][1] = a2.x; m_data[0][2] = a3.x;  m_data[0][3] = 0.f;
        m_data[1][0] = a1.y; m_data[1][1] = a2.y; m_data[1][2] = a3.y;  m_data[1][3] = 0.f;
        m_data[2][0] = a1.z; m_data[2][1] = a2.z; m_data[2][2] = a3.z;  m_data[2][3] = 0.f;
        m_data[3][0] = a1.w; m_data[3][1] = a2.w; m_data[3][2] = a3.w;  m_data[3][3] = 1.f;
    }

    // Constructor
    Matrix4(const Matrix4<T>& matrix) {

        setAllValues(matrix.m_data[0][0], matrix.m_data[0][1], matrix.m_data[0][2], matrix.m_data[0][3],
                     matrix.m_data[1][0], matrix.m_data[1][1], matrix.m_data[1][2], matrix.m_data[1][3],
                     matrix.m_data[2][0], matrix.m_data[2][1], matrix.m_data[2][2], matrix.m_data[2][3],
                     matrix.m_data[3][0], matrix.m_data[3][1], matrix.m_data[3][2], matrix.m_data[3][3]);
    }

    // Create zeros matrix
    static Matrix4<T> zeros() {

        Matrix4<T> result
                {
                        {0, 0, 0, 0},
                        {0, 0, 0, 0},
                        {0, 0, 0, 0}
                };

        return result;
    }

    // + operator
    Matrix4<T> operator+(const Matrix4<T>& rhs) const {
        Matrix4 result = zeros();

        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                result(i,j) = this->m_data[i][j] + rhs(i,j);
            }
        }

        return result;
    }

    // += operator
    Matrix4<T>& operator+=(const Matrix4<T>& rhs) {
        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                this->m_data[i][j] += rhs(i,j);
            }
        }

        return *this;
    }

    // - operator
    Matrix4<T> operator-(const Matrix4<T>& rhs) const {
        Matrix4 result = zeros();

        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                result(i,j) = this->m_data[i][j] - rhs(i,j);
            }
        }

        return result;
    }

    // -= operator
    Matrix4& operator-=(const Matrix4& rhs) {
        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                this->m_data[i][j] -= rhs(i,j);
            }
        }

        return *this;
    }

    // = operator
    Matrix4& operator=(const Matrix4& matrix) {
        if (&matrix != this) {
            setAllValues(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
                         matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
                         matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
                         matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
        }
        return *this;
    }

    // == operator
    bool operator==(const Matrix4& rhs) const {
        return m_data[0][0]==rhs.m_data[0][0] && m_data[0][1]==rhs.m_data[0][1] && m_data[0][2]==rhs.m_data[0][2] && m_data[0][3]==rhs.m_data[0][3] &&
               m_data[1][0]==rhs.m_data[1][0] && m_data[1][1]==rhs.m_data[1][1] && m_data[1][2]==rhs.m_data[1][2] && m_data[1][3]==rhs.m_data[1][3] &&
               m_data[2][0]==rhs.m_data[2][0] && m_data[2][1]==rhs.m_data[2][1] && m_data[2][2]==rhs.m_data[2][2] && m_data[2][3]==rhs.m_data[2][3] &&
               m_data[3][0]==rhs.m_data[3][0] && m_data[3][1]==rhs.m_data[3][1] && m_data[3][2]==rhs.m_data[3][2] && m_data[3][3]==rhs.m_data[3][3];
    }

    // * operator
    Matrix4 operator*(const Matrix4& rhs) const {
        Matrix4 result = zeros();

        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                for (unsigned k=0; k<4; k++) {
                    result(i,j) += this->m_data[i][k] * rhs(k,j);
                }
            }
        }

        return result;
    }

    // * operator
    Vector3<T> operator*(const Vector3<T>& rhs) const {
        Vector3 u =Vector3(m_data[0][0]*rhs.x + m_data[0][1]*rhs.y + m_data[0][2]*rhs.z + m_data[0][3],
                           m_data[1][0]*rhs.x + m_data[1][1]*rhs.y + m_data[1][2]*rhs.z + m_data[1][3],
                           m_data[2][0]*rhs.x + m_data[2][1]*rhs.y + m_data[2][2]*rhs.z + m_data[2][3]);
        float w = m_data[3][0]*rhs.x + m_data[3][1]*rhs.y + m_data[3][2]*rhs.z + m_data[3][3];
        return u/w;
    }

    // * operator
    Vector4<T> operator*(const Vector4<T>& rhs) const {
        Vector4 u = Vector4(m_data[0][0]*rhs.x + m_data[0][1]*rhs.y + m_data[0][2]*rhs.z + rhs.w*m_data[0][3],
                            m_data[1][0]*rhs.x + m_data[1][1]*rhs.y + m_data[1][2]*rhs.z + rhs.w*m_data[1][3],
                            m_data[2][0]*rhs.x + m_data[2][1]*rhs.y + m_data[2][2]*rhs.z + rhs.w*m_data[2][3],
                            m_data[3][0]*rhs.x + m_data[3][1]*rhs.y + m_data[3][2]*rhs.z + rhs.w*m_data[3][3]);
        if(u.w != 0)
            return u/u.w;
        else
            return u;
    }

    // * operator
    Matrix4<T> operator*(T& rhs) const {
        Matrix4 result = zeros();

        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                result(i,j) = this->m_data[i][j] * rhs;
            }
        }

        return result;
    }

    // * operator
    Matrix4<T>& operator*=(T rhs) {
        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                this->m_data[i][j] *= rhs(i,j);
            }
        }

        return *this;
    }

    // / operator
    Matrix4<T> operator/(T rhs) const {
        assert(rhs!=0);
        return Matrix4<T>(m_data[0][0]/rhs, m_data[0][1]/rhs, m_data[0][2]/rhs,  m_data[0][3]/rhs,
                          m_data[1][0]/rhs, m_data[1][1]/rhs, m_data[1][2]/rhs,  m_data[1][3]/rhs,
                          m_data[2][0]/rhs, m_data[2][1]/rhs, m_data[2][2]/rhs,  m_data[2][3]/rhs,
                          m_data[3][0]/rhs, m_data[3][1]/rhs, m_data[3][2]/rhs,  m_data[3][3]/rhs);
    }

    // /= operator
    Matrix4<T> &operator/=(T rhs) {
        assert(rhs!=0);
        for (unsigned i=0; i<4; i++) {
            for (unsigned j=0; j<4; j++) {
                this->m_data[i][j] /= rhs(i,j);
            }
        }

        return *this;
    }

    // - operator
    Matrix4<T> operator-() const {
        return Matrix4<T>(-m_data[0][0], -m_data[0][1], -m_data[0][2],  -m_data[0][3],
                          -m_data[1][0], -m_data[1][1], -m_data[1][2],  -m_data[1][3],
                          -m_data[2][0], -m_data[2][1], -m_data[2][2],  -m_data[2][3],
                          -m_data[3][0], -m_data[3][1], -m_data[3][2],  -m_data[3][3]);
    }

    // Return the transpose matrix
    Matrix4<T> getTranspose() const {
        return Matrix4<T>(m_data[0][0], m_data[1][0], m_data[2][0], m_data[3][0],
                          m_data[0][1], m_data[1][1], m_data[2][1], m_data[3][1],
                          m_data[0][2], m_data[1][2], m_data[2][2], m_data[3][2],
                          m_data[0][3], m_data[1][3], m_data[2][3], m_data[3][3]);
    }

    // Return the 3x3 upper-left matrix
    Matrix3<T> getUpperLeft3x3Matrix() const {
        return Matrix3<T>(m_data[0][0], m_data[0][1], m_data[0][2],
                          m_data[1][0], m_data[1][1], m_data[1][2],
                          m_data[2][0], m_data[2][1], m_data[2][2]);
    }

    // Return the inversed matrix
    Matrix4 getInverse() const {
        int indxc[4], indxr[4];
        int ipiv[4] = { 0, 0, 0, 0 };
        float minv[4][4];
        float temp;

        for (int s=0; s<4; s++) {
            for (int t=0; t<4; t++) {
                minv[s][t] = m_data[s][t];
            }
        }

        for (int i = 0; i < 4; i++) {
            int irow = -1, icol = -1;
            float big = 0.;
            // Choose pivot
            for (int j = 0; j < 4; j++) {
                if (ipiv[j] != 1) {
                    for (int k = 0; k < 4; k++) {
                        if (ipiv[k] == 0) {
                            if (std::fabs(minv[j][k]) >= big) {
                                big = float(std::fabs(minv[j][k]));
                                irow = j;
                                icol = k;
                            }
                        }
                        else if (ipiv[k] > 1) {
                            std::cout << "ERROR: Singular matrix in MatrixInvert\n";
                        }
                    }
                }
            }
            ++ipiv[icol];
            // Swap rows _irow_ and _icol_ for pivot
            if (irow != icol) {
                for (int k = 0; k < 4; ++k){
                    temp = minv[irow][k];
                    minv[irow][k] = minv[icol][k];
                    minv[icol][k] = temp;
                }
            }
            indxr[i] = irow;
            indxc[i] = icol;
            if (minv[icol][icol] == 0.){
                std::cout << "Singular matrix in MatrixInvert\n";
            }
            // Set $m[icol][icol]$ to one by scaling row _icol_ appropriately
            float pivinv = 1.f / minv[icol][icol];
            minv[icol][icol] = 1.f;
            for (int j = 0; j < 4; j++) {
                minv[icol][j] *= pivinv;
            }

            // Subtract this row from others to zero out their columns
            for (int j = 0; j < 4; j++) {
                if (j != icol) {
                    float save = minv[j][icol];
                    minv[j][icol] = 0;
                    for (int k = 0; k < 4; k++) {
                        minv[j][k] -= minv[icol][k]*save;
                    }
                }
            }
        }
        // Swap columns to reflect permutation
        for (int j = 3; j >= 0; j--) {
            if (indxr[j] != indxc[j]) {
                for (auto & k : minv){
                    temp = k[indxr[j]];
                    k[indxr[j]] = k[indxc[j]];
                    k[indxc[j]] = temp;
                }
            }
        }
        return Matrix4(minv);
    }

    // Method to set all the values in the matrix
    void setAllValues(float a1, float a2, float a3, float a4,
                      float b1, float b2, float b3, float b4,
                      float c1, float c2, float c3, float c4,
                      float d1, float d2, float d3, float d4) {
        m_data[0][0] = a1; m_data[0][1] = a2; m_data[0][2] = a3, m_data[0][3] = a4;
        m_data[1][0] = b1; m_data[1][1] = b2; m_data[1][2] = b3; m_data[1][3] = b4;
        m_data[2][0] = c1; m_data[2][1] = c2; m_data[2][2] = c3; m_data[2][3] = c4;
        m_data[3][0] = d1; m_data[3][1] = d2; m_data[3][2] = d3; m_data[3][3] = d4;
    }

    // Set the matrix to the identity matrix
    Matrix4<T> setToIdentity() {
        m_data[0][0] = 1; m_data[0][1] = 0; m_data[0][2] = 0;  m_data[0][3] = 0;
        m_data[1][0] = 0; m_data[1][1] = 1; m_data[1][2] = 0;  m_data[1][3] = 0;
        m_data[2][0] = 0; m_data[2][1] = 0; m_data[2][2] = 1;  m_data[2][3] = 0;
        m_data[3][0] = 0; m_data[3][1] = 0; m_data[3][2] = 0;  m_data[3][3] = 1;
        return *this;
    }

    // Display the matrix
    void print() const {
        for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++) {
                std::cout << m_data[i][j];
            }
            std::cout << std::endl;
        }
    }

    // Return the pointer to the data array of the matrix
    float* dataBlock() {
        return m_data[0];
    }

    // Return the constant pointer to the data array of the matrix
    const float* dataBlock() const {
        return m_data[0];
    }

    // Return a given value from the matrix
    T getValue(int i, int j) const {
        assert(i >= 0 && i<4 && j >= 0 && j<4);
        return m_data[i][j];
    }

    // Return the trace of the matrix
    T getTrace() const {
        // Compute and return the trace
        return (m_data[0][0] + m_data[1][1] + m_data[2][2] + m_data[3][3]);
    }

    // Return a 4x4 translation matrix
    Matrix4<T> translationMatrix(const Vector3<T>& v) {
        return Matrix4<T>(1, 0, 0, v.x,
                          0, 1, 0, v.y,
                          0, 0, 1, v.z,
                          0, 0, 0, 1);
    }

    // Return a 4x4 rotation matrix
    Matrix4<T> rotationMatrix(const Vector3<T>& axis, float angle) {

        T cosA = std::cos(angle);
        T sinA = std::sin(angle);
        Matrix4<T> rotationMatrix;
        rotationMatrix.setToIdentity();

        rotationMatrix.m_data[0][0] = cosA + (1-cosA) * axis.x * axis.x;
        rotationMatrix.m_data[0][1] = (1-cosA) * axis.x * axis.y - axis.z * sinA;
        rotationMatrix.m_data[0][2] = (1-cosA) * axis.x * axis.z + axis.y * sinA;
        rotationMatrix.m_data[0][3] = 0.0;

        rotationMatrix.m_data[1][0] = (1-cosA) * axis.x * axis.y + axis.z * sinA;
        rotationMatrix.m_data[1][1] = cosA + (1-cosA) * axis.y * axis.y;
        rotationMatrix.m_data[1][2] = (1-cosA) * axis.y * axis.z - axis.x * sinA;
        rotationMatrix.m_data[1][3] = 0.0;

        rotationMatrix.m_data[2][0] = (1-cosA) * axis.x * axis.z - axis.y * sinA;
        rotationMatrix.m_data[2][1] = (1-cosA) * axis.y * axis.z + axis.x * sinA;
        rotationMatrix.m_data[2][2] = cosA + (1-cosA) * axis.z * axis.z;
        rotationMatrix.m_data[2][3] = 0.0;

        rotationMatrix.m_data[3][0] = 0.0;
        rotationMatrix.m_data[3][1] = 0.0;
        rotationMatrix.m_data[3][2] = 0.0;
        rotationMatrix.m_data[3][3] = 1.0;

        return rotationMatrix;
    }

    // Return a 4x4 perspective projection matrix
    Matrix4<T> perspectiveProjectionMatrix(float nearDistance, float farDistance, int width, int height, float fieldOfView) {
        // Compute the aspect ratio
        T aspect = float(width) / float(height);

        T top = nearDistance * std::tan((fieldOfView / 2.0) * (M_PI / 180.0));
        T bottom = -top;
        T left = bottom * aspect;
        T right = top * aspect;

        T fx = 2.0f * nearDistance / (right - left);
        T fy = 2.0f * nearDistance / (top - bottom);
        T fz = -(farDistance + nearDistance) / (farDistance - nearDistance);
        T fw = -2.0f * farDistance * nearDistance / (farDistance - nearDistance);

        // Compute the projection matrix
        return Matrix4<T>(fx, 0, 0, 0,
                          0, fy, 0, 0,
                          0, 0, fz, fw,
                          0, 0, -1, 0);
    }

    // Return a 4x4 orthographic projection matrix
    Matrix4<T> orthoProjectionMatrix(float nearDistance, float farDistance, int width, int height) {
        // Compute the aspect ratio
        T aspect = float(width) / float(height);

        T top = height * 0.5;
        T bottom = -top;
        T left = bottom * aspect;
        T right = top * aspect;

        T fx = 2.0 / (right - left);
        T fy = 2.0 / (top - bottom);
        T fz = -2.0 / (farDistance - nearDistance);
        T fw = -(farDistance + nearDistance) / (farDistance - nearDistance);

        // Compute the projection matrix
        return Matrix4<T>(fx, 0, 0, 0,
                           0, fy, 0, 0,
                           0, 0, fz, fw,
                           0, 0, 0, 1);
    }

    // Return the identity matrix
    Matrix4<T> identity() {
        return Matrix4<T>(1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, 1, 0,
                          0, 0, 0, 1);
    }
};

#endif //GRS_SIMULATOR_MATRIX4_H
