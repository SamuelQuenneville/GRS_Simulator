/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MATH_H
#define GRS_SIMULATOR_MATH_H

#pragma once

#include "mathUtils.h"
#include "vector.h"
#include "matrix.h"
#include "quaternion.h"
#include "transform.h"

namespace grs {

// Vector
using Vec2i = Vector<int, 2>;
using Vec3i = Vector<int, 3>;
using Vec4i = Vector<int, 4>;

using Vec2f = Vector<float, 2>;
using Vec3f = Vector<float, 3>;
using Vec4f = Vector<float, 4>;

using Vec2d = Vector<double, 2>;
using Vec3d = Vector<double, 3>;
using Vec4d = Vector<double, 4>;

// Squared Matrix
using Matrix2i = Matrix<int, 2>;
using Matrix3i = Matrix<int, 3>;
using Matrix4i = Matrix<int, 4>;

using Matrix2f = Matrix<float, 2>;
using Matrix3f = Matrix<float, 3>;
using Matrix4f = Matrix<float, 4>;

using Matrix2d = Matrix<double, 2>;
using Matrix3d = Matrix<double, 3>;
using Matrix4d = Matrix<double, 4>;

// Quaternion
using QuaternionF = Quaternion<float>;
using QuaternionD = Quaternion<double>;

// Transform
using TransformF = Transform<float>;
using TransformD = Transform<double>;

}


#endif //GRS_SIMULATOR_MATH_H
