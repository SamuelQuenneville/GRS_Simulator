/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_OBJECT3D_H
#define GRS_SIMULATOR_OBJECT3D_H

#include "Mathematics/vector3.h"
#include "Mathematics/matrix4.h"

// This class represent a generic 3D object on the scene.
class Object3D {

public:
    Object3D();
    virtual ~Object3D() = default;

    // Return the transform matrix
    [[nodiscard]] const Matrix4<double>& getTransformMatrix() const;

    // Set the transform matrix
    void setTransformMatrix(const Matrix4<double>& matrix);
    virtual void updateTransform(double interpolationFactor) {};

    // Set to the identity transform
    void setToIdentity();

    // Return the origin of object in world-space
    [[nodiscard]] Vector3<double> getOrigin() const;

    // Translate the object in world-space
    void translateWorld(const Vector3<double>& vector);

    // Translate the object in local-space
    void translateLocal(const Vector3<double>& vector);

    // Rotate the object in world-space
    void rotateWorld(const Vector3<double>& axis, float angle);

    // Rotate the object in local-space
    void rotateLocal(const Vector3<double>& axis, float angle);

    // Rotate around a world-space point
    void rotateAroundWorldPoint(const Vector3<double>& axis, float angle, const Vector3<double>& point);

    // Rotate around a local-space point
    void rotateAroundLocalPoint(const Vector3<double>& axis, float angle, const Vector3<double>& worldPoint);

protected:
    Matrix4<double> m_transformMatrix = Matrix4<double>::identity();
};


#endif //GRS_SIMULATOR_OBJECT3D_H
