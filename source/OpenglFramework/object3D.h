/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_OBJECT3D_H
#define GRS_SIMULATOR_OBJECT3D_H

#include "Mathematics/math.h"

// This class represent a generic 3D object on the scene.
class Object3D {

public:
    Object3D();
    virtual ~Object3D() = default;

    // Return the transform matrix
    [[nodiscard]] const grs::Matrix4d& getTransformMatrix() const;

    // Set the transform matrix
    void setTransformMatrix(const grs::Matrix4d& matrix);
    virtual void updateTransform(double interpolationFactor) {};

    // Set to the identity transform
    void setToIdentity();

    // Return the origin of object in world-space
    [[nodiscard]] grs::Vec3d getOrigin() const;

    // Translate the object in world-space
    void translateWorld(const grs::Vec3d& vector);

    // Translate the object in local-space
    void translateLocal(const grs::Vec3d& vector);

    // Rotate the object in world-space
    void rotateWorld(const grs::Vec3d& axis, double angle);

    // Rotate the object in local-space
    void rotateLocal(const grs::Vec3d& axis, double angle);

    // Rotate around a world-space point
    void rotateAroundWorldPoint(const grs::Vec3d& axis, double angle, const grs::Vec3d& point);

    // Rotate around a local-space point
    void rotateAroundLocalPoint(const grs::Vec3d& axis, double angle, const grs::Vec3d& worldPoint);

protected:
    grs::Matrix4d m_transformMatrix = grs::Matrix4d::identity();
};


#endif //GRS_SIMULATOR_OBJECT3D_H
