/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_PHYSICSOBJECT_H
#define GRS_SIMULATOR_PHYSICSOBJECT_H

#include "OpenglFramework/meshReader.h"
#include "OpenglFramework/shader.h"
#include "Mathematics/transform.h"
#include "Mathematics/color.h"
#include "RigidBodyAndLink/rigidBody.h"

class PhysicsObject : public Mesh {

public:
    PhysicsObject();
    explicit PhysicsObject(const std::string& meshFilePath);

    virtual void updateTransform(double interpolationFactor) = 0;

    virtual void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) = 0;

    void setColor(const Color& color);

    [[nodiscard]] const Transform& getTransform() const;
    void setTransform(const Transform& transform);

protected:
    RigidBody* m_body;

    Transform m_previousTransform;

    Color m_color;

    Matrix4<double> m_calculateTransform(double interpolationFactor, const Matrix4<double>& scalingMatrix);
};


#endif //GRS_SIMULATOR_PHYSICSOBJECT_H
