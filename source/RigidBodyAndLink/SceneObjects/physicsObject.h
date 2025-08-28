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
#include "RigidBodyAndLink/SceneObjects/frameAxis.h"
#include "RigidBodyAndLink/rigidBody.h"
#include "RigidBodyAndLink/link.h"

class PhysicsObject : public Mesh {

public:
    PhysicsObject();
    explicit PhysicsObject(const std::string& meshFilePath);

    virtual void updatePosition() {};

    virtual void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) = 0;
    virtual void renderVisualObject(Shader& shader, const Matrix4<double>& worldToCameraMatrix);

    void setColor(const Color& color);

    [[nodiscard]] const Transform& getTransform() const;
    void setTransform(const Transform& transform);

protected:
    RigidBody* m_body;
    Link* m_link;

    std::vector<std::shared_ptr<FrameAxis>> m_visualObjects;

    Transform m_previousTransform;

    Color m_color;

    void m_createVisualObject();

    Matrix4<double> m_calculateTransform(double interpolationFactor, const Matrix4<double>& scalingMatrix);
};


#endif //GRS_SIMULATOR_PHYSICSOBJECT_H
