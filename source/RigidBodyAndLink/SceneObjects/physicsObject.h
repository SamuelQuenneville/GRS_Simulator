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
#include "RigidBodyAndLink/link.h"
#include "Definitions/renderObjectDefinitions.h"

class PhysicsObject : public Mesh {

public:
    PhysicsObject();
    explicit PhysicsObject(const std::string& meshFilePath);

    virtual void updatePosition() {};

    virtual void render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) = 0;
    virtual void renderVisualObject(Shader& shader, const grs::Matrix4d& worldToCameraMatrix, bool drawFrames, bool drawForces);

    void setColor(const Color& color);

    [[nodiscard]] const grs::TransformD& getTransform() const;
    void setTransform(const grs::TransformD& transform);

    void setBodyRenderData(const bodyRenderData& renderData);
    void setTetherRenderData(const tetherRenderData& renderData);

protected:
    bodyRenderData m_bodyRenderData{};
    tetherRenderData m_tetherRenderData{};

    std::vector<std::shared_ptr<FrameAxis>> m_visualObjects;

    grs::TransformD m_previousTransform;

    Color m_color;

    void m_createVisualObject();

    grs::Matrix4d m_calculateTransform(double interpolationFactor, const grs::Matrix4d& scalingMatrix);
};


#endif //GRS_SIMULATOR_PHYSICSOBJECT_H
