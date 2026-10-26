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
#include "Definitions/shaderLayout.h"

class PhysicsObject : public Mesh {

public:
    PhysicsObject();
    explicit PhysicsObject(const std::string& meshFilePath);
    ~PhysicsObject() override;

    virtual void updatePosition() {};

    virtual void render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix);

    // Draw the mesh with an explicit local-to-world matrix (used by the close-up view)
    virtual void renderWithModel(Shader& shader, const grs::Matrix4d& localToWorldMatrix, const grs::Matrix4d& worldToCameraMatrix);

    // Mesh-to-body scaling at true size (the scene may draw some objects larger)
    [[nodiscard]] virtual grs::Matrix4d trueScaleMatrix() const {return grs::Matrix4d::identity();};

    [[nodiscard]] const bodyRenderData& getBodyRenderData() const {return m_bodyRenderData;};
    virtual void renderVisualObject(Shader& shader, const grs::Matrix4d& worldToCameraMatrix, bool drawFrames, bool drawForces);

    void createVertexBufferAndArrayObject();

    void setColor(const Color& color);

    [[nodiscard]] const grs::TransformD& getTransform() const;
    void setTransform(const grs::TransformD& transform);

    void setBodyRenderData(const bodyRenderData& renderData);

protected:
    bodyRenderData m_bodyRenderData{};

    // Vertex Buffer Object for the vertices data
    VertexBufferObject m_vertexBufferVertices;

    // Vertex Buffer Object for the normals data
    VertexBufferObject m_vertexBufferNormals;

    // Vertex Buffer Object for the texture coords
    VertexBufferObject m_vertexBufferTextureCoords;

    // Vertex Buffer Object for the indices
    VertexBufferObject m_vertexBufferIndices;

    // Vertex Array Object for the vertex data
    VertexArrayObject m_vertexArrayObject;

    std::vector<std::shared_ptr<FrameAxis>> m_visualObjects;

    grs::TransformD m_previousTransform;

    Color m_color;

    void m_createVisualObject();

    grs::Matrix4d m_calculateTransform(double interpolationFactor, const grs::Matrix4d& scalingMatrix);
};


#endif //GRS_SIMULATOR_PHYSICSOBJECT_H
