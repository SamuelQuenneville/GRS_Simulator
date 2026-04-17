/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TETHERLINE_H
#define GRS_SIMULATOR_TETHERLINE_H

#include "physicsObject.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"

class TetherLine final : public PhysicsObject {

public:
    explicit TetherLine(const tetherRenderData& tetherRenderData);
    ~TetherLine() override = default;

    void updatePosition() override;

    void render(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) override;

private:
    grs::Vec3f m_point1{};
    grs::Vec3f m_point2{};

    std::vector<grs::Vec3f> m_linePoints;

    VertexBufferObject m_vertexBufferVertices;
    VertexArrayObject m_vertexArrayObject;

    void m_createVertexBufferAndArrayObject();
};

#endif //GRS_SIMULATOR_TETHERLINE_H
