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
#include "RigidBodyAndLink/tether.h"


class TetherLine final : public PhysicsObject {

public:
    explicit TetherLine(Tether* tether);
    ~TetherLine() override = default;

    void updatePosition() override;

    void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

private:
    Vector3<float> m_point1;
    Vector3<float> m_point2;

    std::vector<Vector3<float>> m_linePoints;

    VertexBufferObject m_vertexBufferVertices;
    VertexArrayObject m_vertexArrayObject;

    void m_createVertexBufferAndArrayObject();
};

#endif //GRS_SIMULATOR_TETHERLINE_H
