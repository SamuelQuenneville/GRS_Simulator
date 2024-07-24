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
#include "OpenglFramework/shader.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"


class TetherLine : public PhysicsObject {

public:
    TetherLine();
    ~TetherLine() override = default;

    void setPosition(const Vector3<float>& point1, const Vector3<float>& point2);

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
