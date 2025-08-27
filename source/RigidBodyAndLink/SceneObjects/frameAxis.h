/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FRAMEAXIS_H
#define GRS_SIMULATOR_FRAMEAXIS_H

#include "OpenglFramework/shader.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "RigidBodyAndLink/frame.h"
#include "OpenglFramework/object3D.h"
#include "Mathematics/color.h"


class FrameAxis final : public Object3D {

public:
    explicit FrameAxis(Frame* frame, float length = 2.0f);
    ~FrameAxis() override;

    void render(const Shader& shader, const Matrix4<double>& worldToCameraMatrix) const;
    void updateTransform(double interpolationFactor) override;

    struct FrameLine {
        // Constructor
        FrameLine(const Vector3<float>& point1, const Vector3<float>& point2, const uint32_t color)
            :point1(point1), color1(color), point2(point2), color2(color) {
        }

        // First point of the line
        Vector3<float> point1;
        // Color of the first point
        uint32_t color1;
        // Second point of the line
        Vector3<float> point2;
        // Color of the second point
        uint32_t color2;
    };

private:
    Frame* m_frame;
    Transform m_previousTransform;

    VertexBufferObject m_vertexBufferVertices;
    VertexArrayObject m_vertexArrayObject;

    float m_length;

    void m_createVertexBufferAndArrayObject();
};

#endif //GRS_SIMULATOR_FRAMEAXIS_H