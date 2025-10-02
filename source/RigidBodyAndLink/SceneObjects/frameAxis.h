/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FRAMEAXIS_H
#define GRS_SIMULATOR_FRAMEAXIS_H

#include "OpenglFramework/object3D.h"
#include "OpenglFramework/shader.h"
#include "OpenglFramework/vertexBufferObject.h"
#include "OpenglFramework/vertexArrayObject.h"
#include "RigidBodyAndLink/frame.h"


class FrameAxis final : public Object3D {

public:
    explicit FrameAxis(Frame* frame, float length = 2.0f);
    ~FrameAxis() override;

    void render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, bool drawFrames, bool drawForces) const;
    void updateTransform(double interpolationFactor) override;
    void updateForceVector() const;

    struct FrameLine {
        // Constructor
        FrameLine(const grs::Vec3f& point1, const grs::Vec3f& point2, const uint32_t color)
            :point1(point1), color1(color), point2(point2), color2(color) {
        }

        // First point of the line
        grs::Vec3f point1;
        // Color of the first point
        uint32_t color1;
        // Second point of the line
        grs::Vec3f point2;
        // Color of the second point
        uint32_t color2;
    };

private:
    Frame* m_frame;
    grs::TransformD m_previousTransform;

    VertexBufferObject m_vertexBufferVertices;
    VertexArrayObject m_vertexArrayObject;

    VertexBufferObject m_vertexBufferForce;
    VertexArrayObject m_vertexArrayObjectForce;

    float m_length;

    void m_createVertexBufferAndArrayObject();
    void m_createForceBufferAndArrayObject();
};

#endif //GRS_SIMULATOR_FRAMEAXIS_H