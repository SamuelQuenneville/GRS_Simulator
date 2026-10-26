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
#include "Definitions/renderObjectDefinitions.h"

#include <vector>


class FrameAxis final : public Object3D {

public:
    explicit FrameAxis(const frameRenderData& renderData, float length = 2.0f);
    ~FrameAxis() override;

    void render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix, bool drawFrames, bool drawForces) const;
    void updateTransform(double interpolationFactor) override;
    void updateForceVector() const;
    void setFrameRenderData(const frameRenderData& renderData);

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
    frameRenderData m_frameRenderData;

    grs::TransformD m_previousTransform;

    VertexBufferObject m_vertexBufferVertices;
    VertexArrayObject m_vertexArrayObject;

    VertexBufferObject m_vertexBufferForce;
    VertexArrayObject m_vertexArrayObjectForce;

    float m_length;

    // Arrow = thin prism shaft + cone head, all triangles
    static constexpr int kShaftSegments = 6;
    static constexpr int kConeSegments = 12;
    static constexpr int kArrowVertexCount = kShaftSegments * 6 + kConeSegments * 6;
    mutable bool m_hasForce = false;

    static void m_appendArrow(std::vector<Vertex>& vertices, const grs::Vec3f& tip, uint32_t color, float headLength, float headRadius, float shaftRadius);

    void m_createVertexBufferAndArrayObject();
    void m_createForceBufferAndArrayObject();
};

#endif //GRS_SIMULATOR_FRAMEAXIS_H