/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CAMERA_H
#define GRS_SIMULATOR_CAMERA_H

#include "object3D.h"

class Camera final : public Object3D {

public:
    Camera();
    ~Camera() override = default;

    [[nodiscard]] const Matrix4<double>& getProjectionMatrix() const;

    void setDimensions(uint32_t width, uint32_t height);

    // Set the radius of the scene the camera should capture
    // This will update the clipping planes accordingly
    void setSceneRadius(float radius);

    void setClippingPlanes(float nearDistance, float farDistance);

    void setFieldOfView(float fov);

    // Set the zoom of the camera (a fraction between 0 and 1)
    void setZoom(double fraction);

    void translateCamera(float dx, float dy, const Vector3<double>& worldPoint);

    [[nodiscard]] uint32_t getWidth() const;
    [[nodiscard]] uint32_t getHeight() const;

protected:
    float m_fieldOfView;

    float m_sceneRadius;

    float m_nearPlane;
    float m_farPlane;

    uint32_t m_width;
    uint32_t m_height;

    Matrix4<double> m_projectionMatrix;

    void m_updateProjectionMatrix();

};


#endif //GRS_SIMULATOR_CAMERA_H
