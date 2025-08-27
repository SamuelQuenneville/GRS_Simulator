/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SCENE_H
#define GRS_SIMULATOR_SCENE_H


#include "RigidBodyAndLink/SceneObjects/physicsObject.h"
#include "RigidBodyAndLink/SceneObjects/cube.h"
#include "RigidBodyAndLink/SceneObjects/aircraft.h"
#include "RigidBodyAndLink/SceneObjects/launcher.h"
#include "RigidBodyAndLink/SceneObjects/tetherLine.h"
#include "RigidBodyAndLink/SceneObjects/payloadObject.h"
#include "OpenglFramework/light.h"
#include "OpenglFramework/camera.h"
#include "OpenglFramework/frameBufferObject.h"

class Scene {

public:
    explicit Scene(const std::string& executablePath);
    virtual ~Scene();

    virtual void reshape(int width, int height);

    // Update the physics world (take a simulation step) can be called several times per frame
    virtual void updatePhysics() = 0;
    virtual void updateTarget(float groundspeed,  std::vector<double> payloadPosition) = 0;

    virtual void update();
    virtual void render();
    virtual void renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix);
    virtual void drawReferenceFrame(bool shouldDraw);
    virtual void reset();
    virtual void destroy();

    virtual bool mouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY);
    virtual bool mouseMotionEvent(double xMouse, double yMouse, int leftButtonState, int rightButtonState, int middleButtonState, int altKeyState);
    virtual bool scrollingEvent(double xAxis, double yAxis, double scrollSensitivity);

    virtual void setPauseTimerCallback(std::function<void()> callback) = 0;

    void setWindowDimension(int width, int height);
    void setViewport(int x, int y, int width, int height);
    void setInitialZoom(float zoom);

    [[nodiscard]] bool shouldDrawReferenceFrame() const {return m_shouldDrawReferenceFrame;};

    void changeView(const Matrix4<double>& viewMatrix);

    void setInterpolationFactor(double interpolationFactor);

    [[nodiscard]] std::chrono::duration<double> getElapsedTime() const;
    void setElapsedTime(std::chrono::duration<double> elapsedTime);

protected:
    const GLsizei SHADOWMAP_WIDTH  = 2048;
    const GLsizei SHADOWMAP_HEIGHT = 2048;

    static constexpr int NB_SHADOW_MAPS = 1;

    std::chrono::duration<double> m_elapsedTime;   // Elapsed time (in seconds)

    std::vector<std::unique_ptr<PhysicsObject>> m_physicsObjects;
    std::vector<std::unique_ptr<PhysicsObject>> m_tethersObjects;
    std::vector<std::shared_ptr<PhysicsObject>> m_staticObjects;

    Shader m_depthShader;
    Shader m_phongShader;
    Shader m_colorShader;
    Shader m_quadShader;

    Light m_light0;
    Light m_light1;

    Camera m_shadowMapLightCameras[NB_SHADOW_MAPS];
    FrameBufferObject m_frameBufferObjectShadowMap[NB_SHADOW_MAPS];
    Texture2D m_shadowMapTexture[NB_SHADOW_MAPS];

    Matrix4<double> m_shadowMapBiasMatrix;

    Camera m_camera;

    Vector3<double> m_centerOfScene;

    bool m_shouldDrawReferenceFrame = false;

    // Last mouse coordinates on the windows
    double m_lastMouseX;
    double m_lastMouseY;

    int m_windowWidth;
    int m_windowHeight;

    // Interpolation factor for the bodies in the current frame
    double m_interpolationFactor;

    // Current camera vertical angle around the horizontal axis (in radians)
    double m_currentCameraVerticalAngle;

    // Viewport x,y, width and height values
    int m_viewportX;
    int m_viewportY;
    int m_viewportWidth;
    int m_viewportHeight;

    double m_initialZoom;

    void m_createShadowMapFrameBufferObjectAndTexture();
    void m_setScenePosition(const Vector3<double>& position, float sceneRadius);
    void m_resetCameraToViewAll();

    void m_zoom(double zoomDiff);
    void m_translate(int xMouse, int yMouse);
    void m_rotate(int xMouse, int yMouse);
};

#endif //GRS_SIMULATOR_SCENE_H
