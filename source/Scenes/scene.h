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
    explicit Scene(const std::string& executablePath, const sceneConfiguration& config, const systemRenderData& renderData);
    ~Scene();

    void createPhysicsWorld();

    void reshape(int width, int height);

    void updateTarget(std::vector<double> payloadPosition) const;

    void update(const systemRenderData& systemData);
    void render();
    void renderSinglePass(Shader& shader, const grs::Matrix4d& worldToCameraMatrix);
    void drawReferenceFrame(bool shouldDraw);
    void drawForcesVector(bool shouldDraw);
    void reset();
    void destroy();

    bool mouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY);
    bool mouseMotionEvent(double xMouse, double yMouse, int leftButtonState, int rightButtonState, int middleButtonState, int altKeyState);
    bool scrollingEvent(double xAxis, double yAxis, double scrollSensitivity);

    void setWindowDimension(int width, int height);
    void setViewport(int x, int y, int width, int height);
    void setInitialZoom(float zoom);

    [[nodiscard]] bool shouldDrawReferenceFrame() const {return m_shouldDrawReferenceFrame;};
    [[nodiscard]] bool shouldDrawForcesVector() const {return m_shouldDrawForcesVector;};

    void changeView(const grs::Matrix4d& viewMatrix);

    void setInterpolationFactor(double interpolationFactor);

protected:
    const GLsizei SHADOWMAP_WIDTH  = 2048;
    const GLsizei SHADOWMAP_HEIGHT = 2048;

    static constexpr int NB_SHADOW_MAPS = 1;

    sceneConfiguration m_sceneConfig;
    systemRenderData m_systemRenderData;

    std::string m_executablePath;

    std::vector<std::unique_ptr<PhysicsObject>> m_physicsObjects;
    std::vector<std::unique_ptr<PhysicsObject>> m_tethersObjects;
    std::vector<std::shared_ptr<PhysicsObject>> m_staticObjects;

    std::shared_ptr<Cube> m_groundObject;
    std::shared_ptr<Cube> m_targetObject;
    std::vector<std::shared_ptr<Launcher>> m_launchers;

    Shader m_depthShader;
    Shader m_phongShader;
    Shader m_colorShader;
    Shader m_quadShader;

    Light m_light0;
    Light m_light1;

    Camera m_shadowMapLightCameras[NB_SHADOW_MAPS];
    FrameBufferObject m_frameBufferObjectShadowMap[NB_SHADOW_MAPS];
    Texture2D m_shadowMapTexture[NB_SHADOW_MAPS];

    grs::Matrix4d m_shadowMapBiasMatrix{};

    Camera m_camera;

    grs::Vec3d m_centerOfScene{};

    bool m_shouldDrawReferenceFrame = false;
    bool m_shouldDrawForcesVector = false;

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
    void m_setScenePosition(const grs::Vec3d& position, float sceneRadius);
    void m_resetCameraToViewAll();

    void m_zoom(double zoomDiff);
    void m_translate(int xMouse, int yMouse);
    void m_rotate(int xMouse, int yMouse);
};

#endif //GRS_SIMULATOR_SCENE_H
