/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "scene.h"

Scene::Scene(const std::string& executablePath)
    : m_elapsedTime(std::chrono::seconds::zero())
    , m_depthShader(executablePath + R"(/source/Shaders/depth.vert)", executablePath + R"(/source/Shaders/depth.frag)")
    , m_phongShader(executablePath + R"(/source/Shaders/phong.vert)", executablePath + R"(/source/Shaders/phong.frag)")
    , m_colorShader(executablePath + R"(/source/Shaders/color.vert)", executablePath + R"(/source/Shaders/color.frag)")
    , m_quadShader(executablePath + R"(/source/Shaders/quad.vert)", executablePath + R"(/source/Shaders/quad.frag)")
    , m_lastMouseX(0)
    , m_lastMouseY(0)
    , m_windowWidth(0)
    , m_windowHeight(0)
    , m_interpolationFactor(0.0)
    , m_currentCameraVerticalAngle(0.0)
    , m_viewportX(0)
    , m_viewportY(0)
    , m_viewportWidth(0)
    , m_viewportHeight(0)
    , m_initialZoom(2.0f)
{
    m_light0.translateWorld({30.0f, 80.0f, 30.0f});
    m_light1.translateWorld({30.0f, 80.0f, 30.0f});

    constexpr float lightIntensity0 = 0.5f;
    constexpr float lightIntensity1 = 0.2f;
    const Color lightColor0(lightIntensity0 * 1.0f, lightIntensity0 * 1.0f, lightIntensity0 * 1.0f, 1.0f);
    const Color lightColor1(lightIntensity1 * 1.0f, lightIntensity1 * 1.0f, lightIntensity1 * 1.0f, 1.0f);

    m_light0.setDiffuseColor(lightColor0);
    m_light1.setDiffuseColor(lightColor1);

    m_shadowMapLightCameras[0].translateWorld(m_light0.getOrigin());
    m_shadowMapLightCameras[0].rotateLocal({1, 0, 0}, -M_PI / 4.0f);
    m_shadowMapLightCameras[0].rotateLocal({0, 1, 0}, M_PI / 4.0f);

    for (auto & m_shadowMapLightCamera : m_shadowMapLightCameras) {
        m_shadowMapLightCamera.setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
        m_shadowMapLightCamera.setFieldOfView(100.0f);
        m_shadowMapLightCamera.setSceneRadius(100.0f);
    }

    m_shadowMapBiasMatrix.setAllValues(0.5, 0.0, 0.0, 0.5,
                                      0.0, 0.5, 0.0, 0.5,
                                      0.0, 0.0, 0.5, 0.5,
                                      0.0, 0.0, 0.0, 1.0);

    m_createShadowMapFrameBufferObjectAndTexture();
}

Scene::~Scene() {
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {
        m_shadowMapTexture[i].destroy();
        m_frameBufferObjectShadowMap[i].destroy();
    }

    m_depthShader.destroy();
    m_phongShader.destroy();
    m_quadShader.destroy();
    m_colorShader.destroy();
}

void Scene::reshape(const int width, const int height) {
    m_camera.setDimensions(width, height);
}

void Scene::update() {
    for (const auto& physicsObject : m_physicsObjects) {
        // Update the transform used for the rendering
        physicsObject->updateTransform(m_interpolationFactor);
    }

    for (const auto& tetherObject : m_tethersObjects) {
        tetherObject->updatePosition();
    }
}

void Scene::render() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(Color::backgroundColor().r, Color::backgroundColor().g, Color::backgroundColor().b, Color::backgroundColor().a);

    Matrix4<double> shadowMapProjMatrix[NB_SHADOW_MAPS];
    Matrix4<double> worldToLightCameraMatrix[NB_SHADOW_MAPS];
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {

        shadowMapProjMatrix[i] = m_shadowMapLightCameras[i].getProjectionMatrix();
        worldToLightCameraMatrix[i] = m_shadowMapLightCameras[i].getTransformMatrix().getInverse();
    }

    // ---------- Render the scene to generate the shadow map (first pass) ----------- //

    // Culling switching, rendering only backface, this is done to avoid self-shadowing
    glCullFace(GL_BACK);

    // For each shadow map
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {

        m_frameBufferObjectShadowMap[i].bind();

        // Bind the shader
        m_depthShader.bind();

        // Set the variables of the shader
        m_depthShader.setMatrix4x4Uniform("projectionMatrix", shadowMapProjMatrix[i]);

        // Set the viewport to render into the shadow map texture
        glViewport(0, 0, SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);

        // Clear previous frame values
        glClear(GL_DEPTH_BUFFER_BIT);

        // Disable color rendering, we only want to write to the Z-Buffer
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

        // Render the objects of the scene
        renderSinglePass(m_depthShader, worldToLightCameraMatrix[i]);

        // Unbind the shader
        m_depthShader.unbind();

        m_frameBufferObjectShadowMap[i].unbind();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);

    // ---------- Render the scene for final rendering (second pass) ----------- //

    glCullFace(GL_BACK);

    // Get the world-space to camera-space matrix
    const Matrix4<double> worldToCameraMatrix = m_camera.getTransformMatrix().getInverse();

    m_phongShader.bind();

    // Is shadow mapping is enabled
    GLint textureUnits[NB_SHADOW_MAPS];
    for (const auto& i : m_shadowMapTexture) {
        i.bind();
    }
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {
        textureUnits[i] = static_cast<GLint>(m_shadowMapTexture[i].getUnit());
    }

    // Set the variables of the phong shader
    m_phongShader.setMatrix4x4Uniform("projectionMatrix", m_camera.getProjectionMatrix());
    m_phongShader.setMatrix4x4Uniform("shadowMapLight0ProjectionMatrix", m_shadowMapBiasMatrix * shadowMapProjMatrix[0]);
    m_phongShader.setMatrix4x4Uniform("worldToLight0CameraMatrix", worldToLightCameraMatrix[0]);
    m_phongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * m_light0.getOrigin());
    m_phongShader.setVector3Uniform("light1PosCameraSpace", worldToCameraMatrix * m_light1.getOrigin());
    m_phongShader.setVector3Uniform("lightAmbientColor", {0.1f, 0.1f, 0.1f});
    m_phongShader.setVector3Uniform("light0DiffuseColor", {m_light0.getDiffuseColor().r, m_light0.getDiffuseColor().g, m_light0.getDiffuseColor().b});
    m_phongShader.setVector3Uniform("light1DiffuseColor", {m_light1.getDiffuseColor().r, m_light1.getDiffuseColor().g, m_light1.getDiffuseColor().b});
    m_phongShader.setIntUniform("shadowMapSampler0", textureUnits[0]);
    m_phongShader.setIntUniform("isShadowEnabled", true);
    m_phongShader.setVector2Uniform("shadowMapDimension", {static_cast<double>(SHADOWMAP_WIDTH), static_cast<double>(SHADOWMAP_HEIGHT)});
    m_phongShader.unbind();

    // Set the variables of the color shader
    m_colorShader.bind();
    m_colorShader.setMatrix4x4Uniform("projectionMatrix", m_camera.getProjectionMatrix());
    m_colorShader.unbind();

    // Set the viewport to render the scene
    glViewport(m_viewportX, m_viewportY, m_viewportWidth, m_viewportHeight);

    //Enabling color write (previously disabled for light POV z-buffer rendering)
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the objects of the scene
    renderSinglePass(m_phongShader, worldToCameraMatrix);

    // Is shadow mapping is enabled
    for (const auto & texture : m_shadowMapTexture) {
        texture.unbind();
    }
}

void Scene::renderSinglePass(Shader& shader, const Matrix4<double>& worldToCameraMatrix) {
    // Bind the shader
    shader.bind();

    // Render all the moving physics objects of the scene
    for (const auto& physicsObject : m_physicsObjects) {
        physicsObject->render(shader, worldToCameraMatrix);
    }

    // Render the static objects (floor could be obstacle...)
    for (const auto& staticObject : m_staticObjects) {
        staticObject->render(shader, worldToCameraMatrix);
    }

    // Unbind the shader
    shader.unbind();
}

void Scene::drawReferenceFrame(const bool shouldDraw) {
    m_shouldDrawReferenceFrame = shouldDraw;
}

void Scene::reset() {
    m_elapsedTime = std::chrono::seconds::zero();
    changeView(Matrix4<double>::identity());
}

void Scene::destroy() {
    m_elapsedTime = std::chrono::seconds ::zero();
}


bool Scene::mouseButtonEvent(int button, const bool down, int mods, const double mousePosX, const double mousePosY) {
    if (down) {
        m_lastMouseX = mousePosX;
        m_lastMouseY = mousePosY;
    }
    return true;
}

bool Scene::mouseMotionEvent(const double xMouse, const double yMouse, const int leftButtonState, const int rightButtonState, const int middleButtonState, const int altKeyState) {
    // Zoom
    if (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS) {

        // Zoom the camera
        m_zoom(-static_cast<float>(yMouse - m_lastMouseY) / static_cast<float>(m_windowHeight));
    }
        // Translation
    else if (middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS) {
        m_translate(static_cast<int>(xMouse), static_cast<int>(yMouse));
    }
        // Rotation
    else if (leftButtonState == GLFW_PRESS) {
        m_rotate(static_cast<int>(xMouse), static_cast<int>(yMouse));
    }

    // Remember the mouse position
    m_lastMouseX = xMouse;
    m_lastMouseY = yMouse;

    return true;
}

bool Scene::scrollingEvent(const double xAxis,const double yAxis, const double scrollSensitivity) {
    m_zoom(yAxis * scrollSensitivity);
    return true;
}

void Scene::setWindowDimension(const int width, const int height) {
    m_windowWidth = width;
    m_windowHeight = height;
}

void Scene::setViewport(const int x, const int y, const int width, const int height) {
    m_viewportX = x;
    m_viewportY = y;
    m_viewportWidth = width;
    m_viewportHeight = height;
}

void Scene::setInitialZoom(const float zoom) {
    m_initialZoom = zoom;
}

void Scene::changeView(const Matrix4<double>& viewMatrix) {
    m_camera.setTransformMatrix(viewMatrix);
    m_setScenePosition(Vector3<double>::zeros(), 100.0f);
}

void Scene::setInterpolationFactor(const double interpolationFactor) {
    m_interpolationFactor = interpolationFactor;
}

std::chrono::duration<double> Scene::getElapsedTime() const {
    return m_elapsedTime;
}

void Scene::setElapsedTime(const std::chrono::duration<double> elapsedTime) {
    m_elapsedTime = elapsedTime;
}

void Scene::m_createShadowMapFrameBufferObjectAndTexture() {
    // For each shadow map
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {

        // Create the texture for the depth values
        m_shadowMapTexture[i].create(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                                     GL_UNSIGNED_BYTE, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, nullptr);

        m_shadowMapTexture[i].setUnit(i);

        // Make sure that texture lookups outside the texture coords range will not treat as being in shadow
        glBindTexture(GL_TEXTURE_2D, m_shadowMapTexture[i].getID());
        constexpr GLfloat border[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
        glBindTexture(GL_TEXTURE_2D, 0);

        // Create the FBO for the shadow map
        m_frameBufferObjectShadowMap[i].create(0, 0, false);
        m_frameBufferObjectShadowMap[i].bind();

        // Tell OpenGL that we won't bind a color texture with the currently binded FBO
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);

        m_frameBufferObjectShadowMap[i].attachTexture(GL_DEPTH_ATTACHMENT, m_shadowMapTexture[i].getID());
        m_frameBufferObjectShadowMap[i].unbind();
    }
}

void Scene::m_setScenePosition(const Vector3<double>& position, const float sceneRadius) {
    // Set the position and radius of the scene
    m_centerOfScene = position;
    m_camera.setSceneRadius(sceneRadius);

    // Reset the camera position and zoom in order to view all the scene
    m_resetCameraToViewAll();
}

void Scene::m_resetCameraToViewAll() {
    // Move the camera to the origin of the scene
    m_camera.translateWorld(-m_camera.getOrigin());

    // Move the camera
    m_camera.translateWorld(m_centerOfScene);

    // Set the zoom of the camera so that the scene center is in negative view direction of the camera
    m_camera.setZoom(m_initialZoom);
}

void Scene::m_zoom(const double zoomDiff) {
    m_camera.setZoom(zoomDiff);
}

void Scene::m_translate(const int xMouse, const int yMouse) {
    const auto dx = static_cast<float>(xMouse - m_lastMouseX);
    const auto dy = static_cast<float>(yMouse - m_lastMouseY);

    // Translate the camera
    m_camera.translateCamera(-dx / static_cast<float>(m_camera.getWidth()),-dy / static_cast<float>(m_camera.getHeight()), m_centerOfScene);
}

void Scene::m_rotate(const int xMouse, const int yMouse) {
    float deltaHorizRotationAngle = static_cast<float>(m_lastMouseX - xMouse) / static_cast<float>(m_windowWidth) * M_PI;
    float deltaVertRotationAngle  = static_cast<float>(m_lastMouseY - yMouse) / static_cast<float>(m_windowHeight) * M_PI;

    const double newVerticalAngle = m_currentCameraVerticalAngle + deltaVertRotationAngle;

    // Limit Vertical angle
    if (newVerticalAngle > (M_PI * 0.5) || newVerticalAngle < -(M_PI * 0.5)) {
        deltaVertRotationAngle = 0.0;
    }

    const Vector3 localVertAxis = m_camera.getTransformMatrix().getUpperLeft3x3Matrix().getInverse() * Vector3(0.0, 1.0, 0.0);
    m_camera.rotateAroundLocalPoint(Vector3(1.0, 0.0, 0.0), deltaVertRotationAngle, m_centerOfScene);
    m_camera.rotateAroundLocalPoint(localVertAxis, deltaHorizRotationAngle, m_centerOfScene);

    m_currentCameraVerticalAngle += deltaVertRotationAngle;
}
