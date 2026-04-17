/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "scene.h"

Scene::Scene(const std::string& executablePath, const sceneConfiguration& config, const systemRenderData& renderData)
    : m_executablePath(executablePath)
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
    m_light0.translateWorld({30.0, 80.0, 30.0});
    m_light1.translateWorld({30.0, 80.0, 30.0});

    constexpr float lightIntensity0 = 0.5f;
    constexpr float lightIntensity1 = 0.2f;
    constexpr Color lightColor0(lightIntensity0 * 1.0f, lightIntensity0 * 1.0f, lightIntensity0 * 1.0f, 1.0f);
    constexpr Color lightColor1(lightIntensity1 * 1.0f, lightIntensity1 * 1.0f, lightIntensity1 * 1.0f, 1.0f);

    m_light0.setDiffuseColor(lightColor0);
    m_light1.setDiffuseColor(lightColor1);

    m_shadowMapLightCameras[0].translateWorld(m_light0.getOrigin());
    m_shadowMapLightCameras[0].rotateLocal({1, 0, 0}, -M_PI_4);
    m_shadowMapLightCameras[0].rotateLocal({0, 1, 0}, M_PI_4);

    for (auto & m_shadowMapLightCamera : m_shadowMapLightCameras) {
        m_shadowMapLightCamera.setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
        m_shadowMapLightCamera.setFieldOfView(100.0f);
        m_shadowMapLightCamera.setSceneRadius(100.0f);
    }

    m_shadowMapBiasMatrix = {0.5, 0.0, 0.0, 0.5,
                                0.0, 0.5, 0.0, 0.5,
                                0.0, 0.0, 0.5, 0.5,
                                0.0, 0.0, 0.0, 1.0};

    m_createShadowMapFrameBufferObjectAndTexture();

    // Set the center of the scene
    m_setScenePosition({0.0, 0.0, 0.0}, 100.0f);
    setInitialZoom(1.5f);
    m_resetCameraToViewAll();

    m_systemRenderData = renderData;
    m_sceneConfig = config;
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

void Scene::createPhysicsWorld() {

    for (const auto& body : m_systemRenderData.bodies) {

        // ---------- AIRCRAFT ---------- //
        if (body.type == RenderObjectType::AIRCRAFT) {
            auto aircraftObject = std::make_unique<Aircraft>(body, grs::Vec3d(2.0, 2.0, 2.0), m_executablePath);
            aircraftObject->setColor(Color::color(body.id));

            m_physicsObjects.push_back(std::move(aircraftObject));

            // ---------- LAUNCHER ---------- //
            if (m_sceneConfig.haveLauncher) {
                auto launcherObject = std::make_shared<Launcher>(grs::Vec3d(1.0,1.0,1.0), m_executablePath);
                launcherObject->setColor(Color::backgroundColor());

                launcherObject->setTransform(grs::TransformD({-0.849, 0.816, 30.0}, grs::eulerToQuat(M_PI + grs::degToRad(19.0), 0.0, M_PI_2).nedToYup()));

                const double yaw = grs::quatToEuler((body.transform.orientation).yUpToNed())[2];
                launcherObject->rotateAroundWorldPoint({0,1,0}, yaw, {0,0,0});

                m_staticObjects.push_back(std::move(launcherObject));
            }

            // -----------  PAYLOAD ---------- //
        } else if (body.type == RenderObjectType::PAYLOAD) {
            auto payloadObject = std::make_unique<PayloadObject>(body, grs::Vec3f(0.4,0.4,0.4), m_executablePath);
            payloadObject->setColor(Color::blue());

            m_physicsObjects.push_back(std::move(payloadObject));
        }
    }

    // ------------ TETHER ----------- //
    for (const auto& tether : m_systemRenderData.tethers) {
        auto tetherObject = std::make_unique<TetherLine>(tether);
        m_tethersObjects.push_back(std::move(tetherObject));
    }

    // ------------ PAYLOAD TARGET ----------- //
    if (m_sceneConfig.haveTarget) {
        m_targetObject = std::make_shared<Cube>(grs::Vec3f(0.4, 0.4, 0.4), m_executablePath);
        m_targetObject->setColor({1.0f, 0.0f, 0.0f, 0.6f});
        m_targetObject->setTransform(grs::TransformD({0.0,0.0,0.0}, grs::QuaternionD::identity()));

        m_staticObjects.push_back(m_targetObject);
    }

    // ---------- GROUND ---------- //
    m_groundObject = std::make_shared<Cube>(grs::Vec3f(50.0, 0.5, 50.0), m_executablePath);
    m_groundObject->setColor(Color::floorColor());
    m_groundObject->setTransform(grs::TransformD({0.0, -0.5, 0.0}, grs::QuaternionD::identity()));

    m_staticObjects.push_back(m_groundObject);
}

void Scene::reshape(const int width, const int height) {
    m_camera.setDimensions(width, height);
}

void Scene::updateTarget(std::vector<double> payloadPosition) const {
    m_targetObject->setTransform(grs::TransformD({payloadPosition[0], payloadPosition[2],payloadPosition[1]}, grs::QuaternionD::identity()));
}

void Scene::update(const systemRenderData& systemData) {

    const size_t nBodies = std::min(systemData.bodies.size(), m_physicsObjects.size());
    for (size_t i = 0; i < nBodies; i++) {
        const auto& physicsObject = m_physicsObjects[i];
        const auto& body = systemData.bodies[i];

        physicsObject->setBodyRenderData(body);
    }

    for (const auto& physicsObject : m_physicsObjects) {
        // Update the transform used for the rendering
        physicsObject->updateTransform(m_interpolationFactor);
    }

    const size_t nTether = std::min(systemData.tethers.size(), m_tethersObjects.size());
    for (size_t i = 0; i < nTether; i++) {
        const auto& tetherObject = m_tethersObjects[i];
        const auto& tether = systemData.tethers[i];

        tetherObject->setTetherRenderData(tether);
    }

    for (const auto& tetherObject : m_tethersObjects) {
        tetherObject->updatePosition();
    }
}

void Scene::render() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(Color::backgroundColor().r, Color::backgroundColor().g, Color::backgroundColor().b, Color::backgroundColor().a);

    grs::Matrix4d shadowMapProjMatrix[NB_SHADOW_MAPS];
    grs::Matrix4d worldToLightCameraMatrix[NB_SHADOW_MAPS];
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {

        shadowMapProjMatrix[i] = m_shadowMapLightCameras[i].getProjectionMatrix();
        worldToLightCameraMatrix[i] = m_shadowMapLightCameras[i].getTransformMatrix().inverse();
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
    const grs::Matrix4d worldToCameraMatrix = m_camera.getTransformMatrix().inverse();

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
    m_phongShader.setVector3Uniform("light0PosCameraSpace", grs::cast<float>(worldToCameraMatrix * m_light0.getOrigin()));
    m_phongShader.setVector3Uniform("light1PosCameraSpace", grs::cast<float>(worldToCameraMatrix * m_light1.getOrigin()));
    m_phongShader.setVector3Uniform("lightAmbientColor", {0.1f, 0.1f, 0.1f});
    m_phongShader.setVector3Uniform("light0DiffuseColor", {m_light0.getDiffuseColor().r, m_light0.getDiffuseColor().g, m_light0.getDiffuseColor().b});
    m_phongShader.setVector3Uniform("light1DiffuseColor", {m_light1.getDiffuseColor().r, m_light1.getDiffuseColor().g, m_light1.getDiffuseColor().b});
    m_phongShader.setIntUniform("shadowMapSampler0", textureUnits[0]);
    m_phongShader.setIntUniform("isShadowEnabled", true);
    m_phongShader.setVector2Uniform("shadowMapDimension", {static_cast<float>(SHADOWMAP_WIDTH), static_cast<float>(SHADOWMAP_HEIGHT)});
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

void Scene::renderSinglePass(Shader& shader, const grs::Matrix4d& worldToCameraMatrix) {

    for (const auto& tethersObject : m_tethersObjects) {
        tethersObject->render(m_colorShader, worldToCameraMatrix);
    }

    // Bind the shader
    shader.bind();

    // Render all the moving physics objects of the scene
    for (const auto& physicsObject : m_physicsObjects) {
        physicsObject->render(shader, worldToCameraMatrix);
        physicsObject->renderVisualObject(m_colorShader, worldToCameraMatrix, shouldDrawReferenceFrame(), shouldDrawForcesVector());
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

void Scene::drawForcesVector(const bool shouldDraw) {
    m_shouldDrawForcesVector = shouldDraw;
}

void Scene::reset() {
    changeView(grs::Matrix4d::identity());
    destroy();

    createPhysicsWorld();
}

void Scene::destroy() {
    // Destroy all the physics objects of the scene
    m_physicsObjects.clear();
    m_tethersObjects.clear();
    m_staticObjects.clear();
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

void Scene::changeView(const grs::Matrix4d& viewMatrix) {
    m_camera.setTransformMatrix(viewMatrix);
    m_setScenePosition(grs::Vec3d::zeros(), 100.0f);
}

void Scene::setInterpolationFactor(const double interpolationFactor) {
    m_interpolationFactor = interpolationFactor;
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

void Scene::m_setScenePosition(const grs::Vec3d& position, const float sceneRadius) {
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
    const float deltaHorizRotationAngle = static_cast<float>(m_lastMouseX - xMouse) / static_cast<float>(m_windowWidth) * M_PIf;
    float deltaVertRotationAngle  = static_cast<float>(m_lastMouseY - yMouse) / static_cast<float>(m_windowHeight) * M_PIf;

    const double newVerticalAngle = m_currentCameraVerticalAngle + deltaVertRotationAngle;

    // Limit Vertical angle
    if (newVerticalAngle > (M_PI_2) || newVerticalAngle < -(M_PI_2)) {
        deltaVertRotationAngle = 0.0;
    }

    const grs::Vec3d localVertAxis = m_camera.getTransformMatrix().upperLeft3x3().inverse() * grs::Vec3d(0.0, 1.0, 0.0);
    m_camera.rotateAroundLocalPoint(grs::Vec3d(1.0, 0.0, 0.0), deltaVertRotationAngle, m_centerOfScene);
    m_camera.rotateAroundLocalPoint(localVertAxis, deltaHorizRotationAngle, m_centerOfScene);

    m_currentCameraVerticalAngle += deltaVertRotationAngle;
}
