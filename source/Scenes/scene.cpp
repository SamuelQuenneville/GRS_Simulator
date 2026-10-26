/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "scene.h"

#include <algorithm>
#include <cmath>

namespace {

// Scene look
const grs::Vec3f kZenithColor      {0.30f, 0.50f, 0.78f};
const grs::Vec3f kHorizonColor     {0.80f, 0.86f, 0.92f};   // also the fog color
const grs::Vec3f kSunColor         {0.85f, 0.82f, 0.75f};
const grs::Vec3f kFillColor        {0.10f, 0.11f, 0.14f};
const grs::Vec3f kSkyAmbientColor  {0.42f, 0.47f, 0.55f};
const grs::Vec3f kGroundAmbient    {0.22f, 0.24f, 0.20f};
const grs::Vec3f kGroundColor      {0.33f, 0.45f, 0.27f};
const grs::Vec3f kGridColor        {0.80f, 0.85f, 0.78f};
const grs::Vec3f kAxisXColor       {0.78f, 0.35f, 0.35f};   // line z = 0
const grs::Vec3f kAxisZColor       {0.37f, 0.69f, 0.37f};   // line x = 0
constexpr float kFogStart = 250.0f;
constexpr float kFogEnd   = 950.0f;

// Softer than the pure primaries of Color::color(), which look harsh once lit
Color uavColor(const int id) {
    switch (id) {
        case 1:  return {0.86f, 0.28f, 0.24f, 1.0f};   // red
        case 2:  return {0.96f, 0.74f, 0.18f, 1.0f};   // amber
        case 3:  return {0.18f, 0.68f, 0.64f, 1.0f};   // teal
        default: return {0.85f, 0.85f, 0.85f, 1.0f};
    }
}
constexpr Color kPayloadColor  {0.22f, 0.40f, 0.82f, 1.0f};
constexpr Color kLauncherColor {0.55f, 0.57f, 0.60f, 1.0f};
constexpr Color kTargetColor   {0.95f, 0.25f, 0.25f, 0.45f};

grs::Vec3f toVec3f(const grs::Vec3d& v) {
    return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

} // namespace

Scene::Scene(const std::string& executablePath, const sceneConfiguration& config, const systemRenderData& renderData)
    : m_executablePath(executablePath)
    , m_depthShader(executablePath + R"(/source/Shaders/depth.vert)", executablePath + R"(/source/Shaders/depth.frag)")
    , m_phongShader(executablePath + R"(/source/Shaders/phong.vert)", executablePath + R"(/source/Shaders/phong.frag)")
    , m_colorShader(executablePath + R"(/source/Shaders/color.vert)", executablePath + R"(/source/Shaders/color.frag)")
    , m_quadShader(executablePath + R"(/source/Shaders/quad.vert)", executablePath + R"(/source/Shaders/quad.frag)")
    , m_skyboxShader(executablePath + R"(/source/Shaders/skybox.vert)", executablePath + R"(/source/Shaders/skybox.frag)")
    , m_groundShader(executablePath + R"(/source/Shaders/ground.vert)", executablePath + R"(/source/Shaders/ground.frag)")
    , m_skyShader(executablePath + R"(/source/Shaders/sky.vert)", executablePath + R"(/source/Shaders/sky.frag)")
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
    m_skyCross = executablePath + R"(/Assets/Sky/sky.png)";

    // Light positions are only used as directions (sun and fill)
    m_light0.translateWorld({100.0, 150.0, 80.0});
    m_light1.translateWorld({-80.0, 40.0, -100.0});

    m_shadowMapBiasMatrix = {0.5, 0.0, 0.0, 0.5,
                                0.0, 0.5, 0.0, 0.5,
                                0.0, 0.0, 0.5, 0.5,
                                0.0, 0.0, 0.0, 1.0};

    m_createShadowMapFrameBufferObjectAndTexture();

    // Empty VAO for the full-screen sky triangle (vertices generated in the shader)
    glGenVertexArrays(1, &m_skyVertexArray);

    m_createNoiseTexture();

    m_closeUp = std::make_unique<CloseUpView>(executablePath);

    // Set the center of the scene
    m_setScenePosition({0.0, 0.0, 0.0}, 100.0f);
    setInitialZoom(0.8f);
    m_resetCameraToViewAll();

    m_systemRenderData = renderData;
    m_sceneConfig = config;
}

Scene::~Scene() {
    for (int i = 0; i < NB_SHADOW_MAPS; i++) {
        m_shadowMapTexture[i].destroy();
        m_frameBufferObjectShadowMap[i].destroy();
    }

    glDeleteVertexArrays(1, &m_skyVertexArray);
    m_noiseTexture.destroy();

    m_depthShader.destroy();
    m_phongShader.destroy();
    m_groundShader.destroy();
    m_skyShader.destroy();
    m_quadShader.destroy();
    m_colorShader.destroy();
}

void Scene::createPhysicsWorld() {

    for (const auto& body : m_systemRenderData.bodies) {

        // ---------- AIRCRAFT ---------- //
        if (body.type == RenderObjectType::AIRCRAFT) {
            auto aircraftObject = std::make_unique<Aircraft>(body, grs::Vec3d(m_aircraftScale, m_aircraftScale, m_aircraftScale), m_executablePath);
            aircraftObject->setColor(uavColor(body.id));

            m_physicsObjects.push_back(std::move(aircraftObject));

            // ---------- LAUNCHER ---------- //
            if (m_sceneConfig.haveLauncher) {
                auto launcherObject = std::make_shared<Launcher>(grs::Vec3d(1.0,1.0,1.0), m_executablePath);
                launcherObject->setColor(kLauncherColor);

                launcherObject->setTransform(grs::TransformD({-0.849, 0.816, 30.0}, grs::eulerToQuat(M_PI + grs::degToRad(19.0), 0.0, M_PI_2).nedToYup()));

                const double yaw = grs::quatToEuler((body.transform.orientation).yUpToNed())[2];
                launcherObject->rotateAroundWorldPoint(grs::Vec3d(0,1,0), yaw, grs::Vec3d::zeros());

                m_staticObjects.push_back(std::move(launcherObject));
            }

            // -----------  PAYLOAD ---------- //
        } else if (body.type == RenderObjectType::PAYLOAD) {
            auto payloadObject = std::make_unique<PayloadObject>(body, grs::Vec3f(0.4,0.4,0.4), m_executablePath);
            payloadObject->setColor(kPayloadColor);

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
        m_targetObject->setColor(kTargetColor);
        m_targetObject->setTransform(grs::TransformD(grs::Vec3d::zeros(), grs::QuaternionD::identity()));

        m_staticObjects.push_back(m_targetObject);
    }

    // ---------- Skybox ---------- //
    m_sky = std::make_shared<Skybox>(m_skyCross);

    // ---------- GROUND ---------- //
    // Large enough to reach the far plane, where the fog hides its edge.
    // The grid is drawn procedurally by the ground shader.
    m_ground = std::make_shared<Ground>(1500.0f);

}

void Scene::reshape(const int width, const int height) {
    m_camera.setDimensions(width, height);
}

void Scene::updateTarget(std::vector<double> payloadPosition) const {
    if (!m_targetObject || payloadPosition.size() < 3) return;
    m_targetObject->setTransform(grs::TransformD({payloadPosition[0], payloadPosition[2],payloadPosition[1]}, grs::QuaternionD::identity()));
}

void Scene::update(const systemRenderData& systemData) {

    const size_t nBodies = std::min(systemData.bodies.size(), m_physicsObjects.size());
    for (size_t i = 0; i < nBodies; i++) {
        m_physicsObjects[i]->setBodyRenderData(systemData.bodies[i]);
    }

    for (const auto& physicsObject : m_physicsObjects) {
        // Update the transform used for the rendering
        physicsObject->updateTransform(m_interpolationFactor);
    }

    const size_t nTether = std::min(systemData.tethers.size(), m_tethersObjects.size());
    for (size_t i = 0; i < nTether; i++) {
        m_tethersObjects[i]->update(systemData.tethers[i]);
    }

    // Centroid and extent of the moving bodies
    if (!systemData.bodies.empty()) {
        grs::Vec3d center = grs::Vec3d::zeros();
        for (const auto& body : systemData.bodies) {
            center += body.transform.position;
        }
        center = center * (1.0 / static_cast<double>(systemData.bodies.size()));

        double maxDistance = 0.0;
        for (const auto& body : systemData.bodies) {
            maxDistance = std::max(maxDistance, (body.transform.position - center).norm());
        }

        // Quantized so the shadow map resolution does not change every frame
        m_shadowRadius = std::clamp(std::ceil((maxDistance + 8.0) / 10.0) * 10.0, 20.0, 300.0);

        if (m_followSystem) {
            m_camera.translateWorld(center - m_centerOfScene);
            m_centerOfScene = center;
        }
        m_systemCenter = center;
    }
}


void Scene::render() {
    const grs::Matrix4d worldToCameraMatrix = m_camera.getTransformMatrix().inverse();
    const grs::Matrix4d& projectionMatrix = m_camera.getProjectionMatrix();
    const grs::Matrix3d worldToCameraRotation = worldToCameraMatrix.upperLeft3x3();

    const grs::Vec3d sunDirWorld  = grs::normalize(m_light0.getOrigin());
    const grs::Vec3d fillDirWorld = grs::normalize(m_light1.getOrigin());

    // ---------- Shadow map (sun, orthographic, fitted around the system) ---------- //

    grs::Matrix4d worldToLightMatrix;
    grs::Matrix4d lightProjectionMatrix;
    m_computeShadowMatrices(sunDirWorld, worldToLightMatrix, lightProjectionMatrix);
    const grs::Matrix4d lightSpaceMatrix = m_shadowMapBiasMatrix * lightProjectionMatrix * worldToLightMatrix;

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    m_frameBufferObjectShadowMap[0].bind();
    glViewport(0, 0, SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
    glClear(GL_DEPTH_BUFFER_BIT);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    // Both faces so thin surfaces (wings, tethers) cast shadows, offset against acne
    glDisable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);

    m_depthShader.bind();
    m_depthShader.setMatrix4x4Uniform("projectionMatrix", lightProjectionMatrix);
    m_depthShader.unbind();

    m_renderShadowCasters(worldToLightMatrix);

    glDisable(GL_POLYGON_OFFSET_FILL);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    m_frameBufferObjectShadowMap[0].unbind();

    // ---------- Main pass ---------- //

    glViewport(m_viewportX, m_viewportY, m_viewportWidth, m_viewportHeight);
    glClearColor(kHorizonColor[0], kHorizonColor[1], kHorizonColor[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (shouldDrawSky()) {
        m_sky->render(m_skyboxShader, worldToCameraMatrix, projectionMatrix);
    }

    m_shadowMapTexture[0].bind();
    m_noiseTexture.bind();
    const auto shadowUnit = static_cast<GLint>(m_shadowMapTexture[0].getUnit());
    const grs::Vec2f shadowTexel{1.0f / static_cast<float>(SHADOWMAP_WIDTH), 1.0f / static_cast<float>(SHADOWMAP_HEIGHT)};

    // Lit meshes
    m_phongShader.bind();
    m_phongShader.setMatrix4x4Uniform("projectionMatrix", projectionMatrix);
    m_phongShader.setMatrix4x4Uniform("lightSpaceMatrix", lightSpaceMatrix);
    m_phongShader.setVector3Uniform("sunDirCameraSpace", toVec3f(grs::normalize(worldToCameraRotation * sunDirWorld)));
    m_phongShader.setVector3Uniform("fillDirCameraSpace", toVec3f(grs::normalize(worldToCameraRotation * fillDirWorld)));
    m_phongShader.setVector3Uniform("upCameraSpace", toVec3f(worldToCameraRotation * grs::Vec3d(0.0, 1.0, 0.0)));
    m_phongShader.setVector3Uniform("sunColor", kSunColor);
    m_phongShader.setVector3Uniform("fillColor", kFillColor);
    m_phongShader.setVector3Uniform("skyAmbientColor", kSkyAmbientColor);
    m_phongShader.setVector3Uniform("groundAmbientColor", kGroundAmbient);
    m_phongShader.setIntUniform("shadowMapSampler", shadowUnit);
    m_phongShader.setVector2Uniform("shadowMapTexelSize", shadowTexel);
    m_phongShader.setIntUniform("isShadowEnabled", 1);
    m_phongShader.setVector3Uniform("fogColor", kHorizonColor);
    m_phongShader.setFloatUniform("fogStart", kFogStart);
    m_phongShader.setFloatUniform("fogEnd", kFogEnd);
    m_phongShader.unbind();

    // Ground
    m_groundShader.bind();
    m_groundShader.setMatrix4x4Uniform("lightSpaceMatrix", lightSpaceMatrix);
    m_groundShader.setVector3Uniform("groundColor", kGroundColor);
    m_groundShader.setVector3Uniform("gridColor", kGridColor);
    m_groundShader.setVector3Uniform("axisXColor", kAxisXColor);
    m_groundShader.setVector3Uniform("axisZColor", kAxisZColor);
    m_groundShader.setFloatUniform("gridMinorSpacing", 10.0f);
    m_groundShader.setFloatUniform("gridMajorSpacing", 50.0f);
    m_groundShader.setFloatUniform("gridExtent", 1000.0f);
    m_groundShader.setVector3Uniform("sunDirWorld", toVec3f(sunDirWorld));
    m_groundShader.setVector3Uniform("sunColor", kSunColor);
    m_groundShader.setVector3Uniform("skyAmbientColor", kSkyAmbientColor);
    m_groundShader.setIntUniform("shadowMapSampler", shadowUnit);
    m_groundShader.setIntUniform("noiseSampler", static_cast<GLint>(m_noiseTexture.getUnit()));
    m_groundShader.setVector2Uniform("shadowMapTexelSize", shadowTexel);
    m_groundShader.setIntUniform("isShadowEnabled", 1);
    m_groundShader.setVector3Uniform("fogColor", kHorizonColor);
    m_groundShader.setFloatUniform("fogStart", kFogStart);
    m_groundShader.setFloatUniform("fogEnd", kFogEnd);
    m_groundShader.unbind();

    m_colorShader.bind();
    m_colorShader.setMatrix4x4Uniform("projectionMatrix", projectionMatrix);
    m_colorShader.unbind();

    m_ground->render(m_groundShader, worldToCameraMatrix, projectionMatrix);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    for (const auto& tethersObject : m_tethersObjects) {
        tethersObject->render(m_phongShader, worldToCameraMatrix);
    }

    for (const auto& physicsObject : m_physicsObjects) {
        physicsObject->render(m_phongShader, worldToCameraMatrix);
        physicsObject->renderVisualObject(m_colorShader, worldToCameraMatrix, shouldDrawReferenceFrame(), shouldDrawForcesVector());
    }

    // Opaque static objects, then the sky behind everything, then the translucent target
    for (const auto& staticObject : m_staticObjects) {
        if (staticObject != m_targetObject) {
            staticObject->render(m_phongShader, worldToCameraMatrix);
        }
    }

    if (!shouldDrawSky()) {
        m_renderSky(worldToCameraMatrix, projectionMatrix, sunDirWorld, 1.0f);
    }

    if (m_targetObject) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        m_targetObject->render(m_phongShader, worldToCameraMatrix);
        glDisable(GL_BLEND);
    }

    m_noiseTexture.unbind();
    m_shadowMapTexture[0].unbind();
}

void Scene::m_renderShadowCasters(const grs::Matrix4d& worldToLight) {
    for (const auto& tethersObject : m_tethersObjects) {
        tethersObject->render(m_depthShader, worldToLight);
    }

    for (const auto& physicsObject : m_physicsObjects) {
        physicsObject->render(m_depthShader, worldToLight);
    }

    // The translucent target marker does not cast a shadow
    for (const auto& staticObject : m_staticObjects) {
        if (staticObject != m_targetObject) {
            staticObject->render(m_depthShader, worldToLight);
        }
    }
}

void Scene::m_computeShadowMatrices(const grs::Vec3d& sunDirWorld, grs::Matrix4d& worldToLight, grs::Matrix4d& lightProjection) const {
    // Light view: rotation only, looking along the sun rays
    const grs::Vec3d forward = -sunDirWorld;
    grs::Vec3d up(0.0, 1.0, 0.0);
    if (std::abs(grs::dot(forward, up)) > 0.99) {
        up = grs::Vec3d(1.0, 0.0, 0.0);
    }
    const grs::Vec3d side = grs::normalize(grs::cross(forward, up));
    const grs::Vec3d upLight = grs::cross(side, forward);

    worldToLight = { side[0],     side[1],     side[2],    0.0,
                     upLight[0],  upLight[1],  upLight[2], 0.0,
                    -forward[0], -forward[1], -forward[2], 0.0,
                     0.0,         0.0,         0.0,        1.0};

    // Center of the system in light space, snapped to shadow map texels to avoid shimmering
    const double r = m_shadowRadius;
    const double texel = 2.0 * r / static_cast<double>(SHADOWMAP_WIDTH);
    grs::Vec3d c = worldToLight.upperLeft3x3() * m_systemCenter;
    c[0] = std::floor(c[0] / texel) * texel;
    c[1] = std::floor(c[1] / texel) * texel;

    const double left   = c[0] - r;
    const double right  = c[0] + r;
    const double bottom = c[1] - r;
    const double top    = c[1] + r;

    // Light looks down -z; keep receivers up to 250 m beyond the casters (ground below)
    const double nearPlane = -c[2] - r - 50.0;
    const double farPlane  = -c[2] + r + 250.0;

    lightProjection = {2.0 / (right - left), 0.0, 0.0, -(right + left) / (right - left),
                       0.0, 2.0 / (top - bottom), 0.0, -(top + bottom) / (top - bottom),
                       0.0, 0.0, -2.0 / (farPlane - nearPlane), -(farPlane + nearPlane) / (farPlane - nearPlane),
                       0.0, 0.0, 0.0, 1.0};
}

void Scene::m_renderSky(const grs::Matrix4d& worldToCameraMatrix, const grs::Matrix4d& projectionMatrix, const grs::Vec3d& sunDirWorld, const float brightness) {
    grs::Matrix4d viewRotation = worldToCameraMatrix;
    viewRotation(0, 3) = 0.0;
    viewRotation(1, 3) = 0.0;
    viewRotation(2, 3) = 0.0;
    const grs::Matrix4d inverseViewProjection = (projectionMatrix * viewRotation).inverse();

    // Depth is 1.0 in the shader: only pixels still at the cleared depth pass
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);

    m_skyShader.bind();
    m_skyShader.setMatrix4x4Uniform("inverseViewProjection", inverseViewProjection);
    m_skyShader.setVector3Uniform("zenithColor", kZenithColor);
    m_skyShader.setVector3Uniform("horizonColor", kHorizonColor);
    m_skyShader.setVector3Uniform("sunDirWorld", toVec3f(sunDirWorld));
    m_skyShader.setVector3Uniform("sunColor", kSunColor);
    m_skyShader.setFloatUniform("brightness", brightness);

    glBindVertexArray(m_skyVertexArray);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);

    m_skyShader.unbind();

    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
}

void Scene::m_createNoiseTexture() {
    // Smooth tileable noise: random lattice blurred with a wrapping box filter
    constexpr int size = 256;
    std::vector<float> values(size * size);
    uint32_t state = 12345u;
    for (auto& v : values) {
        state = state * 1664525u + 1013904223u;
        v = static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    }

    std::vector<float> tmp(values.size());
    constexpr int radius = 4;
    for (int pass = 0; pass < 2; ++pass) {
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                float sum = 0.0f;
                for (int k = -radius; k <= radius; ++k) sum += values[y * size + (x + k + size) % size];
                tmp[y * size + x] = sum / (2 * radius + 1);
            }
        }
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                float sum = 0.0f;
                for (int k = -radius; k <= radius; ++k) sum += tmp[((y + k + size) % size) * size + x];
                values[y * size + x] = sum / (2 * radius + 1);
            }
        }
    }

    const auto [minIt, maxIt] = std::minmax_element(values.begin(), values.end());
    const float lo = *minIt;
    const float range = std::max(*maxIt - lo, 1e-6f);
    std::vector<uint8_t> pixels(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        pixels[i] = static_cast<uint8_t>(std::lround(255.0f * (values[i] - lo) / range));
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    m_noiseTexture.create(size, size, GL_R8, GL_RED, GL_UNSIGNED_BYTE, GL_LINEAR_MIPMAP_NEAREST, GL_LINEAR, GL_REPEAT, GL_REPEAT, pixels.data());
    glBindTexture(GL_TEXTURE_2D, m_noiseTexture.getID());
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    m_noiseTexture.setUnit(2);
}

void Scene::m_setDefaultView() {
    // Elevated three-quarter view
    constexpr double pitch = -25.0 * M_PI / 180.0;
    constexpr double yaw   =  35.0 * M_PI / 180.0;
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw),   sy = std::sin(yaw);

    const grs::Matrix4d view{ cy, sy * sp, sy * cp, 0.0,
                             0.0,      cp,     -sp, 0.0,
                             -sy, cy * sp, cy * cp, 0.0,
                             0.0,     0.0,     0.0, 1.0};
    changeView(view);
    m_currentCameraVerticalAngle = pitch;
}

void Scene::drawReferenceFrame(const bool shouldDraw) {
    m_shouldDrawReferenceFrame = shouldDraw;
}

void Scene::drawForcesVector(const bool shouldDraw) {
    m_shouldDrawForcesVector = shouldDraw;
}

void Scene::drawSky(const bool shouldDraw) {
    m_shouldDrawSky = shouldDraw;
}

void Scene::reset() {
    m_setDefaultView();
    destroy();

    createPhysicsWorld();
}

void Scene::setCloseUpEnabled(const bool enabled) {
    m_closeUpEnabled = enabled;
    if (!enabled) {
        m_closeUp->hide();
    }
}

void Scene::renderCloseUp(const int framebufferWidth, const int framebufferHeight, const float pixelRatio) {
    if (!m_closeUpEnabled || m_physicsObjects.empty()) {
        m_closeUp->hide();
        return;
    }

    const bool validBody = m_closeUpBody >= 0 && m_closeUpBody < static_cast<int>(m_physicsObjects.size());
    const int body = validBody ? m_closeUpBody : 0;
    const int frame = validBody ? m_closeUpFrame : -1;

    const grs::Vec3d sunDirWorld = grs::normalize(m_light0.getOrigin());
    m_closeUp->render(*m_physicsObjects[body], frame, framebufferWidth, framebufferHeight, pixelRatio, m_colorShader, m_depthShader,
                      [this, sunDirWorld](const grs::Matrix4d& view, const grs::Matrix4d& projection, const float brightness) {
                          m_renderSky(view, projection, sunDirWorld, brightness);
                      });
}

void Scene::drawCloseUpOverlay(NVGcontext* vg) const {
    if (m_closeUpEnabled) {
        m_closeUp->drawOverlay(vg);
    }
}

void Scene::setAircraftDisplayScale(const double scale) {
    m_aircraftScale = scale;
    for (const auto& object : m_physicsObjects) {
        if (auto* aircraft = dynamic_cast<Aircraft*>(object.get())) {
            aircraft->setDisplayScale(scale);
        }
    }
}

void Scene::destroy() {
    // Cached close-up edges refer to the objects about to be destroyed
    m_closeUp->forgetObjects();
    m_closeUp->hide();

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

        // Unit 1: unit 0 is used by the skybox cube map
        m_shadowMapTexture[i].setUnit(i + 1);

        // Make sure that texture lookups outside the texture coords range will not treat as being in shadow
        glBindTexture(GL_TEXTURE_2D, m_shadowMapTexture[i].getID());
        constexpr GLfloat border[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

        // Hardware depth comparison (sampler2DShadow): each lookup returns a filtered 0..1 visibility
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
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
