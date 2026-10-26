/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "simulationViewer.h"

#include <algorithm>
#include <chrono>
#include <thread>

SimulationViewer::SimulationViewer(programArguments& args)
    : m_programArgs(args)
    , m_isInitialized(false)
    , m_window(nullptr, glfwDestroyWindow)
    , m_gui(this)
    , m_scene(nullptr)
    , m_elapsedTime(0.0)
    , m_dynamicStepTime(0.0)
    , m_sceneCreated(false)
    , m_singlePhysicsStepEnabled(false)
    , m_singlePhysicsStepDone(false)

{
    m_world = std::make_unique<World>(m_programArgs);
}

SimulationViewer::~SimulationViewer() {
    m_destroyScenes();
}

void SimulationViewer::start() {
    glfwInit();
    glfwSetTime(0.0);

    // Get the primary monitor
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    // Window size
    m_windowWidth  = mode->width;
    m_windowHeight = mode->height;

    m_setGlfwHints();

    // Create a GLFW window object
    m_window.reset(glfwCreateWindow(m_windowWidth, m_windowHeight, "CREATEK - GRS Simulator", nullptr, nullptr));
    if (m_window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(m_window.get());

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
        throw std::runtime_error("Could not initialize GLAD!");
    glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    m_gui.init(m_window.get());

    int width;
    int height;
    glfwGetFramebufferSize(m_window.get(), &width, &height);
    glViewport(0, 0, width, height);
    glfwSwapInterval(1); // vsync: no tearing, and swap blocks instead of spinning
    glfwSwapBuffers(m_window.get());

    m_gui.drawAll();

    m_enableGlfwDebugOutput();
    glfwSetWindowUserPointer(m_window.get(), this);
    m_setCallback();
    m_isInitialized = true;

    if (!m_programArgs.configFile.empty()) {
        m_gui.setConfigFile(m_programArgs.configFile);
    }

    // Frame pacing: the viewer never needs to run faster than renderFps.
    // When 3D rendering is paused, only the GUI is refreshed at a low rate.
    using clock = std::chrono::steady_clock;
    const auto framePeriod = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1.0, m_programArgs.renderFps)));
    const auto idlePeriod = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(0.1));
    auto nextFrame = clock::now();

    // If the requested rate is at or above the monitor refresh, vsync already
    // paces the loop; sleeping as well would beat against vblank and stutter.
    // Falls back to sleep pacing if vsync turns out to be disabled (driver
    // setting, remote desktop, VM), detected by swaps that return too fast.
    bool vsyncPaced = mode->refreshRate > 0 && m_programArgs.renderFps >= mode->refreshRate - 1;
    int fastSwapCount = 0;

    // Simulation loop
    while (!glfwWindowShouldClose(m_window.get())) {

        // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();

        // Minimized window: draw nothing, just wait for events
        if (glfwGetWindowAttrib(m_window.get(), GLFW_ICONIFIED)) {
            glfwWaitEventsTimeout(0.25);
            nextFrame = clock::now();
            continue;
        }

        if(m_gui.getHasConfigFile() && !m_sceneCreated) {
            m_world->readConfigurationFile(m_gui.configFilePath());
            m_world->createSystem();
            m_createScene();

            m_world->startPhysicsThread();

            m_scene->reshape(m_windowWidth, m_windowHeight);
            m_scene->setWindowDimension(m_windowWidth, m_windowHeight);
            m_scene->reset();
            m_syncGuiWithSystem();
        }

        // End of run reported by the physics thread (handled here, on the UI thread)
        if (m_sceneCreated && m_world->consumeEndOfRun()) {
            m_pauseSimulation();
            LOG_INFO("Reach end of trajectory!");
        }

        const auto snapshot = m_sceneCreated ? m_world->getLatestSnapshot() : nullptr;
        if (snapshot) {
            m_elapsedTime = snapshot->elapsedTime;
        }

        const bool renderScene = m_sceneCreated && m_renderEnabled;

        if (renderScene) {
            m_update(snapshot.get());
        } else {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        m_gui.update(snapshot.get());
        m_gui.draw();

        if (renderScene) {
            m_render();
        }

        m_gui.drawTearDown();

        // Sleep until the next frame instead of spinning
        if (renderScene && vsyncPaced) {
            const auto now = clock::now();
            fastSwapCount = (now - nextFrame < framePeriod / 2) ? fastSwapCount + 1 : 0;
            if (fastSwapCount > 30) {
                vsyncPaced = false;
                LOG_INFO("Vsync not effective, using sleep frame pacing");
            }
            nextFrame = now;
            continue;
        }
        nextFrame += renderScene ? framePeriod : idlePeriod;
        const auto now = clock::now();
        if (now < nextFrame) {
            std::this_thread::sleep_until(nextFrame);
        } else {
            nextFrame = now;
        }
    }

    // Terminate GLFW, clearing any resources allocated by GLFW.
    m_world->stopPhysicsThread();
    glfwTerminate();
}

bool SimulationViewer::onWindowResized(const int width, const int height) {
    if (!m_isInitialized) return false;

    m_gui.onWindowResizeEvent(width, height);

    // Resize the camera viewport
    if (m_scene) {
        m_scene->reshape(width, height);
        m_scene->setWindowDimension(width, height);
    }

    m_windowWidth = width;
    m_windowHeight = height;

    return true;
}

void SimulationViewer::keyboardEvent(const int key, const int scancode, const int action, const int modifiers) {
    m_gui.onKeyboardEvent(key, scancode, action, modifiers);

    // Close application on escape key
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_window.get(), GL_TRUE);
        return;
    }

    // Show/hide the GUI with "c" key
    if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        m_gui.setIsDisplayed(!m_gui.getIsDisplayed());
        return;
    }

    // Toggle the close-up view with "g" key
    if (key == GLFW_KEY_G && action == GLFW_PRESS) {
        m_toggleCloseUp();
        return;
    }

    // Toggle the follow camera with "f" key
    if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        m_toggleFollowSystem();
        return;
    }

    // Pause/resume the 3D rendering with "v" key (simulation keeps running)
    if (key == GLFW_KEY_V && action == GLFW_PRESS) {
        m_renderEnabled = !m_renderEnabled;
        LOG_INFO(m_renderEnabled ? "3D rendering resumed" : "3D rendering paused");
        return;
    }
}

void SimulationViewer::mouseButtonEvent(const int button, const int action, const int modifiers, const double mousePosX, const double mousePosY) const {
    if (m_scene && button == GLFW_MOUSE_BUTTON_LEFT && m_scene->closeUpMouseButton(action == GLFW_PRESS, mousePosX, mousePosY)) {
        return;
    }
    if (m_scene) {
        m_scene->mouseButtonEvent(button, action == GLFW_PRESS, modifiers, mousePosX, mousePosY);
    }
}

void SimulationViewer::mouseMotionEvent(const double x, const double y) const {
    if (m_scene && m_scene->closeUpMouseMotion(x, y)) {
        return;
    }

    const int leftButtonState   = glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_LEFT);
    const int rightButtonState  = glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_RIGHT);
    const int middleButtonState = glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_MIDDLE);
    const int altKeyState       = glfwGetKey(m_window.get(), GLFW_KEY_LEFT_ALT);

    if (m_scene) {
        m_scene->mouseMotionEvent(x, y, leftButtonState, rightButtonState, middleButtonState, altKeyState);
    }
}

void SimulationViewer::scrollEvent(const double x, const double y) const {
    double cursorX;
    double cursorY;
    glfwGetCursorPos(m_window.get(), &cursorX, &cursorY);
    if (m_scene && m_scene->closeUpScroll(y, cursorX, cursorY)) {
        return;
    }
    if (m_scene) {
        m_scene->scrollingEvent(x, y, 0.05f);
    }
}

void SimulationViewer::m_createScene() {
    m_destroyScenes(); // a previous scene may remain after Stop
    m_scene = new Scene(World::getExecutablePath(), m_world->getSceneConfiguration(), *m_world->getLatestSnapshot());
    m_scene->setFollowSystem(m_followSystem);
    m_scene->setAircraftDisplayScale(m_aircraftScale);
    m_scene->setCloseUpEnabled(m_closeUpEnabled);
    m_scene->setCloseUpMode(m_closeUpMode);
    m_sceneCreated = true;
}

void SimulationViewer::m_destroyScenes() {
    delete m_scene;
    m_scene = nullptr;
}

void SimulationViewer::m_playSimulation() {
    if (!m_timer.isRunning()) {
        m_timer.start();
        m_world->start();
    }
}

void SimulationViewer::m_pauseSimulation() {
    if (m_timer.isRunning()) {
        m_timer.stop();
        m_world->stop();
    }
}

void SimulationViewer::m_restartSimulation() {
    if (!m_sceneCreated) return;

    if (m_timer.isRunning()) {
        m_timer.stop();
        m_world->stop();
    }
    m_timer.reset();
    m_elapsedTime = 0.0;

    // resetSystem() stops the physics thread before replacing the system
    m_world->resetSystem();
    m_scene->reset();
    m_syncGuiWithSystem();
}

void SimulationViewer::m_stopSimulation() {
    if (!m_sceneCreated) return;

    if (m_timer.isRunning()) {
        m_timer.stop();
    }
    m_timer.reset();
    m_elapsedTime = 0.0;

    // GUI must forget the frames before the system they describe is deleted
    m_gui.clearFrames();

    // deleteSystem() always stops the physics thread first, even when paused
    m_world->deleteSystem();
    m_scene->destroy();
    m_sceneCreated = false;
}

void SimulationViewer::m_syncGuiWithSystem() {
    const target t = m_world->initialTarget();
    m_gui.setTargetFromConfig(t.groundspeed, t.positionPayload);
    m_scene->updateTarget(t.positionPayload);

    if (const auto snapshot = m_world->getLatestSnapshot()) {
        m_gui.setFrames(*snapshot);
    }
}

void SimulationViewer::m_setGlfwHints() const {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, m_programArgs.msaaSamples); // MSAA: smooth edges on wings, tethers and frame axes
    glfwWindowHint(GLFW_RED_BITS, 8);
    glfwWindowHint(GLFW_GREEN_BITS, 8);
    glfwWindowHint(GLFW_BLUE_BITS, 8);
    glfwWindowHint(GLFW_ALPHA_BITS, 8);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
}

void SimulationViewer::m_setCallback() const {

    glfwSetCursorPosCallback(m_window.get(), [](GLFWwindow* window, const double x, const double y) {
        auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));

        const nanogui::Vector2i p = {static_cast<int>(x), static_cast<int>(y)};

        // Send first to nanogui
        app->m_gui.onMouseMotionEvent(x, y);

        // Check if GUI
        const nanogui::Widget* widget = app->m_gui.findWidget(p);
        const bool guiActive = widget && widget != app->m_gui.screen();

        if (!guiActive) {
            app->mouseMotionEvent(x, y);
        }

    });

    glfwSetMouseButtonCallback(m_window.get(), [](GLFWwindow* window, const int button, const int action, const int modifiers) {
        auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));

        double x;
        double y;
        glfwGetCursorPos(window, &x, &y);
        const nanogui::Vector2i p = {static_cast<int>(x), static_cast<int>(y)};

        // Send first to nanogui
        app->m_gui.onMouseButtonEvent(button, action, modifiers);

        // Check if GUI
        const nanogui::Widget* widget = app->m_gui.findWidget(p);
        const bool guiActive = widget && widget != app->m_gui.screen();

        if (!guiActive) {
            app->mouseButtonEvent(button, action, modifiers, x, y);
        }

    });

    glfwSetKeyCallback(m_window.get(), [](GLFWwindow* window, const int key, const int scancode, const int action, const int mods) {
        auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
        app->keyboardEvent(key, scancode, action, mods);
    });

    glfwSetScrollCallback(m_window.get(), [](GLFWwindow* window, const double x, const double y) {
        auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
        app->scrollEvent(x, y);
    });

    glfwSetFramebufferSizeCallback(m_window.get(), [](GLFWwindow* window, const int width, const int height) {
        auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
        app->onWindowResized(width, height);
    });
}

void GLAPIENTRY SimulationViewer::onOpenGLError(const GLenum source, const GLenum type, GLuint /*id*/, const GLenum severity, GLsizei /*length*/, const GLchar* message, const void* /*userParam*/ ) {
#ifdef GL_DEBUG_OUTPUT
    if (type == GL_DEBUG_TYPE_ERROR) {
        fprintf( stderr, "GL CALLBACK: %s source = 0x%x, type = 0x%x, severity = 0x%x, message = %s\n",
                 ("** GL ERROR **" ), source, type, severity, message );
    }
#endif
}

void SimulationViewer::m_update(const systemRenderData* snapshot) {

    // Compute the interpolation factor , 1 because physic is assume to be quicker than rendering
    constexpr double factor = 1.0;
    assert(factor >= 0.0 && factor <= 1.0);

    // Notify the scene about the interpolation factor
    m_scene->setInterpolationFactor(factor);

    // Update the scene
    if (snapshot) {
        m_scene->update(*snapshot);
    }
}

void SimulationViewer::m_updateTarget(const float groundspeed, const std::vector<double>& payloadPosition) const {
    if (!m_sceneCreated) return;

    m_scene->updateTarget(payloadPosition);

    // Applied by the physics thread before its next step
    target t = m_world->initialTarget();
    t.groundspeed = groundspeed;
    t.positionPayload = payloadPosition;
    m_world->requestTarget(t);
}

void SimulationViewer::m_changeView(const grs::Matrix4d& viewMatrix) const {
    m_scene->changeView(viewMatrix);
}

void SimulationViewer::m_launchTriggered() const {
    if (!m_sceneCreated) return;
    m_world->requestLaunch();
}

void SimulationViewer::m_drawReferenceFrame(const bool shouldDraw) const {
    m_scene->drawReferenceFrame(shouldDraw);
}

void SimulationViewer::m_drawForcesVector(const bool shouldDraw) const {
    m_scene->drawForcesVector(shouldDraw);
}

void SimulationViewer::m_toggleCloseUp() {
    m_closeUpEnabled = !m_closeUpEnabled;
    if (m_scene) {
        m_scene->setCloseUpEnabled(m_closeUpEnabled);
    }
    m_gui.setCloseUpCaptions(m_closeUpEnabled, m_closeUpMode == CloseUpView::Mode::XRay);
}

void SimulationViewer::m_toggleCloseUpMode() {
    m_closeUpMode = m_closeUpMode == CloseUpView::Mode::Wireframe ? CloseUpView::Mode::XRay : CloseUpView::Mode::Wireframe;
    if (m_scene) {
        m_scene->setCloseUpMode(m_closeUpMode);
    }
    m_gui.setCloseUpCaptions(m_closeUpEnabled, m_closeUpMode == CloseUpView::Mode::XRay);
}

void SimulationViewer::m_setAircraftScale(const double scale) {
    m_aircraftScale = scale;
    if (m_scene) {
        m_scene->setAircraftDisplayScale(scale);
    }
}

void SimulationViewer::m_toggleFollowSystem() {
    m_followSystem = !m_followSystem;
    if (m_scene) {
        m_scene->setFollowSystem(m_followSystem);
    }
    m_gui.setFollowCaption(m_followSystem);
}

void SimulationViewer::m_drawSky(const bool shouldDraw) const {
    m_scene->drawSky(shouldDraw);
}

void SimulationViewer::m_render() const {
    int bufferWidth;
    int bufferHeight;
    glfwMakeContextCurrent(m_window.get());
    glfwGetFramebufferSize(m_window.get(), &bufferWidth, &bufferHeight);

    m_scene->setViewport(0, 0, bufferWidth, bufferHeight);
    m_scene->render();

    if (m_closeUpEnabled) {
        int windowWidth;
        int windowHeight;
        glfwGetWindowSize(m_window.get(), &windowWidth, &windowHeight);
        const float pixelRatio = windowWidth > 0 ? static_cast<float>(bufferWidth) / static_cast<float>(windowWidth) : 1.0f;

        const auto [body, frame] = m_gui.selectedFrame();
        m_scene->setCloseUpSelection(body, frame);
        m_scene->renderCloseUp(bufferWidth, bufferHeight, pixelRatio);

        // Markers and labels (NanoVG, window coordinates); the GUI widgets are drawn after
        NVGcontext* vg = m_gui.screen()->nvg_context();
        nvgBeginFrame(vg, static_cast<float>(windowWidth), static_cast<float>(windowHeight), pixelRatio);
        m_scene->drawCloseUpOverlay(vg);
        nvgEndFrame(vg);
    }
}

void SimulationViewer::m_enableGlfwDebugOutput() {
    int glMajorVersion;
    int glMinorVersion;
    glGetIntegerv(GL_MAJOR_VERSION, &glMajorVersion);
    glGetIntegerv(GL_MINOR_VERSION, &glMinorVersion);

    if (glMajorVersion > 4 || (glMajorVersion == 4 && glMinorVersion >= 3)) {
        // Enable OpenGL error reporting
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(onOpenGLError, nullptr);
    }
}
