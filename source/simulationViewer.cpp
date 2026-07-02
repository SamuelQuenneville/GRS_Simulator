/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "simulationViewer.h"

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
    glfwSwapInterval(0);
    glfwSwapBuffers(m_window.get());

    m_gui.drawAll();

    m_enableGlfwDebugOutput();
    glfwSetWindowUserPointer(m_window.get(), this);
    m_setCallback();
    m_isInitialized = true;

    // Simulation loop
    while (!glfwWindowShouldClose(m_window.get())) {

        // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();

        if(m_gui.getHasConfigFile() && !m_sceneCreated) {
            m_world->readConfigurationFile(m_gui.configFilePath());
            m_world->createSystem();
            m_createScene();

            m_world->startPhysicsThread();

            // callback binding, enable to stop timer from system
            m_world->setPauseTimerCallback([this]() {
                m_pauseSimulation();
                LOG_INFO("Reach end of trajectory!");
            });

            m_gui.setTargetFromConfig(m_world->system()->getTarget().groundspeed, m_world->system()->getTarget().positionPayload);
            m_gui.setFrames(m_world->collectFrames());

            m_scene->reshape(m_windowWidth, m_windowHeight);
            m_scene->setWindowDimension(m_windowWidth, m_windowHeight);
            m_scene->reset();
        }

        if (m_sceneCreated) {
            m_update();
        }

        m_gui.update();
        m_gui.draw();

        if (m_sceneCreated) {
            m_render();
        }

        m_gui.drawTearDown();
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
}

void SimulationViewer::mouseButtonEvent(const int button, const int action, const int modifiers, const double mousePosX, const double mousePosY) const {
    if (m_scene) {
        m_scene->mouseButtonEvent(button, action == GLFW_PRESS, modifiers, mousePosX, mousePosY);
    }
}

void SimulationViewer::mouseMotionEvent(const double x, const double y) const {

    const int leftButtonState   = glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_LEFT);
    const int rightButtonState  = glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_RIGHT);
    const int middleButtonState = glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_MIDDLE);
    const int altKeyState       = glfwGetKey(m_window.get(), GLFW_KEY_LEFT_ALT);

    if (m_scene) {
        m_scene->mouseMotionEvent(x, y, leftButtonState, rightButtonState, middleButtonState, altKeyState);
    }
}

void SimulationViewer::scrollEvent(const double x, const double y) const {
    if (m_scene) {
        m_scene->scrollingEvent(x, y, 0.05f);
    }
}

void SimulationViewer::m_createScene() {
    m_scene = new Scene(World::getExecutablePath(), m_world->getSceneConfiguration(), *m_world->getLatestSnapshot());
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
    if (m_timer.isRunning()) {
        m_timer.stop();
        m_world->stop();
        m_timer.reset();
    }
    m_elapsedTime = 0.0;

    m_world->resetSystem();
    m_scene->reset();
}

void SimulationViewer::m_stopSimulation() {
    if (m_timer.isRunning()) {
        m_timer.stop();
        m_world->stop();
        m_world->stopPhysicsThread();
    }
    m_elapsedTime = 0.0;

    m_world->deleteSystem();
    m_scene->destroy();
    m_sceneCreated = false;
}

void SimulationViewer::m_setGlfwHints() {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 0);
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

void SimulationViewer::m_update() {

    // Compute the interpolation factor , 1 because physic is assume to be quicker than rendering
    constexpr double factor = 1.0;
    assert(factor >= 0.0 && factor <= 1.0);

    // Notify the scene about the interpolation factor
    m_scene->setInterpolationFactor(factor);

    // Update the scene
    if (const auto snapshot = m_world->getLatestSnapshot()) {
        m_scene->update(*snapshot);
        m_elapsedTime = snapshot->elapsedTime;
    }
}

void SimulationViewer::m_updateTarget(const float groundspeed, const std::vector<double>& payloadPosition) const {
    m_scene->updateTarget(payloadPosition);

    target t = m_world->system()->getTarget();
    t.groundspeed = groundspeed;
    t.positionPayload = payloadPosition;
    m_world->system()->setTarget(t);
}

void SimulationViewer::m_changeView(const grs::Matrix4d& viewMatrix) const {
    m_scene->changeView(viewMatrix);
}

void SimulationViewer::m_launchTriggered() const {
    m_world->system()->setLaunchTriggered(true);
}

void SimulationViewer::m_drawReferenceFrame(const bool shouldDraw) const {
    m_scene->drawReferenceFrame(shouldDraw);
}

void SimulationViewer::m_drawForcesVector(const bool shouldDraw) const {
    m_scene->drawForcesVector(shouldDraw);
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
