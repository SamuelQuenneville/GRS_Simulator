/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "simulationViewer.h"

SimulationViewer::SimulationViewer()
    : m_isInitialized(false)
    , m_sceneCreated(false)
    , m_window(nullptr)
    , m_gui(this)
    , m_scene(nullptr)
    , m_totalDynamicTime(0)
    , m_dynamicStepTime(0)

{
    m_world = std::make_unique<World>();

    nanogui::init();

    // Get the primary monitor
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    // Window size
    m_windowWidth  = mode->width;
    m_windowHeight = mode->height;
}

SimulationViewer::~SimulationViewer(){
    //m_destroyScenes();
}

void SimulationViewer::start() {
    glfwInit();
    glfwSetTime(0);

    m_setGlfwHints();

    // Create a GLFW window object
    std::string title = "CREATEK - GRS Simulator";
    m_window = glfwCreateWindow(m_windowWidth, m_windowHeight, title.c_str(), nullptr, nullptr);
    if (m_window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(m_window);

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
        throw std::runtime_error("Could not initialize GLAD!");
    glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    m_gui.init(m_window);

    int width;
    int height;
    glfwGetFramebufferSize(m_window, &width, &height);
    glViewport(0, 0, width, height);
    glfwSwapInterval(0);
    glfwSwapBuffers(m_window);

    m_gui.drawAll();

    m_enableGlfwDebugOutput();
    glfwSetWindowUserPointer(m_window, this);
    m_setCallback();
    m_isInitialized = true;

    // Simulation loop
    while (!glfwWindowShouldClose(m_window)) {

        // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();

        if(m_gui.getHasConfigFile() && !m_sceneCreated) {
            m_world->readConfigurationFile(m_gui.configFilePath());
            m_world->createSystem();
            m_createScene();

            m_scene->reshape(m_windowWidth, m_windowHeight);
            m_scene->setWindowDimension(m_windowWidth, m_windowHeight);
            m_scene->reset();

            //m_timer.start();
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
    glfwTerminate();
}

bool SimulationViewer::onWindowResized(int width, int height) {
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

void SimulationViewer::keyboard_event(int key, int scancode, int action, int modifiers) {
    //mGui.onKeyboardEvent(key, scancode, action, modifiers);

    // Close application on escape key
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_window, GL_TRUE);
        return;
    }

    // Show/hide the GUI with "i" key
    if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        m_gui.setIsDisplayed(!m_gui.getIsDisplayed());
        return;
    }
}

void SimulationViewer::mouse_button_event(int button, int action, int modifiers) {
    m_gui.onMouseButtonEvent(button, action, modifiers);

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(m_window, &x, &y);

    bool down = action == GLFW_PRESS;

    if (m_scene) {
        m_scene->mouseButtonEvent(button, down, modifiers, x, y);
    }
}

void SimulationViewer::mouse_motion_event(double x, double y) {
    m_gui.onMouseMotionEvent(x, y);

    int leftButtonState   = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState  = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState       = glfwGetKey(m_window, GLFW_KEY_LEFT_ALT);

    if (m_scene) {
        m_scene->mouseMotionEvent(x, y, leftButtonState, rightButtonState, middleButtonState, altKeyState);
    }
}

void SimulationViewer::scroll_event(double x, double y) {
    if (m_scene) {
        m_scene->scrollingEvent(x, y, 1.0f);
    }
}

void SimulationViewer::m_createScene() {
    systemType type = m_world->getSystemType();
    std::string sceneName{};

    switch(type) {
        case ONE_AIRCRAFT:
            sceneName = "One Aircraft";

            m_sceneCreated = true;
            break;
        case ONE_AIRCRAFT_GROUNDED:
            sceneName = "One Aircraft Grounded";
            m_scene = new OneAircraftGrounded(sceneName, m_world.get());
            m_sceneCreated = true;
            break;
        case TWO_AIRCRAFT_GROUNDED_V:
            sceneName = "Two Aircraft Grounded V";

            m_sceneCreated = true;
            break;
        case TWO_AIRCRAFT_FREE_V:
            sceneName = "Two Aircraft Free V";

            m_sceneCreated = true;
            break;
        case TWO_AIRCRAFT_GROUNDED_Y:
            sceneName = "Two Aircraft Grounded Y";

            m_sceneCreated = true;
            break;
        case TWO_AIRCRAFT_FREE_Y:
            sceneName = "Two Aircraft Free Y";

            m_sceneCreated = true;
            break;
    }
}

void SimulationViewer::m_playSimulation() {
    if (!m_timer.isRunning()) {
        m_timer.start();
    }
}

void SimulationViewer::m_pauseSimulation() {
    if (m_timer.isRunning()) {
        m_timer.stop();
    }
}

void SimulationViewer::m_restartSimulation() {

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

void SimulationViewer::m_setCallback() {
    glfwSetCursorPosCallback(m_window, [](GLFWwindow* window, double x, double y) {
                                 auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
                                 app->mouse_motion_event(x, y);
                             }
    );

    glfwSetMouseButtonCallback(m_window, [](GLFWwindow* window, int button, int action, int modifiers) {
                                   auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
                                   app->mouse_button_event(button, action, modifiers);
                               }
    );

    glfwSetKeyCallback(m_window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
                           auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
                           app->keyboard_event(key, scancode, action, mods);
                       }
    );

    glfwSetScrollCallback(m_window, [](GLFWwindow* window, double x, double y) {
                              auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
                              app->scroll_event(x, y);
                          }
    );

    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow* window, int width, int height) {
                                       auto* app = static_cast<SimulationViewer*>(glfwGetWindowUserPointer(window));
                                       app->onWindowResized(width, height);
                                   }
    );
}

void GLAPIENTRY SimulationViewer::onOpenGLError(GLenum source, GLenum type, GLuint /*id*/, GLenum severity, GLsizei /*length*/, const GLchar* message, const void* /*userParam*/ ) {
#ifdef GL_DEBUG_OUTPUT
    if (type == GL_DEBUG_TYPE_ERROR) {
        fprintf( stderr, "GL CALLBACK: %s source = 0x%x, type = 0x%x, severity = 0x%x, message = %s\n",
                 ("** GL ERROR **" ), source, type, severity, message );
    }
#endif
}

void SimulationViewer::m_updateDynamics() {
    // Update the elapsed time
    m_scene->setElapsedTime(m_timer.getElapsedPhysicsTime());

    if (m_timer.isRunning()) {
        // Compute the time since the last update() call and update the timer
        m_timer.update();

        // While the time accumulator is not empty
        while(m_timer.isPossibleToTakeStep(m_world->simulationConfiguration.dt)) {

            double currentTime = glfwGetTime();

            // Take a physics simulation step
            m_scene->updatePhysics();

            m_dynamicStepTime = glfwGetTime() - currentTime;

            // Update the timer
            m_timer.nextStep(m_world->simulationConfiguration.dt);
        }
    }
}

void SimulationViewer::m_update() {
    double currentTime = glfwGetTime();

    m_updateDynamics();

    // Compute the physics update time
    m_totalDynamicTime = glfwGetTime() - currentTime;

    // Compute the interpolation factor
    double factor = m_timer.computeInterpolationFactor(m_world->simulationConfiguration.dt);
    assert(factor >= 0.0f && factor <= 1.0f);

    // Notify the scene about the interpolation factor
    m_scene->setInterpolationFactor(factor);

    // Update the scene
    m_scene->update();
}

void SimulationViewer::m_render() {
    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(m_window);
    glfwGetFramebufferSize(m_window, &bufferWidth, &bufferHeight);

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
