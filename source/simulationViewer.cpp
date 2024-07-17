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
    , m_window(nullptr)
    , m_gui(this)

{
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

    int width, height;
    glfwGetFramebufferSize(m_window, &width, &height);
    glViewport(0, 0, width, height);
    glfwSwapInterval(0);
    glfwSwapBuffers(m_window);

    m_gui.drawAll();

    int glMajorVersion, glMinorVersion;
    glGetIntegerv(GL_MAJOR_VERSION, &glMajorVersion);
    glGetIntegerv(GL_MINOR_VERSION, &glMinorVersion);

    if (glMajorVersion > 4 || (glMajorVersion == 4 && glMinorVersion >= 3)) {
        // Enable OpenGL error reporting
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(onOpenGLError, nullptr);
    }

    glfwSetWindowUserPointer(m_window, this);
    m_setCallback();
    m_isInitialized = true;

    // Game loop
    while (!glfwWindowShouldClose(m_window)) {

        // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();

        //update();

        m_gui.update();
        m_gui.draw();

        //render();
        m_gui.drawTearDown();
    }

    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();
}

bool SimulationViewer::onWindowResized(int width, int height) {
    if (!m_isInitialized) return false;

    m_gui.onWindowResizeEvent(width, height);

    // Resize the camera viewport
    //mCurrentScene->reshape(width, height);
    //mCurrentScene->setWindowDimension(width, height);

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

    //mCurrentScene->mouseButtonEvent(button, down, modifiers, x, y);
}

void SimulationViewer::mouse_motion_event(double x, double y) {
    m_gui.onMouseMotionEvent(x, y);

    int leftButtonState   = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState  = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState       = glfwGetKey(m_window, GLFW_KEY_LEFT_ALT);

    //mCurrentScene->mouseMotionEvent(x, y, leftButtonState, rightButtonState, middleButtonState, altKeyState);
}

void SimulationViewer::scroll_event(double x, double y) {
    //mCurrentScene->scrollingEvent(x, y, SCROLL_SENSITIVITY);
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
                 ("** GL ERROR **" ),
                 source, type, severity, message );
    }
#endif

}
