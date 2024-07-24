/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SIMULATIONVIEWER_H
#define GRS_SIMULATOR_SIMULATIONVIEWER_H

#include "gui.h" // need to be before GLFW include

#include <GLFW/glfw3.h>

#include "timer.h"
#include "Scenes/oneAircraftGrounded.h"

class SimulationViewer {

public:
    SimulationViewer();
    ~SimulationViewer();

    void start();

    bool onWindowResized(int width, int height);
    [[nodiscard]] double getElapsedTime() const {return m_elapsedTime;};

    // Default event handler
    void keyboard_event(int key, int scancode, int action, int modifiers);
    void mouse_button_event(int button, int action, int modifiers);
    void mouse_motion_event(double x, double y);
    void scroll_event(double x, double y);

    // Called when an OpenGL Error occurs
    static void GLAPIENTRY onOpenGLError(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam );

    friend class Gui;

private:
    bool m_isInitialized;

    GLFWwindow* m_window;
    int m_windowWidth;
    int m_windowHeight;

    std::unique_ptr<World> m_world;
    Timer m_timer;

    Gui m_gui;
    Scene* m_scene;

    double m_elapsedTime;
    double m_totalDynamicTime;
    double m_dynamicStepTime;

    void m_update();
    void m_updateDynamics();
    void m_updateSingleDynamicsStep();

    void m_render();

    bool m_sceneCreated;
    void m_createScene();
    void m_destroyScenes();

    void m_playSimulation();
    void m_toggleTakeSingleStep();
    void m_pauseSimulation();
    void m_restartSimulation();

    bool m_singlePhysicsStepEnabled;
    bool m_singlePhysicsStepDone;

    static void m_setGlfwHints();
    void m_setCallback();
    static void m_enableGlfwDebugOutput();
};


#endif //GRS_SIMULATOR_SIMULATIONVIEWER_H
