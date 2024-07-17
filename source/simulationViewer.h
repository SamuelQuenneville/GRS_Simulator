/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SIMULATIONVIEWER_H
#define GRS_SIMULATOR_SIMULATIONVIEWER_H

#include "gui.h"

#include <iostream>
#include <GLFW/glfw3.h>

class SimulationViewer {

public:
    SimulationViewer();
    ~SimulationViewer();

    void start();
    //void render();

    bool onWindowResized(int width, int height);

    // Default event handler
    void keyboard_event(int key, int scancode, int action, int modifiers);
    void mouse_button_event(int button, int action, int modifiers);
    void mouse_motion_event(double x, double y);
    void scroll_event(double x, double y);

    // Called when an OpenGL Error occurs
    static void GLAPIENTRY onOpenGLError(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam );

    friend class Gui;

private:
    static const float SCROLL_SENSITIVITY;

    bool m_isInitialized;

    GLFWwindow* m_window;
    int m_windowWidth;
    int m_windowHeight;

    Gui m_gui;

    static void m_setGlfwHints();
    void m_setCallback();

    //void m_createScene();
    //void m_destroyScenes();
};


#endif //GRS_SIMULATOR_SIMULATIONVIEWER_H
