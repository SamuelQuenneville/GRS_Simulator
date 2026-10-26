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
#include "timer.h"
#include "world.h"
#include "Scenes/scene.h"

class SimulationViewer {

public:
    explicit SimulationViewer(programArguments& args);
    ~SimulationViewer();

    void start();

    bool onWindowResized(int width, int height);
    [[nodiscard]] double getElapsedTime() const {return m_elapsedTime;};

    // Default event handler
    void keyboardEvent(int key, int scancode, int action, int modifiers);
    void mouseButtonEvent(int button, int action, int modifiers, double mousePosX, double mousePosY) const;
    void mouseMotionEvent(double x, double y) const;
    void scrollEvent(double x, double y) const;

    // Called when an OpenGL Error occurs
    static void GLAPIENTRY onOpenGLError(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam );

    friend class Gui;

private:
    programArguments& m_programArgs;

    bool m_isInitialized;

    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> m_window;
    int m_windowWidth = 0;
    int m_windowHeight = 0;

    std::unique_ptr<World> m_world;
    Timer m_timer;

    Gui m_gui;
    Scene* m_scene;

    double m_elapsedTime;
    double m_dynamicStepTime;

    void m_update(const systemRenderData* snapshot);
    void m_syncGuiWithSystem();

    void m_updateTarget(float groundspeed, const std::vector<double> &payloadPosition) const;
    void m_changeView(const grs::Matrix4d& viewMatrix) const;
    void m_launchTriggered() const;

    void m_drawReferenceFrame(bool shouldDraw) const;
    void m_drawForcesVector(bool shouldDraw) const;
    void m_drawSky(bool shouldDraw) const;
    void m_toggleFollowSystem();
    void m_toggleCloseUp();
    void m_toggleCloseUpMode();
    void m_setAircraftScale(double scale);

    void m_render() const;

    bool m_sceneCreated;
    bool m_renderEnabled = true;
    bool m_followSystem = true;
    bool m_closeUpEnabled = false;
    CloseUpView::Mode m_closeUpMode = CloseUpView::Mode::Wireframe;
    double m_aircraftScale = 2.0;
    void m_createScene();
    void m_destroyScenes();

    void m_playSimulation();
    void m_pauseSimulation();
    void m_restartSimulation();
    void m_stopSimulation();

    bool m_singlePhysicsStepEnabled;
    bool m_singlePhysicsStepDone;

    void m_setGlfwHints() const;
    void m_setCallback() const;
    static void m_enableGlfwDebugOutput();
};


#endif //GRS_SIMULATOR_SIMULATIONVIEWER_H
