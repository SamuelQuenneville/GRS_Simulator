/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_GUI_H
#define GRS_SIMULATOR_GUI_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <sstream>
#include <iomanip>
#include <cstring>

#include "Log/programLogger.h"
#include "RigidBodyAndLink/frame.h"

class SimulationViewer;

class Gui {

public:
    explicit Gui(SimulationViewer* app);
    ~Gui() = default;

    // Initialize the GUI
    void init(GLFWwindow* window);

    void update() const;
    void drawAll();
    void draw();
    void drawTearDown();

    void onWindowResizeEvent(int width, int height);
    void onMouseMotionEvent(double x, double y);
    void onMouseButtonEvent(int button, int action, int modifiers);
    void onKeyboardEvent(int key, int scancode, int action, int modifiers);

    nanogui::Widget* findWidget(const nanogui::Vector2i& p);
    [[nodiscard]] nanogui::ref<nanogui::Screen> screen() const {return m_screen;};

    [[nodiscard]] bool getIsDisplayed() const {return m_isDisplayed;};
    void setIsDisplayed(const bool isDisplayed) {m_isDisplayed = isDisplayed;};

    [[nodiscard]] bool getHasConfigFile() const {return m_hasConfigFile;};
    [[nodiscard]] std::string configFilePath() const {return m_configFilePath;};

    void setTargetFromConfig(float groundspeed, const std::vector<double>& positionTarget);
    void setFrames(const std::vector<Frame*>& frames);

protected:
    SimulationViewer* m_viewer;

    GLFWwindow* m_window;
    nanogui::ref<nanogui::Screen> m_screen;

    // Configuration panel
    nanogui::Widget* m_configurationPanel;
    nanogui::Widget* m_debugPanel;
    nanogui::Label* m_fileLabel;
    nanogui::Label* m_simRunTimeLabel;

    bool m_shouldDrawReferenceFrame = false;
    bool m_shouldDrawForcesVector = false;

    nanogui::TextBox* m_groundspeedTargetText;
    float m_groundspeedTarget = 0.0;

    nanogui::TextBox* m_altitudeTargetText;
    nanogui::TextBox* m_positionXTargetText;
    nanogui::TextBox* m_positionYTargetText;
    std::vector<double> m_payloadTarget = {0.0, 0.0, 0.0};

    nanogui::Label* m_framePositionDisplayed;
    nanogui::Label* m_frameVelocityDisplayed;
    nanogui::Label* m_frameOrientationDisplayed;
    nanogui::Label* m_frameForcesDisplayed;

    nanogui::ComboBox* m_comboBoxFrames = nullptr;

    std::vector<Frame*> m_frames{};
    Frame* m_currentFrameDisplayed{};

    //
    char m_configFilePath[256] = {};
    char m_drive[3] = {};
    char m_dir[256] = {};
    char m_configFileName[256] = {};
    char m_ext[256] = {};

    bool m_hasConfigFile;

    // True if the GUI is displayed
    bool m_isDisplayed;

    void m_createConfigurationPanel();
    void m_createDebugPanel();
    static std::string doubleToString(double value, int precision);
};


#endif //GRS_SIMULATOR_GUI_H
