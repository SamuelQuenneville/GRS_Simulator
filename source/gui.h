/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_GUI_H
#define GRS_SIMULATOR_GUI_H

#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include <sstream>
#include <iomanip>
#include <cstring>

class SimulationViewer;

class Gui {

public:
    explicit Gui(SimulationViewer* app);
    ~Gui() = default;

    /// Initialize the GUI
    void init(GLFWwindow* window);

    void update();
    void drawAll();
    void draw();
    void drawTearDown();

    void onWindowResizeEvent(int width, int height);
    void onMouseMotionEvent(double x, double y);
    bool onScrollEvent(double x, double y);
    void onMouseButtonEvent(int button, int action, int modifiers);
    void onKeyboardEvent(int key, int scancode, int action, int modifiers);

    [[nodiscard]] bool getIsDisplayed() const {return m_isDisplayed;};
    void setIsDisplayed(bool isDisplayed) {m_isDisplayed = isDisplayed;};

    [[nodiscard]] bool getHasConfigFile() const {return m_hasConfigFile;};
    [[nodiscard]] std::string configFilePath() const {return m_configFilePath;};

protected:
    SimulationViewer* m_viewer;

    GLFWwindow* m_window;
    nanogui::Screen* m_screen;

    // Configuration panel
    nanogui::Widget* m_configurationPanel;
    nanogui::Label* m_fileLabel;
    nanogui::Label* m_simRunTimeLabel;

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
    std::string doubleToString(double value, int precision);
};


#endif //GRS_SIMULATOR_GUI_H
