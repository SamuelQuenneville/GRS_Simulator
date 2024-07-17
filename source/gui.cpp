/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "gui.h"
#include <GLFW/glfw3.h>
#include "simulationViewer.h"

Gui::Gui(SimulationViewer* app)
        : m_viewer(app)
        , m_screen(nullptr)
        , m_window(nullptr)
        , m_configurationPanel(nullptr)
        , m_fileLabel(nullptr)
        , m_isDisplayed(true)
{

}

void Gui::init(GLFWwindow* window) {
    m_window = window;

    m_screen = new nanogui::Screen();
    m_screen->initialize(window, true);

    m_createConfigurationPanel();

    m_screen->set_visible(true);
    m_screen->perform_layout();
}

void Gui::update() {
    m_fileLabel->set_caption(std::string("File : ") + m_configFileName);
}

void Gui::drawAll() {
    m_screen->clear();
    m_screen->draw_all();
}

void Gui::draw() {
    if (m_isDisplayed) {
        m_screen->draw_setup();
        m_screen->draw_contents();
    }
}

void Gui::drawTearDown() {
    if (m_isDisplayed) {
        m_screen->draw_widgets();
    }
    m_screen->draw_teardown();
}

void Gui::onWindowResizeEvent(int width, int height) {
    m_screen->resize_callback_event(width, height);
}

void Gui::onMouseMotionEvent(double x, double y) {
    m_screen->cursor_pos_callback_event(x, y);
}

bool Gui::onScrollEvent(double x, double y) {

    double xMouse, yMouse;
    glfwGetCursorPos(m_window, &xMouse, &yMouse);

    m_screen->scroll_callback_event(x, y);

    return false;
}

void Gui::onMouseButtonEvent(int button, int action, int modifiers) {
    m_screen->mouse_button_callback_event(button, action, modifiers);
}

void Gui::onKeyboardEvent(int key, int scancode, int action, int modifiers) {
    m_screen->key_callback_event(key, scancode, action, modifiers);
}

void Gui::m_createConfigurationPanel() {
    m_configurationPanel = new nanogui::Window(m_screen, "Interface");
    m_configurationPanel->set_position(nanogui::Vector2i(15, 15));
    m_configurationPanel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 5));
    m_configurationPanel->set_fixed_width(220);

    // Group 1 Sim File
    new nanogui::Label(m_configurationPanel, std::string("Select Simulation :"),"sans-bold",20);
    auto* group1 = new nanogui::Widget(m_configurationPanel);
    group1->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,nanogui::Alignment::Fill, 5, 5));
    auto* b = new nanogui::Button(group1, "Choose Config");
    b->set_callback([&] {
        std::strcpy(m_configFilePath,nanogui::file_dialog({{"yaml", "Config File"}}, false).c_str());
        _splitpath( m_configFilePath, m_drive, m_dir, m_configFileName, m_ext);
    });

    m_fileLabel = new nanogui::Label(group1, std::string("File : "),"sans-bold");

    // Group 2 Sim Infos
    new nanogui::Label(m_configurationPanel, std::string("Simulation Infos :"),"sans-bold",20);
    auto* group2 = new nanogui::Widget(m_configurationPanel);
    group2->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,nanogui::Alignment::Fill, 5, 5));
    m_simRunTime  = new nanogui::Label(group2, std::string("Run Time : "));
}

std::string Gui::doubleToString(double value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}
