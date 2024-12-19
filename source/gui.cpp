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
        , m_simRunTimeLabel(nullptr)
        , m_hasConfigFile(false)
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
    m_simRunTimeLabel->set_caption(std::string("Run Time : ") + doubleToString(m_viewer->getElapsedTime(),2));
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
    m_configurationPanel->set_fixed_width(260);

    // Group 1 Sim File
    new nanogui::Label(m_configurationPanel, std::string("Select Simulation :"),"sans-bold",20);
    auto* group1 = new nanogui::Widget(m_configurationPanel);
    group1->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,nanogui::Alignment::Fill, 5, 5));
    auto* buttonConfig = new nanogui::Button(group1, "Choose Config");
    buttonConfig->set_callback([&] {
        std::strcpy(m_configFilePath,nanogui::file_dialog({{"yaml", "Config File"}}, false).c_str());
        _splitpath( m_configFilePath, m_drive, m_dir, m_configFileName, m_ext);

        m_hasConfigFile = true;
    });

    m_fileLabel = new nanogui::Label(group1, std::string("File : "),"sans-bold");

    // Group 2 Control Buttons
    new nanogui::Label(m_configurationPanel, std::string("Controls :"),"sans-bold",20);
    auto* group2 = new nanogui::Widget(m_configurationPanel);
    group2->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 5, 5));
    auto* buttonPlay = new nanogui::ToolButton(group2, FA_PLAY);
    buttonPlay->set_flags(nanogui::Button::NormalButton);
    buttonPlay->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_playSimulation();
        }
    });
    auto* buttonPlayStep = new nanogui::ToolButton(group2, FA_STEP_FORWARD);
    buttonPlayStep->set_flags(nanogui::Button::NormalButton);
    buttonPlayStep->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_toggleTakeSingleStep();
        }
    });
    auto* buttonPause = new nanogui::ToolButton(group2, FA_PAUSE);
    buttonPause->set_flags(nanogui::Button::NormalButton);
    buttonPause->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_pauseSimulation();
        }
    });
    auto* buttonRestart = new nanogui::ToolButton(group2, FA_REDO);
    buttonRestart->set_flags(nanogui::Button::NormalButton);
    buttonRestart->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_restartSimulation();
        }
    });
    auto* buttonStop = new nanogui::ToolButton(group2, FA_STOP);
    buttonStop->set_flags(nanogui::Button::NormalButton);
    buttonStop->set_callback([&] {
       if (m_hasConfigFile) {
           m_hasConfigFile = false;
           m_configFilePath[0] = '\0';
           m_configFileName[0] = '\0';
           m_viewer->m_stopSimulation();
       }
    });

    // Group 3 Sim Infos
    new nanogui::Label(m_configurationPanel, std::string("Simulation Infos :"),"sans-bold",20);
    auto* group3 = new nanogui::Widget(m_configurationPanel);
    group3->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,nanogui::Alignment::Fill, 5, 5));
    m_simRunTimeLabel  = new nanogui::Label(group3, std::string("Run Time : "));

    // Group 4 Target Control
    new nanogui::Label(m_configurationPanel, std::string("Targets :"), "sans-bold", 20);
    auto* group4 = new nanogui::Widget(m_configurationPanel);
    group4->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 5));

    // Group 4 - Subgroup1 Groundspeed
    auto* subgroup1 = new nanogui::Widget(group4);
    subgroup1->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));
    auto* a = new nanogui::Label(subgroup1, std::string("Groundspeed (m/s) : "));
    a->set_fixed_width(120);

    m_groundspeedTargetText = new nanogui::TextBox(subgroup1);
    m_groundspeedTargetText->set_value("0.0");
    m_groundspeedTargetText->set_fixed_width(45);

    auto* buttonGroundspeedPlus = new nanogui::ToolButton(subgroup1, FA_PLUS);
    buttonGroundspeedPlus->set_flags(nanogui::Button::NormalButton);
    buttonGroundspeedPlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_groundspeedTarget += 0.5;
            m_groundspeedTargetText->set_value(doubleToString(m_groundspeedTarget,1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonGroundspeedMinus = new nanogui::ToolButton(subgroup1, FA_MINUS);
    buttonGroundspeedMinus->set_flags(nanogui::Button::NormalButton);
    buttonGroundspeedMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_groundspeedTarget -= 0.5;
            m_groundspeedTargetText->set_value(doubleToString(m_groundspeedTarget,1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    // Group 4 - Subgroup2 Altitude
    auto* subgroup2 = new nanogui::Widget(group4);
    subgroup2->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));
    auto* b = new nanogui::Label(subgroup2, std::string("Altitude (m) : "));
    b->set_fixed_width(120);

    m_altitudeTargetText = new nanogui::TextBox(subgroup2);
    m_altitudeTargetText->set_value("0.0");
    m_altitudeTargetText->set_fixed_width(45);

    auto* buttonAltitudePlus = new nanogui::ToolButton(subgroup2, FA_PLUS);
    buttonAltitudePlus->set_flags(nanogui::Button::NormalButton);
    buttonAltitudePlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[2] += 0.5;
            m_altitudeTargetText->set_value(doubleToString(m_payloadTarget[2], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonAltitudeMinus = new nanogui::ToolButton(subgroup2, FA_MINUS);
    buttonAltitudeMinus->set_flags(nanogui::Button::NormalButton);
    buttonAltitudeMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[2] -= 0.5;
            m_altitudeTargetText->set_value(doubleToString(m_payloadTarget[2], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    // Group 4 - Subgroup3 Position Payload x
    auto* subgroup3 = new nanogui::Widget(group4);
    subgroup3->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));
    auto* c = new nanogui::Label(subgroup3, std::string("Position x (m) : "));
    c->set_fixed_width(120);

    m_positionXTargetText = new nanogui::TextBox(subgroup3);
    m_positionXTargetText->set_value("0.0");
    m_positionXTargetText->set_fixed_width(45);

    auto* buttonPositionXPlus = new nanogui::ToolButton(subgroup3, FA_PLUS);
    buttonPositionXPlus->set_flags(nanogui::Button::NormalButton);
    buttonPositionXPlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[0] += 0.1;
            m_positionXTargetText->set_value(doubleToString(m_payloadTarget[0], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonPositionXMinus = new nanogui::ToolButton(subgroup3, FA_MINUS);
    buttonPositionXMinus->set_flags(nanogui::Button::NormalButton);
    buttonPositionXMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[0] -= 0.1;
            m_positionXTargetText->set_value(doubleToString(m_payloadTarget[0], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    // Group 4 - Subgroup4 Position Payload y
    auto* subgroup4 = new nanogui::Widget(group4);
    subgroup4->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));
    auto* d = new nanogui::Label(subgroup4, std::string("Position y (m) : "));
    d->set_fixed_width(120);

    m_positionYTargetText = new nanogui::TextBox(subgroup4);
    m_positionYTargetText->set_value("0.0");
    m_positionYTargetText->set_fixed_width(45);

    auto* buttonPositionYPlus = new nanogui::ToolButton(subgroup4, FA_PLUS);
    buttonPositionYPlus->set_flags(nanogui::Button::NormalButton);
    buttonPositionYPlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[1] += 0.1;
            m_positionYTargetText->set_value(doubleToString(m_payloadTarget[1], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonPositionYMinus = new nanogui::ToolButton(subgroup4, FA_MINUS);
    buttonPositionYMinus->set_flags(nanogui::Button::NormalButton);
    buttonPositionYMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[1] -= 0.1;
            m_positionYTargetText->set_value(doubleToString(m_payloadTarget[1], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });
}

std::string Gui::doubleToString(double value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

void Gui::setTargetFromConfig(double groundspeed, std::vector<double> positionTarget) {
    m_groundspeedTarget = groundspeed;
    m_groundspeedTargetText->set_value(doubleToString(m_groundspeedTarget,1));

    m_payloadTarget = positionTarget;
    m_altitudeTargetText->set_value(doubleToString(m_payloadTarget[2],1));
    m_positionXTargetText->set_value(doubleToString(m_payloadTarget[0],1));
    m_positionYTargetText->set_value(doubleToString(m_payloadTarget[1],1));
}
