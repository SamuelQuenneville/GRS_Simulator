/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "simulationViewer.h"

auto formatVec3 = [](const char* label, const grs::Vec3d& v) {
    std::ostringstream oss;
    constexpr int numWidth = 7;

    // left align label, right align numbers
    oss << std::left << std::setw(18) << label << ": [ "
        << std::fixed << std::showpos << std::right << std::setw(numWidth) << std::setprecision(2) << v[0] << " "
        << std::fixed << std::showpos << std::right << std::setw(numWidth) << std::setprecision(2) << v[1] << " "
        << std::fixed << std::showpos << std::right << std::setw(numWidth) << std::setprecision(2) << v[2] << " ]";

    return oss.str();
};

Gui::Gui(SimulationViewer* app)
    : m_viewer(app)
    , m_window(nullptr)
    , m_screen(nullptr)
    , m_configurationPanel(nullptr)
    , m_debugPanel(nullptr)
    , m_fileLabel(nullptr)
    , m_simRunTimeLabel(nullptr)
    , m_groundspeedTargetText(nullptr)
    , m_altitudeTargetText(nullptr)
    , m_positionXTargetText(nullptr)
    , m_positionYTargetText(nullptr)
    , m_framePositionDisplayed(nullptr)
    , m_frameVelocityDisplayed(nullptr)
    , m_frameOrientationDisplayed(nullptr)
    , m_frameForcesDisplayed(nullptr)
    , m_hasConfigFile(false)
    , m_isDisplayed(true)
{

}

void Gui::init(GLFWwindow* window) {
    m_window = window;

    m_screen = new nanogui::Screen();
    m_screen->initialize(window, true);

    m_createConfigurationPanel();
    m_createDebugPanel();

    m_screen->set_visible(true);
    m_screen->perform_layout();
}

void Gui::update() const {
    m_fileLabel->set_caption(std::string("File : ") + m_configFileName);
    m_simRunTimeLabel->set_caption(std::string("Run Time : ") + doubleToString(m_viewer->getElapsedTime(),2));

    if (m_currentFrameDisplayed != nullptr) {
        const auto pos = m_currentFrameDisplayed->originNode().position();
        const auto vel = m_currentFrameDisplayed->originNode().velocity();
        auto ori = grs::quatToEuler(m_currentFrameDisplayed->transform().orientation.yUpToNed()) * (180.0 / M_PI);
        ori[0] += M_PI_2;

        const auto force = m_currentFrameDisplayed->originNode().force();

        m_framePositionDisplayed->set_caption(formatVec3("Position    (m)", pos));
        m_frameVelocityDisplayed->set_caption(formatVec3("Velocity    (m/s)", vel));
        m_frameOrientationDisplayed->set_caption(formatVec3("Orientation (Deg)", ori));
        m_frameForcesDisplayed->set_caption(formatVec3("Forces      (N)", force));
    }

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

void Gui::onWindowResizeEvent(const int width, const int height) {
    m_screen->resize_callback_event(width, height);
}

void Gui::onMouseMotionEvent(const double x, const double y) {
    m_screen->cursor_pos_callback_event(x, y);
}

void Gui::onMouseButtonEvent(const int button, const int action, const int modifiers) {
    m_screen->mouse_button_callback_event(button, action, modifiers);
}

void Gui::onKeyboardEvent(const int key, const int scancode, const int action, const int modifiers) {
    m_screen->key_callback_event(key, scancode, action, modifiers);
}

nanogui::Widget *Gui::findWidget(const nanogui::Vector2i &p) {
    return m_screen->find_widget(p);
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
    auto* buttonConfig = new nanogui::Button(group1, "Choose Configuration/Test");
    buttonConfig->set_callback([&] {
        std::strcpy(m_configFilePath,nanogui::file_dialog({{"yaml", "Config File"}}, false).c_str());

        const std::filesystem::path path(m_configFilePath);

        std::strcpy(m_dir, path.parent_path().string().c_str()); // Copy directory
        std::strcpy(m_configFileName, path.stem().string().c_str()); // Copy filename (without extension)
        std::strcpy(m_ext, path.extension().string().c_str()); // Copy extension

        m_hasConfigFile = true;
    });

    m_fileLabel = new nanogui::Label(group1, std::string("File : "),"sans-bold");

    // Group 2 Control Buttons
    new nanogui::Label(m_configurationPanel, std::string("Controls :"),"sans-bold",20);
    auto* group2 = new nanogui::Widget(m_configurationPanel);
    group2->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 5));

    auto* subgroup10 = new nanogui::Widget(group2);
    subgroup10->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));

    auto* buttonPlay = new nanogui::ToolButton(subgroup10, FA_PLAY);
    buttonPlay->set_flags(nanogui::Button::NormalButton);
    buttonPlay->set_fixed_width(42);
    buttonPlay->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_playSimulation();
        }
    });
    auto* buttonPlayStep = new nanogui::ToolButton(subgroup10, FA_STEP_FORWARD);
    buttonPlayStep->set_flags(nanogui::Button::NormalButton);
    buttonPlayStep->set_fixed_width(42);
    buttonPlayStep->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_toggleTakeSingleStep();
        }
    });
    auto* buttonPause = new nanogui::ToolButton(subgroup10, FA_PAUSE);
    buttonPause->set_flags(nanogui::Button::NormalButton);
    buttonPause->set_fixed_width(42);
    buttonPause->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_pauseSimulation();
        }
    });
    auto* buttonRestart = new nanogui::ToolButton(subgroup10, FA_REDO);
    buttonRestart->set_flags(nanogui::Button::NormalButton);
    buttonRestart->set_fixed_width(42);
    buttonRestart->set_callback([&] {
        if (m_hasConfigFile) {
            m_viewer->m_restartSimulation();
        }
    });
    auto* buttonStop = new nanogui::ToolButton(subgroup10, FA_STOP);
    buttonStop->set_flags(nanogui::Button::NormalButton);
    buttonStop->set_fixed_width(42);
    buttonStop->set_callback([&] {
       if (m_hasConfigFile) {
           m_hasConfigFile = false;
           m_configFilePath[0] = '\0';
           m_configFileName[0] = '\0';
           m_viewer->m_stopSimulation();
       }
    });

    auto* subgroup11 = new nanogui::Widget(group2);
    subgroup11->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));

    auto* buttonTopView = new nanogui::Button(subgroup11, "Top View");
    buttonTopView->set_fixed_width(73);
    buttonTopView->set_callback([&] {

        const grs::Matrix4d view
            {1,  0,  0, 0,
             0,  0,  1, 0,
             0, -1,  0, 0,
             0,  0,  0, 1};

        m_viewer->m_changeView(view);
    });

    auto* buttonSideView = new nanogui::Button(subgroup11, "Side View");
    buttonSideView->set_fixed_width(74);
    buttonSideView->set_callback([&] {
        m_viewer->m_changeView(grs::Matrix4d::identity());
    });

    auto* buttonIsometricView = new nanogui::Button(subgroup11, "Iso View");
    buttonIsometricView->set_fixed_width(73);
    buttonIsometricView->set_callback([&] {

        constexpr double a = std::sqrt(2.0) / 2.0;
        constexpr double b = std::sqrt(2.0) / std::sqrt(3.0);
        constexpr double c = 1.0 / std::sqrt(3.0);

        const grs::Matrix4d view
            {a, -b/2.0,  c,  0,
             0,      b,  c,  0,
             -a, -b/2.0,  c,  0,
             0,      0,  0,  1};

        m_viewer->m_changeView(view);
    });

    auto* subgroup12 = new nanogui::Widget(group2);
    subgroup12->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 5));

    auto* buttonLaunch = new nanogui::Button(subgroup12, "Trigger Launch");
    buttonLaunch->set_fixed_width(230);
    buttonLaunch->set_callback([&] {
        m_viewer->m_launchTriggered();
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

    auto* buttonGroundspeedMinus = new nanogui::ToolButton(subgroup1, FA_MINUS);
    buttonGroundspeedMinus->set_flags(nanogui::Button::NormalButton);
    buttonGroundspeedMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_groundspeedTarget -= 0.5f;
            m_groundspeedTargetText->set_value(doubleToString(m_groundspeedTarget,1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonGroundspeedPlus = new nanogui::ToolButton(subgroup1, FA_PLUS);
    buttonGroundspeedPlus->set_flags(nanogui::Button::NormalButton);
    buttonGroundspeedPlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_groundspeedTarget += 0.5f;
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

    auto* buttonAltitudeMinus = new nanogui::ToolButton(subgroup2, FA_MINUS);
    buttonAltitudeMinus->set_flags(nanogui::Button::NormalButton);
    buttonAltitudeMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[2] -= 0.5;
            m_altitudeTargetText->set_value(doubleToString(m_payloadTarget[2], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonAltitudePlus = new nanogui::ToolButton(subgroup2, FA_PLUS);
    buttonAltitudePlus->set_flags(nanogui::Button::NormalButton);
    buttonAltitudePlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[2] += 0.5;
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

    auto* buttonPositionXMinus = new nanogui::ToolButton(subgroup3, FA_MINUS);
    buttonPositionXMinus->set_flags(nanogui::Button::NormalButton);
    buttonPositionXMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[0] -= 0.1;
            m_positionXTargetText->set_value(doubleToString(m_payloadTarget[0], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonPositionXPlus = new nanogui::ToolButton(subgroup3, FA_PLUS);
    buttonPositionXPlus->set_flags(nanogui::Button::NormalButton);
    buttonPositionXPlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[0] += 0.1;
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

    auto* buttonPositionYMinus = new nanogui::ToolButton(subgroup4, FA_MINUS);
    buttonPositionYMinus->set_flags(nanogui::Button::NormalButton);
    buttonPositionYMinus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[1] -= 0.1;
            m_positionYTargetText->set_value(doubleToString(m_payloadTarget[1], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });

    auto* buttonPositionYPlus = new nanogui::ToolButton(subgroup4, FA_PLUS);
    buttonPositionYPlus->set_flags(nanogui::Button::NormalButton);
    buttonPositionYPlus->set_callback([&] {
        if (m_hasConfigFile) {
            m_payloadTarget[1] += 0.1;
            m_positionYTargetText->set_value(doubleToString(m_payloadTarget[1], 1));
            m_viewer->m_updateTarget(m_groundspeedTarget, m_payloadTarget);
        }
    });
}

void Gui::m_createDebugPanel() {
    m_debugPanel = new nanogui::Window(m_screen, "Debug Options");
    m_debugPanel->set_position(nanogui::Vector2i(15, 525));
    m_debugPanel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 10, 5));
    m_debugPanel->set_fixed_width(370);

    // Group 1 Visual Objects
    new nanogui::Label(m_debugPanel, std::string("Visual Objects :"),"sans-bold",20);
    auto* group1 = new nanogui::Widget(m_debugPanel);
    group1->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,nanogui::Alignment::Fill, 5, 5));
    auto* buttonFrames = new nanogui::Button(group1, "Show Reference Frames");
    buttonFrames->set_callback([&] {
        m_shouldDrawReferenceFrame = !m_shouldDrawReferenceFrame;
        m_viewer->m_drawReferenceFrame(m_shouldDrawReferenceFrame);
    });

    auto* buttonForces = new nanogui::Button(group1, "Show Forces");
    buttonForces->set_callback([&] {
        m_shouldDrawForcesVector = !m_shouldDrawForcesVector;
        m_viewer->m_drawForcesVector(m_shouldDrawForcesVector);
    });


    // Group 2 Numeric Infos
    new nanogui::Label(m_debugPanel, std::string("Numeric Infos :"),"sans-bold",20);
    auto* group2 = new nanogui::Widget(m_debugPanel);
    group2->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,nanogui::Alignment::Fill, 5, 5));

    m_comboBoxFrames = new nanogui::ComboBox(group2);
    m_comboBoxFrames->set_font_size(20);
    m_comboBoxFrames->set_callback([&](const int index) {
        m_currentFrameDisplayed = m_frames[index];
    });

    m_framePositionDisplayed    = new nanogui::Label(group2, std::string("Position    (m)   : "), "mono", 15);
    m_frameVelocityDisplayed    = new nanogui::Label(group2, std::string("Velocity    (m/s) : "), "mono", 15);
    m_frameOrientationDisplayed = new nanogui::Label(group2, std::string("Orientation (Deg) : "), "mono", 15);
    m_frameForcesDisplayed      = new nanogui::Label(group2, std::string("Forces      (N)   : "), "mono", 15);
}

std::string Gui::doubleToString(const double value, const int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

void Gui::setTargetFromConfig(const float groundspeed, const std::vector<double>& positionTarget) {
    m_groundspeedTarget = groundspeed;
    m_groundspeedTargetText->set_value(doubleToString(m_groundspeedTarget,1));

    m_payloadTarget = positionTarget;
    m_altitudeTargetText->set_value(doubleToString(m_payloadTarget[2],1));
    m_positionXTargetText->set_value(doubleToString(m_payloadTarget[0],1));
    m_positionYTargetText->set_value(doubleToString(m_payloadTarget[1],1));
}

void Gui::setFrames(const std::vector<Frame *> &frames) {
    m_frames = frames;

    std::vector<std::string> names;

    for (const auto& f : m_frames) {
        names.push_back(f->getName());
    }

    m_comboBoxFrames->set_items(names);
    m_screen->perform_layout();
}
