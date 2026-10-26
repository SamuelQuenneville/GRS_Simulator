/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <thread>

#include "simulationViewer.h"

namespace {

std::atomic<bool> g_stopRequested{false};

void onSignal(int) {
    g_stopRequested.store(true);
}

// Runs the simulation without window, GUI or render snapshots.
// Ends on the SITL endSimulation flags or on Ctrl+C.
int runHeadless(programArguments& args) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    World world(args);
    world.readConfigurationFile(args.configFile);
    world.createSystem();
    world.startPhysicsThread();
    world.start();

    LOG_INFO("Headless run started: " + args.configFile);

    auto nextReport = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!g_stopRequested.load()) {
        if (world.consumeEndOfRun()) {
            LOG_INFO("Reach end of trajectory!");
            break;
        }
        if (std::chrono::steady_clock::now() >= nextReport) {
            LOG_INFO("Sim time : " + std::to_string(world.elapsedTime()) + " s");
            nextReport += std::chrono::seconds(10);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    world.deleteSystem(); // stops the physics thread and flushes the logger
    LOG_INFO("Headless run finished at sim time " + std::to_string(world.elapsedTime()) + " s");
    return 0;
}

} // namespace

int main(const int argc, const char * argv[]) {

    programArguments arguments{.sitlMode = false,.inFlightLaunch = false};
    PROGRAM_LOGGER.setLogFileName("log.txt");

    // Loop through command-line arguments
    for (int i = 1; i < argc; ++i) {
        
        std::string arg = argv[i];

        if (arg == "--sitl") {
            arguments.sitlMode = true;
        } else if (arg == "--csv") {
            arguments.inFlightLaunch = true;
        }else if (arg == "--log") {
            arguments.loggingEnabled = true;
        } else if (arg == "--headless" && i + 1 < argc) {
            arguments.headless = true;
            arguments.configFile = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            arguments.configFile = argv[++i];
        } else if (arg == "--msaa" && i + 1 < argc) {
            arguments.msaaSamples = std::clamp(std::stoi(argv[++i]), 0, 16);
        } else if (arg == "--fps" && i + 1 < argc) {
            arguments.renderFps = std::max(1.0, std::stod(argv[++i]));
        } else if (arg == "--verbose") {
            PROGRAM_LOGGER.enableVerbose(true);
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --sitl      Enable SITL mode\n"
                      << "  --csv       Use with sitl to enable commands from .csv\n"
                      << "  --log       Enable logging\n"
                      << "  --fps N     Viewer frame rate cap in Hz (default 60)\n"
                      << "  --msaa N    Viewer anti-aliasing samples, 0 to disable (default 4)\n"
                      << "  --config FILE    Load FILE (.yaml) in the viewer at startup\n"
                      << "  --headless FILE  Run FILE (.yaml) without the viewer\n"
                      << "  --verbose   Enable verbose mode\n"
                      << "  --help      Show this help message\n";
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    LOG_INFO("----- Starting simulation -----");

    if (arguments.headless) {
        return runHeadless(arguments);
    }

    SimulationViewer applicationViewer(arguments);
    applicationViewer.start();

    return 0;
}
