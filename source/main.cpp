/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "simulationViewer.h"

int main(const int argc, const char * argv[]) {

    programArguments arguments{false,false};
    PROGRAM_LOGGER.setLogFileName("log.txt");

    // Loop through command-line arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--sitl") {
            arguments.sitlMode = true;
        } else if (arg == "--log") {
            arguments.loggingEnabled = true;
        } else if (arg == "--verbose") {
            PROGRAM_LOGGER.enableVerbose(true);
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --sitl      Enable SITL mode\n"
                      << "  --log       Enable logging\n"
                      << "  --verbose   Enable verbose mode\n"
                      << "  --help      Show this help message\n";
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    LOG_INFO("----- Starting simulation -----");

    SimulationViewer applicationViewer(arguments);
    applicationViewer.start();

    return 0;
}
