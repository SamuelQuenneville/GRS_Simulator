/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <Windows.h>
#include "world.h"
#include "simulationViewer.h"

int main() {

    SimulationViewer applicationViewer;
    applicationViewer.start();

    World myWorld;

    // TODO get from user input maybe opengl window in future
    std::string fileName = "C:/Users/ques2902/Desktop/simulationV1.yaml";
    myWorld.readConfigurationFile(fileName);

    myWorld.createSystem();

    while (myWorld.isRunning())
    {

        if (myWorld.simulationConfiguration.fixedDuration) {
            int numSteps = myWorld.simulationConfiguration.duration / myWorld.simulationConfiguration.dt.count();

            std::cout << "Simulation start" << std::endl;
            for (int step = 0; step < numSteps+1; ++step){
                myWorld.update();
            }

            myWorld.stop();
            std::cout << "Simulation ended" << std::endl;

        } else {

            std::cout << "Simulation start --> Press 'S' to stop!" << std::endl;
            bool running = true;

            while (running) {
                myWorld.update();

                if (GetKeyState('S') & 0x8000) {
                    myWorld.stop();
                    running = false;
                    std::cout << "Simulation ended" << std::endl;
                }
            }
        }
    }

    return 0;
}
