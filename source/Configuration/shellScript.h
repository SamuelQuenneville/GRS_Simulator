/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef SHELLSCRIPT_H
#define SHELLSCRIPT_H

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include "Definitions/configurationDefinitions.h"

class ShellScript {

public:
    ShellScript() = default;
    ~ShellScript() = default;

    static void createShellScript(const std::string& fileName, int numberOfUav, const std::vector<initialStates>& initialStates);

private:
    static void m_writeHeader(std::ofstream& file);
    static void m_writeCheckBinaryPath(std::ofstream& file, const std::string& binaryPath);
    static void m_writeVehicleCreation(std::ofstream& file, int numberOfUav, const std::vector<initialStates>& initialStates);
};

#endif //SHELLSCRIPT_H
