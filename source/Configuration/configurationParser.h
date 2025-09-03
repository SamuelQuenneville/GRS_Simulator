/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_CONFIGURATIONPARSER_H
#define GRS_SIMULATOR_CONFIGURATIONPARSER_H

#include <string>

#include "yaml-cpp/yaml.h"

#include "Definitions/yamlCustomStructure.h"

class ConfigurationParser {

public:
    ConfigurationParser() = default;
    ~ConfigurationParser() = default;

    static simulationParameters parseSimulationParameters(YAML::Node& node);
    static systemConfiguration parseSystemConfiguration(YAML::Node& node);
    static fixedWingProperties parseFixedWingProperties(const std::string& filePath);
    static tethersProperties parseTetherProperties(const std::string& filePath);
    static payloadProperties parsePayloadProperties(const std::string& filePath);
    static payloadProperties parseNodeProperties(const std::string& filePath);
    static launcherProperties parseLauncherProperties(const std::string& filePath);
    static ctrlGains parseControllerGains(const std::string& filePath);
    static initialStates parseInitialStates(YAML::Node& node, const std::string& nodeName);
    static target parseSystemTarget(YAML::Node& node);
    static std::vector<float> parseWind(const std::string& filePath);
};


#endif //GRS_SIMULATOR_CONFIGURATIONPARSER_H
