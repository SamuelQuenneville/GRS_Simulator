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
#include "Definitions/configurationDefinitions.h"
#include "Definitions/fixedWingDefinitions.h"
#include "Definitions/tetherDefinitions.h"
#include "Definitions/payloadDefinitions.h"
#include "Definitions/controllerGains.h"

class ConfigurationParser {

public:
    ConfigurationParser() = default;
    ~ConfigurationParser() = default;

    static simulationParameters parseSimulationParameters(YAML::Node& node);
    static fixedWingProperties parseFixedWingProperties(const std::string& filePath);
    static tetherProperties parseTetherProperties(const std::string& filePath);
    static payloadProperties parsePayloadProperties(const std::string& filePath);
    static ctrlGains parseControllerGains(const std::string& filePath);
    static initialStates parseInitialStates(YAML::Node& node, const std::string& nodeName);
    static target parseSystemTarget(YAML::Node& node);
};


#endif //GRS_SIMULATOR_CONFIGURATIONPARSER_H
