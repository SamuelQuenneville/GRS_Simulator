/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "configurationParser.h"

simulationParameters ConfigurationParser::parseSimulationParameters(YAML::Node& node) {
    return node["SimulationParameters"].as<simulationParameters>();
}

fixedWingProperties ConfigurationParser::parseFixedWingProperties(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["AircraftProperties"].as<fixedWingProperties>();
}

tetherProperties ConfigurationParser::parseTetherProperties(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["TetherProperties"].as<tetherProperties>();
}

initialStates ConfigurationParser::parseInitialStates(YAML::Node& node, const std::string& nodeName) {
    return node[nodeName].as<initialStates>();
}

target ConfigurationParser::parseSystemTarget(YAML::Node &node) {
    return node["Target"].as<target>();;
}
