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

systemConfiguration ConfigurationParser::parseSystemConfiguration(YAML::Node &node) {
    return node["SystemConfiguration"].as<systemConfiguration>();
}

fixedWingProperties ConfigurationParser::parseFixedWingProperties(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["AircraftProperties"].as<fixedWingProperties>();
}

tetherProperties ConfigurationParser::parseTetherProperties(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["TetherProperties"].as<tetherProperties>();
}

payloadProperties ConfigurationParser::parsePayloadProperties(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["PayloadProperties"].as<payloadProperties>();
}

launcherProperties ConfigurationParser::parseLauncherProperties(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["LauncherProperties"].as<launcherProperties>();
}

ctrlGains ConfigurationParser::parseControllerGains(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);

    ctrlGains gains{};
    gains.hlc.phaseCtrlGains       = node["PhaseController"].as<phaseControllerGains>();
    gains.hlc.lateralCtrlGains     = node["LateralController"].as<lateralControllerGains>();
    gains.hlc.groundspeedCtrlGains = node["GroundspeedController"].as<groundspeedControllerGains>();

    gains.llc.altitudeCtrlGains = node["AltitudeController"].as<altitudeControllerGains>();
    gains.llc.pitchCtrlGains    = node["PitchController"].as<pitchControllerGains>();
    gains.llc.rollCtrlGains     = node["RollController"].as<rollControllerGains>();

    return gains;
}

initialStates ConfigurationParser::parseInitialStates(YAML::Node& node, const std::string& nodeName) {
    if (node[nodeName]) {
        return node[nodeName].as<initialStates>();
    } else {
        throw std::invalid_argument( "Mismatch between number of UAV and number of initial states nodes" );
    }
}

target ConfigurationParser::parseSystemTarget(YAML::Node& node) {
    return node["Target"].as<target>();
}

std::vector<float> ConfigurationParser::parseWind(const std::string& filePath) {
    YAML::Node node = YAML::LoadFile(filePath);
    return node["Wind"].as<std::vector<float>>();
}
