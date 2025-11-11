/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LOGDEFINITIONS_H
#define GRS_SIMULATOR_LOGDEFINITIONS_H

#pragma once

enum class LogType {
    Aircraft,
    Payload,
    Tether,
    Frame,
    Node,
    SitlIn,
    SitlOut,
    HLC,
    LLC,
    Event
};

struct LogItem {
    LogType type;
    std::string line;
};

#endif //GRS_SIMULATOR_LOGDEFINITIONS_H
