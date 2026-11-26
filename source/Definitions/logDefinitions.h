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

#include <string>

// Bumped on every change to a log file's columns or meaning; written to
// manifest.yaml so the MATLAB parser picks the matching reader.
//   1: wallTimeMs added (no units line, no manifest)
//   2: units line, events.csv, manifest.yaml, config/ copies, sitlOut column 1
//      renamed jsonTimestampSec, tether length column, payload fixed precision
inline constexpr int kLogSchemaVersion = 2;

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
