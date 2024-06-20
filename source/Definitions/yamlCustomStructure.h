/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_YAMLCUSTOMSTRUCTURE_H
#define GRS_SIMULATOR_YAMLCUSTOMSTRUCTURE_H

#pragma once

#include "yaml-cpp/node/node.h"

#include "tetherDefinitions.h"
#include "fixedWingDefinitions.h"
#include "configurationDefinitions.h"

/*
 *   YAML template for custom struct
 */

namespace YAML {

    template<>
    struct convert<simulationParameters> {
        static bool decode(const Node& node, simulationParameters& rhs) {
            if(!node.IsMap()) {
                return false;
            }

            rhs.logFilesDirectory               = node["LOG_FILE_DIRECTORY"].as<std::string>();
            rhs.aircraftPropertiesFileDirectory = node["AIRCRAFT_PROPERTIES_FILE_NAME"].as<std::string>();
            rhs.tetherPropertiesFileDirectory   = node["TETHER_PROPERTIES_FILE_NAME"].as<std::string>();
            rhs.systemConfiguration             = node["SYSTEM_CONFIGURATION"].as<std::string>();
            rhs.fixedDuration                   = node["FIXED_DURATION"].as<bool>();
            rhs.duration                        = node["DURATION"].as<float>();
            rhs.hlcRate                         = node["HLC_RATE"].as<int>();
            rhs.llcRate                         = node["LLC_RATE"].as<int>();
            rhs.windOn                          = node["WIND_ON"].as<bool>();
            rhs.tetherAeroOn                    = node["TETHER_AERO_ON"].as<bool>();
            return true;
        }
    };

    template<>
    struct convert<initialStates> {
        static bool decode(const Node& node, initialStates& rhs) {
            if(!node.IsMap()) {
                return false;
            }

            rhs.position    = node["POSITION"].as<std::vector<double>>();
            rhs.speed       = node["SPEED"].as<std::vector<double>>();
            rhs.orientation = node["ORIENTATION"].as<std::vector<double>>();
            return true;
        }
    };

    template<>
    struct convert<target> {
        static bool decode(const Node& node, target& rhs) {
            if(!node.IsMap()) {
                return false;
            }

            rhs.positionPayload = node["POS_PAYLOAD"].as<std::vector<double>>();
            rhs.groundspeed     = node["GROUNDSPEED"].as<float>();
            rhs.aircraftRoll    = node["AIRCRAFT_ROLL"].as<float>();
            return true;
        }
    };

    template<>
    struct convert<tetherProperties> {
        static bool decode(const Node& node, tetherProperties& rhs) {
            if(!node.IsMap()) {
                return false;
            }

            rhs.linearMass   = node["LINEAR_MASS"].as<float>();
            rhs.length       = node["LENGTH"].as<float>();
            rhs.diameter     = node["DIAMETER"].as<float>();
            rhs.rigidity     = node["RIGIDITY"].as<float>();
            rhs.damping      = node["DAMPING"].as<float>();
            rhs.numSegments  = node["NUM_SEGMENTS"].as<int>();
            return true;
        }
    };

    template<>
    struct convert<fixedWingProperties> {
        static bool decode(const Node& node, fixedWingProperties& rhs) {
            if(!node.IsMap()) {
                return false;
            }

            rhs.mass                = node["MASS"].as<double>();
            rhs.wingSurface         = node["WING_SURFACE"].as<double>();
            rhs.wingChord           = node["WING_CHORD"].as<double>();
            rhs.wingSpan            = node["WING_SPAN"].as<double>();
            rhs.aspectRatio         = node["ASPECT_RATIO"].as<double>();
            rhs.oswaldEfficiency    = node["OSWALD_EFF"].as<double>();

            // INERTIA MATRIX
            rhs.inertiaX  = node["INERTIA_X"].as<double>();
            rhs.inertiaY  = node["INERTIA_Y"].as<double>();
            rhs.inertiaZ  = node["INERTIA_Z"].as<double>();
            rhs.inertiaXY = node["INERTIA_XY"].as<double>();
            rhs.inertiaXZ = node["INERTIA_XZ"].as<double>();
            rhs.inertiaYZ = node["INERTIA_YZ"].as<double>();

            // LIMIT CONTROL SURFACES
            rhs.ailMinDeg = node["DELTA_AIL_MIN_DEG"].as<double>();
            rhs.ailMaxDeg = node["DELTA_AIL_MAX_DEG"].as<double>();
            rhs.eleMinDeg = node["DELTA_ELE_MIN_DEG"].as<double>();
            rhs.eleMaxDeg = node["DELTA_ELE_MAX_DEG"].as<double>();
            rhs.rudMinDeg = node["DELTA_RUD_MIN_DEG"].as<double>();
            rhs.rudMaxDeg = node["DELTA_RUD_MAX_DEG"].as<double>();

            // AERO COEFFICIENTS
            // Lift and Drag
            rhs.CL_0     = node["CL_0"].as<double>();
            rhs.CL_ALPHA = node["CL_ALPHA"].as<double>();
            rhs.CD_0     = node["CD_0"].as<double>();
            // Force x direction
            rhs.Cx_q         = node["Cx_q"].as<double>();
            rhs.Cx_DELTA_ELE = node["Cx_DELTA_ELE"].as<double>();
            // Force y direction
            rhs.Cy_0         = node["Cy_0"].as<double>();
            rhs.Cy_BETA      = node["Cy_BETA"].as<double>();
            rhs.Cy_p         = node["Cy_p"].as<double>();
            rhs.Cy_r         = node["Cy_r"].as<double>();
            rhs.Cy_DELTA_AIL = node["Cy_DELTA_AIL"].as<double>();
            rhs.Cy_DELTA_RUD = node["Cy_DELTA_RUD"].as<double>();
            // Force z direction
            rhs.Cz_q         = node["Cz_q"].as<double>();
            rhs.Cz_DELTA_ELE = node["Cz_DELTA_ELE"].as<double>();
            // Moment l
            rhs.Cl_0         = node["Cl_0"].as<double>();
            rhs.Cl_BETA      = node["Cl_BETA"].as<double>();
            rhs.Cl_p         = node["Cl_p"].as<double>();
            rhs.Cl_r         = node["Cl_r"].as<double>();
            rhs.Cl_DELTA_AIL = node["Cl_DELTA_AIL"].as<double>();
            rhs.Cl_DELTA_RUD = node["Cl_DELTA_RUD"].as<double>();
            //Moment m
            rhs.Cm_0         = node["Cm_0"].as<double>();
            rhs.Cm_ALPHA     = node["Cm_ALPHA"].as<double>();
            rhs.Cm_q         = node["Cm_q"].as<double>();
            rhs.Cm_DELTA_ELE = node["Cm_DELTA_ELE"].as<double>();
            // Moment n
            rhs.Cn_0         = node["Cn_0"].as<double>();
            rhs.Cn_BETA      = node["Cn_BETA"].as<double>();
            rhs.Cn_p         = node["Cn_p"].as<double>();
            rhs.Cn_r         = node["Cn_r"].as<double>();
            rhs.Cn_DELTA_AIL = node["Cn_DELTA_AIL"].as<double>();
            rhs.Cn_DELTA_RUD = node["Cn_DELTA_RUD"].as<double>();

            rhs.trimElevator = node["TRIM_ELE"].as<double>();
            rhs.trimAlpha    = node["TRIM_ALPHA"].as<double>();
            return true;
        }
    };
}

#endif //GRS_SIMULATOR_YAMLCUSTOMSTRUCTURE_H
