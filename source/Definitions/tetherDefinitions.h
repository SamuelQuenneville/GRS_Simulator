/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TETHERDEFINITIONS_H
#define GRS_SIMULATOR_TETHERDEFINITIONS_H

#pragma once

#include "Mathematics/vector3.h"

/*
 *   TETHERS STRUCTURES
 */

struct segmentProperties {
    std::string name;
    float linearMass;  // kg/m
    float length;      // m
    float diameter;    // m
    float rigidity;    // spring model k value
    float damping;     // spring model b value
    int   numSegments; // number of segments use to compute aerodynamic forces
};

struct tethersProperties {
    std::vector<segmentProperties> segments;

    segmentProperties& byName(const std::string& name) {
        for(auto& seg : segments) {
            if(seg.name == name) return seg;
        }
        throw std::runtime_error("Tether segment name not found in tethers properties");
    }

    [[nodiscard]] const segmentProperties& byName(const std::string& name) const {
        for(const auto& seg : segments) {
            if(seg.name == name) return seg;
        }
        throw std::runtime_error("Tether segment name not found in tethers properties");
    }
};

struct tetherLogInfos {
    double deformation{};           // m
    Vector3<double> forceSpring;    // N
    Vector3<double> forceAero;      // N
};

#endif //GRS_SIMULATOR_TETHERDEFINITIONS_H
