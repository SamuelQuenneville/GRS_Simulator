/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FILTER_H
#define GRS_SIMULATOR_FILTER_H

#pragma once

#include <cinttypes>

template <class T>
class Filter
{
public:
    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample) = 0;

    // reset - clear the filter
    virtual void reset()  = 0;

};

// Typedef for convenience
typedef Filter<int8_t> FilterInt8;
typedef Filter<uint8_t> FilterUInt8;
typedef Filter<int16_t> FilterInt16;
typedef Filter<uint16_t> FilterUInt16;
typedef Filter<int32_t> FilterInt32;
typedef Filter<uint32_t> FilterUInt32;


#endif //GRS_SIMULATOR_FILTER_H
