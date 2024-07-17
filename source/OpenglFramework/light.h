/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LIGHT_H
#define GRS_SIMULATOR_LIGHT_H

#include "object3D.h"
#include "Mathematics/color.h"

class Light : public Object3D {

public:
    Light();
    ~Light() override = default;

    [[nodiscard]] Color getDiffuseColor() const;
    void setDiffuseColor(const Color& color);

    void init();
    void enable();
    void disable();

private:
    Color m_diffuseColor;
    bool m_isActive;
};


#endif //GRS_SIMULATOR_LIGHT_H
