/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "light.h"

Light::Light()
    : m_diffuseColor(Color::white())
    , m_isActive(false)
{

}

void Light::init() {
    enable();
}

Color Light::getDiffuseColor() const {
    return m_diffuseColor;
}

void Light::setDiffuseColor(const Color& color) {
    m_diffuseColor = color;
}

void Light::enable() {
    m_isActive = true;
}

void Light::disable() {
    m_isActive = false;
}