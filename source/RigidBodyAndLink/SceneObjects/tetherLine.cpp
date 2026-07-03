/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "tetherLine.h"

TetherLine::TetherLine(const tetherRenderData& tetherRenderData)
    : m_cylinder(0.02f, 1.0f, 16)
{
    m_tetherRenderData = tetherRenderData;
}

void TetherLine::render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix) const {
    m_cylinder.render(shader,m_modelMatrix, worldToCameraMatrix);
}

void TetherLine::update(const tetherRenderData& renderData) {
    m_tetherRenderData = renderData;

    const auto p1 = m_tetherRenderData.startPosition.toOpenglRef();
    const auto p2 = m_tetherRenderData.endPosition.toOpenglRef();

    const auto d = p2 - p1;
    const double length = d.norm();

    const auto T = grs::makeTranslation(p1);
    const auto R = grs::makeRotationFromZ(d);
    const auto S = grs::makeScale(grs::Vec3d(1.0, 1.0, length));

    m_modelMatrix = T * R * S;
}
