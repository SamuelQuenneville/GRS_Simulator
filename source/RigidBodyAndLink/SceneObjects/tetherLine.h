/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TETHERLINE_H
#define GRS_SIMULATOR_TETHERLINE_H

#include "cylinder.h"
#include "Definitions/renderObjectDefinitions.h"

class TetherLine {

public:
    explicit TetherLine(const tetherRenderData& tetherRenderData);
    ~TetherLine() = default;

    void update(const tetherRenderData& renderData);
    void render(const Shader& shader, const grs::Matrix4d& worldToCameraMatrix) const;

private:
    grs::Matrix4d m_modelMatrix = grs::Matrix4d::identity();

    Cylinder m_cylinder;
    tetherRenderData m_tetherRenderData{};
};

#endif //GRS_SIMULATOR_TETHERLINE_H
