/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TETHERLINE_H
#define GRS_SIMULATOR_TETHERLINE_H

#include "physicsObject.h"
#include "OpenglFramework/shader.h"


class TetherLine : public PhysicsObject {

public:
    TetherLine();
    ~TetherLine() override = default;

    [[nodiscard]] Vector3<float> getPoint1() const;
    [[nodiscard]] Vector3<float> getPoint2() const;

    void setPosition(const Vector3<float>& point1, const Vector3<float>& point2);

    void updateTransform(double interpolationFactor) override;
    void render(Shader& shader, const Matrix4<double>& worldToCameraMatrix) override;

private:
    Vector3<float> m_point1;
    Vector3<float> m_point2;
};


#endif //GRS_SIMULATOR_TETHERLINE_H
