/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "tetherLine.h"

TetherLine::TetherLine() {
    setPosition({0,0,0},{0,0,30});
}

Vector3<float> TetherLine::getPoint1() const {
    return m_point1;
}

Vector3<float> TetherLine::getPoint2() const {
    return m_point2;
}

void TetherLine::updateTransform(double interpolationFactor) {

}

void TetherLine::render(Shader &shader, const Matrix4<double> &worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", Matrix4<double>::identity());
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    Vector4<double> color(1, 0, 0, 1);
    shader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    shader.setVector4Uniform("globalVertexColor", color, false);

    // Unbind the shader
    shader.unbind();

}

void TetherLine::setPosition(const Vector3<float>& point1, const Vector3<float>& point2) {
    m_point1 = point1;
    m_point2 = point2;
}

