/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_SHADER_H
#define GRS_SIMULATOR_SHADER_H

#include <string>
#include <iostream>
#include <stdexcept>
#include <exception>
#include <cassert>
#include <fstream>
#include <nanogui/opengl.h>

#include "Mathematics/matrix3.h"
#include "Mathematics/matrix4.h"

class Shader {

public:
    Shader();
    Shader(const std::string filenameVertexShader, const std::string filenameFragmentShader);
    ~Shader();

    bool create(const std::string filenameVertexShader, const std::string filenameFragmentShader);

    void destroy();

    void bind() const;
    void unbind() const;

    // Return the location of a uniform variable inside a shader program
    [[nodiscard]] GLint getUniformLocation(const std::string& variableName, bool errorIfMissing = true) const;

    // Return the location of an attribute variable inside a shader program
    [[nodiscard]] GLint getAttribLocation(const std::string& variableName, bool errorIfMissing = true) const;

    // Set an int uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try to set it, an assert will occur)
    void setIntUniform(const std::string& variableName, int value, bool errorIfMissing = true) const;

    // Set a vector 2 uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try to set it, an assert will occur)
    void setVector2Uniform(const std::string& variableName, const Vector2<double>& v, bool errorIfMissing = true) const;

    // Set a vector 3 uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try to set it, an assert will occur)
    void setVector3Uniform(const std::string& variableName, const Vector3<double>& v, bool errorIfMissing = true) const;

    // Set a vector 4 uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try to set it, an assert will occur)
    void setVector4Uniform(const std::string& variableName, const Vector4<double>& v, bool errorIfMissing = true) const;

    // Set a 3x3 matrix uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try to set it, an assert will occur)
    void setMatrix3x3Uniform(const std::string& variableName, const Matrix3<double>& matrix, bool errorIfMissing = true) const;

    // Set a 4x4 matrix uniform value to this shader (be careful if the uniform is not
    // used in the shader, the compiler will remove it, then when you will try to set it, an assert will occur)
    void setMatrix4x4Uniform(const std::string& variableName, const Matrix4<double>& matrix, bool errorIfMissing = true) const;

private:
    GLuint m_programObjectId;
    std::string m_filenameVertexShader;
    std::string m_filenameFragmentShader;
};


#endif //GRS_SIMULATOR_SHADER_H
