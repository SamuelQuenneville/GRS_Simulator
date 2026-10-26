/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "shader.h"

Shader::Shader()
    : m_programObjectId(0)
{

}

Shader::Shader(const std::string& filenameVertexShader, const std::string& filenameFragmentShader)
    : Shader()
{
    create(filenameVertexShader, filenameFragmentShader);
}

Shader::~Shader() {
    destroy();
}

bool Shader::create(const std::string& filenameVertexShader, const std::string& filenameFragmentShader) {
    // Set the shader filenames
    m_filenameVertexShader = filenameVertexShader;
    m_filenameFragmentShader = filenameFragmentShader;

    // Delete the current shader
    destroy();

    assert(!filenameVertexShader.empty() && !filenameFragmentShader.empty());

    // ------------------- Load the vertex shader ------------------- //
    GLuint vertexShaderID;
    std::ifstream fileVertexShader;
    fileVertexShader.open(filenameVertexShader.c_str(), std::ios::binary);

    if (fileVertexShader.is_open()) {

        // Get the size of the file
        fileVertexShader.seekg(0, std::ios::end);
        uint32_t fileSize = static_cast<uint32_t>(fileVertexShader.tellg());
        assert(fileSize != 0);

        // Read the file
        fileVertexShader.seekg(std::ios::beg);
        auto* bufferVertexShader = new GLchar[fileSize + 1];
        fileVertexShader.read(bufferVertexShader, fileSize);
        fileVertexShader.close();
        bufferVertexShader[fileSize] = '\0';

        // Create the OpenGL vertex shader and compile it
        vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
        assert(vertexShaderID != 0);
        glShaderSource(vertexShaderID, 1, &bufferVertexShader, nullptr);
        glCompileShader(vertexShaderID);
        delete[] bufferVertexShader;

        // Get the compilation information
        GLint compiled;
        glGetShaderiv(vertexShaderID, GL_COMPILE_STATUS, &compiled);

        // If the compilation failed
        if (compiled == 0) {

            // Get the log of the compilation
            GLint lengthLog;
            glGetShaderiv(vertexShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
            auto* str = new GLchar[lengthLog];
            glGetShaderInfoLog(vertexShaderID, lengthLog, nullptr, str);

            // Display the log of the compilation
            std::cerr << "Vertex Shader Error (in " << filenameVertexShader << ") : " << str << std::endl;
            delete[] str;
            assert(false);
        }
    }
    else {
        std::cerr << "Error : Impossible to open the vertex shader file " <<
                  filenameVertexShader << std::endl;
        assert(false);
    }

    // ------------------- Load the fragment shader ------------------- //
    GLuint fragmentShaderID;
    std::ifstream fileFragmentShader;
    fileFragmentShader.open(filenameFragmentShader.c_str(), std::ios::binary);

    if (fileFragmentShader.is_open()) {

        // Get the size of the file
        fileFragmentShader.seekg(0, std::ios::end);
        uint32_t fileSize = static_cast<uint32_t>(fileFragmentShader.tellg());
        assert(fileSize != 0);

        // Read the file
        fileFragmentShader.seekg(std::ios::beg);
        auto* bufferFragmentShader = new GLchar[fileSize + 1];
        fileFragmentShader.read(bufferFragmentShader, fileSize);
        fileFragmentShader.close();
        bufferFragmentShader[fileSize] = '\0';

        // Create the OpenGL fragment shader and compile it
        fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
        assert(fragmentShaderID != 0);
        glShaderSource(fragmentShaderID, 1, &bufferFragmentShader, nullptr);
        glCompileShader(fragmentShaderID);
        delete[] bufferFragmentShader;

        // Get the compilation information
        GLint compiled;
        glGetShaderiv(fragmentShaderID, GL_COMPILE_STATUS, &compiled);

        // If the compilation failed
        if (compiled == 0) {

            // Get the log of the compilation
            GLint lengthLog;
            glGetShaderiv(fragmentShaderID, GL_INFO_LOG_LENGTH, &lengthLog);
            auto* str = new GLchar[lengthLog];
            glGetShaderInfoLog(fragmentShaderID, lengthLog, nullptr, str);

            // Display the log of the compilation
            std::cerr << "Fragment Shader Error (in " << filenameFragmentShader << ") : " << str << std::endl;
            delete[] str;
            assert(false);
        }
    }
    else {
        std::cerr << "Error : Impossible to open the fragment shader file " << filenameFragmentShader << std::endl;
        assert(false);
    }

    // Create the shader program and attach the shaders
    m_programObjectId = glCreateProgram();
    assert(m_programObjectId);
    glAttachShader(m_programObjectId, vertexShaderID);
    glAttachShader(m_programObjectId, fragmentShaderID);
    glDeleteShader(vertexShaderID);
    glDeleteShader(fragmentShaderID);

    // Try to link the program
    glLinkProgram(m_programObjectId);
    GLint linked;
    glGetProgramiv(m_programObjectId, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint logLength;
        glGetProgramiv(m_programObjectId, GL_INFO_LOG_LENGTH, &logLength);
        auto* strLog = new GLchar[logLength];
        glGetProgramInfoLog(m_programObjectId, logLength, nullptr, strLog);
        std::cerr << "Linker Error in vertex shader " << filenameVertexShader <<
                  " or in fragment shader " << filenameFragmentShader << " : " << strLog << std::endl;
        delete[] strLog;
        destroy();
        assert(false);
    }

    return true;
}

void Shader::destroy() {
    m_uniformLocations.clear();
    if (m_programObjectId != 0) {
        glDeleteProgram(m_programObjectId);
        m_programObjectId = 0;
    }
}

void Shader::bind() const {
    assert(m_programObjectId != 0);
    glUseProgram(m_programObjectId);
}

void Shader::unbind() const {
    assert(m_programObjectId != 0);
    glUseProgram(0);
}

GLint Shader::getUniformLocation(const std::string& variableName, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);

    if (const auto it = m_uniformLocations.find(variableName); it != m_uniformLocations.end()) {
        return it->second;
    }

    const GLint location = glGetUniformLocation(m_programObjectId, variableName.c_str());
    m_uniformLocations.emplace(variableName, location);
    if (location == -1 && errorIfMissing) {
        std::cerr << "Error in vertex shader " << m_filenameFragmentShader << " or in fragment shader"
                  << m_filenameFragmentShader << " : No Uniform variable : " << variableName << std::endl;
    }

    return location;
}

GLint Shader::getAttribLocation(const std::string& variableName, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    const GLint location = glGetAttribLocation(m_programObjectId, variableName.c_str());
    if (location == -1 && errorIfMissing) {
        std::cerr << "Error in vertex shader " << m_filenameVertexShader << " or in fragment shader"
                  << m_filenameFragmentShader << " : No variable : " << variableName << std::endl;
        throw std::logic_error("Error in Shader");
    }

    return location;
}

void Shader::setIntUniform(const std::string& variableName, const int value, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform1i(location, value);
    }
}

void Shader::setFloatUniform(const std::string& variableName, const float value, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform1f(location, value);
    }
}

void Shader::setVector2Uniform(const std::string& variableName, const grs::Vec2f& v, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform2f(location, v[0], v[1]);
    }
}

void Shader::setVector3Uniform(const std::string& variableName, const grs::Vec3f& v, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform3f(location, v[0], v[1], v[2]);
    }
}

void Shader::setVector4Uniform(const std::string& variableName, const grs::Vec4f& v, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniform4f(location, v[0], v[1], v[2], v[3]);
    }
}

void Shader::setMatrix3x3Uniform(const std::string& variableName, const grs::Matrix3d& matrix, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    GLfloat mat[9];
    for (int i = 0; i < 3 ; i++) {
        for (int j = 0; j < 3 ; j++) {
            mat[i*3 + j] = static_cast<GLfloat>(matrix(i, j));
        }
    }
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniformMatrix3fv(location, 1, GL_TRUE, mat);
    }
}

void Shader::setMatrix4x4Uniform(const std::string& variableName, const grs::Matrix4d& matrix, const bool errorIfMissing) const {
    assert(m_programObjectId != 0);
    GLfloat mat[16];
    for (int i = 0; i < 4 ; i++) {
        for (int j = 0; j < 4; j++) {
            mat[i*4 + j] = static_cast<GLfloat>(matrix(i,j));
        }
    }
    const GLint location = getUniformLocation(variableName, errorIfMissing);
    if (location != -1) {
        glUniformMatrix4fv(location, 1, GL_TRUE, mat);
    }
}
