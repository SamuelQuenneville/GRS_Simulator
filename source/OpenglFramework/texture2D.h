/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TEXTURE2D_H
#define GRS_SIMULATOR_TEXTURE2D_H

#include <string>
#include <cassert>
#include <nanogui/opengl.h>

class Texture2D {

public:
    Texture2D();
    Texture2D(GLsizei width, GLsizei height, GLint internalFormat, GLenum format, GLenum type);
    ~Texture2D();

    // Create the texture
    void create(GLsizei width, GLsizei height, GLint internalFormat, GLenum format, GLenum type, const void* data = nullptr);

    void create(GLsizei width, GLsizei height, GLint internalFormat, GLenum format, GLenum type,
                GLint minFilter, GLint maxFilter, GLint wrapS, GLint wrapT, const void *data);

    void destroy();

    void bind() const;
    void unbind() const;

    // Get the OpenGL texture ID
    [[nodiscard]] unsigned int getID() const;

    [[nodiscard]] unsigned int getUnit() const;
    void setUnit(unsigned int unit);

private:
    GLuint m_id;
    GLuint m_unit;

    GLsizei m_width;
    GLsizei m_height;
};


#endif //GRS_SIMULATOR_TEXTURE2D_H
