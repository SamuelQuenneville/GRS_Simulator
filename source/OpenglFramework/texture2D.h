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
    Texture2D(unsigned int width, unsigned int height, unsigned int internalFormat, unsigned int format, unsigned int type);
    ~Texture2D();

    // Create the texture
    void create(unsigned int width, unsigned int height, unsigned int internalFormat, unsigned int format, unsigned int type, void* data = nullptr);

    void create(unsigned int width, unsigned int height, unsigned int internalFormat, unsigned int format, unsigned int type,
                unsigned int minFilter, unsigned int maxFilter, unsigned int wrapS, unsigned int wrapT, void* data);

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

    unsigned int m_width;
    unsigned int m_height;
};


#endif //GRS_SIMULATOR_TEXTURE2D_H
