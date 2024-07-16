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
    Texture2D(uint32_t width, uint32_t height, uint32_t internalFormat, uint32_t format, uint32_t type);
    ~Texture2D();

    // Create the texture
    void create(uint32_t width, uint32_t height, uint32_t internalFormat, uint32_t format, uint32_t type, void* data = nullptr);

    void create(uint32_t width, uint32_t height, uint32_t internalFormat, uint32_t format, uint32_t type,
                uint32_t minFilter, uint32_t maxFilter, uint32_t wrapS, uint32_t wrapT, void* data);

    void destroy();

    void bind() const;
    void unbind() const;

    // Get the OpenGL texture ID
    [[nodiscard]] uint32_t getID() const;

    [[nodiscard]] uint32_t getUnit() const;
    void setUnit(uint32_t unit);

    [[nodiscard]] uint32_t getWidth() const;
    [[nodiscard]] uint32_t getHeight() const;

private:
    GLuint m_id;
    GLuint m_unit;

    uint32_t m_width;
    uint32_t m_height;
};


#endif //GRS_SIMULATOR_TEXTURE2D_H
