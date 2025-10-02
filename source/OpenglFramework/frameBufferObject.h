/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FRAMEBUFFEROBJECT_H
#define GRS_SIMULATOR_FRAMEBUFFEROBJECT_H

#include <nanogui/opengl.h>
#include <iostream>

class FrameBufferObject {

public:
    FrameBufferObject();
    ~FrameBufferObject();

    bool create(GLsizei width, GLsizei height, bool needRenderBuffer = true);

    void attachTexture(GLenum position, GLuint textureId) const;

    void bind() const;
    void unbind() const;

    void destroy();

private:
    GLuint m_frameBufferId;
    GLuint m_renderBufferId;
};


#endif //GRS_SIMULATOR_FRAMEBUFFEROBJECT_H
