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

    bool create(unsigned int width, unsigned int height, bool needRenderBuffer = true);

    void attachTexture(unsigned int position, unsigned int textureId) const;

    void bind() const;
    void unbind() const;

    void destroy();

private:
    unsigned int m_frameBufferId;
    unsigned int m_renderBufferId;
};


#endif //GRS_SIMULATOR_FRAMEBUFFEROBJECT_H
