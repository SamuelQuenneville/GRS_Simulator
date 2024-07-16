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

    bool create(uint32_t width, uint32_t height, bool needRenderBuffer = true);

    void attachTexture(uint32_t position, uint32_t textureId) const;

    void bind() const;
    void unbind() const;

    void destroy();

private:
    uint32_t m_frameBufferId;
    uint32_t m_renderBufferId;
};


#endif //GRS_SIMULATOR_FRAMEBUFFEROBJECT_H
