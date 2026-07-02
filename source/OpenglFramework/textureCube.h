/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_TEXTURECUBE_H
#define GRS_SIMULATOR_TEXTURECUBE_H

#include <string>
#include <vector>
#include <nanogui/opengl.h>

class TextureCube {
public:
    explicit TextureCube(const std::vector<std::string>& faces);
    explicit TextureCube(const std::string& path);
    ~TextureCube();

    void bind(GLuint unit = 0) const;

private:
    GLuint m_id = 0;
};

#endif //GRS_SIMULATOR_TEXTURECUBE_H
