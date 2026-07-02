/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <iostream>
#include "textureCube.h"

#include "stb_image.h"

TextureCube::TextureCube(const std::vector<std::string>& faces) {
    glGenTextures(1, &m_id);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_id);

    int width, height, nrChannels;

    for (unsigned int i = 0; i < faces.size(); i++) {
        unsigned char* data = stbi_load(
            faces[i].c_str(),
            &width,
            &height,
            &nrChannels,
            0);

        if (!data) {
            std::cerr << "Failed to load cube map face: " << faces[i] << std::endl;
            continue;
        }

        const GLenum format = (nrChannels == 4) ? GL_RGBA : GL_RGB;

        glTexImage2D(
            GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
            0,
            format,
            width,
            height,
            0,
            format,
            GL_UNSIGNED_BYTE,
            data);

        stbi_image_free(data);
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

TextureCube::TextureCube(const std::string& path) {
    int width, height, channels;
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);

    if (!data) {
        std::cerr << "Failed to load cube map: " << path << std::endl;
        return;
    }

    const int faceW = width / 4;
    const int faceH = height / 3;

    auto extractFace = [&](const int fx, const int fy){
        std::vector<unsigned char> face(faceW * faceH * channels);

        for (int y = 0; y < faceH; y++) {
            for (int x = 0; x < faceW; x++) {
                const int srcX = fx * faceW + x;
                const int srcY = fy * faceH + y;

                const int srcIndex = (srcY * width + srcX) * channels;
                const int dstIndex = (y * faceW + x) * channels;

                for (int c = 0; c < channels; c++) {
                    face[dstIndex + c] = data[srcIndex + c];
                }
            }
        }

        return face;
    };

    struct Face {
        GLenum target;
        int x, y;
    };

    std::vector<Face> faces = {
        {GL_TEXTURE_CUBE_MAP_POSITIVE_X, 2, 1}, // right
        {GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, 1}, // left
        {GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 1, 0}, // top
        {GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 1, 2}, // bottom
        {GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 1, 1}, // front
        {GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 3, 1}  // back
    };

    glGenTextures(1, &m_id);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_id);

    for (const auto&[target, x, y] : faces) {
        auto faceData = extractFace(x, y);

        const GLenum format = (channels == 4) ? GL_RGBA : GL_RGB;

        glTexImage2D(
            target,
            0,
            format,
            faceW,
            faceH,
            0,
            format,
            GL_UNSIGNED_BYTE,
            faceData.data()
        );
    }

    stbi_image_free(data);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

TextureCube::~TextureCube() {
    glDeleteTextures(1, &m_id);
}

void TextureCube::bind(const GLuint unit) const {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_id);
}