/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MESH_H
#define GRS_SIMULATOR_MESH_H

#include <vector>
#include <map>
#include "object3D.h"
#include "Mathematics/color.h"
#include "texture2D.h"

class Mesh : public Object3D {

public:
    Mesh() = default;
    ~Mesh() override = default;

    void destroy();

    void calculateNormals();

    [[nodiscard]] unsigned int getNumbersFaces(unsigned int part = 0) const;
    [[nodiscard]] unsigned int getNumbersVertices() const;
    [[nodiscard]] unsigned int getNumbersParts() const;

    [[nodiscard]] const std::vector<Vector3<float>>& getVertices() const;
    void setVertices(std::vector<Vector3<float>>& vertices);

    [[nodiscard]] const std::vector<Vector3<float>>& getNormals() const;
    void setNormals(std::vector<Vector3<float>>& normals);

    [[nodiscard]] const std::vector<Vector2<float>>& getUVs() const;
    void setUVs(std::vector<Vector2<float>>& uvs);

    [[nodiscard]] const std::vector<uint32_t>& getIndices(unsigned int part = 0) const;
    void setIndices(std::vector<std::vector<unsigned int>>& indices);

    [[nodiscard]] const Vector3<float>& getVertex(unsigned int i) const;

    // Return the vertex index of the ith (i=0,1,2) vertex of a given face
    [[nodiscard]] unsigned int getVertexIndexInFace(unsigned int faceIndex, unsigned int i, unsigned int part = 0) const;

    [[nodiscard]] bool hasNormals() const;
    [[nodiscard]] bool hasUVTextureCoordinates() const;
    [[nodiscard]] bool hasTexture() const;

    void* getVerticesPointer();
    void* getNormalsPointer();
    void* getUVTextureCoordinatesPointer();
    void* getIndicesPointer(unsigned int part = 0);

protected:
    // A triplet of vertex indices for each triangle
    std::vector<std::vector<unsigned int>> m_indices;

    // Vertices coordinates (local space)
    std::vector<Vector3<float>> m_vertices;

    // Normals coordinates
    std::vector<Vector3<float>> m_normals;

    // Color for each vertex
    std::vector<Color> m_colors;

    // UV texture coordinates
    std::vector<Vector2<float>> m_uvs;

    // Textures of the mesh (one for each part of the mesh)
    std::map<uint32_t, Texture2D> m_textures;
};


#endif //GRS_SIMULATOR_MESH_H
