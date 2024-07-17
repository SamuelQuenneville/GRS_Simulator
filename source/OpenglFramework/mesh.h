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

    [[nodiscard]] uint32_t getNumbersFaces(uint32_t part = 0) const;
    [[nodiscard]] uint32_t getNumbersVertices() const;
    [[nodiscard]] uint32_t getNumbersParts() const;

    [[nodiscard]] const std::vector<Vector3<double>>& getVertices() const;
    void setVertices(std::vector<Vector3<double>>& vertices);

    [[nodiscard]] const std::vector<Vector3<double>>& getNormals() const;
    void setNormals(std::vector<Vector3<double>>& normals);

    [[nodiscard]] const std::vector<Vector2<double>>& getUVs() const;
    void setUVs(std::vector<Vector2<double>>& uvs);

    [[nodiscard]] const std::vector<uint32_t>& getIndices(uint32_t part = 0) const;
    void setIndices(std::vector<std::vector<uint32_t>>& indices);

    [[nodiscard]] const Vector3<double>& getVertex(uint32_t i) const;

    // Return the vertex index of the ith (i=0,1,2) vertex of a given face
    [[nodiscard]] uint32_t getVertexIndexInFace(uint32_t faceIndex, uint32_t i, uint32_t part = 0) const;

    [[nodiscard]] bool hasNormals() const;
    [[nodiscard]] bool hasUVTextureCoordinates() const;
    [[nodiscard]] bool hasTexture() const;

    void* getVerticesPointer();
    void* getNormalsPointer();
    void* getUVTextureCoordinatesPointer();
    void* getIndicesPointer(uint32_t part = 0);

protected:
    // A triplet of vertex indices for each triangle
    std::vector<std::vector<uint32_t>> m_indices;

    // Vertices coordinates (local space)
    std::vector<Vector3<double>> m_vertices;

    // Normals coordinates
    std::vector<Vector3<double>> m_normals;

    // Tangents coordinates
    std::vector<Vector3<double>> m_tangents;

    // Color for each vertex
    std::vector<Color> m_colors;

    // UV texture coordinates
    std::vector<Vector2<double>> m_uvs;

    // Textures of the mesh (one for each part of the mesh)
    std::map<uint32_t, Texture2D> m_textures;
};


#endif //GRS_SIMULATOR_MESH_H
