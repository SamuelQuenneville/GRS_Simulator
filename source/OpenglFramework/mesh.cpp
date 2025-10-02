/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "mesh.h"

void Mesh::destroy() {
    m_vertices.clear();
    m_normals.clear();
    m_indices.clear();
    m_colors.clear();
    m_uvs.clear();
    m_textures.clear();
}

void Mesh::calculateNormals() {
    m_normals = std::vector<grs::Vec3f>(getNumbersVertices(), grs::Vec3f::zeros());

    // For each triangular face
    for (int i=0; i < getNumbersFaces(); i++) {

        // Get the three vertices index of the current face
        const unsigned int v1 = getVertexIndexInFace(i, 0);
        const unsigned int v2 = getVertexIndexInFace(i, 1);
        const unsigned int v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNumbersVertices());
        assert(v2 < getNumbersVertices());
        assert(v3 < getNumbersVertices());

        // Compute the normal of the face
        grs::Vec3f p = getVertex(v1);
        grs::Vec3f q = getVertex(v2);
        grs::Vec3f r = getVertex(v3);
        const grs::Vec3f normal = grs::normalize(grs::cross(q - p,r - p));

        // Add the face surface normal to the sum of normals at
        // each vertex of the face
        m_normals[v1] += normal;
        m_normals[v2] += normal;
        m_normals[v3] += normal;
    }

    // Normalize the normal at each vertex
    for (int i=0; i < getNumbersVertices(); i++) {
        m_normals[i] = grs::normalize(m_normals[i]);
    }
}

unsigned int Mesh::getNumbersFaces(const unsigned int part) const {
    return m_indices[part].size() / 3;
}

unsigned int Mesh::getNumbersVertices() const {
    return m_vertices.size();
}

unsigned int Mesh::getNumbersParts() const {
    return m_indices.size();
}

const std::vector<grs::Vec3f>& Mesh::getVertices() const {
    return m_vertices;
}

void Mesh::setVertices(const std::vector<grs::Vec3f>& vertices) {
    m_vertices = vertices;
}

const std::vector<grs::Vec3f>& Mesh::getNormals() const {
    return m_normals;
}

void Mesh::setNormals(const std::vector<grs::Vec3f>& normals) {
    m_normals = normals;
}

const std::vector<grs::Vec2f>& Mesh::getUVs() const {
    return m_uvs;
}

void Mesh::setUVs(const std::vector<grs::Vec2f>& uvs) {
    m_uvs = uvs;
}

const std::vector<uint32_t>& Mesh::getIndices(const unsigned int part) const {
    return m_indices[part];
}

void Mesh::setIndices(const std::vector<std::vector<unsigned int>>& indices) {
    m_indices = indices;
}

const grs::Vec3f& Mesh::getVertex(const unsigned int i) const {
    assert(i < getNumbersVertices());
    return m_vertices[i];
}

unsigned int Mesh::getVertexIndexInFace(const unsigned int faceIndex, const unsigned int i, const unsigned int part) const {
    return (m_indices[part])[faceIndex*3 + i];
}

bool Mesh::hasNormals() const {
    return m_normals.size() == m_vertices.size();
}

bool Mesh::hasUVTextureCoordinates() const {
    return m_uvs.size() == m_vertices.size();
}

bool Mesh::hasTexture() const {
    return hasUVTextureCoordinates() && (!m_textures.empty());
}

void* Mesh::getVerticesPointer() {
    return &(m_vertices[0]);
}

void* Mesh::getNormalsPointer() {
    return &(m_normals[0]);
}

void* Mesh::getUVTextureCoordinatesPointer() {
    return &(m_uvs[0]);
}

void* Mesh::getIndicesPointer(const unsigned int part) {
    return &(m_indices[part])[0];
}