/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_MESHREADER_H
#define GRS_SIMULATOR_MESHREADER_H

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <tuple>
#include <algorithm>
#include <memory>

#include "mesh.h"

class MeshReader {

public:
    static void loadMesh(const std::string& filename, Mesh& meshContainer);

    struct meshData {
        std::vector<std::vector<unsigned int>> indices;
        std::vector<Vector3<float>> vertices;
        std::vector<Vector3<float>> normals;
        std::vector<Vector2<float>> uvs;
    };


private:
    MeshReader() = default;
    ~MeshReader() = default;

    // Load .obj file (triangular or quad mesh)
    static void loadObjectFile(const std::string& filename, Mesh& meshContainer);

    static std::unordered_map<std::string, std::shared_ptr<meshData>> meshCache;

};

template<>
struct std::hash<std::tuple<int, int, int>>
{
    size_t operator()(const std::tuple<int, int, int>& key) const noexcept {
        return std::get<0>(key) ^ std::get<1>(key) ^ std::get<2>(key);
    }
};

#endif //GRS_SIMULATOR_MESHREADER_H
