/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "meshReader.h"

void MeshReader::loadMesh(const std::string& filename, Mesh& meshContainer) {
    // Get the extension of the file
    unsigned int startPosExtension = filename.find_last_of('.');
    std::string extension = filename.substr(startPosExtension+1);

    // Load the file using the correct method
    if (extension == "obj") {
        loadObjectFile(filename, meshContainer);
    }
    else {
        // Display an error message and throw an exception
        std::string errorMessage("Error : the MeshReaderWriter class cannot load a file with the extension .");
        errorMessage += extension;
        std::cerr << errorMessage << std::endl;
        throw std::invalid_argument(errorMessage.c_str());
    }
}

void MeshReader::loadObjectFile(const std::string& filename, Mesh& meshContainer) {
    // Open the file
    std::ifstream meshFile(filename.c_str());

    // If we cannot open the file
    if(!meshFile.is_open()) {
        // Throw an exception and display an error message
        std::string errorMessage("Error : Cannot open the file " + filename);
        std::cerr << errorMessage << std::endl;
        throw std::runtime_error(errorMessage);
    }

    std::string buffer;
    std::string line, tmp;
    int id1, id2, id3, id4;
    int nId1, nId2, nId3, nId4;
    int tId1, tId2, tId3, tId4;
    float v1, v2, v3;
    size_t found1, found2;
    std::vector<bool> isQuad;
    std::vector<Vector3<float>> vertices;
    std::vector<Vector3<float>> normals;
    std::vector<Vector2<float>> uvs;
    std::vector<unsigned int > verticesIndices;
    std::vector<unsigned int> normalsIndices;
    std::vector<unsigned int> uvsIndices;

    // ---------- Collect the data from the file ---------- //

    // For each line of the file
    while(std::getline(meshFile, buffer)) {

        std::istringstream lineStream(buffer);
        std::string word;
        lineStream >> word;
        std::transform(word.begin(), word.end(), word.begin(), ::tolower);
        if(word == "usemtl") {  // Material definition

            // Loading of MTL file is not implemented

        }
        else if(word == "v") {  // Vertex position
            std::sscanf(buffer.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
            vertices.emplace_back(v1, v2, v3);
        }
        else if(word == "vt") { // Vertex texture coordinate
            std::sscanf(buffer.c_str(), "%*s %f %f", &v1, &v2);
            uvs.emplace_back(v1,v2);
        }
        else if(word == "vn") { // Vertex normal
            std::sscanf(buffer.c_str(), "%*s %f %f %f", &v1, &v2, &v3);
            normals.emplace_back(v1 ,v2, v3);
        }
        else if (word == "f") { // Face
            line = buffer;
            found1 = static_cast<int>(line.find('/'));
            bool isFaceQuad = false;
            found2 = static_cast<int>(line.substr(found1+1).find('/'));

            // If the face definition is of the form "f v1 v2 v3 v4"
            if(found1 == std::string::npos) {
                int nbVertices = std::sscanf(buffer.c_str(), "%*s %d %d %d %d", &id1, &id2, &id3, &id4);
                if (nbVertices == 4) isFaceQuad = true;
            }
                // If the face definition is of the form "f v1// v2// v3// v4//"
            else if (found2 == 0 && static_cast<int>(line.substr(found1+found2+1).find(' ') == 0)) {
                int nbVertices = std::sscanf(buffer.c_str(), "%*s %d// %d// %d// %d//", &id1, &id2, &id3, &id4);
                if (nbVertices == 4) isFaceQuad = true;
            }
            else {  // If the face definition contains vertices and (texture coordinates or normals)

                tId1 = -1;
                tId2 = -1;
                tId3 = -1;
                tId4 = -1;

                nId1 = -1;
                nId2 = -1;
                nId3 = -1;
                nId4 = -1;

                //get the part of the string until the second index
                tmp = line.substr(found1+1);
                found2 = static_cast<int>(tmp.find(' '));
                tmp = tmp.substr(0,found2);
                found2 = static_cast<int>(tmp.find('/'));

                // If the face definition is of the form "f vert1/textcoord1 vert2/textcoord2 ..."
                if(found2 == std::string::npos) {
                    int n = std::sscanf(buffer.c_str(), "%*s %d/%d %d/%d %d/%d %d/%d", &id1, &tId1, &id2, &tId2, &id3, &tId3, &id4, &tId4);
                    if (n == 8) isFaceQuad = true;

                    uvsIndices.emplace_back(tId1-1);
                    uvsIndices.emplace_back(tId2-1);
                    uvsIndices.emplace_back(tId3-1);
                    if (isFaceQuad) uvsIndices.emplace_back(tId4-1);
                }
                else {
                    tmp = line.substr(found1+1);
                    found2 = static_cast<int>(tmp.find('/'));

                    // If the face definition is of the form "f vert1//normal1 vert2//normal2 ..."
                    if(found2 == 0) {
                        int n = std::sscanf(buffer.c_str(), "%*s %d//%d %d//%d %d//%d %d//%d", &id1, &nId1, &id2, &nId2, &id3, &nId3, &id4, &nId4);
                        if (n == 8) isFaceQuad = true;
                    }
                        // If the face definition is of the form "f vert1/textcoord1/normal1 ..."
                    else {
                        int n = std::sscanf(buffer.c_str(), "%*s %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d", &id1, &tId1, &nId1, &id2, &tId2, &nId2, &id3, &tId3, &nId3, &id4, &tId4, &nId4);
                        if (n == 12) isFaceQuad = true;
                        uvsIndices.emplace_back(tId1-1);
                        uvsIndices.emplace_back(tId2-1);
                        uvsIndices.emplace_back(tId3-1);
                        if (isFaceQuad) uvsIndices.emplace_back(tId4-1);
                    }
                    normalsIndices.emplace_back(nId1-1);
                    normalsIndices.emplace_back(nId2-1);
                    normalsIndices.emplace_back(nId3-1);
                    if (isFaceQuad) normalsIndices.emplace_back(nId4-1);
                }
            }
            verticesIndices.emplace_back(id1-1);
            verticesIndices.emplace_back(id2-1);
            verticesIndices.emplace_back(id3-1);
            if (isFaceQuad) verticesIndices.emplace_back((id4-1));
            isQuad.emplace_back(isFaceQuad);
        }
    }

    assert(!verticesIndices.empty());
    assert(normalsIndices.empty() || normalsIndices.size() == verticesIndices.size());
    assert(uvsIndices.empty() || uvsIndices.size() == verticesIndices.size());
    meshFile.close();

    // ---------- Merge the data that we have collected from the file ---------- //

    // Destroy the current mesh
    meshContainer.destroy();

    // This is used to create duplicate vertices if a vertex with index "i" from a face does not
    // have same texture coordinates or normals as a previous vertex with index "i".
    std::unordered_map<std::tuple<int, int, int>, unsigned int > mapVertNormTexToVertexIndex;

    // Mesh data
    std::vector<std::vector<unsigned int>> meshIndices;
    std::vector<Vector3<float>> meshVertices;
    std::vector<Vector3<float>> meshNormals;
    std::vector<Vector2<float>> meshUVs;

    // We cannot load mesh with several parts for the moment
    unsigned int meshPart = 0;

    const bool hasNormals = !normalsIndices.empty() && !normals.empty();
    const bool hasUvs = !uvsIndices.empty() && !uvs.empty();

    // Fill in the vertex indices
    // We also triangulate each quad face
    meshIndices.emplace_back();
    for(size_t i = 0, j = 0; i < verticesIndices.size(); j++) {

        // 3 if the current vertices form a triangle and 4 if they form a quad
        const int nbVertices = isQuad[j] ? 4 : 3;

        int newVerticesIndices[4] = { -1, -1, -1, -1 };

        // For each vertex, we check if there is already a vertex with same UV and normals.
        for (int v = 0; v < nbVertices; v++) {

            auto normalIndex = static_cast<int>(hasNormals ? normalsIndices[i + v] : -1);
            auto uvIndex = static_cast<int>(hasUvs ? uvsIndices[i + v] : -1);

            // If the vertex with same UV and normal doesn't exist yet in the map
            std::tuple<int, int, int> key = std::make_tuple(verticesIndices[i+v], normalIndex, uvIndex);
            auto itFound = mapVertNormTexToVertexIndex.find(key);
            if (itFound == mapVertNormTexToVertexIndex.end()) {

                // Create a new vertex
                newVerticesIndices[v]= static_cast<int>(meshVertices.size());
                meshVertices.emplace_back(vertices[verticesIndices[i+v]]);
                if (hasNormals) {
                    meshNormals.emplace_back(normals[normalsIndices[i+v]]);
                }
                if (hasUvs) {
                    meshUVs.emplace_back(uvs[uvsIndices[i+v]]);
                }

                mapVertNormTexToVertexIndex.insert(std::make_pair(key, newVerticesIndices[v]));
            }
            else {
                // Get the vertex index to use
                newVerticesIndices[v] = static_cast<int>(itFound->second);
            }
        }

        // Get the current vertex IDs
        unsigned int i1 = newVerticesIndices[0];
        unsigned int i2 = newVerticesIndices[1];
        unsigned int i3 = newVerticesIndices[2];
        unsigned int i4 = newVerticesIndices[3];

        // If the current vertex not in a quad (it is part of a triangle)
        if (!isQuad[j]) {

            // Add the vertex indices
            meshIndices[meshPart].emplace_back(i1);
            meshIndices[meshPart].emplace_back(i2);
            meshIndices[meshPart].emplace_back(i3);

            i+=3;
        }
        else {  // If the current vertex is in a quad

            Vector3<float> v4 = meshVertices[i1];
            Vector3<float> v5 = meshVertices[i2];
            Vector3<float> v6 = meshVertices[i3];
            Vector3<float> v7 = meshVertices[i4];

            Vector3<float> v13 = v6-v4;
            Vector3<float> v12 = v5-v4;
            Vector3<float> v14 = v7-v4;

            float a1 = v13.dot(v12);
            float a2 = v13.dot(v14);
            if((a1 >= 0 && a2 <= 0) || (a1 <= 0 && a2 >= 0)) {
                meshIndices[meshPart].emplace_back(i1);
                meshIndices[meshPart].emplace_back(i2);
                meshIndices[meshPart].emplace_back(i3);
                meshIndices[meshPart].emplace_back(i1);
                meshIndices[meshPart].emplace_back(i3);
                meshIndices[meshPart].emplace_back(i4);
            }
            else {
                meshIndices[meshPart].emplace_back(i1);
                meshIndices[meshPart].emplace_back(i2);
                meshIndices[meshPart].emplace_back(i4);
                meshIndices[meshPart].emplace_back(i2);
                meshIndices[meshPart].emplace_back(i3);
                meshIndices[meshPart].emplace_back(i4);
            }

            i+=4;
        }
    }

    assert(meshNormals.empty() || meshNormals.size() == meshVertices.size());
    assert(meshUVs.empty() || meshUVs.size() == meshVertices.size());

    // Set the data to the mesh
    meshContainer.setIndices(meshIndices);
    meshContainer.setVertices(meshVertices);
    meshContainer.setNormals(meshNormals);
    meshContainer.setUVs(meshUVs);
}
