#ifndef VERTEX_H
#define VERTEX_H

#include <glm/glm.hpp>

typedef struct Vertex {

    glm::vec3 Position;
    glm::vec3 Color;
    glm::vec3 Normal;
    glm::vec2 TextureCoordinate;
    glm::vec3 Tangent;
    glm::vec3 BiTangent;
} Vertex;

#endif // VERTEX_H

/// @file

