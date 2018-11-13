#ifndef CUBE_HPP
#define CUBE_HPP

#include <vector>
#include <glm/glm.hpp>
#include <GL/gl.h>
#include "Shader.hpp"
#include "RenderBundle.h"

typedef struct Cube {

    GLuint vao, vbo, ebo;
    Shader* shader;
    uint32_t n_indices;
    glm::mat4 model_matrix;
    glm::vec3 color;
    glm::vec3 pos;
    glm::vec3 length;
} Cube;

////////////////////////////////////////////////////////////////////////////////
Cube cube_create(float x, float y, float z, glm::vec3 color, float x_len,
                 float y_len, float z_len);

void cube_render(void* ventity, RenderInfo* renderinfo);

/**
 * @brief Calculates how much cube 2 overlaps on cube 1
 * @return Overlap distance along each dimension
 */
void cube_overlap_dist(Cube* c1, Cube* c2, float* x, float* y, float* z);

/**
 * @brief Calculate the percentage overlap of two cubes. Order does not matter
 * @return Percentage overlap along each dimension
 */
void cube_overlap(Cube* c1, Cube* c2, float* x, float* y, float* z);

Cube cube_merge(Cube* c1, Cube* c2);

#endif // CUBE_HPP

/// @file

