#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <GL/gl.h>

#include "Cube.hpp"
#include "ShaderAttributes.hpp"

////////////////////////////////////////////////////////////////////////////////
static std::vector<glm::vec3> generate_vertices(glm::vec3 pos, float x_len, float y_len, float z_len);
static std::vector<GLuint> generate_indices();
static void cube_generate_vertex_buffers(Cube* cube);

////////////////////////////////////////////////////////////////////////////////
Cube cube_create(float x, float y, float z, glm::vec3 color, float x_len, float y_len, float z_len) {

    Cube cube;
    cube.model_matrix = glm::mat4(1.0f);
    cube.length = glm::vec3(x_len, y_len, z_len);
    cube.pos = glm::vec3(x, y, z);
    cube.model_matrix = glm::translate(cube.model_matrix, cube.pos);

    cube.color = color;

    cube_generate_vertex_buffers(&cube);
    std::vector<GLuint> indices = generate_indices();
    std::vector<glm::vec3> vertices = generate_vertices(cube.pos, x_len, y_len, z_len);

    cube.n_indices = indices.size();
    glBindVertexArray(cube.vao);
    glBindBuffer(GL_ARRAY_BUFFER, cube.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3),
                 &vertices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*) 0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindVertexArray(0);

    cube.shader = new Shader("src/shaders/cube",
                              ShaderAttributes::CUBE_ATTRIBUTES);
    return cube;
}

////////////////////////////////////////////////////////////////////////////////
static std::vector<glm::vec3> generate_vertices(glm::vec3 pos, float x_len, float y_len, float z_len) {

    float min_x = 0.0f;
    float min_y = 0.0f;
    float min_z = 0.0f;

    float max_x = min_x + x_len;
    float max_y = min_y + y_len;
    float max_z = min_z + z_len;

    float x0 = min_x;
    float y0 = min_y;
    float z0 = min_z;

    float x1 = max_x;
    float y1 = max_y;
    float z1 = max_z;

    std::vector<glm::vec3> vertices {

        glm::vec3(x0, y0, z0),
        glm::vec3(x1, y0, z0),
        glm::vec3(x1, y1, z0),
        glm::vec3(x0, y1, z0),

        glm::vec3(x0, y0, z1),
        glm::vec3(x1, y0, z1),
        glm::vec3(x1, y1, z1),
        glm::vec3(x0, y1, z1)
    };

    return vertices;
}

////////////////////////////////////////////////////////////////////////////////
static std::vector<GLuint> generate_indices() {

    return std::vector<GLuint> {
        0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 4, 7, 7, 3,
        3, 0, 4, 5, 5, 6, 6, 7, 1, 5, 1, 2, 2, 6
    };
}

////////////////////////////////////////////////////////////////////////////////
static void cube_generate_vertex_buffers(Cube* cube) {

    glGenVertexArrays(1, &cube->vao);
    glGenBuffers(1, &cube->vbo);
    glGenBuffers(1, &cube->ebo);
}

////////////////////////////////////////////////////////////////////////////////
void cube_render(void* ventity, RenderInfo* renderinfo) {

    Cube* cube = (Cube*) ventity;
    cube->shader->Bind();

    glm::mat4 mvp = renderinfo->view_proj * cube->model_matrix;

    glUniformMatrix4fv(glGetUniformLocation(cube->shader->Id(), "mvp"), 1,
                       GL_FALSE, &mvp[0][0]);

    glUniform3f(glGetUniformLocation(cube->shader->Id(), "color"),
        cube->color.r, cube->color.g, cube->color.b);

    glBindVertexArray(cube->vao);
    glDrawElements(GL_LINES, cube->n_indices, GL_UNSIGNED_INT, NULL);
    glBindVertexArray(0);
    cube->shader->Unbind();
}

////////////////////////////////////////////////////////////////////////////////
void cube_overlap_dist(Cube* c1, Cube* c2, float* x, float* y, float* z) {

    glm::vec3 near1 = c1->pos;
    glm::vec3 near2 = c2->pos;

    glm::vec3 far1 = c1->pos + c1->length;
    glm::vec3 far2 = c2->pos + c2->length;

    if (glm::all(glm::lessThanEqual(far1, near2)) ||
        glm::all(glm::lessThanEqual(far2, near2))) {

        *x = 0.0f;
        *y = 0.0f;
        *z = 0.0f;
        return;
    }

    // Get smallest distance to far corner
    float min_far_x = fminf(far1.x, far2.x);
    float min_far_y = fminf(far1.y, far2.y);
    float min_far_z = fminf(far1.z, far2.z);

    // Get greatest distance to near corner
    float max_near_x = fmaxf(near1.x, near2.x);
    float max_near_y = fmaxf(near1.y, near2.y);
    float max_near_z = fmaxf(near1.z, near2.z);

    *x = min_far_x - max_near_x;
    *y = min_far_y - max_near_y;
    *z = min_far_z - max_near_z;
}

////////////////////////////////////////////////////////////////////////////////
void cube_overlap(Cube* c1, Cube* c2, float* x, float* y, float* z) {

    float x_overlap_21, y_overlap_21, z_overlap_21;
    cube_overlap_dist(c1, c2, &x_overlap_21, &y_overlap_21, &z_overlap_21);

    glm::vec3 abs_len1 = glm::abs(c1->length);
    x_overlap_21 /= (abs_len1.x) > 0.0f ? abs_len1.x : 1.0f;
    y_overlap_21 /= (abs_len1.y) > 0.0f ? abs_len1.y : 1.0f;
    z_overlap_21 /= (abs_len1.z) > 0.0f ? abs_len1.z : 1.0f;

    float x_overlap_12, y_overlap_12, z_overlap_12;
    cube_overlap_dist(c2, c1, &x_overlap_12, &y_overlap_12, &z_overlap_12);

    glm::vec3 abs_len2 = glm::abs(c2->length);
    x_overlap_12 /= (abs_len2.x) > 0.0f ? abs_len2.x : 1.0f;
    y_overlap_12 /= (abs_len2.y) > 0.0f ? abs_len2.y : 1.0f;
    z_overlap_12 /= (abs_len2.z) > 0.0f ? abs_len2.z : 1.0f;

    *x = (x_overlap_21 + x_overlap_12) / 2.0f;
    *y = (y_overlap_21 + y_overlap_12) / 2.0f;
    *z = (z_overlap_21 + z_overlap_12) / 2.0f;
}

////////////////////////////////////////////////////////////////////////////////
Cube cube_merge(Cube* c1, Cube* c2) { 

    glm::vec3 near1 = c1->pos;
    glm::vec3 near2 = c2->pos;
    glm::vec3 far1 = c1->pos + c1->length;
    glm::vec3 far2 = c2->pos + c2->length;
    glm::vec3 min = near1;
    glm::vec3 max = far1;

    for (uint8_t l = 0; l < 3; l++) {

        min[l] = fminf(min[l], near2[l]);
        max[l] = fmaxf(max[l], far2[l]);
    }

    glm::vec3 len = max - min;

    Cube merged = cube_create(min[0], min[1], min[2], glm::vec3(0.8f, 0.8f, 0.0f), len[0], len[1], len[2]);
    
    return merged;
}

/// @file

