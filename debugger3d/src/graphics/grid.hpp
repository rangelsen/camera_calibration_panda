#include <GL/glew.h>
#include <glm/glm.hpp>

#include <graphics/RenderBundle.h>
#include <graphics/Shader.hpp>

////////////////////////////////////////////////////////////////////////////////
typedef struct Grid {

    GLuint vao;
    GLuint vbo;
    uint32_t n_vertices;
    Shader* shader;
    glm::vec3 color;
} Grid;

////////////////////////////////////////////////////////////////////////////////
Grid* grid_create(float x0, float z0, float xlen, float zlen);

RenderBundle* grid_create_renderbundle(Grid* grid);

/// @file

