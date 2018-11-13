#ifndef GLVERTEX_BUFFER_HPP
#define GLVERTEX_BUFFER_HPP

#include <GL/gl.h>

#include <common/typedefs.h>
#include <graphics/Vertex.hpp>

////////////////////////////////////////////////////////////////////////////////
typedef struct GLArray {

    GLuint vbo;
    GLuint vao;
    GLuint ebo;
    uint32 n_vertices;
} GLArray;

////////////////////////////////////////////////////////////////////////////////
GLArray glarray_create(Vertex* vertices, uint32 n_vertices,
    char* layout);

#endif // GLVERTEX_BUFFER_HPP

/// @file

