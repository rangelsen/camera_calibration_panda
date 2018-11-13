#include <GL/glew.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "glarray.h"

////////////////////////////////////////////////////////////////////////////////
static void glarray_init(GLArray* glarray, Vertex* vertices, uint32 n_vertices);

static void glarray_describe(uint32 attrib, uint32 attrib_size, uint64 offset);

static void glarray_get_attrib_descriptor(char* attrib,
    uint32* attrib_size, uint32* offset);

////////////////////////////////////////////////////////////////////////////////
GLArray glarray_create(Vertex* vertices, uint32 n_vertices,
    char* layout) {

    GLArray glarray;

    glarray_init(&glarray, vertices, n_vertices);

    char* str = (char*) malloc((strlen(layout) + 1) * sizeof(char));
    strcpy(str, layout);

    char* token = strtok(str, ",");

    uint32 attrib_idx = 0;

    while (token) {

        uint32 attrib_size;
        uint32 offset;

        glarray_get_attrib_descriptor(token, &attrib_size, &offset);
        glarray_describe(attrib_idx++, attrib_size, offset);

        token = strtok(NULL, ",");
    }

    free(str);

    return glarray;
}

////////////////////////////////////////////////////////////////////////////////
static void glarray_init(GLArray* glarray, Vertex* vertices,
    uint32 n_vertices) {
 
    glarray->n_vertices = n_vertices;

    glGenVertexArrays(1, &glarray->vao);
    glGenBuffers(1, &glarray->vbo);
    glBindVertexArray(glarray->vao);
    glBindBuffer(GL_ARRAY_BUFFER, glarray->vbo);
    glBufferData(GL_ARRAY_BUFFER, n_vertices * sizeof(Vertex), &vertices[0],
        GL_STATIC_DRAW);
}

////////////////////////////////////////////////////////////////////////////////
static void glarray_describe(uint32 attrib, uint32 attrib_size, uint64 offset) {

    // TODO: offset needs to be 64 on a 64-bit compiler and 32 on a 32-bit
    // compiler
    glVertexAttribPointer(attrib, attrib_size, GL_FLOAT, GL_FALSE,
        sizeof(Vertex), (GLvoid*) offset);
    glEnableVertexAttribArray(attrib);
}

////////////////////////////////////////////////////////////////////////////////
static void glarray_get_attrib_descriptor(char* attrib,
    uint32* attrib_size, uint32* offset) {

    if (strcmp(attrib, "p") == 0) {

        *attrib_size = 3;
        *offset = offsetof(Vertex, Position);
    }
    else if (strcmp(attrib, "c") == 0) {

        *attrib_size = 3;
        *offset = offsetof(Vertex, Color);
    }
    else if (strcmp(attrib, "t") == 0) {

        *attrib_size = 2;
        *offset = offsetof(Vertex, TextureCoordinate);
    }
}

/// @file

