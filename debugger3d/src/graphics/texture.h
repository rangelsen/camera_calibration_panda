#ifndef TEXTURE_H
#define TEXTURE_H

#include <GL/glew.h>
#include <GL/gl.h>

typedef struct Texture {

    GLuint id;
    char type[1024];
    char path[1024];
} Texture;

////////////////////////////////////////////////////////////////////////////////
void texture_load_cubemap(const char* cubemap_path, GLuint* texture);

GLuint texture_load(const char* file_path);

void texture_print(Texture* texture);

#endif // TEXTURE_H

/// @file

