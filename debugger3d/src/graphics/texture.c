#include <SOIL/SOIL.h>
#include <string.h>
#include <stdio.h>

#include "texture.h"
#include <common/typedefs.h>

////////////////////////////////////////////////////////////////////////////////
void texture_load_cubemap(const char* cubemap_path, GLuint* texture) {

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, *texture);

    for (int i = 0; i < 6; i++) {

        char file_number[1 + 1];
        sprintf(file_number, "%d", i);
        char texture_path[strlen(cubemap_path) + 5 + 1];
        strcpy(texture_path, cubemap_path);
        strcat(texture_path, file_number);
        strcat(texture_path, ".tga");

        int width, height, channels;
        unsigned char* image_data = SOIL_load_image(texture_path, &width,
                                                    &height, &channels,
                                                    SOIL_LOAD_RGBA);

        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,  GL_RGBA, width,
                     height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

////////////////////////////////////////////////////////////////////////////////
GLuint texture_load(const char* file_path) {

    GLuint texture_id;
    glGenTextures(1, &texture_id);

    int width, height;
    uint8* data = SOIL_load_image(file_path, &width, &height, 0, SOIL_LOAD_RGB);

    if (data) {
        
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    else
        printf("Error: Failed to load texture\n");

    SOIL_free_image_data(data);

    return texture_id;
}

////////////////////////////////////////////////////////////////////////////////
void texture_print(Texture* texture) {

    printf("Texture: %p\n", texture);
    printf("id: %u\n", texture->id);
    printf("type: %s\n", texture->type);
    printf("path: %s\n", texture->path);
}

/// @file

