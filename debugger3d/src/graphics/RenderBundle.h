#ifndef RENDER_BUNDLE_H
#define RENDER_BUNDLE_H

#include <glm/glm.hpp>
#include <common/typedefs.h>

////////////////////////////////////////////////////////////////////////////////
typedef struct RenderInfo {

    glm::mat4 view_proj;
    glm::vec3 light_pos;
    glm::vec3 view_pos;
} RenderInfo;

////////////////////////////////////////////////////////////////////////////////
typedef struct RenderBundle {

    void* entity;
    void (*render_procedure)(void*, RenderInfo*);
    void (*destroy_procedure)(void*);
} RenderBundle;

#endif // RENDER_BUNDLE_H

/// @file

