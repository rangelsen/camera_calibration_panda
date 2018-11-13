#ifndef COORD_FRAME_HPP
#define COORD_FRAME_HPP

#include <glm/glm.hpp>

#include <graphics/glarray.h>
#include <graphics/RenderBundle.h>
#include <graphics/Shader.hpp>

typedef struct CoordFrame {

	float len;
	glm::mat4 pose;
	GLArray glarray;
	Shader* shader;
} CoordFrame;

////////////////////////////////////////////////////////////////////////////////
CoordFrame* coordframe_create(float len);

void coordframe_destroy(void* entity);

void coordframe_render(void* entity, RenderInfo* rinfo);

RenderBundle* coordframe_create_renderbundle(CoordFrame* cf);

#endif // COORD_FRAME_HPP

/// @file

