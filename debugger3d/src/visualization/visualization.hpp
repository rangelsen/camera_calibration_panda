#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <graphics/RenderBundle.h>
#include <graphics/Camera.hpp>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
void visualization_setup(Camera* camera);

void visualization_shutdown();

void visualization_render();

void visualization_submit_for_rendering(RenderBundle* renderbundle);

void visualization_render_entities(std::vector<RenderBundle*>* entities,
	uint32 n_entities, RenderInfo* renderinfo);

void visualization_render_entity(RenderBundle* renderbundle,
	RenderInfo* renderinfo);

#endif // VISUALIZATION_HPP

/// @file

