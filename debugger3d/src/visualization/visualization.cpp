#include <iostream>
#include <GL/glew.h>
#include <GL/gl.h>
#include <common/Utility.hpp>

#include "visualization.hpp"

////////////////////////////////////////////////////////////////////////////////
static std::vector<RenderBundle*> renderbundles_;
static Camera* camera_;

////////////////////////////////////////////////////////////////////////////////
void visualization_setup(Camera* camera) {

    camera_ = camera;
}

////////////////////////////////////////////////////////////////////////////////
void visualization_shutdown() {

    for (uint32_t i = 0; i < renderbundles_.size(); i++) {

        void (*destroy_procedure)(void*) = renderbundles_[i]->destroy_procedure;
        (*destroy_procedure)(renderbundles_[i]->entity);
    }
}

////////////////////////////////////////////////////////////////////////////////
void visualization_render() {

    RenderInfo info_main;
    info_main.view_proj = camera_->ProjMatrix() * camera_->ViewMatrix();
    info_main.view_pos = camera_->Position();

    visualization_render_entities(&renderbundles_, renderbundles_.size(), 
                                  &info_main);
}

////////////////////////////////////////////////////////////////////////////////
void visualization_submit_for_rendering(RenderBundle* renderbundle) {

    renderbundles_.push_back(renderbundle);
}

////////////////////////////////////////////////////////////////////////////////
void visualization_render_entities(std::vector<RenderBundle*>* entities, uint32 n_entities,
                                   RenderInfo* renderinfo) {

    for (uint32 i = 0; i < n_entities; i++)
        visualization_render_entity((*entities)[i], renderinfo);
}

////////////////////////////////////////////////////////////////////////////////
void visualization_render_entity(RenderBundle* renderbundle, RenderInfo* renderinfo) {

    void (*render_procedure)(void*, RenderInfo*) = renderbundle->render_procedure;
    void* entity = renderbundle->entity;
    (*render_procedure)(entity, renderinfo);
}

/// @file

