#include <GL/glew.h>
#include <vector>
#include <glm/gtc/matrix_transform.hpp>

#include <graphics/RenderBundle.h>
#include "coord_frame.hpp"
#include <graphics/Vertex.hpp>
#include <graphics/ShaderAttributes.hpp>

////////////////////////////////////////////////////////////////////////////////
static void coordframe_gen_vertices(CoordFrame* cf, Vertex* vertices);

////////////////////////////////////////////////////////////////////////////////
CoordFrame* coordframe_create(float len) {

    CoordFrame* coordframe = new CoordFrame;

	Vertex vertices[6];
	uint32 n_vertices = 6;
	coordframe->len = len;
	coordframe->pose = glm::mat4(1.0f);
	coordframe_gen_vertices(coordframe, vertices);

	coordframe->glarray = glarray_create(vertices, n_vertices, "p,c");

    coordframe->shader = new Shader("src/shaders/pointcloud",
        ShaderAttributes::PCLOUD_ATTRIBUTES);
    return coordframe;
}

////////////////////////////////////////////////////////////////////////////////
void coordframe_destroy(void* entity) { }

////////////////////////////////////////////////////////////////////////////////
static void coordframe_gen_vertices(CoordFrame* cf, Vertex* vertices) {

	Vertex v;

	// X-axis
	v.Position = glm::vec3(0.0f);
	v.Color = glm::vec3(1.0f, 0.0f, 0.0f);
	vertices[0] = v;

	v.Position = glm::vec3(cf->len, 0.0f, 0.0f);
	vertices[1] = v;

	// Y-axis
	v.Color = glm::vec3(0.0f, 1.0f, 0.0f);
	v.Position = glm::vec3(0.0f);
	vertices[2] = v;
	
	v.Position = glm::vec3(0.0f, cf->len, 0.0f);
	vertices[3] = v;

	// Z - axis
	v.Color = glm::vec3(0.0f, 0.0f, 1.0f);
	v.Position = glm::vec3(0.0f);
	vertices[4] = v;

	v.Position = glm::vec3(0.0f, 0.0f, cf->len);
	vertices[5] = v;
}

////////////////////////////////////////////////////////////////////////////////
void coordframe_render(void* entity, RenderInfo* rinfo) {

	CoordFrame* cf = (CoordFrame*) entity;

	cf->shader->Bind();

    glm::mat4 vp = rinfo->view_proj;
    glUniformMatrix4fv(glGetUniformLocation(cf->shader->Id(), "view_proj"), 1,
		GL_FALSE, &vp[0][0]);

    // printf("%p: [%f, %f, %f]\n", entity, cf->pose[3][0], cf->pose[3][1], cf->pose[3][2]);
    glUniformMatrix4fv(glGetUniformLocation(cf->shader->Id(), "model"), 1,
		GL_FALSE, &cf->pose[0][0]);

	glBindVertexArray(cf->glarray.vao);
	glDrawArrays(GL_LINES, 0, cf->glarray.n_vertices);
	glBindVertexArray(0);
	cf->shader->Unbind();
}

////////////////////////////////////////////////////////////////////////////////
RenderBundle* coordframe_create_renderbundle(CoordFrame* cf) {

	RenderBundle* rb = new RenderBundle;

	rb->entity = (void*) cf;
	rb->render_procedure = &coordframe_render;
	rb->destroy_procedure = &coordframe_destroy;
	return rb;
}

/// @file

