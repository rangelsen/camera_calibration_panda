#version 150

in vec3 position;
in vec3 color;

uniform mat4 view_proj;
uniform mat4 model;

out vec3 color_vsout;

////////////////////////////////////////////////////////////////////////////////
void main() {

    mat4 mvp = view_proj * model;
    gl_Position = mvp * vec4(20.0 * position, 1.0);
    gl_PointSize = 2.0;

    color_vsout = color;
}

/// @file

