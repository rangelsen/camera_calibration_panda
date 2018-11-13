#version 130

attribute vec3 position;

uniform mat4 mvp;
uniform vec3 color;

out vec3 color_vsout;

void main() {

    gl_Position = mvp * vec4(position, 1.0);

    color_vsout = color;
}

/// @file

