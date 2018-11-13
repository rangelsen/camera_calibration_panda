#version 130

attribute vec3 position;
attribute vec3 color;

uniform mat4 mvp;

out vec3 color_vsout;

void main() {

    gl_Position = mvp * vec4(position, 1.0);

    color_vsout = vec3(0.4);
}

