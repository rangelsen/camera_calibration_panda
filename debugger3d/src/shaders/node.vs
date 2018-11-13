#version 130

attribute vec3 position;
attribute vec3 color;

uniform mat4 mvp;

out vec3 Color;

void main() {

    gl_Position = mvp * vec4(position, 0.01);

    Color = color;
}

