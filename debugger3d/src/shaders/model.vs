#version 130

attribute vec3 position;
attribute vec3 normal;
attribute vec2 texcoords;

out vec2 texcoords_vsout;

uniform mat4 vp;
uniform mat4 model;

void main()
{
    texcoords_vsout = texcoords;
    gl_Position = vp * model * vec4(position, 1.0);
}

