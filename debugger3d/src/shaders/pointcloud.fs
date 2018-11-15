#version 150

in vec3 color_vsout;

out vec4 frag_color;

////////////////////////////////////////////////////////////////////////////////
void main() {

    frag_color = vec4(color_vsout, 1.0);
}

/// @file

