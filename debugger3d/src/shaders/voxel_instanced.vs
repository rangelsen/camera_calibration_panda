#version 330 core

attribute vec3 position;
attribute mat4 model;

uniform mat4 view_proj;
uniform float miny;
uniform float maxy;

out vec3 color_vsout;

////////////////////////////////////////////////////////////////////////////////
vec3 interpolate_color(float height, float start, float end) {

    float r, g, b;
    float range = end - start;
    float height_n = (height - start) / range;

    if (height_n >= 0.0 && height_n < 0.5) {

        g = height_n;
        r = 1.0 - g;
        b = 0.0;
    }
    else if (height_n >= 0.5 && height_n <= 1.0) {

        r = 0.0;
        b = 2.0 * (height_n - 0.5);
        g = 1.0 - b;
    }
    else {

        r = 1.0;
        g = 1.0;
        r = 1.0;
    }

    return vec3(r, g, b);
}

////////////////////////////////////////////////////////////////////////////////
void main() {

    mat4 mvp = view_proj * model;
    gl_Position = mvp * vec4(position, 1.0);
    color_vsout = interpolate_color(model[3][1], miny, maxy);
}

/// @file

