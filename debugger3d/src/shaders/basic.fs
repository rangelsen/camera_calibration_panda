#version 130

in vec3 color_vsout;

void main() {

    gl_FragColor = vec4(color_vsout, 1.0);
}

