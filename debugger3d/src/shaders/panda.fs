#version 130

out vec4 FragColor;

in vec2 tex_coord_vsout;
flat in vec4 material_color_vsout;
flat in vec4 color_ambient_vsout;
flat in vec4 color_diffuse_vsout;
flat in vec4 color_specular_vsout;

void main() {    

    gl_FragColor = material_color_vsout * (color_ambient_vsout +
        color_diffuse_vsout + color_specular_vsout);
}

/// @file

