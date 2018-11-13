#version 130

attribute vec3 position;
attribute vec3 normal;
attribute vec2 tex_coord;

out vec2 tex_coord_vsout;
flat out vec4 material_color_vsout;
flat out vec4 color_ambient_vsout;
flat out vec4 color_diffuse_vsout;
flat out vec4 color_specular_vsout;

uniform mat4 view_proj;
uniform mat4 model;
uniform vec3 view_pos;
vec3 light_source_pos = vec3(100.0f);
vec3 light_color_ambient = vec3(1.0f);
vec3 light_color_diffuse = vec3(1.0f);

const float SPECULAR_STRENGHT = 1.0;
const float SPECULAR_POWER = 23.0;

////////////////////////////////////////////////////////////////////////////////
vec3 GetSpecularColor() {

    vec3 view_dir = normalize(view_pos - position);    
    vec3 light_dir = normalize(light_source_pos - position);
    vec3 reflect_dir = reflect(-light_dir, normal);
    float spec = pow(max(dot(view_dir, reflect_dir), 0.0), SPECULAR_POWER);
    vec3 specular = spec * light_color_ambient * SPECULAR_STRENGHT;
    return specular;
}

////////////////////////////////////////////////////////////////////////////////
vec3 GetDiffuseColor() {

    vec3 light_dir = normalize(light_source_pos - vec3(model * vec4(position, 1.0)));
    float d = max(dot(light_dir, vec3(model * vec4(normal, 0.0))), 0.0);
    vec3 diffuse = d * light_color_diffuse;
    return diffuse;
}

////////////////////////////////////////////////////////////////////////////////
void main() {

	mat4 mvp = view_proj * model;
    gl_Position = mvp * vec4(position, 1.0);

	vec3 color = vec3(0.7);
    material_color_vsout = vec4(color, 1.0);
    color_ambient_vsout = vec4(light_color_ambient, 1.0);
    color_diffuse_vsout = vec4(GetDiffuseColor(), 1.0);
    // color_specular_vsout = vec4(GetSpecularColor(), 1.0);
	color_specular_vsout = vec4(0.0);

    tex_coord_vsout = tex_coord;    
}

/// @file

