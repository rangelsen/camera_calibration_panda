#version 130

in vec2 texcoords_vsout;

uniform sampler2D texture_diffuse1;
uniform sampler2D texture_specular1;
uniform sampler2D texture_normal1;

void main() {

    // gl_FragColor = texture2D(texture_diffuse1, texcoords_vsout);

	gl_FragColor = mix(texture2D(texture_specular1, texcoords_vsout),
									texture2D(texture_diffuse1, texcoords_vsout), 0.8);
	
	/*
	gl_FragColor = mix(diffuse_specular_mix,
					   texture2D(texture_normal1, texcoords_vsout), 0.1);
	*/
}

/// @file

