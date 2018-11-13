#ifndef SHADER_ATTRIBUTES_H
#define SHADER_ATTRIBUTES_H

#include <vector>
#include <string>

#include "Shader.hpp"

namespace ShaderAttributes {

    const std::vector<std::string> TERRAIN_ATTRIBUTES {
        "position",
        "color",
        "normal"
    };
    
    const std::vector<std::string> WATER_ATTRIBUTES {
        "position",
        "color",
        "phase",
        "triangle_indicator"
    };

   const std::vector<std::string> BASIC_ATTRIBUTES {
        "position",
        "color"
    };

   const std::vector<std::string> CUBE_ATTRIBUTES {
        "position"
    };
  
    const std::vector<std::string> NODE_ATTRIBUTES {
        "position",
        "color"
    };
    
    const std::vector<std::string> VOXEL_INSTANCED_ATTRIBUTES {
        "position",
        "model"
    };

    const std::vector<std::string> PCLOUD_ATTRIBUTES {
        "position"
    };

	const std::vector<std::string> MODEL_ATTRIBUTES {
		"position",
		"normal",
		"texcoords"
	};
}

#endif // SHADER_ATTRIBUTES_H

/// @file
