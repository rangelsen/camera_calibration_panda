#ifndef DISPLAY_SETTINGS_HPP
#define DISPLAY_SETTINGS_HPP

#include <string>
#include <glm/glm.hpp>

namespace  DisplaySettings {

    extern int         WIDTH;
    extern int         HEIGHT;
    extern std::string TITLE;
    extern glm::vec3   CLEAR_COLOR;

    void LoadSettings(const std::string& filename);
}

#endif // DISPLAY_SETTINGS_HPP

/// @file
