#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <SDL2/SDL.h>
#include <map>
#include <vector>

/**
 * @brief Simple static manager that sits on top of the SDL used
 * to obtain the keyboard state
 */
class Keyboard {

private:
    static const std::vector<SDL_Keycode> VALID_KEYS_;

    /**
     * @brief Holds the information about whether any of the valid keys are
     * pressed or not
     */
    static std::map<SDL_Keycode, bool> key_buffer_;

    /**
     * @brief Fills in the keyboard buffer to no keys pressed
     */
    static std::map<SDL_Keycode, bool> Initialize();

    static void Set(SDL_Keycode key, bool val);

    static bool IsValid(SDL_Keycode key);

    static void Release();

public:
    static std::map<SDL_Keycode, bool> GetKeyBuffer();

    /**
     * @brief Get fresh keyboard state from SDL and put relevant information
     * into the keyboard buffer
     */
    static void Read();

    static void PrintKeyBuffer();
};

#endif // KEYBOARD_H

/// @file

