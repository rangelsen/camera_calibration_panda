#include <SDL2/SDL.h>
#include <iostream>

#include "Keyboard.hpp"

std::map<SDL_Keycode, bool> Keyboard::key_buffer_ = Keyboard::Initialize();

////////////////////////////////////////////////////////////////////////////////
const std::vector<SDL_Keycode> Keyboard::VALID_KEYS_ {
    SDLK_a,
    SDLK_w,
    SDLK_s,
    SDLK_d,
    SDLK_q,
    SDLK_e,
    SDLK_r,
    SDLK_t,
    SDLK_UP,
    SDLK_DOWN,
    SDLK_LEFT,
    SDLK_RIGHT,
    SDLK_SPACE,
    SDLK_LCTRL,
    SDLK_ESCAPE
};

////////////////////////////////////////////////////////////////////////////////
std::map<SDL_Keycode, bool> Keyboard::Initialize() {

    std::map<SDL_Keycode, bool> buffer;

    for(int i = 0; i < (int) VALID_KEYS_.size(); i++) {

        SDL_Keycode key = VALID_KEYS_[i];
        buffer[key] = false;
    }

    return buffer;
}

////////////////////////////////////////////////////////////////////////////////
void Keyboard::Read() {

    const Uint8* state = SDL_GetKeyboardState(NULL);

    for(int i = 0; i < (int) VALID_KEYS_.size(); i++) {

        SDL_Scancode scancode = SDL_GetScancodeFromKey(VALID_KEYS_[i]);

        if(state[scancode])
            Set(VALID_KEYS_[i], true);
        else
            Set(VALID_KEYS_[i], false);
    }
}

////////////////////////////////////////////////////////////////////////////////
void Keyboard::Set(SDL_Keycode key, bool val) {

    if(Keyboard::IsValid(key))
        key_buffer_[key] = val;
}

////////////////////////////////////////////////////////////////////////////////
std::map<SDL_Keycode, bool> Keyboard::GetKeyBuffer() {

    Read();
    return key_buffer_;
}

////////////////////////////////////////////////////////////////////////////////
bool Keyboard::IsValid(SDL_Keycode key) {
    
    bool valid = false;

    for(int i = 0; i < (int) VALID_KEYS_.size(); i++) {
        
        if(VALID_KEYS_[i] == key) {
            valid = true;
            break;
        }
    }

    return valid;
}

////////////////////////////////////////////////////////////////////////////////
void Keyboard::Release() {
    
    const Uint8* state = SDL_GetKeyboardState(NULL);

    for(int i = 0; i < (int) VALID_KEYS_.size(); i++) {

        if(state[SDL_GetScancodeFromKey(VALID_KEYS_[i])])
            key_buffer_[VALID_KEYS_[i]] = false;
    }
}

////////////////////////////////////////////////////////////////////////////////
void Keyboard::PrintKeyBuffer() {

    for(int i = 0; i < (int) VALID_KEYS_.size(); i++) {

        if(key_buffer_[VALID_KEYS_[i]] == true)
            std::cout << 1;
        else
            std::cout << 0;

        if(i < (int) VALID_KEYS_.size() - 1)
            std::cout << " ";
    }
    std::cout << std::endl;
}

/// @file

