#include <iostream>

#include "InputHandler.hpp"
#include "Keyboard.hpp"

static float movement_speed = 0.03f;
static float rotation_speed = 0.02f;

////////////////////////////////////////////////////////////////////////////////
void InputHandler::HandleInputs(Display* display, Camera* camera) {

    SDL_PumpEvents();
    std::map<SDL_Keycode, bool> key_buffer = Keyboard::GetKeyBuffer();

    DetectAndExecuteEvents(key_buffer, display, camera);

    SDL_Event event;

    while (SDL_PollEvent(&event)) {

        if (event.type == SDL_QUIT) {

            display->Quit();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void InputHandler::DetectAndExecuteEvents(
    std::map<SDL_Keycode, bool> key_buffer,Display* display, Camera* camera) {
    
    for (std::map<SDL_Keycode, bool>::iterator it = key_buffer.begin();
         it != key_buffer.end(); it++) {


        if(it->second) {

            switch(it->first) {

                case SDLK_a:
                    camera->MoveRight(-movement_speed);
                    break;

                case SDLK_w:
                    camera->MoveForward(movement_speed);
                    break;

                case SDLK_s:
                    camera->MoveForward(-movement_speed);
                    break;

                case SDLK_d:
                    camera->MoveRight(movement_speed);
                    break;

                case SDLK_RIGHT:
                    camera->RotateVertical(-rotation_speed);
                    break;

                case SDLK_LEFT:
                    camera->RotateVertical(rotation_speed);
                    break;

                case SDLK_UP:
                    camera->RotateHorizontal(rotation_speed);
                    break;

                case SDLK_DOWN:
                    camera->RotateHorizontal(-rotation_speed);
                    break;

                case SDLK_SPACE:
                    camera->MoveUp(movement_speed);
                    break;

                case SDLK_LCTRL:
                    camera->MoveUp(-movement_speed);
                    break;

                case SDLK_ESCAPE:
                    display->Quit();
                    break;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
void InputHandler::SetMovementSpeed(float speed) {

	movement_speed = speed;
}

////////////////////////////////////////////////////////////////////////////////
void InputHandler::SetRotationSpeed(float speed) {

	rotation_speed = speed;
}

/// @file

