#ifndef DISPLAY_H
#define DISPLAY_H

#include <SDL2/SDL.h>

/**
 * @brief Wrapper for managing SDL context settings regarding the screen
 * buffer.
 */
class Display {

public:
    Display();

    ~Display();

    /**
     * @brief Swap screen buffers
     */
    void Update();

    bool IsClosed() const;

    void Quit();

    SDL_Window* WindowHandle() const;

private:
    SDL_Window* window_;
    SDL_GLContext context_;
    bool window_is_closed_;
	int width_, height_;
};

#endif // DISPLAY_H

/// @file
