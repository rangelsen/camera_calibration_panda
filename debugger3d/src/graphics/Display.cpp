#include <iostream>
#include <GL/glew.h>
#include <GL/gl.h>

#include "Display.hpp"
#include "DisplaySettings.hpp"
#include <imgui/imgui.h>
#include <imgui/imgui_impl_opengl3.h>
#include <imgui/imgui_impl_sdl.h>

////////////////////////////////////////////////////////////////////////////////
Display::Display() {

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);

    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE, 32);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    SDL_DisplayMode display_mode;
    SDL_GetCurrentDisplayMode(0, &display_mode);

	width_ = DisplaySettings::WIDTH;
	height_ = DisplaySettings::HEIGHT;
    window_ = SDL_CreateWindow(DisplaySettings::TITLE.c_str(),
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width_, height_,
		SDL_WINDOW_OPENGL);

    context_ = SDL_GL_CreateContext(window_);

    glewExperimental = GL_TRUE;

    if(glewInit() != GLEW_OK)
        std::cout << "Error: Could not initialize glew" << std::endl;

    window_is_closed_ = false;

	/* Prevent faces behind other faces from being rendered */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Setup Dear ImGui binding
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

    ImGui_ImplSDL2_InitForOpenGL(window_, context_);
    ImGui_ImplOpenGL3_Init("#version 130");

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
	glViewport(0, 0, width_, height_);
}

////////////////////////////////////////////////////////////////////////////////
Display::~Display() {
    SDL_GL_DeleteContext(context_);
    SDL_DestroyWindow(window_);
    SDL_Quit();
}

////////////////////////////////////////////////////////////////////////////////
void Display::Update() {

    SDL_GL_SwapWindow(window_);
}
    
////////////////////////////////////////////////////////////////////////////////
bool Display::IsClosed() const {

    return window_is_closed_;
}

////////////////////////////////////////////////////////////////////////////////
void Display::Quit() {

    window_is_closed_ = true;
}

////////////////////////////////////////////////////////////////////////////////
SDL_Window* Display::WindowHandle() const {

    return window_;
}

/// @file
