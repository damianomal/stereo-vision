#ifndef GUI_H
#define GUI_H

#define IMGUI_IMPL_OPENGL_LOADER_GL3W

//IMGUI_IMPL_OPENGL_LOADER_GL3W

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
//#else
//#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

#include "imgui.h"
#include "imgui_impl_sdl.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <SDL.h>
#include <iostream>



class GUI
{

private:

    SDL_GLContext gl_context;
    SDL_Window* window;
    ImGuiIO io;
    ImVec4 clear_color;
    bool done;
    int val;
    bool updated;

public:


    void killGUI();
    GUI();
    ~GUI();
    int initializeGUI();
    void updateGUI();
    void setVal(int);
    int getVal();
    bool isDone();
    bool isUpdated();
    void setUpdated(bool);
};

#endif // GUI_H
