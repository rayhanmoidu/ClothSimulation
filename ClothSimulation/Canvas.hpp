#ifndef Canvas_hpp
#define Canvas_hpp

#include <stdio.h>
#include <iostream>
#include "SpringEndpoint.hpp"
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <eigen3/Eigen/Core>
#include <vector>
#include "Spring.hpp"

using namespace std;

class Canvas {
public:
    Canvas(int screenWidth, int screenHeight, char* windowTitle);
    Canvas();
    void initCanvas();
    void drawParticles(vector<SpringEndpoint*> particles);
    void drawSpring(Spring);

    // getters
    GLFWwindow* getWindow();
    int getHeight();
    int getWidth();

private:
    int width;
    int height;
    string title;
    GLFWwindow *window;
};



#endif
