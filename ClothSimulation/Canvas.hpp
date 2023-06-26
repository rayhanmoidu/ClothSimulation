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
#include <eigen3/Eigen/Dense>
#include <vector>
#include "Spring.hpp"

using namespace std;

enum RenderingMode {
    THREE_D = 0,
    TWO_D,
};


class Canvas {
public:
    Canvas(int screenWidth, int screenHeight, char* windowTitle, RenderingMode);
    Canvas();
    void initCanvas();
    void drawParticles(vector<SpringEndpoint*> particles);
    void drawSpring(Spring);

    // getters
    GLFWwindow* getWindow();
    int getHeight();
    int getWidth();

private:
    Eigen::Vector2f rasterizePoint(SpringEndpoint* particle);
    Eigen::Matrix4f perspectiveMatrix(float, float, float, float);
    Eigen::Matrix4f lookatMatrix(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f);
    Eigen::Vector2f scaleNDCToViewport(Eigen::Vector2f);
    
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    
    int width;
    int height;
    string title;
    GLFWwindow *window;
    
    static float camX;
    static float camY;
    static float camZ;
    static float lookAtX;
    static float lookAtY;
    static float lookAtZ;
    
    RenderingMode mode;
};



#endif
