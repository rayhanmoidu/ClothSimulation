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
    Eigen::Vector2f rasterizePoint(SpringEndpoint* particle);
    Eigen::Matrix4f perspectiveMatrix(float, float, float, float);
    Eigen::Matrix4f lookatMatrix(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f);
    Eigen::Vector2f scaleNDCToViewport(Eigen::Vector2f);
    
    int width;
    int height;
    string title;
    GLFWwindow *window;
    
    float camX;
    float camY;
    float camZ;
};



#endif
