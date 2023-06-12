#include "Canvas.hpp"
#include <iostream>
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

Canvas::Canvas() {
}

Canvas::Canvas(int screenWidth, int screenHeight, char* windowTitle) {
    title = windowTitle;
    window = glfwCreateWindow(screenWidth, screenHeight, windowTitle, nullptr, nullptr);
        
    glfwGetFramebufferSize(window, &width, &height);
}

void Canvas::initCanvas() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    glOrtho(0.0f, width, 0.0f, height, 0.0f, 1.0f);
    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity();
    
    glfwPollEvents();
}

void Canvas::drawParticles(vector<SpringEndpoint> particles) {
    glColor3f(0.5f, 0.5f, 0.5f);
    int particleSize = 10;
    for (int i = 0; i < particles.size(); i++) {
        glBegin(GL_QUADS);
        
        glVertex2f(particles[i].getX()-particleSize, particles[i].getY()-particleSize);
        glVertex2f(particles[i].getX()+particleSize, particles[i].getY()-particleSize);
        glVertex2f(particles[i].getX()+particleSize, particles[i].getY()+particleSize);
        glVertex2f(particles[i].getX()-particleSize, particles[i].getY()+particleSize);
        glEnd();
    }
}

void Canvas::drawSpring(Spring s) {
    drawParticles(s.getParticles());
}

GLFWwindow* Canvas::getWindow() {
    return window;
}

int Canvas::getHeight() {
    return height;
}

int Canvas::getWidth() {
    return width;
}
