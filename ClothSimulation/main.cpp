//
//  main.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-22.
//

#include <iostream>
#include "Canvas.hpp"

const GLint WIDTH = 500, HEIGHT = 500;
const float gravitationalConstant = -9.81;

vector<Eigen::Array3f> forceField(Particle p, float t) {
    vector<Eigen::Array3f> resultingForces;
    Eigen::Array3f gravity(0, gravitationalConstant*p.getMass(), 0);
    
    resultingForces.push_back(gravity);
    return resultingForces;
}

Eigen::Array3f addGravitationalForceToParticle(Particle p) {
    return Eigen::Array3f(0, gravitationalConstant*p.getMass(), 0);
}

int main(int argc, const char * argv[]) {
    glfwInit();
    char* windowTitle = "Isosurface Stuffing";
    Canvas canvas(WIDTH, HEIGHT, windowTitle);
    GLFWwindow *window = canvas.getWindow();
    
    
    if (window == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        
        return -1;
    }

    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    
    if (GLEW_OK != glewInit()) {
        std::cout << "Failed to create GLEW" << std::endl;
        
        return -1;
    }
    
    Eigen::Array3f p0(600, 700, 0);
    float p0_mass = 50;
    Eigen::Array3f p0_v0(0, 0, 0);
    
    Eigen::Array3f p1(500, 800, 0);
    float p1_mass = 5;
    Eigen::Array3f p1_v0(0, 0, 0);

    Particle spring_p1(p0, p0_v0, p0_mass);
    Particle spring_p2(p1, p1_v0, p1_mass);

    float time = 0;
    float timeStep = 0.05;

    Spring s(spring_p1, spring_p2, 14000, 200, 400);
    
    
    
    while (!glfwWindowShouldClose(window)) {
        
                
        canvas.initCanvas();
        canvas.drawSpring(s);
        
        s.applyInternalSpringForces();
        s.applyAdditionalForce(&addGravitationalForceToParticle);
        s.stepForward(timeStep);
        time += timeStep;

        glfwSwapBuffers(window);
    }
    
//    Eigen::Array3f p0(500, 1000, 0);
//    float mass = 5;
//    Eigen::Array3f v0(0, 0, 0);
//
//    std::vector<Particle> particles;
//    Particle startPos(p0, v0, mass);
//
//    float time = 0;
//    float timeStep = 0.5;
//
//    particles.push_back(startPos);
    
//    while (!glfwWindowShouldClose(window)) {
//
//
//        canvas.initCanvas();
//        canvas.drawParticles(particles);
//
//        for (int i = 0; i < particles.size(); i++) {
//            particles[i].resetForces(forceField(particles[i], time));
//        }
//
//        for (int i = 0; i < particles.size(); i++) {
//            particles[i].stepForward(timeStep);
//        }
//        time += timeStep;
//
//        glfwSwapBuffers(window);
//    }
    
    glfwTerminate();
    return 0;
}
