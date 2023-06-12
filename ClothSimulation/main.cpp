//
//  main.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-22.
//

#include <iostream>
#include "Canvas.hpp"
#include "Simulation.hpp"

const GLint WIDTH = 500, HEIGHT = 500;
const Eigen::Vector3f gravitationalConstant = Eigen::Vector3f(0, -9.81, 0);

Eigen::Vector3f getSpringForce(SpringEndpoint p, float timestep) {
    float k = 14000;
    float r = 200;
    float displacement = p.getX();
    return Eigen::Vector3f(k * (abs(displacement)/r - 1) * (-displacement/abs(displacement)), 0, 0);
}

Eigen::Vector3f getGravitationalForce(SpringEndpoint p, float timestep) {
    return p.getMass()*gravitationalConstant;
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
 
    // SPRING ENDPOINTS
    std::vector<SpringEndpoint> springEndpoints1;
    SpringEndpoint startPos1(Eigen::Vector3f(100, 500, 0), Eigen::Vector3f(99, 500, 0), 5);
    springEndpoints1.push_back(startPos1);
    
    std::vector<SpringEndpoint> springEndpoints2;
    SpringEndpoint startPos2(Eigen::Vector3f(100, 900, 0), Eigen::Vector3f(99, 900, 0), 5);
    springEndpoints2.push_back(startPos2);
    
    float timeStep = 0.005;
    
    Simulation simulation1(timeStep, springEndpoints1, canvas, OPTIMIZATION_IMPLICIT_EULER);
    Simulation simulation2(timeStep, springEndpoints2, canvas, TIMESTEP);
    
    simulation1.addExternalForce(&getSpringForce);
    simulation2.addExternalForce(&getSpringForce);
    
    while (!glfwWindowShouldClose(window)) {
        canvas.initCanvas();
        simulation1.update();
        //simulation2.update();
        glfwSwapBuffers(window);
    }
    
    glfwTerminate();
    return 0;
}
