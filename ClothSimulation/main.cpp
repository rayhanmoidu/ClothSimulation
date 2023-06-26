//
//  main.cpp
//  ClothSimulation
//
//  Created by Rayhan Moidu on 2023-05-22.
//

#include <iostream>
#include "Canvas.hpp"
#include "Simulation.hpp"
#include "Examples.hpp"

const GLint WIDTH = 500, HEIGHT = 500;
const Eigen::Vector3f gravitationalConstant = Eigen::Vector3f(0, -900.81, 0);

Eigen::Vector3f getGravitationalForce(SpringEndpoint p1, SpringEndpoint p2, float timestep) {
    return p1.getMass()*gravitationalConstant;
}

int main(int argc, const char * argv[]) {
    glfwInit();
    char* windowTitle = "Cloth Simulation";
    Canvas canvas(WIDTH, HEIGHT, windowTitle, TWO_D);
    GLFWwindow *window = canvas.getWindow();
    Examples exampleFactory;
    
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
 
    // *********************************** SIMULARTION SETUP ******************************************
    
    ClothRepresentation pyramidExample = exampleFactory.getExample(BASIC_PYRAMID);

    float timeStep = 0.05;
    
    Simulation simulation1(pyramidExample, timeStep, canvas, OPTIMIZATION_IMPLICIT_EULER);
    
    simulation1.addExternalForce(getGravitationalForce, GRAVITY);
    
    while (!glfwWindowShouldClose(window)) {
        canvas.initCanvas();
        simulation1.update();
        glfwSwapBuffers(window);
    }
    
    glfwTerminate();
    return 0;
}
